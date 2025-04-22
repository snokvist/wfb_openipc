#define _POSIX_C_SOURCE 199309L

#include <arpa/inet.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "bmi270.h"
#include "bmi270_config_file.h"

// ---------------------- CONFIGURABLE HEADER PARAMETERS ----------------------
#define LOG_ID               "custom_logger_name"
#define ORIENTATION          "YxZ"
#define NOTE                 "development_test"
#define FWVERSION            "FIRMWARE_0.1.0"
#define VENDOR               "potatocam"
#define VIDEO_FILENAME       "videofilename.mp4"
#define LENS_PROFILE         "potatocam/potatocam_mark1_prime_7_5mm_4k"
#define LENS_INFO            "wide"
#define FRAME_READOUT_TIME   15.23
#define FRAME_READOUT_DIR    0

// Scales (user-changeable as needed)
#define UPDATE_RATE          200.0           // Hz (max ~1000 Hz)
#define TSCALE               (1.0/UPDATE_RATE)
#define GSCALE               0.00122173047  // deg/s per LSB
#define ASCALE               0.00048828125  // g per LSB

// UDP destination defaults
#define UDP_IP               "192.168.1.2"
#define UDP_PORT             8000

#define NUM_FIELDS           7  // t,gx,gy,gz,ax,ay,az
#define CAL_DURATION_SEC     5  // seconds to sample during calibration
#define CAL_TIMEOUT_SEC     10  // max seconds to wait for user input
#define CAL_FILE_PATH       "/etc/imu.cal"

// Calibration data
static double gyro_bias[3] = {0}, R_cal[3][3] = {{1,0,0},{0,1,0},{0,0,1}};

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [--debug] [--udp] [--log] [--calibration]\n"
        "  --debug       : print raw/corrected data to stdout\n"
        "  --udp         : send binary data via UDP to %s:%d\n"
        "  --log         : write .gcsv file under /tmp\n"
        "  --calibration : run orientation calibration routine\n",
        prog, UDP_IP, UDP_PORT);
}

// Vector ops
typedef struct { double x,y,z; } vec3;
static vec3 vec_cross(const vec3 *a, const vec3 *b) {
    return (vec3){ a->y*b->z - a->z*b->y,
                   a->z*b->x - a->x*b->z,
                   a->x*b->y - a->y*b->x }; }
static double vec_dot(const vec3 *a, const vec3 *b) {
    return a->x*b->x + a->y*b->y + a->z*b->z; }
static double vec_norm(const vec3 *v) {
    return sqrt(vec_dot(v,v)); }
static void vec_normalize(vec3 *v) {
    double n = vec_norm(v);
    if (n > 0) { v->x/=n; v->y/=n; v->z/=n; } }

// Compute rotation matrix to align 'from' vector to 'to' vector
static void compute_rotation(const vec3 *from, const vec3 *to, double R[3][3]) {
    vec3 f = *from, t = *to;
    vec_normalize(&f); vec_normalize(&t);
    vec3 axis = vec_cross(&f,&t);
    double axis_norm = vec_norm(&axis);
    if (axis_norm < 1e-6) {
        // vectors parallel or anti-parallel
        if (vec_dot(&f,&t) > 0) {
            // identity
            memcpy(R, (double[3][3]){{1,0,0},{0,1,0},{0,0,1}}, sizeof(R_cal));
            return;
        }
        // 180 deg rotation around any orthogonal axis (choose X)
        R[0][0] = 1; R[0][1]=0; R[0][2]=0;
        R[1][0]=0; R[1][1]=-1; R[1][2]=0;
        R[2][0]=0; R[2][1]=0; R[2][2]=-1;
        return;
    }
    vec_normalize(&axis);
    double angle = acos(fmax(-1.0,fmin(1.0, vec_dot(&f,&t))));
    double ux=axis.x, uy=axis.y, uz=axis.z;
    double c = cos(angle), s = sin(angle), oc = 1 - c;
    R[0][0] = c + ux*ux*oc;
    R[0][1] = ux*uy*oc - uz*s;
    R[0][2] = ux*uz*oc + uy*s;
    R[1][0] = uy*ux*oc + uz*s;
    R[1][1] = c + uy*uy*oc;
    R[1][2] = uy*uz*oc - ux*s;
    R[2][0] = uz*ux*oc - uy*s;
    R[2][1] = uz*uy*oc + ux*s;
    R[2][2] = c + uz*uz*oc;
}

// Write calibration data to disk
static int save_calibration(const vec3 *flat, const vec3 *tilt) {
    FILE *f = fopen(CAL_FILE_PATH, "w");
    if (!f) return -1;
    // compute rotation to align flat->world-up
    vec3 world_up = {0,0,1};
    compute_rotation(flat, &world_up, R_cal);
    fprintf(f, "flat: %f %f %f\n", flat->x, flat->y, flat->z);
    fprintf(f, "tilt: %f %f %f\n", tilt->x, tilt->y, tilt->z);
    fprintf(f, "gyrobias: %f %f %f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    fprintf(f, "R:"
            " %f %f %f"
            " %f %f %f"
            " %f %f %f\n",
            R_cal[0][0],R_cal[0][1],R_cal[0][2],
            R_cal[1][0],R_cal[1][1],R_cal[1][2],
            R_cal[2][0],R_cal[2][1],R_cal[2][2]);
    fclose(f);
    return 0;
}

// Load calibration data; return 0 on success
static int load_calibration(void) {
    FILE *f = fopen(CAL_FILE_PATH, "r");
    if (!f) return -1;
    vec3 flat, tilt;
    if (fscanf(f, "flat: %lf %lf %lf\n", &flat.x,&flat.y,&flat.z)!=3) { fclose(f); return -1; }
    if (fscanf(f, "tilt: %lf %lf %lf\n", &tilt.x,&tilt.y,&tilt.z)!=3) { fclose(f); return -1; }
    if (fscanf(f, "gyrobias: %lf %lf %lf\n", &gyro_bias[0],&gyro_bias[1],&gyro_bias[2])!=3) { fclose(f); return -1; }
    if (fscanf(f, "R: %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
               &R_cal[0][0],&R_cal[0][1],&R_cal[0][2],
               &R_cal[1][0],&R_cal[1][1],&R_cal[1][2],
               &R_cal[2][0],&R_cal[2][1],&R_cal[2][2])!=9) { fclose(f); return -1; }
    fclose(f);
    return 0;
}

// Perform calibration routine
static int calibration_mode(struct bmi270 *sensor) {
    printf("Calibration mode: will collect data in two poses.\n");
    // helper buffers
    int N = (int)(UPDATE_RATE * CAL_DURATION_SEC);
    vec3 sum_flat={0}, sum_tilt={0};
    vec3 flat, tilt;
    double sum_gx=0,sum_gy=0,sum_gz=0;
    // Step 1: flat
    printf("Step 1: place IMU flat and still, then press Enter (timeout %d s): ", CAL_TIMEOUT_SEC); fflush(stdout);
    fd_set rfds; struct timeval tv;
    FD_ZERO(&rfds); FD_SET(STDIN_FILENO,&rfds);
    tv.tv_sec = CAL_TIMEOUT_SEC; tv.tv_usec = 0;
    if (select(STDIN_FILENO+1, &rfds, NULL, NULL, &tv) <= 0) {
        fprintf(stderr, "\nTimeout waiting for Enter. Aborting calibration.\n");
        return -1;
    }
    // drain input
    char buf[32]; fgets(buf,sizeof(buf),stdin);
    // collect flat samples
    struct timespec st = {0, (long)(TSCALE*1e9)};
    for (int i=0;i<N;i++) {
        int16_t ax,ay,az,gx,gy,gz;
        get_acc_raw(sensor,&ax,&ay,&az);
        get_gyr_raw(sensor,&gx,&gy,&gz);
        sum_flat.x += ax*ASCALE;
        sum_flat.y += ay*ASCALE;
        sum_flat.z += az*ASCALE;
        sum_gx += gx*GSCALE;
        sum_gy += gy*GSCALE;
        sum_gz += gz*GSCALE;
        nanosleep(&st,NULL);
    }
    flat.x = sum_flat.x/N; flat.y = sum_flat.y/N; flat.z = sum_flat.z/N;
    gyro_bias[0] = sum_gx/N; gyro_bias[1] = sum_gy/N; gyro_bias[2] = sum_gz/N;
    printf("\nFlat mean: [%.3f %.3f %.3f], gyro bias: [%.3f %.3f %.3f]\n",
           flat.x,flat.y,flat.z,gyro_bias[0],gyro_bias[1],gyro_bias[2]);
    // Step 2: tilt
    printf("Step 2: tilt IMU forward ~45°, then press Enter (timeout %d s): ", CAL_TIMEOUT_SEC); fflush(stdout);
    FD_ZERO(&rfds); FD_SET(STDIN_FILENO,&rfds);
    tv.tv_sec = CAL_TIMEOUT_SEC; tv.tv_usec = 0;
    if (select(STDIN_FILENO+1, &rfds, NULL, NULL, &tv) <= 0) {
        fprintf(stderr, "\nTimeout waiting for Enter. Aborting calibration.\n");
        return -1;
    }
    fgets(buf,sizeof(buf),stdin);
    for (int i=0;i<N;i++) {
        int16_t ax,ay,az;
        get_acc_raw(sensor,&ax,&ay,&az);
        sum_tilt.x += ax*ASCALE;
        sum_tilt.y += ay*ASCALE;
        sum_tilt.z += az*ASCALE;
        nanosleep(&st,NULL);
    }
    tilt.x = sum_tilt.x/N; tilt.y = sum_tilt.y/N; tilt.z = sum_tilt.z/N;
    printf("Tilt mean: [%.3f %.3f %.3f]\n", tilt.x, tilt.y, tilt.z);
    // save
    if (save_calibration(&flat,&tilt) < 0) {
        perror("Saving calibration");
        return -1;
    }
    printf("Calibration saved to %s\n", CAL_FILE_PATH);
    return 0;
}

int main(int argc, char *argv[]) {
    int do_debug=0, do_udp=0, do_log=0, do_cal=0;
    struct option opts[] = {
        {"debug", no_argument, &do_debug, 1},
        {"udp",   no_argument, &do_udp,   1},
        {"log",   no_argument, &do_log,   1},
        {"calibration", no_argument, &do_cal, 1},
        {0,0,0,0}
    };
    int c;
    while ((c = getopt_long(argc,argv,"",opts,NULL)) != -1) {}
    if (!do_debug && !do_udp && !do_log && !do_cal) {
        print_usage(argv[0]);
        return 1;
    }

    // init sensor
    struct bmi270 sensor = {.i2c_addr=I2C_PRIM_ADDR};
    if (bmi270_init(&sensor) < 0) {
        fprintf(stderr,"Failed to init BMI270\n"); return 1;
    }
    // configure
    set_mode(&sensor,PERFORMANCE_MODE); set_acc_range(&sensor,ACC_RANGE_2G);
    set_gyr_range(&sensor,GYR_RANGE_1000); set_acc_odr(&sensor,ACC_ODR_200);
    set_gyr_odr(&sensor,GYR_ODR_200); set_acc_bwp(&sensor,ACC_BWP_OSR4);
    set_gyr_bwp(&sensor,GYR_BWP_OSR4); disable_fifo_header(&sensor);
    enable_data_streaming(&sensor); enable_acc_filter_perf(&sensor);
    enable_gyr_noise_perf(&sensor); enable_gyr_filter_perf(&sensor);

    // run calibration if requested
    if (do_cal) {
        return calibration_mode(&sensor) < 0 ? 1 : 0;
    }
    // load existing calibration (optional)
    load_calibration();

    // setup UDP
    int sock=-1; struct sockaddr_in addr;
    if (do_udp) {
        sock = socket(AF_INET,SOCK_DGRAM,0);
        if (sock<0) { perror("socket"); return 1; }
        memset(&addr,0,sizeof(addr)); addr.sin_family=AF_INET;
        addr.sin_port=htons(UDP_PORT);
        if (inet_pton(AF_INET,UDP_IP,&addr.sin_addr)!=1) {
            fprintf(stderr,"Invalid UDP_IP '%s'\n",UDP_IP); return 1; }
    }
    // setup log file
    FILE *fp=NULL;
    if (do_log) {
        time_t now=time(NULL);
        char path[256]; snprintf(path,sizeof(path),"/tmp/%s_%ld.gcsv",LOG_ID,now);
        fp=fopen(path,"w"); if(!fp){ perror("fopen"); return 1; }
        // write header
        fprintf(fp,"GYROFLOW IMU LOG\nversion,1.3\nid,%s\norientation,%s\n"
                   "note,%s\nfwversion,%s\ntimestamp,%ld\nvendor,%s\n"
                   "videofilename,%s\nlensprofile,%s\nlens_info,%s\n"
                   "frame_readout_time,%.2f\nframe_readout_direction,%d\n"
                   "tscale,%.6f\ngscale,%.8f\nascale,%.8f\n"
                   "t,gx,gy,gz,ax,ay,az\n",LOG_ID,ORIENTATION,NOTE,FWVERSION,
                now,VENDOR,VIDEO_FILENAME,LENS_PROFILE,LENS_INFO,
                (double)FRAME_READOUT_TIME,FRAME_READOUT_DIR,
                (double)TSCALE,(double)GSCALE,(double)ASCALE);
    }

    // main loop
    struct timespec st = {0,(long)(TSCALE*1e9)}, tic,toc,prev,curr;
    clock_gettime(CLOCK_MONOTONIC,&prev);
    uint32_t count=0;
    while (1) {
        clock_gettime(CLOCK_MONOTONIC,&tic);
        // read raw
        int16_t ax,ay,az,gx,gy,gz;
        get_acc_raw(&sensor,&ax,&ay,&az);
        get_gyr_raw(&sensor,&gx,&gy,&gz);
        // apply calibration
        // subtract gyro bias
        double vg[3] = { gx*GSCALE - gyro_bias[0],
                         gy*GSCALE - gyro_bias[1],
                         gz*GSCALE - gyro_bias[2] };
        double va[3] = { ax*ASCALE, ay*ASCALE, az*ASCALE };
        // rotate
        double ag[3] = { R_cal[0][0]*vg[0] + R_cal[0][1]*vg[1] + R_cal[0][2]*vg[2],
                         R_cal[1][0]*vg[0] + R_cal[1][1]*vg[1] + R_cal[1][2]*vg[2],
                         R_cal[2][0]*vg[0] + R_cal[2][1]*vg[1] + R_cal[2][2]*vg[2] };
        double aa[3] = { R_cal[0][0]*va[0] + R_cal[0][1]*va[1] + R_cal[0][2]*va[2],
                         R_cal[1][0]*va[0] + R_cal[1][1]*va[1] + R_cal[1][2]*va[2],
                         R_cal[2][0]*va[0] + R_cal[2][1]*va[1] + R_cal[2][2]*va[2] };
        // timing
        clock_gettime(CLOCK_MONOTONIC,&curr);
        uint32_t dt = (uint32_t)((curr.tv_sec-prev.tv_sec)*1e3 + (curr.tv_nsec-prev.tv_nsec)/1e6);
        prev=curr;
        // debug
        if(do_debug) {
            printf("%u | dt=%u ms | ax=%.3f ay=%.3f az=%.3f gx=%.3f gy=%.3f gz=%.3f\n",
                   count,dt, aa[0],aa[1],aa[2], ag[0],ag[1],ag[2]);
        }
        // udp
        if(do_udp) {
            int32_t buf[NUM_FIELDS];
            buf[0]=count;
            for(int i=0;i<3;i++) buf[1+i] = (int32_t)(ag[i]/GSCALE);
            for(int i=0;i<3;i++) buf[4+i] = (int32_t)(aa[i]/ASCALE);
            sendto(sock,buf,sizeof(buf),0,(struct sockaddr*)&addr,sizeof(addr));
        }
        // log
        if(do_log) {
            fprintf(fp,"%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                    count, ag[0],ag[1],ag[2], aa[0],aa[1],aa[2]);
            fflush(fp);
        }
        count++;
        // maintain rate
        clock_gettime(CLOCK_MONOTONIC,&toc);
        double elapsed = (toc.tv_sec-tic.tv_sec) + (toc.tv_nsec-tic.tv_nsec)/1e9;
        double wait = TSCALE - elapsed;
        if(wait>0) nanosleep((struct timespec[]){ {0,(long)(wait*1e9)} }, NULL);
    }
    // cleanup
    if(fp) fclose(fp);
    if(sock>=0) close(sock);
    close(sensor.i2c_fd);
    return 0;
}
