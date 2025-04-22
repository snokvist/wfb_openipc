#define _POSIX_C_SOURCE 199309L

#include <arpa/inet.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#define UPDATE_RATE          200.0           // Hz       (Current Max: ~1000.0 Hz)
#define TSCALE               (1.0/UPDATE_RATE)
#define GSCALE               0.00122173047  // deg/s per LSB
#define ASCALE               0.00048828125  // g per LSB

// UDP destination defaults
#define UDP_IP               "192.168.1.2"
#define UDP_PORT             8000

#define NUM_FIELDS           7  // t,gx,gy,gz,ax,ay,az

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [--debug] [--udp] [--log]\n"
        "  --debug    : print raw data to stdout\n"
        "  --udp      : send binary data via UDP to %s:%d\n"
        "  --log      : write .gcsv file under /tmp\n",
        prog, UDP_IP, UDP_PORT);
}

int main(int argc, char *argv[]) {
    int do_debug = 0, do_udp = 0, do_log = 0;
    struct option longopts[] = {
        {"debug", no_argument,      &do_debug, 1},
        {"udp",   no_argument,      &do_udp,   1},
        {"log",   no_argument,      &do_log,   1},
        {0,0,0,0}
    };
    int c;
    while ((c = getopt_long(argc, argv, "", longopts, NULL)) != -1) {
        // flags set by getopt_long
    }
    if (!do_debug && !do_udp && !do_log) {
        print_usage(argv[0]);
        return 1;
    }

    // --------------- Initialize sensor ----------------
    struct bmi270 sensor = {.i2c_addr = I2C_PRIM_ADDR};
    if (bmi270_init(&sensor) < 0) {
        fprintf(stderr, "Failed to initialize BMI270 sensor.\n");
        return 1;
    }
    // configure
    set_mode(&sensor, PERFORMANCE_MODE);
    set_acc_range(&sensor, ACC_RANGE_2G);
    set_gyr_range(&sensor, GYR_RANGE_1000);
    set_acc_odr(&sensor, ACC_ODR_200);
    set_gyr_odr(&sensor, GYR_ODR_200);
    set_acc_bwp(&sensor, ACC_BWP_OSR4);
    set_gyr_bwp(&sensor, GYR_BWP_OSR4);
    disable_fifo_header(&sensor);
    enable_data_streaming(&sensor);
    enable_acc_filter_perf(&sensor);
    enable_gyr_noise_perf(&sensor);
    enable_gyr_filter_perf(&sensor);

    // --------------- Setup UDP ----------------
    int sock = -1;
    struct sockaddr_in addr;
    if (do_udp) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            perror("socket");
            return 1;
        }
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(UDP_PORT);
        // convert IPv4 address string to binary form
        if (inet_pton(AF_INET, UDP_IP, &addr.sin_addr) != 1) {
            fprintf(stderr, "Invalid UDP_IP address '%s'\n", UDP_IP);
            return 1;
        }
    }

    // --------------- Setup file logging ----------------
    FILE *fp = NULL;
    if (do_log) {
        time_t now = time(NULL);
        char path[256];
        snprintf(path, sizeof(path), "/tmp/%s_%ld.gcsv", LOG_ID, now);
        fp = fopen(path, "w");
        if (!fp) {
            perror("fopen");
            return 1;
        }
        // write header
        fprintf(fp, "GYROFLOW IMU LOG\n");
        fprintf(fp, "version,1.3\n");
        fprintf(fp, "id,%s\n", LOG_ID);
        fprintf(fp, "orientation,%s\n", ORIENTATION);
        fprintf(fp, "note,%s\n", NOTE);
        fprintf(fp, "fwversion,%s\n", FWVERSION);
        fprintf(fp, "timestamp,%ld\n", now);
        fprintf(fp, "vendor,%s\n", VENDOR);
        fprintf(fp, "videofilename,%s\n", VIDEO_FILENAME);
        fprintf(fp, "lensprofile,%s\n", LENS_PROFILE);
        fprintf(fp, "lens_info,%s\n", LENS_INFO);
        fprintf(fp, "frame_readout_time,%.2f\n", (double)FRAME_READOUT_TIME);
        fprintf(fp, "frame_readout_direction,%d\n", FRAME_READOUT_DIR);
        fprintf(fp, "tscale,%.6f\n", (double)TSCALE);
        fprintf(fp, "gscale,%.8f\n", (double)GSCALE);
        fprintf(fp, "ascale,%.8f\n", (double)ASCALE);
        fprintf(fp, "t,gx,gy,gz,ax,ay,az\n");
    }

    // --------------- Main loop ----------------
    struct timespec sleep_time = {0, (long)(TSCALE * 1e9)};
    struct timespec tic, toc;
    struct timespec prev, curr;
    clock_gettime(CLOCK_MONOTONIC, &prev);
    uint32_t sample_count = 0;

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &tic);

        // read sensors
        int16_t ax, ay, az, gx, gy, gz;
        get_acc_raw(&sensor, &ax, &ay, &az);
        get_gyr_raw(&sensor, &gx, &gy, &gz);

        // compute delta t in ms
        clock_gettime(CLOCK_MONOTONIC, &curr);
        uint32_t dt = (uint32_t)((curr.tv_sec - prev.tv_sec)*1e3 + (curr.tv_nsec - prev.tv_nsec)/1e6);
        prev = curr;

        // debug print
        if (do_debug) {
            printf("%u | dt=%u ms | ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\n",
                   sample_count, dt, ax, ay, az, gx, gy, gz);
        }
        // UDP send raw ints
        if (do_udp) {
            int32_t buf[NUM_FIELDS] = { (int32_t)sample_count, gx, gy, gz, ax, ay, az };
            sendto(sock, buf, sizeof(buf), 0, (struct sockaddr*)&addr, sizeof(addr));
        }
        // file log
        if (do_log) {
            fprintf(fp, "%u,%d,%d,%d,%d,%d,%d\n",
                    sample_count, gx, gy, gz, ax, ay, az);
            fflush(fp);
        }

        sample_count++;

        // maintain rate
        clock_gettime(CLOCK_MONOTONIC, &toc);
        double elapsed = (toc.tv_sec - tic.tv_sec) + (toc.tv_nsec - tic.tv_nsec)/1e9;
        double wait = TSCALE - elapsed;
        if (wait > 0) {
            struct timespec rem = {0, (long)(wait*1e9)};
            nanosleep(&rem, NULL);
        }
    }

    // cleanup (unreachable)
    if (fp) fclose(fp);
    if (sock>=0) close(sock);
    close(sensor.i2c_fd);
    return 0;
}
