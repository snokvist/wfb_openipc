#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/select.h>
#include <math.h>
#include <stdint.h>
#include <limits.h>
#include <getopt.h>
#include <time.h>

#define BUFFER_SIZE 1024

// -------------------------------
// MAVLink GLOBAL_POSITION_INT (33)
// -------------------------------
typedef struct {
    uint32_t time_boot_ms;   // Time since system boot (ms)
    int32_t lat;             // Latitude (deg * 1e7)
    int32_t lon;             // Longitude (deg * 1e7)
    int32_t alt;             // Altitude (MSL) in mm
    int32_t relative_alt;    // Altitude above home in mm
    int16_t vx;              // Ground speed in cm/s (north)
    int16_t vy;              // Ground speed in cm/s (east)
    int16_t vz;              // Ground speed in cm/s (down)
    uint16_t hdg;            // Heading in cdeg (0.01 deg); UINT16_MAX if unknown
} global_position_int_t;

// -------------------------------
// MAVLink GPS_INPUT (232)
// -------------------------------
typedef struct {
    uint64_t time_usec;      // Timestamp (from GPS, in microseconds)
    uint8_t  gps_id;         // GPS id (set to 0)
    uint16_t ignore_flags;   // Bitmap for ignoring fields (set to 0)
    uint32_t time_week_ms;   // GPS time (ms since start of week) – not computed here
    uint16_t time_week;      // GPS week number – not computed here
    uint8_t  fix_type;       // 0: no fix, 2: 2D fix, 3: 3D fix, etc.
    int32_t  lat;            // Latitude (deg * 1e7)
    int32_t  lon;            // Longitude (deg * 1e7)
    float    alt;            // Altitude (MSL) in m
    float    hdop;           // Horizontal dilution of precision
    float    vdop;           // Vertical dilution (not computed)
    float    vn;             // Velocity in north (m/s)
    float    ve;             // Velocity in east (m/s)
    float    vd;             // Velocity in down (m/s)
    float    speed_accuracy; // Speed accuracy (m/s) – not computed
    float    horiz_accuracy; // Horizontal accuracy (m) – not computed
    float    vert_accuracy;  // Vertical accuracy (m) – not computed
    uint8_t  satellites_visible; // Number of satellites visible
    uint16_t yaw;            // Yaw in cdeg (0.01 deg); 36000 means north if not available
} gps_input_t;

// -------------------------------
// Filtered state structure.
// -------------------------------
typedef struct {
    double lat;
    double lon;
    double alt;
    double speed_knots;
    double course_deg;
    uint32_t time_boot_ms; // Time of last accepted update (ms)
    int valid;
} filtered_state_t;

// -------------------------------
// Filter statistics structure.
// -------------------------------
typedef struct {
    unsigned long lat_rejections;
    unsigned long lon_rejections;
    unsigned long alt_rejections;
    unsigned long speed_rejections;
    unsigned long course_rejections;
    unsigned long forced_updates;
} filter_stats_t;

// -------------------------------
// Global variables holding latest parsed data.
// -------------------------------
static double current_lat = 0.0;
static double current_lon = 0.0;
static double current_speed_knots = 0.0;
static double current_course_deg = 0.0;
static double current_alt = 0.0; // in meters

// Additional info from GPGGA:
static int gps_fix_quality = 0;      // Fix quality: 0 = invalid, 1 = GPS fix, 2 = DGPS fix, etc.
static int satellites_visible = 0;     // Number of satellites used.
static double hdop_value = 0.0;        // HDOP value.

// GPS time from $GPRMC (combined date and time) in microseconds.
static uint64_t gps_time_usec = 0;

// Flags indicating that valid $GPRMC and $GPGGA sentences have been parsed.
static int valid_rmc = 0;
static int valid_gga = 0;

// For computing time_boot_ms.
static struct timeval start_time;

// Fix counter.
static unsigned long fix_count = 0;

// Filtering flag and state.
static int filtering_enabled = 0;
static filtered_state_t filtered_state = { .valid = 0 };
static filter_stats_t filter_stats = {0};

// Debug flag: if set (with -v), extra output is printed.
static int debug = 0;

// -------------------------------
// Serial port configuration: sets 115200 baud, 8N1, raw mode.
// -------------------------------
void configure_serial(int fd, int baud_rate) {
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    if (debug)
        printf("Serial port configured with baud rate %d\n", baud_rate);
}

// -------------------------------
// Parse $GPRMC sentence for position, speed, course, and GPS time.
// Example:
// $GPRMC,185129.40,A,5845.16178,N,01651.50601,E,0.019,,300325,,,A*79
// -------------------------------
int parse_gprmc(char *sentence) {
    char buf[BUFFER_SIZE];
    strncpy(buf, sentence, sizeof(buf));
    buf[sizeof(buf)-1] = '\0';

    char *token = strtok(buf, ",");
    if (!token || strcmp(token, "$GPRMC") != 0)
        return -1;
    
    // Field 1: UTC time (HHMMSS.ss)
    char time_str[16] = {0};
    token = strtok(NULL, ",");
    if (!token) return -1;
    strncpy(time_str, token, sizeof(time_str)-1);
    
    // Field 2: Status (A = valid, V = void)
    token = strtok(NULL, ",");
    if (!token || token[0] != 'A') {
        if (debug)
            printf("GPRMC status invalid: %s\n", token ? token : "NULL");
        return -1;
    }
    
    // Field 3: Latitude (ddmm.mmmm)
    token = strtok(NULL, ",");
    if (!token || strlen(token) == 0) return -1;
    double raw_lat = atof(token);
    
    // Field 4: N/S indicator.
    token = strtok(NULL, ",");
    if (!token) return -1;
    char lat_dir = token[0];
    
    // Field 5: Longitude (dddmm.mmmm)
    token = strtok(NULL, ",");
    if (!token || strlen(token) == 0) return -1;
    double raw_lon = atof(token);
    
    // Field 6: E/W indicator.
    token = strtok(NULL, ",");
    if (!token) return -1;
    char lon_dir = token[0];
    
    // Field 7: Speed over ground in knots.
    token = strtok(NULL, ",");
    if (!token || strlen(token) == 0) return -1;
    double speed_knots = atof(token);
    
    // Field 8: Course over ground (degrees).
    token = strtok(NULL, ",");
    double course_deg = 0.0;
    if (token && strlen(token) > 0)
        course_deg = atof(token);
    
    // Field 9: Date (DDMMYY)
    char date_str[16] = {0};
    token = strtok(NULL, ",");
    if (token && strlen(token) >= 6) {
        strncpy(date_str, token, 6);
        date_str[6] = '\0';
    }
    
    // Convert latitude.
    int lat_deg = (int)(raw_lat / 100);
    double lat_min = raw_lat - (lat_deg * 100);
    double dec_lat = lat_deg + lat_min / 60.0;
    if (lat_dir == 'S')
        dec_lat = -dec_lat;
    
    // Convert longitude.
    int lon_deg = (int)(raw_lon / 100);
    double lon_min = raw_lon - (lon_deg * 100);
    double dec_lon = lon_deg + lon_min / 60.0;
    if (lon_dir == 'W')
        dec_lon = -dec_lon;
    
    current_lat = dec_lat;
    current_lon = dec_lon;
    current_speed_knots = speed_knots;
    current_course_deg = course_deg;
    valid_rmc = 1;
    
    if (debug) {
        printf("Parsed GPRMC: time=%s, date=%s, lat=%.6f, lon=%.6f, speed=%.2f knots, course=%.2f°\n",
               time_str, date_str, current_lat, current_lon, current_speed_knots, current_course_deg);
    }
    
    // Convert date and time to Unix epoch (UTC) in microseconds.
    if (strlen(time_str) >= 6 && strlen(date_str) == 6) {
        struct tm tm_time = {0};
        char temp[3] = {0};
        strncpy(temp, time_str, 2);
        tm_time.tm_hour = atoi(temp);
        strncpy(temp, time_str+2, 2);
        tm_time.tm_min = atoi(temp);
        strncpy(temp, time_str+4, 2);
        tm_time.tm_sec = atoi(temp);
        strncpy(temp, date_str, 2);
        tm_time.tm_mday = atoi(temp);
        strncpy(temp, date_str+2, 2);
        tm_time.tm_mon = atoi(temp) - 1;
        strncpy(temp, date_str+4, 2);
        int yy = atoi(temp);
        tm_time.tm_year = (yy < 70) ? (yy + 2000 - 1900) : (yy + 1900 - 1900);
        time_t t = timegm(&tm_time);
        double frac = 0.0;
        char *dot = strchr(time_str, '.');
        if (dot)
            frac = atof(dot);
        gps_time_usec = ((uint64_t)t * 1000000ULL) + ((uint64_t)(frac * 1000000ULL));
        if (debug)
            printf("GPS time (Unix epoch): %llu us\n", (unsigned long long)gps_time_usec);
    }
    
    return 0;
}

// -------------------------------
// Parse $GPGGA sentence for altitude and additional fix information.
// Example:
// $GPGGA,185129.40,5845.16178,N,01651.50601,E,1,06,1.47,43.3,M,27.7,M,,*68
// -------------------------------
int parse_gpgga(char *sentence) {
    char buf[BUFFER_SIZE];
    strncpy(buf, sentence, sizeof(buf));
    buf[sizeof(buf)-1] = '\0';
    
    char *token = strtok(buf, ","); // "$GPGGA"
    if (!token || strcmp(token, "$GPGGA") != 0)
        return -1;
    
    // Field 2: UTC time (skip)
    token = strtok(NULL, ",");
    if (!token) return -1;
    // Field 3: Latitude (skip)
    token = strtok(NULL, ",");
    if (!token) return -1;
    // Field 4: N/S (skip)
    token = strtok(NULL, ",");
    if (!token) return -1;
    // Field 5: Longitude (skip)
    token = strtok(NULL, ",");
    if (!token) return -1;
    // Field 6: E/W (skip)
    token = strtok(NULL, ",");
    if (!token) return -1;
    
    // Field 7: Fix quality.
    token = strtok(NULL, ",");
    if (!token) return -1;
    gps_fix_quality = atoi(token);
    
    // Field 8: Number of satellites.
    token = strtok(NULL, ",");
    if (!token) return -1;
    satellites_visible = atoi(token);
    
    // Field 9: HDOP.
    token = strtok(NULL, ",");
    if (!token) return -1;
    hdop_value = atof(token);
    
    // Field 10: Altitude.
    token = strtok(NULL, ",");
    if (!token || strlen(token) == 0) return -1;
    double altitude = atof(token);
    current_alt = altitude;
    
    valid_gga = 1;
    
    if (debug)
        printf("Parsed GPGGA: fix_quality=%d, satellites=%d, HDOP=%.2f, altitude=%.2f m\n",
               gps_fix_quality, satellites_visible, hdop_value, current_alt);
    
    return 0;
}

// -------------------------------
// Compute elapsed time since program start in milliseconds.
// -------------------------------
uint32_t get_time_boot_ms() {
    struct timeval now;
    gettimeofday(&now, NULL);
    uint32_t elapsed = (now.tv_sec - start_time.tv_sec) * 1000 +
                       (now.tv_usec - start_time.tv_usec) / 1000;
    return elapsed;
}

// -------------------------------
// Apply filtering to the current measurement using an exponential moving average
// with a dynamic threshold based on recent speed estimates.
// If dt (time since last accepted update) is less than 500 ms, then the new reading
// is accepted only if it is within the dynamic thresholds. Otherwise, the reading is dropped.
// If dt is greater than or equal to 500 ms, a forced update is performed regardless.
// -------------------------------
void apply_filter() {
    uint32_t current_time = get_time_boot_ms();
    double dt = (current_time - filtered_state.time_boot_ms) / 1000.0; // seconds
    double alpha = 0.2; // smoothing factor
    
    // If filter not yet initialized, initialize with current reading.
    if (!filtered_state.valid) {
        filtered_state.lat = current_lat;
        filtered_state.lon = current_lon;
        filtered_state.alt = current_alt;
        filtered_state.speed_knots = current_speed_knots;
        filtered_state.course_deg = current_course_deg;
        filtered_state.time_boot_ms = current_time;
        filtered_state.valid = 1;
        if (debug)
            printf("Filter initialized with current reading.\n");
        return;
    }
    
    // Compute dynamic threshold based on recent speed (m/s) from filtered state.
    double recent_speed = filtered_state.speed_knots * 0.514444;
    if (recent_speed < 5.0) recent_speed = 5.0;
    if (recent_speed > 33.0) recent_speed = 33.0;
    double allowed_disp = recent_speed * dt * 1.5;
    
    // Compute thresholds:
    double threshold_lat = allowed_disp / 111320.0;
    double cos_lat = cos(filtered_state.lat * M_PI / 180.0);
    double threshold_lon = allowed_disp / (111320.0 * (cos_lat > 0 ? cos_lat : 1));
    double threshold_alt = 10.0;    // 10 meters for altitude.
    double threshold_speed = 5.0;   // 5 knots for speed.
    double threshold_course = 10.0; // 10 degrees for course.
    
    // If dt is less than 500 ms, perform rejection filtering.
    if (dt < 0.5) {
        int good = 1;
        if (fabs(current_lat - filtered_state.lat) > threshold_lat) {
            filter_stats.lat_rejections++;
            if (debug)
                printf("Reject latitude outlier: new=%.6f, filtered=%.6f, threshold=%.6f\n",
                       current_lat, filtered_state.lat, threshold_lat);
            good = 0;
        }
        if (fabs(current_lon - filtered_state.lon) > threshold_lon) {
            filter_stats.lon_rejections++;
            if (debug)
                printf("Reject longitude outlier: new=%.6f, filtered=%.6f, threshold=%.6f\n",
                       current_lon, filtered_state.lon, threshold_lon);
            good = 0;
        }
        if (fabs(current_alt - filtered_state.alt) > threshold_alt) {
            filter_stats.alt_rejections++;
            if (debug)
                printf("Reject altitude outlier: new=%.2f, filtered=%.2f, threshold=%.2f\n",
                       current_alt, filtered_state.alt, threshold_alt);
            good = 0;
        }
        if (fabs(current_speed_knots - filtered_state.speed_knots) > threshold_speed) {
            filter_stats.speed_rejections++;
            if (debug)
                printf("Reject speed outlier: new=%.2f, filtered=%.2f, threshold=%.2f\n",
                       current_speed_knots, filtered_state.speed_knots, threshold_speed);
            good = 0;
        }
        double diff_course = current_course_deg - filtered_state.course_deg;
        while(diff_course > 180.0) diff_course -= 360.0;
        while(diff_course < -180.0) diff_course += 360.0;
        if (fabs(diff_course) > threshold_course) {
            filter_stats.course_rejections++;
            if (debug)
                printf("Reject course outlier: new=%.2f, filtered=%.2f, diff=%.2f, threshold=%.2f\n",
                       current_course_deg, filtered_state.course_deg, diff_course, threshold_course);
            good = 0;
        }
        
        // Only update the filter state if all parameters are within thresholds.
        if (good) {
            filtered_state.lat = (1 - alpha) * filtered_state.lat + alpha * current_lat;
            filtered_state.lon = (1 - alpha) * filtered_state.lon + alpha * current_lon;
            filtered_state.alt = (1 - alpha) * filtered_state.alt + alpha * current_alt;
            filtered_state.speed_knots = (1 - alpha) * filtered_state.speed_knots + alpha * current_speed_knots;
            filtered_state.course_deg = (1 - alpha) * filtered_state.course_deg + alpha * current_course_deg;
            filtered_state.time_boot_ms = current_time;
            if (debug)
                printf("Accepted reading (dt < 0.5 s).\n");
        }
        // If not good, drop the reading (do not update filtered state).
        return;
    }
    // If dt is >= 500 ms, force an update even if the reading is an outlier.
    else {
        filtered_state.lat = (1 - alpha) * filtered_state.lat + alpha * current_lat;
        filtered_state.lon = (1 - alpha) * filtered_state.lon + alpha * current_lon;
        filtered_state.alt = (1 - alpha) * filtered_state.alt + alpha * current_alt;
        filtered_state.speed_knots = (1 - alpha) * filtered_state.speed_knots + alpha * current_speed_knots;
        double diff = current_course_deg - filtered_state.course_deg;
        while(diff > 180.0) diff -= 360.0;
        while(diff < -180.0) diff += 360.0;
        filtered_state.course_deg = (1 - alpha) * filtered_state.course_deg + alpha * current_course_deg;
        filtered_state.time_boot_ms = current_time;
        filter_stats.forced_updates++;
        if (debug)
            printf("Forced update (dt >= 0.5 s).\n");
        return;
    }
}

// -------------------------------
// Print filter statistics.
// -------------------------------
void print_filter_stats() {
    printf("\nFilter Statistics:\n");
    printf("  Forced Updates: %lu\n", filter_stats.forced_updates);
    printf("  Latitude Rejections: %lu\n", filter_stats.lat_rejections);
    printf("  Longitude Rejections: %lu\n", filter_stats.lon_rejections);
    printf("  Altitude Rejections: %lu\n", filter_stats.alt_rejections);
    printf("  Speed Rejections: %lu\n", filter_stats.speed_rejections);
    printf("  Course Rejections: %lu\n", filter_stats.course_rejections);
}

// -------------------------------
// Print GLOBAL_POSITION_INT message.
// -------------------------------
void print_global_position_int() {
    global_position_int_t msg;
    msg.time_boot_ms = get_time_boot_ms();
    msg.lat = (int32_t)(current_lat * 1e7);
    msg.lon = (int32_t)(current_lon * 1e7);
    msg.alt = (int32_t)(current_alt * 1000); // m -> mm
    msg.relative_alt = msg.alt;
    double speed_cm_s = current_speed_knots * 0.514444 * 100.0; // knots -> m/s -> cm/s
    double course_rad = current_course_deg * M_PI / 180.0;
    msg.vx = (int16_t)(speed_cm_s * cos(course_rad));
    msg.vy = (int16_t)(speed_cm_s * sin(course_rad));
    msg.vz = 0;
    if (current_course_deg > 0.0)
        msg.hdg = (uint16_t)(current_course_deg * 100);
    else
        msg.hdg = UINT16_MAX;
    
    printf("\nGLOBAL_POSITION_INT:\n");
    printf("  time_boot_ms: %u ms\n", msg.time_boot_ms);
    printf("  lat: %d (degE7)\n", msg.lat);
    printf("  lon: %d (degE7)\n", msg.lon);
    printf("  alt: %d mm\n", msg.alt);
    printf("  relative_alt: %d mm\n", msg.relative_alt);
    printf("  vx: %d cm/s\n", msg.vx);
    printf("  vy: %d cm/s\n", msg.vy);
    printf("  vz: %d cm/s\n", msg.vz);
    if (msg.hdg != UINT16_MAX)
        printf("  hdg: %u cdeg\n", msg.hdg);
    else
        printf("  hdg: UNKNOWN\n");
}

// -------------------------------
// Print GPS_INPUT message (populating all available fields).
// -------------------------------
void print_gps_input() {
    gps_input_t msg;
    if (gps_time_usec != 0)
        msg.time_usec = gps_time_usec;
    else {
        struct timeval now;
        gettimeofday(&now, NULL);
        msg.time_usec = ((uint64_t)now.tv_sec * 1000000ULL) + now.tv_usec;
    }
    msg.gps_id = 0;
    msg.ignore_flags = 0;
    msg.time_week_ms = 0;
    msg.time_week = 0;
    if (gps_fix_quality == 0)
        msg.fix_type = 0;
    else if (gps_fix_quality == 1)
        msg.fix_type = 3;
    else if (gps_fix_quality == 2)
        msg.fix_type = 4;
    else
        msg.fix_type = 3;
    msg.lat = (int32_t)(current_lat * 1e7);
    msg.lon = (int32_t)(current_lon * 1e7);
    msg.alt = (float)current_alt;
    msg.hdop = (float)hdop_value;
    msg.vdop = (float)UINT16_MAX; // Not computed.
    double speed_ms = current_speed_knots * 0.514444;
    msg.vn = (float)(speed_ms * cos(current_course_deg * M_PI / 180.0));
    msg.ve = (float)(speed_ms * sin(current_course_deg * M_PI / 180.0));
    msg.vd = 0;
    msg.speed_accuracy = 0.0f;
    msg.horiz_accuracy = 0.0f;
    msg.vert_accuracy = 0.0f;
    msg.satellites_visible = (uint8_t)satellites_visible;
    if (current_course_deg > 0.0)
        msg.yaw = (uint16_t)(current_course_deg * 100);
    else
        msg.yaw = 36000;
    
    printf("\nGPS_INPUT:\n");
    printf("  time_usec: %llu us\n", (unsigned long long)msg.time_usec);
    printf("  gps_id: %u\n", msg.gps_id);
    printf("  ignore_flags: %u\n", msg.ignore_flags);
    printf("  time_week_ms: %u ms\n", msg.time_week_ms);
    printf("  time_week: %u\n", msg.time_week);
    printf("  fix_type: %u\n", msg.fix_type);
    printf("  lat: %d (degE7)\n", msg.lat);
    printf("  lon: %d (degE7)\n", msg.lon);
    printf("  alt: %.2f m\n", msg.alt);
    printf("  hdop: %.2f\n", msg.hdop);
    printf("  vdop: %.2f\n", msg.vdop);
    printf("  vn: %.2f m/s\n", msg.vn);
    printf("  ve: %.2f m/s\n", msg.ve);
    printf("  vd: %.2f m/s\n", msg.vd);
    printf("  speed_accuracy: %.2f m/s\n", msg.speed_accuracy);
    printf("  horiz_accuracy: %.2f m\n", msg.horiz_accuracy);
    printf("  vert_accuracy: %.2f m\n", msg.vert_accuracy);
    printf("  satellites_visible: %u\n", msg.satellites_visible);
    printf("  yaw: %u cdeg\n", msg.yaw);
}

// -------------------------------
// Print a summary of fix information and filter statistics.
// -------------------------------
void print_fix_summary() {
    printf("\nGPS Fix Summary:\n");
    printf("  Total Fixes: %lu\n", fix_count);
    printf("  Latest Fix Quality: %d\n", gps_fix_quality);
    printf("  Satellites Visible: %d\n", satellites_visible);
    printf("  HDOP: %.2f\n", hdop_value);
    if (filtering_enabled)
        print_filter_stats();
}

// -------------------------------
// Usage message.
// -------------------------------
void usage(const char *progname) {
    printf("Usage: %s [--port <uart_port>] [-v] [--filter]\n", progname);
    printf("  --port   : Specify UART port (default: /dev/ttyS2)\n");
    printf("  -v       : Verbose mode (print extra debug output)\n");
    printf("  --filter : Enable filtering (dynamic threshold based on recent speed)\n");
}

int main(int argc, char *argv[]) {
    char uart_port[128] = "/dev/ttyS2";
    static struct option long_options[] = {
        {"port",    required_argument, 0, 'p'},
        {"verbose", no_argument,       0, 'v'},
        {"filter",  no_argument,       0, 'f'},
        {"help",    no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    while ((opt = getopt_long(argc, argv, "p:vfh", long_options, NULL)) != -1) {
        switch (opt) {
            case 'p':
                strncpy(uart_port, optarg, sizeof(uart_port)-1);
                uart_port[sizeof(uart_port)-1] = '\0';
                break;
            case 'v':
                debug = 1;
                break;
            case 'f':
                filtering_enabled = 1;
                break;
            case 'h':
            default:
                usage(argv[0]);
                return 0;
        }
    }
    
    if (debug)
        printf("Using UART port: %s\n", uart_port);
    if (filtering_enabled && debug)
        printf("Filtering enabled.\n");
    
    int fd = open(uart_port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return 1;
    }
    
    // Set file descriptor to blocking mode.
    fcntl(fd, F_SETFL, 0);
    configure_serial(fd, B115200);
    
    gettimeofday(&start_time, NULL);
    
    char in_buffer[BUFFER_SIZE];
    int buffer_pos = 0;
    memset(in_buffer, 0, sizeof(in_buffer));
    
    while (1) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100 ms timeout
        
        int ret = select(fd + 1, &readfds, NULL, NULL, &timeout);
        if (ret < 0) {
            perror("select error");
            break;
        } else if (ret == 0) {
            continue;
        }
        
        if (FD_ISSET(fd, &readfds)) {
            char temp_buf[128];
            int bytes_read = read(fd, temp_buf, sizeof(temp_buf)-1);
            if (bytes_read > 0) {
                temp_buf[bytes_read] = '\0';
                if (debug)
                    printf("Read %d bytes: %s\n", bytes_read, temp_buf);
                if (buffer_pos + bytes_read < BUFFER_SIZE) {
                    memcpy(in_buffer + buffer_pos, temp_buf, bytes_read);
                    buffer_pos += bytes_read;
                    in_buffer[buffer_pos] = '\0';
                } else {
                    if (debug)
                        printf("Buffer overflow, resetting buffer\n");
                    buffer_pos = 0;
                    memset(in_buffer, 0, sizeof(in_buffer));
                }
                
                // Process complete lines (terminated by '\n').
                char *newline;
                while ((newline = strchr(in_buffer, '\n')) != NULL) {
                    *newline = '\0';
                    if (debug)
                        printf("Processing line: %s\n", in_buffer);
                    
                    if (strncmp(in_buffer, "$GPRMC", 6) == 0) {
                        parse_gprmc(in_buffer);
                    } else if (strncmp(in_buffer, "$GPGGA", 6) == 0) {
                        parse_gpgga(in_buffer);
                    } else {
                        if (debug)
                            printf("Ignored line: %s\n", in_buffer);
                    }
                    
                    int remaining = buffer_pos - (newline - in_buffer + 1);
                    memmove(in_buffer, newline + 1, remaining);
                    buffer_pos = remaining;
                    in_buffer[buffer_pos] = '\0';
                }
                
                if (valid_rmc && valid_gga) {
                    if (filtering_enabled)
                        apply_filter();
                    
                    print_global_position_int();
                    print_gps_input();
                    fix_count++;
                    print_fix_summary();
                    
                    // Reset flags for next fix.
                    valid_rmc = 0;
                    valid_gga = 0;
                }
            } else {
                if (debug)
                    printf("No data read from UART.\n");
            }
        }
    }
    
    close(fd);
    return 0;
}
