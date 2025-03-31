#include "gps.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>

// Global buffers for GPS messages.
static gps_global_buffer_t global_buf = { .count = 0, .index = 0 };
static gps_input_buffer_t input_buf = { .count = 0, .index = 0 };

// Timestamp for last streaming of GLOBAL_POSITION_INT.
static struct timespec last_gps_stream_time = {0, 0};

int gps_init(const char* port, speed_t baudrate, int filter_enabled, int verbosity) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open GPS serial port");
        return -1;
    }
    // Clear non-blocking flag.
    fcntl(fd, F_SETFL, 0);
    // Configure the serial port (similar to our configure_serial in main).
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    if (verbosity >= 1)
        printf("GPS: Serial port %s configured with baud rate %ld\n", port, (long)baudrate);
    clock_gettime(CLOCK_MONOTONIC, &last_gps_stream_time);
    return fd;
}

void gps_process(int gps_fd, int verbosity) {
    uint8_t buf[256];
    int n = read(gps_fd, buf, sizeof(buf));
    if (n <= 0)
        return;
    mavlink_message_t msg;
    mavlink_status_t status;
    for (int i = 0; i < n; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
            if (verbosity >= 2)
                printf("GPS: Received MAVLink message ID %d\n", msg.msgid);
            if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                mavlink_global_position_int_t gp;
                mavlink_msg_global_position_int_decode(&msg, &gp);
                global_buf.messages[global_buf.index] = gp;
                global_buf.index = (global_buf.index + 1) % GPS_BUFFER_SIZE;
                if (global_buf.count < GPS_BUFFER_SIZE)
                    global_buf.count++;
            } else if (msg.msgid == MAVLINK_MSG_ID_GPS_INPUT) {
                mavlink_gps_input_t gi;
                mavlink_msg_gps_input_decode(&msg, &gi);
                input_buf.messages[input_buf.index] = gi;
                input_buf.index = (input_buf.index + 1) % GPS_BUFFER_SIZE;
                if (input_buf.count < GPS_BUFFER_SIZE)
                    input_buf.count++;
            }
        }
    }
}

void gps_stream_update(int serial_fd, int verbosity) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (elapsed_ms(last_gps_stream_time, now) >= 200) {
        if (global_buf.count > 0) {
            int idx = (global_buf.index - 1 + GPS_BUFFER_SIZE) % GPS_BUFFER_SIZE;
            mavlink_message_t msg;
            uint8_t buffer[256];
            // Pack the latest GLOBAL_POSITION_INT message.
            mavlink_msg_global_position_int_pack(1, 1, &msg,
                global_buf.messages[idx].time_boot_ms,
                global_buf.messages[idx].lat,
                global_buf.messages[idx].lon,
                global_buf.messages[idx].alt,
                global_buf.messages[idx].relative_alt,
                global_buf.messages[idx].vx,
                global_buf.messages[idx].vy,
                global_buf.messages[idx].vz,
                global_buf.messages[idx].hdg);
            int len = mavlink_msg_to_send_buffer(buffer, &msg);
            write(serial_fd, buffer, len);
            if (verbosity >= 1)
                printf("GPS: Streamed GLOBAL_POSITION_INT message.\n");
        }
        last_gps_stream_time = now;
    }
}

const gps_global_buffer_t* gps_get_global_buffer(void) {
    return &global_buf;
}

const gps_input_buffer_t* gps_get_input_buffer(void) {
    return &input_buf;
}
