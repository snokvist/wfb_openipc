#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <signal.h>
#include <stdbool.h>
#include <getopt.h>
#include <sys/stat.h>
#include "mavlink_handler.h"
#include "rc_monitor.h"
#include "alink.h"
#include "utils.h"
#include "mavlink/common/mavlink.h"

#define SERIAL_PORT "/dev/ttyS2"
#define BAUDRATE B460800

#define HEARTBEAT_INTERVAL_MS 1000
#define ECHO_INTERVAL_MS 500
#define MSG_CHECK_INTERVAL_MS 100
#define SLEEP_US 2000
#define STATUSTEXT_MAX_LEN 50

volatile bool running = true;

void signal_handler(int signum) {
    running = false;
}

int set_interface_attribs(int fd, int speed) {
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return -1;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }
    return 0;
}

int main(int argc, char *argv[]) {
    int verbosity = 0;
    uint8_t system_id = 1;
    uint8_t component_id = 1;
    uint8_t vehicle_type = MAV_TYPE_FIXED_WING;
    uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
    uint32_t custom_mode = 0;
    uint8_t base_mode = 0;
    char *channels_arg = NULL;
    bool radio_camera_flag = false;

    int opt;
    while ((opt = getopt(argc, argv, "v:s:c:t:a:m:b:C:r")) != -1) {
        switch(opt) {
            case 'v': {
                verbosity++;
                char *arg = argv[optind - 1];
                if (arg[0]=='-' && arg[1]=='v') {
                    for (int j = 2; j < strlen(arg); j++) {
                        if (arg[j]=='v')
                            verbosity++;
                    }
                }
                break;
            }
            case 's': system_id = (uint8_t)atoi(optarg); break;
            case 'c': component_id = (uint8_t)atoi(optarg); break;
            case 't': vehicle_type = (uint8_t)atoi(optarg); break;
            case 'a': autopilot_type = (uint8_t)atoi(optarg); break;
            case 'm': custom_mode = (uint32_t)atoi(optarg); break;
            case 'b': base_mode = (uint8_t)atoi(optarg); break;
            case 'C': channels_arg = strdup(optarg); break;
            case 'r': radio_camera_flag = true; break;
            default:
                fprintf(stderr, "Usage: %s [-v] [-s system_id] [-c component_id] [-t vehicle_type] [-a autopilot_type] [-m custom_mode] [-b base_mode] [-C channels] [-r]\n", argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    signal(SIGINT, signal_handler);

    // Initialize RC monitoring module if channels specified.
    if (channels_arg != NULL) {
        rc_monitor_init(channels_arg);
        free(channels_arg);
    }

    // Initialize alink (UDP output for radio status) if requested.
    if (radio_camera_flag) {
        if (alink_init() < 0) {
            if (verbosity >= 1)
                printf("Failed to initialize alink UDP output.\n");
        }
    }

    int serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("open");
        return 1;
    }
    if (set_interface_attribs(serial_fd, BAUDRATE) != 0) {
        close(serial_fd);
        return 1;
    }

    uint8_t buffer[1024];
    mavlink_message_t msg;
    mavlink_status_t status;
    struct timespec last_heartbeat, last_echo, last_msg_check, current;
    clock_gettime(CLOCK_MONOTONIC, &last_heartbeat);
    last_echo = last_heartbeat;
    last_msg_check = last_heartbeat;

    if (verbosity >= 1) {
        printf("Starting advanced MAVLink autopilot with parameters:\n");
        printf("  System ID: %d\n", system_id);
        printf("  Component ID: %d\n", component_id);
        printf("  Vehicle Type: %d\n", vehicle_type);
        printf("  Autopilot Type: %d\n", autopilot_type);
        printf("  Custom Mode: %u\n", custom_mode);
        printf("  Base Mode: %d\n", base_mode);
        printf("  Verbosity: %d\n", verbosity);
        printf("  Serial Port: %s @ baudrate 460800\n", SERIAL_PORT);
        if (radio_camera_flag)
            printf("  RADIO_STATUS UDP output enabled (to 127.0.0.1:9999)\n");
    }

    while (running) {
        int n = read(serial_fd, buffer, sizeof(buffer));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                    if (verbosity >= 2) {
                        printf("Received message: ID %d from system %d, component %d\n",
                               msg.msgid, msg.sysid, msg.compid);
                    }
                    process_mavlink_message(&msg, serial_fd, verbosity);
                }
            }
        }

        clock_gettime(CLOCK_MONOTONIC, &current);
        if (elapsed_ms(last_heartbeat, current) >= HEARTBEAT_INTERVAL_MS) {
            mavlink_message_t heartbeat;
            uint8_t hb_buffer[256];
            mavlink_msg_heartbeat_pack(system_id, component_id, &heartbeat,
                                       vehicle_type, autopilot_type, base_mode, custom_mode, MAV_STATE_ACTIVE);
            int len = mavlink_msg_to_send_buffer(hb_buffer, &heartbeat);
            write(serial_fd, hb_buffer, len);
            if (verbosity >= 1)
                printf("Sent HEARTBEAT.\n");
            last_heartbeat = current;
        }

        if (elapsed_ms(last_echo, current) >= ECHO_INTERVAL_MS) {
            mavlink_message_t echo_msg;
            uint8_t echo_buffer[256];
            int len;
            const mavlink_rc_channels_override_t *rc = get_last_rc_channels();
            if (rc) {
                mavlink_msg_rc_channels_override_pack(system_id, component_id, &echo_msg,
                    rc->target_system, rc->target_component,
                    rc->chan1_raw, rc->chan2_raw, rc->chan3_raw, rc->chan4_raw,
                    rc->chan5_raw, rc->chan6_raw, rc->chan7_raw, rc->chan8_raw,
                    rc->chan9_raw, rc->chan10_raw, rc->chan11_raw, rc->chan12_raw,
                    rc->chan13_raw, rc->chan14_raw, rc->chan15_raw, rc->chan16_raw,
                    rc->chan17_raw, rc->chan18_raw);
                len = mavlink_msg_to_send_buffer(echo_buffer, &echo_msg);
                write(serial_fd, echo_buffer, len);
            }
            const mavlink_radio_status_t *radio = get_last_radio_status();
            if (radio) {
                mavlink_msg_radio_status_pack(system_id, component_id, &echo_msg,
                                              radio->rssi, radio->remrssi, radio->txbuf,
                                              radio->noise, radio->remnoise, radio->rxerrors,
                                              radio->fixed);
                len = mavlink_msg_to_send_buffer(echo_buffer, &echo_msg);
                write(serial_fd, echo_buffer, len);
            }
            const mavlink_raw_imu_t *imu = get_last_raw_imu();
            if (imu) {
                mavlink_msg_raw_imu_pack(system_id, component_id, &echo_msg,
                                         imu->time_usec, imu->xacc, imu->yacc, imu->zacc,
                                         imu->xgyro, imu->ygyro, imu->zgyro,
                                         imu->xmag, imu->ymag, imu->zmag,
                                         imu->id, imu->temperature);
                len = mavlink_msg_to_send_buffer(echo_buffer, &echo_msg);
                write(serial_fd, echo_buffer, len);
            }
            last_echo = current;
        }

        if (elapsed_ms(last_msg_check, current) >= MSG_CHECK_INTERVAL_MS) {
            if (access("/tmp/message.mavlink", F_OK) == 0) {
                int fd_msg = open("/tmp/message.mavlink", O_RDONLY);
                if (fd_msg >= 0) {
                    char msg_text[STATUSTEXT_MAX_LEN + 1];
                    int nread = read(fd_msg, msg_text, STATUSTEXT_MAX_LEN);
                    if (nread > 0) {
                        msg_text[nread] = '\0';
                        mavlink_message_t statustext_msg;
                        uint8_t statustext_buffer[256];
                        mavlink_msg_statustext_pack(system_id, component_id, &statustext_msg,
                                                    MAV_SEVERITY_INFO, msg_text, 0, 0);
                        int len = mavlink_msg_to_send_buffer(statustext_buffer, &statustext_msg);
                        write(serial_fd, statustext_buffer, len);
                        if (verbosity >= 1)
                            printf("Sent STATUSTEXT: %s\n", msg_text);
                    }
                    close(fd_msg);
                    unlink("/tmp/message.mavlink");
                }
            }
            last_msg_check = current;
        }

        const mavlink_rc_channels_override_t *rc_data = get_last_rc_channels();
        if (rc_data)
            rc_monitor_update(rc_data, verbosity);

        usleep(SLEEP_US);
    }

    close(serial_fd);
    return 0;
}
