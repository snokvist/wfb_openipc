/*
 * advanced_mavlink_autopilot.c
 *
 * A minimal autopilot that:
 *  - Sends periodic HEARTBEAT messages.
 *  - Responds to COMMAND_LONG with COMMAND_ACK.
 *  - Implements a parameter table for FPV-related settings:
 *      * VTX_POWER, VIDEO_CH, VIDEO_BR, and VTX_PROFILE.
 *      * Retrieves parameters using /usr/bin/autopilot_cmd (get) and accepts
 *        updates via /usr/bin/autopilot_cmd (set).
 *  - Saves the latest received RC_CHANNELS_OVERRIDE, RADIO_STATUS, and RAW_IMU
 *    messages and echoes them back every 500ms.
 *  - Checks every 100ms for a file (/tmp/message.mavlink) containing a text
 *    message. If found, sends it as a STATUSTEXT message (appearing in QGC) and deletes the file.
 *  - Parses incoming STATUSTEXT messages and calls
 *        /usr/bin/autopilot_cmd statustext "<text>"
 *  - Monitors specified RC channels (via the -C option, e.g. "-C 5,6") – for channels 5–12,
 *    if a change of more than 20 persists for 100ms, a system command is executed
 *    (with a 1000ms minimum delay between triggers) to update the setting.
 *  - Listens for RADIO_STATUS messages from component 100 (Camera). When received,
 *    a special text string is created using the mapping below and sent via UDP to
 *    127.0.0.1:9999 (if -r is specified) and logged to the console (if -v is used).
 *
 * Mapping for special RADIO_STATUS string:
 *   {timestamp}:{link_health_score_rssi}:{link_health_score_snr}:{recovered_packets}:{recovered_packet_count}:{best_antennas_rssi}:{best_antennas_snr}:{best_antennas_snr}:{best_antennas_snr}
 *
 *   where:
 *     - link_health_score_rssi = 999 if rssi == 0, else round((rssi * 1001 / 254) + 999)
 *     - link_health_score_snr = 999 if noise == 0, else round((noise * 1001 / 254) + 999)
 *     - best_antennas_rssi = round((remrssi * 256 / 254) - 128)
 *     - best_antennas_snr = round(remnoise * 50 / 254)
 *     - recovered_packets = fixed, capped at 254
 *     - recovered_packet_count = rxerrors, capped at 254
 *
 * Command-line options:
 *   -v             : Increase verbosity (use -v for moderate, -vv for extra verbose)
 *   -s <system_id> : Set the MAVLink system ID (default: 1)
 *   -c <comp_id>   : Set the MAVLink component ID (default: MAV_COMP_ID_AUTOPILOT1)
 *   -t <veh_type>  : Set the vehicle type (default: MAV_TYPE_FIXED_WING)
 *   -a <ap_type>   : Set the autopilot type (default: MAV_AUTOPILOT_GENERIC)
 *   -m <custom_mode>: Set the custom mode (default: 0)
 *   -b <base_mode> : Set the base mode (default: 0)
 *   -C <channels>  : Comma-separated list of RC channels to monitor (only channels 5–12 allowed)
 *   -r             : Enable special handling for RADIO_STATUS messages from component 100 (Camera)
 *
 * Usage examples:
 *   ./advanced_mavlink_autopilot -v -s 1 -c 240 -t 2 -a 3 -m 0 -b 81
 *   ./advanced_mavlink_autopilot -vv -C 5,6 -r
 *
 * NOTE: Adjust include paths and message details for your MAVLink dialect.
 */

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
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <math.h>
#include "mavlink/common/mavlink.h"  // Ensure the correct MAVLink dialect header is included

#define SERIAL_PORT "/dev/ttyS2"
#define BAUDRATE B460800

#define HEARTBEAT_INTERVAL_MS 1000
#define ECHO_INTERVAL_MS 500
#define MSG_CHECK_INTERVAL_MS 100
#define SLEEP_US 2000
#define STATUSTEXT_MAX_LEN 50

volatile bool running = true;

/* Parameter table definition */
typedef struct {
    char name[16];
    float value;
    uint8_t type;  // using MAV_PARAM_TYPE_REAL32
} parameter_t;

#define PARAM_COUNT 4
static parameter_t parameters[PARAM_COUNT] = {
    {"VTX_POWER", 20.0f, MAV_PARAM_TYPE_REAL32},
    {"VIDEO_CH",  1.0f, MAV_PARAM_TYPE_REAL32},
    {"VIDEO_BR", 5000.0f, MAV_PARAM_TYPE_REAL32},
    {"VTX_PROFILE", 0.0f, MAV_PARAM_TYPE_REAL32}
};

/* Global storage for echo messages */
static mavlink_rc_channels_override_t last_rc_channels = {0};
static bool last_rc_channels_valid = false;
static mavlink_radio_status_t last_radio_status = {0};
static bool last_radio_status_valid = false;
static mavlink_raw_imu_t last_raw_imu = {0};
static bool last_raw_imu_valid = false;

/* Monitored channel structure for -C option */
typedef struct {
    int channel;   // channel number (only 5 to 12 allowed)
    int baseline;  // last known value
    bool baseline_initialized;
    struct timespec change_start; // when change began
    struct timespec last_trigger; // last time command triggered
    bool active_change;           // flag for ongoing change
} monitored_channel_t;

#define MAX_MONITORED_CHANNELS 8
static monitored_channel_t monitored_channels[MAX_MONITORED_CHANNELS];
static int monitored_channel_count = 0;
#define CHANNEL_THRESHOLD 20
#define CHANNEL_PERSIST_MS 100
#define CHANNEL_TRIGGER_DELAY_MS 1000

/* UDP globals for RADIO_STATUS from Camera */
bool enable_radio_camera_udp = false;
int udp_sock = -1;
struct sockaddr_in udp_addr;

/* Helper function to compute elapsed ms between two timespecs */
long elapsed_ms(struct timespec start, struct timespec end) {
    return (end.tv_sec - start.tv_sec) * 1000 + (end.tv_nsec - start.tv_nsec) / 1000000;
}

/* Helper function to get RC channel value from mavlink_rc_channels_override_t */
int get_rc_channel_value(const mavlink_rc_channels_override_t *rc, int channel) {
    switch(channel) {
        case 1: return rc->chan1_raw;
        case 2: return rc->chan2_raw;
        case 3: return rc->chan3_raw;
        case 4: return rc->chan4_raw;
        case 5: return rc->chan5_raw;
        case 6: return rc->chan6_raw;
        case 7: return rc->chan7_raw;
        case 8: return rc->chan8_raw;
        case 9: return rc->chan9_raw;
        case 10: return rc->chan10_raw;
        case 11: return rc->chan11_raw;
        case 12: return rc->chan12_raw;
        case 13: return rc->chan13_raw;
        case 14: return rc->chan14_raw;
        case 15: return rc->chan15_raw;
        case 16: return rc->chan16_raw;
        case 17: return rc->chan17_raw;
        case 18: return rc->chan18_raw;
        default: return -1;
    }
}

/* Helper functions to run external commands for parameter get/set */
float get_parameter_from_system(const char *param, float current_value, int verbosity) {
    char command[128];
    char buffer[128];
    FILE *fp;
    snprintf(command, sizeof(command), "/usr/bin/autopilot_cmd get %s", param);
    fp = popen(command, "r");
    if (!fp) {
        if (verbosity >= 1) {
            fprintf(stderr, "Error: Failed to run command: %s\n", command);
        }
        return current_value;
    }
    if (fgets(buffer, sizeof(buffer), fp) != NULL) {
        if (strncmp(buffer, "OK", 2) == 0) {
            float val = atof(buffer + 3); // assume output "OK <value>"
            pclose(fp);
            return val;
        } else {
            if (verbosity >= 1) {
                fprintf(stderr, "Error: Command '%s' returned: %s\n", command, buffer);
            }
        }
    }
    pclose(fp);
    return current_value;
}

bool set_parameter_in_system(const char *param, float new_value, int verbosity) {
    char command[128];
    char buffer[128];
    FILE *fp;
    snprintf(command, sizeof(command), "/usr/bin/autopilot_cmd set %s %f", param, new_value);
    fp = popen(command, "r");
    if (!fp) {
        if (verbosity >= 1) {
            fprintf(stderr, "Error: Failed to run command: %s\n", command);
        }
        return false;
    }
    if (fgets(buffer, sizeof(buffer), fp) != NULL) {
        if (strncmp(buffer, "OK", 2) == 0) {
            pclose(fp);
            return true;
        } else {
            if (verbosity >= 1) {
                fprintf(stderr, "Error: Command '%s' returned: %s\n", command, buffer);
            }
        }
    }
    pclose(fp);
    return false;
}

/* Signal handler for graceful exit */
void signal_handler(int signum) {
    running = false;
}

/* Configure the serial port */
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

void usage(const char *progname) {
    printf("Usage: %s [-v] [-s system_id] [-c component_id] [-t vehicle_type] [-a autopilot_type] [-m custom_mode] [-b base_mode] [-C channels] [-r]\n", progname);
}

int main(int argc, char *argv[]) {
    int verbosity = 0; // 0: quiet, 1: moderate, 2: extra verbose
    uint8_t system_id = 1;
    uint8_t component_id = MAV_COMP_ID_AUTOPILOT1;
    uint8_t vehicle_type = MAV_TYPE_FIXED_WING;
    uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
    uint32_t custom_mode = 0;
    uint8_t base_mode = 0;
    char *channels_arg = NULL;
    bool radio_camera_flag = false;

    int opt;
    while ((opt = getopt(argc, argv, "v:s:c:t:a:m:b:C:r")) != -1) {
        switch (opt) {
            case 'v': {
                verbosity++;
                char *arg = argv[optind - 1];
                if (arg[0] == '-' && arg[1] == 'v') {
                    for (int j = 2; j < strlen(arg); j++) {
                        if (arg[j] == 'v')
                            verbosity++;
                    }
                }
                break;
            }
            case 's':
                system_id = (uint8_t)atoi(optarg);
                break;
            case 'c':
                component_id = (uint8_t)atoi(optarg);
                break;
            case 't':
                vehicle_type = (uint8_t)atoi(optarg);
                break;
            case 'a':
                autopilot_type = (uint8_t)atoi(optarg);
                break;
            case 'm':
                custom_mode = (uint32_t)atoi(optarg);
                break;
            case 'b':
                base_mode = (uint8_t)atoi(optarg);
                break;
            case 'C':
                channels_arg = strdup(optarg);
                break;
            case 'r':
                radio_camera_flag = true;
                break;
            default:
                usage(argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    /* Parse channels argument if provided */
    if (channels_arg != NULL) {
        char *token = strtok(channels_arg, ",");
        while (token != NULL && monitored_channel_count < MAX_MONITORED_CHANNELS) {
            int ch = atoi(token);
            if (ch >= 5 && ch <= 12) {
                monitored_channels[monitored_channel_count].channel = ch;
                monitored_channels[monitored_channel_count].baseline = -1;
                monitored_channels[monitored_channel_count].baseline_initialized = false;
                monitored_channels[monitored_channel_count].active_change = false;
                monitored_channels[monitored_channel_count].change_start.tv_sec = 0;
                monitored_channels[monitored_channel_count].change_start.tv_nsec = 0;
                monitored_channels[monitored_channel_count].last_trigger.tv_sec = 0;
                monitored_channels[monitored_channel_count].last_trigger.tv_nsec = 0;
                monitored_channel_count++;
            }
            token = strtok(NULL, ",");
        }
        free(channels_arg);
    }

    /* If radio-camera flag is set, initialize UDP socket */
    if (radio_camera_flag) {
        udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_sock < 0) {
            perror("UDP socket");
            radio_camera_flag = false;
        } else {
            memset(&udp_addr, 0, sizeof(udp_addr));
            udp_addr.sin_family = AF_INET;
            udp_addr.sin_port = htons(9999);
            if (inet_pton(AF_INET, "127.0.0.1", &udp_addr.sin_addr) <= 0) {
                perror("inet_pton");
                radio_camera_flag = false;
            } else {
                enable_radio_camera_udp = true;
            }
        }
    }

    signal(SIGINT, signal_handler);
    int serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        perror("open");
        return 1;
    }
    if (set_interface_attribs(serial_fd, BAUDRATE) != 0) {
        close(serial_fd);
        return 1;
    }

    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buffer[1024];
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
        if (monitored_channel_count > 0) {
            printf("  Monitored channels: ");
            for (int i = 0; i < monitored_channel_count; i++) {
                printf("%d ", monitored_channels[i].channel);
            }
            printf("\n");
        }
        if (enable_radio_camera_udp) {
            printf("  RADIO_STATUS UDP output enabled (to 127.0.0.1:9999)\n");
        }
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
                    switch(msg.msgid) {
                        case MAVLINK_MSG_ID_COMMAND_LONG: {
                            mavlink_command_long_t cmd;
                            mavlink_msg_command_long_decode(&msg, &cmd);
                            if (verbosity >= 2) {
                                printf("Received COMMAND_LONG: command %d, confirmation %d\n",
                                       cmd.command, cmd.confirmation);
                            }
                            mavlink_message_t ack_msg;
                            uint8_t ack_buffer[256];
                            mavlink_msg_command_ack_pack(system_id, component_id, &ack_msg,
                                                         cmd.command, MAV_RESULT_ACCEPTED,
                                                         0, 0,
                                                         cmd.target_system, cmd.target_component);
                            int len = mavlink_msg_to_send_buffer(ack_buffer, &ack_msg);
                            write(serial_fd, ack_buffer, len);
                            if (verbosity >= 1) {
                                printf("Sent COMMAND_ACK for command %d\n", cmd.command);
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
                            if (verbosity >= 1) {
                                printf("Received PARAM_REQUEST_LIST from system %d, component %d\n", msg.sysid, msg.compid);
                            }
                            for (uint8_t i = 0; i < PARAM_COUNT; i++) {
                                /* Update parameter value from system */
                                parameters[i].value = get_parameter_from_system(parameters[i].name, parameters[i].value, verbosity);
                                mavlink_message_t param_msg;
                                uint8_t param_buffer[256];
                                mavlink_msg_param_value_pack(system_id, component_id, &param_msg,
                                                             parameters[i].name,
                                                             parameters[i].value,
                                                             parameters[i].type,
                                                             PARAM_COUNT, i);
                                int len = mavlink_msg_to_send_buffer(param_buffer, &param_msg);
                                write(serial_fd, param_buffer, len);
                                if (verbosity >= 1) {
                                    printf("Sent PARAM_VALUE for %s: %f\n", parameters[i].name, parameters[i].value);
                                }
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
                            mavlink_param_request_read_t req;
                            mavlink_msg_param_request_read_decode(&msg, &req);
                            if (verbosity >= 1) {
                                printf("Received PARAM_REQUEST_READ for %s\n", req.param_id);
                            }
                            for (uint8_t i = 0; i < PARAM_COUNT; i++) {
                                if (strncmp(req.param_id, parameters[i].name, sizeof(parameters[i].name)) == 0) {
                                    parameters[i].value = get_parameter_from_system(parameters[i].name, parameters[i].value, verbosity);
                                    mavlink_message_t param_msg;
                                    uint8_t param_buffer[256];
                                    mavlink_msg_param_value_pack(system_id, component_id, &param_msg,
                                                                 parameters[i].name,
                                                                 parameters[i].value,
                                                                 parameters[i].type,
                                                                 PARAM_COUNT, i);
                                    int len = mavlink_msg_to_send_buffer(param_buffer, &param_msg);
                                    write(serial_fd, param_buffer, len);
                                    if (verbosity >= 1) {
                                        printf("Sent PARAM_VALUE for %s: %f\n", parameters[i].name, parameters[i].value);
                                    }
                                    break;
                                }
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_PARAM_SET: {
                            mavlink_param_set_t set_req;
                            mavlink_msg_param_set_decode(&msg, &set_req);
                            if (verbosity >= 1) {
                                printf("Received PARAM_SET for %s to value %f\n", set_req.param_id, set_req.param_value);
                            }
                            for (uint8_t i = 0; i < PARAM_COUNT; i++) {
                                if (strncmp(set_req.param_id, parameters[i].name, sizeof(parameters[i].name)) == 0) {
                                    if (set_parameter_in_system(parameters[i].name, set_req.param_value, verbosity)) {
                                        parameters[i].value = set_req.param_value;
                                        mavlink_message_t param_msg;
                                        uint8_t param_buffer[256];
                                        mavlink_msg_param_value_pack(system_id, component_id, &param_msg,
                                                                     parameters[i].name,
                                                                     parameters[i].value,
                                                                     parameters[i].type,
                                                                     PARAM_COUNT, i);
                                        int len = mavlink_msg_to_send_buffer(param_buffer, &param_msg);
                                        write(serial_fd, param_buffer, len);
                                        if (verbosity >= 1) {
                                            printf("Updated and sent PARAM_VALUE for %s: %f\n", parameters[i].name, parameters[i].value);
                                        }
                                    } else {
                                        if (verbosity >= 1) {
                                            printf("Failed to update parameter %s via system command.\n", parameters[i].name);
                                        }
                                    }
                                    break;
                                }
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_STATUSTEXT: {
                            mavlink_statustext_t statustext;
                            mavlink_msg_statustext_decode(&msg, &statustext);
                            char statustext_str[STATUSTEXT_MAX_LEN + 1];
                            strncpy(statustext_str, statustext.text, STATUSTEXT_MAX_LEN);
                            statustext_str[STATUSTEXT_MAX_LEN] = '\0';
                            char cmd[256], output[256];
                            snprintf(cmd, sizeof(cmd), "/usr/bin/autopilot_cmd statustext \"%s\"", statustext_str);
                            FILE *fp = popen(cmd, "r");
                            if (fp) {
                                if (fgets(output, sizeof(output), fp) != NULL) {
                                    if (verbosity >= 1) {
                                        printf("Processed STATUSTEXT via autopilot_cmd: %s\n", output);
                                    }
                                } else {
                                    if (verbosity >= 1) {
                                        printf("No output from command: %s\n", cmd);
                                    }
                                }
                                pclose(fp);
                            } else {
                                if (verbosity >= 1) {
                                    printf("Failed to execute command: %s\n", cmd);
                                }
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_RADIO_STATUS: {
                            mavlink_radio_status_t radio_status;
                            mavlink_msg_radio_status_decode(&msg, &radio_status);
                            /* If component id is 100 (Camera) and UDP output is enabled,
                               create a special text string and send it over UDP */
                            if (msg.compid == 100 && enable_radio_camera_udp) {
                                struct timespec ts;
                                clock_gettime(CLOCK_MONOTONIC, &ts);
                                long timestamp = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
                                int link_health_score_rssi;
                                if (radio_status.rssi == 0)
                                    link_health_score_rssi = 999;
                                else
                                    link_health_score_rssi = (int)round((radio_status.rssi * 1001.0 / 254.0) + 999);
                                int link_health_score_snr;
                                if (radio_status.noise == 0)
                                    link_health_score_snr = 999;
                                else
                                    link_health_score_snr = (int)round((radio_status.noise * 1001.0 / 254.0) + 999);
                                int best_antennas_rssi = (int)round((radio_status.remrssi * 256.0 / 254.0) - 128);
                                int best_antennas_snr = (int)round(radio_status.remnoise * 50.0 / 254.0);
                                int recovered_packets = (radio_status.fixed > 254 ? 254 : radio_status.fixed);
                                int recovered_packet_count = (radio_status.rxerrors > 254 ? 254 : radio_status.rxerrors);
                                char special_str[256];
                                snprintf(special_str, sizeof(special_str),
                                         "%ld:%d:%d:%d:%d:%d:%d:%d:%d",
                                         timestamp,
                                         link_health_score_rssi,
                                         link_health_score_snr,
                                         recovered_packets,
                                         recovered_packet_count,
                                         best_antennas_rssi,
                                         best_antennas_snr,
                                         best_antennas_snr,
                                         best_antennas_snr);
                                sendto(udp_sock, special_str, strlen(special_str), 0,
                                       (struct sockaddr*)&udp_addr, sizeof(udp_addr));
                                if (verbosity >= 1) {
                                    printf("UDP Special RADIO_STATUS: %s\n", special_str);
                                }
                            }
                            /* In all cases, store the radio status for echoing */
                            last_radio_status = radio_status;
                            last_radio_status_valid = true;
                            break;
                        }
                        case MAVLINK_MSG_ID_RAW_IMU: {
                            mavlink_msg_raw_imu_decode(&msg, &last_raw_imu);
                            last_raw_imu_valid = true;
                            break;
                        }
                        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
                            mavlink_msg_rc_channels_override_decode(&msg, &last_rc_channels);
                            last_rc_channels_valid = true;
                            break;
                        }
                        default:
                            break;
                    }
                }
            }
        }

        /* Update timers using clock_gettime */
        clock_gettime(CLOCK_MONOTONIC, &current);
        long heartbeat_elapsed = elapsed_ms(last_heartbeat, current);
        if (heartbeat_elapsed >= HEARTBEAT_INTERVAL_MS) {
            mavlink_message_t heartbeat;
            uint8_t hb_buffer[256];
            mavlink_msg_heartbeat_pack(system_id, component_id, &heartbeat,
                                       vehicle_type,
                                       autopilot_type,
                                       base_mode,
                                       custom_mode,
                                       MAV_STATE_ACTIVE);
            int len = mavlink_msg_to_send_buffer(hb_buffer, &heartbeat);
            write(serial_fd, hb_buffer, len);
            if (verbosity >= 1) {
                printf("Sent HEARTBEAT.\n");
            }
            last_heartbeat = current;
        }

        long echo_elapsed = elapsed_ms(last_echo, current);
        if (echo_elapsed >= ECHO_INTERVAL_MS) {
            mavlink_message_t echo_msg;
            uint8_t echo_buffer[256];
            int len;
            if (last_rc_channels_valid) {
                mavlink_msg_rc_channels_override_pack(system_id, component_id, &echo_msg,
                    last_rc_channels.target_system,
                    last_rc_channels.target_component,
                    last_rc_channels.chan1_raw,
                    last_rc_channels.chan2_raw,
                    last_rc_channels.chan3_raw,
                    last_rc_channels.chan4_raw,
                    last_rc_channels.chan5_raw,
                    last_rc_channels.chan6_raw,
                    last_rc_channels.chan7_raw,
                    last_rc_channels.chan8_raw,
                    last_rc_channels.chan9_raw,
                    last_rc_channels.chan10_raw,
                    last_rc_channels.chan11_raw,
                    last_rc_channels.chan12_raw,
                    last_rc_channels.chan13_raw,
                    last_rc_channels.chan14_raw,
                    last_rc_channels.chan15_raw,
                    last_rc_channels.chan16_raw,
                    last_rc_channels.chan17_raw,
                    last_rc_channels.chan18_raw);
                len = mavlink_msg_to_send_buffer(echo_buffer, &echo_msg);
                write(serial_fd, echo_buffer, len);
            }
            if (last_radio_status_valid) {
                mavlink_msg_radio_status_pack(system_id, component_id, &echo_msg,
                                              last_radio_status.rssi,
                                              last_radio_status.remrssi,
                                              last_radio_status.txbuf,
                                              last_radio_status.noise,
                                              last_radio_status.remnoise,
                                              last_radio_status.rxerrors,
                                              last_radio_status.fixed);
                len = mavlink_msg_to_send_buffer(echo_buffer, &echo_msg);
                write(serial_fd, echo_buffer, len);
            }
            if (last_raw_imu_valid) {
                mavlink_msg_raw_imu_pack(system_id, component_id, &echo_msg,
                                         last_raw_imu.time_usec,
                                         last_raw_imu.xacc,
                                         last_raw_imu.yacc,
                                         last_raw_imu.zacc,
                                         last_raw_imu.xgyro,
                                         last_raw_imu.ygyro,
                                         last_raw_imu.zgyro,
                                         last_raw_imu.xmag,
                                         last_raw_imu.ymag,
                                         last_raw_imu.zmag,
                                         last_raw_imu.id,
                                         last_raw_imu.temperature);
                len = mavlink_msg_to_send_buffer(echo_buffer, &echo_msg);
                write(serial_fd, echo_buffer, len);
            }
            last_echo = current;
        }

        long msg_check_elapsed = elapsed_ms(last_msg_check, current);
        if (msg_check_elapsed >= MSG_CHECK_INTERVAL_MS) {
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
                        if (verbosity >= 1) {
                            printf("Sent STATUSTEXT: %s\n", msg_text);
                        }
                    }
                    close(fd_msg);
                    unlink("/tmp/message.mavlink");
                }
            }
            last_msg_check = current;
        }

        /* Process monitored channels (if configured) */
        if (monitored_channel_count > 0 && last_rc_channels_valid) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            for (int i = 0; i < monitored_channel_count; i++) {
                int ch = monitored_channels[i].channel;
                int current_val = get_rc_channel_value(&last_rc_channels, ch);
                if (current_val < 0)
                    continue;
                if (!monitored_channels[i].baseline_initialized) {
                    monitored_channels[i].baseline = current_val;
                    monitored_channels[i].baseline_initialized = true;
                }
                int diff = abs(current_val - monitored_channels[i].baseline);
                if (diff > CHANNEL_THRESHOLD) {
                    if (!monitored_channels[i].active_change) {
                        monitored_channels[i].active_change = true;
                        clock_gettime(CLOCK_MONOTONIC, &monitored_channels[i].change_start);
                    } else {
                        long change_duration = elapsed_ms(monitored_channels[i].change_start, now);
                        if (change_duration >= CHANNEL_PERSIST_MS) {
                            long trigger_elapsed = elapsed_ms(monitored_channels[i].last_trigger, now);
                            if (trigger_elapsed >= CHANNEL_TRIGGER_DELAY_MS) {
                                char cmd[128];
                                char output[128];
                                snprintf(cmd, sizeof(cmd), "/usr/bin/autopilot_cmd %d %d", ch, current_val);
                                FILE *fp = popen(cmd, "r");
                                if (fp) {
                                    if (fgets(output, sizeof(output), fp) != NULL) {
                                        if (verbosity >= 1) {
                                            printf("Triggered channel %d with value %d: %s\n", ch, current_val, output);
                                        }
                                    } else {
                                        if (verbosity >= 1) {
                                            printf("No output from command: %s\n", cmd);
                                        }
                                    }
                                    pclose(fp);
                                } else {
                                    if (verbosity >= 1) {
                                        printf("Failed to execute command: %s\n", cmd);
                                    }
                                }
                                clock_gettime(CLOCK_MONOTONIC, &monitored_channels[i].last_trigger);
                                monitored_channels[i].baseline = current_val;
                                monitored_channels[i].active_change = false;
                            }
                        }
                    }
                } else {
                    monitored_channels[i].active_change = false;
                    monitored_channels[i].baseline = current_val;
                }
            }
        }

        usleep(SLEEP_US);
    }

    close(serial_fd);
    if (udp_sock >= 0) {
        close(udp_sock);
    }
    return 0;
}
