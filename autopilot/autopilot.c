/*
 * advanced_mavlink_autopilot.c
 *
 * A minimal autopilot that:
 *  - Sends periodic HEARTBEAT messages.
 *  - Responds to COMMAND_LONG with COMMAND_ACK.
 *  - Implements basic mission handling:
 *      * Replies to MISSION_REQUEST_LIST with a dummy mission (count=1).
 *      * Replies to MISSION_REQUEST with a dummy mission item.
 *  - Implements a parameter table for FPV-related settings:
 *      * VTX_POWER, VIDEO_CH, VIDEO_BR.
 *      * Responds to PARAM_REQUEST_LIST, PARAM_REQUEST_READ, and PARAM_SET.
 *  - Saves the latest received RC_CHANNELS_OVERRIDE, RADIO_STATUS, and RAW_IMU
 *    messages and echoes them back every 500ms.
 *  - Checks every 100ms for a file (/tmp/message.mavlink) containing a text
 *    message. If found, sends it as a STATUSTEXT message (which shows up in QGC)
 *    and deletes the file.
 *
 * Command-line options:
 *   -v             : Increase verbosity (can be specified twice for extra detail)
 *   -s <system_id> : Set the MAVLink system ID (default: 1)
 *   -c <comp_id>   : Set the MAVLink component ID (default: MAV_COMP_ID_AUTOPILOT1)
 *   -t <veh_type>  : Set the vehicle type (default: MAV_TYPE_FIXED_WING)
 *   -a <ap_type>   : Set the autopilot type (default: MAV_AUTOPILOT_GENERIC)
 *   -m <custom_mode>: Set the custom mode (default: 0)
 *   -b <base_mode> : Set the base mode (default: 0)
 *
 * Usage example:
 *   ./advanced_mavlink_autopilot -v -s 1 -c 240 -t 2 -a 3 -m 0 -b 81
 *   ./advanced_mavlink_autopilot -vv ... (for extra verbose output)
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
#include <sys/time.h>
#include <signal.h>
#include <stdbool.h>
#include <getopt.h>
#include <sys/stat.h>
#include "mavlink/common/mavlink.h"  // Ensure the correct MAVLink dialect header is included

#define SERIAL_PORT "/dev/ttyS2"
#define BAUDRATE B460800
#define HEARTBEAT_INTERVAL_SEC 1
#define ECHO_INTERVAL_MS 500
#define MSG_CHECK_INTERVAL_MS 100
#define MESSAGE_FILE "/tmp/message.mavlink"
#define STATUSTEXT_MAX_LEN 50

volatile bool running = true;

/* Global storage for the latest received messages to echo back */
static mavlink_rc_channels_override_t last_rc_channels = {0};
static bool last_rc_channels_valid = false;

static mavlink_radio_status_t last_radio_status = {0};
static bool last_radio_status_valid = false;

static mavlink_raw_imu_t last_raw_imu = {0};
static bool last_raw_imu_valid = false;

/* Parameter table definition */
typedef struct {
    char name[16];
    float value;
    uint8_t type;  // using MAV_PARAM_TYPE_REAL32
} parameter_t;

#define PARAM_COUNT 3
static parameter_t parameters[PARAM_COUNT] = {
    {"VTX_POWER", 20.0f, MAV_PARAM_TYPE_REAL32},   // e.g., power in dBm or mW
    {"VIDEO_CH",  1.0f, MAV_PARAM_TYPE_REAL32},      // video channel
    {"VIDEO_BR", 5000.0f, MAV_PARAM_TYPE_REAL32}      // video bitrate (e.g., in kbps)
};

/* Dummy mission parameters */
#define MISSION_COUNT 1

/* Dummy mission item values (example waypoint) */
#define DUMMY_LATITUDE   47.397742f   // degrees
#define DUMMY_LONGITUDE   8.545594f   // degrees
#define DUMMY_ALTITUDE   10.0f       // meters

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

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // disable break processing
    tty.c_lflag = 0;                            // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                        // non-blocking read
    tty.c_cc[VTIME] = 5;                        // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // shut off xon/xoff control
    tty.c_cflag |= (CLOCAL | CREAD);             // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);           // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return -1;
    }
    return 0;
}

void usage(const char *progname) {
    printf("Usage: %s [-v] [-s system_id] [-c component_id] [-t vehicle_type] [-a autopilot_type] [-m custom_mode] [-b base_mode]\n", progname);
}

int main(int argc, char *argv[]) {
    int verbosity = 0; // 0: quiet, 1: moderate, 2: extra verbose
    uint8_t system_id = 1;                        // Default system ID
    uint8_t component_id = MAV_COMP_ID_AUTOPILOT1;  // Default component ID
    uint8_t vehicle_type = MAV_TYPE_FIXED_WING;     // Default vehicle type
    uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;   // Default autopilot type
    uint32_t custom_mode = 0;                       // Default custom mode
    uint8_t base_mode = 0;                          // Default base mode

    int opt;
    while ((opt = getopt(argc, argv, "v:s:c:t:a:m:b:")) != -1) {
        switch (opt) {
            case 'v':
                verbosity++;  // Use -v for moderate, -vv for extra verbose
                break;
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
            default:
                usage(argv[0]);
                exit(EXIT_FAILURE);
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

    struct timeval last_heartbeat, last_echo, last_msg_check, current;
    gettimeofday(&last_heartbeat, NULL);
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
    }

    while (running) {
        int n = read(serial_fd, buffer, sizeof(buffer));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                    if (verbosity >= 2) {  // Extra verbose prints every received message
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
                            /* Use the received target_system/target_component for reply */
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
                            /* Send all parameters in our table */
                            for (uint8_t i = 0; i < PARAM_COUNT; i++) {
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
                                    printf("Sent PARAM_VALUE for %s\n", parameters[i].name);
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
                            /* Search our parameter table */
                            for (uint8_t i = 0; i < PARAM_COUNT; i++) {
                                if (strncmp(req.param_id, parameters[i].name, sizeof(parameters[i].name)) == 0) {
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
                                        printf("Sent PARAM_VALUE for %s\n", parameters[i].name);
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
                            /* Update parameter if found */
                            for (uint8_t i = 0; i < PARAM_COUNT; i++) {
                                if (strncmp(set_req.param_id, parameters[i].name, sizeof(parameters[i].name)) == 0) {
                                    parameters[i].value = set_req.param_value;
                                    /* Respond with updated parameter */
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
                                        printf("Updated and sent PARAM_VALUE for %s\n", parameters[i].name);
                                    }
                                    break;
                                }
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
                            if (verbosity >= 1) {
                                printf("Received MISSION_REQUEST_LIST from system %d, component %d\n", msg.sysid, msg.compid);
                            }
                            mavlink_message_t mission_count_msg;
                            uint8_t mission_buffer[256];
                            /* Reply with a mission count of 1 and type MAV_MISSION_TYPE_MISSION (usually 0) */
                            mavlink_msg_mission_count_pack(system_id, component_id, &mission_count_msg,
                                                           msg.sysid, msg.compid,
                                                           MISSION_COUNT, MAV_MISSION_TYPE_MISSION);
                            int len = mavlink_msg_to_send_buffer(mission_buffer, &mission_count_msg);
                            write(serial_fd, mission_buffer, len);
                            if (verbosity >= 1) {
                                printf("Sent MISSION_COUNT with count=%d\n", MISSION_COUNT);
                            }
                            break;
                        }
                        case MAVLINK_MSG_ID_MISSION_REQUEST: {
                            mavlink_mission_request_t req;
                            mavlink_msg_mission_request_decode(&msg, &req);
                            if (verbosity >= 1) {
                                printf("Received MISSION_REQUEST for seq %d\n", req.seq);
                            }
                            /* For our single mission item (seq==0) */
                            if (req.seq == 0) {
                                mavlink_message_t mission_item_msg;
                                uint8_t mission_buffer[256];
                                /* Pack a dummy mission item (e.g., a waypoint) */
                                mavlink_msg_mission_item_pack(system_id, component_id, &mission_item_msg,
                                                              msg.sysid, msg.compid,  // target system/component
                                                              0,                     // seq
                                                              MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                                              MAV_CMD_NAV_WAYPOINT,
                                                              0,                     // current (0=false)
                                                              1,                     // autocontinue
                                                              0.0f, 0.0f, 0.0f, 0.0f, // params 1-4
                                                              DUMMY_LATITUDE,
                                                              DUMMY_LONGITUDE,
                                                              DUMMY_ALTITUDE,
                                                              MAV_MISSION_TYPE_MISSION);
                                int len = mavlink_msg_to_send_buffer(mission_buffer, &mission_item_msg);
                                write(serial_fd, mission_buffer, len);
                                if (verbosity >= 1) {
                                    printf("Sent MISSION_ITEM for seq 0\n");
                                }
                            }
                            break;
                        }
                        /* Save incoming messages for echoing */
                        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
                            mavlink_msg_rc_channels_override_decode(&msg, &last_rc_channels);
                            last_rc_channels_valid = true;
                            break;
                        }
                        case MAVLINK_MSG_ID_RADIO_STATUS: {
                            mavlink_msg_radio_status_decode(&msg, &last_radio_status);
                            last_radio_status_valid = true;
                            break;
                        }
                        case MAVLINK_MSG_ID_RAW_IMU: {
                            mavlink_msg_raw_imu_decode(&msg, &last_raw_imu);
                            last_raw_imu_valid = true;
                            break;
                        }
                        default:
                            break;
                    } // switch
                }
            }
        }

        /* Send heartbeat every HEARTBEAT_INTERVAL_SEC */
        gettimeofday(&current, NULL);
        if ((current.tv_sec - last_heartbeat.tv_sec) >= HEARTBEAT_INTERVAL_SEC) {
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

        /* Echo stored messages every ECHO_INTERVAL_MS (500ms) */
        long elapsed_ms = (current.tv_sec - last_echo.tv_sec) * 1000 +
                          (current.tv_usec - last_echo.tv_usec) / 1000;
        if (elapsed_ms >= ECHO_INTERVAL_MS) {
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

        /* Check every MSG_CHECK_INTERVAL_MS (100ms) for a message file */
        long elapsed_msg_ms = (current.tv_sec - last_msg_check.tv_sec) * 1000 +
                              (current.tv_usec - last_msg_check.tv_usec) / 1000;
        if (elapsed_msg_ms >= MSG_CHECK_INTERVAL_MS) {
            if (access(MESSAGE_FILE, F_OK) == 0) {
                int fd_msg = open(MESSAGE_FILE, O_RDONLY);
                if (fd_msg >= 0) {
                    char msg_text[STATUSTEXT_MAX_LEN + 1];
                    int nread = read(fd_msg, msg_text, STATUSTEXT_MAX_LEN);
                    if (nread > 0) {
                        msg_text[nread] = '\0';
                        mavlink_message_t statustext_msg;
                        uint8_t statustext_buffer[256];
                        /* Use severity INFO (6) and pass 0 for id and chunk_seq */
                        mavlink_msg_statustext_pack(system_id, component_id, &statustext_msg,
                                                    MAV_SEVERITY_INFO, msg_text, 0, 0);
                        int len = mavlink_msg_to_send_buffer(statustext_buffer, &statustext_msg);
                        write(serial_fd, statustext_buffer, len);
                        if (verbosity >= 1) {
                            printf("Sent STATUSTEXT: %s\n", msg_text);
                        }
                    }
                    close(fd_msg);
                    unlink(MESSAGE_FILE);
                }
            }
            last_msg_check = current;
        }
    }

    close(serial_fd);
    return 0;
}
