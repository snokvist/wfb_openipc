#include "mavlink_handler.h"
#include "params.h"
#include "utils.h"
#include "alink.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define STATUSTEXT_MAX_LEN 50

// Static globals to store the latest messages.
static mavlink_rc_channels_override_t last_rc_channels;
static int last_rc_channels_valid = 0;
static mavlink_radio_status_t last_radio_status;
static int last_radio_status_valid = 0;
static mavlink_raw_imu_t last_raw_imu;
static int last_raw_imu_valid = 0;

void process_mavlink_message(const mavlink_message_t *msg, int serial_fd, int verbosity) {
    switch(msg->msgid) {
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(msg, &cmd);
            if (verbosity >= 2) {
                printf("Received COMMAND_LONG: command %d, confirmation %d\n", cmd.command, cmd.confirmation);
            }
            mavlink_message_t ack_msg;
            uint8_t ack_buffer[256];
            // Using system_id and component_id as 1 (could be parameterized)
            mavlink_msg_command_ack_pack(1, 1, &ack_msg,
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
                printf("Received PARAM_REQUEST_LIST\n");
            }
            handle_param_request_list(1, 1, serial_fd, verbosity);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
            mavlink_param_request_read_t req;
            mavlink_msg_param_request_read_decode(msg, &req);
            if (verbosity >= 1) {
                printf("Received PARAM_REQUEST_READ for %s\n", req.param_id);
            }
            handle_param_request_read(1, 1, &req, serial_fd, verbosity);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_SET: {
            mavlink_param_set_t set_req;
            mavlink_msg_param_set_decode(msg, &set_req);
            if (verbosity >= 1) {
                printf("Received PARAM_SET for %s to value %f\n", set_req.param_id, set_req.param_value);
            }
            handle_param_set(1, 1, &set_req, serial_fd, verbosity);
            break;
        }
        case MAVLINK_MSG_ID_STATUSTEXT: {
            mavlink_statustext_t statustext;
            mavlink_msg_statustext_decode(msg, &statustext);
            char statustext_str[STATUSTEXT_MAX_LEN + 1];
            strncpy(statustext_str, statustext.text, STATUSTEXT_MAX_LEN);
            statustext_str[STATUSTEXT_MAX_LEN] = '\0';
            char cmd[256], output[256];
            snprintf(cmd, sizeof(cmd), "/usr/bin/autopilot_cmd statustext \"%s\"", statustext_str);
            if (run_command(cmd, output, sizeof(output)) == 0 && verbosity >= 1) {
                printf("Processed STATUSTEXT via autopilot_cmd: %s\n", output);
            } else if (verbosity >= 1) {
                printf("Failed to process STATUSTEXT command: %s\n", cmd);
            }
            break;
        }
        case MAVLINK_MSG_ID_RADIO_STATUS: {
            mavlink_radio_status_t radio_status;
            mavlink_msg_radio_status_decode(msg, &radio_status);
            if (msg->compid == 100) { // Camera
                alink_process_radio_status(&radio_status, verbosity);
            }
            last_radio_status = radio_status;
            last_radio_status_valid = 1;
            break;
        }
        case MAVLINK_MSG_ID_RAW_IMU: {
            mavlink_msg_raw_imu_decode(msg, &last_raw_imu);
            last_raw_imu_valid = 1;
            break;
        }
        case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
            mavlink_msg_rc_channels_override_decode(msg, &last_rc_channels);
            last_rc_channels_valid = 1;
            break;
        }
        default:
            if (verbosity >= 2) {
                printf("Unhandled MAVLink message ID: %d\n", msg->msgid);
            }
            break;
    }
}

const mavlink_rc_channels_override_t* get_last_rc_channels(void) {
    return last_rc_channels_valid ? &last_rc_channels : NULL;
}

const mavlink_radio_status_t* get_last_radio_status(void) {
    return last_radio_status_valid ? &last_radio_status : NULL;
}

const mavlink_raw_imu_t* get_last_raw_imu(void) {
    return last_raw_imu_valid ? &last_raw_imu : NULL;
}
