#include "params.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Parameter table definition.
#define PARAM_COUNT 4
typedef struct {
    char name[16];
    float value;
    uint8_t type;  // using MAV_PARAM_TYPE_REAL32
} parameter_t;

static parameter_t parameters[PARAM_COUNT] = {
    {"VTX_POWER", 20.0f, MAV_PARAM_TYPE_REAL32},
    {"VIDEO_CH",  1.0f, MAV_PARAM_TYPE_REAL32},
    {"VIDEO_BR", 5000.0f, MAV_PARAM_TYPE_REAL32},
    {"VTX_PROFILE", 0.0f, MAV_PARAM_TYPE_REAL32}
};

void handle_param_request_list(uint8_t system_id, uint8_t component_id, int serial_fd, int verbosity) {
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
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
}

void handle_param_request_read(uint8_t system_id, uint8_t component_id, const mavlink_param_request_read_t *req, int serial_fd, int verbosity) {
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(req->param_id, parameters[i].name, sizeof(parameters[i].name)) == 0) {
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
}

void handle_param_set(uint8_t system_id, uint8_t component_id, const mavlink_param_set_t *set_req, int serial_fd, int verbosity) {
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(set_req->param_id, parameters[i].name, sizeof(parameters[i].name)) == 0) {
            if (set_parameter_in_system(parameters[i].name, set_req->param_value, verbosity)) {
                parameters[i].value = set_req->param_value;
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
}
