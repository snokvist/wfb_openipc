#ifndef PARAMS_H
#define PARAMS_H

#include "mavlink/common/mavlink.h"

// These functions handle incoming parameter messages.
void handle_param_request_list(uint8_t system_id, uint8_t component_id, int serial_fd, int verbosity);
void handle_param_request_read(uint8_t system_id, uint8_t component_id, const mavlink_param_request_read_t *req, int serial_fd, int verbosity);
void handle_param_set(uint8_t system_id, uint8_t component_id, const mavlink_param_set_t *set_req, int serial_fd, int verbosity);

#endif // PARAMS_H
