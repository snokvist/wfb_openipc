#ifndef MAVLINK_HANDLER_H
#define MAVLINK_HANDLER_H

#include "mavlink/common/mavlink.h"

// Processes an incoming MAVLink message.
void process_mavlink_message(const mavlink_message_t *msg, int serial_fd, int verbosity);

// Getter functions to access the latest messages.
const mavlink_rc_channels_override_t* get_last_rc_channels(void);
const mavlink_radio_status_t* get_last_radio_status(void);
const mavlink_raw_imu_t* get_last_raw_imu(void);

#endif // MAVLINK_HANDLER_H
