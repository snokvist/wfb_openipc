#ifndef UTILS_H
#define UTILS_H

#include <termios.h>  // for speed_t
#include <time.h>
#include "mavlink/common/mavlink.h"

// Returns the elapsed milliseconds between two CLOCK_MONOTONIC timestamps.
long elapsed_ms(struct timespec start, struct timespec end);

// Runs an external command using popen(). The command output is written to output (up to output_size).
// Returns 0 on success, nonzero on error.
int run_command(const char *cmd, char *output, size_t output_size);

// Retrieve a parameter from the system by calling an external command.
// Returns the new value, or current_value if an error occurred.
float get_parameter_from_system(const char *param, float current_value, int verbosity);

// Set a parameter in the system by calling an external command.
// Returns true on success.
bool set_parameter_in_system(const char *param, float new_value, int verbosity);

// Returns the raw RC channel value from a MAVLink RC_CHANNELS_OVERRIDE message.
int get_rc_channel_value(const mavlink_rc_channels_override_t *rc, int channel);

// Parses a string baud rate into a speed_t constant.
speed_t parse_baudrate(const char *str);

#endif // UTILS_H
