#ifndef GPS_H
#define GPS_H

#include "mavlink/common/mavlink.h"

#define GPS_BUFFER_SIZE 300

// Buffer for GLOBAL_POSITION_INT messages.
typedef struct {
    mavlink_global_position_int_t messages[GPS_BUFFER_SIZE];
    int count;  // number of valid messages in buffer
    int index;  // next insertion index (circular)
} gps_global_buffer_t;

// Buffer for GPS_INPUT messages.
typedef struct {
    mavlink_gps_input_t messages[GPS_BUFFER_SIZE];
    int count;
    int index;
} gps_input_buffer_t;

// Initialize GPS module with given serial port, baud rate and filter flag.
// Returns a file descriptor for the GPS UART or -1 on error.
int gps_init(const char* port, speed_t baudrate, int filter_enabled, int verbosity);

// Process incoming GPS data from the GPS UART (nonblocking).
// This function parses MAVLink messages and stores GLOBAL_POSITION_INT and GPS_INPUT messages into their buffers.
void gps_process(int gps_fd, int verbosity);

// Stream the latest GLOBAL_POSITION_INT message over the main serial port every 200ms.
void gps_stream_update(int serial_fd, int verbosity);

// Getter functions for the GPS buffers (for webui endpoints, etc.).
const gps_global_buffer_t* gps_get_global_buffer(void);
const gps_input_buffer_t* gps_get_input_buffer(void);

#endif // GPS_H
