#ifndef ALINK_H
#define ALINK_H

#include "mavlink/common/mavlink.h"

// Initializes the "alink" UDP module. Returns 0 on success, nonzero on error.
int alink_init(void);

// Processes a RADIO_STATUS message (from component 100) by mapping its fields into a special text string
// and sending it via UDP. Also logs to console if verbosity is enabled.
void alink_process_radio_status(const mavlink_radio_status_t *radio_status, int verbosity);

#endif // ALINK_H
