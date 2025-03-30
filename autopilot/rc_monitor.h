#ifndef RC_MONITOR_H
#define RC_MONITOR_H

#include "mavlink/common/mavlink.h"

// Initializes RC channel monitoring with a comma-separated list (e.g., "5,6").
void rc_monitor_init(const char *channels_arg);

// Checks for sustained changes in the monitored RC channels and triggers external commands.
void rc_monitor_update(const mavlink_rc_channels_override_t *last_rc, int verbosity);

#endif // RC_MONITOR_H
