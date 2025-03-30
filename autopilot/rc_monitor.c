#include "rc_monitor.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Monitoring thresholds and timing.
#define MAX_MONITORED_CHANNELS 8
#define CHANNEL_THRESHOLD 20
#define CHANNEL_PERSIST_MS 100
#define CHANNEL_TRIGGER_DELAY_MS 1000

typedef struct {
    int channel;   // Allowed channels: 5 to 12.
    int baseline;  // Last known value.
    int baseline_initialized;
    struct timespec change_start; // When change began.
    struct timespec last_trigger; // Last time command triggered.
    int active_change;            // True if change is ongoing.
} monitored_channel_t;

static monitored_channel_t monitored_channels[MAX_MONITORED_CHANNELS];
static int monitored_channel_count = 0;

void rc_monitor_init(const char *channels_arg) {
    if (channels_arg == NULL)
        return;
    char *copy = strdup(channels_arg);
    char *token = strtok(copy, ",");
    while (token != NULL && monitored_channel_count < MAX_MONITORED_CHANNELS) {
        int ch = atoi(token);
        if (ch >= 5 && ch <= 12) {
            monitored_channels[monitored_channel_count].channel = ch;
            monitored_channels[monitored_channel_count].baseline = -1;
            monitored_channels[monitored_channel_count].baseline_initialized = 0;
            monitored_channels[monitored_channel_count].active_change = 0;
            monitored_channels[monitored_channel_count].change_start.tv_sec = 0;
            monitored_channels[monitored_channel_count].change_start.tv_nsec = 0;
            monitored_channels[monitored_channel_count].last_trigger.tv_sec = 0;
            monitored_channels[monitored_channel_count].last_trigger.tv_nsec = 0;
            monitored_channel_count++;
        }
        token = strtok(NULL, ",");
    }
    free(copy);
}

void rc_monitor_update(const mavlink_rc_channels_override_t *last_rc, int verbosity) {
    if (monitored_channel_count == 0 || last_rc == NULL)
        return;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    for (int i = 0; i < monitored_channel_count; i++) {
        int ch = monitored_channels[i].channel;
        int current_val = get_rc_channel_value(last_rc, ch);
        if (current_val < 0)
            continue;
        if (!monitored_channels[i].baseline_initialized) {
            monitored_channels[i].baseline = current_val;
            monitored_channels[i].baseline_initialized = 1;
        }
        int diff = abs(current_val - monitored_channels[i].baseline);
        if (diff > CHANNEL_THRESHOLD) {
            if (!monitored_channels[i].active_change) {
                monitored_channels[i].active_change = 1;
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
                        monitored_channels[i].active_change = 0;
                    }
                }
            }
        } else {
            monitored_channels[i].active_change = 0;
            monitored_channels[i].baseline = current_val;
        }
    }
}
