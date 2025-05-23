#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

long elapsed_ms(struct timespec start, struct timespec end) {
    return (end.tv_sec - start.tv_sec) * 1000 +
           (end.tv_nsec - start.tv_nsec) / 1000000;
}

int run_command(const char *cmd, char *output, size_t output_size) {
    FILE *fp = popen(cmd, "r");
    if (!fp) {
        return -1;
    }
    if (fgets(output, output_size, fp) == NULL) {
        pclose(fp);
        return -1;
    }
    pclose(fp);
    return 0;
}

float get_parameter_from_system(const char *param, float current_value, int verbosity) {
    char command[128];
    char buffer[128];
    FILE *fp;
    snprintf(command, sizeof(command), "/usr/bin/autopilot_cmd get %s", param);
    fp = popen(command, "r");
    if (!fp) {
        if (verbosity >= 1)
            fprintf(stderr, "Error: Failed to run command: %s\n", command);
        return current_value;
    }
    if (fgets(buffer, sizeof(buffer), fp) != NULL) {
        if (strncmp(buffer, "OK", 2) == 0) {
            float val = atof(buffer + 3);
            pclose(fp);
            return val;
        } else {
            if (verbosity >= 1)
                fprintf(stderr, "Error: Command '%s' returned: %s\n", command, buffer);
        }
    }
    pclose(fp);
    return current_value;
}

bool set_parameter_in_system(const char *param, float new_value, int verbosity) {
    char command[128];
    char buffer[128];
    FILE *fp;
    snprintf(command, sizeof(command), "/usr/bin/autopilot_cmd set %s %f", param, new_value);
    fp = popen(command, "r");
    if (!fp) {
        if (verbosity >= 1)
            fprintf(stderr, "Error: Failed to run command: %s\n", command);
        return false;
    }
    if (fgets(buffer, sizeof(buffer), fp) != NULL) {
        if (strncmp(buffer, "OK", 2) == 0) {
            pclose(fp);
            return true;
        } else {
            if (verbosity >= 1)
                fprintf(stderr, "Error: Command '%s' returned: %s\n", command, buffer);
        }
    }
    pclose(fp);
    return false;
}

int get_rc_channel_value(const mavlink_rc_channels_override_t *rc, int channel) {
    switch(channel) {
        case 1: return rc->chan1_raw;
        case 2: return rc->chan2_raw;
        case 3: return rc->chan3_raw;
        case 4: return rc->chan4_raw;
        case 5: return rc->chan5_raw;
        case 6: return rc->chan6_raw;
        case 7: return rc->chan7_raw;
        case 8: return rc->chan8_raw;
        case 9: return rc->chan9_raw;
        case 10: return rc->chan10_raw;
        case 11: return rc->chan11_raw;
        case 12: return rc->chan12_raw;
        case 13: return rc->chan13_raw;
        case 14: return rc->chan14_raw;
        case 15: return rc->chan15_raw;
        case 16: return rc->chan16_raw;
        case 17: return rc->chan17_raw;
        case 18: return rc->chan18_raw;
        default: return -1;
    }
}

speed_t parse_baudrate(const char *str) {
    long baud = atol(str);
    switch (baud) {
        case 50: return B50;
        case 75: return B75;
        case 110: return B110;
        case 134: return B134;
        case 150: return B150;
        case 200: return B200;
        case 300: return B300;
        case 600: return B600;
        case 1200: return B1200;
        case 1800: return B1800;
        case 2400: return B2400;
        case 4800: return B4800;
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
        default:
            fprintf(stderr, "Unsupported baud rate: %ld. Defaulting to 115200.\n", baud);
            return B115200;
    }
}
