#include "alink.h"
#include "utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <math.h>

static int udp_sock = -1;
static struct sockaddr_in udp_addr;
static int udp_initialized = 0;

int alink_init(void) {
    udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_sock < 0) {
        perror("UDP socket");
        return -1;
    }
    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_port = htons(9999);
    if (inet_pton(AF_INET, "127.0.0.1", &udp_addr.sin_addr) <= 0) {
        perror("inet_pton");
        close(udp_sock);
        return -1;
    }
    udp_initialized = 1;
    return 0;
}

void alink_process_radio_status(const mavlink_radio_status_t *radio_status, int verbosity) {
    if (!udp_initialized)
        return;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    long timestamp = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    int link_health_score_rssi;
    if (radio_status->rssi == 0)
        link_health_score_rssi = 999;
    else
        link_health_score_rssi = (int)round((radio_status->rssi * 1001.0 / 254.0) + 999);
    int link_health_score_snr;
    if (radio_status->noise == 0)
        link_health_score_snr = 999;
    else
        link_health_score_snr = (int)round((radio_status->noise * 1001.0 / 254.0) + 999);
    int best_antennas_rssi = (int)round((radio_status->remrssi * 256.0 / 254.0) - 128);
    int best_antennas_snr = (int)round(radio_status->remnoise * 50.0 / 254.0);
    int recovered_packets = radio_status->fixed > 254 ? 254 : radio_status->fixed;
    int recovered_packet_count = radio_status->rxerrors > 254 ? 254 : radio_status->rxerrors;
    char special_str[256];
    snprintf(special_str, sizeof(special_str),
             "%ld:%d:%d:%d:%d:%d:%d:%d:%d",
             timestamp,
             link_health_score_rssi,
             link_health_score_snr,
             recovered_packets,
             recovered_packet_count,
             best_antennas_rssi,
             best_antennas_snr,
             best_antennas_snr,
             best_antennas_snr);
    sendto(udp_sock, special_str, strlen(special_str), 0,
           (struct sockaddr*)&udp_addr, sizeof(udp_addr));
    if (verbosity >= 1) {
        printf("UDP Special RADIO_STATUS: %s\n", special_str);
    }
}
