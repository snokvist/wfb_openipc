#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gio/gio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <time.h>

#define SOCKET_PATH "/tmp/rtp_local"
#define MAX_PACKET_SIZE 4096

static guint packet_counter = 0;
static time_t last_report_time = 0;

/* Callback that pulls a datagram from the UNIX socket and pushes it into appsrc */
static gboolean read_unix_socket(GIOChannel *source, GIOCondition cond, gpointer data)
{
    GstAppSrc *appsrc = GST_APP_SRC(data);
    char buffer[MAX_PACKET_SIZE];
    ssize_t received = recv(g_io_channel_unix_get_fd(source), buffer, sizeof(buffer), 0);

    if (received < 0) {
        if (errno != EINTR)
            perror("recv");
        return TRUE; /* keep watching */
    }

    /* Wrap bytes in a GstBuffer */
    GstBuffer *buf = gst_buffer_new_allocate(NULL, received, NULL);
    gst_buffer_fill(buf, 0, buffer, received);

    /* Push into the pipeline */
    if (gst_app_src_push_buffer(appsrc, buf) != GST_FLOW_OK) {
        g_printerr("[ERR ] gst_app_src_push_buffer failed\n");
        return FALSE; /* stop the mainloop */
    }

    /* Simple throughput counter */
    packet_counter++;
    time_t now = time(NULL);
    if (now != last_report_time) {
        g_print("[INFO] Packets pushed in last second: %u\n", packet_counter);
        packet_counter = 0;
        last_report_time = now;
    }
    return TRUE;
}

int main(int argc, char *argv[])
{
    /* ---------- 1.  Prepare UNIX DATAGRAM SOCKET ---------- */
    g_print("[INFO] Starting RTP bridge on %s\n", SOCKET_PATH);
    unlink(SOCKET_PATH);
    int sockfd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket");
        return EXIT_FAILURE;
    }

    struct sockaddr_un addr = {0};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sockfd);
        return EXIT_FAILURE;
    }

    /* ---------- 2.  Build GStreamer pipeline ---------- */
    gst_init(&argc, &argv);

    GstElement *pipeline   = gst_pipeline_new("rtp-pipeline");
    GstElement *appsrc     = gst_element_factory_make("appsrc",        "src");
    GstElement *capsfilter = gst_element_factory_make("capsfilter",    NULL);
    GstElement *jitterbuf  = gst_element_factory_make("rtpjitterbuffer","jb");
    GstElement *depay      = gst_element_factory_make("rtph265depay",  NULL);
    GstElement *parse      = gst_element_factory_make("h265parse",     NULL);
    GstElement *decoder    = gst_element_factory_make("mppvideodec",   NULL);
    GstElement *sink       = gst_element_factory_make("kmssink",       NULL);

    if (!pipeline||!appsrc||!capsfilter||!jitterbuf||!depay||!parse||!decoder||!sink) {
        g_printerr("[ERR ] Failed to create some GStreamer elements\n");
        return EXIT_FAILURE;
    }

    /* capsfilter to describe incoming RTP */
    GstCaps *caps = gst_caps_from_string(
        "application/x-rtp, media=video, clock-rate=90000, encoding-name=H265");
    g_object_set(capsfilter, "caps", caps, NULL);
    gst_caps_unref(caps);

    /* appsrc properties */
    g_object_set(appsrc,
                 "stream-type", 0,          /* GST_APP_STREAM_TYPE_STREAM */
                 "is-live", TRUE,
                 "do-timestamp", TRUE,      /* Attach running-time stamps */
                 NULL);

    /* jitterbuffer: 200 ms to start; tune as needed */
    g_object_set(jitterbuf, "latency", 200, NULL);

    /* add + link */
    gst_bin_add_many(GST_BIN(pipeline),
                     appsrc, capsfilter, jitterbuf, depay, parse, decoder, sink, NULL);
    if (!gst_element_link_many(appsrc, capsfilter, jitterbuf, depay, parse, decoder, sink, NULL)) {
        g_printerr("[ERR ] Pipeline linking failed\n");
        return EXIT_FAILURE;
    }

    /* ---------- 3.  Feed socket into appsrc via GIOChannel ---------- */
    GIOChannel *channel = g_io_channel_unix_new(sockfd);
    g_io_add_watch(channel, G_IO_IN | G_IO_ERR | G_IO_HUP, read_unix_socket, appsrc);

    /* ---------- 4.  Run ---------- */
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_print("[INFO] Pipeline PLAYING; waiting for data…\n");

    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);

    /* ---------- 5.  Cleanup ---------- */
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    g_main_loop_unref(loop);
    close(sockfd);
    unlink(SOCKET_PATH);
    return EXIT_SUCCESS;
}
