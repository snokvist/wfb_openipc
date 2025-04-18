#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gio/gio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <errno.h>
#include <time.h>

#define SOCKET_PATH "/tmp/rtp_local"
#define MAX_PACKET_SIZE 4096
#define RTP_HEADER_LEN 12

static guint  pkt_counter = 0;
static time_t last_report = 0;

/* socket → appsrc */
static gboolean on_socket(GIOChannel *src, GIOCondition cond, gpointer user)
{
    GstAppSrc *appsrc = GST_APP_SRC(user);
    guint8 buf[MAX_PACKET_SIZE];
    ssize_t n = recv(g_io_channel_unix_get_fd(src), buf, sizeof(buf), 0);
    if (n <= RTP_HEADER_LEN) return TRUE;

    GstBuffer *buffer = gst_buffer_new_allocate(NULL, n, NULL);
    gst_buffer_fill(buffer, 0, buf, n);
    if (gst_app_src_push_buffer(appsrc, buffer) != GST_FLOW_OK) {
        g_printerr("push error\n");
        return FALSE;
    }
    pkt_counter++; time_t now=time(NULL);
    if(now!=last_report){g_print("[INFO] pkts/s %u\n",pkt_counter);pkt_counter=0;last_report=now;}
    return TRUE;
}

int main(int argc,char**argv)
{
    unlink(SOCKET_PATH);
    int sock=socket(AF_UNIX,SOCK_DGRAM,0);
    if(sock<0){perror("socket");return 1;}
    struct sockaddr_un addr={0};addr.sun_family=AF_UNIX;strncpy(addr.sun_path,SOCKET_PATH,sizeof(addr.sun_path)-1);
    if(bind(sock,(struct sockaddr*)&addr,sizeof(addr))<0){perror("bind");return 1;}

    gst_init(&argc,&argv);
    GstElement *pipe=gst_pipeline_new("p"),
               *src =gst_element_factory_make("appsrc","src"),
               *capsf=gst_element_factory_make("capsfilter",NULL),
               *jb  =gst_element_factory_make("rtpjitterbuffer","jb"),
               *queue=gst_element_factory_make("queue",NULL),
               *depay=gst_element_factory_make("rtph265depay",NULL),
               *parse=gst_element_factory_make("h265parse",NULL),
               *dec  =gst_element_factory_make("mppvideodec",NULL),
               *sink =gst_element_factory_make("kmssink",NULL);
    if(!pipe||!src||!capsf||!jb||!queue||!depay||!parse||!dec||!sink){g_printerr("element fail\n");return 1;}

    GstCaps *caps=gst_caps_from_string("application/x-rtp, media=video, clock-rate=90000, encoding-name=H265");
    g_object_set(capsf,"caps",caps,NULL);gst_caps_unref(caps);
        g_object_set(src,
                 "stream-type", 0,   /* GST_APP_STREAM_TYPE_STREAM */
                 "is-live",    TRUE,
                 NULL);
        /* jitterbuffer tuned for Wi‑Fi bursts */
    g_object_set(jb,
                 "latency",          300,   /* ms target */
                 "drop-on-latency", TRUE,
                 NULL);

    /* queue: cap at 200 ms worth of data to avoid extra delay */
    g_object_set(queue,
                 "max-size-time",   200000000, /* 200 ms */
                 "max-size-buffers", 0,
                 "max-size-bytes",   0,
                 NULL);

    /* decoder: show late frames instead of dropping */
    g_object_set(dec, "qos", FALSE, NULL);

    /* sink: present immediately (no clock sync) */
    g_object_set(sink, "sync", FALSE, NULL);

    gst_bin_add_many(GST_BIN(pipe),src,capsf,jb,queue,depay,parse,dec,sink,NULL);
    if(!gst_element_link_many(src,capsf,jb,queue,depay,parse,dec,sink,NULL)){
        g_printerr("link fail\n");return 1;}

    GIOChannel *chan=g_io_channel_unix_new(sock);
    g_io_add_watch(chan,G_IO_IN|G_IO_ERR|G_IO_HUP,on_socket,src);

    gst_element_set_state(pipe,GST_STATE_PLAYING);
    g_print("[INFO] pipeline running\n");
    GMainLoop *loop=g_main_loop_new(NULL,FALSE);
    g_main_loop_run(loop);

    gst_element_set_state(pipe,GST_STATE_NULL);
    gst_object_unref(pipe);
    g_main_loop_unref(loop);
    close(sock);unlink(SOCKET_PATH);
    return 0;
}
