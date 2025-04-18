wfb_rx_unix -p 0 -K /etc/gs.key -R 2097152 -l 1000 -i 7669206 wlx200db0c4a76a
gst-launch-1.0 -e udpsrc port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H265" ! rtpjitterbuffer ! queue ! rtph265depay ! h265parse ! mppvideodec ! kmssink

gcc unix_sock.c -o unix_sock   `pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gio-2.0 glib-2.0`


gst-launch-1.0 -e   udpsrc port=5600          caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H265"   ! rtpjitterbuffer latency=250 drop-on-latency=true   ! rtph265depay   ! h265parse   ! mppvideodec   ! kmssink sync=false
