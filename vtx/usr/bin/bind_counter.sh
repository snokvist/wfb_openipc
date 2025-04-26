#!/bin/sh
#
# startup-guard.sh – run “bind” setup after 3 reboots inside 30 s,
#                    but NEVER terminate on errors.
#-----------------------------------------------------------------------------

set +e                     # absolutely never exit on error
trap '' ERR                # ignore uncaught errors

COUNTER_FILE=/etc/restart_counter
GRACE=30                   # grace period (seconds)

#----------------------------------------------------------------------------
# 1.  Read counter (anything non-numeric → 0)
#----------------------------------------------------------------------------
if [ -f "$COUNTER_FILE" ]; then
    count=$(cat "$COUNTER_FILE" 2>/dev/null) || count=0
    case "$count" in ''|*[!0-9]*) count=0 ;; esac
else
    count=0
fi

#----------------------------------------------------------------------------
# 2.  Increment & store
#----------------------------------------------------------------------------
count=$((count + 1))
echo "$count" > "$COUNTER_FILE" 2>/dev/null || true

#----------------------------------------------------------------------------
# 3a.  Not yet 3?  Hold the value for 30 s, then clear and leave
#----------------------------------------------------------------------------
if [ "$count" -lt 3 ]; then
    echo "Counter currently at: $count"
    sleep "$GRACE" || true
    echo 0 > "$COUNTER_FILE" 2>/dev/null || true
    exit 0
fi

#----------------------------------------------------------------------------
# 3b.  Third fast boot – run payload right now, then clear counter
#----------------------------------------------------------------------------
echo 0 > "$COUNTER_FILE" 2>/dev/null || true

##############################################################################
# PAYLOAD (WiFi-bind setup) – every step protected with “|| true”
##############################################################################

# Ensure we have a bind key
[ -f /etc/bind.key ] || \
  echo "OoLVgEYyFofg9zdhfYPks8/L8fqWaF9Jk8aEnynFPsXNqhSpRCMbVKBFP4fCEOv5DGcbXmUHV5eSykAbFB70ew==" \
    | base64 -d 2>/dev/null > /etc/bind.key || true

# Re-configure the radio
wifibroadcast stop                       || true
wifibroadcast stop                       || true
sleep 1
iw dev wlan0 set channel 165 HT20        || true
iw dev wlan0 set txpower fixed 100       || true

sleep 2 || true

# Start the bind tunnel
wfb_rx -K /etc/bind.key -i 10531917 -p 255 -u 5800 wlan0           >/dev/null 2>&1 &
wfb_tx -K /etc/bind.key -M 1 -B 20 -k 1 -n 2 -S 0 -L 0 \
       -i 10531917 -p 127 -u 5801 wlan0                            >/dev/null 2>&1 &
wfb_tun -t drone-bind -a 10.5.99.10/24                             >/dev/null 2>&1 &

echo "Waiting for binding request..."

exit 0
