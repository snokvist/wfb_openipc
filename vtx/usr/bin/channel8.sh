#!/bin/sh

if [ "$1" -ge 950 ]  && [ "$1" -le 1050 ]; then
    # position 1 (around 1000)
elif [ "$1" -ge 1150 ] && [ "$1" -le 1250 ]; then
    # position 2 (around 1200)
elif [ "$1" -ge 1350 ] && [ "$1" -le 1450 ]; then
    # position 3 (around 1400)
elif [ "$1" -ge 1550 ] && [ "$1" -le 1650 ]; then
    # position 4 (around 1600)
elif [ "$1" -ge 1750 ] && [ "$1" -le 1850 ]; then
    # position 5 (around 1800)
elif [ "$1" -ge 1950 ] && [ "$1" -le 2050 ]; then
    # position 6 (around 2000)
    channel=165
    wifibroadcast cli -s .wireless.channel $channel
    wifibroadcast stop ;wifibroadcast stop; sleep 1;  wifibroadcast start
    #sed -i "s/^wifi_channel =.*/wifi_channel = $channel/" /etc/wifibroadcast.cfg
fi

echo "CPU:&C &B temp:&T &L30 &G8 &F30" >/tmp/MSPOSD.msg

exit 1
**
