#!/bin/sh

if [ "$1" -ge 950 ]  && [ "$1" -le 1050 ]; then
    tx_manager.sh set_tx_power 0
elif [ "$1" -ge 1150 ] && [ "$1" -le 1250 ]; then
    tx_manager.sh set_tx_power 0
elif [ "$1" -ge 1350 ] && [ "$1" -le 1450 ]; then
    tx_manager.sh set_tx_power 0
elif [ "$1" -ge 1550 ] && [ "$1" -le 1650 ]; then
    tx_manager.sh set_tx_power 0
elif [ "$1" -ge 1750 ] && [ "$1" -le 1850 ]; then
    tx_manager.sh set_tx_power 0
elif [ "$1" -ge 1950 ] && [ "$1" -le 2050 ]; then
    tx_manager.sh set_tx_power 0
fi

sleep 2
echo "CPU:&C &B temp:&T\nChannel: $channel\nAttempted bitrate: $bitrate &L30 &G8 &F30" >/tmp/MSPOSD.msg

exit 1
**
