#!/bin/sh

if [ "$1" -ge 950 ]  && [ "$1" -le 1050 ]; then
    # position 1 (around 1000)
    bitrate=6666
    mcs=2
    bw=20
    curl -s 'http://localhost/api/v1/set?video0.bitrate=2888'
    wfb_tx_cmd 8000 set_radio -B $bw -G short -S 1 -L 1 -M $mcs
    sleep 1
    curl -s 'http://localhost/api/v1/set?video0.bitrate=6666'
elif [ "$1" -ge 1150 ] && [ "$1" -le 1250 ]; then
    # position 2 (around 1200)
    bitrate=11333                                                      
    mcs=4
    bw=20
    curl -s 'http://localhost/api/v1/set?video0.bitrate=2800'            
    wfb_tx_cmd 8000 set_radio -B $bw -G short -S 1 -L 1 -M $mcs
    sleep 1
    curl -s 'http://localhost/api/v1/set?video0.bitrate=11333'
elif [ "$1" -ge 1350 ] && [ "$1" -le 1450 ]; then
    # position 3 (around 1400)
    channel=48                                                                                           
    echo "CPU:&C &B temp:&T\nSetting channel: $channel ... please wait ... &L30 &G8 &F30" >/tmp/MSPOSD.msg
    sleep 1.2                                    
    iw dev wlan0 set channel $channel HT20
elif [ "$1" -ge 1550 ] && [ "$1" -le 1650 ]; then
    # position 4 (around 1600)
    channel=104                                                                                           
    echo "CPU:&C &B temp:&T\nSetting channel: $channel ... please wait ... &L30 &G8 &F30" >/tmp/MSPOSD.msg
    sleep 1.2                                    
    iw dev wlan0 set channel $channel HT20
elif [ "$1" -ge 1750 ] && [ "$1" -le 1850 ]; then
    # position 5 (around 1800)
    channel=124
    echo "CPU:&C &B temp:&T\nSetting channel: $channel ... please wait ... &L30 &G8 &F30" >/tmp/MSPOSD.msg
    sleep 1.2 
    iw dev wlan0 set channel $channel HT20
elif [ "$1" -ge 1950 ] && [ "$1" -le 2050 ]; then
    # position 6 (around 2000)
    channel=165
    echo "CPU:&C &B temp:&T\nSetting channel: $channel ... please wait ... &L30 &G8 &F30" >/tmp/MSPOSD.msg
    sleep 1.2
    iw dev wlan0 set channel $channel HT20
fi

sleep 2
echo "CPU:&C &B temp:&T\nChannel: $channel\nAttempted bitrate: $bitrate &L30 &G8 &F30" >/tmp/MSPOSD.msg

exit 1
**
