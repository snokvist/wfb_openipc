#!/bin/sh

if [ "$1" -ge 950 ]  && [ "$1" -le 1050 ]; then
    # position 1 (around 1000)
    echo "1"
elif [ "$1" -ge 1150 ] && [ "$1" -le 1250 ]; then
    # position 2 (around 1200)
    echo "1"
elif [ "$1" -ge 1350 ] && [ "$1" -le 1450 ]; then
    # position 3 (around 1400)
    channel=40 
    echo "CPU:&C &B temp:&T\nSetting channel: $channel ... please wait ... &L30 &G8 &F30" >/tmp/MSPOSD.msg
    sleep 1.2                 
    iw dev wlan0 set channel $channel
elif [ "$1" -ge 1550 ] && [ "$1" -le 1650 ]; then                                                         
    # position 4 (around 1600)                   
    channel=48                                   
    echo "CPU:&C &B temp:&T\nSetting channel: $channel ... please wait ... &L30 &G8 &F30" >/tmp/MSPOSD.msg
    sleep 1.2                                                                                             
    iw dev wlan0 set channel $channel                                                                     
elif [ "$1" -ge 1750 ] && [ "$1" -le 1850 ]; then                                                         
    # position 5 (around 1800)                   
    channel=140                                  
    echo "CPU:&C &B temp:&T\nSetting channel: $channel ... please wait ... &L30 &G8 &F30" >/tmp/MSPOSD.msg
    sleep 1.2                                                                                             
    iw dev wlan0 set channel $channel                                                                     
elif [ "$1" -ge 1950 ] && [ "$1" -le 2050 ]; then                                                         
    # position 6 (around 2000)       
    channel=161                      
    echo "CPU:&C &B temp:&T\nSetting channel: $channel ... please wait ... &L30 &G8 &F30" >/tmp/MSPOSD.msg
    sleep 1.2                                                             
    iw dev wlan0 set channel $channel                                     
fi                                                                        
                                                                          
      
sleep 2
echo "CPU:&C &B temp:&T\nChannel: $channel &L30 &G8 &F30" >/tmp/MSPOSD.msg
 
exit 1
**
