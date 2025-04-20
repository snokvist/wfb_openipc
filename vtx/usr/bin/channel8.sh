#!/bin/sh

if [ "$1" -lt 1100 ]; then

elif [ "$1" -gt 1099 ] && [ "$1" -lt 1200 ]; then

elif [ "$1" -gt 1199 ] && [ "$1" -lt 1300 ]; then

elif [ "$1" -gt 1299 ] && [ "$1" -lt 1400 ]; then

elif [ "$1" -gt 1399 ] && [ "$1" -lt 1500 ]; then

elif [ "$1" -gt 1499 ] && [ "$1" -lt 1600 ]; then

elif [ "$1" -gt 1599 ] && [ "$1" -lt 1700 ]; then

elif [ "$1" -gt 1699 ] && [ "$1" -lt 1800 ]; then

elif [ "$1" -gt 1799 ] && [ "$1" -lt 1900 ]; then

elif [ "$1" -gt 1899 ]; then

fi

echo "CPU:&C &B temp:&T &L30 &G8 &F30" >/tmp/MSPOSD.msg

exit 1
