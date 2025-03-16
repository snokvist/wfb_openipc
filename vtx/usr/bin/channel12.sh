#!/bin/bash
# Live bitrate setter with near max TX power.
# Convert incoming argument $1 (range: 950-2050) to a new value in the range 2800-18000.
# The linear conversion formula is:
#   new_value = ((input - 950) * (18000 - 2800)) / (2050 - 950) + 2800
# Simplified, that becomes:
#   new_value = ((input - 950) * 15200) / 1100 + 2800

new_value=$(( (($1 - 950) * 15200) / 1100 + 2800 ))
echo "Channel12 conversion: Input $1 -> New value $new_value"

echo "Running logic on msposd channel 12"
set_live_bitrate.sh "$new_value"
set_live_tx_pwr.sh 10

exit 0
