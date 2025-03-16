#!/bin/sh

# Configuration parameters (adjust these as needed)
INPUT_MIN=1000
INPUT_MAX=2000
OUTPUT_MIN=2800
OUTPUT_MAX=18000

# Input value from argument $1 (expected range: 950-2050)
input_value=$1

# Perform the linear conversion:
new_value=$(( ( (input_value - INPUT_MIN) * (OUTPUT_MAX - OUTPUT_MIN) ) / (INPUT_MAX - INPUT_MIN) + OUTPUT_MIN ))
echo "Channel12 conversion: Input $input_value -> New value $new_value (Output range: $OUTPUT_MIN-$OUTPUT_MAX)"

echo "Running logic on msposd channel 12"
set_live_bitrate.sh "$new_value"
set_live_tx_pwr.sh 10

exit 0
