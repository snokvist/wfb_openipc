#!/bin/bash
# This script reads configuration from /etc/pixelpilot_wfb.cfg and starts wfb_rx
# in either local or aggregation mode based on the MODE setting.
#
# Expected configuration in /etc/pixelpilot_wfb.cfg (in the [pixelpilot_wfb] section):
#   MODE=local                # "local" or "aggregation"
#   LOG_INTERVAL=100          # e.g., 100 (ms)
#   WFB_OUTPUT=127.0.0.1:5600   # or "unix" for UNIX sockets
#   WFB_KEY=/etc/gs.key
#   WFB_NICS="wlx200db0c4a76a wlx200db0c4a76b"  # one or more space-separated NIC names

CONFIG_FILE="/etc/pixelpilot_wfb.cfg"

# Verify config file exists
if [ ! -f "$CONFIG_FILE" ]; then
  echo "Config file $CONFIG_FILE not found!"
  exit 1
fi

# Create a temporary file to hold our filtered config variables.
TMP_CONFIG=$(mktemp)

# Extract the key=value lines from the [pixelpilot_wfb] section.
# This awk script:
#   - Activates when it sees the [pixelpilot_wfb] section header.
#   - Stops processing when another section header is encountered.
#   - For lines in the section, it strips inline comments and prints lines that contain "=".
awk 'BEGIN { in_section=0 }
     /^\[pixelpilot_wfb\]/ { in_section=1; next }
     /^\[/ { in_section=0 }
     in_section {
         # Remove any inline comments.
         sub(/#.*/, "")
         # Only output non-empty lines (that have an equals sign).
         if ($0 ~ /=/)
             print
     }' "$CONFIG_FILE" > "$TMP_CONFIG"

# Source the temporary configuration file to import the variables.
source "$TMP_CONFIG"
rm "$TMP_CONFIG"

# At this point we expect variables such as:
#   MODE, LOG_INTERVAL, WFB_OUTPUT, WFB_KEY, and WFB_NICS

# Determine how to set the output option.
if [ "$WFB_OUTPUT" = "unix" ]; then
    # If UNIX sockets are used, set the option accordingly.
    OUTPUT_OPTION="-U rtp_local"
else
    # Otherwise, split the value (e.g., "127.0.0.1:5600") into IP and port.
    IFS=':' read -r output_ip output_port <<< "$WFB_OUTPUT"
    OUTPUT_OPTION="-c $output_ip -u $output_port"
fi

# Function to cleanup background processes on exit.
_cleanup() {
    echo "Termination signal received. Killing all background processes..."
    # Kill all background processes started by this script.
    kill $(jobs -p) 2>/dev/null
    exit 0
}

# Trap SIGINT (Ctrl+C) and SIGTERM to ensure the cleanup function is executed.
trap _cleanup SIGINT SIGTERM

# Check which mode to run
if [ "$MODE" = "local" ]; then
    echo "Starting in LOCAL mode..."
    # For local mode, iterate through each NIC specified in WFB_NICS.
    # (WFB_NICS can contain space-separated NIC names.)
    for nic in $WFB_NICS; do
        echo "Launching wfb_rx on NIC: $nic"
        wfb_rx -p 0 $OUTPUT_OPTION -K "$WFB_KEY" -R 2097152 -l "$LOG_INTERVAL" -i 7669206 "$nic" &
    done
elif [ "$MODE" = "cluster" ]; then
    echo "Starting in AGGREGATION mode..."
    wfb_rx -a 10000 -p 0 $OUTPUT_OPTION -K "$WFB_KEY" -R 2097152 -l "$LOG_INTERVAL" -i 7669206 &
else
    echo "Error: Unknown MODE \"$MODE\" in config."
    exit 1
fi

# Wait for at least one background job to complete before exiting.
wait -n
