#!/bin/bash
#
# This script sets up the wifibroadcast service for binding a drone.
# It stops any conflicting background services, ensures that the required
# bind key exists, starts the wifibroadcast service in bind mode, and then
# manages the connect.py process that performs the bind operation.
#
# Usage:
#   sudo ./script.sh <operation> [folder_path]
#
# Positional arguments:
#   <operation>   Operation to perform. Currently, only "bind" is supported.
#   [folder_path] Required for the "bind" operation. Specifies the folder containing bind data.
#
# Example:
#   sudo ./script.sh bind /path/to/bind_data_folder

########################################
# Display usage information and exit
########################################
usage() {
    echo "Usage:"
    echo "  $0 <operation> [folder_path]"
    echo
    echo "Positional arguments:"
    echo "  <operation>   Must be 'bind' (currently, only this operation is supported)"
    echo "  [folder_path] Required for 'bind', specifying the folder containing bind data."
    echo
    echo "Example:"
    echo "  sudo $0 bind /path/to/bind_data_folder"
    exit 1
}

########################################
# Ensure the script is run as root
########################################
if [[ "$EUID" -ne 0 ]]; then
    echo "Error: This script must be run with root privileges (try: sudo $0 ...)"
    exit 1
fi

########################################
# Global variable to hold connect.py PID
########################################
CONNECT_PID=""

########################################
# Cleanup function to stop background processes on script exit
########################################
cleanup() {
    echo "Cleaning up background processes..."
    # If connect.py is still running, gracefully terminate it.
    if [[ -n "$CONNECT_PID" ]] && ps -p "$CONNECT_PID" &>/dev/null; then
        echo "Stopping connect.py (PID: $CONNECT_PID)..."
        kill -TERM "$CONNECT_PID"
        wait "$CONNECT_PID"
    fi
    systemctl stop wifibroadcast@gs_bind
    systemctl start wifibroadcast@gs
}
trap cleanup EXIT

########################################
# Validate input arguments
########################################
if [[ $# -lt 1 ]]; then
    echo "Error: Not enough arguments provided."
    usage
fi

# Assign input arguments
operation="$1"
optional_param="$2"

########################################
# Stop any conflicting background services
########################################
systemctl stop wfb-cluster-node
systemctl stop wfb-cluster-manager
systemctl stop wifibroadcast@gs
systemctl stop wifibroadcast@gs_bind

########################################
# Ensure the bind key file exists; create it with a default key if missing
########################################
if ! [ -f /etc/bind.key ]; then
    echo "Creating default bind key at /etc/bind.key..."
    echo "OoLVgEYyFofg9zdhfYPks8/L8fqWaF9Jk8aEnynFPsXNqhSpRCMbVKBFP4fCEOv5DGcbXmUHV5eSykAbFB70ew==" | base64 -d > /etc/bind.key
fi

########################################
# Start the wifibroadcast service in bind mode
########################################
echo "Starting wifibroadcast@gs_bind service..."
systemctl start wifibroadcast@gs_bind
sleep 3      # Allow the service time to initialize

########################################
# Execute the requested operation via connect.py
########################################
case "$operation" in
    bind)
        # The 'bind' operation requires a folder path containing bind data.
        if [[ -z "$optional_param" ]]; then
            echo "Error: For 'bind', a folder path must be provided."
            usage
        fi
        echo "Starting connect.py for bind operation with data folder: $optional_param"
        ./connect.py --ip 10.5.99.2 --bind "$optional_param" &
        CONNECT_PID=$!
        wait "$CONNECT_PID"
        ;;
    *)
        echo "Error: Invalid operation '$operation'. Only 'bind' is supported."
        usage
        ;;
esac

exit 0
