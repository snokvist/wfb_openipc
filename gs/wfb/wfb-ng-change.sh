#!/bin/bash
set -e

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "This script must be run as root."
  exit 1
fi

# Initialize optional variables with defaults
MODE="forwarder"
CLIENT_IP=""
CLIENT_PORT=""
INIT_METHOD="systemctl"

# Parse optional arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --client_ip)
      CLIENT_IP="$2"
      shift 2
      ;;
    --client_port)
      CLIENT_PORT="$2"
      shift 2
      ;;
    --init)
      INIT_METHOD="$2"
      shift 2
      ;;
    *)
      break
      ;;
  esac
done

# Now there should be either 3 or 4 positional parameters: <channel> <bandwidth> <region> [<server_ip>]
if [ "$#" -lt 3 ] || [ "$#" -gt 4 ]; then
  echo "Usage: $0 [--mode <mode>] [--client_ip <client_ip>] [--client_port <client_port>] [--init <init>] <channel> <bandwidth> <region> [<server_ip>]"
  exit 1
fi

CHANNEL=$1
BANDWIDTH=$2
REGION=$3

if [ "$#" -eq 4 ]; then
  SERVER_IP=$4
else
  SERVER_IP=""
fi

echo "Updating default values in /usr/sbin/wfb-ng.sh to:"
echo "  DEFAULT_CHANNEL=${CHANNEL}"
echo "  DEFAULT_BANDWIDTH=${BANDWIDTH}"
echo "  DEFAULT_REGION=${REGION}"
if [ -n "$SERVER_IP" ]; then
  echo "  DEFAULT_SERVER_IP=${SERVER_IP}"
fi
echo "  MODE=${MODE}"
echo "  CLIENT_IP=${CLIENT_IP}"
echo "  CLIENT_PORT=${CLIENT_PORT}"
echo "  INIT_METHOD=${INIT_METHOD}"

# Update the default values in /usr/sbin/wfb-ng.sh
sed -i "s/^DEFAULT_CHANNEL=.*/DEFAULT_CHANNEL=${CHANNEL}/" /usr/sbin/wfb-ng.sh
sed -i "s/^DEFAULT_BANDWIDTH=.*/DEFAULT_BANDWIDTH=\"${BANDWIDTH}\"/" /usr/sbin/wfb-ng.sh
sed -i "s/^DEFAULT_REGION=.*/DEFAULT_REGION=\"${REGION}\"/" /usr/sbin/wfb-ng.sh

# Only update DEFAULT_SERVER_IP if a new one was provided
if [ -n "$SERVER_IP" ]; then
  sed -i "s/^DEFAULT_SERVER_IP=.*/DEFAULT_SERVER_IP=\"${SERVER_IP}\"/" /usr/sbin/wfb-ng.sh
fi

sed -i "s/^MODE=.*/MODE=\"${MODE}\"/" /usr/sbin/wfb-ng.sh
sed -i "s/^CLIENT_IP=.*/CLIENT_IP=\"${CLIENT_IP}\"/" /usr/sbin/wfb-ng.sh
sed -i "s/^CLIENT_PORT=.*/CLIENT_PORT=\"${CLIENT_PORT}\"/" /usr/sbin/wfb-ng.sh
sed -i "s/^INIT_METHOD=.*/INIT_METHOD=\"${INIT_METHOD}\"/" /usr/sbin/wfb-ng.sh

echo "Defaults updated successfully."

# Restart the wfb-cluster-node service based on the INIT_METHOD
echo "Restarting wfb-cluster-node service..."
if [ "$INIT_METHOD" = "init.d" ]; then
  if ! /etc/init.d/wfb-ng restart; then
    echo "Failed to restart wfb-cluster-node service."
    exit 1
  fi
else
  if ! systemctl restart wfb-cluster-node; then
    echo "Failed to restart wfb-cluster-node service."
    exit 1
  fi
fi

echo "Service restarted successfully."
exit 0
