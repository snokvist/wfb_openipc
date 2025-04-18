#!/usr/bin/env bash
set -e

# must be root
if [ "$EUID" -ne 0 ]; then
  echo "This script must be run as root." >&2
  exit 1
fi

# expect exactly 4 args
if [ "$#" -ne 4 ]; then
  echo "Usage: $0 <channel> <bandwidth> <region> <server_ip>" >&2
  exit 1
fi

CHANNEL=$1
BANDWIDTH=$2
REGION=$3
SERVER_IP=$4

SCRIPT_PATH="/usr/sbin/wfb-ng.sh"

echo "Updating defaults in $SCRIPT_PATH:"
echo "  DEFAULT_CHANNEL=$CHANNEL"
echo "  DEFAULT_BANDWIDTH=\"$BANDWIDTH\""
echo "  DEFAULT_REGION=\"$REGION\""
echo "  DEFAULT_SERVER_IP=\"$SERVER_IP\""

# apply the changes
sed -i \
  -e "s/^DEFAULT_CHANNEL=.*/DEFAULT_CHANNEL=${CHANNEL}/" \
  -e "s|^DEFAULT_BANDWIDTH=.*|DEFAULT_BANDWIDTH=\"${BANDWIDTH}\"|" \
  -e "s|^DEFAULT_REGION=.*|DEFAULT_REGION=\"${REGION}\"|" \
  -e "s|^DEFAULT_SERVER_IP=.*|DEFAULT_SERVER_IP=\"${SERVER_IP}\"|" \
  "$SCRIPT_PATH"

echo "Restarting wfb-ng service..."

# Try systemctl first
if command -v systemctl &>/dev/null; then
  if systemctl restart wfb-cluster-node; then
    echo "Service restarted via systemctl."
    exit 0
  else
    echo "systemctl restart failed, falling back to init.d…" >&2
  fi
fi

# Fallback to init.d
if [ -x /etc/init.d/wfb-ng ]; then
  if /etc/init.d/wfb-ng restart; then
    echo "Service restarted via init.d."
    exit 0
  else
    echo "Failed to restart via init.d." >&2
    exit 1
  fi
fi

echo "No suitable restart method found; please restart 'wfb-cluster-node' manually." >&2
exit 1
