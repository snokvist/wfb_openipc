#!/bin/bash
set -emb

export LC_ALL=C

# Default values for wireless configuration
DEFAULT_CHANNEL=161
DEFAULT_BANDWIDTH="HT40+"
DEFAULT_REGION="US"
DEFAULT_SERVER_IP="127.0.0.1"

# Default mode is forwarder;
MODE="forwarder"
CLIENT_IP=""
CLIENT_PORT=""

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
    *)
      # Stop parsing options when encountering non-option argument
      break
      ;;
  esac
done

# Positional parameters for wireless configuration: CHANNEL, BANDWIDTH, REGION, SERVER_IP
if [ $# -eq 4 ]; then
  CHANNEL=$1
  BANDWIDTH=$2
  REGION=$3
  SERVER_IP=$4
else
  CHANNEL=$DEFAULT_CHANNEL
  BANDWIDTH=$DEFAULT_BANDWIDTH
  REGION=$DEFAULT_REGION
  SERVER_IP=$DEFAULT_SERVER_IP
fi

echo "Using channel: $CHANNEL"
echo "Using bandwidth: $BANDWIDTH"
echo "Using region: $REGION"
echo "Using server ip: $SERVER_IP"
echo "Operating mode: $MODE"
if [ "$MODE" != "forwarder" ]; then
  echo "Client IP: $CLIENT_IP"
  echo "Client Port: $CLIENT_PORT"
fi

# Set wireless region
iw reg set "$REGION"

# Read available WLAN interfaces from config file
WFB_NICS=$(grep '^WFB_NICS=' /etc/default/wifibroadcast | cut -d'=' -f2 | tr -d '"')

# Check if any interfaces were found
if [ -z "$WFB_NICS" ]; then
  echo "No WLAN interfaces found in /etc/default/wifibroadcast."
  exit 1
fi

# Convert the string into an array
read -r -a WLAN_INTERFACES <<< "$WFB_NICS"

# Clean up function to kill background jobs on exit
_cleanup()
{
  plist=$(jobs -p)
  if [ -n "$plist" ]; then
      kill -TERM $plist || true
  fi
  exit 1
}
trap _cleanup EXIT

# Initialize each WLAN interface
for wlan in "${WLAN_INTERFACES[@]}"; do
  echo "Initializing $wlan"

  if which nmcli > /dev/null && ! nmcli device show "$wlan" | grep -q '(unmanaged)'; then
    nmcli device set "$wlan" managed no
    sleep 1
  fi

  ip link set "$wlan" down
  iw dev "$wlan" set monitor otherbss
  ip link set "$wlan" up
  iw dev "$wlan" set channel "$CHANNEL" "$BANDWIDTH"
  ip link set dev "$wlan" mtu 3994
  iw dev wlx200db0c4a76a set txpower fixed -100
done

# Execute commands based on mode
case "$MODE" in
  forwarder)
    # Original gs_video and gs_tunnel commands
    wfb_rx -f -c "$SERVER_IP" -u 10000 -p 0 -i 7669206 -R 2097152 "${WLAN_INTERFACES[@]}" &
    wfb_rx -f -c "$SERVER_IP" -u 10001 -p 32 -i 7669206 -R 2097152 "${WLAN_INTERFACES[@]}" &
    wfb_tx -I 11001 -R 2097152 "${WLAN_INTERFACES[@]}" &
    ;;
  local)
    # Check that required parameters are provided
    if [ -z "$CLIENT_IP" ] || [ -z "$CLIENT_PORT" ]; then
      echo "Error: In local mode, --client_ip and --client_port must be provided."
      exit 1
    fi
    # Run the local mode command
    wfb_rx -p 0 -c "$CLIENT_IP" -u "$CLIENT_PORT" -K /etc/gs.key -R 2097152 -l 1000 -i 7669206 "${WLAN_INTERFACES[@]}" &
    ;;
  aggregator)
    # Check that required parameters are provided
    if [ -z "$CLIENT_IP" ] || [ -z "$CLIENT_PORT" ]; then
      echo "Error: In aggregator mode, --client_ip and --client_port must be provided."
      exit 1
    fi
    # Run the aggregator commands
    wfb_rx -a 10000 -p 0 -c "$CLIENT_IP" -u "$CLIENT_PORT" -K /etc/gs.key -R 2097152 -l 1000 -i 7669206 &
    wfb_rx -f -c 127.0.0.1 -u 10000 -p 0 -i 7669206 -R 2097152 "${WLAN_INTERFACES[@]}" &
    ;;
  *)
    echo "Invalid mode: $MODE. Accepted modes: forwarder, aggregator, local."
    exit 1
    ;;
esac

echo "WFB-ng init done"
wait -n
