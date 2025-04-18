#!/usr/bin/env bash
set -e

export LC_ALL=C

# Default wireless settings
DEFAULT_CHANNEL=165
DEFAULT_BANDWIDTH="HT20"
DEFAULT_REGION="US"
DEFAULT_SERVER_IP="192.168.1.20"

# Positional args: CHANNEL BANDWIDTH REGION SERVER_IP
if [ $# -eq 4 ]; then
  CHANNEL=$1; BANDWIDTH=$2; REGION=$3; SERVER_IP=$4
else
  CHANNEL=$DEFAULT_CHANNEL
  BANDWIDTH=$DEFAULT_BANDWIDTH
  REGION=$DEFAULT_REGION
  SERVER_IP=$DEFAULT_SERVER_IP
fi

echo "Channel:   $CHANNEL"
echo "Bandwidth: $BANDWIDTH"
echo "Region:    $REGION"
echo "Server:    $SERVER_IP"

# Apply wireless region
iw reg set "$REGION"

# Load interfaces from config
WFB_NICS=$(grep '^WFB_NICS=' /etc/default/wifibroadcast \
           | cut -d'=' -f2 | tr -d '"')
if [ -z "$WFB_NICS" ]; then
  echo "ERROR: no WFB_NICS found" >&2
  exit 1
fi
read -r -a WLAN_INTERFACES <<< "$WFB_NICS"

# Cleanup on exit
_cleanup(){
  jobs -p | xargs --no-run-if-empty kill -TERM
  exit 1
}
trap _cleanup EXIT

# Init each interface
for wlan in "${WLAN_INTERFACES[@]}"; do
  echo "Initializing $wlan"
  if command -v nmcli &>/dev/null && \
     ! nmcli device show "$wlan" | grep -q '(unmanaged)'; then
    nmcli device set "$wlan" managed no
    sleep 1
  fi

  ip link set "$wlan" down
  iw dev "$wlan" set monitor otherbss
  ip link set "$wlan" up
  iw dev "$wlan" set channel "$CHANNEL" "$BANDWIDTH"
done

# --- gs_video ---
wfb_rx -f -c "$SERVER_IP" -u 10000 -p 0  -i 7669206 -R 2097152 "${WLAN_INTERFACES[@]}" &

# --- gs_mavlink ---
wfb_rx -f -c "$SERVER_IP" -u 10001 -p 16 -i 7669206 -R 2097152 "${WLAN_INTERFACES[@]}" &
wfb_tx          -I 11001          -R 2097152 "${WLAN_INTERFACES[@]}" &

# --- gs_tunnel ---
wfb_rx -f -c "$SERVER_IP" -u 10002 -p 32 -i 7669206 -R 2097152 "${WLAN_INTERFACES[@]}" &
wfb_tx          -I 11002          -R 2097152 "${WLAN_INTERFACES[@]}" &

echo "All forwarder instances started"
wait -n
