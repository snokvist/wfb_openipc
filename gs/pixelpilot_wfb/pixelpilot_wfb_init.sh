#!/bin/bash
set -emb

export LC_ALL=C

# Cleanup function to kill background jobs on exit
_cleanup() {
  plist=$(jobs -p)
  if [ -n "$plist" ]; then
      kill -TERM $plist || true
  fi
  exit 1
}
trap _cleanup EXIT

CONFIG_FILE="/etc/pixelpilot_wfb.cfg"

# Verify config file exists
if [ ! -f "$CONFIG_FILE" ]; then
  echo "Config file $CONFIG_FILE not found!"
  exit 1
fi

# Extract settings from the [pixelpilot_wfb] section of the config file.
TMP_CONFIG=$(mktemp)
awk 'BEGIN { in_section=0 }
     /^\[pixelpilot_wfb\]/ { in_section=1; next }
     /^\[/ { in_section=0 }
     in_section {
         sub(/#.*/, "")          # Remove inline comments.
         if ($0 ~ /=/)
             print
     }' "$CONFIG_FILE" > "$TMP_CONFIG"
source "$TMP_CONFIG"
rm "$TMP_CONFIG"

# Ensure required config variable exists
if [ -z "$WFB_NICS" ]; then
  echo "WFB_NICS is not set in the config file!"
  exit 1
fi

# For aggregation mode and for -I option commands we use only the first NIC.
first_nic=$(echo $WFB_NICS | awk '{print $1}')

# Set regulatory domain using REGION from the config file.
# (Note: Your sample config file provides REGION=00; ensure this is appropriate.)
if [ -n "$REGION" ]; then
    iw reg set "$REGION"
else
    iw reg set US
fi

# === WiFi NIC Initialization ===
# Iterate through each NIC provided in WFB_NICS.
for nic in $WFB_NICS; do
    # If nmcli exists and the NIC is not already unmanaged, set it to unmanaged.
    if which nmcli > /dev/null && nmcli device show "$nic" 2>/dev/null | grep -qv '(unmanaged)'; then
        nmcli device set "$nic" managed no
        sleep 1
    fi
    ip link set "$nic" down
    iw dev "$nic" set monitor otherbss
    ip link set "$nic" up
    iw dev "$nic" set channel "$CHANNEL" "$BANDWIDTH"
    # Set TX power if TX_POWER is nonempty.
    if [ -n "$TX_POWER" ]; then
        iw dev "$nic" set txpower fixed "$TX_POWER"
    fi
done

# === Telemetry Instances ===

# Execute commands conditionally by MODE.
if [ "$MODE" = "cluster" ]; then
    echo "Running in AGGREGATION mode..."

    # Aggregation mode: use only the first NIC.
    # GS Video forwarders (do not change -c and -u options):
    wfb_rx -f -c 127.0.0.1 -u 10000 -p 0 -i 7669206 -R 2097152 "$first_nic" &
    # GS MAVLink:
    wfb_rx -f -c 127.0.0.1 -u 10001 -p 16 -i 7669206 -R 2097152 "$first_nic" &

    # Forwarders for telemetry aggregation;
    # update -K and -l options from config file.
    wfb_tx -d -f data -p 144 -u 14551 -K "$WFB_KEY" -B 20 -G long -S 1 -L 1 -M 1 -k 1 -n 2 -T 0 -F 0 \
           -i 7669206 -R 2097152 -l "$LOG_INTERVAL" -C 8000 127.0.0.1 11001 &
    wfb_tx -d -f data -p 160 -u 5801 -K "$WFB_KEY" -B 20 -G long -S 1 -L 0 -M 1 -k 1 -n 2 -T 0 -F 0 \
           -i 7669206 -R 2097152 -l "$LOG_INTERVAL" -C 8001 127.0.0.1 11002 &
    wfb_rx -a 10001 -p 16 -u 14550 -K "$WFB_KEY" -R 2097152 -l "$LOG_INTERVAL" -i 7669206 &
    wfb_rx -a 10002 -p 32 -u 5800 -K "$WFB_KEY" -R 2097152 -l "$LOG_INTERVAL" -i 7669206 &

    echo "$first_nic"
    # Commands with the -I option (use only the first NIC):
    wfb_tx -I 11001 -R 2097152 "$first_nic" &
    wfb_tx -I 11002 -R 2097152 "$first_nic" &
fi

if [ "$MODE" = "local" ]; then
    echo "Running in LOCAL mode..."

    # Local mode -- MAVLink pair: iterate over all NICs from config.
    for nic in $WFB_NICS; do
        wfb_tx -f data -p 144 -u 14551 -K "$WFB_KEY" -B 20 -G long -S 1 -L 1 -M 1 -k 1 -n 2 -T 0 -F 0 \
               -i 7669206 -R 2097152 -l "$LOG_INTERVAL" -C 8000 "$nic" &
        wfb_rx -p 16 -u 14550 -K "$WFB_KEY" -R 2097152 -l "$LOG_INTERVAL" -i 7669206 "$nic" &
    done

    # Local mode -- Tunnel pair: iterate over all NICs from config.
    for nic in $WFB_NICS; do
        wfb_tx -f data -p 160 -u 5801 -K "$WFB_KEY" -B 20 -G long -S 1 -L 0 -M 1 -k 1 -n 2 -T 0 -F 0 \
               -i 7669206 -R 2097152 -l "$LOG_INTERVAL" -C 8001 "$nic" &
        wfb_rx -p 32 -u 5800 -K "$WFB_KEY" -R 2097152 -l "$LOG_INTERVAL" -i 7669206 "$nic" &
    done
fi

# === Tunnel Block ===
# If TUNNEL is enabled in the config file, execute the tunnel command.
if [ "$TUNNEL" = "enabled" ]; then
    echo "Starting tunnel..."
    wfb_tun -a 10.5.0.10/24 &
fi

echo "WFB-ng init done"
wait -n
