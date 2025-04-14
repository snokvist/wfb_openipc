#!/bin/bash
set -o errexit
set -o pipefail
set -o nounset

export LC_ALL=C

usage() {
  cat <<EOF
Usage: $0 [--channel <channel> --bandwidth <HT20|HT40+>] [--tx-power <value>] [--save] [--get] [--help]

Options:
  --channel   Set the channel (must be passed together with --bandwidth).
  --bandwidth Set the bandwidth (e.g., HT20 or HT40+). Must be passed together with --channel.
  --tx-power  Set TX power (can be passed alone or together with channel/bandwidth).
  --save      Update the configuration file (/etc/pixelpilot_wfb.cfg) with the new values (only for provided arguments) if all settings are applied successfully.
  --get       Print a summary of current settings for all NICs (output: <nic> <channel> <bandwidth> <tx_pwr>).
  --help      Display this help message.

Note:
  If no setting options are provided and --get is passed, the script will only list NIC information without applying any changes.
  In manual mode, if only --tx-power is provided, only TX power is set.
EOF
}

# Initialize variables for command-line options.
cmd_channel=""
cmd_bandwidth=""
cmd_tx_power=""
save_flag=0
get_flag=0

# Parse command-line arguments.
while [[ $# -gt 0 ]]; do
  case "$1" in
    --channel)
      if [[ $# -lt 2 ]]; then
        echo "Error: --channel requires an argument." >&2
        usage
        exit 1
      fi
      cmd_channel="$2"
      shift 2
      ;;
    --bandwidth)
      if [[ $# -lt 2 ]]; then
        echo "Error: --bandwidth requires an argument." >&2
        usage
        exit 1
      fi
      cmd_bandwidth="$2"
      shift 2
      ;;
    --tx-power)
      if [[ $# -lt 2 ]]; then
        echo "Error: --tx-power requires an argument." >&2
        usage
        exit 1
      fi
      cmd_tx_power="$2"
      shift 2
      ;;
    --save)
      save_flag=1
      shift
      ;;
    --get)
      get_flag=1
      shift
      ;;
    --help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

# Determine pure GET mode: if --get is passed without any setting arguments.
if [[ "$get_flag" -eq 1 && -z "$cmd_channel" && -z "$cmd_bandwidth" && -z "$cmd_tx_power" ]]; then
  pure_get=true
else
  pure_get=false
fi

CONFIG_FILE="/etc/pixelpilot_wfb.cfg"
if [ ! -f "$CONFIG_FILE" ]; then
  echo "Config file $CONFIG_FILE not found!" >&2
  exit 1
fi

# Extract settings from the [pixelpilot_wfb] section.
TMP_CONFIG=$(mktemp)
awk 'BEGIN { in_section=0 }
     /^\[pixelpilot_wfb\]/ { in_section=1; next }
     /^\[/ { in_section=0 }
     in_section {
         sub(/#.*/, "")  # Remove inline comments.
         if ($0 ~ /=/)
             print
     }' "$CONFIG_FILE" > "$TMP_CONFIG"
source "$TMP_CONFIG"
rm "$TMP_CONFIG"

# Check that the NIC list is available.
if [[ -z "${WFB_NICS:-}" ]]; then
  echo "Error: WFB_NICS is not set in the configuration file." >&2
  exit 1
fi

# If pure GET mode, list NIC info and exit.
if $pure_get; then
  echo "Pure GET operation: Displaying current NIC settings only."
  echo ""
  echo "Current NIC settings summary:"
  for nic in $WFB_NICS; do
    info=$(iw dev "$nic" info 2>/dev/null || echo "Error reading info")
    if [[ "$info" == "Error reading info" ]]; then
      echo "$nic: unable to retrieve info."
      continue
    fi
    channel_line=$(echo "$info" | grep -E "channel")
    if [[ $channel_line =~ channel[[:space:]]+([0-9]+).*width:[[:space:]]+([0-9]+) ]]; then
      current_channel="${BASH_REMATCH[1]}"
      width="${BASH_REMATCH[2]}"
      if [ "$width" -eq 40 ]; then
        current_bandwidth="HT40+"
      elif [ "$width" -eq 20 ]; then
        current_bandwidth="HT20"
      else
        current_bandwidth="width:${width}MHz"
      fi
    else
      current_channel="unknown"
      current_bandwidth="unknown"
    fi
    tx_line=$(echo "$info" | grep -E "txpower")
    if [[ $tx_line =~ txpower[[:space:]]+([-0-9.]+) ]]; then
      current_tx_power=$(printf "%.0f" "${BASH_REMATCH[1]}")
    else
      current_tx_power="unknown"
    fi
    echo "$nic  $current_channel  $current_bandwidth  $current_tx_power"
  done
  exit 0
fi

# Determine which settings are in manual mode.
manual_chbw=false
manual_tx=false
if [[ -n "$cmd_channel" || -n "$cmd_bandwidth" ]]; then
  # Both must be provided.
  if [[ -z "$cmd_channel" || -z "$cmd_bandwidth" ]]; then
    echo "Error: --channel and --bandwidth must be provided together." >&2
    usage
    exit 1
  fi
  manual_chbw=true
fi
if [[ -n "$cmd_tx_power" ]]; then
  manual_tx=true
fi

# Manual mode: if any setting arguments are given, then use them;
# Otherwise, use values from config file.
if $manual_chbw; then
  channel="$cmd_channel"
  bandwidth="$cmd_bandwidth"
else
  # Only use channel and bandwidth if no manual override was provided.
  # In manual mode with tx-power only, we do NOT set channel/bandwidth.
  channel=""
  bandwidth=""
fi

if $manual_tx; then
  tx_power="$cmd_tx_power"
else
  tx_power="${TX_POWER:-}"
fi

# Apply settings for each NIC.
apply_error=0
for nic in $WFB_NICS; do
  echo "Applying settings on NIC $nic:"
  if $manual_chbw; then
    echo "  Setting channel $channel and bandwidth $bandwidth..."
    if ! output=$(iw dev "$nic" set channel "$channel" "$bandwidth" 2>&1); then
      echo "Error setting channel/bandwidth on $nic:"
      echo "$output"
      apply_error=1
      break
    fi
  fi

  if $manual_tx; then
    echo "  Setting TX power to $tx_power..."
    if ! output=$(iw dev "$nic" set txpower fixed "$tx_power" 2>&1); then
      echo "Error setting TX power on $nic:"
      echo "$output"
      apply_error=1
      break
    fi
  fi

  # In the default (no manual arguments) mode, we apply all settings from the config file.
  if ! $manual_chbw && ! $manual_tx; then
    echo "  Applying channel ${CHANNEL} and bandwidth ${BANDWIDTH} from config..."
    if ! output=$(iw dev "$nic" set channel "$CHANNEL" "$BANDWIDTH" 2>&1); then
      echo "Error setting channel/bandwidth on $nic:"
      echo "$output"
      apply_error=1
      break
    fi
    if [ -n "${TX_POWER:-}" ]; then
      echo "  Applying TX power ${TX_POWER} from config..."
      if ! output=$(iw dev "$nic" set txpower fixed "$TX_POWER" 2>&1); then
        echo "Error setting TX power on $nic:"
        echo "$output"
        apply_error=1
        break
      fi
    fi
  fi
done

if [ "$apply_error" -ne 0 ]; then
  echo "Failed to apply settings. Exiting without updating the configuration file." >&2
  exit 1
fi

echo "Settings applied successfully."

# Save updates if --save is specified.
# Only update the settings that were provided manually.
if [ "$save_flag" -eq 1 ]; then
  echo "Updating configuration file $CONFIG_FILE..."
  if $manual_chbw; then
    sed -i -e "/^\[pixelpilot_wfb\]/,/^\[/ s/^CHANNEL=.*/CHANNEL=$channel/" "$CONFIG_FILE"
    sed -i -e "/^\[pixelpilot_wfb\]/,/^\[/ s/^BANDWIDTH=.*/BANDWIDTH=$bandwidth/" "$CONFIG_FILE"
  fi
  if $manual_tx; then
    sed -i -e "/^\[pixelpilot_wfb\]/,/^\[/ s/^TX_POWER=.*/TX_POWER=$tx_power/" "$CONFIG_FILE"
  fi
  echo "Configuration file updated."
fi

# If --get was also specified, print a summary of current NIC settings.
if [ "$get_flag" -eq 1 ]; then
  echo ""
  echo "Current NIC settings summary:"
  for nic in $WFB_NICS; do
    info=$(iw dev "$nic" info 2>/dev/null || echo "Error reading info")
    if [[ "$info" == "Error reading info" ]]; then
      echo "$nic: unable to retrieve info."
      continue
    fi
    channel_line=$(echo "$info" | grep -E "channel")
    if [[ $channel_line =~ channel[[:space:]]+([0-9]+).*width:[[:space:]]+([0-9]+) ]]; then
      current_channel="${BASH_REMATCH[1]}"
      width="${BASH_REMATCH[2]}"
      if [ "$width" -eq 40 ]; then
        current_bandwidth="HT40+"
      elif [ "$width" -eq 20 ]; then
        current_bandwidth="HT20"
      else
        current_bandwidth="width:${width}MHz"
      fi
    else
      current_channel="unknown"
      current_bandwidth="unknown"
    fi
    tx_line=$(echo "$info" | grep -E "txpower")
    if [[ $tx_line =~ txpower[[:space:]]+([-0-9.]+) ]]; then
      current_tx_power=$(printf "%.0f" "${BASH_REMATCH[1]}")
    else
      current_tx_power="unknown"
    fi
    echo "$nic  $current_channel  $current_bandwidth  $current_tx_power"
  done
fi

exit 0
