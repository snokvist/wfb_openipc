#!/usr/bin/env bash
set -euo pipefail

# --- user variables ---
IFACE="wlx200db0c4a76a"
NODES=(192.168.2.30 192.168.2.31)
SERVER_IP="192.168.1.20"

# --- previous state ---
last_chan=""
last_bw=""

while true; do
  # 1) extract channel (first numeric field after "channel")
  chan=$(iw dev "$IFACE" info \
    | awk '/channel [0-9]+/ {print $2; exit}')

  # 2) extract width number via regex
  width_val=$(iw dev "$IFACE" info \
    | sed -nE 's/.*width: *([0-9]+) .*/\1/p')

  # 3) map to iw‑friendly string
  if [ "$width_val" -eq 20 ]; then
    bw_str="HT20"
  else
    bw_str="HT40+"
  fi

  # 4) region code
  region=$(iw reg get \
    | awk '/country/ {print $2; exit}' \
    | cut -d: -f1)

  # 5) on change → push out
  if [[ "$chan" != "$last_chan" || "$bw_str" != "$last_bw" ]]; then
    echo "$(date +'%F %T') → Change detected: channel=$chan, bw=$bw_str, region=$region"
    for node in "${NODES[@]}"; do
      if timeout 5 ssh \
           -o BatchMode=yes \
           -o ConnectTimeout=5 \
           -o StrictHostKeyChecking=no \
           root@"$node" \
           "wfb-ng-change.sh $chan $bw_str $region $SERVER_IP"; then
        echo "  → updated $node"
      else
        echo "  !> failed or timed out on $node"
      fi
    done
    last_chan="$chan"
    last_bw="$bw_str"
  fi

  sleep 5
done
