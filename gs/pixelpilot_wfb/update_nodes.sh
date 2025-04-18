#!/usr/bin/env bash
#
# watch-wfb.sh
#

config_file="/etc/pixelpilot_wfb.cfg"

# --- parse only the lines we need ---
nodes_str=""
nics_str=""
server=""

while IFS= read -r line; do
    # strip comments and whitespace
    line="${line%%#*}"
    line="${line//[$'\t\r\n ']}"
    [[ -z "$line" || "$line" =~ ^\[[[:alnum:]_]+\]$ ]] && continue

    key="${line%%=*}"
    value="${line#*=}"

    case "$key" in
        NODES)    nodes_str="${value//\"/}"    ;;  # strip quotes
        WFB_NICS) nics_str="${value//\"/}"      ;;
        SERVER)   server="$value"               ;;
    esac
done < "$config_file"

# build arrays from space‑separated strings
read -ra nodes <<< "$nodes_str"
read -ra nics  <<< "$nics_str"

if (( ${#nics[@]} == 0 )); then
    echo "ERROR: no WFB_NICS found in $config_file" >&2
    exit 1
fi

iface="${nics[0]}"    # first adapter
last_chan=""
last_bw=""

while true; do
    # get channel, width
    info=$(iw dev "$iface" info 2>/dev/null)
    chan=$(awk '/channel/ {print $2; exit}' <<< "$info")
    width=$(awk '/width:/  {print $2; exit}' <<< "$info")

    # map to HT20/HT40+
    if [[ "$width" == "20" ]]; then
        bw_str="HT20"
    else
        bw_str="HT40+"
    fi

    # get region code (e.g. "00")
    region=$(iw reg get 2>/dev/null \
             | awk '/country/ {print $2; exit}' \
             | cut -d: -f1)

    # if either changed, push update
    if [[ "$chan" != "$last_chan" || "$bw_str" != "$last_bw" ]]; then
        for node in "${nodes[@]}"; do
            sshpass -p 12345 ssh -o StrictHostKeyChecking=no \
                root@"$node" \
                "wfb-ng-change.sh $chan $bw_str $region $server"
        done
        last_chan="$chan"
        last_bw="$bw_str"
    fi

    sleep 3
done
