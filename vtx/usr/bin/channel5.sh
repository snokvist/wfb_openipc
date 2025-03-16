#!/bin/sh

echo "Running logic on msposd channel 5"

# Retrieve the telemetry setting from the YAML file
tx_on_arm=$(yaml-cli -i /etc/wfb.yaml -g .telemetry.tx_on_arm)
# Only run the TX ON ARM code if the setting is "enabled"
if [ "$tx_on_arm" = "enabled" ]; then
    ##TX ON ARM BEGIN
    if [ "$1" -lt 1200 ]; then
        echo "Disarm detected, setting low power"
        set_live_tx_pwr.sh 1
    elif [ "$1" -gt 1800 ]; then
        echo "ARM detected, setting high power"
        set_live_tx_pwr.sh 9
    fi
    ##TX ON ARM END
fi

exit 0
