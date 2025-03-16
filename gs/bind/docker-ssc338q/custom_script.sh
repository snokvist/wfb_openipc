#!/bin/sh

echo "Put custom commands to be executed in this script."

# Capture the output of the command.
output=$(wfb-cli -g .common.passphrase)

# Check if the output is non-empty.
if [ -n "$output" ]; then
  echo "Passphrase retrieved: $output"
  # Run the additional command here.
  keygen $output
else
  echo "No passphrase retrieved, please check the command."
  #cp etc/gs.key /etc/drone.key
fi


if [ -f etc/rc.local ]; then
  echo "rc.local detected, copy to /etc/rc.local."
  cp etc/rc.local /etc/rc.local
fi


exit 0
