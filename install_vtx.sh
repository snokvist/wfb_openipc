#!/bin/sh

# Ensure the script is run with root privileges
if [ "$(id -u)" -ne 0 ]; then
    echo "This script requires root privileges. Please run with sudo."
    exit 1
fi

# Use the first argument as IP if supplied, otherwise default to 192.168.1.10
IP="${1:-192.168.1.10}"
ssh-keygen -f '/home/radxa/.ssh/known_hosts' -R "$IP"

# Warn the user about replacing /etc/gs.key
echo "WARNING: /etc/gs.key will be replaced with a key using passphrase \"openipc\"."
if [ -f /etc/gs.key ]; then
    echo "A file already exists at /etc/gs.key."
fi

# Ask user if they want to continue (to allow time for backup)
printf "Do you want to continue? (yes/no): "
read answer
if [ "$answer" != "yes" ]; then
    echo "Operation cancelled. Please backup /etc/gs.key if needed."
    exit 1
fi

echo "Copying gs.key with passphrase \"openipc\" to /etc/"
cp gs/gs.key /etc/gs.key

echo "chmod +x on relevant files ..."
chmod -R +x drone/usr/bin*
chmod -R +x drone/etc/init.d/*

echo "Starting scp ..."
SSHPASS="12345" sshpass -e scp -o StrictHostKeyChecking=no -O -v -r -p /etc/gs.key root@"$IP":/etc/drone.key 2>&1 | grep -v debug1
SSHPASS="12345" sshpass -e scp -o StrictHostKeyChecking=no -O -v -r -p drone/* root@"$IP":/ 2>&1 | grep -v debug1

echo "Scp completed ... rebooting ... wait for reconnect..."
SSHPASS="12345" sshpass -e ssh -o StrictHostKeyChecking=no -t root@"$IP" 'reboot' 2>&1 | grep -v debug1

echo "Reconnecting in 25s..."
# Visual countdown using a loop and printf
for i in $(seq 25 -1 1); do
    printf "\r%d seconds remaining..." "$i"
    sleep 1
done
echo ""

SSHPASS="12345" sshpass -e ssh -o StrictHostKeyChecking=no root@"$IP"
