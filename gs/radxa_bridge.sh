#!/usr/bin/env bash
# Quickly (re)create bridge br0 with fast-recovery settings
set -euo pipefail

### —— adjustable bits ————————————————
IP4_ADDR="192.168.2.20/24"
GATEWAY4="192.168.2.30"
DNS4="192.168.2.30"
BR="br0"
SLAVE="eth0"
CON_BR="bridge-${BR}"
CON_SLAVE="${BR}-${SLAVE}"
### ————————————————————————————————---------

echo "==> Removing old bridge profiles (if any)…"
nmcli -g NAME connection show | grep -E "^${CON_BR}(-[0-9]+)?$|^${CON_SLAVE}(-[0-9]+)?$" \
  || true | while read -r id; do
    echo "   deleting $id"
    nmcli connection delete "$id"
done

echo "==> Creating ${CON_BR}…"
nmcli connection add type bridge ifname "$BR" con-name "$CON_BR" \
      ipv4.addresses "$IP4_ADDR" ipv4.gateway "$GATEWAY4" ipv4.dns "$DNS4" \
      ipv4.method manual autoconnect yes \
      bridge.stp no bridge.forward-delay 0   # fast, loop-free on a single host :contentReference[oaicite:0]{index=0}

echo "==> Adding ${CON_SLAVE}…"
nmcli connection add type bridge-slave ifname "$SLAVE" con-name "$CON_SLAVE" \
      master "$BR" autoconnect yes

echo "==> Extra NetworkManager tweaks…"
nmcli connection modify "$CON_BR" connection.wait-device-timeout 5

echo "==> Sysctl tunables (ARP notify, fast DAD)…"
cat >/etc/sysctl.d/99-${BR}.conf <<EOF
net.ipv4.conf.${BR}.arp_notify = 1
net.ipv6.conf.all.dad_transmits = 1
EOF
sysctl -p /etc/sysctl.d/99-${BR}.conf >/dev/null

echo "==> Bringing the bridge up…"
nmcli connection up "$CON_BR"
nmcli connection up "$CON_SLAVE"
echo "All done – link will be usable ~1 s after the cable is in."
