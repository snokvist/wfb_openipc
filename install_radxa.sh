#!/usr/bin/bash

apt-get install iptables-persistent
iptables -t nat -A POSTROUTING -s 10.5.0.10 -o eth0 -j MASQUERADE
iptables -A FORWARD -s 10.5.0.10 -o eth0 -j ACCEPT
iptables -A FORWARD -d 10.5.0.10 -i eth0 -j ACCEPT
iptables -t nat -A PREROUTING -p tcp --dport 8080 -d 192.168.1.20 -j DNAT --to-destination 10.5.0.10:80 sudo iptables -t nat -A POSTROUTING -p tcp -d 10.5.0.10 --dport 80 -j MASQUERADE
iptables -t nat -A PREROUTING -p tcp --dport 2222 -d 192.168.1.20 -j DNAT --to-destination 10.5.0.10:20 sudo iptables -t nat -A POSTROUTING -p tcp -d 10.5.0.10 --dport 22 -j MASQUERADE
sysctl -w net.ipv4.ip_forward=1
sysctl net.ipv4.ip_forward
sed -i '/^net.ipv4.ip_forward=/d' /etc/sysctl.conf && echo 'net.ipv4.ip_forward=1' | sudo tee -a /etc/sysctl.conf && sudo sysctl -p
netfilter-persistent save

exit 0
