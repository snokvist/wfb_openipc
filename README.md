
OpenIPC:
ip route add default via 192.168.1.9 dev eth0

radxa:
sudo iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
sudo iptables -A FORWARD -i end0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i end0 -o wlan0 -j ACCEPT


usb_modeswitch -KW -v 0bda -p 1a2b
