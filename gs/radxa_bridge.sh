nmcli con add type bridge ifname br0 \
       ipv4.addresses 192.168.2.20/24 \
       ipv4.gateway 192.168.2.1 \
       ipv4.dns 192.168.2.1 \
       ipv4.method manual autoconnect yes

nmcli con add type bridge-slave ifname eth0 master br0 autoconnect yes
