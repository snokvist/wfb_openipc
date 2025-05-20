/etc/udev/rules.d/10-bonnet.rules

# bonnet0  – MAC 00:22:23:34:45:66 (currently shows up as wlx002223344566)
SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="00:22:23:34:45:66", NAME="bonnet0"

# bonnet1  – MAC 00:12:23:34:45:66 (currently shows up as wlx001223344566)
SUBSYSTEM=="net", ACTION=="add", ATTR{address}=="00:12:23:34:45:66", NAME="bonnet1"
