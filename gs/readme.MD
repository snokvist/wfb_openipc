sudo systemctl edit wfb-cluster-node.service

Inside the editor insert only what you need, e.g. to disable the start-limit and make the unit always restart:

# /etc/systemd/system/wfb-cluster-node.service.d/override.conf
[Unit]
StartLimitIntervalSec=0
StartLimitBurst=0

[Service]
Restart=always
RestartSec=5s     # keep your original delay
Save & exit.

Then reload units and reset the failure counter (so the new rules take effect immediately):

sudo systemctl daemon-reload
sudo systemctl reset-failed wfb-cluster-node.service
sudo systemctl start wfb-cluster-node.service
