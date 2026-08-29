#!/usr/bin/env bash
set -euo pipefail
sudo systemctl set-default graphical.target
echo "Reboot to apply: sudo reboot"
