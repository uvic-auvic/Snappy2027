#!/usr/bin/env bash
set -euo pipefail
sudo systemctl set-default multi-user.target
echo "Reboot to apply: sudo reboot"
