#!/usr/bin/env bash
set -euo pipefail

# udev rule for STLINK-V3PWR (VID 0x3757)
RULE_FILE="/etc/udev/rules.d/60-stlinkv3pwr.rules"

echo "[*] Writing udev rule (sudo)…"
echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="3757", MODE:="0666"' | sudo tee "${RULE_FILE}" >/dev/null

echo "[*] Reloading udev…"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "[*] Done. Unplug/replug STLINK-V3PWR if it’s connected."
