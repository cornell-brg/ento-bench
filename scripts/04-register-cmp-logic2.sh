#!/usr/bin/env bash
set -euo pipefail

# Helper to make it easy to run CMP and Logic 2 after you’ve downloaded them.
# Pass the paths if you know them; otherwise edit below then re-run.

CMP_DIR_DEFAULT="${HOME}/workspace/external/STMicroelectronics"
LOGIC2_APPIMAGE_DEFAULT="${HOME}/external/logic2/Logic-*.AppImage"

CMP_DIR="${1:-${CMP_DIR_DEFAULT}}"
LOGIC2_APPIMAGE="${2:-${LOGIC2_APPIMAGE_DEFAULT}}"

BASHRC="${HOME}/.bashrc"
added=0

if ! grep -q 'alias cube-monitor-pwr' "${BASHRC}"; then
  echo "alias cube-monitor-pwr='java -jar \"${CMP_DIR}/STM32CubeMonitor-Power/STM32CubeMonitor-Power.jar\"'" >> "${BASHRC}"
  echo "[*] Registered alias: cube-monitor-pwr"
  added=1
fi

if ! grep -q 'alias logic2' "${BASHRC}"; then
  echo "alias logic2='\"${LOGIC2_APPIMAGE}\"'" >> "${BASHRC}"
  echo "[*] Registered alias: logic2"
  added=1
fi

if [[ $added -eq 1 ]]; then
  echo "[*] Aliases added to ~/.bashrc. Run: source ~/.bashrc"
else
  echo "[*] Aliases already present."
fi
