#!/usr/bin/env bash
set -euo pipefail

# Helper to register aliases for STM32CubeMonitor-Power (CMP) and Saleae Logic 2.
# Usage:
#   ./04-register-cmp-logic2.sh [CMP_DIR] [LOGIC2_APPIMAGE]
#
# Defaults:
#   CMP_DIR="$HOME/workspace/external/STMicroelectronics"
#   LOGIC2_APPIMAGE="$HOME/external/logic2/Logic-*.AppImage"

CMP_DIR_DEFAULT="${HOME}/workspace/external/STMicroelectronics"
LOGIC2_APPIMAGE_DEFAULT="${HOME}/external/logic2/Logic-*.AppImage"

CMP_DIR="${1:-${CMP_DIR_DEFAULT}}"
LOGIC2_APPIMAGE="${2:-${LOGIC2_APPIMAGE_DEFAULT}}"

BASHRC="${HOME}/.bashrc"
added=0

# CMP jar path (common install layout)
CMP_JAR="${CMP_DIR}/STM32CubeMonitor-Power/STM32CubeMonitor-Power.jar"

if [[ -f "$CMP_JAR" ]]; then
  if ! grep -q 'alias cube-monitor-pwr' "${BASHRC}"; then
    printf "alias cube-monitor-pwr='java -jar \"%s\"'\n" "$CMP_JAR" >> "${BASHRC}"
    echo "[*] Registered alias: cube-monitor-pwr -> $CMP_JAR"
    added=1
  else
    echo "[*] Alias already present: cube-monitor-pwr"
  fi
else
  echo "[!] CMP jar not found at: $CMP_JAR"
  echo "    If installed elsewhere, re-run with explicit CMP_DIR:"
  echo "    ./04-register-cmp-logic2.sh /path/to/STMicroelectronics"
fi

# Resolve Logic 2 AppImage (glob OK)
LOGIC2_PATH=$(compgen -G "$LOGIC2_APPIMAGE" || true)
if [[ -n "$LOGIC2_PATH" && -f "$LOGIC2_PATH" ]]; then
  chmod +x "$LOGIC2_PATH" || true
  if ! grep -q 'alias logic2' "${BASHRC}"; then
    printf "alias logic2='%s'\n" "$LOGIC2_PATH" >> "${BASHRC}"
    echo "[*] Registered alias: logic2 -> $LOGIC2_PATH"
    added=1
  else
    echo "[*] Alias already present: logic2"
  fi
else
  echo "[!] Logic 2 AppImage not found matching: $LOGIC2_APPIMAGE"
  echo "    Re-run with explicit path if needed:"
  echo "    ./04-register-cmp-logic2.sh '' /full/path/to/Logic-2.x.y.AppImage"
fi

if [[ $added -eq 1 ]]; then
  echo "[*] Aliases added to ~/.bashrc. Run: source ~/.bashrc"
else
  echo "[*] No changes made to ~/.bashrc"
fi
