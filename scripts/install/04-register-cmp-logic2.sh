#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# Register aliases to launch:
#   - STM32CubeMonitor-Power (cube-monitor-pwr) using bundled JRE
#   - Saleae Logic 2 (logic2) via a wrapper that picks the newest AppImage
# =============================================================================

CMP_DIR_DEFAULT="${HOME}/external/STMicroelectronics/STM32CubeMonitor-Power"
LOGIC2_DIR_DEFAULT="${HOME}/external/logic2"

CMP_DIR="${1:-${CMP_DIR_DEFAULT}}"
LOGIC2_DIR="${2:-${LOGIC2_DIR_DEFAULT}}"

BASHRC="${HOME}/.bashrc"
BIN_DIR="${HOME}/.local/bin"
LOGIC2_WRAPPER="${BIN_DIR}/logic2-launch"

mkdir -p "${BIN_DIR}"

# --- Write Logic 2 wrapper (avoids alias glob issues; accepts optional dir arg) ---
cat > "${LOGIC2_WRAPPER}" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
search_dir="${1:-$HOME/external/logic2}"

shopt -s nullglob
apps=( "$search_dir"/Logic-*.AppImage "$HOME/Downloads"/Logic-*.AppImage )
shopt -u nullglob

if (( ${#apps[@]} == 0 )); then
  echo "[ERR] Logic 2 AppImage not found in: $search_dir or ~/Downloads" >&2
  echo "      Download from https://support.saleae.com/logic-software/sw-installation" >&2
  exit 1
fi

# pick newest/lexicographically highest
IFS=$'\n' read -r -d '' -a apps_sorted < <(printf '%s\n' "${apps[@]}" | sort -V && printf '\0')
app="${apps_sorted[-1]}"

chmod +x "$app" 2>/dev/null || true

# If FUSE error occurs, user might need: sudo apt-get install -y libfuse2
exec "$app"
EOF
chmod +x "${LOGIC2_WRAPPER}"

# --- Ensure ~/.local/bin is on PATH for interactive shells (idempotent) ---
if ! grep -Fq '~/.local/bin' "${BASHRC}"; then
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> "${BASHRC}"
  echo "[*] Added ~/.local/bin to PATH in ${BASHRC}"
fi

# --- Ensure aliases exist in ~/.bashrc (idempotent) ---

# Escape CMP_DIR safely for alias composition
esc_cmp_dir=$(printf '%q' "${CMP_DIR}")

# cube-monitor-pwr alias (bundled JRE + XRDP-friendly flags)
if ! grep -Fq 'alias cube-monitor-pwr=' "${BASHRC}"; then
  echo "alias cube-monitor-pwr='SWT_GTK3=0 _JAVA_OPTIONS=\"-Dsun.java2d.xrender=false\" ${esc_cmp_dir}/jre/bin/java -jar ${esc_cmp_dir}/STM32CubeMonitor-Power.jar'" >> "${BASHRC}"
  echo "[*] Registered alias: cube-monitor-pwr"
else
  echo "[*] Alias already present: cube-monitor-pwr"
fi

# logic2 alias -> wrapper (so wildcard expands at runtime inside wrapper)
if ! grep -Fq 'alias logic2=' "${BASHRC}"; then
  echo "alias logic2='${LOGIC2_WRAPPER}'" >> "${BASHRC}"
  echo "[*] Registered alias: logic2"
else
  echo "[*] Alias already present: logic2"
fi

echo "[*] Aliases added/verified in ${BASHRC}"
echo "[*] Wrapper installed at ${LOGIC2_WRAPPER}"
echo "[*] Run: source ~/.bashrc"
