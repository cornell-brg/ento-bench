#!/usr/bin/env bash
set -euo pipefail

# Launch STM32CubeMonitor-Power (CMP) and Saleae Logic 2 from an XRDP/XFCE session.
# Tries aliases first, then known paths. Gives actionable hints if launch fails.
#
# Usage: scripts/flow/01-launch-apps.sh [--check-only]
#   --check-only   Only verify we can find the apps; do not launch.

info(){ echo "[INFO] $*"; }
ok(){ echo "[OK]   $*"; }
warn(){ echo "[WARN] $*"; }
err(){ echo "[ERR]  $*" 1>&2; }

CHECK_ONLY=false
if [[ "${1:-}" == "--check-only" ]]; then CHECK_ONLY=true; fi

# Must be in a GUI session
if [[ -z "${DISPLAY:-}" ]]; then
  warn "\$DISPLAY is not set. Run this inside your XRDP/XFCE desktop session (not pure SSH)."
fi

launch_cmd() {
  # runs a command in background unless CHECK_ONLY
  if $CHECK_ONLY; then
    echo "  > $*"
  else
    (setsid "$@" >/dev/null 2>&1 &)
  fi
}

launch_cmp() {
  # allow override: scripts/flow/01-launch-apps.sh --cmp-dir /custom/path
  local cmp_dir="${1:-${HOME}/external/STMicroelectronics/STM32CubeMonitor-Power}"
  local jar="${cmp_dir}/STM32CubeMonitor-Power.jar"
  local java_bin="${cmp_dir}/jre/bin/java"

  # 0) try bundled JRE first (this is what worked on your box)
  if [[ -x "$java_bin" && -f "$jar" ]]; then
    info "STM32CubeMonitor-Power via bundled JRE: $java_bin"
    (setsid env SWT_GTK3=0 _JAVA_OPTIONS="-Dsun.java2d.xrender=false" \
      "$java_bin" -jar "$jar" >/dev/null 2>&1 &) || true
    ok "CMP launch invoked."
    return 0
  fi

  # 1) if user has sourced ~/.bashrc, use the alias
  if command -v cube-monitor-pwr >/dev/null 2>&1; then
    info "STM32CubeMonitor-Power via alias: cube-monitor-pwr"
    (setsid cube-monitor-pwr >/dev/null 2>&1 &) || true
    ok "CMP launch invoked."
    return 0
  fi

  # 2) final hint
  err "CMP not found. Expected jar at: $jar and bundled java at: $java_bin"
  err "Fix: run ./scripts/flow/04-register-cmp-logic2.sh and then 'source ~/.bashrc'"
  return 1
}

find_logic2_appimage() {
  # Common locations for Logic 2 AppImage
  local cands=(
    "${HOME}/external/logic2/Logic-"*.AppImage
    "${HOME}/Downloads/Logic-"*.AppImage
    "${HOME}/Applications/Logic-"*.AppImage
    "/opt/logic2/Logic-"*.AppImage
  )
  for p in "${cands[@]}"; do
    # glob may expand to itself if no match; guard with -e
    [[ -e "$p" ]] || continue
    echo "$p"
    return 0
  done
  return 1
}

launch_logic2() {
  # 1) alias
  if command -v logic2 >/dev/null 2>&1; then
    info "Saleae Logic 2 via alias: logic2"
    launch_cmd logic2 || true
    ok "Logic 2 launch invoked."
    return 0
  fi
  # 2) AppImage search
  local app
  if app="$(find_logic2_appimage)"; then
    chmod +x "$app" || true
    info "Saleae Logic 2 via AppImage: $app"
    if $CHECK_ONLY; then
      echo "  > $app"
      ok "Logic 2 found."
      return 0
    fi
    # Try launch; if it exits immediately, hint about libfuse2
    if ! (setsid "$app" >/dev/null 2>&1 &); then
      err "Failed to start Logic 2."
      warn "If you see 'FUSE: failed to mount', install libfuse2:"
      warn "  sudo apt-get update && sudo apt-get install -y libfuse2"
      return 1
    fi
    ok "Logic 2 launch invoked."
    return 0
  fi
  err "Logic 2 AppImage not found. Place it under ~/external/logic2/ and re-run 04-register-cmp-logic2.sh"
  return 1
}

info "Launching measurement applications..."
CMP_OK=0; L2_OK=0
launch_cmp || CMP_OK=$?
launch_logic2 || L2_OK=$?

if $CHECK_ONLY; then
  if [[ $CMP_OK -eq 0 && $L2_OK -eq 0 ]]; then
    ok "Both applications located (check-only)."
  else
    warn "One or both apps not found; see messages above."
  fi
  exit 0
fi

if [[ $CMP_OK -ne 0 || $L2_OK -ne 0 ]]; then
  warn "One or more apps may not have launched; see messages above."
else
  ok "Both launch commands issued. Switch to the GUIs in your XRDP session."
fi
