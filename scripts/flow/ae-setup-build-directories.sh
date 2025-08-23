#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# EntoBench Build Directory Setup Script (flow version, portable)
# Creates per-MCU build dirs under --directory (default: ./build)
# and configures CMake with the appropriate toolchains.
# MCUs: g474, h7a3, u575
# =============================================================================

info()  { echo "[INFO] $*"; }
ok()    { echo "[OK]   $*"; }
warn()  { echo "[WARN] $*"; }
err()   { echo "[ERR]  $*" 1>&2; }

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  -h, --help             Show this help
  -f, --force            Force recreation of existing build dirs
  -v, --verbose          Verbose CMake output (adds -Wdev)
  -c, --clean            Remove existing build dirs before configuring
  -d, --directory DIR    Base build directory (default: ./build)

Examples:
  $(basename "$0")
  $(basename "$0") -f
  $(basename "$0") -c -v
  $(basename "$0") -d /tmp/builds
EOF
}

# Defaults
BASE_DIR="./build"
FORCE=false
VERBOSE=false
CLEAN=false

# ----- Portable resolution of this script's dir (no readlink -f, no Python) -----
SCRIPT="$0"
# If not absolute, prepend current working dir
case "$SCRIPT" in
  /*) : ;;
  *)  SCRIPT="$(pwd)/$SCRIPT" ;;
esac
# Strip trailing filename to get directory, then canonify with 'cd -P'
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT")" >/dev/null 2>&1 && pwd -P)"

# ----- Find repo root: walk up until CMakeLists.txt is found -----
REPO_ROOT="$SCRIPT_DIR"
FOUND=""
for _ in 1 2 3 4 5 6 7 8; do
  if [[ -f "$REPO_ROOT/CMakeLists.txt" ]]; then
    FOUND="$REPO_ROOT"
    break
  fi
  REPO_ROOT="$(cd "$REPO_ROOT/.." && pwd -P)"
done
REPO_ROOT="$FOUND"

[[ -n "$REPO_ROOT" ]] || { err "Could not locate repository root from $SCRIPT_DIR"; exit 1; }
[[ -f "$REPO_ROOT/CMakeLists.txt" ]] || { err "CMakeLists.txt not found at $REPO_ROOT"; exit 1; }

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    -f|--force) FORCE=true; shift ;;
    -v|--verbose) VERBOSE=true; shift ;;
    -c|--clean) CLEAN=true; shift ;;
    -d|--directory) BASE_DIR="$2"; shift 2 ;;
    -*) err "Unknown option: $1"; usage; exit 1 ;;
    *)  err "Unexpected argument: $1"; usage; exit 1 ;;
  esac
done

# MCU maps
MCU_NAMES=(g474 h7a3 u575)
TOOLCHAIN_FILES=(stm32-g474re.cmake stm32-h7a3zi.cmake stm32-u575ziq.cmake)
OPENOCD_CFGS=(semihosting_stm32g4.cfg semihosting_stm32h7.cfg semihosting_stm32u5.cfg)

mkdir -p "$BASE_DIR"
info "Repo root: $REPO_ROOT"
info "Base build directory: $BASE_DIR"
info "Force: $FORCE  Clean: $CLEAN  Verbose: $VERBOSE"

for i in "${!MCU_NAMES[@]}"; do
  MCU="${MCU_NAMES[$i]}"
  TOOL="${REPO_ROOT}/stm32-cmake/${TOOLCHAIN_FILES[$i]}"
  OCD="${OPENOCD_CFGS[$i]}"
  BDIR="${BASE_DIR}/build-${MCU}"

  info "Configuring ${MCU} -> ${BDIR}"
  if [[ -d "$BDIR" ]]; then
    if $CLEAN; then
      info "Removing existing: $BDIR"
      rm -rf "$BDIR"
    elif ! $FORCE; then
      warn "Exists: $BDIR (use --force or --clean to recreate). Skipping."
      continue
    else
      info "Recreating: $BDIR"
      rm -rf "$BDIR"
    fi
  fi

  # Configure out-of-source
  CM_OPTS=(
    -S "$REPO_ROOT"
    -B "$BDIR"
    -DCMAKE_TOOLCHAIN_FILE="$TOOL"
    -DFETCH_ST_SOURCES=True
    -DOPENOCD_CFG="$OCD"
    -DLATENCY_MEASUREMENT=1
  )
  $VERBOSE && CM_OPTS+=(-Wdev)

  if cmake "${CM_OPTS[@]}" >/dev/null 2>&1; then
    ok "Configured $MCU at $BDIR"
  else
    err "CMake failed for $MCU (see $BDIR/CMakeCache.txt if present)"
  fi
done

info "Done."
