#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# EntoBench Clean and Rebuild Script (portable, -S/-B)
# - Cleans CMake cache/files in each build dir
# - Reconfigures with the correct toolchain and options
# Options:
#   -h, --help        Show help
#   -v, --verbose     Show CMake output (adds -Wdev)
#   -f, --force       Create build dirs if missing
#   -d, --directory   Base build directory (default: ./build)
#   --compile         Also compile after configure (cmake --build)
# =============================================================================

info()  { echo "[INFO] $*"; }
ok()    { echo "[OK]   $*"; }
warn()  { echo "[WARN] $*"; }
err()   { echo "[ERR]  $*" 1>&2; }

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  -h, --help          Show this help
  -v, --verbose       Verbose CMake output (adds -Wdev)
  -f, --force         Create build dirs if missing
  -d, --directory DIR Base build directory (default: ./build)
      --compile       Build targets after reconfigure (cmake --build)

Examples:
  $(basename "$0")
  $(basename "$0") -v
  $(basename "$0") -f --compile
EOF
}

# Defaults
BASE_DIR="./build"
VERBOSE=false
FORCE=false
DO_BUILD=false

# ----- Portable script dir & repo root (same as your setup script) -----
SCRIPT="$0"
case "$SCRIPT" in /*) : ;; *) SCRIPT="$(pwd)/$SCRIPT" ;; esac
SCRIPT_DIR="$(cd "$(dirname "$SCRIPT")" >/dev/null 2>&1 && pwd -P)"

REPO_ROOT="$SCRIPT_DIR"
FOUND=""
for _ in 1 2 3 4 5 6 7 8; do
  if [[ -f "$REPO_ROOT/CMakeLists.txt" ]]; then FOUND="$REPO_ROOT"; break; fi
  REPO_ROOT="$(cd "$REPO_ROOT/.." && pwd -P)"
done
REPO_ROOT="$FOUND"
[[ -n "$REPO_ROOT" && -f "$REPO_ROOT/CMakeLists.txt" ]] || { err "CMakeLists.txt not found; cannot locate repo root."; exit 1; }

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    -v|--verbose) VERBOSE=true; shift ;;
    -f|--force) FORCE=true; shift ;;
    -d|--directory) BASE_DIR="$2"; shift 2 ;;
    --compile) DO_BUILD=true; shift ;;
    -*) err "Unknown option: $1"; usage; exit 1 ;;
    *)  err "Unexpected argument: $1"; usage; exit 1 ;;
  esac
done

# MCU maps (match your setup script)
MCU_NAMES=(g474 h7a3 u575)
TOOLCHAIN_FILES=(stm32-g474re.cmake stm32-h7a3zi.cmake stm32-u575ziq.cmake)
OPENOCD_CFGS=(semihosting_stm32g4.cfg semihosting_stm32h7.cfg semihosting_stm32u5.cfg)

mkdir -p "$BASE_DIR"
info "Repo root: $REPO_ROOT"
info "Base build directory: $BASE_DIR"
info "Verbose: $VERBOSE  Force: $FORCE  Compile: $DO_BUILD"

successful=()
failed=()

for i in "${!MCU_NAMES[@]}"; do
  MCU="${MCU_NAMES[$i]}"
  TOOL="${REPO_ROOT}/stm32-cmake/${TOOLCHAIN_FILES[$i]}"
  OCD="${OPENOCD_CFGS[$i]}"
  BDIR="${BASE_DIR}/build-${MCU}"

  info "Processing ${MCU} -> ${BDIR}"

  if [[ ! -d "$BDIR" ]]; then
    if $FORCE; then
      info "Creating missing build dir: $BDIR"
      mkdir -p "$BDIR"
    else
      warn "Missing $BDIR (use --force to create). Skipping."
      failed+=("$MCU")
      continue
    fi
  fi

  # Clean CMake cache (leave external toolchain downloads intact)
  rm -f  "${BDIR}/CMakeCache.txt"
  rm -rf "${BDIR}/CMakeFiles" "${BDIR}/cmake_install.cmake" "${BDIR}/Makefile"

  # Configure with -S/-B (never depends on PWD)
  CM_OPTS=(
    -S "$REPO_ROOT"
    -B "$BDIR"
    -DCMAKE_TOOLCHAIN_FILE="$TOOL"
    -DFETCH_ST_SOURCES=True
    -DOPENOCD_CFG="$OCD"
    -DLATENCY_MEASUREMENT=1
  )
  $VERBOSE && CM_OPTS+=(-Wdev)

  if $VERBOSE; then
    cmake "${CM_OPTS[@]}"
  else
    cmake "${CM_OPTS[@]}" >/dev/null 2>&1
  fi

  if [[ $? -ne 0 ]]; then
    err "CMake configure failed for ${MCU} (see ${BDIR}/CMakeCache.txt if present)"
    failed+=("$MCU")
    continue
  fi

  if $DO_BUILD; then
    info "Building ${MCU}..."
    if $VERBOSE; then
      cmake --build "$BDIR" -- -j
    else
      cmake --build "$BDIR" -- -j >/dev/null 2>&1
    fi
    [[ $? -ne 0 ]] && { err "Build failed for ${MCU}"; failed+=("$MCU"); continue; }
  fi

  ok "Reconfigured${DO_BUILD:+ and built} ${MCU}"
  successful+=("$MCU")
done

info ""
info "Clean & rebuild summary:"
if ((${#successful[@]})); then
  ok "Success (${#successful[@]}): ${successful[*]}"
fi
if ((${#failed[@]})); then
  warn "Failed  (${#failed[@]}): ${failed[*]}"
  exit 1
fi
