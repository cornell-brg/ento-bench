#!/usr/bin/env bash
set -euo pipefail

# =============================================================================
# EntoBench Experiment Structure Setup Script
# =============================================================================
# Creates experiments/<project>/<mcu>/<stage>/<algorithm>/ directories.
# Default MCUs: m33 m4 m7
# Run from anywhere.
# =============================================================================

info()  { echo "[INFO] $*"; }
ok()    { echo "[OK]   $*"; }
warn()  { echo "[WARN] $*"; }
err()   { echo "[ERR]  $*" 1>&2; }

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS] [MCU_NAMES...]

Options:
  -h, --help             Show this help
  -d, --directory DIR    Base experiments dir (default: ./experiments)
  -p, --project NAME     Project name under experiments/ (default: ae)
  -f, --force            Overwrite existing MCU dirs
  -v, --verbose          Verbose output

Examples:
  $(basename "$0")
  $(basename "$0") -p iiswc m4 m7
  $(basename "$0") -d /data/experiments -p ae m4
EOF
}

# Defaults
BASE_DIR="./experiments"
PROJECT="ae"
FORCE=false
VERBOSE=false
MCUS=()

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    -d|--directory) BASE_DIR="$2"; shift 2 ;;
    -p|--project) PROJECT="$2"; shift 2 ;;
    -f|--force) FORCE=true; shift ;;
    -v|--verbose) VERBOSE=true; shift ;;
    -*) err "Unknown option: $1"; usage; exit 1 ;;
    *)  MCUS+=("$1"); shift ;;
  esac
done

# Defaults if none provided (removed m0+ and m55)
if [[ ${#MCUS[@]} -eq 0 ]]; then
  MCUS=(m33 m4 m7)
  info "No MCUs provided; using defaults: ${MCUS[*]}"
fi

mkdir -p "${BASE_DIR}/${PROJECT}"

stages=(
  "control:adaptive-controller,geometric-controller,lqr,mpc,tinympc"
  "ekf:robobee-ekf,robofly-ekf,seq-update,sync-update,truncated-update"
  "perception:bbof,fastbrief,iiof,lkof,orb,sift"
  "pose-est:3pt,5pt,8pt,dlt,homography,p3p,up2p"
  "example:example-cache,example-nocache"
)

for mcu in "${MCUS[@]}"; do
  MCU_DIR="${BASE_DIR}/${PROJECT}/${mcu}"
  if [[ -d "${MCU_DIR}" && "${FORCE}" = false ]]; then
    warn "Exists: ${MCU_DIR} (use --force to overwrite). Skipping."
    continue
  fi

  info "Creating ${MCU_DIR}"
  rm -rf "${MCU_DIR}"
  mkdir -p "${MCU_DIR}"

  for s in "${stages[@]}"; do
    stage="${s%%:*}"
    IFS=',' read -r -a algs <<< "${s#*:}"
    STAGE_DIR="${MCU_DIR}/${stage}"
    mkdir -p "${STAGE_DIR}"
    ${VERBOSE} && info "  stage: ${stage}"
    for a in "${algs[@]}"; do
      mkdir -p "${STAGE_DIR}/${a}"
    done
  done

  ok "Experiment structure created for ${mcu}"
done

info "Done."
