#!/usr/bin/env bash
set -euo pipefail

# Wrapper to set up build directories and experiment structure.
# Usage examples:
#   scripts/flow/00-setup-experiment-env.sh
#   scripts/flow/00-setup-experiment-env.sh --force --verbose
#   scripts/flow/00-setup-experiment-env.sh --build-dir ./build --exp-dir ./experiments --project ae

# Resolve this script's directory and the repo root (two levels up from scripts/flow)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../" && pwd)"

echo "Repo root: $REPO_ROOT"

# Correct locations of the helper scripts (now in scripts/flow)
BUILD_SCRIPT="${REPO_ROOT}/scripts/flow/ae-setup-build-directories.sh"
EXPS_SCRIPT="${REPO_ROOT}/scripts/flow/ae-setup-experiment-folder.sh"

# Sanity checks
[[ -f "${REPO_ROOT}/CMakeLists.txt" ]] || { echo "[ERR] CMakeLists.txt not found at ${REPO_ROOT}"; exit 1; }
[[ -x "${BUILD_SCRIPT}" ]] || { echo "[ERR] Build script missing or not executable: ${BUILD_SCRIPT}"; exit 1; }
[[ -x "${EXPS_SCRIPT}" ]] || { echo "[ERR] Experiment script missing or not executable: ${EXPS_SCRIPT}"; exit 1; }

# Defaults
BUILD_DIR="${REPO_ROOT}/build"
EXP_DIR="${REPO_ROOT}/experiments"
PROJECT="ae"
FORCE=""
VERBOSE=""
CLEAN=""

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  --build-dir <dir>    Build root (default: ./build)
  --exp-dir <dir>      Experiments root (default: ./experiments)
  --project <name>     Project name under experiments/ (default: ae)
  --force              Force recreate (passes through to both scripts)
  --verbose            Verbose output (passes to build script)
  --clean              Clean builds before setup (build script only)
  -h, --help           Show this help
EOF
  exit 1
}

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-dir) BUILD_DIR="$2"; shift 2 ;;
    --exp-dir)   EXP_DIR="$2"; shift 2 ;;
    --project)   PROJECT="$2"; shift 2 ;;
    --force)     FORCE="--force"; shift ;;
    --verbose)   VERBOSE="--verbose"; shift ;;
    --clean)     CLEAN="--clean"; shift ;;
    -h|--help)   usage ;;
    *) echo "Unknown option: $1" >&2; usage ;;
  esac
done

# 1) Build directories
echo "[INFO] Configuring build directories at: ${BUILD_DIR}"
"${BUILD_SCRIPT}" ${FORCE:+$FORCE} ${VERBOSE:+$VERBOSE} ${CLEAN:+$CLEAN} \
  --directory "${BUILD_DIR}"

# 2) Experiment structure (defaults to MCUs defined in that script)
echo "[INFO] Creating experiment structure at: ${EXP_DIR} (project: ${PROJECT})"
"${EXPS_SCRIPT}" ${FORCE:+$FORCE} \
  --directory "${EXP_DIR}" \
  --project "${PROJECT}"

echo "[INFO] Setup complete."
echo "[INFO] Build dirs: ${BUILD_DIR}/build-g474, ${BUILD_DIR}/build-h7a3, ${BUILD_DIR}/build-u575"
echo "[INFO] Experiments: ${EXP_DIR}/${PROJECT}/m4/example/{example-cache,example-nocache}/"
