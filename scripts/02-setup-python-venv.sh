#!/usr/bin/env bash
set -euo pipefail

# Uses python3.12 if available; falls back to python3
ENV_DIR="${HOME}/.venvs/entobench-ae"
REQ_FILE="${1:-requirements.txt}"
PKGS_DEFAULT="numpy pandas scipy matplotlib"

pick_py() {
  for p in python3.12 python3; do
    command -v "$p" >/dev/null 2>&1 && { echo "$p"; return; }
  done
  echo "ERROR: Need python3" >&2; exit 1
}

PY_EXE="$(pick_py)"
echo "[*] Using ${PY_EXE} ($($PY_EXE --version 2>/dev/null | awk '{print $1,$2}'))"

# Ensure venv
$PY_EXE -m venv --help >/dev/null 2>&1 || {
  echo "[*] Installing python3-venv (sudo)…"
  sudo apt-get install -y python3-venv
}

mkdir -p "$(dirname "${ENV_DIR}")"
$PY_EXE -m venv "${ENV_DIR}"

# shellcheck disable=SC1090
source "${ENV_DIR}/bin/activate"
python -m pip install --upgrade pip wheel setuptools

if [[ -f "${REQ_FILE}" ]]; then
  echo "[*] Installing from ${REQ_FILE}…"
  pip install -r "${REQ_FILE}"
else
  echo "[*] Installing common scientific packages…"
  pip install ${PKGS_DEFAULT}
fi

cat <<'PY' | python
import sys, importlib
print("Python:", sys.version.split()[0])
for m in ["numpy","pandas","scipy","matplotlib"]:
  importlib.import_module(m)
  print("OK:", m)
PY

echo
echo "✅ Venv ready at ${ENV_DIR}"
echo "   Activate with: source ${ENV_DIR}/bin/activate"
