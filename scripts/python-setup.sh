#!/usr/bin/env bash
set -euo pipefail

# --- Config ---
ENV_DIR="${HOME}/.venvs/entobench-ae"
PKGS_DEFAULT="numpy pandas scipy matplotlib"
REQ_FILE="${1:-requirements.txt}"

# --- Pick a python ---
pick_python() {
  for exe in python3.12 python3.11 python3; do
    if command -v "$exe" >/dev/null 2>&1; then
      echo "$exe"
      return
    fi
  done
  echo "ERROR: No suitable python found (need python3.12/3.11/3.x)" >&2
  exit 1
}

PY_EXE="$(pick_python)"
echo "[*] Using ${PY_EXE} ($( $PY_EXE --version 2>/dev/null | awk '{print $1,$2}' ))"

# --- Ensure venv module is available ---
if ! "$PY_EXE" -c "import venv" 2>/dev/null; then
  echo "[*] Installing venv support… (sudo may prompt)"
  if command -v apt >/dev/null 2>&1; then
    sudo apt-get update -y
    sudo apt-get install -y python3-venv
  else
    echo "ERROR: python venv module missing and apt not available. Install venv manually." >&2
    exit 1
  fi
fi

# --- Create venv ---
echo "[*] Creating virtual env at: ${ENV_DIR}"
mkdir -p "$(dirname "${ENV_DIR}")"
"$PY_EXE" -m venv "${ENV_DIR}"

# --- Activate & upgrade pip ---
# shellcheck disable=SC1090
source "${ENV_DIR}/bin/activate"
python -m pip install --upgrade pip wheel setuptools

# --- Install deps ---
if [[ -f "${REQ_FILE}" ]]; then
  echo "[*] Found ${REQ_FILE}; installing from it…"
  pip install -r "${REQ_FILE}"
else
  echo "[*] Installing common scientific packages…"
  pip install ${PKGS_DEFAULT}
fi

# --- Verify ---
python - <<'PY'
import sys, importlib
mods = ["numpy","pandas","scipy","matplotlib"]
print("Python:", sys.version.split()[0])
for m in mods:
  try:
    importlib.import_module(m)
    print(f"OK: {m}")
  except Exception as e:
    print(f"FAIL: {m} -> {e}")
PY

echo
echo "✅ Done. To use later:  source ${ENV_DIR}/bin/activate"
