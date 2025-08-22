#!/usr/bin/env bash
set -euo pipefail

echo "=== Verifying toolchain ==="
if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  arm-none-eabi-gcc --version | head -n1
else
  echo "[!] arm-none-eabi-gcc NOT found on PATH"
fi

echo
echo "=== Verifying Java ==="
java -version || echo "[!] Java runtime not found"

echo
echo "=== Verifying Python env ==="
if [[ -d "${HOME}/.venvs/entobench-ae" ]]; then
  # shellcheck disable=SC1090
  source "${HOME}/.venvs/entobench-ae/bin/activate"
  python -c "import numpy, pandas, scipy, matplotlib; print('OK: python stack')" || echo "[!] Missing python packages"
else
  echo "[!] Venv not found at ~/.venvs/entobench-ae"
fi

echo
echo "=== Verifying udev rule ==="
if [[ -f /etc/udev/rules.d/60-stlinkv3pwr.rules ]]; then
  echo "OK: /etc/udev/rules.d/60-stlinkv3pwr.rules present"
else
  echo "[!] udev rule missing"
fi

echo
echo "=== Hints ==="
echo "- Open a new shell or 'source ~/.bashrc' if PATH updates aren't visible."
echo "- Use 'cube-monitor-pwr' and 'logic2' if you added those aliases."
