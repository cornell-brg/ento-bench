#!/usr/bin/env bash
set -euo pipefail

# Config
TOOL_ROOT="${HOME}/.toolchains"
VER="14.3"
ARCHIVE_NAME="arm-gnu-toolchain-${VER}.rel1-x86_64-arm-none-eabi.tar.xz"
URL="https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/${VER}.rel1/binrel/${ARCHIVE_NAME}"
DEST="${TOOL_ROOT}/arm-none-eabi-${VER}"

mkdir -p "${TOOL_ROOT}"
cd "${TOOL_ROOT}"

if [[ -d "${DEST}" ]]; then
  echo "[*] ARM toolchain ${VER} already present at ${DEST}"
else
  echo "[*] Downloading ${URL}"
  wget -q --show-progress "${URL}"
  echo "[*] Extracting to ${DEST}"
  mkdir -p "${DEST}"
  tar -xf "${ARCHIVE_NAME}" -C "${DEST}" --strip-components=1
fi

# Export in shell startup if not present
BASHRC="${HOME}/.bashrc"
if ! grep -q 'ARM_TOOLCHAIN_ROOT' "${BASHRC}"; then
  {
    echo ""
    echo "# EntoBench ARM toolchain"
    echo "export ARM_TOOLCHAIN_ROOT=\"${DEST}\""
    echo 'export PATH="$ARM_TOOLCHAIN_ROOT/bin:$PATH"'
  } >> "${BASHRC}"
  echo "[*] Added ARM toolchain to ~/.bashrc (open a new shell or run: source ~/.bashrc)"
fi

# Verify
if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  echo "[*] arm-none-eabi-gcc found: $(arm-none-eabi-gcc --version | head -n1)"
else
  echo "[!] arm-none-eabi-gcc not on PATH yet. Run: source ~/.bashrc"
fi

echo "[*] Done."
