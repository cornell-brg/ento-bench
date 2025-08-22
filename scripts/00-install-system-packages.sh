#!/usr/bin/env bash
set -euo pipefail

echo "[*] Installing system packages (Ubuntu 24.04)…"
sudo apt-get update -y

# Build & tool helpers
sudo apt-get install -y \
  build-essential pkg-config autoconf automake libtool cmake \
  git curl wget unzip \
  libjim-dev jimsh python3-venv

# Java 17 runtime for STM32CubeMonitor-Power installer
sudo apt-get install -y openjdk-17-jre

# Optional: set default Java to 17 (if multiple installed)
if update-alternatives --query java >/dev/null 2>&1; then
  sudo update-alternatives --set java /usr/lib/jvm/java-1.17.0-openjdk-amd64/bin/java || true
fi

echo "[*] Done."
