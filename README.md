<div align="center">
  <h1>EntoBench: A Benchmarking Framework for Insect-Scale Robotics</h1>
  
  <img src="misc/entobench-logo.png" alt="EntoBench Logo" width="400"/>
  
  [![Tests](https://github.com/derinozturk/ento-bench/actions/workflows/tests.yml/badge.svg)](https://github.com/derinozturk/ento-bench/actions/workflows/tests.yml)
  [![Build Status](https://github.com/derinozturk/ento-bench/actions/workflows/build.yml/badge.svg)](https://github.com/derinozturk/ento-bench/actions/workflows/build.yml)
  [![Platform](https://img.shields.io/badge/Platform-Linux%20%7C%20Cortex--M%20%7C%20STM32-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors.html)
  [![CMake](https://img.shields.io/badge/CMake-3.16%2B-064F8C?logo=cmake&logoColor=white)](https://cmake.org/)
</div>

## Overview

EntoBench is a comprehensive evaluation framework and benchmark suite for computer vision, state estimation, and control algorithms on insect-scale robots. The framework targets ARM Cortex-M microcontrollers (STM32) and provides sophisticated benchmarking capabilities with performance measurement tools for latency, peak power, and energy analysis.

### Key Features

- **Multi-target support**: Native (x86/ARM64), STM32 microcontrollers, gem5 simulation
- **Comprehensive algorithm coverage**: Computer vision, pose estimation, attitude estimation, control systems  
- **Performance measurement**: Cycle counts, latency, peak power, and energy analysis
- **Professional benchmarking harness**: Configurable metrics, dataset-driven testing
- **STM32 integration**: Automated flashing, semihosting support, OpenOCD integration
- **JSON-based configuration**: Sophisticated parameter control for experiments

## Supported Hardware

EntoBench supports the following STM32 development boards:

- **NUCLEO-STM32G474RE** (Cortex-M4 with FPU)
- **NUCLEO-STM32U575ZIQ** (Cortex-M33 with FPU)  
- **NUCLEO-STM32H7A3ZIQ** (Cortex-M7 with FPU)
- **NUCLEO-C092RC** (Cortex-M0+)

### Measurement Hardware (Optional)

For power and energy measurements:
- **STLINK-V3PWR** - Current measurements, flashing, and semihosting
- **Saleae Logic Pro Analyzer** - Logic analysis and timing measurements

> Alternative: X-NUCLEO-LPM01A current probe (commonly used in TinyML papers) may work but is not officially supported.

## Installation

### Prerequisites

#### Ubuntu 24.04

```bash
# Install system packages
sudo apt update
sudo apt install build-essential pkg-config automake libtool autoconf cmake \
                 libjim-dev jimsh openjdk-17-jre

# Verify Java version
java -version
# If not version 17, configure it:
sudo update-alternatives --config java
```

#### macOS

```bash
# Install system packages
brew install cmake pkg-config automake libtool autoconf
brew install openjdk@17
```

### ARM GNU Embedded Toolchain

Download and install the official ARM toolchain:

```bash
# Create toolchain directory
mkdir -p ~/.toolchains
cd ~/.toolchains

# Download ARM GNU Toolchain 14.3
wget "https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi.tar.xz" -O arm-none-eabi-14.3.tar.xz

# Extract toolchain
tar -xf arm-none-eabi-14.3.tar.xz

# Add to PATH
echo 'export ARM_TOOLCHAIN_ROOT=$HOME/.toolchains/arm-gnu-toolchain-14.3.rel1-x86_64-arm-none-eabi' >> ~/.bashrc
echo 'export PATH="$ARM_TOOLCHAIN_ROOT/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Verify installation
arm-none-eabi-gcc --version
```

### OpenOCD (STM32 Support)

```bash
# Clone and build OpenOCD with STM32 support
git clone https://git.code.sf.net/p/openocd/code openocd
cd openocd
git checkout bc9ca5f4a
./bootstrap
./configure
make -j$(nproc)
sudo make install

# Set environment variables
echo 'export OPENOCD_HOME="/usr/local/share/openocd/scripts"' >> ~/.bashrc
echo 'export OPENOCD_PATH="/usr/local/share/openocd/scripts"' >> ~/.bashrc
source ~/.bashrc
```

### Power Measurement Tools (Optional)

#### STM32CubeMonPwr

1. Download from [STM32CubeMonPwr page](https://www.st.com/en/development-tools/stm32cubemonpwr.html)
2. Create STMicroelectronics account if needed
3. Install:

```bash
mkdir -p ~/workspace/external
mv ~/Downloads/cubemonpwr-lin-v1-2-1.zip ~/workspace/external/
cd ~/workspace/external
unzip cubemonpwr-lin-v1-2-1.zip
cd cubemonpwr-1.2.1

# Install (choose ~/workspace/external/STMicroelectronics when prompted)
java -jar SetupSTM32CubeMonitor-Power.jar

# Create alias
echo 'alias cube-monitor-pwr="java -jar ~/workspace/external/STMicroelectronics/STM32CubeMonitor-Power/STM32CubeMonitor-Power.jar"' >> ~/.bashrc
```

4. Update udev rules for STLINK-V3PWR:

```bash
sudo tee /etc/udev/rules.d/60-stlinkv3pwr.rules << EOF
SUBSYSTEMS=="usb", ATTRS{idVendor}=="3757", MODE:="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### Saleae Logic 2

Install from [Saleae Logic 2 page](https://support.saleae.com/logic-software/sw-installation).

## Quick Start

### 1. Clone and Setup

```bash
git clone https://github.com/derinozturk/ento-bench.git
cd ento-bench
```

### 2. Native Build (Quick Test)

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make check
```

### 3. STM32 Build and Flash

```bash
# Create build directory for your target MCU
mkdir build-g474 && cd build-g474

# Configure for STM32G474RE
cmake -DCMAKE_TOOLCHAIN_FILE=../stm32-cmake/stm32-g474re.cmake \
      -DFETCH_ST_SOURCES=True \
      -DOPENOCD_CFG=semihosting_stm32g4.cfg \
      ..

# Build benchmarks
make -j$(nproc)

# Flash and run a benchmark (connect STM32 board first)
make stm32-flash-bench-mahony-float-imu-semihosted
```

## Available Benchmarks

EntoBench provides benchmarks across multiple robotics domains:

### Perception  
- **Feature Detection/Description**: FAST+BRIEF, SIFT, ORB
- **Optical Flow**: Lucas-Kanade, block matching, image interpolation

### State Estimation
- **Attitude Estimation**: Mahony, Madgwick, Fourati filters
- **Pose Estimation**: P3P, UP2P, DLT, 5-point, 8-point, Homography, Upright Planar 2pt, Upright Planar 3pt, Upright 3pt
- **Robust Estimation**: LO-RANSAC variants for outlier rejection
- **Extended Kalman Filters**: RoboFly and RoboBee EKF implementations given a ToF+IMU, and ToF+IMU+OpticalFlow sensor suites respectively.

### Control Systems
- **Linear Controllers**: LQR for RoboFly
- **Nonlinear Controllers**: Geometric control on SO(3)
- **Model Predictive Control**: TinyMPC implementation for RoboFly
- **Adaptive Control**: Sliding Mode Adaptive Controller for RoboBee 

## Benchmark Naming Convention

Individual benchmarks follow this pattern:
```bash
# Format: bench-<algorithm>-<precision>-<variant>
make stm32-flash-bench-mahony-float-imu-semihosted
make stm32-flash-bench-p3p-double-semihosted  
make stm32-flash-bench-madgwick-q7-24-marg-semihosted
```

## Power and Energy Measurements

For detailed power analysis:

1. Connect STLINK-V3PWR to target board
2. Start STM32CubeMonPwr application
3. Configure Logic 2 for timing analysis
4. Run benchmarks with current measurement
5. Process results using provided analysis scripts

See `docs/hw-setup.md` and `docs/cube-monitor-cfg.md` for detailed setup guides.

## Configuration

EntoBench uses JSON-based configuration for flexible benchmark tuning:

- `benchmark/configs/estimation_benchmarks.json` - State estimation parameters
- `benchmark/configs/control_benchmarks.json` - Control system parameters  
- `benchmark/configs/perception_benchmarks.json` - Computer vision parameters

Each configuration supports:
- **Repetitions**: Number of benchmark runs
- **Inner repetitions**: Algorithm iterations per run
- **Verbosity levels**: Output detail control
- **Cache settings**: Memory optimization flags
- **Problem sizes**: Dataset scaling parameters

## Documentation

- `docs/build_system_analysis.md` - Build system architecture
- `docs/stm32_flash_targets.md` - Available STM32 flash targets
- `docs/cmake_cleanup_plan.md` - Build system documentation
- Hardware setup guides in `docs/` directory

## Contributing

EntoBench is actively developed for insect-scale robotics research. Please see the issues page for known limitations and planned improvements.

## License

[License information to be added]

## Citation

If you use EntoBench in your research, please cite:

```bibtex
[Citation to be added when published]
```

---

*EntoBench is developed for advancing embedded system benchmarking in insect-scale robotics applications.*


