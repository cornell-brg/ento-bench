<div align="center">
  <h1>EntoBench: A Benchmarking Framework for Insect-Scale Robotics</h1>

  <img src="misc/entobench-logo.png" alt="EntoBench Logo" width="400"/>

  <!-- CI badges intentionally omitted for AE to avoid red badges -->
  <a href="https://www.st.com/en/microcontrollers-microprocessors.html">
    <img src="https://img.shields.io/badge/Platform-Linux%20%7C%20Cortex--M%20%7C%20STM32-blue.svg" alt="Platform"/>
  </a>
  <a href="https://cmake.org/">
    <img src="https://img.shields.io/badge/CMake-3.16%2B-064F8C?logo=cmake&logoColor=white" alt="CMake"/>
  </a>

  <p><strong>Version:</strong> v1.0.0-ae</p>
</div>

---

## Overview

**EntoBench** is a framework and benchmark suite for computer vision, state estimation, and control kernels on insect-scale robots.  
It targets **ARM Cortex-M (STM32)** MCUs and provides reproducible measurements of:

- Cycle counts
- Latency
- Peak power
- Energy

This **Artifact Evaluation (AE)** release reproduces the results in the paper’s *Workload Characterization* section using automated build scripts and GUI-based measurement tools.

---

## Supported Hardware

### Target Boards
- **NUCLEO-STM32G474RE** (Cortex-M4F)
- **NUCLEO-STM32U575ZIQ** (Cortex-M33F)
- **NUCLEO-STM32H7A3ZIQ** (Cortex-M7F)

### Measurement Hardware (optional)
- **STLINK-V3PWR** — current measurements, flashing, semihosting
- **Saleae Logic Pro** — logic timing & trigger capture

> Alternative probes (e.g., X-NUCLEO-LPM01A) may work but are not officially supported.

---

## Quick Links

- **Docs**: `docs/` (hardware setup, Logic 2, CMP config)
- **Experiment flow scripts**: `scripts/flow/`
- **One-shot install scripts**: `scripts/00..05-*.sh`

---

## Installation (Ubuntu 24.04)

All steps are automated via scripts in `scripts/`. Run them in order:

### 1) System Packages
```bash
cd scripts
./00-install-system-packages.sh
```

### 2) ARM GNU Toolchain (14.3)
```bash
./01-install-arm-toolchain.sh
```

### 3) Python Environment
```bash
./02-setup-python-venv.sh
source ~/.venvs/entobench-ae/bin/activate
```

### 4) STLINK-V3PWR udev Rules
```bash
./03-setup-udev-stlinkv3pwr.sh
# unplug/replug STLINK-V3PWR after this
```

### 5) GUI Apps (manual download, then register)
- **STM32CubeMonPwr**: download from ST and extract to  
  `~/external/STMicroelectronics/STM32CubeMonitor-Power/`
- **Saleae Logic 2**: place AppImage under  
  `~/external/logic2/`

Register convenient launchers:
```bash
./04-register-cmp-logic2.sh
source ~/.bashrc
```

### 6) Verify Setup
```bash
./05-verify-setup.sh
```

---

## Quick Start (AE Workflow)

### 0) Clone
```bash
git clone https://github.com/cornell-brg/ento-bench.git
cd ento-bench
```

### 1) Create build & experiment structure
```bash
./scripts/flow/00-setup-experiment-env.sh
```
Creates:
- Builds: `build/build-g474`, `build/build-u575`, `build/build-h7a3`
- Experiments: `experiments/ae/m4/example/{example-cache,example-nocache}`

### 2) Launch GUIs
```bash
./scripts/flow/01-launch-apps.sh
```
Then in your desktop/XRDP session, bring **STM32CubeMonitor-Power** and **Logic 2** to the foreground.

### 3) Flash & run the example benchmark (with acquisition)
1. In **Logic 2**: press **Play**  
2. In **CMP**: press **Start Acquisition**  
3. In a terminal:
   ```bash
   cd build/build-g474
   make stm32-flash-bench-example-semihosted
   ```
4. Wait until the trigger pin (D7) goes low in **Logic 2**, then stop both acquisitions.

### 4) Save measurement data
- **Logic 2**: Save a `.sal` **and** Export Raw Data to  
  `experiments/ae/m4/example/example-cache/`
- **CMP**: “Save Graph” to the **same** directory  
  *(tip: the button can be finicky—click near the lower-right corner)*

### 5) Analyze
```bash
source ~/.venvs/entobench-ae/bin/activate
./scripts/flow/02-analyze-energy.sh \
  -d experiments/ae/m4/example/example-cache
```
Outputs include cycle counts, average energy, peak power, and latency.  
Expected/illustrative outputs are shown in `docs/expected-results.md` (PDF provided).

---

## Experiment Customization

Change per-benchmark parameters in:
- `benchmark/configs/example_benchmarks.json` (for the example)
- Other domains under `benchmark/configs/`

**Disable caches** by setting:
```json
"enable_caches": false
```

Re-configure builds to propagate config changes:
```bash
./scripts/clean-and-rebuild.sh
```

---

## Troubleshooting

**CMP won’t launch (JavaFX error) or no GUI under XRDP**
- Use the registered launcher which runs the **bundled JRE** and XRDP-friendly flags:
  ```bash
  cube-monitor-pwr
  ```
  (The `04-register-cmp-logic2.sh` script sets this up as:
  `SWT_GTK3=0 _JAVA_OPTIONS="-Dsun.java2d.xrender=false" "$CMP_DIR/jre/bin/java" -jar "$CMP_DIR/STM32CubeMonitor-Power.jar"`)

**Logic 2 AppImage won’t start (FUSE)**
- Install FUSE v2 compatibility: `sudo apt-get install -y libfuse2`

**udev permissions**
- Re-run `./03-setup-udev-stlinkv3pwr.sh` and unplug/replug the device.

**CMake floods warnings**
- AE release globally suppresses most warnings in the top-level CMake for clarity.

---

## Repository Structure (high level)

```
ento-bench/
├─ benchmark/                # Benchmarks and configs
│  └─ configs/               # JSON configs (e.g., enable_caches)
├─ docs/                     # Setup guides, expected results, customization
├─ scripts/                  # Install & helper scripts
│  ├─ install/               # AE install scripts (00,01,02...,05)
│  └─ flow/                  # AE workflow scripts (00,01,02...)
├─ src/                      # Source code (kernels, eval framework)
├─ stm32-cmake/              # Toolchain files & MCU configs
├─ tools/                    # Analysis scripts (rename/sync/analyze)
└─ build/                    # Generated build dirs (after setup)
```

---

## Versioning

- **Tag**: `v1.0.0-ae`  
- Any AE-only fixes will bump the last segment: `v1.0.0-ae.1`, `v1.0.0-ae.2`, …

---

## Citation

Please cite our paper (to appear) if you use EntoBench in your research. Will be added
upon publication.

---

## License

TBD (will be updated upon camera-ready).

---

## Contact / Issues

- Issues & feature requests: GitHub Issues
- Hardware enablement requests welcome (open an issue with board details)
