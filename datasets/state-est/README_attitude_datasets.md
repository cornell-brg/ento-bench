# Attitude Estimation Dataset Generation Guide

This guide explains how to generate datasets for attitude estimation benchmarks from the `golden.csv` ground truth data.

## Quick Start

### 1. Generate Sensor Data and Tuned Parameters
```bash
# Generate ICM-42688-P sensor data (recommended - cleaner sensor)
python generate_marg_from_golden.py --sensor-model ICM-42688-P --sample-rate 1000 --interpolate --output-prefix tuned_icm42688_1khz_

# Generate ICM-20948 sensor data (original, noisier sensor)
python generate_marg_from_golden.py --sensor-model ICM-20948 --sample-rate 1000 --interpolate --output-prefix tuned_icm20948_1khz_
```

### 2. Create Benchmark Datasets
```bash
# Create C++ benchmark datasets from the generated sensor data
python create_benchmark_dataset.py \
  --sensor-file tuned_icm42688_1khz_sensor_data.csv \
  --gt-file tuned_icm42688_1khz_gt_quats.csv \
  --output-prefix tuned_icm42688_1khz
```

## File Outputs

### Primary Dataset Files (used by C++ benchmarks):
- `tuned_icm42688_1khz_marg_dataset.txt` - MARG format: `ax ay az gx gy gz mx my mz qw qx qy qz dt`
- `tuned_icm42688_1khz_imu_dataset.txt` - IMU format: `ax ay az gx gy gz qw qx qy qz dt`

### Intermediate Files (for analysis):
- `tuned_icm42688_1khz_sensor_data.csv` - Raw sensor data with realistic noise
- `tuned_icm42688_1khz_gt_quats.csv` - Ground truth quaternions
- `tuned_icm42688_1khz_*_quats.csv` - Python AHRS filter results (for tuning validation)

## Key Parameters

### Sensor Models:
- **ICM-42688-P**: 2.8 mdps/√Hz gyro, 65 µg/√Hz accel (recommended)
- **ICM-20948**: 15 mdps/√Hz gyro, 230 µg/√Hz accel (legacy)

### Optimized Gains (auto-tuned via grid search):
- **Mahony IMU**: kp=0.01, ki=0.001
- **Mahony MARG**: kp=0.01, ki=0.001  
- **Madgwick IMU**: gain=0.001
- **Madgwick MARG**: gain=0.001
- **Fourati MARG**: gain=0.1

### Dataset Characteristics:
- **Sample Rate**: 1kHz (interpolated from ~230Hz golden.csv)
- **Duration**: ~5.2 seconds, 5222 samples
- **Noise**: Realistic MEMS IMU noise based on datasheets
- **Format**: Space-separated values with "Attitude Estimation Problem" header

## Quick Commands Reference

```bash
# Generate both sensor models with default settings
python generate_marg_from_golden.py --sensor-model ICM-42688-P --sample-rate 1000 --interpolate --output-prefix tuned_icm42688_1khz_
python generate_marg_from_golden.py --sensor-model ICM-20948 --sample-rate 1000 --interpolate --output-prefix tuned_icm20948_1khz_

# Create benchmark datasets for both models
python create_benchmark_dataset.py --sensor-file tuned_icm42688_1khz_sensor_data.csv --gt-file tuned_icm42688_1khz_gt_quats.csv --output-prefix tuned_icm42688_1khz
python create_benchmark_dataset.py --sensor-file tuned_icm20948_1khz_sensor_data.csv --gt-file tuned_icm20948_1khz_gt_quats.csv --output-prefix tuned_icm20948_1khz
```

## Dependencies
- Python packages: `numpy`, `scipy`, `ahrs`, `matplotlib`, `tqdm`
- Input file: `golden.csv` (ground truth trajectory data)

## Notes
- Use `--interpolate` flag for sample rates > 250Hz
- The `generate_marg_from_golden.py` script performs automatic parameter tuning via grid search
- Generated datasets include proper scaling (nT→µT magnetometer conversion)
- All filter gains are pre-tuned for optimal performance on this specific dataset 