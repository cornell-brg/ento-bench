#!/usr/bin/env python
"""
Create benchmark dataset in the format expected by AttitudeProblem.

Expected format for MARG (UseMag=true):
ax ay az gx gy gz mx my mz qw qx qy qz dt

Expected format for IMU (UseMag=false):  
ax ay az gx gy gz qw qx qy qz dt
"""

import pandas as pd
import numpy as np
import argparse

def create_benchmark_dataset(sensor_file, gt_file, output_file, use_mag=True):
    """
    Combine sensor data and ground truth into AttitudeProblem format.
    
    Args:
        sensor_file: CSV file with sensor data (timestamp,gyr_x,gyr_y,gyr_z,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z)
        gt_file: CSV file with ground truth (timestamp,qw,qx,qy,qz)
        output_file: Output file for combined dataset
        use_mag: Whether to include magnetometer data (MARG vs IMU)
    """
    
    # Read sensor data
    sensor_data = pd.read_csv(sensor_file, skiprows=1)  # Skip the "Attitude Estimation Problem" header
    gt_data = pd.read_csv(gt_file, skiprows=1)  # Skip the "Attitude Estimation Problem" header
    
    print(f"Sensor data shape: {sensor_data.shape}")
    print(f"Ground truth shape: {gt_data.shape}")
    
    # Ensure same number of samples
    min_samples = min(len(sensor_data), len(gt_data))
    sensor_data = sensor_data.iloc[:min_samples]
    gt_data = gt_data.iloc[:min_samples]
    
    # Calculate dt from timestamps
    timestamps = sensor_data['timestamp'].values
    dt_values = np.diff(timestamps)
    dt_avg = np.mean(dt_values)
    print(f"Average dt: {dt_avg:.6f} seconds")
    
    # Create combined dataset
    combined_data = []
    
    for i in range(len(sensor_data)):
        # Get sensor measurements
        acc_x = sensor_data.iloc[i]['acc_x']
        acc_y = sensor_data.iloc[i]['acc_y'] 
        acc_z = sensor_data.iloc[i]['acc_z']
        
        gyr_x = sensor_data.iloc[i]['gyr_x']
        gyr_y = sensor_data.iloc[i]['gyr_y']
        gyr_z = sensor_data.iloc[i]['gyr_z']
        
        # Get ground truth quaternion
        qw = gt_data.iloc[i]['qw']
        qx = gt_data.iloc[i]['qx']
        qy = gt_data.iloc[i]['qy']
        qz = gt_data.iloc[i]['qz']
        
        # Use average dt for all samples
        dt = dt_avg
        
        if use_mag:
            # MARG format: ax ay az gx gy gz mx my mz qw qx qy qz dt
            mag_x = sensor_data.iloc[i]['mag_x']
            mag_y = sensor_data.iloc[i]['mag_y']
            mag_z = sensor_data.iloc[i]['mag_z']
            
            row = [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, 
                   mag_x, mag_y, mag_z, qw, qx, qy, qz, dt]
        else:
            # IMU format: ax ay az gx gy gz qw qx qy qz dt
            row = [acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, 
                   qw, qx, qy, qz, dt]
        
        combined_data.append(row)
    
    # Save to file (no header, space-separated)
    combined_array = np.array(combined_data)
    
    # Add header as first line, then save data with fixed decimal notation
    with open(output_file, 'w') as f:
        f.write("Attitude Estimation Problem\n")
        np.savetxt(f, combined_array, fmt='%.6f', delimiter=' ')
    
    print(f"Created {output_file} with {len(combined_data)} samples")
    print(f"Format: {'MARG' if use_mag else 'IMU'}")
    print(f"Columns: {combined_array.shape[1]}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Create benchmark datasets from sensor and ground truth files')
    parser.add_argument('--sensor-file', required=True, help='CSV file with sensor data')
    parser.add_argument('--gt-file', required=True, help='CSV file with ground truth quaternions')
    parser.add_argument('--output-prefix', default='benchmark', help='Output file prefix')
    parser.add_argument('--marg-only', action='store_true', help='Create only MARG dataset')
    parser.add_argument('--imu-only', action='store_true', help='Create only IMU dataset')
    
    args = parser.parse_args()
    
    # Create datasets based on flags
    if not args.imu_only:
        create_benchmark_dataset(
            args.sensor_file,
            args.gt_file, 
            f'{args.output_prefix}_marg_dataset.txt',
            use_mag=True
        )
    
    if not args.marg_only:
        create_benchmark_dataset(
            args.sensor_file,
            args.gt_file,
            f'{args.output_prefix}_imu_dataset.txt', 
            use_mag=False
        )
    
    print("\nDataset creation complete!")
    if not args.imu_only:
        print(f"  {args.output_prefix}_marg_dataset.txt - For MARG filters (with magnetometer)")
    if not args.marg_only:
        print(f"  {args.output_prefix}_imu_dataset.txt  - For IMU filters (without magnetometer)") 