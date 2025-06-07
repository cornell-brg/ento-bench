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
    sensor_data = pd.read_csv(sensor_file)
    gt_data = pd.read_csv(gt_file)
    
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
    np.savetxt(output_file, combined_array, fmt='%.12e', delimiter=' ')
    
    print(f"Created {output_file} with {len(combined_data)} samples")
    print(f"Format: {'MARG' if use_mag else 'IMU'}")
    print(f"Columns: {combined_array.shape[1]}")

if __name__ == "__main__":
    # Create both MARG and IMU datasets
    create_benchmark_dataset(
        'benchmark_sensor_data.csv',
        'benchmark_gt_quats.csv', 
        'benchmark_marg_dataset.txt',
        use_mag=True
    )
    
    create_benchmark_dataset(
        'benchmark_sensor_data.csv',
        'benchmark_gt_quats.csv',
        'benchmark_imu_dataset.txt', 
        use_mag=False
    )
    
    print("\nDataset creation complete!")
    print("Files created:")
    print("  benchmark_marg_dataset.txt - For MARG filters (with magnetometer)")
    print("  benchmark_imu_dataset.txt  - For IMU filters (without magnetometer)") 