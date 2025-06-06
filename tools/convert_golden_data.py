#!/usr/bin/env python3
"""
Convert golden.csv data to EKFProblem format for RoboBee EKF benchmarking.

Golden CSV format (21 columns):
timestamp,rx,ry,rz,tof,actual_TCP_pose_0,...,actual_TCP_pose_5,x,y,z,roll,pitch,yaw,tx,ty,tz,ft

EKFProblem format for RoboBee (4 measurements, 1 control):
timestamp,dt,meas_0,meas_1,meas_2,meas_3,control_0,sensor_mask_0,sensor_mask_1,sensor_mask_2,sensor_mask_3

RoboBee EKF measurements:
- meas_0: rx (gyro x)
- meas_1: ry (gyro y) 
- meas_2: rz (gyro z)
- meas_3: tof (time-of-flight range sensor, convert cm to m)
"""

import pandas as pd
import numpy as np
import argparse
import sys
from pathlib import Path

def convert_golden_to_ekf_format(input_file, output_file, robot_type="robobee"):
    """
    Convert golden.csv to EKFProblem format for EKF benchmarking.
    
    Args:
        input_file: Path to golden.csv
        output_file: Path to output CSV for EKFProblem
        robot_type: Type of robot ("robobee" or "robofly")
    """
    print(f"Reading golden data from: {input_file}")
    print(f"Converting for robot type: {robot_type}")
    
    # Read the golden data
    try:
        df = pd.read_csv(input_file)
        print(f"Loaded {len(df)} data points")
    except Exception as e:
        print(f"Error reading {input_file}: {e}")
        return False
    
    # Validate expected columns exist
    required_cols = ['timestamp', 'rx', 'ry', 'rz', 'tof']
    if robot_type == "robobee":
        required_cols.extend(['tx', 'ty', 'tz', 'ft'])  # RoboBee needs 4 controls
    
    missing_cols = [col for col in required_cols if col not in df.columns]
    if missing_cols:
        print(f"Error: Missing required columns: {missing_cols}")
        print(f"Available columns: {list(df.columns)}")
        return False
    
    print("✓ All required columns found")
    
    # Remove duplicate timestamps (I noticed some in the data)
    initial_count = len(df)
    df = df.drop_duplicates(subset=['timestamp'], keep='first')
    if len(df) < initial_count:
        print(f"⚠ Removed {initial_count - len(df)} duplicate timestamps")
    
    # Sort by timestamp to ensure proper ordering
    df = df.sort_values('timestamp').reset_index(drop=True)
    
    # Compute dt (time differences)
    dt = np.diff(df['timestamp'].values)
    dt = np.append(dt, dt[-1])  # Use last dt for final point
    
    # Extract measurements based on robot type
    if robot_type == "robobee":
        # RoboBee: 4 measurements, 4 controls
        measurements = pd.DataFrame({
            'timestamp': df['timestamp'],
            'dt': dt,
            'meas_0': df['rx'],  # Gyro x (rad/s)
            'meas_1': df['ry'],  # Gyro y (rad/s)  
            'meas_2': df['rz'],  # Gyro z (rad/s)
            'meas_3': df['tof'] / 100.0,  # ToF range (cm -> m)
            'control_0': df['tx'],  # Torque x
            'control_1': df['ty'],  # Torque y
            'control_2': df['tz'],  # Torque z
            'control_3': df['ft'],  # Thrust force
        })
        
        print("RoboBee control value ranges:")
        print(f"  Tx: [{measurements['control_0'].min():.6f}, {measurements['control_0'].max():.6f}]")
        print(f"  Ty: [{measurements['control_1'].min():.6f}, {measurements['control_1'].max():.6f}]")
        print(f"  Tz: [{measurements['control_2'].min():.6f}, {measurements['control_2'].max():.6f}]")
        print(f"  F:  [{measurements['control_3'].min():.6f}, {measurements['control_3'].max():.6f}]")
        
    else:  # robofly
        # RoboFly: 4 measurements, 1 control (simplified for now)
        measurements = pd.DataFrame({
            'timestamp': df['timestamp'],
            'dt': dt,
            'meas_0': df['rx'],  # Gyro x (rad/s)
            'meas_1': df['ry'],  # Gyro y (rad/s)  
            'meas_2': df['rz'],  # Gyro z (rad/s)
            'meas_3': df['tof'] / 100.0,  # ToF range (cm -> m)
            'control_0': 0.0,  # Placeholder for RoboFly control
        })
    
    # Set sensor masks (all sensors active by default)
    measurements['sensor_mask_0'] = 1  # Gyro x active
    measurements['sensor_mask_1'] = 1  # Gyro y active
    measurements['sensor_mask_2'] = 1  # Gyro z active
    measurements['sensor_mask_3'] = 1  # ToF active
    
    # Check for any NaN values and handle them
    nan_count = measurements.isnull().sum().sum()
    if nan_count > 0:
        print(f"⚠ Found {nan_count} NaN values, filling with interpolation")
        measurements = measurements.interpolate(method='linear').fillna(method='bfill').fillna(method='ffill')
    
    # Data quality checks
    print("\n=== Data Quality Report ===")
    print(f"Time range: {measurements['timestamp'].min():.3f} to {measurements['timestamp'].max():.3f} seconds")
    print(f"Duration: {measurements['timestamp'].max() - measurements['timestamp'].min():.3f} seconds")
    print(f"Average dt: {measurements['dt'].mean()*1000:.2f} ms")
    print(f"Gyro ranges: rx=[{measurements['meas_0'].min():.3f}, {measurements['meas_0'].max():.3f}] rad/s")
    print(f"             ry=[{measurements['meas_1'].min():.3f}, {measurements['meas_1'].max():.3f}] rad/s")
    print(f"             rz=[{measurements['meas_2'].min():.3f}, {measurements['meas_2'].max():.3f}] rad/s")
    print(f"ToF range: [{measurements['meas_3'].min():.3f}, {measurements['meas_3'].max():.3f}] m")
    
    # Write the converted data
    try:
        measurements.to_csv(output_file, index=False, float_format='%.6f')
        print(f"\n✓ Successfully converted {len(measurements)} data points")
        print(f"✓ Output written to: {output_file}")
        return True
    except Exception as e:
        print(f"Error writing {output_file}: {e}")
        return False

def generate_synthetic_robofly_data(output_file, duration=5.0, dt=0.004, noise_level=0.01):
    """
    Generate synthetic data for RoboFly EKF (4-state, 4-measurement).
    
    RoboFly measurements:
    - meas_0: Range/altitude (ToF sensor)
    - meas_1: Optical flow x-velocity  
    - meas_2: Accelerometer x-component
    - meas_3: Accelerometer z-component
    """
    print(f"Generating synthetic RoboFly data for {duration}s at {dt}s intervals")
    
    timestamps = np.arange(0, duration, dt)
    n_points = len(timestamps)
    
    # Simulate simple hovering flight with small perturbations
    # RoboFly state: [theta, vx, z, vz]
    theta = 0.1 * np.sin(0.5 * timestamps) + noise_level * np.random.randn(n_points)  # pitch angle
    vx = 0.2 * np.cos(0.3 * timestamps) + noise_level * np.random.randn(n_points)     # x velocity
    z = 0.5 + 0.1 * np.sin(0.2 * timestamps) + noise_level * np.random.randn(n_points)  # altitude
    vz = np.gradient(z) / dt + noise_level * np.random.randn(n_points)                # z velocity
    
    # Generate measurements based on RoboFly measurement model
    g = 9.81  # gravity
    
    # Measurement model from robofly_ekf_v1.h
    meas_0 = z / np.cos(theta) + 0.01 * np.random.randn(n_points)  # Range = altitude/cos(pitch)
    meas_1 = vx + 0.02 * np.random.randn(n_points)                  # Optical flow (simplified)
    meas_2 = -g * np.sin(theta) + 0.1 * np.random.randn(n_points)  # Accel x from gravity
    meas_3 = -g * np.cos(theta) + 0.1 * np.random.randn(n_points)  # Accel z from gravity
    
    # Control input (angular velocity command)
    omega = 0.1 * np.sin(1.0 * timestamps) + 0.02 * np.random.randn(n_points)
    
    # Create DataFrame
    data = pd.DataFrame({
        'timestamp': timestamps,
        'dt': np.full(n_points, dt),
        'meas_0': meas_0,  # Range
        'meas_1': meas_1,  # Optical flow
        'meas_2': meas_2,  # Accel x
        'meas_3': meas_3,  # Accel z
        'control_0': omega,  # Angular velocity command
        'sensor_mask_0': 1,
        'sensor_mask_1': 1,
        'sensor_mask_2': 1,
        'sensor_mask_3': 1
    })
    
    try:
        data.to_csv(output_file, index=False, float_format='%.6f')
        print(f"✓ Generated {len(data)} synthetic RoboFly data points")
        print(f"✓ Output written to: {output_file}")
        return True
    except Exception as e:
        print(f"Error writing {output_file}: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Convert golden data or generate synthetic data for EKF benchmarking')
    parser.add_argument('--input', '-i', type=str, help='Input golden.csv file')
    parser.add_argument('--output', '-o', type=str, required=True, help='Output CSV file for EKFProblem')
    parser.add_argument('--robot', '-r', choices=['robobee', 'robofly'], default='robobee',
                       help='Robot type (determines EKF format)')
    parser.add_argument('--synthetic', '-s', action='store_true',
                       help='Generate synthetic data instead of converting')
    parser.add_argument('--duration', '-d', type=float, default=5.0,
                       help='Duration for synthetic data (seconds)')
    parser.add_argument('--dt', type=float, default=0.004,
                       help='Time step for synthetic data (seconds)')
    
    args = parser.parse_args()
    
    # Ensure output directory exists
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    if args.synthetic:
        if args.robot == 'robofly':
            success = generate_synthetic_robofly_data(args.output, args.duration, args.dt)
        else:
            print("Synthetic data generation only implemented for RoboFly currently")
            success = False
    else:
        if not args.input:
            print("Error: --input required when not using --synthetic")
            return 1
            
        if not Path(args.input).exists():
            print(f"Error: Input file {args.input} does not exist")
            return 1
            
        success = convert_golden_to_ekf_format(args.input, args.output, args.robot)
    
    return 0 if success else 1

if __name__ == '__main__':
    sys.exit(main()) 