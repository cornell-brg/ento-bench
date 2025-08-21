#!/usr/bin/env python3
import numpy as np
import pandas as pd
import math
from scipy.spatial.transform import Rotation as R

# Load the trajectory data
csv_file = 'robofly-traj1.csv'
print(f"Loading trajectory data from {csv_file}...")
df = pd.read_csv(csv_file)

# Function to convert quaternion to roll-pitch-yaw (Euler angles)
def quat_to_euler(w, x, y, z):
    rot = R.from_quat([x, y, z, w])  # scipy uses [x, y, z, w] format
    euler = rot.as_euler('xyz', degrees=False)
    return euler

# Function to calculate velocities using finite differences
def calculate_velocities(df, dt=0.0001):  # dt from the CSV time step
    # Calculate position velocities
    df['vel_x'] = df['pos_x'].diff() / dt
    df['vel_y'] = df['pos_y'].diff() / dt
    df['vel_z'] = df['pos_z'].diff() / dt
    
    # Fill NaN values in the first row
    df.loc[0, 'vel_x'] = 0.0
    df.loc[0, 'vel_y'] = 0.0
    df.loc[0, 'vel_z'] = 0.0
    
    return df

# Function to calculate Euler angles from quaternions
def calculate_euler_angles(df):
    # Calculate roll, pitch, yaw from quaternions
    euler_angles = np.array([quat_to_euler(row['quat_w'], row['quat_x'], row['quat_y'], row['quat_z']) 
                            for _, row in df.iterrows()])
    
    df['roll'] = euler_angles[:, 0]   # x-axis rotation
    df['pitch'] = euler_angles[:, 1]  # y-axis rotation
    df['yaw'] = euler_angles[:, 2]    # z-axis rotation
    
    return df

# Function to calculate angular velocities
def calculate_angular_velocities(df, dt=0.0001):
    # Calculate angular velocities using finite differences
    df['omega_x'] = df['roll'].diff() / dt
    df['omega_y'] = df['pitch'].diff() / dt
    df['omega_z'] = df['yaw'].diff() / dt
    
    # Fill NaN values in the first row
    df.loc[0, 'omega_x'] = 0.0
    df.loc[0, 'omega_y'] = 0.0
    df.loc[0, 'omega_z'] = 0.0
    
    return df

# Process the data
df = calculate_velocities(df)
df = calculate_euler_angles(df)
df = calculate_angular_velocities(df)

# Select a few time points for testing
times = [0.0, 0.1, 0.5, 1.0, 1.5, 2.0]
test_states = []

print("\nState vectors for testing LQR controller:")
print("Format: [x, y, z, vx, vy, vz, roll, pitch, omega_x, omega_y]")

for t in times:
    # Find the closest time point in the data
    idx = (df['Time'] - t).abs().idxmin()
    row = df.loc[idx]
    
    # Extract the state vector components
    state = [
        row['pos_x'], row['pos_y'], row['pos_z'],
        row['vel_x'], row['vel_y'], row['vel_z'],
        row['roll'], row['pitch'],
        row['omega_x'], row['omega_y']
    ]
    
    # Format for C++ code
    cpp_state = ", ".join(f"{v:.6f}f" for v in state)
    
    print(f"\nTime {row['Time']:.4f}:")
    print(f"x0 << {cpp_state};")
    
    # Reference state (desired position with zeros for other components)
    ref_state = [
        row['des_x'], row['des_y'], row['des_z'],
        0.0, 0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0
    ]
    cpp_ref = ", ".join(f"{v:.6f}f" for v in ref_state)
    print(f"x_ref << {cpp_ref};")
    
print("\nProcessing complete!") 