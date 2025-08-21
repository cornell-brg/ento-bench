#!/usr/bin/env python3
"""
Generate RoboFly control datasets from test trajectory data.

This script converts the hardcoded trajectory data from the test files
into CSV datasets that can be used by the idiomatic control benchmarks.

Outputs:
  datasets/control/robofly_lqr_linear.csv
  datasets/control/robofly_lqr_trajectory.csv  
  datasets/control/robofly_tinympc_linear.csv
  datasets/control/robofly_tinympc_trajectory.csv
"""

import numpy as np
import os
from pathlib import Path

# Trajectory data from test_lqr.cc (the "authors' XLS data" converted to hardcoded values)
trajectory_data = [
    # Test point 1: t=0.0
    [0.092087, -0.167926, 0.261501, 0.000000, 0.000000, 0.000000, 0.021123, 0.030017, 0.000000, 0.000000],
    # Test point 2: t=0.5  
    [0.108290, -0.175858, 0.299720, 0.501430, -0.800790, 2.290600, 0.101524, 0.131200, 31.073203, -124.922303],
    # Test point 3: t=2.0
    [0.104976, -0.171701, 0.308092, 0.703340, 1.477890, 0.656840, 0.032568, 0.067413, 110.511214, -107.171688]
]

# Reference state (same for all points)
reference_state = [0.090000, -0.168000, 0.288000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]

def generate_linear_trajectory(num_points=50):
    """Generate a simple linear trajectory for testing."""
    trajectory = []
    
    # Start and end points
    start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    end = np.array([1.0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    for i in range(num_points):
        t = i / (num_points - 1)  # 0 to 1
        state = start + t * (end - start)
        trajectory.append(state.tolist())
    
    return trajectory

def generate_extended_trajectory(base_data, num_points=100):
    """Extend the base trajectory data with interpolation and extrapolation."""
    trajectory = []
    
    # Add the original data points
    for point in base_data:
        trajectory.append(point)
    
    # Generate additional points by interpolating between existing points
    # and adding some variation
    for i in range(num_points - len(base_data)):
        # Pick two random existing points to interpolate between
        idx1 = np.random.randint(0, len(base_data))
        idx2 = np.random.randint(0, len(base_data))
        
        # Random interpolation factor
        alpha = np.random.random()
        
        # Interpolate
        point1 = np.array(base_data[idx1])
        point2 = np.array(base_data[idx2])
        new_point = alpha * point1 + (1 - alpha) * point2
        
        # Add small random noise
        noise = np.random.normal(0, 0.01, 10)
        new_point += noise
        
        trajectory.append(new_point.tolist())
    
    return trajectory

def save_dataset(data, filename, header_comment="RoboFly control dataset"):
    """Save trajectory data to CSV file."""
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    
    with open(filename, 'w') as f:
        f.write(f"# {header_comment}\n")
        f.write("# State format: x,y,z,vx,vy,vz,roll,pitch,omega_x,omega_y\n")
        
        for state in data:
            # Format each state as comma-separated values
            line = ",".join(f"{val:.6f}" for val in state)
            f.write(line + "\n")
    
    print(f"Generated {filename} with {len(data)} trajectory points")

def main():
    """Generate all RoboFly control datasets."""
    print("Generating RoboFly control datasets...")
    
    # Create datasets directory
    datasets_dir = Path("../datasets/control")
    datasets_dir.mkdir(parents=True, exist_ok=True)
    
    # Generate linear trajectory datasets
    linear_traj = generate_linear_trajectory(50)
    save_dataset(linear_traj, 
                datasets_dir / "robofly_lqr_linear.csv",
                "RoboFly LQR linear trajectory dataset")
    save_dataset(linear_traj, 
                datasets_dir / "robofly_tinympc_linear.csv", 
                "RoboFly TinyMPC linear trajectory dataset")
    
    # Generate extended trajectory datasets from test data
    extended_traj = generate_extended_trajectory(trajectory_data, 80)
    save_dataset(extended_traj,
                datasets_dir / "robofly_lqr_trajectory.csv",
                "RoboFly LQR trajectory dataset (from test data)")
    save_dataset(extended_traj,
                datasets_dir / "robofly_tinympc_trajectory.csv", 
                "RoboFly TinyMPC trajectory dataset (from test data)")
    
    print("\n✅ All RoboFly control datasets generated successfully!")
    print(f"📁 Datasets saved to: {datasets_dir.resolve()}")
    print("\nDatasets created:")
    print("  - robofly_lqr_linear.csv (50 points)")
    print("  - robofly_lqr_trajectory.csv (80 points)")  
    print("  - robofly_tinympc_linear.csv (50 points)")
    print("  - robofly_tinympc_trajectory.csv (80 points)")
    print("\nThese datasets can now be used by the idiomatic control benchmarks!")

if __name__ == "__main__":
    main() 