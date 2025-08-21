#!/usr/bin/env python3
"""
Convert FDCL JSON trajectory data to CSV format for ento-bench.

This script converts the existing FDCL debug JSON files to the CSV format
expected by our GeometricControllerProblem.

The output format is: [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
"""

import json
import csv
import numpy as np
from scipy.spatial.transform import Rotation
import argparse
import os

def rotation_matrix_to_quaternion(R):
    """Convert rotation matrix to quaternion [qw, qx, qy, qz]"""
    rot = Rotation.from_matrix(R)
    quat = rot.as_quat()  # Returns [qx, qy, qz, qw]
    return np.array([quat[3], quat[0], quat[1], quat[2]])  # Reorder to [qw, qx, qy, qz]

def convert_json_to_csv(json_file, csv_file, description=""):
    """
    Convert FDCL JSON trajectory to CSV format
    
    Args:
        json_file: Input JSON file path
        csv_file: Output CSV file path
        description: Optional description for header
    """
    
    # Load JSON data
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    # Create output directory if needed
    os.makedirs(os.path.dirname(csv_file), exist_ok=True)
    
    # Convert to CSV
    with open(csv_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write header
        if description:
            writer.writerow([f"# {description}"])
        writer.writerow([
            "# State trajectory (x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz)"
        ])
        
        # Process each time step
        for step in data:
            state = step['state']
            
            # Extract position and velocity
            pos = state['position']
            vel = state['velocity']
            
            # Convert rotation matrix to quaternion
            R = np.array(state['rotation'])
            quat = rotation_matrix_to_quaternion(R)
            
            # Extract angular velocity
            omega = state['angular_velocity']
            
            # Create state vector: [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
            state_vector = pos + vel + quat.tolist() + omega
            
            # Write to CSV
            writer.writerow([f"{x:.10f}" for x in state_vector])
    
    print(f"Converted {len(data)} trajectory points from {json_file} to {csv_file}")

def main():
    parser = argparse.ArgumentParser(description="Convert FDCL JSON to CSV")
    parser.add_argument("--input", required=True, help="Input JSON file")
    parser.add_argument("--output", required=True, help="Output CSV file")
    parser.add_argument("--description", default="", help="Description for header")
    
    args = parser.parse_args()
    
    convert_json_to_csv(args.input, args.output, args.description)

if __name__ == "__main__":
    main() 