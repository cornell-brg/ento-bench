#!/usr/bin/env python3
"""
Generate Template MPC trajectory dataset for benchmarking.

This script generates a helix trajectory dataset matching the format expected
by the Template MPC benchmark. The trajectory is based on the FlightTasks::helix
function from the test file.

Output format: 12-state vector [x,y,z,ox,oy,oz,vx,vy,vz,wx,wy,wz]
where ox,oy,oz is the orientation vector (thrust direction)
"""

import numpy as np
import os
import csv
import math

def helix_trajectory(t, initial_pos, traj_amp=80.0, traj_freq=1.0, dz=0.15, use_y=True):
    """
    Generate helix trajectory point at time t (in milliseconds)
    
    Args:
        t: Time in milliseconds
        initial_pos: Initial position [x, y, z] in mm
        traj_amp: Trajectory amplitude in mm
        traj_freq: Trajectory frequency in Hz
        dz: Climb rate in mm/ms
        use_y: Whether to use Y component for 3D helix
    
    Returns:
        Dictionary with position, velocity, orientation
    """
    traj_omg = 2.0 * math.pi * traj_freq * 1e-3  # to KHz, then to rad/ms
    
    # Position
    position = np.array(initial_pos, dtype=float)
    position[0] += traj_amp * math.sin(traj_omg * t)
    
    if use_y:
        position[1] += traj_amp * (1.0 - math.cos(traj_omg * t))
    
    if traj_amp > 1e-3:  # otherwise assume hover
        position[2] += dz * t
    
    # Velocity
    velocity = np.zeros(3)
    velocity[0] = traj_amp * traj_omg * math.cos(traj_omg * t)
    
    if use_y:
        velocity[1] = traj_amp * traj_omg * math.sin(traj_omg * t)
    
    if traj_amp > 1e-3:
        velocity[2] = dz
    
    # Orientation (upright for template MPC)
    orientation = np.array([0.0, 0.0, 1.0])  # VERTICAL
    
    return {
        'position': position,
        'velocity': velocity, 
        'orientation': orientation
    }

def generate_template_mpc_dataset(duration_ms=2000, dt_ms=5.0, 
                                 traj_amp=50.0, traj_freq=1.0, dz=0.15):
    """
    Generate template MPC trajectory dataset
    
    Args:
        duration_ms: Simulation duration in milliseconds
        dt_ms: Time step in milliseconds
        traj_amp: Helix amplitude in mm
        traj_freq: Helix frequency in Hz
        dz: Climb rate in mm/ms
    
    Returns:
        List of 12-element state vectors
    """
    initial_pos = [0.0, 0.0, 0.0]  # Start at origin
    trajectory = []
    
    num_steps = int(duration_ms / dt_ms)
    
    for i in range(num_steps):
        t = i * dt_ms
        
        # Generate trajectory point
        traj_point = helix_trajectory(t, initial_pos, traj_amp, traj_freq, dz, True)
        
        # Create 12-state vector: [x,y,z,ox,oy,oz,vx,vy,vz,wx,wy,wz]
        state_vector = np.zeros(12)
        
        # Position (mm)
        state_vector[0:3] = traj_point['position']
        
        # Orientation vector (thrust direction)
        state_vector[3:6] = traj_point['orientation']
        
        # Velocity (mm/ms)
        state_vector[6:9] = traj_point['velocity']
        
        # Angular velocity (rad/ms) - zero for this trajectory
        state_vector[9:12] = [0.0, 0.0, 0.0]
        
        trajectory.append(state_vector.tolist())
    
    return trajectory

def save_dataset(trajectory, filename, description="Template MPC helix trajectory"):
    """
    Save trajectory dataset to CSV file
    
    Args:
        trajectory: List of state vectors
        filename: Output filename
        description: Description for header
    """
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write header
        writer.writerow([f"# {description}"])
        writer.writerow([
            "# State format: x,y,z,ox,oy,oz,vx,vy,vz,wx,wy,wz"
        ])
        writer.writerow([
            "# Units: position(mm), orientation(unit vector), velocity(mm/ms), angular_velocity(rad/ms)"
        ])
        
        # Write data
        for state_vector in trajectory:
            # Format with 6 decimal places for precision
            line = [f"{x:.6f}" for x in state_vector]
            writer.writerow(line)
    
    print(f"Generated {filename} with {len(trajectory)} trajectory points")

def main():
    """Generate template MPC trajectory datasets"""
    print("Generating Template MPC trajectory datasets...")
    
    # Create datasets directory
    datasets_dir = "datasets/control"
    os.makedirs(datasets_dir, exist_ok=True)
    
    # Generate helix trajectory (2 seconds, 5ms timestep = 400 points)
    helix_traj = generate_template_mpc_dataset(
        duration_ms=2000,  # 2 seconds
        dt_ms=5.0,         # 5ms timestep (matching template MPC)
        traj_amp=50.0,     # 50mm amplitude
        traj_freq=1.0,     # 1 Hz frequency
        dz=0.15            # 0.15 mm/ms climb rate
    )
    
    # Save helix dataset
    save_dataset(helix_traj, 
                os.path.join(datasets_dir, "template_mpc_helix.csv"),
                "Template MPC helix trajectory (50mm amplitude, 1Hz, 2s duration)")
    
    # Generate hover trajectory for comparison
    hover_traj = generate_template_mpc_dataset(
        duration_ms=1000,  # 1 second
        dt_ms=5.0,         # 5ms timestep
        traj_amp=0.0,      # No amplitude = hover
        traj_freq=1.0,
        dz=0.0             # No climb = hover
    )
    
    # Save hover dataset
    save_dataset(hover_traj,
                os.path.join(datasets_dir, "template_mpc_hover.csv"), 
                "Template MPC hover trajectory (1s duration)")
    
    print("\n✅ Template MPC datasets generated successfully!")
    print(f"📁 Datasets saved to: {os.path.abspath(datasets_dir)}")
    print("\nDatasets created:")
    print(f"  - template_mpc_helix.csv ({len(helix_traj)} points)")
    print(f"  - template_mpc_hover.csv ({len(hover_traj)} points)")
    print("\nThese datasets can now be used by the Template MPC benchmark!")

if __name__ == "__main__":
    main() 