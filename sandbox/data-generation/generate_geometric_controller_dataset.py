#!/usr/bin/env python3
"""
Generate trajectory datasets for geometric controller benchmarking.

This script uses the FDCL Python implementation to generate reference trajectories
and simulated responses that can be used as datasets for benchmarking the 
geometric controller implementation in ento-bench.

The output format matches the CSV format expected by OptControlProblem:
State vector: [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
"""

import sys
import os
import numpy as np
import csv
from scipy.spatial.transform import Rotation
import argparse

# Add FDCL Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'external', 'uav_geometric_control', 'python'))

try:
    from scripts.uav_dynamics import UAVDynamics
    from scripts.geometric_controller import GeometricController
    from scripts.trajectory_generator import TrajectoryGenerator
except ImportError as e:
    print(f"Error importing FDCL modules: {e}")
    print("Make sure the FDCL Python implementation is available")
    sys.exit(1)

class DatasetGenerator:
    """Generate trajectory datasets using FDCL implementation"""
    
    def __init__(self, dt=0.01, mass=1.95, inertia=None):
        """
        Initialize dataset generator
        
        Args:
            dt: Time step (seconds)
            mass: Vehicle mass (kg) 
            inertia: Inertia matrix (3x3) or None for default
        """
        self.dt = dt
        
        # Vehicle parameters (matching QuadrotorTraits)
        self.mass = mass
        self.gravity = 9.81
        
        if inertia is None:
            # Default inertia for 1.95kg quadrotor (matching QuadrotorTraits)
            self.inertia = np.array([
                [0.02, 0.0, 0.0],
                [0.0, 0.02, 0.0], 
                [0.0, 0.0, 0.04]
            ])
        else:
            self.inertia = inertia
            
        # Initialize FDCL components
        self.uav = UAVDynamics(mass=self.mass, inertia=self.inertia, gravity=self.gravity)
        self.controller = GeometricController(self.uav)
        self.trajectory_gen = TrajectoryGenerator()
        
        # Set FDCL gains (matching our implementation)
        self.controller.set_gains(
            kx=np.array([16.0, 16.0, 16.0]),
            kv=np.array([13.0, 13.0, 13.0]),
            kR=np.array([1.6, 1.6, 0.60]),
            kW=np.array([0.40, 0.40, 0.10]),
            kIX=4.0,
            ki=0.01,
            kIR=0.015,
            kI=0.01,
            kyI=0.02,
            c1=1.0,
            c2=1.0,
            c3=1.0
        )
        
    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion [qw, qx, qy, qz]"""
        rot = Rotation.from_matrix(R)
        quat = rot.as_quat()  # Returns [qx, qy, qz, qw]
        return np.array([quat[3], quat[0], quat[1], quat[2]])  # Reorder to [qw, qx, qy, qz]
    
    def state_to_vector(self, state):
        """
        Convert FDCL state to benchmark state vector format
        
        Args:
            state: FDCL state dict with keys: x, v, R, W
            
        Returns:
            13-element state vector: [x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz]
        """
        pos = state['x'].flatten()
        vel = state['v'].flatten()
        quat = self.rotation_matrix_to_quaternion(state['R'])
        omega = state['W'].flatten()
        
        return np.concatenate([pos, vel, quat, omega])
    
    def generate_hover_dataset(self, duration=5.0, hover_pos=None):
        """
        Generate hover trajectory dataset
        
        Args:
            duration: Simulation duration (seconds)
            hover_pos: Hover position [x, y, z] or None for [0, 0, 1]
            
        Returns:
            List of state vectors
        """
        if hover_pos is None:
            hover_pos = np.array([0.0, 0.0, 1.0])
            
        # Initialize state at hover position
        state = {
            'x': hover_pos.reshape(3, 1),
            'v': np.zeros((3, 1)),
            'R': np.eye(3),
            'W': np.zeros((3, 1))
        }
        
        # Reset controller integral errors
        self.controller.reset_integral_errors()
        
        # Generate trajectory
        trajectory = []
        num_steps = int(duration / self.dt)
        
        for i in range(num_steps):
            t = i * self.dt
            
            # Hover reference (constant position)
            ref = {
                'x': hover_pos.reshape(3, 1),
                'v': np.zeros((3, 1)),
                'a': np.zeros((3, 1)),
                'j': np.zeros((3, 1)),
                's': np.zeros((3, 1)),
                'b1d': np.array([1, 0, 0]).reshape(3, 1),
                'b1d_dot': np.zeros((3, 1)),
                'b1d_ddot': np.zeros((3, 1))
            }
            
            # Compute control
            control = self.controller.compute_control(state, ref, self.dt)
            
            # Store current state
            state_vector = self.state_to_vector(state)
            trajectory.append(state_vector)
            
            # Simulate forward
            state = self.uav.simulate_step(state, control, self.dt)
            
        return trajectory
    
    def generate_helix_dataset(self, duration=10.0, radius=2.0, frequency=0.1, 
                              climb_rate=0.2, center=None):
        """
        Generate helix trajectory dataset
        
        Args:
            duration: Simulation duration (seconds)
            radius: Helix radius (meters)
            frequency: Helix frequency (Hz)
            climb_rate: Vertical climb rate (m/s)
            center: Helix center [x, y, z] or None for [0, 0, 1]
            
        Returns:
            List of state vectors
        """
        if center is None:
            center = np.array([0.0, 0.0, 1.0])
            
        # Initialize state at starting position
        start_pos = center + np.array([radius, 0.0, 0.0])
        state = {
            'x': start_pos.reshape(3, 1),
            'v': np.zeros((3, 1)),
            'R': np.eye(3),
            'W': np.zeros((3, 1))
        }
        
        # Reset controller integral errors
        self.controller.reset_integral_errors()
        
        # Generate trajectory
        trajectory = []
        num_steps = int(duration / self.dt)
        
        for i in range(num_steps):
            t = i * self.dt
            
            # Generate helix reference
            omega = 2 * np.pi * frequency
            
            # Position
            x_ref = center[0] + radius * np.cos(omega * t)
            y_ref = center[1] + radius * np.sin(omega * t)
            z_ref = center[2] + climb_rate * t
            
            # Velocity
            vx_ref = -radius * omega * np.sin(omega * t)
            vy_ref = radius * omega * np.cos(omega * t)
            vz_ref = climb_rate
            
            # Acceleration
            ax_ref = -radius * omega**2 * np.cos(omega * t)
            ay_ref = -radius * omega**2 * np.sin(omega * t)
            az_ref = 0.0
            
            # Jerk
            jx_ref = radius * omega**3 * np.sin(omega * t)
            jy_ref = -radius * omega**3 * np.cos(omega * t)
            jz_ref = 0.0
            
            # Snap
            sx_ref = radius * omega**4 * np.cos(omega * t)
            sy_ref = radius * omega**4 * np.sin(omega * t)
            sz_ref = 0.0
            
            ref = {
                'x': np.array([x_ref, y_ref, z_ref]).reshape(3, 1),
                'v': np.array([vx_ref, vy_ref, vz_ref]).reshape(3, 1),
                'a': np.array([ax_ref, ay_ref, az_ref]).reshape(3, 1),
                'j': np.array([jx_ref, jy_ref, jz_ref]).reshape(3, 1),
                's': np.array([sx_ref, sy_ref, sz_ref]).reshape(3, 1),
                'b1d': np.array([1, 0, 0]).reshape(3, 1),
                'b1d_dot': np.zeros((3, 1)),
                'b1d_ddot': np.zeros((3, 1))
            }
            
            # Compute control
            control = self.controller.compute_control(state, ref, self.dt)
            
            # Store current state
            state_vector = self.state_to_vector(state)
            trajectory.append(state_vector)
            
            # Simulate forward
            state = self.uav.simulate_step(state, control, self.dt)
            
        return trajectory
    
    def generate_figure8_dataset(self, duration=10.0, scale=1.0, frequency=0.2):
        """
        Generate figure-8 trajectory dataset
        
        Args:
            duration: Simulation duration (seconds)
            scale: Figure-8 scale factor
            frequency: Figure-8 frequency (Hz)
            
        Returns:
            List of state vectors
        """
        # Initialize state
        state = {
            'x': np.array([scale, 0.0, 1.0]).reshape(3, 1),
            'v': np.zeros((3, 1)),
            'R': np.eye(3),
            'W': np.zeros((3, 1))
        }
        
        # Reset controller integral errors
        self.controller.reset_integral_errors()
        
        # Generate trajectory
        trajectory = []
        num_steps = int(duration / self.dt)
        
        for i in range(num_steps):
            t = i * self.dt
            
            # Generate figure-8 reference
            omega = 2 * np.pi * frequency
            
            # Position (Lemniscate of Gerono)
            x_ref = scale * np.cos(omega * t)
            y_ref = scale * np.sin(omega * t) * np.cos(omega * t)
            z_ref = 1.0  # Constant altitude
            
            # Velocity
            vx_ref = -scale * omega * np.sin(omega * t)
            vy_ref = scale * omega * (np.cos(omega * t)**2 - np.sin(omega * t)**2)
            vz_ref = 0.0
            
            # Acceleration
            ax_ref = -scale * omega**2 * np.cos(omega * t)
            ay_ref = -4 * scale * omega**2 * np.sin(omega * t) * np.cos(omega * t)
            az_ref = 0.0
            
            # Higher derivatives (simplified)
            ref = {
                'x': np.array([x_ref, y_ref, z_ref]).reshape(3, 1),
                'v': np.array([vx_ref, vy_ref, vz_ref]).reshape(3, 1),
                'a': np.array([ax_ref, ay_ref, az_ref]).reshape(3, 1),
                'j': np.zeros((3, 1)),  # Simplified
                's': np.zeros((3, 1)),  # Simplified
                'b1d': np.array([1, 0, 0]).reshape(3, 1),
                'b1d_dot': np.zeros((3, 1)),
                'b1d_ddot': np.zeros((3, 1))
            }
            
            # Compute control
            control = self.controller.compute_control(state, ref, self.dt)
            
            # Store current state
            state_vector = self.state_to_vector(state)
            trajectory.append(state_vector)
            
            # Simulate forward
            state = self.uav.simulate_step(state, control, self.dt)
            
        return trajectory
    
    def save_dataset(self, trajectory, filename, description=""):
        """
        Save trajectory dataset to CSV file
        
        Args:
            trajectory: List of state vectors
            filename: Output filename
            description: Optional description for header
        """
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            if description:
                writer.writerow([f"# {description}"])
            writer.writerow([
                "# State trajectory (x, y, z, vx, vy, vz, qw, qx, qy, qz, wx, wy, wz)"
            ])
            
            # Write data
            for state_vector in trajectory:
                writer.writerow([f"{x:.10f}" for x in state_vector])
                
        print(f"Saved {len(trajectory)} trajectory points to {filename}")

def main():
    parser = argparse.ArgumentParser(description="Generate geometric controller datasets")
    parser.add_argument("--output-dir", default="datasets/geometric-control", 
                       help="Output directory for datasets")
    parser.add_argument("--dt", type=float, default=0.01, 
                       help="Time step (seconds)")
    parser.add_argument("--duration", type=float, default=10.0,
                       help="Simulation duration (seconds)")
    
    args = parser.parse_args()
    
    # Create generator
    generator = DatasetGenerator(dt=args.dt)
    
    print(f"Generating geometric controller datasets...")
    print(f"Time step: {args.dt}s, Duration: {args.duration}s")
    print(f"Output directory: {args.output_dir}")
    
    # Generate hover dataset
    print("\nGenerating hover dataset...")
    hover_trajectory = generator.generate_hover_dataset(duration=args.duration)
    generator.save_dataset(
        hover_trajectory, 
        os.path.join(args.output_dir, "hover.csv"),
        f"Hover trajectory at [0,0,1] for {args.duration}s"
    )
    
    # Generate helix dataset (conservative parameters)
    print("\nGenerating helix dataset...")
    helix_trajectory = generator.generate_helix_dataset(
        duration=args.duration,
        radius=2.0,      # Large radius for stability
        frequency=0.1,   # Slow frequency
        climb_rate=0.2   # Moderate climb rate
    )
    generator.save_dataset(
        helix_trajectory,
        os.path.join(args.output_dir, "helix.csv"), 
        f"Helix trajectory (r=2.0m, f=0.1Hz, climb=0.2m/s) for {args.duration}s"
    )
    
    # Generate figure-8 dataset
    print("\nGenerating figure-8 dataset...")
    fig8_trajectory = generator.generate_figure8_dataset(
        duration=args.duration,
        scale=1.0,
        frequency=0.2
    )
    generator.save_dataset(
        fig8_trajectory,
        os.path.join(args.output_dir, "figure8.csv"),
        f"Figure-8 trajectory (scale=1.0m, f=0.2Hz) for {args.duration}s"
    )
    
    # Generate aggressive helix for testing limits
    print("\nGenerating aggressive helix dataset...")
    aggressive_helix = generator.generate_helix_dataset(
        duration=5.0,    # Shorter duration
        radius=0.5,      # Smaller radius
        frequency=0.5,   # Higher frequency
        climb_rate=0.5   # Faster climb
    )
    generator.save_dataset(
        aggressive_helix,
        os.path.join(args.output_dir, "helix_aggressive.csv"),
        f"Aggressive helix trajectory (r=0.5m, f=0.5Hz, climb=0.5m/s) for 5.0s"
    )
    
    print(f"\nDataset generation complete!")
    print(f"Generated {len(os.listdir(args.output_dir))} datasets in {args.output_dir}")

if __name__ == "__main__":
    main() 