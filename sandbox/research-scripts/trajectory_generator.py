#!/usr/bin/env python3
"""
Trajectory Generator for ento-bench
==================================

Generates standard MAV trajectories for controller benchmarking:
- Hover
- Simple circle
- Figure-eight
- Helix
- Step responses
- Aggressive maneuvers

Output format: CSV compatible with ento-bench
State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw, yaw_rate, wx, wy, wz]
"""

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import List, Tuple, Dict, Optional
import json

class TrajectoryGenerator:
    """Generate standard MAV trajectories for benchmarking"""
    
    def __init__(self, dt: float = 0.01):
        self.dt = dt  # Time step
        self.state_size = 13  # ento-bench state vector size
    
    def generate_hover(self, duration: float = 5.0, position: Tuple[float, float, float] = (0, 0, 1)) -> List[List[float]]:
        """Generate hover trajectory"""
        print(f"Generating hover trajectory: {duration}s at {position}")
        
        num_points = int(duration / self.dt)
        trajectory = []
        
        x, y, z = position
        
        for i in range(num_points):
            # Constant position and zero velocities
            state = [x, y, z,  # position
                    0, 0, 0,   # velocity
                    0, 0, 0,   # roll, pitch, yaw
                    0,         # yaw_rate
                    0, 0, 0]   # angular velocities
            trajectory.append(state)
        
        return trajectory
    
    def generate_circle(self, duration: float = 10.0, radius: float = 1.0, 
                       center: Tuple[float, float, float] = (0, 0, 1),
                       angular_velocity: float = 0.5) -> List[List[float]]:
        """Generate circular trajectory"""
        print(f"Generating circle trajectory: r={radius}m, ω={angular_velocity}rad/s")
        
        num_points = int(duration / self.dt)
        trajectory = []
        
        cx, cy, cz = center
        
        for i in range(num_points):
            t = i * self.dt
            angle = angular_velocity * t
            
            # Position
            x = cx + radius * np.cos(angle)
            y = cy + radius * np.sin(angle)
            z = cz
            
            # Velocity
            vx = -radius * angular_velocity * np.sin(angle)
            vy = radius * angular_velocity * np.cos(angle)
            vz = 0
            
            # Orientation (tangent to circle)
            roll = 0
            pitch = 0
            yaw = angle + np.pi/2  # Tangent direction
            yaw_rate = angular_velocity
            
            # Angular velocities
            wx = 0
            wy = 0
            wz = yaw_rate
            
            state = [x, y, z, vx, vy, vz, roll, pitch, yaw, yaw_rate, wx, wy, wz]
            trajectory.append(state)
        
        return trajectory
    
    def generate_figure_eight(self, duration: float = 20.0, scale: float = 1.0,
                             center: Tuple[float, float, float] = (0, 0, 1),
                             frequency: float = 0.1) -> List[List[float]]:
        """Generate figure-eight trajectory"""
        print(f"Generating figure-eight trajectory: scale={scale}m, f={frequency}Hz")
        
        num_points = int(duration / self.dt)
        trajectory = []
        
        cx, cy, cz = center
        omega = 2 * np.pi * frequency
        
        for i in range(num_points):
            t = i * self.dt
            
            # Lemniscate (figure-eight) parametric equations
            # x = a * cos(t) / (1 + sin²(t))
            # y = a * sin(t) * cos(t) / (1 + sin²(t))
            
            sin_t = np.sin(omega * t)
            cos_t = np.cos(omega * t)
            denom = 1 + sin_t**2
            
            # Position
            x = cx + scale * cos_t / denom
            y = cy + scale * sin_t * cos_t / denom
            z = cz
            
            # Velocity (derivatives)
            # dx/dt = a * omega * [-sin(t) * (1 + sin²(t)) - cos(t) * 2*sin(t)*cos(t)] / (1 + sin²(t))²
            # dy/dt = a * omega * [cos²(t) - sin²(t)] / (1 + sin²(t)) - a * omega * sin(t)*cos(t) * 2*sin(t)*cos(t) / (1 + sin²(t))²
            
            denom_sq = denom**2
            vx = scale * omega * (-sin_t * denom - cos_t * 2 * sin_t * cos_t) / denom_sq
            vy = scale * omega * ((cos_t**2 - sin_t**2) * denom - sin_t * cos_t * 2 * sin_t * cos_t) / denom_sq
            vz = 0
            
            # Orientation (tangent to trajectory)
            if abs(vx) > 1e-6 or abs(vy) > 1e-6:
                yaw = np.arctan2(vy, vx)
            else:
                yaw = 0
            
            # Yaw rate (derivative of yaw)
            if i > 0:
                prev_yaw = trajectory[-1][8]
                yaw_rate = (yaw - prev_yaw) / self.dt
                # Handle angle wrapping
                if yaw_rate > np.pi:
                    yaw_rate -= 2 * np.pi
                elif yaw_rate < -np.pi:
                    yaw_rate += 2 * np.pi
            else:
                yaw_rate = 0
            
            roll = 0
            pitch = 0
            wx = 0
            wy = 0
            wz = yaw_rate
            
            state = [x, y, z, vx, vy, vz, roll, pitch, yaw, yaw_rate, wx, wy, wz]
            trajectory.append(state)
        
        return trajectory
    
    def generate_helix(self, duration: float = 15.0, radius: float = 1.0,
                      climb_rate: float = 0.2, frequency: float = 0.2,
                      start_height: float = 1.0) -> List[List[float]]:
        """Generate helical trajectory"""
        print(f"Generating helix trajectory: r={radius}m, climb={climb_rate}m/s, f={frequency}Hz")
        
        num_points = int(duration / self.dt)
        trajectory = []
        
        omega = 2 * np.pi * frequency
        
        for i in range(num_points):
            t = i * self.dt
            angle = omega * t
            
            # Position
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = start_height + climb_rate * t
            
            # Velocity
            vx = -radius * omega * np.sin(angle)
            vy = radius * omega * np.cos(angle)
            vz = climb_rate
            
            # Orientation
            roll = 0
            pitch = 0
            yaw = angle + np.pi/2  # Tangent to helix
            yaw_rate = omega
            
            # Angular velocities
            wx = 0
            wy = 0
            wz = yaw_rate
            
            state = [x, y, z, vx, vy, vz, roll, pitch, yaw, yaw_rate, wx, wy, wz]
            trajectory.append(state)
        
        return trajectory
    
    def generate_step_response(self, step_size: Tuple[float, float, float] = (1, 0, 0),
                              duration: float = 5.0, settle_time: float = 1.0) -> List[List[float]]:
        """Generate step response trajectory"""
        print(f"Generating step response: {step_size} over {duration}s")
        
        num_points = int(duration / self.dt)
        trajectory = []
        
        dx, dy, dz = step_size
        
        for i in range(num_points):
            t = i * self.dt
            
            if t < settle_time:
                # Smooth step using sigmoid
                progress = 1 / (1 + np.exp(-10 * (t / settle_time - 0.5)))
            else:
                progress = 1.0
            
            # Position
            x = dx * progress
            y = dy * progress
            z = 1.0 + dz * progress  # Start at hover height
            
            # Velocity (derivative of smooth step)
            if t < settle_time:
                sigmoid_val = 1 / (1 + np.exp(-10 * (t / settle_time - 0.5)))
                sigmoid_deriv = 10 * sigmoid_val * (1 - sigmoid_val) / settle_time
                vx = dx * sigmoid_deriv
                vy = dy * sigmoid_deriv
                vz = dz * sigmoid_deriv
            else:
                vx = vy = vz = 0
            
            # Orientation (face direction of motion)
            if abs(vx) > 1e-6 or abs(vy) > 1e-6:
                yaw = np.arctan2(vy, vx)
            else:
                yaw = 0
            
            roll = 0
            pitch = 0
            yaw_rate = 0
            wx = wy = wz = 0
            
            state = [x, y, z, vx, vy, vz, roll, pitch, yaw, yaw_rate, wx, wy, wz]
            trajectory.append(state)
        
        return trajectory
    
    def generate_aggressive_maneuver(self, duration: float = 8.0) -> List[List[float]]:
        """Generate aggressive maneuver for stress testing"""
        print(f"Generating aggressive maneuver: {duration}s")
        
        num_points = int(duration / self.dt)
        trajectory = []
        
        for i in range(num_points):
            t = i * self.dt
            
            # Combination of multiple frequencies for aggressive motion
            f1, f2, f3 = 0.5, 1.2, 0.8
            a1, a2, a3 = 1.0, 0.5, 0.3
            
            # Position
            x = a1 * np.sin(2 * np.pi * f1 * t) + a3 * np.cos(2 * np.pi * f3 * t)
            y = a2 * np.cos(2 * np.pi * f2 * t) + a3 * np.sin(2 * np.pi * f3 * t)
            z = 1.0 + 0.5 * np.sin(2 * np.pi * 0.3 * t)
            
            # Velocity (derivatives)
            vx = a1 * 2 * np.pi * f1 * np.cos(2 * np.pi * f1 * t) - a3 * 2 * np.pi * f3 * np.sin(2 * np.pi * f3 * t)
            vy = -a2 * 2 * np.pi * f2 * np.sin(2 * np.pi * f2 * t) + a3 * 2 * np.pi * f3 * np.cos(2 * np.pi * f3 * t)
            vz = 0.5 * 2 * np.pi * 0.3 * np.cos(2 * np.pi * 0.3 * t)
            
            # Orientation
            if abs(vx) > 1e-6 or abs(vy) > 1e-6:
                yaw = np.arctan2(vy, vx)
            else:
                yaw = 0
            
            # Add some roll/pitch for aggressive maneuver
            roll = 0.2 * np.sin(2 * np.pi * 1.5 * t)
            pitch = 0.15 * np.cos(2 * np.pi * 1.8 * t)
            
            # Yaw rate
            if i > 0:
                prev_yaw = trajectory[-1][8]
                yaw_rate = (yaw - prev_yaw) / self.dt
                # Handle angle wrapping
                if yaw_rate > np.pi:
                    yaw_rate -= 2 * np.pi
                elif yaw_rate < -np.pi:
                    yaw_rate += 2 * np.pi
            else:
                yaw_rate = 0
            
            # Angular velocities
            wx = 0.2 * 2 * np.pi * 1.5 * np.cos(2 * np.pi * 1.5 * t)
            wy = -0.15 * 2 * np.pi * 1.8 * np.sin(2 * np.pi * 1.8 * t)
            wz = yaw_rate
            
            state = [x, y, z, vx, vy, vz, roll, pitch, yaw, yaw_rate, wx, wy, wz]
            trajectory.append(state)
        
        return trajectory
    
    def save_trajectory(self, trajectory: List[List[float]], filename: str, 
                       description: str = "Generated trajectory"):
        """Save trajectory to CSV file"""
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow([f"# {description}"])
            writer.writerow([f"# State vector: x,y,z,vx,vy,vz,roll,pitch,yaw,yaw_rate,wx,wy,wz"])
            writer.writerow([f"# Time step: {self.dt}s"])
            writer.writerow([f"# Duration: {len(trajectory) * self.dt:.2f}s"])
            
            # Write data
            for state in trajectory:
                writer.writerow([f"{x:.10f}" for x in state])
        
        print(f"Saved {len(trajectory)} trajectory points to {filename}")
    
    def plot_trajectory(self, trajectory: List[List[float]], title: str = "Trajectory"):
        """Plot 3D trajectory for visualization"""
        trajectory = np.array(trajectory)
        
        fig = plt.figure(figsize=(12, 8))
        
        # 3D trajectory plot
        ax1 = fig.add_subplot(221, projection='3d')
        ax1.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2])
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title(f'{title} - 3D Path')
        
        # Velocity profile
        ax2 = fig.add_subplot(222)
        time = np.arange(len(trajectory)) * self.dt
        ax2.plot(time, trajectory[:, 3], label='vx')
        ax2.plot(time, trajectory[:, 4], label='vy')
        ax2.plot(time, trajectory[:, 5], label='vz')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Velocity Profile')
        ax2.legend()
        ax2.grid(True)
        
        # Orientation
        ax3 = fig.add_subplot(223)
        ax3.plot(time, np.degrees(trajectory[:, 6]), label='roll')
        ax3.plot(time, np.degrees(trajectory[:, 7]), label='pitch')
        ax3.plot(time, np.degrees(trajectory[:, 8]), label='yaw')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angle (deg)')
        ax3.set_title('Orientation')
        ax3.legend()
        ax3.grid(True)
        
        # Angular velocities
        ax4 = fig.add_subplot(224)
        ax4.plot(time, trajectory[:, 10], label='wx')
        ax4.plot(time, trajectory[:, 11], label='wy')
        ax4.plot(time, trajectory[:, 12], label='wz')
        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Angular velocity (rad/s)')
        ax4.set_title('Angular Velocities')
        ax4.legend()
        ax4.grid(True)
        
        plt.tight_layout()
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='Generate MAV trajectories for ento-bench')
    parser.add_argument('--type', choices=['hover', 'circle', 'figure8', 'helix', 'step', 'aggressive', 'all'],
                       default='all', help='Trajectory type to generate')
    parser.add_argument('--output-dir', default='data/trajectories', help='Output directory')
    parser.add_argument('--dt', type=float, default=0.01, help='Time step (seconds)')
    parser.add_argument('--plot', action='store_true', help='Plot trajectories')
    parser.add_argument('--duration', type=float, help='Override default duration')
    
    args = parser.parse_args()
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    generator = TrajectoryGenerator(dt=args.dt)
    
    trajectories_to_generate = []
    
    if args.type == 'all':
        trajectories_to_generate = [
            ('hover', 'hover_5s.csv', lambda: generator.generate_hover(duration=args.duration or 5.0)),
            ('circle', 'circle_slow.csv', lambda: generator.generate_circle(duration=args.duration or 10.0, angular_velocity=0.3)),
            ('circle', 'circle_fast.csv', lambda: generator.generate_circle(duration=args.duration or 8.0, angular_velocity=0.8)),
            ('figure8', 'figure8.csv', lambda: generator.generate_figure_eight(duration=args.duration or 20.0)),
            ('helix', 'helix_gentle.csv', lambda: generator.generate_helix(duration=args.duration or 15.0, frequency=0.1)),
            ('helix', 'helix_aggressive.csv', lambda: generator.generate_helix(duration=args.duration or 12.0, frequency=0.3, climb_rate=0.4)),
            ('step', 'step_x.csv', lambda: generator.generate_step_response(step_size=(2, 0, 0), duration=args.duration or 5.0)),
            ('step', 'step_xyz.csv', lambda: generator.generate_step_response(step_size=(1, 1, 0.5), duration=args.duration or 6.0)),
            ('aggressive', 'aggressive_maneuver.csv', lambda: generator.generate_aggressive_maneuver(duration=args.duration or 8.0))
        ]
    else:
        # Generate single trajectory type
        if args.type == 'hover':
            trajectories_to_generate = [('hover', 'hover.csv', lambda: generator.generate_hover(duration=args.duration or 5.0))]
        elif args.type == 'circle':
            trajectories_to_generate = [('circle', 'circle.csv', lambda: generator.generate_circle(duration=args.duration or 10.0))]
        elif args.type == 'figure8':
            trajectories_to_generate = [('figure8', 'figure8.csv', lambda: generator.generate_figure_eight(duration=args.duration or 20.0))]
        elif args.type == 'helix':
            trajectories_to_generate = [('helix', 'helix.csv', lambda: generator.generate_helix(duration=args.duration or 15.0))]
        elif args.type == 'step':
            trajectories_to_generate = [('step', 'step.csv', lambda: generator.generate_step_response(duration=args.duration or 5.0))]
        elif args.type == 'aggressive':
            trajectories_to_generate = [('aggressive', 'aggressive.csv', lambda: generator.generate_aggressive_maneuver(duration=args.duration or 8.0))]
    
    # Generate and save trajectories
    for traj_type, filename, generator_func in trajectories_to_generate:
        print(f"\n=== Generating {traj_type} trajectory ===")
        trajectory = generator_func()
        
        output_file = output_dir / filename
        generator.save_trajectory(trajectory, str(output_file), f"{traj_type.title()} trajectory for ento-bench")
        
        if args.plot:
            generator.plot_trajectory(trajectory, f"{traj_type.title()} Trajectory")
    
    print(f"\n✅ Generated trajectories saved to {output_dir}")
    print("\nTo use with ento-bench:")
    print("1. Copy trajectory files to your benchmark data directory")
    print("2. Run controller benchmarks with these standardized trajectories")
    print("3. Compare performance across different controllers")

if __name__ == '__main__':
    main() 