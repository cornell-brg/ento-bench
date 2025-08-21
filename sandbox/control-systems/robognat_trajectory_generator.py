#!/usr/bin/env python3
"""
Generate trajectories using the actual robognat simulator and convert to RoboFly format

This script:
1. Uses the real robognat physics simulation with LQR control
2. Runs with zero wind (no wind compensation needed)
3. Generates waypoint-following or other dynamic trajectories
4. Converts the 6-state robognat output to 10-state RoboFly format
"""

import numpy as np
import sys
import os
from numpy import array, sin, cos, pi
import matplotlib.pyplot as plt

# Add the robognat simulator to path
script_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.dirname(script_dir)
robognat_path = os.path.join(repo_root, 'external', 'robognat_simulator')
sys.path.append(robognat_path)

# Change to script directory so robognat can find photo.mat (we copied it there)
original_cwd = os.getcwd()
os.chdir(script_dir)

try:
    # Import robognat simulator
    import robognat_clean as robognat
    from robognat_clean import (
        simulate, zero_wind, lqr_controller_a, lqr_controller_ao,
        estimator_dynamics_a, estimator_dynamics_ao,
        ideal_ao_sensor_model, ideal_a_sensor_model, dynamics_model, nowind_dynamics_model
    )
    print("Successfully imported robognat simulator")
    
    # Restore original working directory
    os.chdir(original_cwd)
    
except ImportError as e:
    os.chdir(original_cwd)  # Restore directory even on error
    print(f"Error importing robognat: {e}")
    print("Make sure robognat_clean.py is in external/robognat_simulator/")
    sys.exit(1)

def robognat_to_robofly_state(q_robognat):
    """
    Convert robognat state (6D, no wind) to RoboFly state (10D)
    
    Args:
        q_robognat: [thetay, omegay, x, vx, z, vz] (6 elements)
    
    Returns:
        q_robofly: [x, y, z, vx, vy, vz, roll, pitch, ωx, ωy] (10 elements)
    """
    if len(q_robognat) == 7:
        # Drop wind state if present
        q_robognat = q_robognat[:6]
    
    thetay, omegay, x, vx, z, vz = q_robognat
    
    # Map to RoboFly format
    q_robofly = array([
        x,        # x position
        0.0,      # y position (planar motion)
        z,        # z position (height)
        vx,       # x velocity
        0.0,      # y velocity (planar motion)
        vz,       # z velocity
        thetay,   # roll (pitch around y-axis)
        0.0,      # pitch (assume small for planar motion)
        0.0,      # ωx (assume small for planar motion)
        omegay    # ωy (angular velocity around y-axis)
    ])
    
    return q_robofly

def waypoint_controller(q, waypoints, current_waypoint_idx, time_at_waypoint, hover_time, p, t):
    """
    Simple waypoint following controller using robognat's LQR
    """
    # Extract current position
    current_pos = (q[2], q[4])  # (x, z)
    
    # Get the current waypoint target
    if current_waypoint_idx[0] >= len(waypoints):
        # Finished trajectory - hover at final position
        target_pos = waypoints[-1]
    else:
        target_pos = waypoints[current_waypoint_idx[0]]
        
        # Check if we've reached current waypoint
        distance = np.sqrt((current_pos[0] - target_pos[0])**2 + 
                          (current_pos[1] - target_pos[1])**2)
        
        distance_threshold = 0.02  # 2cm threshold
        
        if distance < distance_threshold:
            time_at_waypoint[0] += robognat.DT
            min_hover_time = max(0.5, hover_time * 0.5)  # At least 0.5s hover
            
            if time_at_waypoint[0] >= min_hover_time:
                # Move to next waypoint
                current_waypoint_idx[0] = min(current_waypoint_idx[0] + 1, len(waypoints) - 1)
                time_at_waypoint[0] = 0.0
                print(f"t={t:.1f}s: Moving to waypoint {current_waypoint_idx[0]}: {target_pos}")
        else:
            time_at_waypoint[0] = 0.0  # Reset timer if moving toward waypoint
    
    # Update desired position for LQR controller
    p.pos_desired = target_pos
    
    # Use standard robognat LQR controller (no wind compensation needed)
    return lqr_controller_a(q, p, t)

def generate_robognat_trajectory(trajectory_type="hover", duration=10.0, output_file=None):
    """
    Generate trajectory using robognat simulator
    
    Args:
        trajectory_type: "hover", "waypoints", "circle"
        duration: Simulation duration in seconds
        output_file: Output CSV file path (optional)
    
    Returns:
        time, q_data (robognat format), q_robofly_data (converted format), waypoints
    """
    print(f"Generating {trajectory_type} trajectory using robognat simulator...")
    
    # Set up robognat parameters (use default gnatbot parameters)
    p = robognat.p  # Default parameters from robognat
    
    # Define waypoints based on trajectory type
    if trajectory_type == "hover":
        waypoints = [(0.1, 0.15)]  # Single hover point
        hover_time = duration
        start_pos = (0.05, 0.1)  # Start away from target
        
    elif trajectory_type == "waypoints":
        # Square pattern
        waypoints = [
            (0.05, 0.1),   # Start
            (0.15, 0.1),   # Right
            (0.15, 0.2),   # Up
            (0.05, 0.2),   # Left
            (0.05, 0.1)    # Back to start
        ]
        hover_time = 1.0  # 1 second at each waypoint
        start_pos = (0.02, 0.08)  # Start away from first waypoint
        
    elif trajectory_type == "circle":
        # Generate circular waypoints - more points for smoother motion
        n_points = 16  # Increased from 8 to 16 for smoother circle
        radius = 0.05
        center = (0.1, 0.15)
        waypoints = []
        for i in range(n_points + 1):  # +1 to close the loop
            angle = 2 * pi * i / n_points
            x = center[0] + radius * cos(angle)
            z = center[1] + radius * sin(angle)
            waypoints.append((x, z))
        hover_time = 0.8  # Increased from 0.5 to 0.8 seconds at each waypoint
        start_pos = (center[0] - radius - 0.02, center[1] - 0.02)  # Start outside circle
        
    else:
        raise ValueError(f"Unknown trajectory type: {trajectory_type}")
    
    print(f"Waypoints: {waypoints}")
    print(f"Starting position: {start_pos}")
    
    # Set up waypoint following
    current_waypoint_idx = [0]  # Use list to make it mutable
    time_at_waypoint = [0.0]
    
    def trajectory_controller(q, p, t):
        return waypoint_controller(q, waypoints, current_waypoint_idx, 
                                 time_at_waypoint, hover_time, p, t)
    
    # Initial state (start away from target) - use 6 states for no-wind dynamics
    initial_state = array([0, 0, start_pos[0], 0, start_pos[1], 0])  # 6 states, no wind
    
    # Run robognat simulation with zero wind
    print(f"Running robognat simulation for {duration}s...")
    time, q_data, u_data, y_data, qhat_data = simulate(
        p=p,
        q=initial_state,
        tfinal=duration,
        sensor_model=ideal_a_sensor_model,  # Real accelerometer sensor model
        dynamics_model=nowind_dynamics_model,  # No wind dynamics
        controller=trajectory_controller,
        estimator_dynamics=estimator_dynamics_a,  # Real accelerometer estimator
        wind_function=zero_wind,  # No wind
        use_estimator=False,  # Use ground truth for control, but estimator still runs
        noisy=False,  # No sensor noise for clean trajectories
        perturb_qhat=False  # Start with perfect state estimate
    )
    
    print(f"Simulation complete: {len(time)} time steps")
    
    # Convert to RoboFly format
    print("Converting to RoboFly format...")
    q_robofly_data = np.zeros((len(time), 10))
    for i, q_robognat in enumerate(q_data):
        q_robofly_data[i] = robognat_to_robofly_state(q_robognat)
    
    # Save to file if requested
    if output_file:
        print(f"Saving trajectory to {output_file}")
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        
        header = f"# RoboFly trajectory generated from robognat simulator\n"
        header += f"# Trajectory type: {trajectory_type}\n"
        header += f"# Duration: {duration}s\n"
        header += "# State format: x,y,z,vx,vy,vz,roll,pitch,omega_x,omega_y"
        
        np.savetxt(output_file, q_robofly_data, delimiter=',', 
                   fmt='%.6f', header=header, comments='')
        
        print(f"Saved {len(q_robofly_data)} trajectory points")
    
    return time, q_data, q_robofly_data, waypoints

def plot_trajectory(time, q_data, q_robofly_data, trajectory_type, waypoints):
    """
    Plot the generated trajectory with 3D visualization and 2D plane cuts
    """
    fig = plt.figure(figsize=(16, 12))
    
    # Convert waypoints to arrays for easier plotting
    waypoint_x = [wp[0] for wp in waypoints]
    waypoint_z = [wp[1] for wp in waypoints]
    waypoint_y = [0.0] * len(waypoints)  # All waypoints at y=0 for planar motion
    
    # Create a 2x3 subplot layout
    # Top row: 3D plot, X-Z plane, X-Y plane  
    # Bottom row: Velocities, Attitude, Position vs time
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    # Plot actual trajectory
    ax1.plot(q_robofly_data[:, 0], q_robofly_data[:, 1], q_robofly_data[:, 2], 'b-', linewidth=2, label='Actual Trajectory')
    # Plot waypoints and desired path
    ax1.plot(waypoint_x, waypoint_y, waypoint_z, 'r--', linewidth=2, alpha=0.7, label='Desired Path')
    ax1.scatter(waypoint_x, waypoint_y, waypoint_z, color='red', s=80, label='Waypoints', marker='D', alpha=0.8)
    # Start and end points
    ax1.scatter(q_robofly_data[0, 0], q_robofly_data[0, 1], q_robofly_data[0, 2], 
                color='green', s=100, label='Start', marker='o')
    ax1.scatter(q_robofly_data[-1, 0], q_robofly_data[-1, 1], q_robofly_data[-1, 2], 
                color='orange', s=100, label='End', marker='s')
    ax1.set_xlabel('X position (m)')
    ax1.set_ylabel('Y position (m)')
    ax1.set_zlabel('Z position (m)')
    ax1.set_title(f'3D Trajectory - {trajectory_type}')
    ax1.legend()
    ax1.grid(True)
    
    # X-Z plane (side view) - This is the main one you wanted!
    ax2 = fig.add_subplot(2, 3, 2)
    # Plot actual trajectory
    ax2.plot(q_robofly_data[:, 0], q_robofly_data[:, 2], 'b-', linewidth=2, label='Actual Trajectory')
    # Plot desired path (connect waypoints)
    ax2.plot(waypoint_x, waypoint_z, 'r--', linewidth=2, alpha=0.7, label='Desired Path')
    # Plot waypoints as distinct markers
    ax2.scatter(waypoint_x, waypoint_z, color='red', s=80, label='Waypoints', marker='D', alpha=0.8, zorder=5)
    # Start and end points
    ax2.plot(q_robofly_data[0, 0], q_robofly_data[0, 2], 'go', markersize=10, label='Start', zorder=6)
    ax2.plot(q_robofly_data[-1, 0], q_robofly_data[-1, 2], 'o', color='orange', markersize=10, label='End', zorder=6)
    
    # Add waypoint labels
    for i, (x, z) in enumerate(waypoints):
        ax2.annotate(f'WP{i}', (x, z), xytext=(5, 5), textcoords='offset points', 
                    fontsize=8, alpha=0.8, color='darkred')
    
    ax2.set_xlabel('X position (m)')
    ax2.set_ylabel('Z position (m)')
    ax2.set_title('X-Z Plane (Side View) - Waypoint Following')
    ax2.grid(True)
    ax2.legend()
    ax2.axis('equal')
    
    # X-Y plane (top view)
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(q_robofly_data[:, 0], q_robofly_data[:, 1], 'b-', linewidth=2, label='Actual Trajectory')
    ax3.plot(waypoint_x, waypoint_y, 'r--', linewidth=2, alpha=0.7, label='Desired Path')
    ax3.scatter(waypoint_x, waypoint_y, color='red', s=80, label='Waypoints', marker='D', alpha=0.8)
    ax3.plot(q_robofly_data[0, 0], q_robofly_data[0, 1], 'go', markersize=10, label='Start')
    ax3.plot(q_robofly_data[-1, 0], q_robofly_data[-1, 1], 'o', color='orange', markersize=10, label='End')
    ax3.set_xlabel('X position (m)')
    ax3.set_ylabel('Y position (m)')
    ax3.set_title('X-Y Plane (Top View)')
    ax3.grid(True)
    ax3.legend()
    ax3.axis('equal')
    
    # Velocity plot
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(time, q_robofly_data[:, 3], 'r-', label='vx', linewidth=2)
    ax4.plot(time, q_robofly_data[:, 4], 'g-', label='vy', linewidth=2)
    ax4.plot(time, q_robofly_data[:, 5], 'b-', label='vz', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Velocity (m/s)')
    ax4.set_title('Velocities')
    ax4.grid(True)
    ax4.legend()
    
    # Attitude plot (angles and angular velocities)
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(time, np.rad2deg(q_robofly_data[:, 6]), 'g-', label='Roll (deg)', linewidth=2)
    ax5.plot(time, np.rad2deg(q_robofly_data[:, 7]), 'r-', label='Pitch (deg)', linewidth=2)
    ax5_twin = ax5.twinx()
    ax5_twin.plot(time, np.rad2deg(q_robofly_data[:, 8]), 'm--', label='ωx (deg/s)', linewidth=2)
    ax5_twin.plot(time, np.rad2deg(q_robofly_data[:, 9]), 'c--', label='ωy (deg/s)', linewidth=2)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Angle (deg)', color='black')
    ax5_twin.set_ylabel('Angular velocity (deg/s)', color='purple')
    ax5.set_title('Attitude & Angular Rates')
    ax5.grid(True)
    ax5.legend(loc='upper left')
    ax5_twin.legend(loc='upper right')
    
    # Position vs time
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(time, q_robofly_data[:, 0], 'r-', label='x', linewidth=2)
    ax6.plot(time, q_robofly_data[:, 1], 'g-', label='y', linewidth=2)
    ax6.plot(time, q_robofly_data[:, 2], 'b-', label='z', linewidth=2)
    
    # Add horizontal lines for waypoint positions
    for i, (x, z) in enumerate(waypoints):
        ax6.axhline(y=x, color='red', linestyle=':', alpha=0.5, linewidth=1)
        ax6.axhline(y=z, color='blue', linestyle=':', alpha=0.5, linewidth=1)
    
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Position (m)')
    ax6.set_title('Position vs Time (with waypoint levels)')
    ax6.grid(True)
    ax6.legend()
    
    plt.tight_layout()
    
    # Add overall title
    fig.suptitle(f'Robognat {trajectory_type.title()} Trajectory Analysis', 
                 fontsize=16, y=0.98)
    
    return fig

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage:")
        print("  python robognat_trajectory_generator.py <type> <output.csv> [duration]")
        print("  where type = hover|waypoints|circle")
        print("  duration = simulation time in seconds (default: 10.0)")
        print("")
        print("Examples:")
        print("  python robognat_trajectory_generator.py hover datasets/control/robofly_hover.csv")
        print("  python robognat_trajectory_generator.py waypoints datasets/control/robofly_square.csv 20")
        print("  python robognat_trajectory_generator.py circle datasets/control/robofly_circle.csv 15")
        sys.exit(1)
    
    trajectory_type = sys.argv[1]
    output_file = sys.argv[2]
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
    
    if trajectory_type not in ["hover", "waypoints", "circle"]:
        print(f"Error: Unknown trajectory type '{trajectory_type}'")
        print("Supported types: hover, waypoints, circle")
        sys.exit(1)
    
    try:
        # Generate trajectory
        time, q_data, q_robofly_data, waypoints = generate_robognat_trajectory(
            trajectory_type=trajectory_type,
            duration=duration,
            output_file=output_file
        )
        
        # Create plot
        fig = plot_trajectory(time, q_data, q_robofly_data, trajectory_type, waypoints)
        
        # Save plot
        plot_file = output_file.replace('.csv', '_plot.png')
        # Move plots to benchmark_results directory
        if 'datasets/control/' in plot_file:
            plot_file = plot_file.replace('datasets/control/', 'benchmark_results/control/trajectory_plots/')
            os.makedirs(os.path.dirname(plot_file), exist_ok=True)
        
        fig.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"Saved trajectory plot to {plot_file}")
        
        print("Trajectory generation successful!")
        
    except Exception as e:
        print(f"Error generating trajectory: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1) 