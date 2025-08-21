#!/usr/bin/env python3
"""
Plot helix trajectory output from template MPC controller by parsing debug output
"""

import numpy as np
import matplotlib.pyplot as plt
import subprocess
import sys
import os
import re

def run_helix_test_and_capture_output():
    """Run the helix test and capture debug output"""
    
    # Change to build directory
    build_dir = "/Users/derinozturk/research/repos/ento-bench/build/native"
    os.chdir(build_dir)
    
    print("Running helix trajectory test (test 8)...")
    
    # Run the helix test and capture output
    try:
        result = subprocess.run(
            ["./src/ento-control/bin/test_template_mpc_opt_control", "8"], 
            capture_output=True, 
            text=True,
            timeout=30
        )
        
        if result.returncode != 0:
            print(f"Test failed with return code {result.returncode}")
            print(f"stderr: {result.stderr}")
            return None
            
        print("Test completed successfully!")
        return result.stdout
        
    except subprocess.TimeoutExpired:
        print("Test timed out")
        return None
    except Exception as e:
        print(f"Error running test: {e}")
        return None

def parse_debug_output(output):
    """Parse the debug output to extract trajectory data"""
    
    if not output:
        return None
        
    lines = output.split('\n')
    data = []
    
    # First try to extract from "Step X: tracking_error=..., pos=[x,y,z]" lines (every 100 steps)
    step_data = []
    for line in lines:
        if "Step" in line and "pos=" in line and "tracking_error=" in line:
            step_match = re.search(r'Step (\d+):', line)
            pos_match = re.search(r'pos=\[([-\d.]+),([-\d.]+),([-\d.]+)\]', line)
            
            if step_match and pos_match:
                step_num = int(step_match.group(1))
                x, y, z = float(pos_match.group(1)), float(pos_match.group(2)), float(pos_match.group(3))
                time = step_num * 5.0  # 5ms timestep
                step_data.append([time, x, y, z])
    
    # Also extract from "simulate_forward output: pos=" lines (every step)
    forward_data = []
    step_counter = 0
    for line in lines:
        if "simulate_forward output: pos=" in line:
            pos_match = re.search(r'pos=\[([-\d.]+),([-\d.]+),([-\d.]+)\]', line)
            if pos_match:
                x, y, z = float(pos_match.group(1)), float(pos_match.group(2)), float(pos_match.group(3))
                time = step_counter * 5.0  # 5ms timestep
                forward_data.append([time, x, y, z])
                step_counter += 1
    
    # Use the dataset with more points
    if len(forward_data) > len(step_data):
        data = forward_data
        print(f"Extracted {len(data)} trajectory points from simulate_forward output")
    elif len(step_data) > 0:
        data = step_data
        print(f"Extracted {len(data)} trajectory points from step output")
    else:
        print("Could not parse any trajectory data from output")
        return None
    
    return np.array(data)

def generate_reference_trajectory(times):
    """Generate the reference helix trajectory for comparison"""
    
    # Parameters from the test (matching FlightTasks::helix with useY=true)
    traj_amp = 50.0  # mm (from test: 50.0f)
    traj_freq = 1.0  # Hz (from test: 1.0f)
    dz = 0.15  # mm/ms (from test: 0.15f)
    use_y = True  # from test: useY=true for 3D helix
    
    traj_omg = 2.0 * np.pi * traj_freq * 1e-3  # to rad/ms
    
    ref_data = []
    for t in times:
        x_ref = traj_amp * np.sin(traj_omg * t)
        y_ref = 0.0 if not use_y else traj_amp * (1.0 - np.cos(traj_omg * t))
        z_ref = dz * t
        ref_data.append([t, x_ref, y_ref, z_ref])
    
    return np.array(ref_data)

def plot_trajectory_data(actual_data, ref_data=None):
    """Plot the trajectory data"""
    
    if actual_data is None:
        print("No data to plot")
        return
        
    # Extract columns
    time = actual_data[:, 0]
    actual_x, actual_y, actual_z = actual_data[:, 1], actual_data[:, 2], actual_data[:, 3]
    
    # Generate reference if not provided
    if ref_data is None:
        ref_data = generate_reference_trajectory(time)
    
    ref_x, ref_y, ref_z = ref_data[:, 1], ref_data[:, 2], ref_data[:, 3]
    
    # Calculate tracking error
    tracking_error = np.sqrt((actual_x - ref_x)**2 + (actual_y - ref_y)**2 + (actual_z - ref_z)**2)
    
    # Create plots
    fig = plt.figure(figsize=(15, 10))
    
    # 3D trajectory plot
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot(ref_x, ref_y, ref_z, 'r--', linewidth=2, label='Reference')
    ax1.plot(actual_x, actual_y, actual_z, 'b-', linewidth=2, label='Actual')
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')
    ax1.set_title('3D Helix Trajectory')
    ax1.legend()
    ax1.grid(True)
    
    # X position over time
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(time, ref_x, 'r--', linewidth=2, label='Reference')
    ax2.plot(time, actual_x, 'b-', linewidth=2, label='Actual')
    ax2.set_xlabel('Time (ms)')
    ax2.set_ylabel('X Position (mm)')
    ax2.set_title('X Position vs Time')
    ax2.legend()
    ax2.grid(True)
    
    # Y position over time
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(time, ref_y, 'r--', linewidth=2, label='Reference')
    ax3.plot(time, actual_y, 'b-', linewidth=2, label='Actual')
    ax3.set_xlabel('Time (ms)')
    ax3.set_ylabel('Y Position (mm)')
    ax3.set_title('Y Position vs Time')
    ax3.legend()
    ax3.grid(True)
    
    # Z position over time
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(time, ref_z, 'r--', linewidth=2, label='Reference')
    ax4.plot(time, actual_z, 'b-', linewidth=2, label='Actual')
    ax4.set_xlabel('Time (ms)')
    ax4.set_ylabel('Z Position (mm)')
    ax4.set_title('Z Position vs Time')
    ax4.legend()
    ax4.grid(True)
    
    # Tracking error over time
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.plot(time, tracking_error, 'k-', linewidth=2)
    ax5.set_xlabel('Time (ms)')
    ax5.set_ylabel('Tracking Error (mm)')
    ax5.set_title('Tracking Error vs Time')
    ax5.grid(True)
    
    # Position components together
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.plot(time, actual_x, 'r-', linewidth=2, label='X')
    ax6.plot(time, actual_y, 'g-', linewidth=2, label='Y') 
    ax6.plot(time, actual_z, 'b-', linewidth=2, label='Z')
    ax6.set_xlabel('Time (ms)')
    ax6.set_ylabel('Position (mm)')
    ax6.set_title('All Position Components')
    ax6.legend()
    ax6.grid(True)
    
    plt.tight_layout()
    plt.savefig('helix_trajectory_plot.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Print statistics
    print(f"\nTrajectory Statistics:")
    print(f"  Simulation time: {time[-1]:.1f} ms")
    print(f"  Number of steps: {len(time)}")
    print(f"  Max tracking error: {np.max(tracking_error):.3f} mm")
    print(f"  Mean tracking error: {np.mean(tracking_error):.3f} mm")
    print(f"  Final tracking error: {tracking_error[-1]:.3f} mm")
    print(f"  Position range:")
    print(f"    X: [{np.min(actual_x):.1f}, {np.max(actual_x):.1f}] mm")
    print(f"    Y: [{np.min(actual_y):.1f}, {np.max(actual_y):.1f}] mm") 
    print(f"    Z: [{np.min(actual_z):.1f}, {np.max(actual_z):.1f}] mm")

if __name__ == "__main__":
    print("Capturing helix trajectory data from template MPC test...")
    
    # Run test and capture output
    output = run_helix_test_and_capture_output()
    
    if output:
        print("Parsing debug output...")
        actual_data = parse_debug_output(output)
        
        if actual_data is not None:
            print("Plotting trajectory...")
            plot_trajectory_data(actual_data)
            print("Plot saved as 'helix_trajectory_plot.png'")
        else:
            print("Failed to parse trajectory data")
            print("\nRaw output (first 2000 chars):")
            print(output[:2000])
    else:
        print("Failed to capture test output")
        sys.exit(1) 