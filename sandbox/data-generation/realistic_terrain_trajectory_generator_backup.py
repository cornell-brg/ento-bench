#!/usr/bin/env python3
"""
Realistic Terrain-Crossing Trajectory Generator for EKF Testing

This generator uses the original robognat physics and LQR controller but modifies
the position setpoints to create trajectories that cross terrain features properly.
This ensures we test the EKF's response to real terrain variations.
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from numpy import array, sin, cos, pi
from numpy.random import normal
import csv
import argparse

# Change to robognat directory to import modules
original_dir = os.getcwd()
robognat_dir = os.path.abspath('../external/robognat_simulator')

# Add robognat directory to Python path
sys.path.insert(0, robognat_dir)

# Change to robognat_data directory where the data files are located
data_dir = os.path.join(original_dir, 'robognat_data')
os.chdir(data_dir)

# Import the original robognat functions
from robognat_clean import *

# Change back to original directory
os.chdir(original_dir)

# Import our terrain functions - use direct import instead of from another module
# since there may be path conflicts
def generate_ground_height(x, z, terrain_type="flat", terrain_params=None):
    """
    Generate ground height at position (x, z) for different terrain types
    (copied from robognat_robofly_validation.py to avoid import conflicts)
    """
    if terrain_params is None:
        terrain_params = {}
    
    if terrain_type == "flat":
        return 0.0
        
    elif terrain_type == "gradual_slope":
        slope_x = terrain_params.get('slope_x', 0.02)  # 2cm/m = 1.15° slope
        slope_z = terrain_params.get('slope_z', 0.01)  # 1cm/m = 0.57° slope
        return slope_x * x + slope_z * z
        
    elif terrain_type == "steep_slope":
        slope_x = terrain_params.get('slope_x', 0.1)   # 10cm/m = 5.7° slope
        slope_z = terrain_params.get('slope_z', 0.05)  # 5cm/m = 2.9° slope
        return slope_x * x + slope_z * z
        
    elif terrain_type == "sinusoidal":
        slope_amplitude = terrain_params.get('amplitude', 0.02)  # 2cm height variations
        freq_x = terrain_params.get('freq_x', 0.5)  # cycles per meter in x
        freq_z = terrain_params.get('freq_z', 0.3)  # cycles per meter in z
        roughness_amp = terrain_params.get('roughness', 0.005)  # 5mm roughness
        
        ground_height = (slope_amplitude * sin(2*pi*freq_x * x) * 
                        cos(2*pi*freq_z * z))
        
        # Add fine roughness
        roughness = roughness_amp * sin(10*x) * cos(8*z)
        return ground_height + roughness
        
    elif terrain_type == "step_terrain":
        step_height = terrain_params.get('step_height', 0.02)  # 2cm steps
        step_size_x = terrain_params.get('step_size_x', 0.5)   # 50cm step length
        
        # Create discrete steps
        step_number = int(x / step_size_x)
        return step_number * step_height
        
    else:
        return 0.0  # Default flat

def get_terrain_slope(x, z, terrain_type, terrain_params=None, delta=0.001):
    """Get terrain slope at position (x, z) using numerical differentiation"""
    if terrain_params is None:
        terrain_params = {}
        
    h_center = generate_ground_height(x, z, terrain_type, terrain_params)
    h_x_plus = generate_ground_height(x + delta, z, terrain_type, terrain_params)
    h_z_plus = generate_ground_height(x, z + delta, terrain_type, terrain_params)
    
    slope_x = (h_x_plus - h_center) / delta
    slope_z = (h_z_plus - h_center) / delta
    
    return slope_x, slope_z

def robofly_sensor_model_from_state(q, noisy=True, terrain_type="flat", terrain_params=None):
    """
    Generate RoboFly sensor measurements from robognat state
    (copied from robognat_robofly_validation.py to avoid import conflicts)
    """
    thetay = q[0]
    omegay = q[1] 
    x = q[2]  # x position
    v_world = q[[3, 5]]  # [vx, vz]
    z = q[4]  # height above nominal ground
    R2 = R(thetay)
    v_body = R2.T @ v_world
    
    cos_theta = cos(thetay)
    sin_theta = sin(thetay)
    
    # Ground height at current x,z position
    ground_height = generate_ground_height(x, z, terrain_type, terrain_params)
    actual_height_above_ground = z - ground_height
    
    # RoboFly sensors with ground terrain effects
    if abs(cos_theta) > 0.01:
        # ToF range measurement: distance to ground (affected by ground topology)
        range_sensor = actual_height_above_ground / cos_theta
    else:
        range_sensor = 100.0  # Sensor saturation when nearly horizontal
    
    # Optical flow: assumes flat ground in the EKF model, but reality may differ
    if terrain_type != "flat":
        # Ground slopes affect apparent optical flow due to varying ground distance
        ground_slope_x, ground_slope_z = get_terrain_slope(x, z, terrain_type, terrain_params)
        flow_correction = (ground_slope_x * v_world[0] + ground_slope_z * v_world[1]) / (actual_height_above_ground + 0.01)  # slope effect on flow
        optical_flow = omegay - v_body[0] * cos_theta / actual_height_above_ground + flow_correction
    else:
        optical_flow = omegay - v_body[0] * cos_theta / z  # Standard formula for flat ground
    
    # Accelerometers measure body-frame acceleration (unaffected by ground topology)
    accel_x = -g * sin_theta  # Body-frame accelerometer
    accel_z = g * cos_theta   # Body-frame accelerometer
    
    # Add realistic noise levels
    if noisy:
        range_sensor += normal(0, 0.001)      # 1mm range noise
        optical_flow += normal(0, 0.05)       # 0.05 rad/s flow noise  
        accel_x += normal(0, 0.1)             # 0.1 m/s^2 accel noise
        accel_z += normal(0, 0.1)             # 0.1 m/s^2 accel noise
    
    return array([range_sensor, optical_flow, accel_x, accel_z])

# Constants (from original robognat)
DT = 1./200  # 200 Hz sampling
g = 9.81

def create_terrain_crossing_waypoints(terrain_type, terrain_params, trajectory_length=2.0, n_waypoints=5):
    """
    Create waypoint trajectories that specifically cross terrain features
    
    Args:
        terrain_type: Type of terrain to test
        terrain_params: Parameters for terrain generation
        trajectory_length: Total distance to traverse (meters)
        n_waypoints: Number of waypoints in trajectory
    
    Returns:
        waypoints: List of (x, z) waypoint coordinates
        hover_time: Time to hover at each waypoint (seconds)
    """
    waypoints = []
    hover_time = 1.0  # Reduce hover time for faster traversal (was 2.0)
    
    if terrain_type == "flat":
        # Simple forward trajectory for baseline
        x_positions = np.linspace(0.0, trajectory_length, n_waypoints)
        z_target = 0.15  # 15cm hover height
        waypoints = [(x, z_target) for x in x_positions]
        
    elif terrain_type in ["gradual_slope", "steep_slope"]:
        # Cross the slope: start at origin, traverse forward
        x_positions = np.linspace(0.0, trajectory_length, n_waypoints)
        z_base = 0.15  # Base hover height
        
        # Add extra height to compensate for ground slope and maintain clearance
        for x in x_positions:
            ground_h = generate_ground_height(x, 0, terrain_type, terrain_params)
            z_hover = z_base + ground_h + 0.05  # Maintain ~5cm above actual ground
            waypoints.append((x, z_hover))
            
    elif terrain_type == "step_terrain":
        # Traverse across step boundaries - this is critical for testing
        step_size_x = terrain_params.get('step_size_x', 0.5)
        
        # Create waypoints that specifically cross step boundaries
        x_positions = np.linspace(0.0, trajectory_length, n_waypoints)
        z_base = 0.15
        
        for x in x_positions:
            ground_h = generate_ground_height(x, 0, terrain_type, terrain_params)
            z_hover = z_base + ground_h + 0.08  # Stay well above the steps
            waypoints.append((x, z_hover))
            
    elif terrain_type in ["sinusoidal", "random_smooth", "random_rough"]:
        # Follow the terrain profile
        x_positions = np.linspace(0.0, trajectory_length, n_waypoints)
        z_base = 0.15
        
        for x in x_positions:
            ground_h = generate_ground_height(x, 0, terrain_type, terrain_params)
            z_hover = z_base + ground_h + 0.05  # Close to ground for these terrains
            waypoints.append((x, z_hover))
            
    elif terrain_type == "indoor_bumps":
        # Forward trajectory with height variation
        x_positions = np.linspace(0.0, trajectory_length, n_waypoints)
        z_base = 0.12
        waypoints = [(x, z_base + 0.03 * np.sin(2*np.pi*x/trajectory_length)) for x in x_positions]
        
    # Ensure we have the right number of waypoints
    while len(waypoints) < n_waypoints:
        waypoints.append(waypoints[-1])
    waypoints = waypoints[:n_waypoints]
    
    # Add some debug info
    print(f"Generated waypoints for {trajectory_length}m trajectory:")
    for i, wp in enumerate(waypoints):
        print(f"  WP{i}: ({wp[0]:.3f}, {wp[1]:.3f})")
    
    return waypoints, hover_time

def waypoint_following_controller(q, waypoints, current_waypoint_idx, time_at_waypoint, hover_time, p, t):
    """
    LQR controller modified for waypoint following
    Uses the same controller structure as the original robognat lqr_controller_ao
    """
    # Constants
    g = 9.81  # Define g at the start
    
    # Extract current position
    current_pos = (q[2], q[4])  # (x, z)
    
    # Get the current waypoint target
    if current_waypoint_idx >= len(waypoints):
        # Finished trajectory - hover at final position
        target_pos = waypoints[-1]
    else:
        target_pos = waypoints[current_waypoint_idx]
        
        # Check if we've reached current waypoint
        distance = np.sqrt((current_pos[0] - target_pos[0])**2 + 
                          (current_pos[1] - target_pos[1])**2)
        
        # More aggressive waypoint switching
        distance_threshold = 0.05  # 5cm threshold (was 3cm, but be more forgiving)
        
        if distance < distance_threshold:
            time_at_waypoint[0] += DT
            # Reduced hover time for faster transitions
            min_hover_time = max(0.5, hover_time * 0.3)  # At least 0.5s, but reduce requested hover time
            
            if time_at_waypoint[0] >= min_hover_time:
                # Move to next waypoint
                current_waypoint_idx = min(current_waypoint_idx + 1, len(waypoints) - 1)
                time_at_waypoint[0] = 0.0
                
                # Debug output every 5 seconds
                if t % 5.0 < DT:
                    print(f"t={t:.1f}s: At ({current_pos[0]:.3f}, {current_pos[1]:.3f}), "
                          f"Moving to WP{current_waypoint_idx}: {target_pos} "
                          f"(distance: {distance:.3f}m)")
        else:
            time_at_waypoint[0] = 0.0  # Reset timer if moving toward waypoint
            
            # Debug output for large distances (potential issues)
            if distance > 0.3 and t % 10.0 < DT:  # Every 10 seconds
                print(f"t={t:.1f}s: Far from WP{current_waypoint_idx} ({distance:.3f}m) "
                      f"Current: ({current_pos[0]:.3f}, {current_pos[1]:.3f}), "
                      f"Target: {target_pos}")
    
    # Use EXACTLY the same controller structure as robognat lqr_controller_ao
    # but change the desired position dynamically
    
    # Set desired state (7-state version with wind, like original robognat)
    q_desired = array((0, 0, target_pos[0], 0, target_pos[1], 0, 0))
    
    # Wind compensation (same as original robognat)
    vwx_estimate = q[6]  # wind estimate from state
    delta_thetay = -p.b * vwx_estimate/(p.m * g)
    delta_alphay = -p.d_z * p.b * vwx_estimate / p.J
    
    q_desired += (delta_thetay, 0, 0, 0, 0, 0, 0)
    delta_u = (0, delta_alphay)
    
    # Apply LQR control law with original robognat gains
    # We need to use K_wind from the original robognat
    if not hasattr(waypoint_following_controller, 'K_wind'):
        # These are the K_wind gains from the original robognat
        # K_wind = np.hstack((K, np.zeros((2,1)))) where K is the 6-state LQR gain
        
        # First compute the 6-state LQR gains like original robognat
        import control as ct
        
        # Physical parameters (same as robognat)
        m = p.m
        l = 0.004
        J = p.J  
        b = p.b
        d_z = p.d_z
        c = p.c
        
        # 6-state system matrices (without wind)
        A_6 = array([[0,                1,    0,              0, 0,        0], # thetay
                     [0,         -c/J,    0, -b*d_z/J, 0,        0], # omegay
                     [0,                0,    0,              1, 0,        0], # x
                     [g,   -b*d_z/m,    0,       -b/m, 0,        0], # vx
                     [0,                0,    0,              0, 0,        1], # z
                     [0,                0,    0,              0, 0, -b/m]])# vz
        
        B_6 = array([[0,     0], 
                     [0,     1], 
                     [0,     0],
                     [0,     0], 
                     [0,     0],
                     [1,     0]])
        
        # LQR weights - Use higher position gains for better waypoint tracking
        # Original robognat: QQ = np.diag([10, 10, 100, 10, 100, 10])  # For hovering
        # Modified for waypoint following: increase position gains significantly
        QQ = np.diag([10, 10, 10000, 100, 10000, 100])  # Higher x,z position gains for tracking
        RR = np.diag([1, 1e-3])  # control cost [a_z, alpha_y] - keep original
        
        # Compute 6-state LQR gains
        K_6, _, _ = ct.lqr(A_6, B_6, QQ, RR)
        
        # Extend to 7-state with wind (same as robognat K_wind)
        waypoint_following_controller.K_wind = np.hstack((K_6, np.zeros((2,1))))
        print("Computed LQR gains for waypoint following controller")
        print(f"K_wind shape: {waypoint_following_controller.K_wind.shape}")
        print(f"K_wind gains:")
        print(waypoint_following_controller.K_wind)
    
    K_wind = waypoint_following_controller.K_wind
    
    # Compute control (exactly like original robognat)
    q_error = q_desired - q
    u = K_wind @ q_error + delta_u
    
    # Limit control authority (same as original robognat)
    u[0] = np.clip(u[0], -0.5*g, 0.5*g)  # z acceleration limit
    u[1] = np.clip(u[1], -10, 10)        # angular acceleration limit
    
    return u, current_waypoint_idx

def generate_terrain_crossing_trajectory(terrain_type="flat", terrain_params=None, 
                                       trajectory_length=2.0, tfinal=30.0,
                                       n_waypoints=8, save_csv=True, use_ground_truth_control=True,
                                       sensor_rates=None):
    """
    Generate realistic physics-based trajectory that crosses terrain features
    
    Args:
        terrain_type: Type of terrain to test
        terrain_params: Terrain parameters
        trajectory_length: Spatial distance to cover (meters)
        tfinal: Total simulation time (seconds)
        n_waypoints: Number of waypoints in trajectory
        save_csv: Whether to save CSV file
        use_ground_truth_control: If True, controller uses ground truth state (eliminates tracking error)
        sensor_rates: Dict of sensor update rates (Hz) e.g., {'range': 100, 'flow': 100, 'accel': 200}
                     If None, uses synchronous 200Hz for all sensors
    
    Returns:
        time, q_data, robofly_measurements, waypoints
    """
    print(f"Generating terrain-crossing trajectory:")
    print(f"  Terrain: {terrain_type}")
    print(f"  Spatial coverage: {trajectory_length:.1f}m")
    print(f"  Duration: {tfinal:.1f}s")
    print(f"  Waypoints: {n_waypoints}")
    print(f"  Ground truth control: {use_ground_truth_control}")
    
    # Default synchronous sensor rates (200Hz)
    if sensor_rates is None:
        sensor_rates = {'range': 200, 'flow': 200, 'accel': 200}
    print(f"  Sensor rates: {sensor_rates}")
    
    # Create waypoints that cross terrain features
    waypoints, hover_time = create_terrain_crossing_waypoints(
        terrain_type, terrain_params, trajectory_length, n_waypoints)
    
    print(f"  Waypoints: {waypoints}")
    
    # Initialize controller state
    current_waypoint_idx = 0
    time_at_waypoint = [0.0]  # Use list to make it mutable
    
    # Create controller function that maintains waypoint state
    def terrain_crossing_controller(q, p, t):
        nonlocal current_waypoint_idx
        u, current_waypoint_idx = waypoint_following_controller(
            q, waypoints, current_waypoint_idx, time_at_waypoint, hover_time, p, t)
        return u
    
    # Run simulation using original robognat physics
    print("Running physics simulation...")
    
    # Starting position at first waypoint
    initial_state = array([0, 0, waypoints[0][0], 0, waypoints[0][1], 0, 0])  # 7 states including wind
    
    time, q_data, u_data, y_data, qhat_data = simulate(
        p=p,
        q=initial_state,
        tfinal=tfinal,
        sensor_model=ideal_ao_sensor_model,  # Original robognat sensors
        dynamics_model=dynamics_model,       # Original robognat physics  
        controller=terrain_crossing_controller,  # Our waypoint controller
        estimator_dynamics=estimator_dynamics_ao,  # Original estimator
        wind_function=zero_wind,
        use_estimator=not use_ground_truth_control,  # Use ground truth if requested
        noisy=True,
        perturb_qhat=True
    )
    
    print(f"Simulation complete: {len(time)} samples")
    
    # Analyze trajectory coverage
    x_positions = q_data[:, 2]
    z_positions = q_data[:, 4]
    x_range = np.max(x_positions) - np.min(x_positions)
    z_range = np.max(z_positions) - np.min(z_positions)
    
    print(f"Trajectory coverage:")
    print(f"  X range: {x_range:.3f}m ({np.min(x_positions):.3f} to {np.max(x_positions):.3f})")
    print(f"  Z range: {z_range*1000:.1f}mm ({np.min(z_positions):.3f} to {np.max(z_positions):.3f})")
    
    # Generate RoboFly sensor measurements with terrain effects at different rates
    print("Generating asynchronous RoboFly sensor measurements...")
    robofly_measurements = generate_asynchronous_sensor_data(
        time, q_data, terrain_type, terrain_params, sensor_rates)
    
    # Save to CSV if requested
    if save_csv:
        filename = f"terrain_crossing_{terrain_type}.csv"
        save_robofly_csv_async(time, q_data, u_data, robofly_measurements, filename, terrain_type, sensor_rates)
        print(f"Saved trajectory to {filename}")
    
    return time, q_data, robofly_measurements, waypoints

def generate_asynchronous_sensor_data(time, q_data, terrain_type, terrain_params, sensor_rates):
    """
    Generate sensor measurements at different update rates for asynchronous EKF testing
    
    Args:
        time: Simulation time vector (200Hz base rate)
        q_data: State trajectory data
        terrain_type: Type of terrain
        terrain_params: Terrain parameters
        sensor_rates: Dict with sensor update rates in Hz, or None for synchronous 200Hz
    
    Returns:
        measurements: Dict for async sensors, numpy array for sync sensors (backward compatibility)
    """
    base_rate = 200  # Hz - robognat simulation rate
    dt_base = 1.0 / base_rate
    
    # If sensor_rates is None, return synchronous measurements (old format)
    if sensor_rates is None:
        robofly_measurements = np.zeros((len(time), 4))
        for i in range(len(time)):
            robofly_measurements[i] = robofly_sensor_model_from_state(
                q_data[i], noisy=True, terrain_type=terrain_type, terrain_params=terrain_params)
        return robofly_measurements
    
    # Otherwise, generate asynchronous measurements (new format)
    measurements = {}
    
    for sensor_name, rate in sensor_rates.items():
        if rate > base_rate:
            raise ValueError(f"Sensor rate {rate}Hz cannot exceed base simulation rate {base_rate}Hz")
        
        # Calculate decimation factor
        decimation = base_rate // rate
        
        # Generate measurements at reduced rate
        if sensor_name == 'range':
            sensor_data = []
            sensor_times = []
            for i in range(0, len(time), decimation):
                meas = robofly_sensor_model_from_state(
                    q_data[i], noisy=True, terrain_type=terrain_type, terrain_params=terrain_params)
                sensor_data.append(meas[0])  # Range sensor
                sensor_times.append(time[i])
            measurements['range'] = {'data': np.array(sensor_data), 'time': np.array(sensor_times)}
            
        elif sensor_name == 'flow':
            sensor_data = []
            sensor_times = []
            for i in range(0, len(time), decimation):
                meas = robofly_sensor_model_from_state(
                    q_data[i], noisy=True, terrain_type=terrain_type, terrain_params=terrain_params)
                sensor_data.append(meas[1])  # Optical flow
                sensor_times.append(time[i])
            measurements['flow'] = {'data': np.array(sensor_data), 'time': np.array(sensor_times)}
            
        elif sensor_name == 'accel':
            sensor_data_x = []
            sensor_data_z = []
            sensor_times = []
            for i in range(0, len(time), decimation):
                meas = robofly_sensor_model_from_state(
                    q_data[i], noisy=True, terrain_type=terrain_type, terrain_params=terrain_params)
                sensor_data_x.append(meas[2])  # Accel X
                sensor_data_z.append(meas[3])  # Accel Z
                sensor_times.append(time[i])
            measurements['accel'] = {
                'data_x': np.array(sensor_data_x), 
                'data_z': np.array(sensor_data_z), 
                'time': np.array(sensor_times)
            }
    
    return measurements

def save_robofly_csv_async(time, q_data, u_data, measurements, filename, terrain_type, sensor_rates):
    """Save asynchronous sensor data in EKF-compatible format"""
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Add metadata as comments
        writer.writerow([f"# Terrain-crossing trajectory for EKF testing"])
        writer.writerow([f"# Terrain type: {terrain_type}"])
        writer.writerow([f"# Generated with realistic robognat physics"])
        writer.writerow([f"# Spatial coverage: {np.max(q_data[:,2]) - np.min(q_data[:,2]):.3f}m"])
        writer.writerow([f"# Asynchronous sensor rates: {sensor_rates}"])
        writer.writerow([f"# Format: timestamp,dt,range,optical_flow,accel_x,accel_z,omega,mask0,mask1,mask2,mask3"])
        writer.writerow([f"# mask=1 means sensor data available at this timestep, mask=0 means no data"])
        
        # Create a unified timeline at base rate (200Hz) with sensor masks
        base_dt = time[1] - time[0]  # Should be 1/200 = 0.005s
        
        # Create sensor index tracking
        range_idx = 0
        flow_idx = 0
        accel_idx = 0
        
        for i, timestamp in enumerate(time):
            dt = base_dt
            ctrl0 = q_data[i, 1]  # omega_y from state
            
            # Initialize sensor values and masks
            range_val = 0.0
            flow_val = 0.0
            accel_x_val = 0.0
            accel_z_val = 0.0
            mask_range = 0
            mask_flow = 0
            mask_accel = 0
            
            # Check if range sensor has data at this time
            if 'range' in measurements:
                range_times = measurements['range']['time']
                if range_idx < len(range_times) and abs(timestamp - range_times[range_idx]) < base_dt/2:
                    range_val = measurements['range']['data'][range_idx]
                    mask_range = 1
                    range_idx += 1
            
            # Check if optical flow sensor has data at this time  
            if 'flow' in measurements:
                flow_times = measurements['flow']['time']
                if flow_idx < len(flow_times) and abs(timestamp - flow_times[flow_idx]) < base_dt/2:
                    flow_val = measurements['flow']['data'][flow_idx]
                    mask_flow = 1
                    flow_idx += 1
            
            # Check if accelerometer has data at this time
            if 'accel' in measurements:
                accel_times = measurements['accel']['time']
                if accel_idx < len(accel_times) and abs(timestamp - accel_times[accel_idx]) < base_dt/2:
                    accel_x_val = measurements['accel']['data_x'][accel_idx]
                    accel_z_val = measurements['accel']['data_z'][accel_idx]
                    mask_accel = 1
                    accel_idx += 1
            
            # Write row with sensor data and masks
            writer.writerow([timestamp, dt, range_val, flow_val, accel_x_val, accel_z_val, ctrl0, 
                           mask_range, mask_flow, mask_accel, mask_accel])  # accel has same mask for x,z

def plot_terrain_crossing_analysis(time, q_data, robofly_measurements, waypoints, terrain_type):
    """Plot comprehensive analysis of terrain-crossing trajectory"""
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle(f'Terrain-Crossing Trajectory Analysis: {terrain_type}', fontsize=14)
    
    # 2D trajectory with waypoints
    axes[0,0].plot(q_data[:,2], q_data[:,4], 'b-', linewidth=2, label='Flight path')
    waypoint_x, waypoint_z = zip(*waypoints)
    axes[0,0].plot(waypoint_x, waypoint_z, 'ro-', markersize=8, label='Waypoints')
    axes[0,0].set_xlabel('X position (m)')
    axes[0,0].set_ylabel('Z position (m)')
    axes[0,0].set_title('2D Trajectory')
    axes[0,0].legend()
    axes[0,0].grid(True, alpha=0.3)
    axes[0,0].axis('equal')
    
    # Height profile with ground terrain
    x_ground = np.linspace(np.min(q_data[:,2]), np.max(q_data[:,2]), 100)
    z_ground = [generate_ground_height(x, 0, terrain_type) for x in x_ground]
    
    axes[0,1].plot(q_data[:,2], q_data[:,4], 'b-', linewidth=2, label='Flight height')
    axes[0,1].plot(x_ground, z_ground, 'k-', linewidth=2, label='Ground profile')
    axes[0,1].fill_between(x_ground, z_ground, alpha=0.3, color='brown')
    axes[0,1].set_xlabel('X position (m)')
    axes[0,1].set_ylabel('Z position (m)')
    axes[0,1].set_title('Height Profile vs Ground')
    axes[0,1].legend()
    axes[0,1].grid(True, alpha=0.3)
    
    # Sensor measurements over time
    axes[0,2].plot(time, robofly_measurements[:,0], label='ToF Range (m)')
    axes[0,2].set_xlabel('Time (s)')
    axes[0,2].set_ylabel('Range (m)')
    axes[0,2].set_title('ToF Range Sensor')
    axes[0,2].grid(True, alpha=0.3)
    
    axes[1,0].plot(time, robofly_measurements[:,1], label='Optical Flow (rad/s)', color='green')
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Optical Flow (rad/s)')
    axes[1,0].set_title('Optical Flow Sensor')
    axes[1,0].grid(True, alpha=0.3)
    
    # Accelerometer readings
    axes[1,1].plot(time, robofly_measurements[:,2], label='Accel X (m/s²)', color='red')
    axes[1,1].plot(time, robofly_measurements[:,3], label='Accel Z (m/s²)', color='orange')
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Acceleration (m/s²)')
    axes[1,1].set_title('Accelerometer Readings')
    axes[1,1].legend()
    axes[1,1].grid(True, alpha=0.3)
    
    # Coverage analysis
    x_range = np.max(q_data[:,2]) - np.min(q_data[:,2])
    z_range = np.max(q_data[:,4]) - np.min(q_data[:,4])
    
    coverage_text = f"""Trajectory Coverage Analysis:
    
X Range: {x_range:.3f}m
Z Range: {z_range*1000:.1f}mm

Sensor Statistics:
Range: {np.std(robofly_measurements[:,0])*1000:.1f}mm std
Flow: {np.std(robofly_measurements[:,1]):.3f} rad/s std
Accel X: {np.std(robofly_measurements[:,2]):.3f} m/s² std
Accel Z: {np.std(robofly_measurements[:,3]):.3f} m/s² std

Terrain Coverage: {'✅ GOOD' if x_range > 1.0 else '❌ LIMITED'}
"""
    
    axes[1,2].text(0.05, 0.95, coverage_text, transform=axes[1,2].transAxes, 
                   fontsize=10, verticalalignment='top', fontfamily='monospace')
    axes[1,2].set_xlim(0, 1)
    axes[1,2].set_ylim(0, 1)
    axes[1,2].axis('off')
    axes[1,2].set_title('Coverage Statistics')
    
    plt.tight_layout()
    plt.savefig(f'terrain_crossing_{terrain_type}_analysis.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    return fig

def main():
    """Generate terrain-crossing trajectories for multiple terrain types"""
    parser = argparse.ArgumentParser(description='Generate Realistic Terrain-Crossing Trajectories')
    parser.add_argument('--terrain', type=str, default='gradual_slope',
                       choices=['flat', 'gradual_slope', 'steep_slope', 'sinusoidal', 
                               'random_smooth', 'random_rough', 'step_terrain', 'indoor_bumps'],
                       help='Terrain type to test')
    parser.add_argument('--length', type=float, default=2.0, help='Trajectory length (meters)')
    parser.add_argument('--duration', type=float, default=30.0, help='Simulation duration (seconds)')
    parser.add_argument('--waypoints', type=int, default=8, help='Number of waypoints')
    parser.add_argument('--all', action='store_true', help='Generate all terrain types')
    parser.add_argument('--use-estimator', action='store_true', 
                       help='Use state estimator for control (default: use ground truth)')
    parser.add_argument('--async-sensors', action='store_true',
                       help='Use asynchronous sensor rates for testing sequential updates')
    parser.add_argument('--range-rate', type=int, default=100, help='ToF range sensor rate (Hz)')
    parser.add_argument('--flow-rate', type=int, default=100, help='Optical flow sensor rate (Hz)')
    parser.add_argument('--accel-rate', type=int, default=200, help='Accelerometer rate (Hz)')
    
    args = parser.parse_args()
    
    # Configure sensor rates
    if args.async_sensors:
        sensor_rates = {
            'range': args.range_rate,
            'flow': args.flow_rate, 
            'accel': args.accel_rate
        }
        print(f"Using asynchronous sensors: {sensor_rates}")
    else:
        sensor_rates = None  # Default synchronous 200Hz
        print("Using synchronous 200Hz sensors")
    
    # Configure control method
    use_ground_truth_control = not args.use_estimator
    print(f"Control method: {'Ground truth' if use_ground_truth_control else 'State estimator'}")
    
    if args.all:
        terrain_types = ['flat', 'gradual_slope', 'steep_slope', 'step_terrain', 'sinusoidal']
        
        for terrain in terrain_types:
            print(f"\n{'='*60}")
            print(f"GENERATING TRAJECTORY FOR {terrain.upper()}")
            print(f"{'='*60}")
            
            terrain_params = {}
            if terrain == 'gradual_slope':
                terrain_params = {'slope_x': 0.03, 'slope_z': 0.01}  # 3cm/m slope
            elif terrain == 'steep_slope':
                terrain_params = {'slope_x': 0.08, 'slope_z': 0.04}  # 8cm/m slope
            elif terrain == 'step_terrain':
                terrain_params = {'step_height': 0.025, 'step_size_x': 0.4}  # 2.5cm steps every 40cm
            
            time, q_data, measurements, waypoints = generate_terrain_crossing_trajectory(
                terrain_type=terrain,
                terrain_params=terrain_params,
                trajectory_length=args.length,
                tfinal=args.duration,
                n_waypoints=args.waypoints,
                use_ground_truth_control=use_ground_truth_control,
                sensor_rates=sensor_rates
            )
            
            # Only plot for synchronous sensors (measurements is numpy array)
            if not args.async_sensors and isinstance(measurements, np.ndarray):
                plot_terrain_crossing_analysis(time, q_data, measurements, waypoints, terrain)
    
    else:
        # Single terrain type
        terrain_params = {}
        if args.terrain == 'gradual_slope':
            terrain_params = {'slope_x': 0.02, 'slope_z': 0.01}
        elif args.terrain == 'steep_slope':
            terrain_params = {'slope_x': 0.1, 'slope_z': 0.05}
        elif args.terrain == 'step_terrain':
            terrain_params = {'step_height': 0.02, 'step_size_x': 0.5}
        
        time, q_data, measurements, waypoints = generate_terrain_crossing_trajectory(
            terrain_type=args.terrain,
            terrain_params=terrain_params, 
            trajectory_length=args.length,
            tfinal=args.duration,
            n_waypoints=args.waypoints,
            use_ground_truth_control=use_ground_truth_control,
            sensor_rates=sensor_rates
        )
        
        # Only plot for synchronous sensors (measurements is numpy array)
        if not args.async_sensors and isinstance(measurements, np.ndarray):
            plot_terrain_crossing_analysis(time, q_data, measurements, waypoints, args.terrain)

if __name__ == "__main__":
    main() 