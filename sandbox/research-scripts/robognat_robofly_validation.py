#!/usr/bin/env python3
"""
RoboFly sensor data generator using original robognat simulation.
Takes the original robognat trajectory and adds RoboFly sensor measurements.
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
# This ensures robognat_clean.py can find photo.mat, 15mmgust.txt, 30mmgust.txt
data_dir = os.path.join(original_dir, 'robognat_data')
os.chdir(data_dir)

# Import the original robognat functions
from robognat_clean import *

# Change back to original directory
os.chdir(original_dir)

# Constants (from original robognat)
DT = 1./200  # 200 Hz sampling
g = 9.81

# Robognat parameters (from original)
class RobognatParams:
    def __init__(self):
        # Gnatbot physical parameters
        self.h = 0.001  # height of body [m]
        self.l = .004   # length of body [m]
        self.m = 10e-6  # mass [kg] (gnatbot)
        self.J = self.m * self.l**2 / 12  # J for a thin rod [kgm^2]
        
        # Aerodynamic parameters
        b_over_m_gnatbot = 1.0  # Simplified for now
        self.b = b_over_m_gnatbot * self.m  # aero drag on wings
        self.d_z = 0.0002  # z-distance from wings to center of mass [m]
        self.winglength = 4.6e-3  # [m]
        self.c = (self.l/2 + self.winglength * 2./3)**2 * self.b  # rotational drag
        
        # Control parameters
        self.pos_desired = (0.1, 0.15)  # [x_world, z_world] - target position
        
        # Sensor noise (simplified)
        self.sensor_noise_std = (0.01, 0.01, 0.1)  # [accel_x, accel_z, optic flow]

def R(theta):
    """2D rotation matrix: vworld = R @ vbody"""
    return array(((cos(theta), sin(theta)), 
                  (-sin(theta), cos(theta))))

def cross(a, b):
    """3D cross product"""
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])

def aerodynamics(t, v_2D, thetadoty, p, R2, v_wind=(0,0)):
    """
    Stroke-averaged aerodynamic forces and torques (from original robognat)
    """
    d_z_2D = R2 @ (0, p.d_z)  # vector from CM to center of drag in world coords
    d_z = (d_z_2D[0], 0, d_z_2D[1])  # y-component is zero when only in planar motion
    v = array((v_2D[0], 0, v_2D[1]))  # 3D velocity
    omega = array((0, thetadoty, 0))  # angular velocity vector
        
    v_wings = v + cross(omega, d_z) 
    airspeed_wings = (v_wind[0], 0, v_wind[1]) - v_wings
    f_d = p.b * airspeed_wings
    tauy_d = cross(d_z, f_d) - p.c * omega
    return (f_d[0], f_d[2]), tauy_d[1]

def dynamics_model(t, q, u, p, v_wind=(0,0)):
    """
    Original robognat dynamics with 7 states: [thetay, omegay, x, vx, z, vz, vw]
    """
    thetay = q[0]
    thetadoty = q[1]
    v_world = q[[3, 5]]  # [vx, vz]
    R2 = R(thetay)
    
    # Forces in world frame [f_x, f_z]
    f_d, tauy_d = aerodynamics(t, v_world, thetadoty, p, R2, v_wind)
    f_g = array((0, -p.m * g))  # gravity
    f_c = R2 @ (0, u[0] + g) * p.m  # control accel + baseline g thrust
    f = f_g + f_c + f_d

    # Torques (body y-axis)
    tauy = u[1] * p.J + tauy_d 

    # Calculate derivatives (7 states)
    thetadotdoty = 1/p.J * tauy
    vdot = 1/p.m * f
    qdot = array((thetadoty, thetadotdoty, v_world[0], vdot[0], v_world[1], vdot[1], 0))
    return qdot

def generate_ground_height(x, z, terrain_type="flat", terrain_params=None):
    """
    Generate ground height at position (x, z) for different terrain types
    
    Args:
        x, z: Position coordinates (meters)
        terrain_type: Type of terrain to generate
            - "flat": Perfectly flat ground (z=0)
            - "gradual_slope": Gentle linear slope
            - "steep_slope": Steep linear slope  
            - "sinusoidal": Smooth rolling hills (original implementation)
            - "random_smooth": Random but smooth depth map using Perlin-like noise
            - "random_rough": Random rough terrain with higher frequency variations
            - "step_terrain": Discrete height steps (like stairs/platforms)
            - "indoor_bumps": Small indoor variations (cables, debris, etc.)
        terrain_params: Dictionary of parameters specific to each terrain type
    
    Returns:
        ground_height: Height of ground at position (x, z) in meters
    """
    if terrain_params is None:
        terrain_params = {}
    
    if terrain_type == "flat":
        return 0.0
        
    elif terrain_type == "gradual_slope":
        # Gentle slope - realistic for indoor environments
        slope_x = terrain_params.get('slope_x', 0.02)  # 2cm/m = 1.15° slope
        slope_z = terrain_params.get('slope_z', 0.01)  # 1cm/m = 0.57° slope
        return slope_x * x + slope_z * z
        
    elif terrain_type == "steep_slope":
        # Steeper slope - challenging for EKF
        slope_x = terrain_params.get('slope_x', 0.1)   # 10cm/m = 5.7° slope
        slope_z = terrain_params.get('slope_z', 0.05)  # 5cm/m = 2.9° slope
        return slope_x * x + slope_z * z
        
    elif terrain_type == "sinusoidal":
        # Original smooth rolling hills implementation
        slope_amplitude = terrain_params.get('amplitude', 0.02)  # 2cm height variations
        freq_x = terrain_params.get('freq_x', 0.5)  # cycles per meter in x
        freq_z = terrain_params.get('freq_z', 0.3)  # cycles per meter in z
        roughness_amp = terrain_params.get('roughness', 0.005)  # 5mm roughness
        
        ground_height = (slope_amplitude * sin(2*pi*freq_x * x) * 
                        cos(2*pi*freq_z * z))
        
        # Add fine roughness
        roughness = roughness_amp * sin(10*x) * cos(8*z)
        return ground_height + roughness
        
    elif terrain_type == "random_smooth":
        # Random but smooth terrain using multiple frequency components
        # This creates a realistic "random depth map" that's still smooth
        amplitude = terrain_params.get('amplitude', 0.015)  # Reduced from 0.03 to 0.015 (1.5cm variations)
        seed = terrain_params.get('seed', 42)
        
        # Create multiple frequency components for realistic terrain
        # Use only low frequency components to keep it smooth
        np.random.seed(int(seed + 1000*x + 100*z) % 2**32)  # Position-dependent seed
        
        # Low frequency components only (large, smooth features)
        h1 = amplitude * 0.5 * sin(2*pi*0.15*x + np.random.uniform(0, 2*pi)) * cos(2*pi*0.1*z + np.random.uniform(0, 2*pi))  # Very low freq
        h2 = amplitude * 0.3 * sin(2*pi*0.25*x + np.random.uniform(0, 2*pi)) * cos(2*pi*0.2*z + np.random.uniform(0, 2*pi))  # Low freq
        h3 = amplitude * 0.2 * sin(2*pi*0.35*x + np.random.uniform(0, 2*pi)) * cos(2*pi*0.3*z + np.random.uniform(0, 2*pi))  # Medium-low freq
        
        return h1 + h2 + h3
        
    elif terrain_type == "random_rough":
        # Rougher random terrain with higher frequency variations
        amplitude = terrain_params.get('amplitude', 0.025)  # 2.5cm variations
        roughness = terrain_params.get('roughness', 0.01)   # 1cm roughness
        seed = terrain_params.get('seed', 42)
        
        # Position-dependent randomness
        np.random.seed(int(seed + 1000*x + 100*z) % 2**32)
        
        # Multiple scales of variation
        h_large = amplitude * 0.5 * sin(2*pi*0.3*x + np.random.uniform(0, 2*pi)) * cos(2*pi*0.25*z + np.random.uniform(0, 2*pi))
        h_medium = amplitude * 0.3 * sin(2*pi*0.7*x + np.random.uniform(0, 2*pi)) * cos(2*pi*0.6*z + np.random.uniform(0, 2*pi))
        h_small = roughness * sin(2*pi*2.0*x + np.random.uniform(0, 2*pi)) * cos(2*pi*1.8*z + np.random.uniform(0, 2*pi))
        h_fine = roughness * 0.5 * sin(2*pi*5.0*x + np.random.uniform(0, 2*pi)) * cos(2*pi*4.5*z + np.random.uniform(0, 2*pi))
        
        return h_large + h_medium + h_small + h_fine
        
    elif terrain_type == "step_terrain":
        # Discrete step changes (like platforms or stairs)
        step_height = terrain_params.get('step_height', 0.02)  # 2cm steps
        step_size_x = terrain_params.get('step_size_x', 0.5)   # 50cm step length
        step_size_z = terrain_params.get('step_size_z', 0.4)   # 40cm step width
        
        # Create discrete steps
        step_x = int(x / step_size_x) 
        step_z = int(z / step_size_z)
        
        # Height based on step pattern
        height = step_height * ((step_x + step_z) % 3 - 1)  # Pattern: -1, 0, 1
        
        return height
        
    elif terrain_type == "indoor_bumps":
        # Small indoor variations: cables, debris, small objects on floor
        bump_amplitude = terrain_params.get('amplitude', 0.005)  # 5mm typical
        cable_height = terrain_params.get('cable_height', 0.002)  # 2mm cables
        debris_height = terrain_params.get('debris_height', 0.008) # 8mm debris
        
        ground_height = 0.0
        
        # Simulate occasional cables (long thin bumps)
        cable_spacing = 1.2  # Every 1.2m on average
        if abs(x % cable_spacing) < 0.01:  # 1cm wide "cable"
            ground_height += cable_height
            
        # Simulate small debris (localized bumps)
        debris_spacing_x = 0.8
        debris_spacing_z = 0.6
        if (abs(x % debris_spacing_x) < 0.05 and abs(z % debris_spacing_z) < 0.05):
            ground_height += debris_height
            
        # Add fine surface texture
        texture = bump_amplitude * sin(2*pi*8*x) * cos(2*pi*6*z)
        
        return ground_height + texture
        
    else:
        raise ValueError(f"Unknown terrain_type: {terrain_type}")


def get_terrain_slope(x, z, terrain_type, terrain_params=None, delta=0.001):
    """
    Compute ground slope at position (x, z) for optical flow corrections
    Returns (slope_x, slope_z) - derivatives of ground height
    """
    if terrain_type == "flat":
        return 0.0, 0.0
        
    # Numerical differentiation for slope calculation
    h_center = generate_ground_height(x, z, terrain_type, terrain_params)
    h_x_plus = generate_ground_height(x + delta, z, terrain_type, terrain_params)
    h_z_plus = generate_ground_height(x, z + delta, terrain_type, terrain_params)
    
    slope_x = (h_x_plus - h_center) / delta
    slope_z = (h_z_plus - h_center) / delta
    
    return slope_x, slope_z

def lqr_controller(q, p, t):
    """
    Simplified LQR controller to maintain stable flight
    """
    # Target state: hover at desired position
    q_desired = array((0, 0, p.pos_desired[0], 0, p.pos_desired[1], 0, 0))
    q_error = q_desired - q
    
    # Simple proportional gains (simplified LQR)
    K = array([[0.1, 0.1, 0.01, 0.1, 0.01, 0.1, 0],   # u[0]: z acceleration
               [1.0, 0.5, 0,    0,   0,    0,   0]])   # u[1]: angular acceleration
    
    u = K @ q_error
    
    # Limit control authority
    u[0] = np.clip(u[0], -0.5*g, 0.5*g)  # z acceleration limit
    u[1] = np.clip(u[1], -10, 10)        # angular acceleration limit
    
    return u

def robofly_sensor_model_from_state(q, noisy=True, terrain_type="flat", terrain_params=None):
    """
    Generate RoboFly sensor measurements from robognat state
    q = [thetay, omegay, x, vx, z, vz, vw]
    Returns: [range, optical_flow, accel_x, accel_z]
    
    terrain_type: Type of ground terrain affecting ToF and optical flow sensors
    terrain_params: Parameters for the specific terrain type
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

def generate_robofly_data_from_robognat(tfinal=10.0, use_estimator=True, noisy=True, terrain_type="flat", terrain_params=None):
    """
    Run original robognat simulation and extract RoboFly sensor data
    terrain_type: Type of ground terrain affecting sensor measurements
    terrain_params: Parameters for the specific terrain type
    """
    print(f"Running robognat simulation (terrain_type={terrain_type})...")
    
    # Use the original robognat simulation function with accelerometer+optical flow sensors
    time, q_data, u_data, y_data, qhat_data = simulate(
        p=p, 
        q=None,  # Use default initial conditions
        tfinal=tfinal, 
        sensor_model=ideal_ao_sensor_model,  # Original airspeed + optical flow
        dynamics_model=dynamics_model,       # Original dynamics
        controller=lqr_controller_ao,        # Original LQR controller  
        estimator_dynamics=estimator_dynamics_ao,  # Original estimator
        wind_function=zero_wind,
        use_estimator=use_estimator, 
        noisy=noisy, 
        perturb_qhat=True
    )
    
    print(f"Simulation complete: {len(time)} samples over {time[-1]:.1f} seconds")
    
    # Generate RoboFly sensor measurements from the robognat states
    print(f"Generating RoboFly sensor measurements with {terrain_type} terrain...")
    robofly_measurements = np.zeros((len(time), 4))
    
    for i in range(len(time)):
        robofly_measurements[i] = robofly_sensor_model_from_state(
            q_data[i], noisy=noisy, terrain_type=terrain_type, terrain_params=terrain_params)
    
    return time, q_data, u_data, y_data, qhat_data, robofly_measurements

def save_robofly_csv(time, q_data, u_data, robofly_measurements, filename, 
                     async_sensors=False, imu_rate_hz=1000, flow_rate_hz=100, sync_rate_hz=50):
    """
    Save data in RoboFly EKF format: timestamp,dt,meas0,meas1,meas2,meas3,ctrl0,mask0,mask1,mask2,mask3
    
    Args:
        time: Time vector
        q_data: State data
        u_data: Control data  
        robofly_measurements: Measurement data
        filename: Output CSV filename
        async_sensors: If True, generate asynchronous sensor measurements
        imu_rate_hz: IMU sampling rate (Hz) - sensors 2,3 (accel_x, accel_z)
        flow_rate_hz: Optical flow rate (Hz) - sensor 1 (optical_flow)  
        sync_rate_hz: Synchronized measurement rate (Hz) - all sensors 0,1,2,3
        
    Asynchronous sensor schedule (based on paper):
    - IMU (accel_x, accel_z): Available every 1ms (1000 Hz)
    - Optical flow: Available every 10ms (100 Hz) in addition to IMU
    - All sensors synchronized: Available every 20ms (50 Hz)
    
    This creates:
    - 50 occurrences/sec with all 4 measurements (every 20ms)
    - 50 occurrences/sec with 3 measurements (IMU + flow, every 20ms offset)
    - 900 occurrences/sec with 2 measurements (IMU only, remaining 1ms slots)
    """
    # Use the filename as provided (can be relative path)
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        row_count = 0  # Track row count manually
        
        if not async_sensors:
            # Original synchronous behavior - all sensors at same time
            for i in range(len(time)):
                timestamp = time[i]
                dt = DT
                
                # RoboFly measurements: [range, optical_flow, accel_x, accel_z]
                meas0, meas1, meas2, meas3 = robofly_measurements[i]
                
                # Controls: Use actual angular velocity from state
                ctrl0 = q_data[i, 1]  # omegay (angular velocity) from robognat state
                
                # All sensors active (synchronous)
                mask0, mask1, mask2, mask3 = 1, 1, 1, 1
                
                # Write CSV row
                writer.writerow([timestamp, dt, meas0, meas1, meas2, meas3, ctrl0, mask0, mask1, mask2, mask3])
                row_count += 1
        else:
            # Asynchronous sensor behavior
            # Calculate time periods
            imu_period = 1.0 / imu_rate_hz      # 0.001s (1ms)
            flow_period = 1.0 / flow_rate_hz    # 0.01s (10ms)  
            sync_period = 1.0 / sync_rate_hz    # 0.02s (20ms)
            
            # Generate asynchronous measurement schedule
            current_time = time[0]
            
            # Interpolation function for measurements
            def interpolate_measurement(target_time, time_vec, meas_data):
                # Find closest time index
                idx = np.argmin(np.abs(time_vec - target_time))
                return meas_data[idx]
            
            def interpolate_state(target_time, time_vec, state_data):
                # Find closest time index  
                idx = np.argmin(np.abs(time_vec - target_time))
                return state_data[idx]
            
            # Generate measurements at IMU rate (1ms intervals)
            time_step = 0
            while current_time <= time[-1]:
                # Get interpolated measurements and state
                meas = interpolate_measurement(current_time, time, robofly_measurements)
                state = interpolate_state(current_time, time, q_data)
                
                meas0, meas1, meas2, meas3 = meas
                ctrl0 = state[1]  # omegay
                
                # Determine sensor availability based on time step
                # Every 20ms (20 steps): All sensors (sync)
                # Every 10ms (10 steps): IMU + optical flow  
                # Every 1ms (1 step): IMU only
                
                if time_step % 20 == 0:
                    # All sensors synchronized (every 20ms)
                    mask0, mask1, mask2, mask3 = 1, 1, 1, 1
                elif time_step % 10 == 0:
                    # IMU + optical flow (every 10ms, excluding sync times)
                    mask0, mask1, mask2, mask3 = 0, 1, 1, 1  # No range sensor
                else:
                    # Only IMU (every 1ms, excluding flow and sync times)
                    mask0, mask1, mask2, mask3 = 0, 0, 1, 1  # Only accelerometers
                
                # Calculate dt (constant for simplicity)
                dt = imu_period
                
                # Write CSV row
                writer.writerow([current_time, dt, meas0, meas1, meas2, meas3, ctrl0, mask0, mask1, mask2, mask3])
                row_count += 1
                
                # Advance to next time step
                time_step += 1
                current_time += imu_period
    
    if async_sensors:
        print(f"Saved RoboFly asynchronous validation data to {filename}")
        print(f"  IMU rate: {imu_rate_hz} Hz, Flow rate: {flow_rate_hz} Hz, Sync rate: {sync_rate_hz} Hz")
        print(f"  Total rows generated: {row_count}")
    else:
        print(f"Saved RoboFly synchronous validation data to {filename}")
        print(f"  Total rows generated: {row_count}")

def plot_comparison(time, q_data, y_original, robofly_measurements):
    """Plot comparison between original robognat and RoboFly measurements"""
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))  # Changed from 3x2 to 3x3 for trajectory plot
    fig.suptitle('Original Robognat vs RoboFly Sensor Comparison')
    
    # 2D Trajectory plot (NEW!)
    axes[0,0].plot(q_data[:,2], q_data[:,4], 'b-', linewidth=2, label='Flight trajectory')
    axes[0,0].set_xlabel('X position (m)')
    axes[0,0].set_ylabel('Z position (m)')
    axes[0,0].set_title('2D Flight Trajectory')
    axes[0,0].legend()
    axes[0,0].grid(True)
    axes[0,0].axis('equal')
    
    # Add trajectory coverage statistics
    x_range = np.max(q_data[:,2]) - np.min(q_data[:,2])
    z_range = np.max(q_data[:,4]) - np.min(q_data[:,4])
    axes[0,0].text(0.02, 0.98, f'X range: {x_range:.3f}m\nZ range: {z_range*1000:.1f}mm', 
                   transform=axes[0,0].transAxes, verticalalignment='top', 
                   bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Original trajectory time series
    axes[0,1].plot(time, q_data[:,2], label='x (m)')
    axes[0,1].plot(time, q_data[:,4], label='z (m)')
    axes[0,1].set_ylabel('Position (m)')
    axes[0,1].set_title('Original Robognat Trajectory')
    axes[0,1].legend()
    axes[0,1].grid(True)
    
    axes[0,2].plot(time, q_data[:,0] * 180/np.pi, label='theta (deg)')
    axes[0,2].plot(time, q_data[:,1] * 180/np.pi, label='omega (deg/s)')
    axes[0,2].set_ylabel('Attitude')
    axes[0,2].set_title('Original Robognat Attitude')
    axes[0,2].legend()
    axes[0,2].grid(True)
    
    # Original robognat sensors vs RoboFly sensors
    axes[1,0].plot(time, y_original[:,2], label='Robognat Optical Flow', alpha=0.7)
    axes[1,0].plot(time, robofly_measurements[:,1], label='RoboFly Optical Flow', alpha=0.7)
    axes[1,0].set_ylabel('Optical Flow (rad/s)')
    axes[1,0].set_title('Optical Flow Comparison')
    axes[1,0].legend()
    axes[1,0].grid(True)
    
    # RoboFly specific sensors
    axes[1,1].plot(time, robofly_measurements[:,0], label='Range (m)')
    axes[1,1].set_ylabel('ToF Range (m)')
    axes[1,1].set_title('RoboFly Range Sensor')
    axes[1,1].legend()
    axes[1,1].grid(True)
    
    axes[1,2].plot(time, robofly_measurements[:,2], label='Accel X (m/s²)')
    axes[1,2].set_ylabel('Acceleration X (m/s²)')
    axes[1,2].set_title('RoboFly Accelerometer X')
    axes[1,2].legend()
    axes[1,2].grid(True)
    
    axes[2,0].plot(time, robofly_measurements[:,3], label='Accel Z (m/s²)')
    axes[2,0].set_ylabel('Acceleration Z (m/s²)')
    axes[2,0].set_title('RoboFly Accelerometer Z')
    axes[2,0].legend()
    axes[2,0].grid(True)
    
    # Velocity plots for better understanding of motion
    axes[2,1].plot(time, q_data[:,3], label='Vx (m/s)', color='blue')
    axes[2,1].plot(time, q_data[:,5], label='Vz (m/s)', color='orange')
    axes[2,1].set_ylabel('Velocity (m/s)')
    axes[2,1].set_title('Robognat Velocities')
    axes[2,1].legend()
    axes[2,1].grid(True)
    
    # Control inputs
    # Note: We need to add u_data to the function parameters
    axes[2,2].text(0.5, 0.5, 'Control inputs\n(add u_data to\nfunction params)', 
                   transform=axes[2,2].transAxes, ha='center', va='center')
    axes[2,2].set_title('Control Commands')
    
    for ax in axes.flat:
        ax.set_xlabel('Time (s)')
    
    plt.tight_layout()
    return fig

def plot_2d_trajectory_with_terrain(q_data, terrain_type="flat", terrain_params=None, save_name=None):
    """
    Plot 2D trajectory with ground terrain profile
    
    Args:
        q_data: State trajectory [N x 7] - [theta, omega, x, vx, z, vz, vw]
        terrain_type: Type of terrain to visualize
        terrain_params: Parameters for terrain generation
        save_name: Optional filename to save plot
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Extract trajectory
    x_traj = q_data[:, 2]  # x position
    z_traj = q_data[:, 4]  # z position (height)
    
    # Create ground profile along trajectory path
    x_ground = np.linspace(np.min(x_traj), np.max(x_traj), 200)
    z_ground = [generate_ground_height(x, 0, terrain_type, terrain_params) for x in x_ground]
    
    # Plot 1: Top view (X-Y, though we don't have Y movement in 2D)
    axes[0].plot(x_traj, z_traj, 'b-', linewidth=2, label='Flight path')
    axes[0].set_xlabel('X position (m)')
    axes[0].set_ylabel('Z position (m)')
    axes[0].set_title('2D Flight Trajectory (Top View)')
    axes[0].legend()
    axes[0].grid(True)
    axes[0].axis('equal')
    
    # Add coverage stats
    x_range = np.max(x_traj) - np.min(x_traj)
    z_range = np.max(z_traj) - np.min(z_traj)
    axes[0].text(0.02, 0.98, f'Coverage:\nX: {x_range:.3f}m\nZ: {z_range*1000:.1f}mm', 
                 transform=axes[0].transAxes, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    # Plot 2: Side view with ground terrain
    axes[1].plot(x_traj, z_traj, 'b-', linewidth=3, label='Flight height')
    axes[1].plot(x_ground, z_ground, 'brown', linewidth=2, label='Ground profile')
    axes[1].fill_between(x_ground, z_ground, alpha=0.3, color='brown', label='Ground')
    axes[1].set_xlabel('X position (m)')
    axes[1].set_ylabel('Height (m)')
    axes[1].set_title(f'Side View: Flight vs {terrain_type.replace("_", " ").title()} Terrain')
    axes[1].legend()
    axes[1].grid(True)
    
    # Add terrain effect analysis
    if terrain_type != "flat":
        min_clearance = np.min([z_traj[i] - generate_ground_height(x_traj[i], 0, terrain_type, terrain_params) 
                               for i in range(len(x_traj))])
        axes[1].text(0.02, 0.02, f'Min clearance: {min_clearance*1000:.1f}mm', 
                     transform=axes[1].transAxes, verticalalignment='bottom',
                     bbox=dict(boxstyle='round', facecolor='yellow', alpha=0.8))
    
    plt.tight_layout()
    
    if save_name:
        plt.savefig(save_name, dpi=300, bbox_inches='tight')
        print(f"Saved trajectory plot to {save_name}")
    
    return fig

def main():
    parser = argparse.ArgumentParser(description='RoboFly Data Generation and EKF Validation with Multiple Terrain Types')
    parser.add_argument('--tfinal', type=float, default=10.0, help='Simulation time (seconds)')
    parser.add_argument('--terrain-type', type=str, default="flat", 
                        choices=["flat", "gradual_slope", "steep_slope", "sinusoidal", 
                                "random_smooth", "random_rough", "step_terrain", "indoor_bumps"],
                        help='Type of terrain affecting sensor measurements')
    parser.add_argument('--terrain-params', type=str, 
                        help='JSON-style dict of terrain parameters, e.g. \'{"amplitude": 0.05, "seed": 123}\'')
    parser.add_argument('--no-noise', action='store_true', help='Disable sensor noise')
    parser.add_argument('--no-estimator', action='store_true', help='Disable robognat estimator')
    parser.add_argument('--run-python-ekf', action='store_true', help='Run Python EKF validation with RMSE')
    parser.add_argument('--output', type=str, default='robofly_robognat_validation.csv', 
                        help='Output CSV filename')
    parser.add_argument('--plot', action='store_true', help='Show plots after generation')
    
    # Asynchronous sensor options
    parser.add_argument('--async-sensors', action='store_true', 
                        help='Generate asynchronous sensor measurements (default: synchronous)')
    parser.add_argument('--imu-rate', type=int, default=1000,
                        help='IMU sampling rate in Hz (default: 1000)')
    parser.add_argument('--flow-rate', type=int, default=100,
                        help='Optical flow sampling rate in Hz (default: 100)')
    parser.add_argument('--sync-rate', type=int, default=50,
                        help='Synchronized measurement rate in Hz (default: 50)')
    
    # Terrain analysis options
    parser.add_argument('--visualize-terrains', action='store_true', 
                        help='Create comparison visualization of all terrain types')
    parser.add_argument('--analyze-terrain-effects', action='store_true',
                        help='Analyze how the selected terrain type affects sensor measurements')
    
    # NEW: Waypoint trajectory option
    parser.add_argument('--waypoint', action='store_true', 
                        help='Use waypoint-following controller to cross terrain features (vs hovering)')
    parser.add_argument('--trajectory-length', type=float, default=2.0, 
                        help='Spatial distance to traverse with waypoint controller (meters)')
    parser.add_argument('--waypoints', type=int, default=8, 
                        help='Number of waypoints for terrain crossing trajectory')
    
    # Add helpful terrain examples in help
    parser.epilog = """
    TERRAIN TYPE EXAMPLES:
    
    # Flat ground (default)
    --terrain-type flat
    
    # Gentle slope (1-2 degrees)  
    --terrain-type gradual_slope --terrain-params '{"slope_x": 0.02, "slope_z": 0.01}'
    
    # Steep slope (5-10 degrees) - challenging case
    --terrain-type steep_slope --terrain-params '{"slope_x": 0.15, "slope_z": 0.08}'
    
    # Smooth rolling hills
    --terrain-type sinusoidal --terrain-params '{"amplitude": 0.03, "freq_x": 0.4}'
    
    # Random smooth terrain (like outdoor ground)
    --terrain-type random_smooth --terrain-params '{"amplitude": 0.04, "seed": 42}'
    
    # Rough random terrain (challenging case)
    --terrain-type random_rough --terrain-params '{"amplitude": 0.03, "roughness": 0.015, "seed": 123}'
    
    # Step/platform terrain
    --terrain-type step_terrain --terrain-params '{"step_height": 0.025, "step_size_x": 0.6}'
    
    # Indoor environment (cables, debris)
    --terrain-type indoor_bumps --terrain-params '{"cable_height": 0.003, "debris_height": 0.01}'
    
    ASYNCHRONOUS SENSOR EXAMPLES:
    
    # Generate asynchronous measurements matching paper rates
    --async-sensors --imu-rate 1000 --flow-rate 100 --sync-rate 50
    
    # Custom asynchronous rates
    --async-sensors --imu-rate 500 --flow-rate 50 --sync-rate 25
    
    ANALYSIS OPTIONS:
    
    # Visualize all terrain types
    --visualize-terrains --plot
    
    # Analyze effects of a specific terrain
    --terrain-type random_rough --analyze-terrain-effects --plot
    """
    
    args = parser.parse_args()
    
    # Parse terrain parameters
    terrain_params = None
    if args.terrain_params:
        try:
            # Support both JSON-style and Python dict literal syntax
            import json
            terrain_params = json.loads(args.terrain_params)
        except json.JSONDecodeError:
            try:
                # Fall back to Python literal_eval for dict syntax
                import ast
                terrain_params = ast.literal_eval(args.terrain_params)
            except (ValueError, SyntaxError) as e:
                print(f"Error parsing terrain_params: {e}")
                print("Use JSON format like: '{\"amplitude\": 0.05, \"seed\": 123}'")
                return
    
    # Handle visualization requests
    if args.visualize_terrains:
        print("Creating terrain type comparison visualization...")
        visualize_terrain_types()
        if args.plot:
            plt.show()
        return
    
    if args.analyze_terrain_effects:
        print(f"Analyzing terrain effects for {args.terrain_type}...")
        analyze_terrain_effects(args.terrain_type, terrain_params)
        if args.plot:
            plt.show()
        return
    
    print(f"Generating data with terrain type: {args.terrain_type}")
    if terrain_params:
        print(f"Terrain parameters: {terrain_params}")
    
    # Generate robognat data
    time, q_data, u_data, y_data, qhat_data, robofly_measurements = generate_robofly_data_from_robognat(
        tfinal=args.tfinal,
        use_estimator=not args.no_estimator,
        noisy=not args.no_noise,
        terrain_type=args.terrain_type,
        terrain_params=terrain_params
    )
    
    # Save CSV data
    save_robofly_csv(time, q_data, u_data, robofly_measurements, args.output,
                     async_sensors=args.async_sensors, 
                     imu_rate_hz=args.imu_rate,
                     flow_rate_hz=args.flow_rate, 
                     sync_rate_hz=args.sync_rate)
    
    # Run Python EKF validation if requested
    if args.run_python_ekf:
        py_ekf, ground_truth, rmse_results = run_python_ekf_validation(time, q_data, u_data, robofly_measurements)
        
        # Plot comparison
        plot_ekf_comparison(time, ground_truth, py_ekf.trajectory, args.terrain_type != "flat")
    
    # Plot trajectory comparison if requested
    if args.plot:
        # Plot detailed comparison with new trajectory view
        plot_comparison(time, q_data, y_data, robofly_measurements)
        
        # Plot dedicated 2D trajectory with terrain
        plot_2d_trajectory_with_terrain(q_data, args.terrain_type, terrain_params, 
                                      save_name=f'trajectory_2d_{args.terrain_type}.png')
        plt.show()

def plot_ekf_comparison(time, ground_truth, ekf_trajectory, non_planar_ground=False):
    """Plot comparison between ground truth and EKF estimates"""
    ground_truth = np.array(ground_truth)
    ekf_trajectory = np.array(ekf_trajectory)
    
    # Ensure same length
    min_len = min(len(ground_truth), len(ekf_trajectory))
    time = time[:min_len]
    ground_truth = ground_truth[:min_len]
    ekf_trajectory = ekf_trajectory[:min_len]
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    ground_type = "Non-Planar Ground" if non_planar_ground else "Flat Ground"
    fig.suptitle(f'RoboFly EKF Validation ({ground_type})')
    
    # State comparisons
    states = ['Theta (rad)', 'Vx (m/s)', 'Z (m)', 'Vz (m/s)']
    
    for i, (ax, state_name) in enumerate(zip(axes.flat, states)):
        ax.plot(time, ground_truth[:, i], 'b-', label='Ground Truth', linewidth=2)
        ax.plot(time, ekf_trajectory[:, i], 'r--', label='EKF Estimate', linewidth=1.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(state_name)
        ax.legend()
        ax.grid(True)
        
        # Compute and display RMSE for this state
        rmse = np.sqrt(np.mean((ground_truth[:, i] - ekf_trajectory[:, i])**2))
        ax.set_title(f'{state_name} (RMSE: {rmse:.4f})')
    
    plt.tight_layout()
    output_name = f'robofly_ekf_validation_{"nonplanar_ground" if non_planar_ground else "flat_ground"}.png'
    plt.savefig(output_name, dpi=300, bbox_inches='tight')
    print(f"Saved EKF validation plot to {output_name}")
    plt.show()

def visualize_terrain_types():
    """
    Create visualization of all available terrain types for comparison
    """
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    fig.suptitle('Available Terrain Types for RoboFly EKF Testing', fontsize=14)
    
    # Create a grid of x,z positions for visualization
    x_range = np.linspace(0, 2, 100)  # 2 meters
    z_range = np.linspace(0, 1.5, 75)  # 1.5 meters  
    X, Z = np.meshgrid(x_range, z_range)
    
    terrain_configs = [
        ("flat", None, "Flat Ground"),
        ("gradual_slope", {"slope_x": 0.03, "slope_z": 0.015}, "Gradual Slope (1.7°)"),
        ("steep_slope", {"slope_x": 0.1, "slope_z": 0.06}, "Steep Slope (5.7°)"),
        ("sinusoidal", {"amplitude": 0.025, "freq_x": 0.6, "freq_z": 0.4}, "Sinusoidal Hills"),
        ("random_smooth", {"amplitude": 0.015, "seed": 42}, "Random Smooth"),
        ("random_rough", {"amplitude": 0.025, "roughness": 0.01, "seed": 123}, "Random Rough"),
        ("step_terrain", {"step_height": 0.02, "step_size_x": 0.5, "step_size_z": 0.4}, "Step Terrain"),
        ("indoor_bumps", {"amplitude": 0.005, "cable_height": 0.003}, "Indoor Bumps")
    ]
    
    for idx, (terrain_type, terrain_params, title) in enumerate(terrain_configs):
        row = idx // 4
        col = idx % 4
        ax = axes[row, col]
        
        # Generate height map
        heights = np.zeros_like(X)
        for i in range(X.shape[0]):
            for j in range(X.shape[1]):
                heights[i, j] = generate_ground_height(X[i, j], Z[i, j], terrain_type, terrain_params)
        
        # Plot terrain
        im = ax.contourf(X, Z, heights, levels=20, cmap='terrain')
        ax.contour(X, Z, heights, levels=10, colors='black', alpha=0.3, linewidths=0.5)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Z (m)')
        ax.set_title(title)
        ax.set_aspect('equal')
        
        # Add colorbar for height scale
        cbar = plt.colorbar(im, ax=ax, shrink=0.8)
        cbar.set_label('Height (m)', fontsize=8)
        
        # Add statistics
        height_range = np.max(heights) - np.min(heights)
        ax.text(0.02, 0.98, f'Range: {height_range*1000:.1f}mm', 
                transform=ax.transAxes, fontsize=8, verticalalignment='top',
                bbox=dict(boxstyle="round,pad=0.2", facecolor="white", alpha=0.7))
    
    plt.tight_layout()
    plt.savefig('terrain_types_comparison.png', dpi=300, bbox_inches='tight')
    print("Saved terrain comparison plot to terrain_types_comparison.png")
    return fig

def analyze_terrain_effects(terrain_type, terrain_params=None, trajectory_length=2.0):
    """
    Analyze how different terrain types affect sensor measurements along a flight path
    """
    print(f"\\nAnalyzing terrain effects for {terrain_type}...")
    
    # Create a simple flight trajectory
    t = np.linspace(0, 5, 500)  # 5 second flight
    x_traj = 0.3 * t  # Move forward at 0.3 m/s
    z_traj = 0.1 + 0.05 * np.sin(2*np.pi*0.2*t)  # Small altitude variation around 10cm
    
    # Generate ground heights and sensor effects along trajectory
    ground_heights = []
    range_effects = []
    flow_effects = []
    
    for i in range(len(t)):
        # Ground height at this position
        ground_height = generate_ground_height(x_traj[i], z_traj[i], terrain_type, terrain_params)
        ground_heights.append(ground_height)
        
        # Range sensor effect (assumes level flight)
        actual_height = z_traj[i] - ground_height
        range_effect = actual_height - z_traj[i]  # Difference from flat ground assumption
        range_effects.append(range_effect)
        
        # Optical flow effect from ground slope
        if terrain_type != "flat":
            slope_x, slope_z = get_terrain_slope(x_traj[i], z_traj[i], terrain_type, terrain_params)
            # Assuming typical forward velocity of 0.3 m/s
            flow_effect = slope_x * 0.3 / (actual_height + 0.01)
            flow_effects.append(flow_effect)
        else:
            flow_effects.append(0.0)
    
    # Create analysis plot
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f'Terrain Effects Analysis: {terrain_type}', fontsize=14)
    
    # Flight trajectory with ground profile
    axes[0,0].plot(x_traj, z_traj, 'b-', label='Flight trajectory', linewidth=2)
    axes[0,0].fill_between(x_traj, ground_heights, alpha=0.3, color='brown', label='Ground profile')
    axes[0,0].axhline(y=0, color='gray', linestyle='--', alpha=0.5, label='Nominal ground')
    axes[0,0].set_xlabel('X position (m)')
    axes[0,0].set_ylabel('Height (m)')
    axes[0,0].set_title('Flight Path vs Ground Profile')
    axes[0,0].legend()
    axes[0,0].grid(True, alpha=0.3)
    
    # Ground height variations
    axes[0,1].plot(t, np.array(ground_heights)*1000, 'brown', linewidth=2)
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Ground Height (mm)')
    axes[0,1].set_title('Ground Height Variations')
    axes[0,1].grid(True, alpha=0.3)
    
    # Range sensor errors
    axes[1,0].plot(t, np.array(range_effects)*1000, 'r-', linewidth=2)
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Range Error (mm)')
    axes[1,0].set_title('ToF Range Sensor Error from Flat Ground Assumption')
    axes[1,0].grid(True, alpha=0.3)
    
    # Optical flow effects
    axes[1,1].plot(t, np.array(flow_effects), 'g-', linewidth=2)
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Flow Correction (rad/s)')
    axes[1,1].set_title('Optical Flow Ground Slope Correction')
    axes[1,1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Print statistics
    print(f"Ground height range: {(np.max(ground_heights) - np.min(ground_heights))*1000:.1f} mm")
    print(f"Max range error: {np.max(np.abs(range_effects))*1000:.1f} mm")
    print(f"Max flow correction: {np.max(np.abs(flow_effects)):.4f} rad/s")
    
    return fig

if __name__ == "__main__":
    main() 