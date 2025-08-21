#!/usr/bin/env python3
"""
Simple standalone RoboFly EKF test with non-planar ground effects
No external dependencies - generates synthetic trajectory and tests EKF
"""

import numpy as np
import matplotlib.pyplot as plt
import argparse
import csv
from numpy import array, sin, cos, pi, sqrt
from numpy.random import normal

# Constants
DT = 1./200  # 200 Hz sampling
g = 9.81

class PythonRoboFlyEKF:
    """
    Python implementation of RoboFly EKF for validation against C++ version
    State: [theta, vx, z, vz]
    Measurements: [range, optical_flow, accel_x, accel_z]
    """
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        self.x = np.array(initial_state)  # State vector
        self.P = np.array(initial_covariance)  # Covariance matrix
        self.Q = np.array(process_noise)  # Process noise
        self.R = np.array(measurement_noise)  # Measurement noise
        
        # Store trajectory for RMSE computation
        self.trajectory = [self.x.copy()]
        self.covariances = [self.P.copy()]
        
    def predict(self, u, dt):
        """Prediction step with dynamics model"""
        theta, vx, z, vz = self.x
        omega = u[0]
        
        # Dynamics: x' = f(x, u) * dt
        f = np.array([omega, 0, vz, 0])
        self.x = self.x + f * dt
        
        # Jacobian F = df/dx
        F = np.array([
            [0, 0, 0, 0],  # d(theta')/d[theta, vx, z, vz] = [0, 0, 0, 0]
            [0, 0, 0, 0],  # d(vx')/d[theta, vx, z, vz] = [0, 0, 0, 0]  
            [0, 0, 0, 1],  # d(z')/d[theta, vx, z, vz] = [0, 0, 0, 1]
            [0, 0, 0, 0]   # d(vz')/d[theta, vx, z, vz] = [0, 0, 0, 0]
        ])
        
        # Covariance prediction: P = F*P*F^T + Q
        self.P = F @ self.P @ F.T + self.Q * dt
        
    def update(self, z_meas, omega):
        """Measurement update step"""
        theta, vx, z, vz = self.x
        
        cos_theta = cos(theta)
        sin_theta = sin(theta)
        
        # Predicted measurements h(x)
        h = np.array([
            z / cos_theta,  # Range
            (cos_theta / z) * (vx * cos_theta + vz * sin_theta) - omega,  # Optical flow
            -g * sin_theta,  # Accel X
            g * cos_theta   # Accel Z
        ])
        
        # Measurement Jacobian H = dh/dx
        if abs(cos_theta) < 0.01:  # Avoid singularity
            cos_theta = 0.01 if cos_theta >= 0 else -0.01
            
        H = np.array([
            [z * sin_theta / (cos_theta**2), 0, 1/cos_theta, 0],  # dh0/dx
            [-(sin_theta/z) * (vx*cos_theta + vz*sin_theta) - (cos_theta/z) * (-vx*sin_theta + vz*cos_theta), 
             cos_theta**2 / z, 
             -(cos_theta/z**2) * (vx*cos_theta + vz*sin_theta), 
             cos_theta * sin_theta / z],  # dh1/dx (optical flow)
            [-g * cos_theta, 0, 0, 0],  # dh2/dx
            [g * sin_theta, 0, 0, 0]    # dh3/dx
        ])
        
        # Innovation
        y = z_meas - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update
        self.P = (np.eye(4) - K @ H) @ self.P
        
        # Store trajectory
        self.trajectory.append(self.x.copy())
        self.covariances.append(self.P.copy())

def generate_ground_height(x, z, non_planar=False):
    """
    Generate ground height at position (x, z)
    If non_planar=True, ground has slopes and variations
    If non_planar=False, ground is perfectly flat at z=0
    """
    if non_planar:
        # Non-planar ground: sinusoidal terrain with slopes
        # Create realistic terrain variations for indoor flight
        slope_amplitude = 0.02  # 2cm height variations
        slope_frequency_x = 0.5  # cycles per meter in x
        slope_frequency_z = 0.3  # cycles per meter in z
        
        ground_height = (slope_amplitude * sin(2*pi*slope_frequency_x * x) * 
                        cos(2*pi*slope_frequency_z * z))
        
        # Add some random roughness
        roughness = 0.005 * sin(10*x) * cos(8*z)  # 5mm roughness
        
        return ground_height + roughness
    else:
        # Planar ground: perfectly flat
        return 0.0

def generate_synthetic_trajectory(tfinal=10.0):
    """Generate a simple synthetic RoboFly trajectory"""
    time = np.arange(0, tfinal, DT)
    n_samples = len(time)
    
    # Simple hovering trajectory with small movements
    states = np.zeros((n_samples, 7))  # [theta, omega, x, vx, z, vz, vw]
    controls = np.zeros((n_samples, 1))  # [omega]
    
    for i, t in enumerate(time):
        # Simple oscillating trajectory
        states[i, 0] = 0.05 * sin(0.5 * t)  # theta: small pitch oscillation
        states[i, 1] = 0.05 * 0.5 * cos(0.5 * t)  # omega: derivative of theta
        states[i, 2] = 0.02 * t  # x: slow forward movement
        states[i, 3] = 0.02  # vx: constant forward velocity
        states[i, 4] = 0.12 + 0.02 * sin(0.2 * t)  # z: hover with small vertical oscillation
        states[i, 5] = 0.02 * 0.2 * cos(0.2 * t)  # vz: derivative of z
        states[i, 6] = 0  # vw: unused
        
        controls[i, 0] = states[i, 1]  # control is angular velocity
    
    return time, states, controls

def generate_sensor_measurements(states, non_planar_ground=False, noisy=True):
    """Generate sensor measurements from true states"""
    n_samples = len(states)
    measurements = np.zeros((n_samples, 4))
    
    for i in range(n_samples):
        theta = states[i, 0]
        omega = states[i, 1]
        x = states[i, 2]
        vx = states[i, 3]
        z = states[i, 4]
        vz = states[i, 5]
        
        cos_theta = cos(theta)
        sin_theta = sin(theta)
        
        # Ground height at current position
        ground_height = generate_ground_height(x, z, non_planar_ground)
        actual_height_above_ground = z - ground_height
        
        # ToF range measurement
        if abs(cos_theta) > 0.01:
            range_sensor = actual_height_above_ground / cos_theta
        else:
            range_sensor = 100.0
        
        # Optical flow
        if non_planar_ground:
            # Ground slopes affect optical flow
            ground_slope_x = 0.02 * 2*pi*0.5 * cos(2*pi*0.5 * x)
            flow_correction = ground_slope_x * vx / (z + 0.01)
            optical_flow = omega - vx * cos_theta / actual_height_above_ground + flow_correction
        else:
            optical_flow = omega - vx * cos_theta / z
        
        # Accelerometers
        accel_x = -g * sin_theta
        accel_z = g * cos_theta
        
        measurements[i] = [range_sensor, optical_flow, accel_x, accel_z]
        
        # Add noise
        if noisy:
            measurements[i, 0] += normal(0, 0.001)  # 1mm range noise
            measurements[i, 1] += normal(0, 0.05)   # 0.05 rad/s flow noise
            measurements[i, 2] += normal(0, 0.1)    # 0.1 m/s^2 accel noise
            measurements[i, 3] += normal(0, 0.1)    # 0.1 m/s^2 accel noise
    
    return measurements

def compute_rmse(estimated_trajectory, ground_truth_trajectory):
    """Compute Root Mean Square Error between estimated and ground truth trajectories"""
    estimated = np.array(estimated_trajectory)
    ground_truth = np.array(ground_truth_trajectory)
    
    # Ensure same length
    min_len = min(len(estimated), len(ground_truth))
    estimated = estimated[:min_len]
    ground_truth = ground_truth[:min_len]
    
    # Compute RMSE for each state
    rmse = np.sqrt(np.mean((estimated - ground_truth)**2, axis=0))
    
    return {
        'theta_rmse': rmse[0],
        'vx_rmse': rmse[1], 
        'z_rmse': rmse[2],
        'vz_rmse': rmse[3],
        'overall_rmse': np.sqrt(np.mean(rmse**2))
    }

def run_ekf_test(tfinal=10.0, non_planar_ground=False, noisy=True):
    """Run complete EKF test and return results"""
    print(f"Generating synthetic trajectory ({tfinal:.1f}s)...")
    time, states, controls = generate_synthetic_trajectory(tfinal)
    
    print(f"Generating sensor measurements (non_planar_ground={non_planar_ground}, noisy={noisy})...")
    measurements = generate_sensor_measurements(states, non_planar_ground, noisy)
    
    # Ground truth for RoboFly states [theta, vx, z, vz]
    ground_truth = []
    for i in range(len(states)):
        gt_state = [states[i, 0], states[i, 3], states[i, 4], states[i, 5]]
        ground_truth.append(gt_state)
    
    print("Running Python EKF...")
    # EKF parameters
    initial_state = [0.0, 0.0, 0.12, 0.0]  # [theta, vx, z, vz]
    initial_covariance = np.eye(4) * 0.001
    process_noise = np.eye(4) * 0.001
    measurement_noise = np.diag([0.001, 0.05, 0.1, 0.1])
    
    ekf = PythonRoboFlyEKF(initial_state, initial_covariance, process_noise, measurement_noise)
    
    # Run EKF
    for i in range(len(time)):
        u = [controls[i, 0]]
        z_meas = measurements[i]
        
        ekf.predict(u, DT)
        ekf.update(z_meas, u[0])
    
    # Compute RMSE
    rmse_results = compute_rmse(ekf.trajectory, ground_truth)
    
    # Define ground type for results
    ground_type = "Non-Planar Ground" if non_planar_ground else "Flat Ground"
    print(f"\n=== EKF Results ({ground_type}) ===")
    print(f"Theta RMSE: {rmse_results['theta_rmse']:.6f} rad ({rmse_results['theta_rmse']*180/pi:.3f} deg)")
    print(f"Vx RMSE: {rmse_results['vx_rmse']:.6f} m/s")
    print(f"Z RMSE: {rmse_results['z_rmse']:.6f} m")
    print(f"Vz RMSE: {rmse_results['vz_rmse']:.6f} m/s")
    print(f"Overall RMSE: {rmse_results['overall_rmse']:.6f}")
    
    return time, ground_truth, ekf.trajectory, measurements, rmse_results

def save_csv_data(time, ground_truth, measurements, controls, filename):
    """Save data in RoboFly EKF format"""
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        for i in range(len(time)):
            timestamp = time[i]
            dt = DT
            
            # Measurements: [range, optical_flow, accel_x, accel_z]
            meas0, meas1, meas2, meas3 = measurements[i]
            
            # Control: angular velocity
            ctrl0 = controls[i, 0] if i < len(controls) else 0.0
            
            # Sensor mask: all sensors active
            mask0, mask1, mask2, mask3 = 1, 1, 1, 1
            
            writer.writerow([timestamp, dt, meas0, meas1, meas2, meas3, ctrl0, mask0, mask1, mask2, mask3])
    
    print(f"Saved data to {filename}")

def plot_results(time, ground_truth, ekf_trajectory, non_planar_ground=False):
    """Plot EKF results"""
    ground_truth = np.array(ground_truth)
    ekf_trajectory = np.array(ekf_trajectory)
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    ground_type = "Non-Planar Ground" if non_planar_ground else "Flat Ground"
    fig.suptitle(f'RoboFly EKF Validation ({ground_type})')
    
    states = ['Theta (rad)', 'Vx (m/s)', 'Z (m)', 'Vz (m/s)']
    
    for i, (ax, state_name) in enumerate(zip(axes.flat, states)):
        ax.plot(time, ground_truth[:, i], 'b-', label='Ground Truth', linewidth=2)
        ax.plot(time, ekf_trajectory[:, i], 'r--', label='EKF Estimate', linewidth=1.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(state_name)
        ax.legend()
        ax.grid(True)
        
        rmse = np.sqrt(np.mean((ground_truth[:, i] - ekf_trajectory[:, i])**2))
        ax.set_title(f'{state_name} (RMSE: {rmse:.4f})')
    
    plt.tight_layout()
    output_name = f'simple_robofly_ekf_{"nonplanar" if non_planar_ground else "planar"}.png'
    plt.savefig(output_name, dpi=300, bbox_inches='tight')
    print(f"Saved plot to {output_name}")
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Simple RoboFly EKF Test with Non-Planar Ground')
    parser.add_argument('--tfinal', type=float, default=10.0, help='Simulation time (seconds)')
    parser.add_argument('--non-planar-ground', action='store_true', help='Enable non-planar ground effects')
    parser.add_argument('--no-noise', action='store_true', help='Disable sensor noise')
    parser.add_argument('--save-csv', type=str, help='Save data to CSV file')
    parser.add_argument('--plot', action='store_true', help='Show plots')
    
    args = parser.parse_args()
    
    # Run EKF test
    time, ground_truth, ekf_trajectory, measurements, rmse_results = run_ekf_test(
        tfinal=args.tfinal,
        non_planar_ground=args.non_planar_ground,
        noisy=not args.no_noise
    )
    
    # Save CSV if requested
    if args.save_csv:
        # Generate controls array
        controls = np.zeros((len(time), 1))
        for i in range(len(time)):
            if i < len(ground_truth):
                # Use ground truth angular velocity (derivative of theta)
                if i > 0:
                    controls[i, 0] = (ground_truth[i][0] - ground_truth[i-1][0]) / DT
                else:
                    controls[i, 0] = 0.0
        
        save_csv_data(time, ground_truth, measurements, controls, args.save_csv)
    
    # Plot if requested
    if args.plot:
        plot_results(time, ground_truth, ekf_trajectory, args.non_planar_ground)

if __name__ == "__main__":
    main() 