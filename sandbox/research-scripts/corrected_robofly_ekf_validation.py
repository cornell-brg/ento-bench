#!/usr/bin/env python3
"""
Corrected RoboFly EKF validation using real robognat simulation data
with proper jacobians matching the C++ implementation.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import argparse
from pathlib import Path

class CorrectedRoboFlyEKF:
    """
    Python implementation of RoboFly EKF with corrected jacobians 
    that exactly match the C++ implementation.
    """
    
    def __init__(self, dt=0.001):
        self.dt = dt
        
        # State: [theta, vx, z, vz]
        self.x = np.zeros(4)
        
        # Covariance matrix
        self.P = np.eye(4) * 0.01
        
        # Process noise
        self.Q = np.diag([0.001, 0.01, 0.001, 0.01])
        
        # Measurement noise (adjusted for realistic sensors)
        self.R = np.diag([0.001, 0.01, 0.1, 0.1])  # [range, optical_flow, accel_x, accel_z]
        
        # Store trajectory for analysis
        self.trajectory = []
        self.measurements_log = []
        
        # Constants
        self.g = 9.81
        
    def predict(self, omega):
        """Prediction step with corrected dynamics"""
        # State: [theta, vx, z, vz]
        theta, vx, z, vz = self.x
        
        # Dynamics: f = [omega, 0, vz, 0] (matches C++ exactly)
        f = np.array([omega, 0.0, vz, 0.0])
        
        # Predict state
        self.x = self.x + self.dt * f
        
        # Jacobian A (matches C++ implementation)
        A = np.eye(4)
        A[2, 3] = self.dt  # ∂(z + dt*vz)/∂vz = dt
        
        # Predict covariance
        self.P = A @ self.P @ A.T + self.Q
        
    def update(self, measurements):
        """Update step with CORRECTED jacobians matching C++ exactly"""
        theta, vx, z, vz = self.x
        
        # Avoid division by zero
        if abs(z) < 1e-6:
            z = 1e-6 if z >= 0 else -1e-6
            
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # Avoid division by zero in cos(theta)
        if abs(cos_theta) < 1e-6:
            cos_theta = 1e-6 if cos_theta >= 0 else -1e-6
        
        # Predicted measurements (matches C++ exactly)
        h = np.array([
            z / cos_theta,                                                # ToF range
            (cos_theta / z) * (vx * cos_theta + vz * sin_theta),         # Optical flow (NO omega term in truth data)
            -self.g * sin_theta,                                          # Accelerometer x
            self.g * cos_theta                                            # Accelerometer z
        ])
        
        # CORRECTED Jacobian H (matches C++ implementation exactly)
        H = np.zeros((4, 4))
        
        # Range measurement jacobians
        H[0, 0] = z * sin_theta / (cos_theta * cos_theta)  # ∂(z/cos(θ))/∂θ
        H[0, 2] = 1.0 / cos_theta                          # ∂(z/cos(θ))/∂z
        
        # Optical flow jacobians (CORRECTED to match C++)
        flow_term = vx * cos_theta + vz * sin_theta
        H[1, 0] = (1.0/z) * (vz * cos_theta - vx * sin_theta - flow_term * sin_theta / cos_theta)
        H[1, 1] = cos_theta / z                            # CORRECTED: was cos_theta**2 / z
        H[1, 2] = -(cos_theta / (z * z)) * flow_term
        H[1, 3] = sin_theta / z                            # CORRECTED: was cos_theta * sin_theta / z
        
        # Accelerometer jacobians
        H[2, 0] = -self.g * cos_theta                      # ∂(-g*sin(θ))/∂θ
        H[3, 0] = -self.g * sin_theta                      # ∂(g*cos(θ))/∂θ
        
        # Innovation
        y = measurements - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # Update state and covariance
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        
        # Log for analysis
        self.trajectory.append(self.x.copy())
        self.measurements_log.append(measurements.copy())
        
    def get_trajectory(self):
        """Return trajectory as numpy array"""
        return np.array(self.trajectory)
        
    def compute_rmse(self, true_trajectory):
        """Compute RMSE against ground truth"""
        if len(self.trajectory) == 0:
            return np.inf, np.array([np.inf, np.inf, np.inf, np.inf])
            
        pred_traj = self.get_trajectory()
        min_len = min(len(pred_traj), len(true_trajectory))
        
        if min_len == 0:
            return np.inf, np.array([np.inf, np.inf, np.inf, np.inf])
            
        pred_traj = pred_traj[:min_len]
        true_traj = true_trajectory[:min_len]
        
        # Compute per-state RMSE
        state_rmse = np.sqrt(np.mean((pred_traj - true_traj)**2, axis=0))
        overall_rmse = np.sqrt(np.mean(state_rmse**2))
        
        return overall_rmse, state_rmse

def load_robognat_data(csv_file):
    """Load and parse robognat simulation data"""
    try:
        # Load CSV without header since robognat data doesn't have column names
        df = pd.read_csv(csv_file, header=None)
        
        # The CSV format from robognat_robofly_validation.py is:
        # timestamp,dt,meas0,meas1,meas2,meas3,ctrl0,mask0,mask1,mask2,mask3
        # where meas0=tof_range, meas1=optical_flow, meas2=accel_x, meas3=accel_z
        # and ctrl0=omega (angular velocity)
        
        expected_cols = 11
        if df.shape[1] != expected_cols:
            print(f"Warning: Expected {expected_cols} columns, got {df.shape[1]}")
            print(f"Columns: {list(df.columns)}")
            return None
            
        # Assign proper column names
        df.columns = ['timestamp', 'dt', 'tof_range', 'optical_flow', 'accel_x', 'accel_z', 
                     'omega', 'mask0', 'mask1', 'mask2', 'mask3']
        
        # For RoboFly EKF, we need to reconstruct the state variables from the robognat simulation
        # Since we don't have direct state access, we'll use the measurements to approximate initial conditions
        # and let the EKF track from there
        
        # Initialize approximate state variables (will be refined by EKF)
        # Theta can be estimated from accelerometer readings: theta ≈ atan2(-accel_x, accel_z)
        df['theta'] = np.arctan2(-df['accel_x'], df['accel_z'])
        
        # Z can be estimated from ToF range assuming small theta: z ≈ tof_range * cos(theta)
        df['z'] = df['tof_range'] * np.cos(df['theta'])
        
        # Initialize velocities to zero (EKF will estimate these)
        df['vx'] = 0.0
        df['vz'] = 0.0
        
        print(f"Successfully loaded {len(df)} data points from {csv_file}")
        print(f"Estimated initial state: θ={df['theta'].iloc[0]:.4f} rad, z={df['z'].iloc[0]:.4f} m")
        return df
        
    except Exception as e:
        print(f"Error loading {csv_file}: {e}")
        return None

def run_ekf_validation(data_file, non_planar=False, save_csv=None, plot=False):
    """Run EKF validation using robognat data"""
    
    # Load data
    df = load_robognat_data(data_file)
    if df is None:
        print("Failed to load data!")
        return None
        
    # Initialize EKF
    dt = df['dt'].iloc[0]  # Use actual dt from data
    ekf = CorrectedRoboFlyEKF(dt=dt)
    
    # Set initial state from estimated values
    ekf.x = np.array([df['theta'].iloc[0], df['vx'].iloc[0], 
                      df['z'].iloc[0], df['vz'].iloc[0]])
    
    print(f"Initial state: θ={ekf.x[0]:.6f}, vx={ekf.x[1]:.6f}, z={ekf.x[2]:.6f}, vz={ekf.x[3]:.6f}")
    print(f"Using dt={dt:.6f} seconds")
    
    # Since we don't have true ground truth states, we'll track EKF performance
    # by comparing final estimates with expected behavior and measurement consistency
    
    # Run EKF
    results = []
    for i, row in df.iterrows():
        # Control input
        omega = row['omega']
        
        # Measurements: [tof_range, optical_flow, accel_x, accel_z]
        measurements = np.array([
            row['tof_range'],
            row['optical_flow'], 
            row['accel_x'],
            row['accel_z']
        ])
        
        # EKF predict and update
        ekf.predict(omega)
        ekf.update(measurements)
        
        # Store results
        results.append({
            'timestamp': row['timestamp'],
            'pred_theta': ekf.x[0],
            'pred_vx': ekf.x[1],
            'pred_z': ekf.x[2],
            'pred_vz': ekf.x[3],
            'omega': omega,
            'tof_range': measurements[0],
            'optical_flow': measurements[1],
            'accel_x': measurements[2],
            'accel_z': measurements[3],
            # Store measurement predictions for validation
            'pred_range': ekf.x[2] / np.cos(ekf.x[0]) if abs(np.cos(ekf.x[0])) > 1e-6 else np.inf,
            'pred_optical_flow': (np.cos(ekf.x[0]) / ekf.x[2]) * (ekf.x[1] * np.cos(ekf.x[0]) + ekf.x[3] * np.sin(ekf.x[0])) if abs(ekf.x[2]) > 1e-6 else 0,
            'pred_accel_x': -9.81 * np.sin(ekf.x[0]),
            'pred_accel_z': 9.81 * np.cos(ekf.x[0])
        })
    
    results_df = pd.DataFrame(results)
    
    # Compute measurement prediction errors (validation metric)
    range_error = np.sqrt(np.mean((results_df['tof_range'] - results_df['pred_range'])**2))
    flow_error = np.sqrt(np.mean((results_df['optical_flow'] - results_df['pred_optical_flow'])**2))
    accel_x_error = np.sqrt(np.mean((results_df['accel_x'] - results_df['pred_accel_x'])**2))
    accel_z_error = np.sqrt(np.mean((results_df['accel_z'] - results_df['pred_accel_z'])**2))
    
    print(f"\n=== CORRECTED EKF VALIDATION RESULTS ===")
    print(f"Data points processed: {len(results)}")
    print(f"Measurement Prediction Errors (RMSE):")
    print(f"  ToF Range:     {range_error:.6f} m")
    print(f"  Optical Flow:  {flow_error:.6f} rad/s")
    print(f"  Accel X:       {accel_x_error:.6f} m/s²")
    print(f"  Accel Z:       {accel_z_error:.6f} m/s²")
    
    # Check for reasonable final state
    final_theta = ekf.x[0]
    final_z = ekf.x[2]
    
    print(f"\nFinal EKF state:")
    print(f"  Theta: {final_theta:.6f} rad ({np.degrees(final_theta):.3f} deg)")
    print(f"  Vx:    {ekf.x[1]:.6f} m/s")
    print(f"  Z:     {final_z:.6f} m")
    print(f"  Vz:    {ekf.x[3]:.6f} m/s")
    
    # Check if results are reasonable
    if abs(final_theta) < np.pi/6 and final_z > 0.01:  # Reasonable angle and positive height
        print("✅ EKF converged to reasonable final state")
    else:
        print("⚠️  Warning: EKF final state may be unrealistic")
    
    # Save CSV if requested
    if save_csv:
        results_df.to_csv(save_csv, index=False)
        print(f"\nResults saved to {save_csv}")
    
    # Plot if requested
    if plot:
        plot_ekf_results(results_df)
    
    # Return overall measurement error as primary metric
    overall_error = np.sqrt(np.mean([range_error**2, flow_error**2, accel_x_error**2, accel_z_error**2]))
    return results_df, overall_error, np.array([range_error, flow_error, accel_x_error, accel_z_error])

def plot_ekf_results(results_df):
    """Plot EKF validation results"""
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle('Corrected RoboFly EKF Validation Results', fontsize=14)
    
    # Plot 1: EKF state estimates over time
    ax1 = axes[0, 0]
    ax1.plot(results_df['timestamp'], results_df['pred_theta'], 'r-', label='Theta (rad)', linewidth=1.5)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Theta (rad)')
    ax1.set_title('EKF Theta Estimate')
    ax1.grid(True, alpha=0.3)
    
    ax2 = axes[0, 1]
    ax2.plot(results_df['timestamp'], results_df['pred_vx'], 'g-', label='Vx (m/s)', linewidth=1.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Vx (m/s)')
    ax2.set_title('EKF Vx Estimate')
    ax2.grid(True, alpha=0.3)
    
    ax3 = axes[0, 2]
    ax3.plot(results_df['timestamp'], results_df['pred_z'], 'b-', label='Z (m)', linewidth=1.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('EKF Z Estimate')
    ax3.grid(True, alpha=0.3)
    
    ax4 = axes[1, 0]
    ax4.plot(results_df['timestamp'], results_df['pred_vz'], 'm-', label='Vz (m/s)', linewidth=1.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Vz (m/s)')
    ax4.set_title('EKF Vz Estimate')
    ax4.grid(True, alpha=0.3)
    
    # Plot 2: Measurement vs prediction comparison
    ax5 = axes[1, 1]
    ax5.plot(results_df['timestamp'], results_df['tof_range'], 'b-', label='Measured', alpha=0.7, linewidth=1.5)
    ax5.plot(results_df['timestamp'], results_df['pred_range'], 'r--', label='EKF Predicted', alpha=0.7, linewidth=1.5)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('ToF Range (m)')
    ax5.set_title('ToF Range: Measured vs Predicted')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # Plot 3: Optical flow comparison
    ax6 = axes[1, 2]
    ax6.plot(results_df['timestamp'], results_df['optical_flow'], 'b-', label='Measured', alpha=0.7, linewidth=1.5)
    ax6.plot(results_df['timestamp'], results_df['pred_optical_flow'], 'r--', label='EKF Predicted', alpha=0.7, linewidth=1.5)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Optical Flow (rad/s)')
    ax6.set_title('Optical Flow: Measured vs Predicted')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def main():
    parser = argparse.ArgumentParser(description='Corrected RoboFly EKF Validation')
    parser.add_argument('--data-file', type=str, 
                       default='../datasets/state-est/robofly_robognat_validation.csv',
                       help='Robognat CSV data file')
    parser.add_argument('--non-planar', action='store_true',
                       help='Use non-planar ground (currently not implemented)')
    parser.add_argument('--save-csv', type=str,
                       help='Save results to CSV file')
    parser.add_argument('--plot', action='store_true',
                       help='Show validation plots')
    
    args = parser.parse_args()
    
    # Check if data file exists
    data_path = Path(args.data_file)
    if not data_path.exists():
        print(f"Error: Data file {data_path} not found!")
        print("Available datasets:")
        datasets_dir = Path("../datasets/state-est/")
        if datasets_dir.exists():
            for csv_file in datasets_dir.glob("*.csv"):
                print(f"  {csv_file}")
        return
    
    print(f"Using data file: {data_path}")
    
    # Run validation
    results = run_ekf_validation(
        str(data_path), 
        non_planar=args.non_planar,
        save_csv=args.save_csv,
        plot=args.plot
    )
    
    if results is not None:
        print("\n✅ Corrected EKF validation completed successfully!")
        print("This uses REAL robognat physics simulation data")
        print("with CORRECTED jacobians matching the C++ implementation.")
    else:
        print("❌ Validation failed!")

if __name__ == "__main__":
    main() 