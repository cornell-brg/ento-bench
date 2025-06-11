import argparse as ap
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
import ahrs
import matplotlib.pyplot as plt
from itertools import product
from tqdm import tqdm

def gt_from_golden():
    golden = np.genfromtxt("golden.csv", delimiter=",")
    data = golden[1:, :]
    gt = np.zeros((data.shape[0], 4))
    gt[:, :3] = data[:, 8:11]  # Roll, Pitch, Yaw (in degrees)
    gt[:, 3] = data[:, 7]      # Z position (not needed for quaternion conversion)
    t = data[:, 0]             # Timestamps
    return gt, t

def quats_from_gt(gt):
    r = R.from_euler('ZYX', gt[:, :3], degrees=True)  # Convert ground truth to quaternions
    return r.as_quat(scalar_first=True)

def compute_errors(gt_quats, estimated_quats):
    return np.array([ahrs.utils.qad(gt_quats[i], estimated_quats[i]) for i in range(len(gt_quats))])

def generate_grid(param_ranges, step_sizes):
    return {key: np.arange(val[0], val[1] + step_sizes[key], step_sizes[key]).tolist() for key, val in param_ranges.items()}

def run_filter_test(gt_quats, gyros, accels, mags, filter_class, param_grid, dt, use_marg=True):
    best_params = None
    best_error = float('inf')
    best_Q = None
    param_combinations = list(product(*param_grid.values()))
    
    for params in tqdm(param_combinations, desc=f'Tuning {filter_class.__name__} ({"MARG" if use_marg else "IMU"})'): 
        param_dict = dict(zip(param_grid.keys(), params))
        param_dict['q0'] = gt_quats[0]  # Set initial quaternion
        filter_instance = filter_class(**param_dict)
        
        Q = np.tile(gt_quats[0], (len(gyros), 1))  # Seed initial quaternion from ground truth
        for t in range(1, len(gyros)):
            if isinstance(filter_instance, ahrs.filters.fourati.Fourati):
                Q[t] = filter_instance.update(Q[t-1], gyr=gyros[t], acc=accels[t], mag=mags[t], dt=dt)
            elif use_marg:
                Q[t] = filter_instance.updateMARG(Q[t-1], gyr=gyros[t], acc=accels[t], mag=mags[t], dt=dt)
            else:
                Q[t] = filter_instance.updateIMU(Q[t-1], gyr=gyros[t], acc=accels[t], dt=dt)
        
        errors = compute_errors(gt_quats, Q)
        mean_error = np.mean(errors)
        
        if mean_error < best_error:
            best_error = mean_error
            best_params = param_dict
            best_Q = Q
            print(f'\rNew best {filter_class.__name__} ({"MARG" if use_marg else "IMU"}) params: {best_params}, Error: {best_error}', end='', flush=True)
    
    print(f'\nBest params for {filter_class.__name__} ({"MARG" if use_marg else "IMU"}): {best_params}, Error: {best_error}')
    return best_Q, best_params

def plot_euler_errors(gt_quats, Q_madgwick, Q_mahony, Q_fourati, time):
    gt_euler = R.from_quat(gt_quats).as_euler('XYZ', degrees=True)
    madgwick_euler = R.from_quat(Q_madgwick).as_euler('XYZ', degrees=True)
    mahony_euler = R.from_quat(Q_mahony).as_euler('XYZ', degrees=True)
    fourati_euler = R.from_quat(Q_fourati).as_euler('XYZ', degrees=True)

    labels = ['Roll', 'Pitch', 'Yaw']
    fig, axes = plt.subplots(3, 1, figsize=(15, 8))
    
    for i in range(3):
        axes[i].plot(time, gt_euler[:, i], label='Ground Truth', linewidth=2, linestyle='dashed')
        axes[i].plot(time, madgwick_euler[:, i], label='Madgwick', linewidth=1.5)
        axes[i].plot(time, mahony_euler[:, i], label='Mahony', linewidth=1.5)
        axes[i].plot(time, fourati_euler[:, i], label='Fourati', linewidth=1.5)
        
        axes[i].set_xlabel('Time [s]')
        axes[i].set_ylabel(f'{labels[i]} [deg]')
        axes[i].set_title(f'{labels[i]} Comparison')
        axes[i].legend()
        axes[i].grid(True)

    plt.tight_layout()
    plt.show()

def print_sensor_ranges(gyros, accels, mags):
    """Print sensor value ranges in the format matching real robot data analysis"""
    
    # Convert magnetometer from nT to µT for analysis
    mags_ut = mags / 1000.0
    
    print("\n=== SENSOR VALUE RANGES ===")
    print("Format: ax, ay, az, gx, gy, gz, mx, my, mz")
    print("Units: accel=m/s², gyro=rad/s, mag=µT")
    print()
    
    # Print minimums
    min_vals = [
        accels[:, 0].min(), accels[:, 1].min(), accels[:, 2].min(),  # accel x,y,z
        gyros[:, 0].min(), gyros[:, 1].min(), gyros[:, 2].min(),     # gyro x,y,z
        mags_ut[:, 0].min(), mags_ut[:, 1].min(), mags_ut[:, 2].min() # mag x,y,z
    ]
    
    # Print maximums
    max_vals = [
        accels[:, 0].max(), accels[:, 1].max(), accels[:, 2].max(),  # accel x,y,z
        gyros[:, 0].max(), gyros[:, 1].max(), gyros[:, 2].max(),     # gyro x,y,z
        mags_ut[:, 0].max(), mags_ut[:, 1].max(), mags_ut[:, 2].max() # mag x,y,z
    ]
    
    print("Minimums:")
    print("\t".join([f"{val:.6f}" for val in min_vals]))
    print("Maximums:")
    print("\t".join([f"{val:.6f}" for val in max_vals]))
    
    print("\nIndividual sensor ranges:")
    print(f"Accelerometer X: [{accels[:, 0].min():.6f}, {accels[:, 0].max():.6f}] m/s²")
    print(f"Accelerometer Y: [{accels[:, 1].min():.6f}, {accels[:, 1].max():.6f}] m/s²")
    print(f"Accelerometer Z: [{accels[:, 2].min():.6f}, {accels[:, 2].max():.6f}] m/s²")
    print(f"Gyroscope X:     [{gyros[:, 0].min():.6f}, {gyros[:, 0].max():.6f}] rad/s")
    print(f"Gyroscope Y:     [{gyros[:, 1].min():.6f}, {gyros[:, 1].max():.6f}] rad/s")
    print(f"Gyroscope Z:     [{gyros[:, 2].min():.6f}, {gyros[:, 2].max():.6f}] rad/s")
    print(f"Magnetometer X:  [{mags_ut[:, 0].min():.6f}, {mags_ut[:, 0].max():.6f}] µT")
    print(f"Magnetometer Y:  [{mags_ut[:, 1].min():.6f}, {mags_ut[:, 1].max():.6f}] µT")
    print(f"Magnetometer Z:  [{mags_ut[:, 2].min():.6f}, {mags_ut[:, 2].max():.6f}] µT")
    
    # Calculate and print ranges (max - min)
    print("\nValue ranges (max - min):")
    print(f"Accelerometer: [{accels.max() - accels.min():.6f}] m/s² span")
    print(f"Gyroscope:     [{gyros.max() - gyros.min():.6f}] rad/s span")
    print(f"Magnetometer:  [{mags_ut.max() - mags_ut.min():.6f}] µT span")

def save_quaternions(filename, time, quats):
    data = np.column_stack((time, quats))
    header = "Attitude Estimation Problem\ntimestamp,qw,qx,qy,qz"
    # Use fixed decimal notation, not scientific
    np.savetxt(filename, data, delimiter=",", header=header, comments="", fmt='%.6f')

def save_sensor_data(filename, time, gyros, accels, mags):
    # Convert magnetometer from nT (AHRS output) to µT (C++ algorithms expect µT)
    # 1 µT = 1000 nT, so divide by 1000
    mags_ut = mags / 1000.0  # Convert from nT to µT
    data = np.hstack((time[:, None], gyros, accels, mags_ut))
    header = "Attitude Estimation Problem\ntimestamp,gyr_x,gyr_y,gyr_z,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z"
    # Use fixed decimal notation, not scientific
    np.savetxt(filename, data, delimiter=",", header=header, comments="", fmt='%.6f')

def main():
    parser = ap.ArgumentParser(description='Generate realistic MARG sensor data from golden ground truth')
    
    # Parameter tuning controls
    parser.add_argument('--step-gain', type=float, default=0.01, help='Step size for gain parameter')
    parser.add_argument('--step-kp', type=float, default=0.1, help='Step size for Mahony kp parameter')
    parser.add_argument('--step-ki', type=float, default=0.05, help='Step size for Mahony ki parameter')
    
    # Sensor model selection
    parser.add_argument('--sensor-model', type=str, default='ICM-42688-P', 
                       choices=['ICM-20948', 'ICM-42688-P'], 
                       help='IMU sensor model to simulate')
    
    # Sampling rate and noise parameters (sensor-specific defaults)
    parser.add_argument('--sample-rate', type=float, default=1000.0, help='IMU sampling rate (Hz) - affects noise calculation')
    parser.add_argument('--interpolate', action='store_true', help='Interpolate golden.csv data to match sample rate (required for >250Hz)')
    
    # Sensor-specific noise density parameters (will be set based on sensor model)
    parser.add_argument('--gyr-noise-density', type=float, default=None, help='Gyroscope noise density (mdps/√Hz) - auto-set by sensor model')
    parser.add_argument('--acc-noise-density', type=float, default=None, help='Accelerometer noise density (µg/√Hz) - auto-set by sensor model')
    parser.add_argument('--mag-noise', type=float, default=0.1, help='Magnetometer noise RMS (µT) - typical MEMS: 0.1')
    
    # Bias parameters (ICM-42688-P: gyro ±5mdps/°C, assume ±20°C temp variation)
    parser.add_argument('--gyr-bias', type=float, default=0.00175, help='Gyroscope bias drift (rad/s) - ICM-42688-P: 0.00175 (±5mdps/°C * ±20°C)')
    parser.add_argument('--acc-bias', type=float, default=0.05, help='Accelerometer bias (m/s²) - typical MEMS: 0.05')
    
    # Legacy parameters (calculated from density if not specified)
    parser.add_argument('--gyr-noise', type=float, default=None, help='Gyroscope noise RMS (rad/s) - auto-calculated from density if not specified')
    parser.add_argument('--acc-noise', type=float, default=None, help='Accelerometer noise RMS (m/s²) - auto-calculated from density if not specified')
    
    # Parameter ranges (can be customized)
    parser.add_argument('--madgwick-gain-min', type=float, default=0.001, help='Minimum Madgwick gain')
    parser.add_argument('--madgwick-gain-max', type=float, default=0.5, help='Maximum Madgwick gain')
    parser.add_argument('--mahony-kp-min', type=float, default=0.1, help='Minimum Mahony Kp')
    parser.add_argument('--mahony-kp-max', type=float, default=2.0, help='Maximum Mahony Kp')
    parser.add_argument('--mahony-ki-min', type=float, default=0.01, help='Minimum Mahony Ki')
    parser.add_argument('--mahony-ki-max', type=float, default=0.5, help='Maximum Mahony Ki')
    parser.add_argument('--fourati-gain-min', type=float, default=0.05, help='Minimum Fourati gain')
    parser.add_argument('--fourati-gain-max', type=float, default=0.2, help='Maximum Fourati gain')
    
    # Control flags
    parser.add_argument('--debug', action='store_true', help='Run in debug mode with a small subset of data')
    parser.add_argument('--no-tuning', action='store_true', help='Skip parameter tuning, use defaults')
    parser.add_argument('--no-plots', action='store_true', help='Skip plotting')
    parser.add_argument('--output-prefix', type=str, default='', help='Prefix for output files')
    
    args = parser.parse_args()
    
    # Set sensor-specific noise density defaults if not specified
    if args.gyr_noise_density is None or args.acc_noise_density is None:
        if args.sensor_model == 'ICM-20948':
            # ICM-20948 specifications from transition guide Table 1
            if args.gyr_noise_density is None:
                args.gyr_noise_density = 15.0  # mdps/√Hz
            if args.acc_noise_density is None:
                args.acc_noise_density = 230.0  # µg/√Hz
        elif args.sensor_model == 'ICM-42688-P':
            # ICM-42688-P specifications 
            if args.gyr_noise_density is None:
                args.gyr_noise_density = 2.8  # mdps/√Hz
            if args.acc_noise_density is None:
                args.acc_noise_density = 65.0  # µg/√Hz (XY: 65, Z: 70, using average)
    
    # Calculate noise levels from datasheet specifications
    if args.gyr_noise is None:
        # Convert: mdps/√Hz -> rad/s RMS at given sample rate
        # 2.8 mdps/√Hz * √(sample_rate) * (π/180/1000) = rad/s RMS
        sqrt_sample_rate = np.sqrt(args.sample_rate)
        gyr_noise_mdps = args.gyr_noise_density * sqrt_sample_rate  # mdps RMS
        gyr_noise_rads = gyr_noise_mdps * np.pi / 180.0 / 1000.0    # convert to rad/s
        args.gyr_noise = gyr_noise_rads
    
    if args.acc_noise is None:
        # Convert: µg/√Hz -> m/s² RMS at given sample rate  
        # 65 µg/√Hz * √(sample_rate) * 9.81e-6 = m/s² RMS
        sqrt_sample_rate = np.sqrt(args.sample_rate)
        acc_noise_ug = args.acc_noise_density * sqrt_sample_rate     # µg RMS
        acc_noise_ms2 = acc_noise_ug * 9.81e-6                      # convert to m/s²
        args.acc_noise = acc_noise_ms2
    
    print(f"Using {args.sensor_model} noise model at {args.sample_rate}Hz:")
    print(f"  Gyro noise: {args.gyr_noise:.6f} rad/s RMS ({args.gyr_noise*180/np.pi*1000:.1f} mdps RMS)")
    print(f"  Accel noise: {args.acc_noise:.6f} m/s² RMS ({args.acc_noise/9.81e-6:.0f} µg RMS)")
    print(f"  Mag noise: {args.mag_noise:.3f} µT RMS")

    gt, time = gt_from_golden()
    gt_quats = quats_from_gt(gt)

    print(f'First quat: {gt_quats[1]}')
    
    # Handle data length and interpolation
    if args.debug:
        time = time[1:100]  # Use more samples for debug (was only 3)
        gt_quats = gt_quats[1:100]
    else:
        time = time[1:]
        gt_quats = gt_quats[1:]
    
    # Check if interpolation is needed
    original_dt = np.mean(np.diff(time))
    target_dt = 1.0 / args.sample_rate
    original_rate = 1.0 / original_dt
    
    print(f"Original sampling rate: {original_rate:.1f} Hz (dt = {original_dt:.6f}s)")
    print(f"Target sampling rate: {args.sample_rate:.1f} Hz (dt = {target_dt:.6f}s)")
    
    if target_dt < original_dt and not args.interpolate:
        print(f"WARNING: Target rate ({args.sample_rate:.1f}Hz) is higher than original rate ({original_rate:.1f}Hz)")
        print("Consider using --interpolate flag to upsample the data")
    
    # Interpolate if requested and needed
    if args.interpolate and target_dt < original_dt:
        print(f"Interpolating data from {original_rate:.1f}Hz to {args.sample_rate:.1f}Hz...")
        
        # Ensure strictly increasing time for interpolation
        # Remove duplicate timestamps and ensure monotonic increase
        unique_indices = []
        last_time = -np.inf
        for i, t in enumerate(time):
            if t > last_time:
                unique_indices.append(i)
                last_time = t
        
        if len(unique_indices) < len(time):
            print(f"Removed {len(time) - len(unique_indices)} duplicate/non-monotonic timestamps")
            time = time[unique_indices]
            gt_quats = gt_quats[unique_indices]
        
        # Create new time vector at target sampling rate
        time_start = time[0] 
        time_end = time[-1]
        time_new = np.arange(time_start, time_end, target_dt)
        
        # Ensure we don't go past the end
        time_new = time_new[time_new <= time_end]
        
        # Interpolate quaternions using SLERP (spherical linear interpolation)
        r_original = R.from_quat(gt_quats, scalar_first=True)
        
        # Perform SLERP interpolation
        slerp = Slerp(time, r_original)
        r_new = slerp(time_new)
        
        # Update data
        old_length = len(time)
        time = time_new
        gt_quats = r_new.as_quat(scalar_first=True)
        
        print(f"Interpolated from {old_length} to {len(time_new)} samples")
    
    dt = target_dt if args.interpolate else np.mean(np.diff(time))
    print(f"Using time step: {dt:.6f}s ({1.0/dt:.1f}Hz)")
    print(f"Noise parameters: gyr={args.gyr_noise:.6f}, acc={args.acc_noise:.6f}, mag={args.mag_noise}")
    
    # Generate realistic sensor data with proper noise levels
    simulated_sensor_data = ahrs.Sensors(
        gt_quats, 
        num_samples=gt_quats.shape[0], 
        freq=1.0/dt, 
        gyr_noise=args.gyr_noise,    # Realistic gyro noise
        acc_noise=args.acc_noise,    # Realistic accel noise  
        mag_noise=args.mag_noise     # Realistic mag noise
    )
    
    accels = simulated_sensor_data.accelerometers
    gyros = simulated_sensor_data.gyroscopes
    mags = simulated_sensor_data.magnetometers
    
    # Add bias drift to gyroscopes (realistic for MEMS)
    if args.gyr_bias > 0:
        bias_drift = np.random.normal(0, args.gyr_bias, (len(gyros), 3))
        gyros += np.cumsum(bias_drift, axis=0) * dt
        print(f"Added gyroscope bias drift: {args.gyr_bias} rad/s")
    
    # Add accelerometer bias
    if args.acc_bias > 0:
        acc_bias = np.random.normal(0, args.acc_bias, 3)
        accels += acc_bias
        print(f"Added accelerometer bias: {acc_bias}")
    
    print(f"Gyro range: [{gyros.min():.6f}, {gyros.max():.6f}] rad/s")
    print(f"Accel range: [{accels.min():.6f}, {accels.max():.6f}] m/s²")
    print(f"Mag range: [{mags.min():.6f}, {mags.max():.6f}] nT (will be converted to µT for C++)")
    print(f"Mag range (converted): [{mags.min()/1000.0:.6f}, {mags.max()/1000.0:.6f}] µT")
    
    # Print detailed sensor ranges in format matching real robot data
    print_sensor_ranges(gyros, accels, mags)
    
    num_samples = len(gt_quats)
    
    if args.no_tuning:
        # Use default parameters without tuning
        print("Skipping parameter tuning, using defaults...")
        # You could run filters with default params here
        # For now, just save the sensor data
        save_quaternions(f"{args.output_prefix}gt_quats.csv", time, gt_quats)
        save_sensor_data(f"{args.output_prefix}sensor_data.csv", time, gyros, accels, mags)
        return
    
    # Define parameter ranges based on command line arguments
    if not args.debug:
        madgwick_ranges = {'gain': (args.madgwick_gain_min, args.madgwick_gain_max)}
        mahony_ranges = {'k_P': (args.mahony_kp_min, args.mahony_kp_max), 'k_I': (args.mahony_ki_min, args.mahony_ki_max)}
        fourati_ranges = {'gain': (args.fourati_gain_min, args.fourati_gain_max)}
    else:
        # Use single values for debug mode
        madgwick_ranges = {'gain': (0.1, 0.1)}
        mahony_ranges = {'k_P': (1.0, 1.0), 'k_I': (0.1, 0.1)}
        fourati_ranges = {'gain': (0.1, 0.1)}
        
    # Generate grids based on explicit step size
    step_sizes = {'gain': args.step_gain, 'k_P': args.step_kp, 'k_I': args.step_ki}
    madgwick_grid = generate_grid(madgwick_ranges, step_sizes)
    mahony_grid = generate_grid(mahony_ranges, step_sizes)
    fourati_grid = generate_grid(fourati_ranges, step_sizes)
    
    # Run grid search for each filter with both MARG and IMU updates
    print(f'Running filter test for madgwick (MARG)...\n')
    Q_madgwick_marg, best_madgwick_marg = run_filter_test(gt_quats, gyros, accels, mags, ahrs.filters.Madgwick, madgwick_grid, dt, use_marg=True)

    print(f'Running filter test for madgwick (IMU)...\n')
    Q_madgwick_imu, best_madgwick_imu = run_filter_test(gt_quats, gyros, accels, mags, ahrs.filters.Madgwick, madgwick_grid, dt, use_marg=False)

    print(f'Running filter test for mahony (MARG)...\n')
    Q_mahony_marg, best_mahony_marg = run_filter_test(gt_quats, gyros, accels, mags, ahrs.filters.Mahony, mahony_grid, dt, use_marg=True)

    print(f'Running filter test for mahony (IMU)...\n')
    Q_mahony_imu, best_mahony_imu = run_filter_test(gt_quats, gyros, accels, mags, ahrs.filters.Mahony, mahony_grid, dt, use_marg=False)

    print(f'Running filter test for fourati (MARG)...\n')
    Q_fourati, best_fourati = run_filter_test(gt_quats, gyros, accels, mags, ahrs.filters.fourati.Fourati, fourati_grid, dt)
        
    # Compute and display errors
    madgwick_errors = compute_errors(gt_quats, Q_madgwick_marg)
    mahony_errors = compute_errors(gt_quats, Q_mahony_marg)
    fourati_errors = compute_errors(gt_quats, Q_fourati)
    
    print(f"\nFinal Results:")
    print(f"Madgwick MARG - Mean Error: {np.mean(madgwick_errors):.6f} rad, Std: {np.std(madgwick_errors):.6f}")
    print(f"Mahony MARG   - Mean Error: {np.mean(mahony_errors):.6f} rad, Std: {np.std(mahony_errors):.6f}")
    print(f"Fourati MARG  - Mean Error: {np.mean(fourati_errors):.6f} rad, Std: {np.std(fourati_errors):.6f}")
    
    if not args.no_plots and not args.debug:
        plt.figure(figsize=(12, 8))
        plt.subplot(2, 1, 1)
        plt.plot(time, madgwick_errors, label='Madgwick', alpha=0.7)
        plt.plot(time, mahony_errors, label='Mahony', alpha=0.7)
        plt.plot(time, fourati_errors, label='Fourati', alpha=0.7)
        plt.xlabel('Time [s]')
        plt.ylabel('Quaternion Angle Difference [rad]')
        plt.title('Quaternion Error Comparison Over Time')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(2, 1, 2)
        plt.hist([madgwick_errors, mahony_errors, fourati_errors], 
                bins=50, alpha=0.7, label=['Madgwick', 'Mahony', 'Fourati'])
        plt.xlabel('Quaternion Angle Difference [rad]')
        plt.ylabel('Frequency')
        plt.title('Error Distribution')
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.show()

        # Plot Euler angle errors
        plot_euler_errors(gt_quats, Q_madgwick_marg, Q_mahony_marg, Q_fourati, time)

    # Save all results with optional prefix
    save_quaternions(f"{args.output_prefix}madgwick_marg_quats.csv", time, Q_madgwick_marg)
    save_quaternions(f"{args.output_prefix}madgwick_imu_quats.csv", time, Q_madgwick_imu)
    save_quaternions(f"{args.output_prefix}mahony_marg_quats.csv", time, Q_mahony_marg)
    save_quaternions(f"{args.output_prefix}mahony_imu_quats.csv", time, Q_mahony_imu)
    save_quaternions(f"{args.output_prefix}fourati_quats.csv", time, Q_fourati)
    save_quaternions(f"{args.output_prefix}gt_quats.csv", time, gt_quats)
    save_sensor_data(f"{args.output_prefix}sensor_data.csv", time, gyros, accels, mags)
    
    print(f"\nFiles saved with prefix: '{args.output_prefix}'")

if __name__ == "__main__":
    main()

