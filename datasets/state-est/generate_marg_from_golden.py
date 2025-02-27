import argparse as ap
import numpy as np
from scipy.spatial.transform import Rotation as R
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
    return r.as_quat()

def compute_errors(gt_quats, estimated_quats):
    return np.array([ahrs.utils.qad(gt_quats[i], estimated_quats[i]) for i in range(len(gt_quats))])

def generate_grid(param_ranges, step_sizes):
    return {key: np.arange(val[0], val[1] + step_sizes[key], step_sizes[key]).tolist() for key, val in param_ranges.items()}

def run_filter_test(gt_quats, gyros, accels, mags, filter_class, param_grid):
    best_params = None
    best_error = float('inf')
    best_Q = None
    param_combinations = list(product(*param_grid.values()))
    
    for params in tqdm(param_combinations, desc=f'Tuning {filter_class.__name__}'): 
        param_dict = dict(zip(param_grid.keys(), params))
        filter_instance = filter_class(**param_dict)
        
        Q = np.tile(gt_quats[0], (len(gyros), 1))  # Seed initial quaternion from ground truth
        for t in range(1, len(gyros)):
            if isinstance(filter_instance, ahrs.filters.fourati.Fourati):
                Q[t] = filter_instance.update(Q[t-1], gyr=gyros[t], acc=accels[t], mag=mags[t])
            else:
                Q[t] = filter_instance.updateMARG(Q[t-1], gyr=gyros[t], acc=accels[t], mag=mags[t])
        
        errors = compute_errors(gt_quats, Q)
        mean_error = np.mean(errors)
        
        if mean_error < best_error:
            best_error = mean_error
            best_params = param_dict
            best_Q = Q
            print(f'\rNew best {filter_class.__name__} params: {best_params}, Error: {best_error}', end='', flush=True)
    
    print(f'\nBest params for {filter_class.__name__}: {best_params}, Error: {best_error}')
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
        #axes[i].plot(time, mahony_euler[:, i], label='Mahony', linewidth=1.5)
        axes[i].plot(time, fourati_euler[:, i], label='Fourati', linewidth=1.5)
        
        axes[i].set_xlabel('Time [s]')
        axes[i].set_ylabel(f'{labels[i]} [deg]')
        axes[i].set_title(f'{labels[i]} Comparison')
        axes[i].legend()
        axes[i].grid(True)

    plt.tight_layout()
    plt.show()

def main():
    parser = ap.ArgumentParser()
    parser.add_argument('--step-gain', type=float, default=0.01, help='Step size for gain parameter')
    parser.add_argument('--step-kp', type=float, default=0.1, help='Step size for Mahony kp parameter')
    parser.add_argument('--step-ki', type=float, default=0.05, help='Step size for Mahony ki parameter')
    args = parser.parse_args()
    
    gt, time = gt_from_golden()
    gt_quats = quats_from_gt(gt)
    dt = time[1] - time[0]
    
    simulated_sensor_data = ahrs.Sensors(gt_quats, num_samples=gt_quats.shape[0], freq=1.0/dt, gyr_noise=0.0)
    accels = simulated_sensor_data.accelerometers
    gyros = simulated_sensor_data.gyroscopes
    mags = simulated_sensor_data.magnetometers
    num_samples = len(gt_quats)
    
    # Define parameter ranges based on defaults
    madgwick_ranges = {'gain': (0.001, 0.5)}
    mahony_ranges = {'k_P': (0.1, 2.0), 'k_I': (0.01, 0.5)}
    fourati_ranges = {'gain': (0.05, 0.2)}
    
    # Generate grids based on explicit step size
    step_sizes = {'gain': args.step_gain, 'k_P': args.step_kp, 'k_I': args.step_ki}
    madgwick_grid = generate_grid(madgwick_ranges, step_sizes)
    mahony_grid = generate_grid(mahony_ranges, step_sizes)
    fourati_grid = generate_grid(fourati_ranges, step_sizes)
    
    # Run grid search for each filter
    Q_madgwick, best_madgwick = run_filter_test(gt_quats, gyros, accels, mags, ahrs.filters.Madgwick, madgwick_grid)
    Q_mahony, best_mahony = run_filter_test(gt_quats, gyros, accels, mags, ahrs.filters.Mahony, mahony_grid)
    Q_fourati, best_fourati = run_filter_test(gt_quats, gyros, accels, mags, ahrs.filters.fourati.Fourati, fourati_grid)
    
    # Compute errors
    plt.figure(figsize=(10, 6))
    plt.plot(compute_errors(gt_quats, Q_madgwick), label='Madgwick')
    plt.plot(compute_errors(gt_quats, Q_mahony), label='Mahony')
    plt.plot(compute_errors(gt_quats, Q_fourati), label='Fourati')
    plt.xlabel('Time Step')
    plt.ylabel('Quaternion Angle Difference (rad)')
    plt.title('Quaternion Error Comparison')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    # Plot Euler angle errors
    plot_euler_errors(gt_quats, Q_madgwick, Q_mahony, Q_fourati, time)

if __name__ == "__main__":
    main()

