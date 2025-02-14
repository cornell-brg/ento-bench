import argparse as ap
import numpy as np
from scipy.spatial.transform import Rotation as R
import ahrs
import matplotlib.pyplot as plt

def gt_from_golden():
    golden = np.genfromtxt("golden.csv", delimiter=",")
    data = golden[1:, :]
    gt = np.zeros((data.shape[0], 5))
    gt[:, 1:4] = data[:,8:11]
    gt[:,   4] = data[:, 7]
    gt[:,   0] = data[:, 0]
    return gt

def quats_from_gt(gt):
    r = R.from_euler('XYZ', gt[:, 1:4], degrees=False)
    quats = r.as_quat()
    return quats

def plot_gyro_data(gyros, time):
    plt.figure(figsize=(10, 6))

    # Plot each axis
    plt.plot(time, gyros[:, 0], label='Gyroscope X', linewidth=1.5)
    plt.plot(time, gyros[:, 1], label='Gyroscope Y', linewidth=1.5)
    plt.plot(time, gyros[:, 2], label='Gyroscope Z', linewidth=1.5)

    # Add labels and title
    plt.xlabel('Time [s]')
    plt.ylabel('Angular Velocity [rad/s]')
    plt.title('Simulated Gyroscope Data')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # Show the plot
    plt.show()

def main():
    gt = gt_from_golden()
    quats = quats_from_gt(gt)

    dt = gt[1,0] - gt[0,0]
    print(dt, 1.0/dt)
    simulated_sensor_data = ahrs.Sensors(quats, num_samples=quats.shape[0], freq=1.0/dt, gyr_noise=0.0) 

    accels = simulated_sensor_data.accelerometers
    gyros = simulated_sensor_data.gyroscopes
    mags = simulated_sensor_data.magnetometers

    print(gt.shape, accels.shape, gt[0,:].shape)
    plot_gyro_data(gyros, gt[:,0])

    

if __name__ == "__main__":
    main()

