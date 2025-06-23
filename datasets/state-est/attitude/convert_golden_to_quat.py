import pandas as pd
import numpy as np
import os


def deg_to_rad(deg_values):
    """Convert degrees to radians"""
    return np.array(deg_values) * np.pi / 180.0


def convert_csv_to_attitude_format(csv_file, output_marg_file, output_imu_file, sensor_choice=1):
    """
    Convert CSV file to attitude estimation format.
    
    Args:
        csv_file: Input CSV file path
        output_marg_file: Output MARG dataset file path  
        output_imu_file: Output IMU dataset file path
        sensor_choice: 1 for ax1,ay1,az1,gx1,gy1,gz1 or 2 for ax2,ay2,az2,gx2,gy2,gz2
    """
    print(f"Converting {csv_file}...")
    
    # Read CSV file
    df = pd.read_csv(csv_file)
    
    # Select sensor data based on choice
    if sensor_choice == 1:
        ax_col, ay_col, az_col = 'ax1', 'ay1', 'az1'
        gx_col, gy_col, gz_col = 'gx1', 'gy1', 'gz1'
    else:
        ax_col, ay_col, az_col = 'ax2', 'ay2', 'az2'
        gx_col, gy_col, gz_col = 'gx2', 'gy2', 'gz2'
    
    # Extract sensor data
    ax = df[ax_col].values  # m/s²
    ay = df[ay_col].values  # m/s²
    az = df[az_col].values  # m/s²
    
    gx = deg_to_rad(df[gx_col].values)  # Convert deg/s to rad/s
    gy = deg_to_rad(df[gy_col].values)  # Convert deg/s to rad/s
    gz = deg_to_rad(df[gz_col].values)  # Convert deg/s to rad/s
    
    mx = df['mx'].values  # µT
    my = df['my'].values  # µT
    mz = df['mz'].values  # µT
    
    # Generate identity quaternion for ground truth (since we don't have actual ground truth)
    # This will effectively test the filter's ability to maintain stability without reference
    qw = np.ones(len(df))    # Identity quaternion w component
    qx = np.zeros(len(df))   # Identity quaternion x component  
    qy = np.zeros(len(df))   # Identity quaternion y component
    qz = np.zeros(len(df))   # Identity quaternion z component
    
    # Time step: 1ms = 0.001 seconds
    dt = np.full(len(df), 0.001)
    
    # Write MARG dataset (with magnetometer)
    print(f"Writing MARG dataset to {output_marg_file}...")
    with open(output_marg_file, 'w') as f:
        f.write("Attitude Estimation Problem\n")
        for i in range(len(df)):
            f.write(f"{ax[i]:.6f} {ay[i]:.6f} {az[i]:.6f} "
                   f"{gx[i]:.6f} {gy[i]:.6f} {gz[i]:.6f} "
                   f"{mx[i]:.6f} {my[i]:.6f} {mz[i]:.6f} "
                   f"{qw[i]:.6f} {qx[i]:.6f} {qy[i]:.6f} {qz[i]:.6f} "
                   f"{dt[i]:.6f}\n")
    
    # Write IMU dataset (without magnetometer)
    print(f"Writing IMU dataset to {output_imu_file}...")
    with open(output_imu_file, 'w') as f:
        f.write("Attitude Estimation Problem\n")
        for i in range(len(df)):
            f.write(f"{ax[i]:.6f} {ay[i]:.6f} {az[i]:.6f} "
                   f"{gx[i]:.6f} {gy[i]:.6f} {gz[i]:.6f} "
                   f"{qw[i]:.6f} {qx[i]:.6f} {qy[i]:.6f} {qz[i]:.6f} "
                   f"{dt[i]:.6f}\n")
    
    print(f"Conversion complete! Generated {len(df)} samples.")
    print(f"  - MARG dataset: {output_marg_file}")
    print(f"  - IMU dataset: {output_imu_file}")
    
    # Print some statistics
    print(f"\nSensor data statistics (sensor choice {sensor_choice}):")
    print(f"  Accelerometer range: [{ax.min():.3f}, {ax.max():.3f}] x [{ay.min():.3f}, {ay.max():.3f}] x [{az.min():.3f}, {az.max():.3f}] m/s²")
    print(f"  Gyroscope range: [{np.rad2deg(gx.min()):.1f}, {np.rad2deg(gx.max()):.1f}] x [{np.rad2deg(gy.min()):.1f}, {np.rad2deg(gy.max()):.1f}] x [{np.rad2deg(gz.min()):.1f}, {np.rad2deg(gz.max()):.1f}] deg/s")
    print(f"  Magnetometer range: [{mx.min():.1f}, {mx.max():.1f}] x [{my.min():.1f}, {my.max():.1f}] x [{mz.min():.1f}, {mz.max():.1f}] µT")
    
    return len(df)


def main():
    # Define input and output paths
    input_dir = "."
    output_dir = "."
    
    # Input CSV files
    csv_files = [
        "gamma-bot-imu-data-iiswc25 - 1637steering{L0R182}{1k_800hz}.csv",
        "gamma-bot-imu-data-iiswc25 - 1647straig{L182R127}{1k_800hz}.csv"
    ]
    
    # Convert each CSV file
    total_samples = 0
    for csv_file in csv_files:
        csv_path = os.path.join(input_dir, csv_file)
        
        if not os.path.exists(csv_path):
            print(f"Warning: {csv_path} not found, skipping...")
            continue
        
        # Create output filenames based on input filename
        base_name = csv_file.replace(".csv", "").replace(" ", "_").replace("{", "_").replace("}", "_")
        
        # Use sensor 1 data (ax1, ay1, az1, gx1, gy1, gz1)
        marg_output = os.path.join(output_dir, f"{base_name}_sensor1_marg_dataset.txt")
        imu_output = os.path.join(output_dir, f"{base_name}_sensor1_imu_dataset.txt")
        
        samples = convert_csv_to_attitude_format(csv_path, marg_output, imu_output, sensor_choice=1)
        total_samples += samples
        
        print()  # Add spacing between files
    
    print(f"=== CONVERSION COMPLETE ===")
    print(f"Total samples processed: {total_samples}")
    print(f"Files processed: {len([f for f in csv_files if os.path.exists(os.path.join(input_dir, f))])}")
    print()
    print("Note: Generated datasets use identity quaternion as ground truth.")
    print("This will test the filter's ability to maintain stability and track orientation")
    print("without a reference, which is perfect for analyzing failure rates across Q formats.")
    print()
    print("Generated datasets can be used with the attitude estimation benchmark by")
    print("updating the dataset paths in the benchmark configuration.")


if __name__ == "__main__":
    main()
