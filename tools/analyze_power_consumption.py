import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def analyze_power_consumption(dataset_parent_dir, dataset_name, window_size, rising_threshold, falling_threshold, direction, plot_data):
    """
    Analyze power consumption from an experiment dataset.

    Parameters:
        dataset_parent_dir (str): Parent directory of the dataset.
        dataset_name (str): Name of the dataset (subdirectory).
        window_size (int): Window size for analysis.
        rising_threshold (float): Threshold for detecting power increase.
        falling_threshold (float): Threshold for detecting power decrease.
        direction (int): Direction of search (0 or 1).
        plot_data (bool): Whether to generate plots.

    Returns:
        tuple: (tdiffs, energy_segments, latencies, adjusted_latencies, success)
    """
    print(f'Starting energy consumption analysis: {dataset_name}')

    dataset_path = os.path.join(dataset_parent_dir, dataset_name)
    csv_path = os.path.join(dataset_path, f"{dataset_name}_combined.csv")

    if not os.path.exists(csv_path):
        print(f"Error: File {csv_path} not found.")
        return [], [], [], [], False

    # Constants
    voltage = 3.3  # Volts

    # Read CSV into DataFrame
    data = pd.read_csv(csv_path)

    if 'latency' not in data.columns or 'time' not in data.columns or 'current' not in data.columns:
        print("Error: Missing required columns in CSV file.")
        return [], [], [], [], False

    # Find indices where 'latency' is 1
    latency_indices = data.index[data['latency'] == 1].to_numpy()
    selected_indices = latency_indices[:]

    if len(selected_indices) < 2:
        print("Error: Not enough latency events detected.")
        return [], [], [], [], False

    # Compute latencies
    latencies = data['time'].iloc[selected_indices[1::2]].values - data['time'].iloc[selected_indices[::2]].values

    # Convert current from µA to mA
    data['current_mA'] = data['current'] / 1000

    # Initialize variables
    energy_segments = []
    currents = []
    tdiffs = []
    adjusted_latencies = []
    total_energy = 0

    # Create directory for plots
    plot_dir = os.path.join(dataset_parent_dir, 'plots', dataset_name)
    os.makedirs(plot_dir, exist_ok=True)

    # Plot full dataset
    if plot_data:
        plt.figure()
        plt.plot(data['time'], data['current_mA'], linewidth=2, label="Current")
        plt.xlabel('Time (ms)')
        plt.ylabel('Current (mA)')
        plt.title(f'PIL Test: {dataset_name}')
        for idx in selected_indices:
            plt.axvline(data['time'].iloc[idx], color='r', linestyle='--')
        plt.legend()
        plt.savefig(os.path.join(plot_dir, "full_plot.png"))
        plt.close()

    # Process each latency segment
    for i in range(0, len(selected_indices) - 1, 2):
        idx1 = selected_indices[i]
        idx2 = selected_indices[i + 1]

        duration = data['time'].iloc[idx2] - data['time'].iloc[idx1]
        search_start_time = (data['time'].iloc[idx2] - window_size * duration) if direction == 0 else (data['time'].iloc[idx1] + window_size * duration)
        search_start_time = max(search_start_time, data['time'].iloc[0])  # Ensure valid range

        # Convert search start time to index
        search_start = (data['time'] >= search_start_time).idxmax()
        search_end = idx2

        # Extract current values in the window
        current_window = data.loc[search_start:search_end, 'current_mA']

        # Compute rising threshold
        min_current = current_window.min()
        max_current = current_window.max()
        current_threshold_rise = min_current + rising_threshold * (max_current - min_current)

        # Find the first current spike
        spike_idx = (current_window > current_threshold_rise).idxmax()
        idx1_adj = spike_idx if spike_idx else idx1

        # Adjusted latency calculations
        tstart_adj = data['time'].iloc[idx1_adj]
        tend_adj_est = tstart_adj + duration
        tend_adj_est_idx = (data['time'] >= tend_adj_est).idxmax()

        tend_window_size = 0.05
        tend_half_window_size = round((tend_adj_est_idx - idx1_adj) * tend_window_size * 0.5)
        drop_window_start = max(idx1_adj, tend_adj_est_idx - tend_half_window_size)
        drop_window_end = min(len(data) - 1, tend_adj_est_idx + tend_half_window_size)

        drop_window = data.loc[drop_window_start:drop_window_end, 'current_mA']
        current_threshold_drop = max_current - falling_threshold * (max_current - min_current)

        # Find the first current drop
        drop_idx = (drop_window < current_threshold_drop).idxmax()
        idx2_adj = drop_idx if drop_idx else tend_adj_est_idx

        # Compute adjusted latency and energy
        adjusted_latency = data['time'].iloc[idx2_adj] - data['time'].iloc[idx1_adj]
        adjusted_latencies.append(adjusted_latency)

        tdiff = (data['time'].iloc[idx2] - data['time'].iloc[idx1]) - adjusted_latency
        tdiffs.append(tdiff)

        time_segment = data.loc[idx1_adj:idx2_adj, 'time']
        current_segment = data.loc[idx1_adj:idx2_adj, 'current_mA']

        # Compute energy
        energy_segment = np.trapz(current_segment, time_segment) * voltage * 1e-6  # Convert µA·s to mJ
        rel_lat_error = 100 * (tdiff / (data['time'].iloc[idx2] - data['time'].iloc[idx1]))

        energy_adjustment = tdiff * energy_segment if abs(rel_lat_error) > 10 else 0
        energy_segments.append(energy_segment + energy_adjustment)
        currents.append(current_segment.mean())

        total_energy += energy_segment + energy_adjustment

        # Plot adjusted segment
        if plot_data:
            plt.figure()
            plt.plot(data['time'].iloc[idx1_adj:idx2_adj], data['current_mA'].iloc[idx1_adj:idx2_adj], linestyle='--', marker='o', color='purple')
            plt.axvline(data['time'].iloc[idx1_adj], color='b', linestyle='--')
            plt.axvline(data['time'].iloc[idx2_adj], color='b', linestyle='--')
            plt.xlabel('Time (ms)')
            plt.ylabel('Current (mA)')
            plt.title(f'Segment {i//2 + 1}')
            plt.savefig(os.path.join(plot_dir, f'segment_{i//2 + 1}.png'))
            plt.close()

    # Summary statistics
    average_energy = np.mean(energy_segments)
    average_current = np.mean(currents)
    average_tdiff = np.mean(tdiffs) * 1e3  # Convert ms to µs
    average_latency = np.mean(latencies) * 1e3  # Convert ms to µs

    print(f'Total energy consumed: {total_energy:.8f} mJ')
    print(f'Average current consumed: {average_current:.8f} mA')
    print(f'Average energy per segment: {average_energy:.8f} mJ')
    print(f'Latency stats (avg, max, min): {average_latency:.6f} µs, {np.max(latencies)*1e3:.6f} µs, {np.min(latencies)*1e3:.6f} µs')
    print(f'Average time difference error (t_meas - t_adj): {average_tdiff:.6f} µs')
    print(f'Average time difference percentage: {100 * (average_tdiff / average_latency):.6f}%')

    return tdiffs, energy_segments, latencies, adjusted_latencies, True
