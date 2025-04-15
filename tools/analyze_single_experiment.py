import os
import numpy as np
import matplotlib.pyplot as plt
from natsort import natsorted
import pandas as pd
import sys


def find_combined_csvs(dataset_path, dataset_name):
    csv_files = []
    file_path = os.path.join(dataset_path, dataset_name)

    for root, _, files in os.walk(file_path):
        for file in files:
            if file.endswith("_combined.csv"):
                csv_files.append(os.path.join(root, file))

    return csv_files


def find_spike_dynamic(current_segment, threshold):
    indices = np.where(current_segment > threshold)[0]
    return indices[0] if len(indices) > 0 else None


def find_drop_dynamic(current_segment, threshold):
    indices = np.where(current_segment < threshold)[0]
    return indices[0] if len(indices) > 0 else None


def analyze_power_consumption(parent_dir, dataset_name, window_size, rising_threshold, falling_threshold, direction, plot_data):
    csv_paths = find_combined_csvs(parent_dir, dataset_name)
    if not csv_paths:
        print(f"No combined.csv files found in {dataset_name}")
        return [], [], [], [], [], False

    csv_path = csv_paths[0]
    print(f"Starting energy consumption analysis: {os.path.basename(csv_path)}")
    voltage = 3.3
    success = True
    energy_segments = []
    currents = []
    tdiffs = []
    adjusted_latencies = []
    total_energy = 0
    
    # New: Store per-iteration data
    iteration_data = []

    data = pd.read_csv(csv_path)
    if 'latency' not in data.columns or 'time' not in data.columns or 'current' not in data.columns:
        print(f"Skipping {csv_path}, missing required columns.")
        return [], [], [], [], [], False

    latency_indices = np.where(data['latency'] == 1)[0]
    if len(latency_indices) % 2 != 0:
        latency_indices = latency_indices[:-1]
    selected_indices = latency_indices
    if len(selected_indices) < 2:
        print(f"Not enough latency events detected in {csv_path}")
        return [], [], [], [], [], False

    latencies = data['time'].iloc[selected_indices[1::2]].values - data['time'].iloc[selected_indices[::2]].values
    data['current_mA'] = data['current'] / 1000

    if plot_data:
        plot_dir = os.path.join(os.path.dirname(csv_path), 'plots')
        os.makedirs(plot_dir, exist_ok=True)
        plt.figure(figsize=(12, 6))
        plt.plot(data['time'], data['current_mA'], linewidth=2.5, label='Current Trace')
        for idx in selected_indices:
            plt.axvline(x=data['time'].iloc[idx], color='r', linestyle='--', linewidth=1.75)

    window_offsets_initialized = False
    relative_start_offset = 0
    relative_end_offset = 0

    prev_adj_start = None
    prev_adj_end = None

    for i in range(0, len(selected_indices) - 1, 2):
        iteration_number = i // 2 + 1  # Calculate iteration number (1-based)
        idx1 = selected_indices[i]
        idx2 = selected_indices[i + 1]
        duration = data['time'].iloc[idx2] - data['time'].iloc[idx1]

        if prev_adj_start is not None and prev_adj_end is not None:
            prev_duration_adj = data['time'].iloc[prev_adj_end] - data['time'].iloc[prev_adj_start]
            # Apply the same window adjustment (shift) to the next window from the current latency indices
            shift_amount = data['time'].iloc[prev_adj_end] - data['time'].iloc[prev_adj_start]  # The previous shift
            search_start_time = (data['time'].iloc[prev_adj_end] - window_size * prev_duration_adj) if direction == 0 else (data['time'].iloc[prev_adj_start] + window_size * prev_duration_adj)
            search_start_time = max(search_start_time, data['time'].iloc[0])

            # Shift the current window by the same amount as the previous adjusted window
            search_start = np.where(data['time'] >= search_start_time)[0][0]
            search_end = idx2
        else:
            # For the first window, calculate the start time as before
            search_start_time = (data['time'].iloc[idx2] - window_size * duration) if direction == 0 else (data['time'].iloc[idx1] + window_size * duration)
            search_start_time = max(search_start_time, data['time'].iloc[0])
            search_start = np.where(data['time'] >= search_start_time)[0][0]
            search_end = idx2


        search_start = max(0, min(search_start, len(data) - 1))
        search_end = max(0, min(search_end, len(data) - 1))

        current_window = data['current_mA'].iloc[search_start:search_end]
        min_current = current_window.min()
        max_current = current_window.max()
        current_threshold_rise = min_current + rising_threshold * (max_current - min_current)

        spike_idx = find_spike_dynamic(current_window, current_threshold_rise)
        idx1_adj = search_start + spike_idx if spike_idx is not None else idx1
        tstart_adj = data['time'].iloc[idx1_adj]
        tend_adj_est = tstart_adj + duration
        tend_adj_est_idx = np.where(data['time'] >= tend_adj_est)[0][0]

        tend_window_size = 0.05
        tend_half_window_size = round((tend_adj_est_idx - idx1_adj) * tend_window_size * 0.5)
        drop_window_start = max(idx1_adj, tend_adj_est_idx - tend_half_window_size)
        drop_window_end = min(len(data) - 1, tend_adj_est_idx + tend_half_window_size)
        drop_window = data['current_mA'].iloc[drop_window_start:drop_window_end]
        current_threshold_drop = max_current - falling_threshold * (max_current - min_current)
        drop_idx = find_drop_dynamic(drop_window, current_threshold_drop)

        iters = 0
        while drop_idx is None and iters < 1000:
            tend_window_size += 0.05
            tend_half_window_size = round((tend_adj_est_idx - idx1_adj) * tend_window_size * 0.5)
            drop_window_start = max(idx1_adj, tend_adj_est_idx - tend_half_window_size)
            drop_window_end = min(len(data) - 1, tend_adj_est_idx + tend_half_window_size)
            drop_window = data['current_mA'].iloc[drop_window_start:drop_window_end]
            drop_idx = find_drop_dynamic(drop_window, current_threshold_drop)
            iters += 1
            if iters == 1000:
                print("Got stuck in inf loop... exiting")
                return [], [], [], [], [], False

        idx2_adj = drop_window_start + drop_idx if drop_idx is not None else tend_adj_est_idx

        tstart = data['time'].iloc[idx1]
        tend = data['time'].iloc[idx2]
        tstart_adj = data['time'].iloc[idx1_adj]
        tend_adj = data['time'].iloc[idx2_adj]
        adjusted_latency = tend_adj - tstart_adj
        adjusted_latencies.append(adjusted_latency)
        tdiff = (tend - tstart) - (tend_adj - tstart_adj)
        tdiffs.append(tdiff)

        time_segment = data['time'].iloc[idx1_adj:idx2_adj].ffill().bfill()
        current_segment = data['current_mA'].iloc[idx1_adj:idx2_adj].ffill().bfill()

        rel_lat_error = 100 * (tdiff / (tend - tstart))
        current = current_segment.mean()
        energy_segment = np.trapezoid(current_segment, time_segment) * voltage * 1e-6
        energy_adjustment = current * voltage * tdiff * 1e-3 if abs(rel_lat_error) > 10 else 0
        total_segment_energy = energy_segment + energy_adjustment

        energy_segments.append(total_segment_energy)
        currents.append(current)
        total_energy += total_segment_energy

        if plot_data:
            plt.figure(figsize=(12, 6))
            plot_buffer = round(window_size * (idx2_adj - idx1_adj))
            plot_start = max(0, idx1_adj - plot_buffer)
            plot_end = min(len(data) - 1, idx2_adj + plot_buffer)

            shifted_window_start = search_start
            shifted_window_end = search_end
            plt.plot(data['time'].iloc[shifted_window_start:shifted_window_end],
                    data['current_mA'].iloc[shifted_window_start:shifted_window_end],
                    color='purple', linewidth=2.5, label='Shifted Window')

            # Plot the main current window
            plt.plot(data['time'].iloc[plot_start:plot_end],
                     data['current_mA'].iloc[plot_start:plot_end],
                     linewidth=2.5)

            plt.axvline(x=data['time'].iloc[idx1], color='r', linestyle='--', linewidth=1.5, label='Latency')
            plt.axvline(x=data['time'].iloc[idx2], color='r', linestyle='--', linewidth=1.5)
            plt.axvline(x=data['time'].iloc[idx1_adj], color='b', linestyle='--', linewidth=1.5, label='Adjusted')
            plt.axvline(x=data['time'].iloc[idx2_adj], color='b', linestyle='--', linewidth=1.5)

            plt.xlabel('Time (ms)')
            plt.ylabel('Current (mA)')
            plt.title(f'Segment {(i // 2) + 1}')
            plt.legend()
            plt.tight_layout()
            plt.savefig(os.path.join(plot_dir, f'segment_{(i // 2) + 1}.png'))
            plt.close()

    if plot_data:
        handles, labels = plt.gca().get_legend_handles_labels()
        if labels:
            plt.legend()
        plt.savefig(os.path.join(plot_dir, 'full_plot.png'))
        plt.close()

    average_energy = np.mean(energy_segments)
    average_current = np.mean(currents)
    average_tdiff = np.mean(tdiffs) * 1e3
    average_latency = np.mean(latencies) * 1e3

    print(f'Total energy consumed: {total_energy:.8f} mJ')
    print(f'Average current consumed: {average_current:.8f} mA')
    print(f'Average energy per segment: {average_energy:.8f} mJ')
    print(f'Latency stats (avg, max, min): {np.mean(latencies)*1e3:.6f} µs, {np.max(latencies)*1e3:.6f} µs, {np.min(latencies)*1e3:.6f} µs')
    print(f'Average cycles: {(average_latency/1e6)/(1/170e6):.6f}')
    print(f'Average time difference error (t_meas - t_adj): {average_tdiff:.6f} µs')
    print(f'Average time difference percentage: {100 * (average_tdiff / average_latency):.6f}%\n')

    return tdiffs, energy_segments, latencies, adjusted_latencies, iteration_data, success


def analyze_single_experiment(parent_dir, dataset_name, window_size, rising_threshold, falling_threshold, plot_data):
    print(f"Analyzing single dataset: {dataset_name}")
    
    direction = 0
    tdiffs, energy_segments, latencies, adjusted_latencies, iteration_data, success = analyze_power_consumption(
        parent_dir, dataset_name, window_size, rising_threshold, falling_threshold, direction, plot_data)
    
    if not success:
        direction = 1
        tdiffs, energy_segments, latencies, adjusted_latencies, iteration_data, success = analyze_power_consumption(
            parent_dir, dataset_name, window_size, rising_threshold, falling_threshold, direction, plot_data)
        if success:
            print(f"Succeeded analyzing with direction = {direction}")
    
    if not success:
        print("Failed to analyze the dataset.")
        return None
    
    result = {
        'tdiffs': tdiffs,
        'energy_segments': energy_segments,
        'average_energy': np.mean(energy_segments),
        'latencies': latencies,
        'adjusted_latencies': adjusted_latencies,
        'iteration_data': iteration_data
    }
    return result


def main():
    if len(sys.argv) != 7:
        print("Usage: python3 analyze_single_experiment.py <parent_dir> <dataset_name> <window_size> <rising_threshold> <falling_threshold> <plot_data>")
        sys.exit(1)

    parent_dir = sys.argv[1]
    dataset_name = sys.argv[2]
    window_size = float(sys.argv[3])
    rising_threshold = float(sys.argv[4])
    falling_threshold = float(sys.argv[5])
    plot_data = sys.argv[6].lower() == 'true'

    results = analyze_single_experiment(
        parent_dir,
        dataset_name,
        window_size,
        rising_threshold,
        falling_threshold,
        plot_data
    )

    return results


if __name__ == "__main__":
    main()