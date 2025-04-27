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
        return [], [], [], [], False

    csv_path = csv_paths[0]
    print(f"Starting energy consumption analysis: {os.path.basename(csv_path)}")

    # Initializations
    voltage = 3.3  # Volts
    success = True
    energy_segments = []
    currents = []
    tdiffs = []
    adjusted_latencies = []
    total_energy = 0

    data = pd.read_csv(csv_path)

    if 'latency' not in data.columns or 'time' not in data.columns or 'current' not in data.columns:
        print(f"Skipping {csv_path}, missing required columns.")
        return [], [], [], [], False

    # Find where latency indices are 1
    latency_indices = np.where(data['latency'] == 1)[0]
    if len(latency_indices) % 2 != 0:
        latency_indices = latency_indices[:-1]
    selected_indices = latency_indices
    # Make sure there's enough latency events
    if len(selected_indices) < 2:
        print(f"Not enough latency events detected in {csv_path}")
        return [], [], [], [], False
    latencies = data['time'].iloc[selected_indices[1::2]].values - \
                data['time'].iloc[selected_indices[::2]].values

    data['current_mA'] = data['current'] / 1000

    width_scale_factor = 0.5
    segment_width = 3 * width_scale_factor
    vertical_width = 3.5 * width_scale_factor
    trace_width = 2.5 * width_scale_factor

    if plot_data:
        plot_dir = os.path.join(os.path.dirname(csv_path), 'plots')
        os.makedirs(plot_dir, exist_ok=True)

    if plot_data:
        plt.figure(figsize=(12, 6))
        plt.plot(data['time'], data['current_mA'], linewidth=trace_width)
        plt.xlabel('Time (ms)')
        plt.ylabel('Current (mA)')
        plt.title(f"PIL Test: {os.path.basename(csv_path)}")

        for idx in selected_indices:
            plt.axvline(x=data['time'].iloc[idx], color='r', linestyle='--', linewidth=vertical_width)
    
    # Track previous time shift for prealignment
    previous_shift = 0
    
    # Processing for each latency segment
    for i in range(0, len(selected_indices) - 1, 2):
        idx1 = selected_indices[i]
        idx2 = selected_indices[i + 1]

        # Define the search window
        duration = data['time'].iloc[idx2] - data['time'].iloc[idx1]
        
        # Prealign based on previous segment if its not the first segment
        if i > 0 and previous_shift != 0:
            # For prealignment, calculate a time offset based on the previous shift
            search_start_time = (data['time'].iloc[idx2] - window_size * duration) + previous_shift
            search_end_time = (data['time'].iloc[idx1] + window_size * duration) + previous_shift
            # print(f'Previous shift: {previous_shift}')
            # print(f'New search/end: {search_start_time}:{search_end_time}')
            # print(f'Old search/end: {search_start_time-previous_shift}:{search_end_time-previous_shift}')
        # If first segment, use previous method
        else:
            search_start_time = data['time'].iloc[idx2] - window_size * duration
            search_end_time = data['time'].iloc[idx1] + window_size * duration
        
        search_start_time = max(search_start_time, data['time'].iloc[0])
        search_start = np.where(data['time'] >= search_start_time)[0][0]

        # Adjust search end using window
        search_end_time = min(search_end_time, data['time'].iloc[-1])
        search_end = (data['time'] >= search_end_time).idxmax()

        # Original search window plot
        if plot_data:
            plt.axvline(x=data['time'].iloc[search_start], color='c', linestyle='--', linewidth=vertical_width)
            plt.axvline(x=data['time'].iloc[search_end], color='c', linestyle='--', linewidth=vertical_width)
            
            # Prealigned search window plot
            if i > 0 and previous_shift != 0:
                plt.axvline(x=data['time'].iloc[search_start], color='g', linestyle='--', linewidth=vertical_width)

        current_window = data['current_mA'].iloc[search_start:search_end]
        min_current = current_window.min()
        max_current = current_window.max()
        current_threshold_rise = min_current + rising_threshold * (max_current - min_current)

        # Find rising edge
        spike_idx = find_spike_dynamic(current_window, current_threshold_rise)
        idx1_adj = search_start + spike_idx if spike_idx is not None else idx1

        # Calculate the time shift to use for the next segment
        original_start_time = data['time'].iloc[idx1]
        adjusted_start_time = data['time'].iloc[idx1_adj]
        time_diff = adjusted_start_time - original_start_time
        # print("------------------------------------------------------------")
        # print(f'Original start time: {original_start_time}')
        # print(f'Adjusted start time: {adjusted_start_time}')
        previous_shift = time_diff

        tstart_adj = data['time'].iloc[idx1_adj]
        tend_adj_est = tstart_adj + duration
        tend_adj_est_idx = np.where(data['time'] >= tend_adj_est)[0][0]

        # Falling edge detection
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
                return [], [], [], [], False

        idx2_adj = drop_window_start + drop_idx if drop_idx is not None else tend_adj_est_idx
        
        # If window is larger than duration of latency indices
        max_allowed_tend = tstart_adj + duration
        actual_tend_time = data['time'].iloc[idx2_adj]

        if actual_tend_time > max_allowed_tend:
            idx2_adj = np.where(data['time'] >= max_allowed_tend)[0][0]

        # Calculate timing differences
        tstart = data['time'].iloc[idx1]
        tend = data['time'].iloc[idx2]
        tstart_adj = data['time'].iloc[idx1_adj]
        tend_adj = data['time'].iloc[idx2_adj]
        adjusted_latency = tend_adj - tstart_adj
        adjusted_latencies.append(adjusted_latency)
        tdiff = (tend - tstart) - (tend_adj - tstart_adj)
        tdiffs.append(tdiff)

        # Calculate energy
        time_segment = data['time'].iloc[idx1_adj:idx2_adj]
        current_segment = data['current_mA'].iloc[idx1_adj:idx2_adj]

        time_segment = time_segment.ffill().bfill()
        current_segment = current_segment.ffill().bfill()

        rel_lat_error = 100 * (tdiff / (tend - tstart))
        current = current_segment.mean()
        energy_segment = np.trapezoid(current_segment, time_segment) * voltage * 1e-6  # mJ

        if abs(rel_lat_error) > 10:
            energy_adjustment = current * voltage * tdiff * 1e-3
        else:
            energy_adjustment = 0

        energy_segments.append(energy_segment + energy_adjustment)
        currents.append(current)
        total_energy += energy_segment + energy_adjustment
        peak_current = current_segment.max()

        # print(f'Energy consumed for segment {(i+1)//2 + 1}: {energy_segment + energy_adjustment:.8f} mJ')
        # print(f'Average current consumed for segment {(i+1)//2 + 1}: {current:.8f} mA')
        # print(f'Peak current consumed for segment {(i+1)//2 + 1}: {peak_current:.8f} mA')

        if plot_data:
            plt.plot(data['time'].iloc[idx1_adj:idx2_adj],
                    data['current_mA'].iloc[idx1_adj:idx2_adj],
                    color='#7E2F8E', linestyle='--', marker='s',
                    linewidth=segment_width)
            plt.axvline(x=data['time'].iloc[idx1_adj], color='b',
                       linestyle='--', linewidth=vertical_width)
            plt.axvline(x=data['time'].iloc[idx2_adj], color='b',
                       linestyle='--', linewidth=vertical_width)

            plt.figure(figsize=(12, 6))
            plot_buffer = round(window_size * (idx2_adj - idx1_adj))
            plot_start = max(0, idx1_adj - plot_buffer)
            plot_end = min(len(data) - 1, idx2_adj + plot_buffer)
            
            plt.plot(data['time'].iloc[plot_start:plot_end],
                    data['current_mA'].iloc[plot_start:plot_end],
                    linewidth=trace_width)
            plt.axvline(x=data['time'].iloc[idx1], color='r',
                       linestyle='--', linewidth=vertical_width, label='Latency Indices')
            plt.axvline(x=data['time'].iloc[idx2], color='r',
                       linestyle='--', linewidth=vertical_width)
            plt.axvline(x=data['time'].iloc[idx1_adj], color='b',
                       linestyle='--', linewidth=vertical_width, label='Adjusted Indices')
            plt.axvline(x=data['time'].iloc[idx2_adj], color='b',
                       linestyle='--', linewidth=vertical_width)
            
            # Add original search window in cyan
            plt.axvline(x=data['time'].iloc[search_start], color='c',
                       linestyle='--', linewidth=vertical_width, label='Search Window')
            plt.axvline(x=data['time'].iloc[search_end], color='c',
                       linestyle='--', linewidth=vertical_width)
            
            # Add prealigned search window in green (if not the first segment)
            if i > 0 and previous_shift != 0:
                # Calculate where the prealigned window would have been
                prealigned_time = search_start_time
                prealigned_idx = np.where(data['time'] >= prealigned_time)[0][0]
                plt.axvline(x=data['time'].iloc[prealigned_idx], color='g',
                          linestyle='--', linewidth=vertical_width, label='Prealigned Window')
            
            plt.xlabel('Time (ms)')
            plt.ylabel('Current (mA)')
            plt.title(f'Segment: {(i+1)//2 + 1}')
            plt.legend()
            plt.savefig(os.path.join(plot_dir, f'segment_{(i+1)//2 + 1}.png'))
            plt.close()

    if plot_data:
        # Update legend to include the green prealigned search window
        plt.legend(['Data', 'Latency Indices', 'Search Window', 'Prealigned Window', 'Adjusted Indices'])
        plt.savefig(os.path.join(plot_dir, 'full_plot.png'))
        plt.close()

    average_energy = np.mean(energy_segments)
    average_current = np.mean(currents)
    average_tdiff = np.mean(tdiffs) * 1e3
    average_latency = np.mean(latencies) * 1e3
    
    print("============================================================")
    print(f'Total energy consumed: {total_energy:.8f} mJ')
    print(f'Average current consumed: {average_current:.8f} mA')
    print(f'Average energy per segment: {average_energy:.8f} mJ')
    print(f'Latency stats (avg, max, min): {np.mean(latencies)*1e3:.6f} µs, {np.max(latencies)*1e3:.6f} µs, {np.min(latencies)*1e3:.6f} µs')
    print(f'Average cycles: {(average_latency/1e6)/(1/170e6):.6f}')
    print(f'Average time difference error (t_meas - t_adj): {average_tdiff:.6f} µs')
    print(f'Average time difference percentage: {100 * (average_tdiff / average_latency):.6f}%\n')

    return tdiffs, energy_segments, latencies, adjusted_latencies, success


def analyze_single_experiment(parent_dir, dataset_name, window_size, rising_threshold, falling_threshold, plot_data):
    print(f"Analyzing single dataset: {dataset_name}")
    
    direction = 0
    tdiffs, energy_segments, latencies, adjusted_latencies, success = analyze_power_consumption(
        parent_dir, dataset_name, window_size, rising_threshold, falling_threshold, direction, plot_data)
    
    if not success:
        direction = 1
        tdiffs, energy_segments, latencies, adjusted_latencies, success = analyze_power_consumption(
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
