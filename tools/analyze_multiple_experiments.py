import os
import numpy as np
import matplotlib.pyplot as plt
from natsort import natsorted
import pandas as pd
import sys


def find_combined_csvs(dataset_path, dataset_name):
    """Find combined CSV files for a specific dataset."""
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
    # Find all combined CSV files for the dataset
    csv_paths = find_combined_csvs(parent_dir, dataset_name)
    
    if not csv_paths:
        print(f"No combined.csv files found in {dataset_name}")
        return [], [], [], [], False
    
    # Use the first CSV file found for analysis
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

    # Find indices where 'latency' is 1
    latency_indices = np.where(data['latency'] == 1)[0]
    # If there's an odd number of latency events, drop the last one
    if len(latency_indices) % 2 != 0:
        latency_indices = latency_indices[:-1]
    selected_indices = latency_indices

    if len(selected_indices) < 2:
        print(f"Not enough latency events detected in {csv_path}")
        return [], [], [], [], False

    latencies = data['time'].iloc[selected_indices[1::2]].values - \
                data['time'].iloc[selected_indices[::2]].values

    # Convert current to mA
    data['current_mA'] = data['current'] / 1000

    # Plot parameters
    width_scale_factor = 0.5
    segment_width = 3 * width_scale_factor
    vertical_width = 3.5 * width_scale_factor
    trace_width = 2.5 * width_scale_factor

    if plot_data:
        plot_dir = os.path.join(os.path.dirname(csv_path), 'plots')
        os.makedirs(plot_dir, exist_ok=True)

    # Plotting full dataset
    if plot_data:
        plt.figure(figsize=(12, 6))
        plt.plot(data['time'], data['current_mA'], linewidth=trace_width)
        plt.xlabel('Time (ms)')
        plt.ylabel('Current (mA)')
        plt.title(f"PIL Test: {os.path.basename(csv_path)}")

        for idx in selected_indices:
            plt.axvline(x=data['time'].iloc[idx], color='r', linestyle='--', linewidth=vertical_width)

    # Process each latency segment
    for i in range(0, len(selected_indices) - 1, 2):
        idx1 = selected_indices[i]
        idx2 = selected_indices[i + 1]

        # Define search window
        duration = data['time'].iloc[idx2] - data['time'].iloc[idx1]
        search_start_time = (data['time'].iloc[idx2] - window_size * duration) if direction == 0 else (data['time'].iloc[idx1] + window_size * duration)
        search_start_time = max(search_start_time, data['time'].iloc[0])
        search_start = np.where(data['time'] >= search_start_time)[0][0]
        search_end = idx2

        if plot_data:
            plt.axvline(x=data['time'].iloc[search_start], color='c', linestyle='--', linewidth=vertical_width)
            plt.axvline(x=data['time'].iloc[search_end], color='c', linestyle='--', linewidth=vertical_width)

        current_window = data['current_mA'].iloc[search_start:search_end]
        min_current = current_window.min()
        max_current = current_window.max()
        current_threshold_rise = min_current + rising_threshold * (max_current - min_current)

        # Find rising edge
        spike_idx = find_spike_dynamic(current_window, current_threshold_rise)
        idx1_adj = search_start + spike_idx if spike_idx is not None else idx1

        # Adjusted times
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

        # Expand window if no drop is found
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
        
        # Handle any NaN values using ffill() and bfill()
        time_segment = time_segment.ffill().bfill()
        current_segment = current_segment.ffill().bfill()
        
        rel_lat_error = 100 * (tdiff / (tend - tstart))
        current = current_segment.mean()
        energy_segment = np.trapezoid(current_segment, time_segment) * voltage * 1e-6  # mJ

        if abs(rel_lat_error) > 10:
            # Use a different adjustment method based on current and time difference
            energy_adjustment = current * voltage * tdiff * 1e-3  # Convert ms to seconds for J calculation
        else:
            energy_adjustment = 0
            
        energy_segments.append(energy_segment + energy_adjustment)
        currents.append(current)
        total_energy += energy_segment + energy_adjustment

        if plot_data:
            plt.plot(data['time'].iloc[idx1_adj:idx2_adj],
                    data['current_mA'].iloc[idx1_adj:idx2_adj],
                    color='#7E2F8E', linestyle='--', marker='s',
                    linewidth=segment_width)
            plt.axvline(x=data['time'].iloc[idx1_adj], color='b',
                       linestyle='--', linewidth=vertical_width)
            plt.axvline(x=data['time'].iloc[idx2_adj], color='b',
                       linestyle='--', linewidth=vertical_width)

            # Save segment plot
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
            plt.axvline(x=data['time'].iloc[search_start], color='c',
                       linestyle='-', linewidth=vertical_width, label='Search Window')
            plt.axvline(x=data['time'].iloc[search_end], color='c',
                       linestyle='-', linewidth=vertical_width)
            plt.xlabel('Time (ms)')
            plt.ylabel('Current (mA)')
            plt.title(f'Segment: {(i+1)//2 + 1}')
            plt.legend()
            plt.savefig(os.path.join(plot_dir, f'segment_{(i+1)//2 + 1}.png'))
            plt.close()

    if plot_data:
        plt.legend(['Data', 'Latency Indices', 'Adjusted Indices', 'Search Window'])
        plt.savefig(os.path.join(plot_dir, 'full_plot.png'))
        plt.close()

    average_energy = np.mean(energy_segments)
    average_current = np.mean(currents)
    average_tdiff = np.mean(tdiffs) * 1e3  # Convert to µs
    average_latency = np.mean(latencies) * 1e3  # Convert to µs
    
    # Print Analysis Results
    print(f'Total energy consumed: {total_energy:.8f} mJ')
    print(f'Average current consumed: {average_current:.8f} mA')
    print(f'Average energy per segment: {average_energy:.8f} mJ')
    print(f'Latency stats (avg, max, min): {np.mean(latencies)*1e3:.6f} µs, {np.max(latencies)*1e3:.6f} µs, {np.min(latencies)*1e3:.6f} µs')
    print(f'Average cycles: {(average_latency/1e6)/(1/170e6):.6f}')
    print(f'Average time difference error (t_meas - t_adj): {average_tdiff:.6f} µs')
    print(f'Average time difference percentage: {100 * (average_tdiff / average_latency):.6f}%\n')

    return tdiffs, energy_segments, latencies, adjusted_latencies, success


def analyze_multiple_experiments(parent_dir, window_size, rising_threshold, falling_threshold, plot_data):
    # Get list of subdirectories
    subdirs = [d for d in os.listdir(parent_dir)
               if os.path.isdir(os.path.join(parent_dir, d))
               and d not in ['.', '..', 'plots']]
    subdirs = natsorted(subdirs)
    
    # Initialize results dictionary
    results = {}
    
    # Process each subdirectory
    for dataset_name in subdirs:
        print(f"Found dataset: {dataset_name}")
        
        # Skip directories ending with 'meas' or 'plots'
        if dataset_name.endswith('meas') or dataset_name == 'plots':
            print(f"Skipping non-matching experiment: {dataset_name}.\n")
            continue
        
        # Skip directories ending with 'dcache'
        if dataset_name.endswith('dcache'):
            print(f"Skipping non-matching experiment: {dataset_name}.\n")
            continue
        
        # Parse tokens for dimension extraction
        tokens = dataset_name.split('-')
        last_part = tokens[-1]
        
        # Dynamically adjust window size based on matrix dimensions (like MATLAB does)
        m_value = 0
        dlt_method = 0
        
        if last_part.endswith('x3'):
            parts = last_part.split('x')
            m_value = int(parts[0])
            n_value = int(parts[1])
            print(f"Detected 2Nx3 format with N = {m_value}")
            dlt_method = 1
        elif last_part.endswith('x9'):
            parts = last_part.split('x')
            m_value = int(parts[0])
            n_value = int(parts[1])
            dlt_method = 0
            print(f"Detected 2Nx9 format with N = {m_value}")
        else:
            # Try to extract numeric values from the string
            numeric_values = [int(s) for s in last_part if s.isdigit()]
            if numeric_values:
                m_value = int(''.join(map(str, numeric_values)))
                print(f"Extracted dimension: {m_value}")
            else:
                print(f"Skipping non-matching experiment: {dataset_name}")
                continue
        
        # Adjust window size based on matrix dimension
        if (m_value > 0) and (m_value < 8) and (dlt_method == 0):
            window_size = 20
        elif (m_value > 8) and (m_value < 32) and (dlt_method == 0):
            window_size = 10
        elif (m_value < 32) and (m_value > 0) and (dlt_method == 1):
            window_size = 20
        
        print(f"Analyzing dataset: {dataset_name}")
        
        # First try with direction = 0
        direction = 0
        tdiffs, energy_segments, latencies, adjusted_latencies, success = analyze_power_consumption(
            parent_dir, dataset_name, window_size, rising_threshold, falling_threshold, direction, plot_data)
        
        # If failed, try with direction = 1
        if not success:
            direction = 1
            tdiffs, energy_segments, latencies, adjusted_latencies, success = analyze_power_consumption(
                parent_dir, dataset_name, window_size, rising_threshold, falling_threshold, direction, plot_data)
            if success:
                print(f"Succeeded analyzing energy consumption with dir = {direction}")
        
        if not success:
            continue
        
        # Store the results
        normalized_name = dataset_name.replace('-', '_')
        
        # Extract N value for sorting
        n_value = m_value / 2  # Following the MATLAB logic for group_labels
        
        results[normalized_name] = {
            'tdiffs': tdiffs,
            'energy_segments': energy_segments,
            'average_energy': np.mean(energy_segments),
            'latencies': latencies,
            'adjusted_latencies': adjusted_latencies,
            'n_value': n_value
        }
    
    # Plot generation for results (similar to MATLAB)
    if results:
        plt.figure(figsize=(12, 6))
        
        # Extract dataset names, energies and N values
        dataset_names = list(results.keys())
        avg_energies = [results[name]['average_energy'] for name in dataset_names]
        n_values = [results[name]['n_value'] for name in dataset_names]
        
        # Sort by N values
        sorted_indices = np.argsort(n_values)
        sorted_n_values = [n_values[i] for i in sorted_indices]
        sorted_energies = [avg_energies[i] for i in sorted_indices]
        sorted_dataset_names = [dataset_names[i] for i in sorted_indices]
        
        # Generate group labels
        group_labels = [f"{int(n)}" for n in sorted_n_values]
        
        # Create color map based on unique N values
        unique_n_values = sorted(list(set(sorted_n_values)))
        cmap = plt.cm.jet(np.linspace(0, 1, len(unique_n_values)))
        bar_colors = np.zeros((len(sorted_n_values), 3))
        
        for i, n in enumerate(unique_n_values):
            indices = [j for j, x in enumerate(sorted_n_values) if x == n]
            for idx in indices:
                bar_colors[idx] = cmap[i, :3]  # Exclude alpha channel
        
        # Plot bar chart
        bars = plt.bar(range(len(sorted_energies)), sorted_energies)
        
        # Set colors for bars
        for i, bar in enumerate(bars):
            bar.set_color(cmap[unique_n_values.index(sorted_n_values[i])])
        
        plt.xticks(range(len(sorted_energies)), group_labels, rotation=45)
        plt.xlabel('N (Matrix Dimension)')
        plt.ylabel('Average Energy (mJ)')
        plt.title('Average Energy Consumption Across Experiments')
        plt.tight_layout()
        
        # Save plot
        plot_path = os.path.join(parent_dir, 'average_energy_consumption.png')
        plt.savefig(plot_path)
        plt.close()
    
    return results


def main():
    # Check if all command line arguments are given
    if len(sys.argv) != 6:
        print("Usage: python3 analyze_multiple_experiments.py <parent_dir> <window_size> <rising_threshold> <falling_threshold> <plot_data>")
        sys.exit(1)
    
    # Parse command line arguments
    parent_dir = sys.argv[1]
    window_size = float(sys.argv[2])
    rising_threshold = float(sys.argv[3])
    falling_threshold = float(sys.argv[4])
    plot_data = sys.argv[5].lower() == 'true'
    
    # Run the analysis
    results = analyze_multiple_experiments(
        parent_dir,
        window_size,
        rising_threshold,
        falling_threshold,
        plot_data
    )
    
    return results


if __name__ == "__main__":
    main()
