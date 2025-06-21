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

def find_first_pulse( data, start_idx, end_idx, rising_threshold, falling_threshold ):
    # calculate expected pulse duration
    analyzer_start_time = data['time'].iloc[start_idx]
    analyzer_end_time   = data['time'].iloc[end_idx]
    analyzer_duration   = analyzer_end_time - analyzer_start_time

    search_start_time = analyzer_start_time - 0.1
    search_start_time = max( search_start_time, data['time'].iloc[0] )
    search_start_idx  = data[ data['time'] <= search_start_time ].index[-1]

    search_end_time = analyzer_end_time + 0.1
    search_end_time = min( search_end_time, data['time'].iloc[-1] )
    search_end_idx  = data[ data['time'] >= search_end_time ].index[0]

    # if the search area doesn't contain the pulse, we'll get bad thresholds
    # so use a big area to find the min and max currents
    current_window = data.iloc[0:search_end_idx]
    min_current    = min( current_window['current'] )
    max_current    = max( current_window['current'] )
    rise_thresh    = min_current + ( rising_threshold * ( max_current - min_current ) )
    fall_thresh    = min_current + ( falling_threshold * ( max_current - min_current ) )

    search_window = data.iloc[search_start_idx:search_end_idx]
    if not search_window[ search_window['current'] >= rise_thresh ].index.empty:
        pulse_start_idx = search_window[ search_window['current'] >= rise_thresh ].index[0]
        fall_window = data.iloc[pulse_start_idx:search_end_idx]
        if not fall_window[ fall_window['current'] <= fall_thresh ].index.empty:
            pulse_end_idx = fall_window[ fall_window['current'] <= fall_thresh ].index[0]
            pulse_start_time = data['time'].iloc[pulse_start_idx]
            pulse_end_time = data['time'].iloc[pulse_end_idx]
            pulse_duration = pulse_end_time - pulse_start_time
            if ( abs( pulse_duration - analyzer_duration ) < ( 0.1 * analyzer_duration ) ):
                return data.iloc[pulse_start_idx:pulse_end_idx], True

    search_window = data.iloc[0:search_end_idx]
    search_start_idx = search_window[ search_window['current'] <= fall_thresh ].index[0]
    search_window = data.iloc[search_start_idx:search_end_idx]
    while not search_window[ search_window['current'] > rise_thresh ].index.empty:
        pulse_start_idx = search_window[ search_window['current'] > rise_thresh ].index[0]
        fall_window = data.iloc[pulse_start_idx:search_end_idx]
        if not fall_window[ fall_window['current'] <= fall_thresh ].index.empty:
            pulse_end_idx = fall_window[ fall_window['current'] <= fall_thresh ].index[0]
            pulse_window = data.iloc[pulse_start_idx:pulse_end_idx]
            pulse_duration = pulse_window['time'].iloc[-1] - pulse_window['time'].iloc[0]
            if ( abs( pulse_duration - analyzer_duration ) < ( 0.10 * analyzer_duration ) ):
                return data.iloc[pulse_start_idx:pulse_end_idx], True
        search_start_idx = pulse_end_idx
        search_window = data.iloc[search_start_idx:search_end_idx]
    return None, False

def plot_segment( plot_dir, trace_width, vertical_width, window, seg_num, estimated_times, pulse_times ):
    plt.figure( figsize = ( 12, 6 ) )
    plt.plot( window['time'], window['current_mA'], linewidth = trace_width )
    plt.xlabel( 'Time (ms)' )
    plt.ylabel( 'Current (mA)' )
    plt.title( 'Segment {}'.format( seg_num ) )
    for time in estimated_times:
        plt.axvline( x = time, color = 'r', linestyle = '--', linewidth = vertical_width )
    for time in pulse_times:
        plt.axvline( x = time, color = 'b', linestyle = '--', linewidth = vertical_width )
    plt.legend()
    plt.savefig( os.path.join( plot_dir, 'segment_{}.png'.format( seg_num ) ) )
    plt.close()


def analyze_power_consumption(parent_dir, dataset_name, rising_threshold, falling_threshold, plot_data):
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
    avg_currents = []
    peak_currents = []
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

    data['current'] = data['current'].bfill().ffill()
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

    pulse_window, success = find_first_pulse( data, selected_indices[0], selected_indices[1], rising_threshold, falling_threshold )
    if not success:
        print( 'Failed to find the first pulse' )
        return [], [], [], [], False

    pulse_start_time = pulse_window['time'].iloc[0]
    pulse_end_time   = pulse_window['time'].iloc[-1]
    pulse_latency    = pulse_end_time - pulse_start_time
    adjusted_latencies.append( pulse_latency )

    analyzer_start_time = data['time'].iloc[selected_indices[0]]
    analyzer_end_time   = data['time'].iloc[selected_indices[1]]
    analyzer_latency    = analyzer_end_time - analyzer_start_time
    tdiffs.append( abs( analyzer_latency - pulse_latency ) )

    pulse_avg_current = np.mean( pulse_window['current_mA'] )
    pulse_energy      = np.trapz( pulse_window['current_mA'], pulse_window['time'] ) * voltage * 1e-6  # mJ

    energy_segments.append( pulse_energy )
    avg_currents.append( pulse_avg_current )
    total_energy += pulse_energy
    peak_current = max( pulse_window['current_mA'] )
    peak_currents.append( peak_current )

    if plot_data:
        analyzer_times = [ analyzer_start_time, analyzer_end_time ]
        pulse_times = [ pulse_start_time, pulse_end_time ]
        plot_segment( plot_dir, trace_width, vertical_width,
                      data.iloc[0:selected_indices[1]], 1, analyzer_times, pulse_times )

    prev_pulse_end_idx = pulse_window.index[-1]
    for i in range(2, len(selected_indices) - 1, 2):
        analyzer_start_idx  = selected_indices[i]
        analyzer_time_delta = data['time'].iloc[analyzer_start_idx] - analyzer_start_time
        analyzer_start_time = data['time'].iloc[analyzer_start_idx]
        analyzer_end_idx    = selected_indices[i + 1]
        analyzer_end_time   = data['time'].iloc[analyzer_end_idx]
        analyzer_duration   = analyzer_end_time - analyzer_start_time

        pulse_start_time_est = pulse_start_time + analyzer_time_delta
        pulse_end_time_est   = pulse_start_time_est + analyzer_duration

        search_start_time = pulse_start_time_est - 0.1
        search_start_time = max( data['time'].iloc[0], search_start_time )
        search_end_time   = pulse_end_time_est + 0.1
        search_end_time   = min( data['time'].iloc[-1], search_end_time )
        search_start_idx  = data[ data['time'] <= search_start_time ].index[-1]
        search_end_idx    = data[ data['time'] >= search_end_time ].index[0]
        search_window     = data.iloc[search_start_idx:search_end_idx]

        max_current = max( search_window['current'] )
        min_current = min( search_window['current'] )
        rise_thresh = min_current + ( rising_threshold * ( max_current - min_current ) )
        fall_thresh = min_current + ( falling_threshold * ( max_current - min_current ) )

        pulse_found  = False
        pulse_window = None

        # find rising edge
        pulse_start_idx = search_window[ search_window['current'] >= rise_thresh ].index[0]

        # find falling edge
        fall_window = data.iloc[pulse_start_idx:search_end_idx]
        if not fall_window[ fall_window['current'] <= fall_thresh ].index.empty:
            pulse_end_idx = fall_window[ fall_window['current'] <= fall_thresh ].index[0]
            pulse_window = data.iloc[pulse_start_idx:pulse_end_idx]

        # check pulse is similar to past pulses
        if pulse_window is not None:
            pulse_duration = pulse_window['time'].iloc[-1] - pulse_window['time'].iloc[0]
            pulse_avg_current = np.mean( pulse_window['current_mA'] )
            running_avg_current = np.mean( avg_currents )
            if  ( ( abs( pulse_duration - analyzer_duration ) < ( 0.10 * analyzer_duration ) ) and
                  ( abs( pulse_avg_current - running_avg_current ) < ( 0.05 * running_avg_current ) ) ):
                pulse_found = True

        # search for pulse starting from end of previous pulse
        if not pulse_found:
            search_start_idx = prev_pulse_end_idx + 1
            search_window    = data.iloc[search_start_idx:search_end_idx] 

            max_current = max( search_window['current'] )
            min_current = min( search_window['current'] )
            rise_thresh = min_current + ( rising_threshold * ( max_current - min_current ) )
            fall_thresh = min_current + ( falling_threshold * ( max_current - min_current ) )

            search_start_idx = search_window[ search_window['current'] <= fall_thresh ].index[0]
            search_window = data.iloc[search_start_idx:search_end_idx]
            while not search_window[ search_window['current'] > rise_thresh ].index.empty:
                pulse_start_idx = search_window[ search_window['current'] > rise_thresh ].index[0]
                fall_window = data.iloc[pulse_start_idx:search_end_idx]
                if not fall_window[ fall_window['current'] <= fall_thresh ].index.empty:
                    pulse_end_idx = fall_window[ fall_window['current'] <= fall_thresh ].index[0]
                    pulse_window = data.iloc[pulse_start_idx:pulse_end_idx]
                    pulse_duration = pulse_window['time'].iloc[-1] - pulse_window['time'].iloc[0]
                    pulse_avg_current = np.mean( pulse_window['current_mA'] )
                    running_avg_current = np.mean( avg_currents )
                    if  ( ( abs( pulse_duration - analyzer_duration ) < ( 0.10 * analyzer_duration ) ) and
                          ( abs( pulse_avg_current - running_avg_current ) < ( 0.05 * running_avg_current ) ) ):
                        pulse_found = True
                        break
                search_start_idx = pulse_end_idx
                search_window = data.iloc[search_start_idx:search_end_idx]

        if not pulse_found:
            print( 'Failed to find segment {}'.format( ( i // 2 ) + 1 ) )
            return [], [], [], [], False

        prev_pulse_end_idx = pulse_window.index[-1]

        pulse_start_time = pulse_window['time'].iloc[0]
        pulse_end_time   = pulse_window['time'].iloc[-1]
        pulse_latency    = pulse_end_time - pulse_start_time
        adjusted_latencies.append( pulse_latency )

        tdiffs.append( abs( analyzer_duration - pulse_latency ) )

        pulse_avg_current = np.mean( pulse_window['current_mA'] )
        pulse_energy      = np.trapz( pulse_window['current_mA'], pulse_window['time'] ) * voltage * 1e-6  # mJ

        energy_segments.append( pulse_energy )
        avg_currents.append( pulse_avg_current )
        total_energy += pulse_energy
        peak_current = max( pulse_window['current_mA'] )
        peak_currents.append( peak_current )

        if plot_data:
            estimated_times = [ pulse_start_time_est, pulse_end_time_est ]
            pulse_times   = [ pulse_start_time, pulse_end_time ]
            plot_segment( plot_dir, trace_width, vertical_width, search_window, ( ( i // 2 ) + 1 ), estimated_times, pulse_times )

    if plot_data:
        plt.legend(['Data', 'Latency Indices', 'Adjusted Indices'])
        plt.savefig(os.path.join(plot_dir, 'full_plot.png'))
        plt.close()

    average_energy = np.mean(energy_segments)
    average_current = np.mean(avg_currents)
    average_tdiff = np.mean(tdiffs) * 1e3
    average_latency = np.mean(latencies) * 1e3
    peak_current = np.max(peak_currents)
    peak_power = peak_current * voltage  # mW
    
    print("============================================================")
    print(f'Total energy consumed: {total_energy:.8f} mJ')
    print(f'Average current consumed: {average_current:.8f} mA')
    print(f'Average energy per segment: {average_energy:.8f} mJ')
    print(f'Latency stats (avg, max, min): {np.mean(latencies)*1e3:.6f} µs, {np.max(latencies)*1e3:.6f} µs, {np.min(latencies)*1e3:.6f} µs')
    print(f'Average time difference error (t_meas - t_adj): {average_tdiff:.6f} µs')
    print(f'Average time difference percentage: {100 * (average_tdiff / average_latency):.6f}%')
    print(f'Peak power: {peak_power:.3f} mW')

    return tdiffs, energy_segments, latencies, adjusted_latencies, success


def analyze_single_experiment(parent_dir, dataset_name, rising_threshold, falling_threshold, plot_data):
    print(f"Analyzing single dataset: {dataset_name}")
    
    tdiffs, energy_segments, latencies, adjusted_latencies, success = analyze_power_consumption(
        parent_dir, dataset_name, rising_threshold, falling_threshold, plot_data)
    
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
    if len(sys.argv) != 6:
        print("Usage: python3 analyze_single_experiment.py <parent_dir> <dataset_name> <rising_threshold> <falling_threshold> <plot_data>")
        sys.exit(1)

    parent_dir = sys.argv[1]
    dataset_name = sys.argv[2]
    rising_threshold = float(sys.argv[3])
    falling_threshold = float(sys.argv[4])
    plot_data = sys.argv[5].lower() == 'true'

    results = analyze_single_experiment(
        parent_dir,
        dataset_name,
        rising_threshold,
        falling_threshold,
        plot_data
    )

    return results


if __name__ == "__main__":
    main()
