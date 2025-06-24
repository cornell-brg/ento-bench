import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import argparse

def find_combined_csvs(dataset_path, dataset_name):
    csv_files = []
    file_path = os.path.join(dataset_path, dataset_name)

    for root, _, files in os.walk(file_path):
        for file in files:
            if file.endswith("_combined.csv"):
                csv_files.append(os.path.join(root, file))

    return csv_files

def find_latest_rising_edge_idx( window, low_thresh, high_thresh ):
    if window[ window['current'] <= low_thresh ].index.empty:
        return None, False

    if window[ window['current'] >= high_thresh ].index.empty:
        return None, False

    latest_high_idx = window[ window['current'] >= high_thresh ].index[-1]
    search_window = window.loc[window.index[0]:latest_high_idx]

    # make sure there is a low value earlier than the latest high value
    if search_window[ search_window['current'] <= low_thresh ].index.empty:
        return None, False

    latest_low_idx = search_window[ search_window['current'] <= low_thresh ].index[-1]
    search_window = search_window.loc[latest_low_idx:latest_high_idx]
    return search_window[ search_window['current'] >= high_thresh ].index[0], True


def find_pulse_end_idx( window, pulse_start_idx, pulse_duration_est, low_thresh ):
    pulse_start_time   = window['time'].loc[pulse_start_idx]
    pulse_end_time_est = pulse_start_time + pulse_duration_est

    search_window_start_time = pulse_end_time_est - 0.01
    search_window_start_time = max( search_window_start_time, pulse_start_time + 0.01 )

    search_window_end_time = pulse_end_time_est + 0.01
    search_window_end_time = min( search_window_end_time, window['time'].iloc[-1] )

    search_window_start_idx = window[ window['time'] <= search_window_start_time ].index[-1]
    search_window_end_idx   = window[ window['time'] >= search_window_end_time ].index[0]

    search_window = window.loc[search_window_start_idx:search_window_end_idx]

    while search_window[ search_window['current'] <= low_thresh ].index.empty:
        new_search_start_time = search_window_start_time - 0.01
        new_search_start_time = max( new_search_start_time, pulse_start_time + 0.01 )

        new_search_end_time = search_window_end_time + 0.01
        new_search_end_time = min( new_search_end_time, window['time'].iloc[-1] )

        if ( ( new_search_start_time == search_window_start_time ) and
             ( new_search_end_time == search_window_end_time ) ):
            return None, False

        search_window_start_time = new_search_start_time
        search_window_end_time   = new_search_end_time

        search_window_start_idx = window[ window['time'] <= search_window_start_time ].index[-1]
        search_window_end_idx   = window[ window['time'] >= search_window_end_time ].index[0]

        search_window = window.loc[search_window_start_idx:search_window_end_idx]

    return search_window[ search_window['current'] <= low_thresh ].index[0] - 1, True



def find_first_pulse_window( data, start_idx, end_idx, quick_search_size, high_thresh_pct, low_thresh_pct, latency_error_ms, verbose ):
    # calculate expected pulse duration
    analyzer_start_time = data['time'].loc[start_idx]
    analyzer_end_time   = data['time'].loc[end_idx]
    analyzer_duration   = analyzer_end_time - analyzer_start_time

    search_start_time = analyzer_start_time - quick_search_size
    search_start_time = max( search_start_time, data['time'].iloc[0] )
    search_start_idx  = data[ data['time'] <= search_start_time ].index[-1]

    search_end_time = analyzer_end_time + quick_search_size
    search_end_time = min( search_end_time, data['time'].iloc[-1] )
    search_end_idx  = data[ data['time'] >= search_end_time ].index[0]

    if verbose:
        print( 'first pulse' )
        print( '  expected time: ( {}, {} )'.format( analyzer_start_time, analyzer_end_time ) )
        print( '  quick search window: ( {}, {} )'.format( search_start_time, search_end_time ) )

    # if the search area doesn't contain the pulse, we'll get bad thresholds
    # so use a big area to find the min and max currents
    # we make a copy so we can prune data from this df to refine our thresholds
    current_window = data.loc[data.index[0]:search_end_idx].copy()

    # if there are very high current spikes we might set the thresholds too high
    # so eliminate current values from non-experiment spikes from the threshold calculation
    # number of iterations is arbitrary
    iters = 0
    while iters < 5:
        if verbose:
            print( '  search iter {}'.format( iters ) )
        min_current    = min( current_window['current'] )
        max_current    = max( current_window['current'] )
        high_thresh    = min_current + ( high_thresh_pct * ( max_current - min_current ) )
        low_thresh     = min_current + ( low_thresh_pct * ( max_current - min_current ) )

        if verbose:
            print( '  current bounds: ( {}, {} )'.format( min_current, max_current ) )
            print( '  thresholds: ( {}, {} )'.format( low_thresh, high_thresh ) )

        # check for a pulse at the expected time from the analyzer
        search_window = data.loc[search_start_idx:search_end_idx]
        if not search_window[ search_window['current'] <= low_thresh ].index.empty:
            first_low_idx = search_window[ search_window['current'] <= low_thresh].index[0]
            search_window = search_window.loc[first_low_idx:search_end_idx]
            if not search_window[ search_window['current'] >= high_thresh ].index.empty:
                pulse_start_idx = search_window[ search_window['current'] >= high_thresh ].index[0]
                if verbose:
                    print( '  quick search rising edge: {}'.format( search_window['time'].loc[pulse_start_idx] ) )
                pulse_end_idx, success = find_pulse_end_idx( search_window, pulse_start_idx, analyzer_duration, low_thresh )
                if success:
                    pulse_window = data.loc[pulse_start_idx:pulse_end_idx]
                    pulse_duration = pulse_window['time'].iloc[-1] - pulse_window['time'].iloc[0]
                    pulse_charge = np.trapz( pulse_window['current'], pulse_window['time'] )
                    thresh_charge = high_thresh * analyzer_duration
                    if verbose:
                        print( '    falling edge: {}'.format( search_window['time'].loc[pulse_start_idx] ) )
                        print( '    duration: {}'.format( pulse_duration ) )
                        print( '    expected: {}'.format( analyzer_duration ) )
                        print( '    charge: {}'.format( pulse_charge ) )
                        print( '    threshold: {}'.format( thresh_charge ) )
                    if ( ( abs( pulse_duration - analyzer_duration ) < latency_error_ms ) and
                         ( pulse_charge > thresh_charge ) ):
                        return data.loc[pulse_start_idx:pulse_end_idx], True

        # check all pulses before the end of the expected time, dropping current data from false matches
        search_window = data.loc[data.index[0]:search_end_idx]
        if verbose:
            print( '  long search window ( {}, {} )'.format( search_window['time'].iloc[0], search_window['time'].iloc[-1] ) )
        while not search_window[ search_window['current'] >= high_thresh ].index.empty:
            pulse_start_idx, success = find_latest_rising_edge_idx( search_window, low_thresh, high_thresh )
            if success:
                if verbose:
                    print( '  long search rising edge: {}'.format( search_window['time'].loc[pulse_start_idx] ) )
                pulse_end_idx, success = find_pulse_end_idx( search_window, pulse_start_idx, analyzer_duration, low_thresh )
                if success:
                    pulse_window = data.loc[pulse_start_idx:pulse_end_idx]
                    pulse_duration = pulse_window['time'].iloc[-1] - pulse_window['time'].iloc[0]
                    pulse_charge = np.trapz( pulse_window['current'], pulse_window['time'] )
                    thresh_charge = high_thresh * analyzer_duration
                    if verbose:
                        print( '    falling edge: {}'.format( search_window['time'].loc[pulse_start_idx] ) )
                        print( '    duration: {}'.format( pulse_duration ) )
                        print( '    expected: {}'.format( analyzer_duration ) )
                        print( '    charge: {}'.format( pulse_charge ) )
                        print( '    threshold: {}'.format( thresh_charge ) )
                    if ( ( abs( pulse_duration - analyzer_duration ) < latency_error_ms ) and
                         ( pulse_charge > thresh_charge ) ):
                        return data.loc[pulse_start_idx:pulse_end_idx], True
                    else:
                        # drop current data using the first falling edge as the end of the pulse
                        fall_window = data.loc[pulse_start_idx:search_end_idx]
                        pulse_end_idx = fall_window[ fall_window['current'] <= low_thresh ].index[0]
                        pulse_window = search_window.loc[pulse_start_idx:pulse_end_idx]
                        current_window = current_window.drop( pulse_window.index.intersection( current_window.index ) )
                        # advance search window
                        search_window = search_window.loc[search_window.index[0]:pulse_start_idx-1]
                else:
                    pulse_window = search_window.loc[pulse_start_idx:search_window.index[-1]]
                    current_window = current_window.drop( pulse_window.index.intersection( current_window.index ) )
                    search_window = search_window.loc[search_window.index[0]:pulse_start_idx-1]

            else:
                # if we saw high values, but no pulse start, data must start above the high threshold
                pulse_end_idx = search_window[ search_window['current'] <= low_thresh ].index[0]
                pulse_window = data.loc[data.index[0]:pulse_end_idx]
                current_window = current_window.drop( pulse_window.index.intersection( current_window.index ) )
                break

        iters +=1

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


def analyze_power_consumption( args ):
    parent_dir         = args.parent_dir
    dataset_name       = args.dataset_name
    high_thresh_pct    = args.high_thresh
    low_thresh_pct     = args.low_thresh
    current_error_pct  = args.current_error
    latency_error_ms   = args.latency_error
    plot_data          = not args.no_plot
    quick_search_size  = args.quick_search_window
    pre_pulse_plot_ms  = args.plot_before
    post_pulse_plot_ms = args.plot_after
    verbose            = args.verbose

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

    pulse_window, success = find_first_pulse_window( data, selected_indices[0], selected_indices[1],
                                                     quick_search_size, high_thresh_pct, low_thresh_pct,
                                                     latency_error_ms, verbose )
    if not success:
        print( 'Failed to find the first segment' )
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

    plot_start_time = min( pulse_start_time, analyzer_start_time ) - pre_pulse_plot_ms
    plot_end_time   = max( pulse_end_time, analyzer_end_time ) + post_pulse_plot_ms
    plot_start_time = max( plot_start_time, data['time'].iloc[0] )
    plot_end_time   = min( plot_end_time, data['time'].iloc[-1] )
    plot_start_idx  = data[ data['time'] <= plot_start_time ].index[-1]
    plot_end_idx    = data[ data['time'] >= plot_end_time ].index[0]
    plot_window     = data.loc[plot_start_idx:plot_end_idx]

    energy_segments.append( pulse_energy )
    avg_currents.append( pulse_avg_current )
    total_energy += pulse_energy
    peak_currents.append( max( pulse_window['current_mA'] ) )

    if verbose:
        print( 'first segment:' )
        print( '  time: ( {}, {} )'.format( pulse_start_time, pulse_end_time ) )
        print( '  avg current: {}'.format( pulse_avg_current ) )

    if plot_data:
        analyzer_times = [ analyzer_start_time, analyzer_end_time ]
        pulse_times = [ pulse_start_time, pulse_end_time ]
        plot_segment( plot_dir, trace_width, vertical_width,
                      plot_window, 1, analyzer_times, pulse_times )

    prev_pulse_end_idx = pulse_window.index[-1]
    prev_analyzer_start_time = analyzer_start_time
    prev_pulse_start_time = pulse_start_time
    for i in range(2, len(selected_indices) - 1, 2):
        analyzer_start_idx  = selected_indices[i]
        analyzer_start_time = data['time'].loc[analyzer_start_idx]
        analyzer_end_idx    = selected_indices[i + 1]
        analyzer_end_time   = data['time'].loc[analyzer_end_idx]
        analyzer_duration   = analyzer_end_time - analyzer_start_time

        analyzer_time_delta  = analyzer_start_time - prev_analyzer_start_time
        pulse_start_time_est = prev_pulse_start_time + analyzer_time_delta
        pulse_end_time_est   = pulse_start_time_est + analyzer_duration

        prev_analyzer_start_time = analyzer_start_time

        search_start_time = pulse_start_time_est - quick_search_size
        search_start_time = max( data['time'].iloc[0], search_start_time )
        search_end_time   = pulse_end_time_est + quick_search_size
        search_end_time   = min( data['time'].iloc[-1], search_end_time )
        search_start_idx  = data[ data['time'] <= search_start_time ].index[-1]
        search_end_idx    = data[ data['time'] >= search_end_time ].index[0]
        search_window     = data.loc[search_start_idx:search_end_idx]

        high_current = np.mean( avg_currents ) * 1000
        min_current  = min( search_window['current'] )
        high_thresh  = min_current + ( high_thresh_pct * ( high_current - min_current ) )
        low_thresh   = min_current + ( low_thresh_pct * ( high_current - min_current ) )

        pulse_found = False
        pulse_window = None

        # search for pulse in expected location based on time delta
        if not search_window[ search_window['current'] <= low_thresh ].index.empty:
            first_low_idx = search_window[ search_window['current'] <= low_thresh ].index[0]
            search_window = search_window.loc[first_low_idx:search_end_idx]
            if not search_window[ search_window['current'] >= high_thresh ].index.empty:
                pulse_start_idx = search_window[ search_window['current'] >= high_thresh ].index[0]
                pulse_end_idx, success = find_pulse_end_idx( search_window, pulse_start_idx, analyzer_duration, low_thresh )
                if success:
                    pulse_window = data.loc[pulse_start_idx:pulse_end_idx]
                    pulse_duration = pulse_window['time'].iloc[-1] - pulse_window['time'].iloc[0]
                    pulse_avg_current = np.mean( pulse_window['current_mA'] )
                    running_avg_current = np.mean( avg_currents )
                    if  ( ( abs( pulse_duration - analyzer_duration ) < latency_error_ms ) and
                          ( abs( pulse_avg_current - running_avg_current ) < ( current_error_pct * running_avg_current ) ) ):
                        pulse_found = True
                    # else:
                    #     print( 'quick search failed' )
                    #     print( '  expected timing ( {}, {} )'.format( pulse_start_time_est, pulse_end_time_est ) )
                    #     print( '  found timing ( {}, {} )'.format( pulse_window['time'].iloc[0], pulse_window['time'].iloc[-1] ) )
                    #     print( '  expected current {}'.format( running_avg_current ) )
                    #     print( '  found current {}'.format( pulse_avg_current ) )

        # search for pulse from end of previous pulse to end of analyzer window
        pulses_checked = []
        if not pulse_found:
            search_start_idx = prev_pulse_end_idx + 1
            search_window    = data.loc[search_start_idx:search_end_idx] 

            min_current = min( search_window['current'] )
            low_thresh = min_current + ( low_thresh_pct * ( high_current - min_current ) )

            # print( 'performing slow search on segment {}'.format( ( i// 2 ) + 1 ) )
            # print( '  searching for duration {}'.format( analyzer_duration ) )
            # print( '  searching for current {}'.format( np.mean( avg_currents ) ) )
            # print( '  searching with thresholds ( {}, {} )'.format( low_thresh, high_thresh ) )
            # print( '  from current bounds ( {}, {} )'.format( min_current, high_current ) )
            # print( '  in current range ( {}, {} )'.format( min( search_window['current'] ), max( search_window['current'] ) ) )
            # print( '  estimated start time {}'.format( pulse_start_time_est ) ) 
            # print( '  estimated end time {}'.format( pulse_end_time_est ) ) 
            # print( '  ========================================' )

            while not search_window[ search_window['current'] >= high_thresh ].index.empty:
                pulse_start_idx, success= find_latest_rising_edge_idx( search_window, low_thresh, high_thresh )
                if not success:
                    # found no rising edges in the search window
                    break
                pulse_end_idx, success = find_pulse_end_idx( search_window, pulse_start_idx, analyzer_duration, low_thresh )
                if success:
                    pulse_window = data.loc[pulse_start_idx:pulse_end_idx]
                    pulse_duration = pulse_window['time'].iloc[-1] - pulse_window['time'].iloc[0]
                    pulse_avg_current = np.mean( pulse_window['current_mA'] )
                    running_avg_current = np.mean( avg_currents )
                    # print( 'checked for pulse at ( {}, {} )'.format( data['time'].loc[pulse_start_idx], data['time'].loc[pulse_end_idx] ) )
                    if ( ( abs( pulse_duration - analyzer_duration ) < latency_error_ms ) and
                         ( abs( pulse_avg_current - running_avg_current ) < ( current_error_pct * running_avg_current ) ) ):
                        pulse_found = True
                        break
                    if ( abs( pulse_duration - analyzer_duration ) > latency_error_ms ):
                        if ( pulse_duration < analyzer_duration ):
                            err_string = 'duration {} is too short, expecting {}'.format( pulse_duration, analyzer_duration )
                        else:
                            err_string = 'duration {} is too long, expecting {}'.format( pulse_duration, analyzer_duration )
                        pulses_checked.append( ( pulse_start_idx, pulse_end_idx, err_string ) )
                    if ( abs( pulse_avg_current - running_avg_current ) > ( current_error_pct * running_avg_current ) ):
                        if ( pulse_avg_current < running_avg_current ):
                            err_string = 'current {} is too low, expecting {}'.format( pulse_avg_current, running_avg_current )
                        else:
                            err_string = 'current {} is too high, expecting {}'.format( pulse_avg_current, running_avg_current )
                        pulses_checked.append( ( pulse_start_idx, pulse_end_idx, err_string ) )

                search_end_idx = pulse_start_idx - 1
                search_window = search_window.loc[search_start_idx:search_end_idx]

        if not pulse_found:
            print( 'Failed to find segment {}'.format( ( i // 2 ) + 1 ) )
            print( '  using edge thresholds: ( {}, {} )'.format( low_thresh / 1000, high_thresh / 1000 ) )
            print( '  estimated location: ( {}, {} )'.format( pulse_start_time_est, pulse_end_time_est ) )
            print( '  found candidates at:' )
            for pulse in pulses_checked:
                print( '    ( {}, {} ) - {}'.format( data['time'].loc[pulse[0]], data['time'].loc[pulse[1]], pulse[2] ) )
            search_end_idx    = data[ data['time'] >= search_end_time ].index[0]
            plot_start_time = pulse_start_time_est - 75
            plot_end_time   = pulse_end_time_est + 75
            plot_start_time = max( plot_start_time, data['time'].iloc[0] )
            plot_end_time   = min( plot_end_time, data['time'].iloc[-1] )
            plot_start_idx  = data[ data['time'] <= plot_start_time ].index[-1]
            plot_end_idx    = data[ data['time'] >= plot_end_time ].index[0]
            plot_window     = data.loc[plot_start_idx:plot_end_idx]
            estimated_times = [ pulse_start_time_est, pulse_end_time_est ]
            plot_segment( plot_dir, trace_width, vertical_width, plot_window, ( ( i // 2 ) + 1 ), estimated_times, [] )
            return [], [], [], [], False

        prev_pulse_end_idx = pulse_window.index[-1]

        pulse_start_time = pulse_window['time'].iloc[0]
        pulse_end_time   = pulse_window['time'].iloc[-1]
        pulse_latency    = pulse_end_time - pulse_start_time
        adjusted_latencies.append( pulse_latency )

        prev_pulse_start_time = pulse_start_time

        tdiffs.append( abs( analyzer_duration - pulse_latency ) )

        pulse_avg_current = np.mean( pulse_window['current_mA'] )
        pulse_energy      = np.trapz( pulse_window['current_mA'], pulse_window['time'] ) * voltage * 1e-6  # mJ

        energy_segments.append( pulse_energy )
        avg_currents.append( pulse_avg_current )
        total_energy += pulse_energy
        peak_currents.append( max( pulse_window['current_mA'] ) )

        if plot_data:
            plot_start_time = min( pulse_start_time, pulse_start_time_est ) - 55
            plot_end_time   = max( pulse_end_time, pulse_end_time_est ) + 35
            plot_start_time = max( plot_start_time, data['time'].iloc[0] )
            plot_end_time   = min( plot_end_time, data['time'].iloc[-1] )
            plot_start_idx  = data[ data['time'] <= plot_start_time ].index[-1]
            plot_end_idx    = data[ data['time'] >= plot_end_time ].index[0]
            plot_window     = data.loc[plot_start_idx:plot_end_idx]
            estimated_times = [ pulse_start_time_est, pulse_end_time_est ]
            pulse_times   = [ pulse_start_time, pulse_end_time ]
            plot_segment( plot_dir, trace_width, vertical_width, plot_window, ( ( i // 2 ) + 1 ), estimated_times, pulse_times )

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


def analyze_single_experiment( args ):
    print(f"Analyzing single dataset: {args.dataset_name}")
    
    tdiffs, energy_segments, latencies, adjusted_latencies, success = analyze_power_consumption( args )
    
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
    parser = argparse.ArgumentParser( formatter_class = argparse.ArgumentDefaultsHelpFormatter )
    parser.add_argument( 'parent_dir',
                         help = 'experiment parent directory' )
    parser.add_argument( 'dataset_name',
                         help = 'name of the experiment to analyze' )
    parser.add_argument( '--high-thresh',
                         help = 'fraction of high current to use as rising edge threshold',
                         type = float,
                         default = 0.7 )
    parser.add_argument( '--low-thresh',
                         help = 'fraction of high current to use as falling edge threshold',
                         type = float,
                         default = 0.3 )
    parser.add_argument( '--current-error',
                         help = 'maximum fraction of average current that a segement can be off by',
                         type = float,
                         default = 0.15 )
    parser.add_argument( '--latency-error',
                         help = 'maximum number of ms that latency can differ from the logic analyzer by',
                         type = float,
                         default = 0.025 )
    parser.add_argument( '--quick-search-window',
                         help = 'number of ms to add to each side of expected latency to create the quick search window',
                         type = float,
                         default = 0.1 )
    parser.add_argument( '--no-plot',
                         help = 'skip plot generation',
                         action = 'store_true' )
    parser.add_argument( '--plot-before',
                         help = 'number of ms before segment to include in plot',
                         type = float,
                         default = 55 )
    parser.add_argument( '--plot-after',
                         help = 'number of ms after segment to include in plot',
                         type = float,
                         default = 35 )
    parser.add_argument( '--verbose',
                         help = 'turn on verbose printing',
                         action = 'store_true' )
    args = parser.parse_args()

    results = analyze_single_experiment( args )

    return results


if __name__ == "__main__":
    main()
