#!/usr/bin/env python3
"""
Plot Q-format failure rates for attitude estimation across different datasets.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

def load_and_process_data(filename):
    """Load CSV data and extract Q-format information."""
    df = pd.read_csv(filename)
    
    # Extract integer and fractional bits from format names
    df['IntegerBits'] = df['IntegerBits'].astype(int)
    df['FractionalBits'] = df['FractionalBits'].astype(int)
    df['TotalBits'] = df['TotalBits'].astype(int)
    
    # Create format labels
    df['FormatLabel'] = df['Format'].apply(lambda x: x.replace('Q', 'Q') if 'Q' in x else x)
    
    # Filter out float/double for cleaner plots
    df = df[df['Format'].str.contains('Q', na=False)]
    
    return df

def create_plot(data, dataset_name, save_path):
    """Create a comprehensive plot for one dataset."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    # Remove title for paper use
    # fig.suptitle(f'Fixed-Point Q-Format Failure Rates: {dataset_name} (32-bit)', fontsize=16, fontweight='bold')
    
    # Better color scheme for algorithms
    colors = {'Madgwick': '#2E86AB', 'Mahoney': '#A23B72', 'Fourati': '#F18F01'}
    
    # 1. 32-bit formats (IMU)
    data_32_imu = data[(data['TotalBits'] == 32) & (data['Mode'] == 'IMU')]
    for kernel in ['Madgwick', 'Mahoney', 'Fourati']:
        kernel_data = data_32_imu[data_32_imu['Kernel'].str.contains(kernel)]
        ax1.plot(kernel_data['IntegerBits'], kernel_data['FailureRate'], 
                marker='o', linewidth=1.5, markersize=4, label=kernel, color=colors[kernel])
    
    ax1.set_xlabel('Integer Bits')
    ax1.set_ylabel('Failure Rate (%)')
    ax1.set_title('32-bit Q-Formats (IMU)')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, 100)
    
    # 2. 32-bit formats (MARG)
    data_32_marg = data[(data['TotalBits'] == 32) & (data['Mode'] == 'MARG')]
    for kernel in ['Madgwick', 'Mahoney', 'Fourati']:
        kernel_data = data_32_marg[data_32_marg['Kernel'].str.contains(kernel)]
        ax2.plot(kernel_data['IntegerBits'], kernel_data['FailureRate'], 
                marker='s', linewidth=1.5, markersize=4, label=kernel, color=colors[kernel])
    
    ax2.set_xlabel('Integer Bits')
    ax2.set_ylabel('Failure Rate (%)')
    ax2.set_title('32-bit Q-Formats (MARG)')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim(0, 100)
    
    # Clean shared legend below subplots
    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.02), ncol=2, fontsize=10)
    
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.15)  # Make room for legend below
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    # Print summary statistics
    print(f"\n=== {dataset_name} Summary (32-bit) ===")
    print("32-bit IMU - Best formats:")
    best_32_imu = data_32_imu[data_32_imu['FailureRate'] < 10].sort_values('FailureRate')
    if not best_32_imu.empty:
        for _, row in best_32_imu.head(3).iterrows():
            print(f"  {row['FormatLabel']}: {row['FailureRate']:.1f}% ({row['Kernel']})")
    
    print("32-bit MARG - Best formats:")
    best_32_marg = data_32_marg[data_32_marg['FailureRate'] < 10].sort_values('FailureRate')
    if not best_32_marg.empty:
        for _, row in best_32_marg.head(3).iterrows():
            print(f"  {row['FormatLabel']}: {row['FailureRate']:.1f}% ({row['Kernel']})")

def main():
    """Main function to create plots for all datasets."""
    datasets = [
        ('icm', 'ICM42688-P (Tuned)', 'multi_kernel_qformat_sweep_icm_results.csv'),
        ('steering', 'Gamma-Bot Steering', 'multi_kernel_qformat_sweep_steering_results.csv'),
        ('straight', 'Gamma-Bot Straight', 'multi_kernel_qformat_sweep_straight_results.csv')
    ]
    
    for dataset_key, dataset_name, filename in datasets:
        print(f"\nProcessing {dataset_name}...")
        
        try:
            data = load_and_process_data(filename)
            save_path = f'qformat_failure_rates_{dataset_key}_32bit.png'
            create_plot(data, dataset_name, save_path)
            print(f"Plot saved as {save_path}")
            
        except FileNotFoundError:
            print(f"Warning: {filename} not found, skipping...")
        except Exception as e:
            print(f"Error processing {dataset_name}: {e}")
    
    print("\n=== Plotting Complete ===")
    print("Generated plots:")
    for dataset_key, dataset_name, _ in datasets:
        print(f"  - qformat_failure_rates_{dataset_key}_32bit.png ({dataset_name})")

if __name__ == "__main__":
    main() 