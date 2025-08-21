#!/usr/bin/env python3
"""
Create comparison plots between different datasets for key Q-formats.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def load_data(filename):
    """Load CSV data."""
    df = pd.read_csv(filename)
    df['IntegerBits'] = df['IntegerBits'].astype(int)
    df['FractionalBits'] = df['FractionalBits'].astype(int)
    df['TotalBits'] = df['TotalBits'].astype(int)
    df['FormatLabel'] = df['Format'].apply(lambda x: x.replace('Q', 'Q') if 'Q' in x else x)
    # Remove Q-format filter to include all data
    return df

def create_comparison_plot():
    """Create comparison plots between datasets."""
    datasets = [
        ('icm', 'ICM42688-P (Tuned)', 'multi_kernel_qformat_sweep_icm_results.csv'),
        ('steering', 'Gamma-Bot Steering', 'multi_kernel_qformat_sweep_steering_results.csv'),
        ('straight', 'Gamma-Bot Straight', 'multi_kernel_qformat_sweep_straight_results.csv')
    ]
    
    all_data = {}
    for key, name, filename in datasets:
        try:
            all_data[key] = load_data(filename)
            all_data[key]['IsFloat'] = all_data[key]['Format'].isin(['Float', 'Double'])
        except FileNotFoundError:
            print(f"Warning: {filename} not found")
            continue
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 7))
    # fig.suptitle('32-bit Q-Format Performance Comparison', fontsize=16, fontweight='bold') # Removed for paper
    
    colors = {'Madgwick': '#2E86AB', 'Mahoney': '#A23B72', 'Fourati': '#F18F01'}
    line_styles = {'icm': '-', 'steering': '--', 'straight': ':'}
    
    # Plotting function
    def plot_subplot(ax, title, bit_width, mode):
        ax.set_title(title)
        kernels = ['Madgwick', 'Mahoney'] if mode == 'IMU' else ['Madgwick', 'Mahoney', 'Fourati']
        
        for dataset_key, dataset_name, _ in datasets:
            if dataset_key not in all_data:
                continue
            
            data_subset = all_data[dataset_key]
            # Filter for non-float data
            filtered_data = data_subset[(data_subset['TotalBits'] == bit_width) & 
                                        (data_subset['Mode'] == mode) & 
                                        (data_subset['IsFloat'] == False)]
            
            for kernel in kernels:
                kernel_data = filtered_data[filtered_data['Kernel'].str.contains(kernel)].sort_values('IntegerBits')
                if not kernel_data.empty:
                    ax.plot(kernel_data['IntegerBits'], kernel_data['FailureRate'],
                            linestyle=line_styles[dataset_key], linewidth=1.5, markersize=4,
                            label=f'{dataset_name} ({kernel})', color=colors[kernel])

        ax.set_xlabel('Integer Bits')
        if show_ylabel:
            ax.set_ylabel('Failure Rate (%)')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 105)
        ax.set_xticks(np.arange(0, bit_width + 1, 2))

    # Create subplots for 32-bit only
    plot_subplot(ax1, '32-bit Q-Formats (IMU)', 32, 'IMU', show_ylabel=True)
    plot_subplot(ax2, '32-bit Q-Formats (MARG)', 32, 'MARG', show_ylabel=False)

    handles, labels = [], []
    for kernel, color in colors.items():
        handles.append(plt.Line2D([0], [0], color=color, lw=2))
        labels.append(kernel)
    for key, name, _ in datasets:
        handles.append(plt.Line2D([0], [0], color='gray', linestyle=line_styles[key], lw=2))
        labels.append(name.split(' (')[0])
        
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.05), ncol=6, fontsize=10)
    fig.tight_layout(rect=[0, 0.05, 1, 0.95])
    
    plt.savefig('qformat_comparison_32bit.png', dpi=300, bbox_inches='tight')
    plt.close()
    
    print("\nComparison plot saved as qformat_comparison_32bit.png")

def main():
    create_comparison_plot()

if __name__ == "__main__":
    main() 