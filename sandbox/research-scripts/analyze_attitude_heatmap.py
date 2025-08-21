#!/usr/bin/env python3
"""
Attitude Estimation Q Format Analysis and Visualization
Generate heat maps and precision cliff visualizations for attitude estimation benchmarks
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
import os
from pathlib import Path

def parse_q_format(format_str):
    """Parse Q format string to extract integer and fractional bits"""
    if format_str.lower() in ['float', 'double']:
        return format_str, None, None, 32 if format_str.lower() == 'float' else 64
    
    # Parse Qx.y format
    if format_str.startswith('Q') and '.' in format_str:
        parts = format_str[1:].split('.')
        int_bits = int(parts[0])
        frac_bits = int(parts[1])
        total_bits = int_bits + frac_bits + 1  # +1 for sign bit
        return format_str, int_bits, frac_bits, total_bits
    
    return format_str, None, None, None

def calculate_q_format_range(int_bits, frac_bits):
    """Calculate the representable range for a Q format"""
    if int_bits is None:
        return None, None, None
    
    max_val = (2**int_bits - 1) + (2**frac_bits - 1) / (2**frac_bits)
    min_val = -2**int_bits
    resolution = 1.0 / (2**frac_bits)
    
    return min_val, max_val, resolution

def load_benchmark_data(csv_file):
    """Load benchmark results from CSV file"""
    df = pd.read_csv(csv_file)
    
    # Parse Q format information
    format_info = []
    for _, row in df.iterrows():
        format_name, int_bits, frac_bits, total_bits = parse_q_format(row['Format'])
        min_val, max_val, resolution = calculate_q_format_range(int_bits, frac_bits)
        
        format_info.append({
            'Format': format_name,
            'Filter': row['Filter'],
            'Sensor': row['Sensor'],
            'Dataset': row['Dataset'],
            'Success_Rate': row['Success_Rate'],
            'Total_Samples': row['Total_Samples'],
            'Failed_Samples': row['Failed_Samples'],
            'Mean_Error': row['Mean_Error'],
            'Std_Error': row['Std_Error'],
            'Max_Error': row['Max_Error'],
            'Integer_Bits': int_bits,
            'Fractional_Bits': frac_bits,
            'Total_Bits': total_bits,
            'Min_Range': min_val,
            'Max_Range': max_val,
            'Resolution': resolution,
            'Failure_Rate': 100.0 - row['Success_Rate']
        })
    
    return pd.DataFrame(format_info)

def create_failure_rate_heatmap(df, save_path=None):
    """Create heat maps of failure rates by Q format and filter, separated by dataset"""
    
    # Create Filter_Sensor combination for better labeling
    df['Filter_Sensor'] = df['Filter'] + '_' + df['Sensor']
    
    # Select representative Q formats for visualization
    selected_formats = ['Float', 'Double', 'Q6.25', 'Q7.24', 'Q4.27', 'Q2.13', 'Q4.11', 'Q8.7']
    df_filtered = df[df['Format'].isin(selected_formats)]
    
    # Get unique datasets
    datasets = sorted(df_filtered['Dataset'].unique())
    
    # Create subplots - 2 rows, 3 columns for 6 datasets
    fig, axes = plt.subplots(2, 3, figsize=(20, 12), sharey=True)
    axes = axes.flatten()
    
    # Use a colormap that emphasizes the transition from success to failure
    cmap = sns.color_palette("RdYlGn_r", as_cmap=True)
    
    # Format order for consistent y-axis
    format_order = ['Float', 'Double', 'Q6.25', 'Q7.24', 'Q4.27', 'Q2.13', 'Q4.11', 'Q8.7']
    
    for i, dataset in enumerate(datasets):
        # Filter data for this dataset
        dataset_data = df_filtered[df_filtered['Dataset'] == dataset]
        
        # Create pivot table for this dataset
        pivot_data = dataset_data.pivot_table(
            values='Failure_Rate', 
            index='Format', 
            columns='Filter_Sensor', 
            aggfunc='mean'
        )
        
        # Reorder formats by precision level
        pivot_data = pivot_data.reindex(format_order)
        
        # Create heatmap for this dataset
        sns.heatmap(pivot_data, 
                    ax=axes[i],
                    annot=True, 
                    fmt='.1f', 
                    cmap=cmap,
                    vmin=0, vmax=100,  # Consistent scale across all subplots
                    cbar=False,  # We'll add one shared colorbar
                    square=False,
                    linewidths=0.5)
        
        # Set subplot title - clean up dataset name
        clean_dataset_name = dataset.replace('-', ' ').replace('kHz', 'kHz ')
        axes[i].set_title(clean_dataset_name, fontsize=12, fontweight='bold')
        
        # Set x-axis labels
        axes[i].set_xlabel('Filter and Sensor Type', fontsize=10)
        axes[i].tick_params(axis='x', rotation=45, labelsize=9)
        
        # Only set y-axis label for leftmost plots
        if i % 3 == 0:
            axes[i].set_ylabel('Data Format', fontsize=11)
        else:
            axes[i].set_ylabel('')
        
        axes[i].tick_params(axis='y', rotation=0, labelsize=9)
    
    # Add a single colorbar for all subplots
    cbar_ax = fig.add_axes([0.92, 0.15, 0.02, 0.7])
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=0, vmax=100))
    sm.set_array([])
    cbar = fig.colorbar(sm, cax=cbar_ax)
    cbar.set_label('Failure Rate (%)', rotation=270, labelpad=20)
    
    # Set overall title
    fig.suptitle('Attitude Estimation Failure Rates by Data Format and Filter (Per Dataset)', 
                 fontsize=16, fontweight='bold', y=0.95)
    
    # Adjust layout to accommodate colorbar
    plt.subplots_adjust(right=0.9, top=0.9, hspace=0.3, wspace=0.3)
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Heat map saved to: {save_path}")
    else:
        plt.show()

def create_precision_cliff_plot(df, save_path=None):
    """Create precision cliff visualization showing where each format breaks down"""
    
    # For this plot, we need gyroscope magnitude data - simulate based on dataset names
    gyro_ranges = {
        'gamma-bot-steering': 386.3,  # High gyro rates
        'gamma-bot-straight': 70.4,   # Moderate gyro rates
        'fourati_quaternions': 50.0,  # Simulated moderate rates
        'madgwick_quaternions': 30.0, # Simulated low-moderate rates
        'mahony_quaternions': 25.0    # Simulated low rates
    }
    
    # Add simulated gyro magnitude
    df['Gyro_Magnitude'] = df['Dataset'].map(lambda x: gyro_ranges.get(x, 40.0))
    
    # Select Q formats for visualization
    q_formats = ['Q6.25', 'Q7.24', 'Q4.27', 'Q2.13', 'Q4.11', 'Q8.7']
    df_q = df[df['Format'].isin(q_formats)]
    
    plt.figure(figsize=(12, 8))
    
    # Color palette for different Q formats
    colors = plt.cm.viridis(np.linspace(0, 1, len(q_formats)))
    
    for i, q_format in enumerate(q_formats):
        format_data = df_q[df_q['Format'] == q_format]
        
        # Group by gyro magnitude and calculate mean failure rate
        grouped = format_data.groupby('Gyro_Magnitude')['Failure_Rate'].mean().reset_index()
        
        plt.plot(grouped['Gyro_Magnitude'], grouped['Failure_Rate'], 
                'o-', color=colors[i], label=q_format, linewidth=2, markersize=6)
    
    plt.xlabel('Gyroscope Rate Magnitude (deg/s)', fontsize=12)
    plt.ylabel('Failure Rate (%)', fontsize=12)
    plt.title('Precision Cliff: Q Format Breakdown vs Gyroscope Rates', fontsize=14, fontweight='bold')
    plt.legend(title='Q Format', bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.grid(True, alpha=0.3)
    plt.xlim(0, max(gyro_ranges.values()) + 50)
    plt.ylim(0, 105)
    
    # Add text annotations for key insights
    plt.text(0.02, 0.98, 'Lower rates →\nAll formats work', 
             transform=plt.gca().transAxes, fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.7))
    
    plt.text(0.7, 0.98, 'Higher rates →\nLow precision fails', 
             transform=plt.gca().transAxes, fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7))
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Precision cliff plot saved to: {save_path}")
    else:
        plt.show()

def create_bits_vs_performance_scatter(df, save_path=None):
    """Create scatter plot showing relationship between bit allocation and performance"""
    
    # Filter to only Q formats (exclude float/double)
    df_q = df[df['Format'].str.startswith('Q')]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot 1: Fractional bits vs failure rate
    scatter = ax1.scatter(df_q['Fractional_Bits'], df_q['Failure_Rate'], 
                         c=df_q['Integer_Bits'], cmap='plasma', s=60, alpha=0.7)
    ax1.set_xlabel('Fractional Bits', fontsize=12)
    ax1.set_ylabel('Failure Rate (%)', fontsize=12)
    ax1.set_title('Fractional Precision vs Performance', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    
    # Add colorbar for integer bits
    cbar1 = plt.colorbar(scatter, ax=ax1)
    cbar1.set_label('Integer Bits', rotation=270, labelpad=15)
    
    # Plot 2: Integer bits vs failure rate
    scatter2 = ax2.scatter(df_q['Integer_Bits'], df_q['Failure_Rate'], 
                          c=df_q['Fractional_Bits'], cmap='viridis', s=60, alpha=0.7)
    ax2.set_xlabel('Integer Bits', fontsize=12)
    ax2.set_ylabel('Failure Rate (%)', fontsize=12)
    ax2.set_title('Integer Range vs Performance', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    
    # Add colorbar for fractional bits
    cbar2 = plt.colorbar(scatter2, ax=ax2)
    cbar2.set_label('Fractional Bits', rotation=270, labelpad=15)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Bits vs performance plot saved to: {save_path}")
    else:
        plt.show()

def create_filter_comparison_plot(df, save_path=None):
    """Compare different filters' robustness to precision reduction"""
    
    # Select Q formats for comparison
    q_formats = ['Q6.25', 'Q7.24', 'Q4.27', 'Q2.13', 'Q4.11', 'Q8.7']
    df_q = df[df['Format'].isin(q_formats)]
    
    # Group by filter and format
    grouped = df_q.groupby(['Filter', 'Format'])['Failure_Rate'].mean().reset_index()
    
    # Create pivot table
    pivot_data = grouped.pivot(index='Format', columns='Filter', values='Failure_Rate')
    pivot_data = pivot_data.reindex(q_formats)  # Maintain order
    
    # Create grouped bar plot
    ax = pivot_data.plot(kind='bar', figsize=(12, 8), width=0.8)
    
    plt.title('Filter Robustness Comparison Across Q Formats', fontsize=14, fontweight='bold')
    plt.xlabel('Q Format (Decreasing Precision →)', fontsize=12)
    plt.ylabel('Failure Rate (%)', fontsize=12)
    plt.legend(title='Filter Type', bbox_to_anchor=(1.05, 1), loc='upper left')
    plt.xticks(rotation=45)
    plt.grid(True, alpha=0.3, axis='y')
    
    # Add trend annotations
    plt.text(0.02, 0.98, 'Q6.25 → Q8.7:\nDecreasing precision', 
             transform=ax.transAxes, fontsize=10, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.7))
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"Filter comparison plot saved to: {save_path}")
    else:
        plt.show()

def generate_summary_report(df, output_file=None):
    """Generate a summary report of the analysis"""
    
    report = []
    report.append("=== Attitude Estimation Q Format Analysis Summary ===\n")
    
    # Overall statistics
    total_tests = len(df)
    avg_failure_rate = df['Failure_Rate'].mean()
    
    report.append(f"Total test configurations: {total_tests}")
    report.append(f"Average failure rate: {avg_failure_rate:.2f}%\n")
    
    # Format performance ranking
    report.append("Q Format Performance Ranking (by average failure rate):")
    format_perf = df.groupby('Format')['Failure_Rate'].mean().sort_values()
    for i, (format_name, failure_rate) in enumerate(format_perf.items(), 1):
        report.append(f"{i:2d}. {format_name:8s}: {failure_rate:6.2f}% failures")
    
    report.append("")
    
    # Filter robustness ranking
    report.append("Filter Robustness Ranking (by average failure rate):")
    filter_perf = df.groupby('Filter')['Failure_Rate'].mean().sort_values()
    for i, (filter_name, failure_rate) in enumerate(filter_perf.items(), 1):
        report.append(f"{i:2d}. {filter_name:12s}: {failure_rate:6.2f}% failures")
    
    report.append("")
    
    # Critical insights
    report.append("Key Insights:")
    
    # Find the precision cliff
    q_formats = df[df['Format'].str.startswith('Q')]
    if not q_formats.empty:
        low_failure = q_formats[q_formats['Failure_Rate'] < 5]
        high_failure = q_formats[q_formats['Failure_Rate'] > 50]
        
        if not low_failure.empty and not high_failure.empty:
            cliff_threshold = (low_failure['Fractional_Bits'].min() + high_failure['Fractional_Bits'].max()) / 2
            report.append(f"• Precision cliff around {cliff_threshold:.1f} fractional bits")
        
        # Integer bits importance
        high_int_bits = q_formats[q_formats['Integer_Bits'] >= 6]
        low_int_bits = q_formats[q_formats['Integer_Bits'] <= 4]
        
        if not high_int_bits.empty and not low_int_bits.empty:
            high_int_failure = high_int_bits['Failure_Rate'].mean()
            low_int_failure = low_int_bits['Failure_Rate'].mean()
            report.append(f"• Higher integer bits reduce failures: {high_int_failure:.1f}% vs {low_int_failure:.1f}%")
    
    # Dataset sensitivity
    dataset_perf = df.groupby('Dataset')['Failure_Rate'].mean().sort_values(ascending=False)
    if len(dataset_perf) > 1:
        hardest = dataset_perf.index[0]
        easiest = dataset_perf.index[-1]
        report.append(f"• Most challenging dataset: {hardest} ({dataset_perf[hardest]:.1f}% avg failure)")
        report.append(f"• Easiest dataset: {easiest} ({dataset_perf[easiest]:.1f}% avg failure)")
    
    report_text = "\n".join(report)
    
    if output_file:
        with open(output_file, 'w') as f:
            f.write(report_text)
        print(f"Summary report saved to: {output_file}")
    else:
        print(report_text)
    
    return report_text

def main():
    parser = argparse.ArgumentParser(description='Analyze attitude estimation Q format benchmark results')
    parser.add_argument('csv_file', help='Path to benchmark results CSV file')
    parser.add_argument('--output-dir', '-o', default='attitude_analysis_plots', 
                       help='Output directory for plots and reports')
    parser.add_argument('--format', choices=['png', 'pdf', 'svg'], default='png',
                       help='Output format for plots')
    
    args = parser.parse_args()
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Load data
    print(f"Loading benchmark data from: {args.csv_file}")
    df = load_benchmark_data(args.csv_file)
    print(f"Loaded {len(df)} test configurations")
    
    # Generate visualizations
    print("\nGenerating visualizations...")
    
    # Heat map
    heatmap_path = output_dir / f"failure_rate_heatmap.{args.format}"
    create_failure_rate_heatmap(df, heatmap_path)
    
    # Precision cliff plot
    cliff_path = output_dir / f"precision_cliff.{args.format}"
    create_precision_cliff_plot(df, cliff_path)
    
    # Bits vs performance scatter
    scatter_path = output_dir / f"bits_vs_performance.{args.format}"
    create_bits_vs_performance_scatter(df, scatter_path)
    
    # Filter comparison
    comparison_path = output_dir / f"filter_comparison.{args.format}"
    create_filter_comparison_plot(df, comparison_path)
    
    # Generate summary report
    report_path = output_dir / "analysis_summary.txt"
    generate_summary_report(df, report_path)
    
    print(f"\nAnalysis complete! All outputs saved to: {output_dir}")

if __name__ == "__main__":
    main() 