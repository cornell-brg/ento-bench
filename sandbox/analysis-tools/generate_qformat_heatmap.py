#!/usr/bin/env python3
"""
Q-Format Failure Rate Visualization Generator

Generates multiple visualization types for Q-format failure rates in attitude estimation:
- Line plots showing trends vs fractional bits
- Bar charts for easy comparison
- Diagonal heatmaps optimized for Q-format constraints
- Summary statistics

Usage:
    python tools/generate_qformat_heatmap.py [--input CSV_FILE] [--output-prefix PREFIX]

Author: EntoBench Team
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import argparse
import sys
from pathlib import Path

def setup_plot_style():
    """Configure matplotlib and seaborn for publication-quality plots."""
    plt.style.use('seaborn-v0_8-whitegrid')
    sns.set_palette("viridis")
    
    # Set font sizes for publication quality
    plt.rcParams.update({
        'font.size': 10,
        'axes.titlesize': 12,
        'axes.labelsize': 11,
        'xtick.labelsize': 9,
        'ytick.labelsize': 9,
        'legend.fontsize': 9,
        'figure.titlesize': 14
    })

def load_and_validate_data(csv_file):
    """Load CSV data and validate required columns."""
    try:
        df = pd.read_csv(csv_file)
        print(f"Loaded data: {len(df)} format test results")
        
        # Validate required columns
        required_cols = ['format_name', 'integer_bits', 'fractional_bits', 'total_bits', 
                        'is_float', 'failure_rate', 'test_succeeded']
        missing_cols = [col for col in required_cols if col not in df.columns]
        if missing_cols:
            raise ValueError(f"Missing required columns: {missing_cols}")
        
        # Show basic statistics
        successful_tests = df['test_succeeded'].sum()
        print(f"Successful tests: {successful_tests}/{len(df)} ({100*successful_tests/len(df):.1f}%)")
        
        # Show failure rate range
        fixed_point_df = df[~df['is_float']]
        if not fixed_point_df.empty:
            min_rate = fixed_point_df['failure_rate'].min()
            max_rate = fixed_point_df['failure_rate'].max()
            print(f"Failure rate range: {min_rate:.2f}% to {max_rate:.2f}%")
        
        return df
        
    except Exception as e:
        print(f"Error loading data from {csv_file}: {e}")
        sys.exit(1)

def create_trend_plot(df, output_file, kernel_name=None):
    """Create line plots showing failure rate trends vs fractional bits."""
    setup_plot_style()
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    
    # Filter data
    df_16 = df[(df['total_bits'] == 16) & (~df['is_float']) & (df['test_succeeded'])].copy()
    df_32 = df[(df['total_bits'] == 32) & (~df['is_float']) & (df['test_succeeded'])].copy()
    
    # 16-bit plot
    if not df_16.empty:
        df_16_sorted = df_16.sort_values('fractional_bits')
        ax1.plot(df_16_sorted['fractional_bits'], df_16_sorted['failure_rate'], 
                'o-', linewidth=2, markersize=6, color='red', alpha=0.7)
        ax1.set_title('16-bit Q-Formats\nFailure Rate vs Fractional Bits', fontweight='bold')
        ax1.set_xlabel('Fractional Bits')
        ax1.set_ylabel('Failure Rate (%)')
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(0, 105)
        
        # Annotate best 16-bit format
        best_16 = df_16.loc[df_16['failure_rate'].idxmin()]
        ax1.annotate(f'Best: {best_16["format_name"]}\n{best_16["failure_rate"]:.1f}%',
                    xy=(best_16['fractional_bits'], best_16['failure_rate']),
                    xytext=(10, -30), textcoords='offset points',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7),
                    arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
    
    # 32-bit plot  
    if not df_32.empty:
        df_32_sorted = df_32.sort_values('fractional_bits')
        
        # Use log scale for better visualization of low failure rates
        ax2.semilogy(df_32_sorted['fractional_bits'], df_32_sorted['failure_rate'] + 0.001, 
                    'o-', linewidth=2, markersize=6, color='blue', alpha=0.7)
        ax2.set_title('32-bit Q-Formats\nFailure Rate vs Fractional Bits (Log Scale)', fontweight='bold')
        ax2.set_xlabel('Fractional Bits')
        ax2.set_ylabel('Failure Rate (%) + 0.001')
        ax2.grid(True, alpha=0.3)
        
        # Annotate top 3 formats
        top_3 = df_32.nsmallest(3, 'failure_rate')
        colors = ['darkgreen', 'green', 'lightgreen']
        for i, (_, row) in enumerate(top_3.iterrows()):
            ax2.annotate(f'{row["format_name"]}: {row["failure_rate"]:.3f}%',
                        xy=(row['fractional_bits'], row['failure_rate'] + 0.001),
                        xytext=(10, 10 + i*15), textcoords='offset points',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor=colors[i], alpha=0.7),
                        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
    
    # Add floating-point reference
    float_df = df[df['is_float'] & df['test_succeeded']]
    if not float_df.empty:
        float_info = []
        for _, row in float_df.iterrows():
            float_info.append(f"{row['format_name']}: {row['failure_rate']:.3f}%")
        
        reference_text = "Floating-point baselines:\n" + "\n".join(float_info)
        fig.text(0.02, 0.02, reference_text, fontsize=10, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))
    
    # Dynamic title based on kernel
    if kernel_name:
        title = f'Q-Format Performance Trends for {kernel_name} Filter'
    else:
        title = 'Q-Format Performance Trends for Multi-Kernel Analysis'
    plt.suptitle(title, fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout()
    plt.subplots_adjust(top=0.88, bottom=0.15)
    
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Trend plot saved to: {output_file}")
    return True

def create_bar_chart(df, output_file, kernel_name=None):
    """Create bar chart showing top performers and worst performers."""
    setup_plot_style()
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # Get fixed-point data
    fixed_df = df[(~df['is_float']) & (df['test_succeeded'])].copy()
    
    # Top 10 best performers
    top_10 = fixed_df.nsmallest(10, 'failure_rate')
    colors_good = plt.cm.RdYlGn_r(np.linspace(0.2, 0.8, len(top_10)))
    
    bars1 = ax1.bar(range(len(top_10)), top_10['failure_rate'], color=colors_good)
    ax1.set_title('Top 10 Best Performing Q-Formats', fontweight='bold')
    ax1.set_xlabel('Q-Format')
    ax1.set_ylabel('Failure Rate (%)')
    ax1.set_xticks(range(len(top_10)))
    ax1.set_xticklabels(top_10['format_name'], rotation=45, ha='right')
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Add value labels on bars
    for i, (bar, rate) in enumerate(zip(bars1, top_10['failure_rate'])):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height + 0.1,
                f'{rate:.3f}%', ha='center', va='bottom', fontsize=8)
    
    # Bottom 10 (worst performers that aren't 100%)
    bottom_10 = fixed_df[fixed_df['failure_rate'] < 100].nlargest(10, 'failure_rate')
    colors_bad = plt.cm.Reds(np.linspace(0.4, 0.9, len(bottom_10)))
    
    bars2 = ax2.bar(range(len(bottom_10)), bottom_10['failure_rate'], color=colors_bad)
    ax2.set_title('Worst 10 Performing Q-Formats (Excluding 100% Failures)', fontweight='bold')
    ax2.set_xlabel('Q-Format')
    ax2.set_ylabel('Failure Rate (%)')
    ax2.set_xticks(range(len(bottom_10)))
    ax2.set_xticklabels(bottom_10['format_name'], rotation=45, ha='right')
    ax2.grid(True, alpha=0.3, axis='y')
    
    # Add value labels on bars
    for i, (bar, rate) in enumerate(zip(bars2, bottom_10['failure_rate'])):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{rate:.1f}%', ha='center', va='bottom', fontsize=8)
    
    # Add floating-point reference
    float_df = df[df['is_float'] & df['test_succeeded']]
    if not float_df.empty:
        float_info = []
        for _, row in float_df.iterrows():
            float_info.append(f"{row['format_name']}: {row['failure_rate']:.3f}%")
        
        reference_text = "Floating-point baselines: " + " | ".join(float_info)
        fig.text(0.5, 0.02, reference_text, ha='center', fontsize=10, 
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))
    
    # Dynamic title based on kernel
    if kernel_name:
        title = f'Q-Format Performance Comparison for {kernel_name} Filter'
    else:
        title = 'Q-Format Performance Comparison for Multi-Kernel Analysis'
    plt.suptitle(title, fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout()
    plt.subplots_adjust(top=0.93, bottom=0.15)
    
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Bar chart saved to: {output_file}")
    return True

def create_diagonal_heatmap(df, output_file):
    """Create optimized diagonal heatmap for Q-format visualization."""
    setup_plot_style()
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))
    
    # Prepare data for diagonal visualization
    def prepare_diagonal_data(data, total_bits):
        filtered = data[(data['total_bits'] == total_bits) & 
                       (~data['is_float']) & 
                       (data['test_succeeded'])].copy()
        
        if filtered.empty:
            return None, None
            
        # Create a square matrix with NaN for invalid combinations
        max_bits = total_bits
        matrix = np.full((max_bits, max_bits), np.nan)
        labels = np.full((max_bits, max_bits), '', dtype=object)
        
        for _, row in filtered.iterrows():
            i_bits = int(row['integer_bits']) - 1  # Convert to 0-indexed
            f_bits = int(row['fractional_bits']) - 1  # Convert to 0-indexed
            if 0 <= i_bits < max_bits and 0 <= f_bits < max_bits:
                matrix[f_bits, i_bits] = row['failure_rate']
                labels[f_bits, i_bits] = f"Q{row['integer_bits']}.{row['fractional_bits']}\n{row['failure_rate']:.2f}%"
        
        return matrix, labels
    
    # 16-bit diagonal heatmap
    matrix_16, labels_16 = prepare_diagonal_data(df, 16)
    if matrix_16 is not None:
        # Mask invalid combinations
        mask_16 = np.isnan(matrix_16)
        
        sns.heatmap(matrix_16, mask=mask_16, annot=labels_16, fmt='', 
                   cmap='RdYlBu_r', vmin=0, vmax=100, ax=ax1, cbar=False,
                   linewidths=1, linecolor='white', square=True)
        ax1.set_title('16-bit Q-Formats\n(Valid Diagonal Combinations)', fontweight='bold')
        ax1.set_xlabel('Integer Bits')
        ax1.set_ylabel('Fractional Bits')
        
        # Customize axis labels
        ax1.set_xticklabels(range(1, 17))
        ax1.set_yticklabels(range(1, 17))
        ax1.invert_yaxis()
    
    # 32-bit diagonal heatmap  
    matrix_32, labels_32 = prepare_diagonal_data(df, 32)
    if matrix_32 is not None:
        # Mask invalid combinations
        mask_32 = np.isnan(matrix_32)
        
        sns.heatmap(matrix_32, mask=mask_32, annot=labels_32, fmt='', 
                   cmap='RdYlBu_r', vmin=0, vmax=100, ax=ax2,
                   linewidths=1, linecolor='white', square=True,
                   cbar_kws={'label': 'Failure Rate (%)', 'shrink': 0.6})
        ax2.set_title('32-bit Q-Formats\n(Valid Diagonal Combinations)', fontweight='bold')
        ax2.set_xlabel('Integer Bits')
        ax2.set_ylabel('Fractional Bits')
        
        # Customize axis labels
        ax2.set_xticklabels(range(1, 33))
        ax2.set_yticklabels(range(1, 33))
        ax2.invert_yaxis()
    
    plt.suptitle('Q-Format Diagonal Constraint Visualization\n(Only valid I+F=Total combinations shown)', 
                fontsize=16, fontweight='bold', y=0.98)
    plt.tight_layout()
    plt.subplots_adjust(top=0.88)
    
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Diagonal heatmap saved to: {output_file}")
    return True

def create_failure_mode_breakdown(df, output_file_base):
    """Create stacked bar chart showing failure mode breakdown by format."""
    setup_plot_style()
    
    # Focus on formats with interesting failure patterns
    analysis_df = df[~df['is_float'] & df['test_succeeded']].copy()
    
    if analysis_df.empty:
        print("No fixed-point data available for failure mode analysis")
        return
    
    # Calculate failure mode percentages
    analysis_df['overflow_rate'] = (analysis_df['overflow'] / analysis_df['total_samples'] * 100)
    analysis_df['bad_norm_rate'] = (analysis_df['bad_norm'] / analysis_df['total_samples'] * 100) 
    analysis_df['near_zero_rate'] = (analysis_df['near_zero'] / analysis_df['total_samples'] * 100)
    analysis_df['excess_err_rate'] = (analysis_df['excess_err'] / analysis_df['total_samples'] * 100)
    
    # Select interesting formats (high failure rates or specific patterns)
    interesting_formats = analysis_df[analysis_df['failure_rate'] > 10].nlargest(10, 'failure_rate')
    
    if interesting_formats.empty:
        print("No formats with significant failure rates found")
        return
    
    # Create stacked bar chart
    fig, ax = plt.subplots(figsize=(12, 6))
    
    formats = interesting_formats['format_name']
    width = 0.8
    
    ax.bar(formats, interesting_formats['overflow_rate'], width, 
           label='Overflow/Saturation', color='#d62728')
    ax.bar(formats, interesting_formats['bad_norm_rate'], width,
           bottom=interesting_formats['overflow_rate'], 
           label='Bad Quaternion Norm', color='#ff7f0e')
    ax.bar(formats, interesting_formats['near_zero_rate'], width,
           bottom=interesting_formats['overflow_rate'] + interesting_formats['bad_norm_rate'],
           label='Near-Zero Divisors', color='#2ca02c')
    ax.bar(formats, interesting_formats['excess_err_rate'], width,
           bottom=interesting_formats['overflow_rate'] + interesting_formats['bad_norm_rate'] + interesting_formats['near_zero_rate'],
           label='Excessive Error', color='#9467bd')
    
    ax.set_ylabel('Failure Rate (%)')
    ax.set_xlabel('Q-Format')
    ax.set_title('Failure Mode Breakdown by Q-Format\n(Top 10 Highest Failure Rate Formats)')
    ax.legend()
    
    plt.xticks(rotation=45)
    plt.tight_layout()
    
    breakdown_file = output_file_base.replace('.png', '_breakdown.png')
    plt.savefig(breakdown_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Failure mode breakdown saved to: {breakdown_file}")

def generate_summary_statistics(df):
    """Print comprehensive summary statistics."""
    print("\n" + "="*60)
    print("COMPREHENSIVE Q-FORMAT ANALYSIS SUMMARY")
    print("="*60)
    
    # Overall statistics
    total_formats = len(df)
    successful_tests = df['test_succeeded'].sum()
    print(f"Total formats tested: {total_formats}")
    print(f"Successful tests: {successful_tests} ({100*successful_tests/total_formats:.1f}%)")
    
    # Floating-point baseline
    float_df = df[df['is_float'] & df['test_succeeded']]
    if not float_df.empty:
        print(f"\nFloating-point baselines:")
        for _, row in float_df.iterrows():
            print(f"  {row['format_name']}: {row['failure_rate']:.3f}% failure rate")
    
    # Fixed-point analysis
    fixed_df = df[~df['is_float'] & df['test_succeeded']]
    if not fixed_df.empty:
        print(f"\nFixed-point format analysis:")
        print(f"  Total fixed-point formats: {len(fixed_df)}")
        
        # Best performing formats
        best_formats = fixed_df.nsmallest(5, 'failure_rate')
        print(f"\nTop 5 best performing formats:")
        for _, row in best_formats.iterrows():
            print(f"  {row['format_name']}: {row['failure_rate']:.2f}% failure rate")
        
        # Worst performing formats  
        worst_formats = fixed_df.nlargest(5, 'failure_rate')
        print(f"\nTop 5 worst performing formats:")
        for _, row in worst_formats.iterrows():
            print(f"  {row['format_name']}: {row['failure_rate']:.2f}% failure rate")
        
        # Bit width analysis
        for bits in [16, 32]:
            bit_df = fixed_df[fixed_df['total_bits'] == bits]
            if not bit_df.empty:
                avg_failure = bit_df['failure_rate'].mean()
                print(f"\n{bits}-bit formats:")
                print(f"  Average failure rate: {avg_failure:.2f}%")
                print(f"  Range: {bit_df['failure_rate'].min():.2f}% to {bit_df['failure_rate'].max():.2f}%")
                
                # Best format for this bit width
                best = bit_df.loc[bit_df['failure_rate'].idxmin()]
                print(f"  Best format: {best['format_name']} ({best['failure_rate']:.2f}%)")

def main():
    parser = argparse.ArgumentParser(description='Generate Q-format failure rate visualizations')
    parser.add_argument('--input', '-i', default='qformat_sweep_results.csv',
                       help='Input CSV file with Q-format test results')
    parser.add_argument('--output-prefix', '-o', default='qformat_',
                       help='Output file prefix for visualizations')
    parser.add_argument('--breakdown', action='store_true',
                       help='Also generate failure mode breakdown chart')
    parser.add_argument('--type', choices=['all', 'trend', 'bar', 'diagonal'], 
                       default='all', help='Type of visualization to generate')
    args = parser.parse_args()

    # Validate input file
    if not Path(args.input).exists():
        print(f"Error: Input file '{args.input}' not found!")
        print("Run the comprehensive sweep first to generate the CSV data.")
        sys.exit(1)

    # Load and process data
    df = load_and_validate_data(args.input)

    # Generate all-kernel plots as before
    outputs = []
    success_count = 0

    if args.type in ['all', 'trend']:
        success = create_trend_plot(df, args.output_prefix + 'trends.png')
        if success:
            outputs.append(f"  - Trend plot: {args.output_prefix}trends.png")
            success_count += 1

    if args.type in ['all', 'bar']:
        success = create_bar_chart(df, args.output_prefix + 'comparison.png')
        if success:
            outputs.append(f"  - Bar chart: {args.output_prefix}comparison.png")
            success_count += 1

    if args.type in ['all', 'diagonal']:
        success = create_diagonal_heatmap(df, args.output_prefix + 'diagonal.png')
        if success:
            outputs.append(f"  - Diagonal heatmap: {args.output_prefix}diagonal.png")
            success_count += 1

    # Now generate per-kernel plots
    if 'kernel' in df.columns:
        for kernel in df['kernel'].unique():
            kernel_df = df[df['kernel'] == kernel]
            kname = kernel.lower()
            print(f"\n=== Generating plots for kernel: {kernel} ===")
            if args.type in ['all', 'trend']:
                create_trend_plot(kernel_df, f"{args.output_prefix}trends_{kname}.png", kernel)
            if args.type in ['all', 'bar']:
                create_bar_chart(kernel_df, f"{args.output_prefix}comparison_{kname}.png", kernel)
            if args.type in ['all', 'diagonal']:
                create_diagonal_heatmap(kernel_df, f"{args.output_prefix}diagonal_{kname}.png")
            if args.breakdown:
                create_failure_mode_breakdown(kernel_df, f"{args.output_prefix}breakdown_{kname}.png")
            generate_summary_statistics(kernel_df)

    if success_count > 0:
        print("Visualization generation completed successfully!")
        if args.breakdown:
            create_failure_mode_breakdown(df, args.output_prefix + 'breakdown.png')
            outputs.append(f"  - Failure breakdown: {args.output_prefix}breakdown.png")
        generate_summary_statistics(df)
        print(f"\nGenerated outputs:")
        for output in outputs:
            print(output)
    else:
        print("Failed to generate any visualizations!")
        sys.exit(1)

if __name__ == '__main__':
    main() 