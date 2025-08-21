#!/usr/bin/env python3
"""
Comprehensive Multi-Kernel Q-Format Comparison Generator

Creates comprehensive comparison plots showing:
- All kernels (Madgwick vs Mahoney) on same plot
- All modes (IMU vs MARG) on same plot  
- Focus on 32-bit formats (since 16-bit mostly fail)
- Different colors for estimators, different line styles for modes

Usage:
    python tools/generate_comprehensive_comparison.py --input CSV_FILE [--output PREFIX]

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
    
    # Set font sizes for publication quality
    plt.rcParams.update({
        'font.size': 12,
        'axes.titlesize': 14,
        'axes.labelsize': 13,
        'xtick.labelsize': 11,
        'ytick.labelsize': 11,
        'legend.fontsize': 11,
        'figure.titlesize': 16
    })

def load_and_validate_data(csv_file):
    """Load CSV data and validate required columns."""
    try:
        df = pd.read_csv(csv_file)
        print(f"Loaded data: {len(df)} format test results")
        
        # Validate required columns
        required_cols = ['Kernel', 'Format', 'IntegerBits', 'FractionalBits', 'TotalBits', 
                        'IsFloat', 'Mode', 'FailureRate', 'TestSucceeded']
        missing_cols = [col for col in required_cols if col not in df.columns]
        if missing_cols:
            raise ValueError(f"Missing required columns: {missing_cols}")
        
        return df
        
    except Exception as e:
        print(f"Error loading data from {csv_file}: {e}")
        sys.exit(1)

def create_comprehensive_32bit_comparison(df, output_file):
    """Create comprehensive comparison plot for 32-bit formats only."""
    setup_plot_style()
    
    # Filter to 32-bit fixed-point formats only
    df_32 = df[
        (df['TotalBits'] == 32) & 
        (~df['IsFloat']) & 
        (df['TestSucceeded'])
    ].copy()
    
    if df_32.empty:
        print("No 32-bit fixed-point data available")
        return False
    
    # Sort by fractional bits for proper line plotting
    df_32 = df_32.sort_values('FractionalBits')
    
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Define colors and line styles
    colors = {'Madgwick': '#1f77b4', 'Mahoney': '#ff7f0e'}  # Blue and Orange
    line_styles = {'IMU': '-', 'MARG': '--'}  # Solid and Dashed
    markers = {'IMU': 'o', 'MARG': 's'}  # Circle and Square
    
        # Plot each kernel-mode combination - focus on IMU vs MARG differences
    for kernel in df_32['Kernel'].unique():
        for mode in df_32['Mode'].unique():
            subset = df_32[(df_32['Kernel'] == kernel) & (df_32['Mode'] == mode)]
            
            if not subset.empty:
                ax.plot(subset['FractionalBits'], subset['FailureRate'],
                       color=colors[kernel], 
                       linestyle=line_styles[mode],
                       marker=markers[mode],
                       linewidth=1.5, 
                       markersize=4,
                       alpha=0.9,
                       label=f'{kernel} {mode}')
    
    # Customize plot
    ax.set_xlabel('Fractional Bits', fontweight='bold')
    ax.set_ylabel('Failure Rate (%)', fontweight='bold')
    ax.set_title('32-bit Q-Format Performance Comparison\nAll Kernels and Modes\n(Note: Madgwick≈Mahoney lines overlap)', 
                fontweight='bold', pad=20)
    
    # Use log scale for better visualization of low failure rates
    ax.set_yscale('log')
    ax.set_ylim(0.01, 110)
    
    # Grid and legend
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right', frameon=True, fancybox=True, shadow=True)
    
    # Annotate best performers
    best_overall = df_32.loc[df_32['FailureRate'].idxmin()]
    ax.annotate(f'Best: {best_overall["Format"]}\n{best_overall["Kernel"]} {best_overall["Mode"]}\n{best_overall["FailureRate"]:.3f}%',
               xy=(best_overall['FractionalBits'], best_overall['FailureRate']),
               xytext=(10, 30), textcoords='offset points',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.8),
               arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.2'))
    
    # Add floating-point reference
    float_df = df[(df['IsFloat']) & (df['TestSucceeded'])]
    if not float_df.empty:
        float_info = []
        for _, row in float_df.iterrows():
            float_info.append(f"{row['Format']} ({row['Kernel']} {row['Mode']}): {row['FailureRate']:.3f}%")
        
        reference_text = "Floating-point baselines:\n" + "\n".join(float_info[:4])  # Show first 4
        ax.text(0.02, 0.98, reference_text, transform=ax.transAxes, fontsize=10,
               verticalalignment='top',
               bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.8))
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Comprehensive 32-bit comparison saved to: {output_file}")
    return True

def create_performance_summary_table(df, output_file):
    """Create a summary table showing best formats per kernel-mode combination."""
    setup_plot_style()
    
    # Filter to successful tests only
    df_clean = df[df['TestSucceeded']].copy()
    
    # Create summary for each kernel-mode combination
    summary_data = []
    
    for kernel in df_clean['Kernel'].unique():
        for mode in df_clean['Mode'].unique():
            subset = df_clean[(df_clean['Kernel'] == kernel) & (df_clean['Mode'] == mode)]
            
            if not subset.empty:
                # Best 32-bit format
                subset_32 = subset[(subset['TotalBits'] == 32) & (~subset['IsFloat'])]
                if not subset_32.empty:
                    best_32 = subset_32.loc[subset_32['FailureRate'].idxmin()]
                    summary_data.append({
                        'Kernel': kernel,
                        'Mode': mode,
                        'Bit_Width': '32-bit',
                        'Best_Format': best_32['Format'],
                        'Failure_Rate': best_32['FailureRate'],
                        'Integer_Bits': best_32['IntegerBits'],
                        'Fractional_Bits': best_32['FractionalBits']
                    })
                
                # Best 16-bit format
                subset_16 = subset[(subset['TotalBits'] == 16) & (~subset['IsFloat'])]
                if not subset_16.empty:
                    best_16 = subset_16.loc[subset_16['FailureRate'].idxmin()]
                    summary_data.append({
                        'Kernel': kernel,
                        'Mode': mode,
                        'Bit_Width': '16-bit',
                        'Best_Format': best_16['Format'],
                        'Failure_Rate': best_16['FailureRate'],
                        'Integer_Bits': best_16['IntegerBits'],
                        'Fractional_Bits': best_16['FractionalBits']
                    })
    
    # Create DataFrame and pivot for better visualization
    summary_df = pd.DataFrame(summary_data)
    
    if summary_df.empty:
        print("No data available for summary table")
        return False
    
    # Create visualization
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.axis('tight')
    ax.axis('off')
    
    # Create table
    table_data = []
    headers = ['Kernel', 'Mode', 'Bit Width', 'Best Format', 'Failure Rate (%)', 'Int Bits', 'Frac Bits']
    
    for _, row in summary_df.iterrows():
        table_data.append([
            row['Kernel'],
            row['Mode'], 
            row['Bit_Width'],
            row['Best_Format'],
            f"{row['Failure_Rate']:.3f}",
            int(row['Integer_Bits']),
            int(row['Fractional_Bits'])
        ])
    
    table = ax.table(cellText=table_data, colLabels=headers, cellLoc='center', loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1.2, 2)
    
    # Color code by performance
    for i, row in enumerate(table_data):
        failure_rate = float(row[4])
        if failure_rate < 1.0:
            color = 'lightgreen'
        elif failure_rate < 10.0:
            color = 'lightyellow'
        else:
            color = 'lightcoral'
        
        for j in range(len(headers)):
            table[(i+1, j)].set_facecolor(color)
    
    # Header styling
    for j in range(len(headers)):
        table[(0, j)].set_facecolor('lightblue')
        table[(0, j)].set_text_props(weight='bold')
    
    ax.set_title('Best Q-Format Performance Summary\nBy Kernel and Mode', 
                fontweight='bold', pad=20, fontsize=16)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Performance summary table saved to: {output_file}")
    return True

def main():
    parser = argparse.ArgumentParser(description='Generate comprehensive multi-kernel Q-format comparison')
    parser.add_argument('--input', '-i', required=True,
                       help='Input CSV file with multi-kernel Q-format test results')
    parser.add_argument('--output-prefix', '-o', default='comprehensive_comparison',
                       help='Output file prefix for visualizations')
    args = parser.parse_args()

    # Validate input file
    if not Path(args.input).exists():
        print(f"Error: Input file '{args.input}' not found!")
        sys.exit(1)

    # Load and process data
    df = load_and_validate_data(args.input)

    # Generate comprehensive comparison plot
    success1 = create_comprehensive_32bit_comparison(df, f"{args.output_prefix}_32bit.png")
    
    # Generate summary table
    success2 = create_performance_summary_table(df, f"{args.output_prefix}_summary.png")
    
    if success1 or success2:
        print("\nComprehensive comparison generation completed successfully!")
        print(f"Generated outputs:")
        if success1:
            print(f"  - 32-bit comparison plot: {args.output_prefix}_32bit.png")
        if success2:
            print(f"  - Performance summary table: {args.output_prefix}_summary.png")
    else:
        print("Failed to generate visualizations!")
        sys.exit(1)

if __name__ == '__main__':
    main() 