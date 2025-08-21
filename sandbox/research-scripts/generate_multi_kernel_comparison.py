"""
Multi-Kernel Q-Format Comparison Visualizer

Generates specialized visualizations comparing different attitude estimation kernels
(Madgwick, Mahoney, Fourati) across Q-format configurations.

Usage:
    python tools/generate_multi_kernel_comparison.py [--input CSV_FILE] [--output-prefix PREFIX]

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
        print(f"Loaded data: {len(df)} kernel/format test results")
        
        # Validate required columns
        required_cols = ['kernel_name', 'format_name', 'integer_bits', 'fractional_bits', 
                        'total_bits', 'is_float', 'failure_rate', 'test_succeeded']
        missing_cols = [col for col in required_cols if col not in df.columns]
        if missing_cols:
            raise ValueError(f"Missing required columns: {missing_cols}")
        
        # Show basic statistics
        successful_tests = df['test_succeeded'].sum()
        print(f"Successful tests: {successful_tests}/{len(df)} ({100*successful_tests/len(df):.1f}%)")
        
        # Show kernels tested
        kernels = df['kernel_name'].str.split('_').str[0].unique()
        print(f"Kernels tested: {', '.join(kernels)}")
        
        return df
        
    except Exception as e:
        print(f"Error loading data from {csv_file}: {e}")
        sys.exit(1)

def create_kernel_comparison_chart(df, output_file):
    """Create simple comparison plot showing 4 lines for 32-bit formats."""
    setup_plot_style()
    
    # DEBUG: Print raw kernel names
    print("DEBUG: Raw kernel names in data:")
    print(df['kernel_name'].unique())
    
    # Extract kernel base names
    df['kernel_base'] = df['kernel_name'].str.split('_').str[0]
    
    # DEBUG: Print extracted kernel base names
    print("DEBUG: Extracted kernel base names:")
    print(df['kernel_base'].unique())
    
    # Filter to 32-bit fixed-point formats only
    fixed_32_df = df[(df['total_bits'] == 32) & (~df['is_float'])].copy()
    
    # DEBUG: Print filtered data info
    print(f"DEBUG: 32-bit fixed-point data shape: {fixed_32_df.shape}")
    print("DEBUG: Raw data sample:")
    print(fixed_32_df[['kernel_name', 'format_name', 'mode', 'fractional_bits', 'failure_rate']].head(10))
    print("\nDEBUG: Unique kernel-mode combinations in 32-bit data:")
    for kernel in fixed_32_df['kernel_base'].unique():
        for mode in fixed_32_df['mode'].unique():
            subset = fixed_32_df[(fixed_32_df['kernel_base'] == kernel) & (fixed_32_df['mode'] == mode)]
            if not subset.empty:
                print(f"  {kernel} {mode}: {len(subset)} samples")
                print(f"    Sample data: {subset[['format_name', 'fractional_bits', 'failure_rate']].head(3).to_dict('records')}")
    
    if fixed_32_df.empty:
        print("No 32-bit fixed-point data available")
        return False
    
    # Create single plot
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Define colors and line styles for the 4 combinations
    colors = {'Madgwick': '#1f77b4', 'Mahoney': '#ff7f0e'}  # Blue and Orange
    line_styles = {'IMU': '-', 'MARG': '--'}  # Solid and Dashed
    
    # Plot each kernel-mode combination
    for kernel in fixed_32_df['kernel_base'].unique():
        for mode in fixed_32_df['mode'].unique():
            subset = fixed_32_df[(fixed_32_df['kernel_base'] == kernel) & (fixed_32_df['mode'] == mode)]
            
            if not subset.empty:
                # Sort by fractional bits
                subset = subset.sort_values('fractional_bits')
                
                # Add small offset to avoid log(0)
                failure_rates = subset['failure_rate'] + 0.001
                
                print(f"DEBUG: Plotting {kernel} {mode} with {len(subset)} points")
                print(f"  Fractional bits range: {subset['fractional_bits'].min()} to {subset['fractional_bits'].max()}")
                print(f"  Failure rate range: {subset['failure_rate'].min():.3f} to {subset['failure_rate'].max():.3f}")
                
                ax.semilogy(subset['fractional_bits'], failure_rates,
                           color=colors[kernel],
                           linestyle=line_styles[mode],
                           linewidth=2,
                           marker='o',
                           markersize=4,
                           alpha=0.8,
                           label=f'{kernel} {mode}')
    
    # Customize plot
    ax.set_xlabel('Fractional Bits', fontweight='bold', fontsize=12)
    ax.set_ylabel('Failure Rate (%) + 0.001', fontweight='bold', fontsize=12)
    ax.set_title('32-bit Q-Format Performance Comparison\nMadgwick vs Mahoney (IMU vs MARG)', 
                fontweight='bold', fontsize=14, pad=20)
    
    # Set log scale with fixed range
    ax.set_yscale('log')
    ax.set_ylim(0.01, 110)
    
    # Grid and legend
    ax.grid(True, alpha=0.3, which='both')
    ax.legend(loc='upper right', frameon=True, fancybox=True, shadow=True, fontsize=11)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Simple kernel comparison saved to: {output_file}")
    return True

def create_kernel_heatmap(df, output_file):
    """Create heatmap showing kernel performance across formats."""
    setup_plot_style()
    
    # Extract kernel base names
    df['kernel_base'] = df['kernel_name'].str.split('_').str[0]
    
    # Focus on fixed-point formats for heatmap
    fixed_df = df[~df['is_float']].copy()
    
    if fixed_df.empty:
        print("No fixed-point data available for heatmap")
        return False
    
    # Create separate heatmaps for IMU and MARG modes
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
    
    for i, mode in enumerate(['IMU', 'MARG']):
        mode_df = fixed_df[fixed_df['mode'] == mode]
        
        if not mode_df.empty:
            # Create pivot table for this mode
            heatmap_data = mode_df.pivot_table(
                index='format_name',
                columns='kernel_base', 
                values='failure_rate',
                aggfunc='mean'
            )
            
            # Sort by average performance across kernels
            heatmap_data['avg'] = heatmap_data.mean(axis=1)
            heatmap_data = heatmap_data.sort_values('avg')
            heatmap_data = heatmap_data.drop('avg', axis=1)
            
            # Create heatmap
            sns.heatmap(heatmap_data, annot=True, fmt='.2f', cmap='RdYlBu_r', 
                       vmin=0, vmax=min(100, heatmap_data.max().max()), 
                       ax=ax1 if i == 0 else ax2,
                       linewidths=0.5, linecolor='white',
                       cbar_kws={'label': 'Failure Rate (%)', 'shrink': 0.8})
            
            (ax1 if i == 0 else ax2).set_title(f'Kernel Performance Heatmap - {mode} Mode\nFailure Rate (%) by Q-Format and Kernel', 
                                              fontweight='bold', pad=20)
            (ax1 if i == 0 else ax2).set_xlabel('Kernel')
            (ax1 if i == 0 else ax2).set_ylabel('Q-Format')
            
            # Rotate x-axis labels for better readability
            plt.setp((ax1 if i == 0 else ax2).get_xticklabels(), rotation=0)
            plt.setp((ax1 if i == 0 else ax2).get_yticklabels(), rotation=0)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"Kernel heatmap saved to: {output_file}")
    return True

def generate_summary_statistics(df):
    """Print comprehensive summary statistics for multi-kernel analysis."""
    print("\n" + "="*70)
    print("MULTI-KERNEL Q-FORMAT ANALYSIS SUMMARY")
    print("="*70)
    
    # Extract kernel base names
    df['kernel_base'] = df['kernel_name'].str.split('_').str[0]
    
    # Overall statistics
    total_tests = len(df)
    successful_tests = df['test_succeeded'].sum()
    print(f"Total tests conducted: {total_tests}")
    print(f"Successful tests: {successful_tests} ({100*successful_tests/total_tests:.1f}%)")
    
    # Kernels tested
    kernels = df['kernel_base'].unique()
    print(f"Kernels tested: {', '.join(kernels)}")
    
    # Floating-point baselines
    float_df = df[df['is_float']]
    if not float_df.empty:
        print(f"\nFloating-point baselines:")
        for kernel in kernels:
            kernel_float = float_df[float_df['kernel_base'] == kernel]
            if not kernel_float.empty:
                avg_rate = kernel_float['failure_rate'].mean()
                print(f"  {kernel}: {avg_rate:.3f}% average failure rate")
    
    # Fixed-point analysis by kernel
    fixed_df = df[~df['is_float']]
    if not fixed_df.empty:
        print(f"\nFixed-point format analysis:")
        
        for kernel in kernels:
            kernel_data = fixed_df[fixed_df['kernel_base'] == kernel]
            if not kernel_data.empty:
                print(f"\n{kernel} kernel:")
                print(f"  Formats tested: {len(kernel_data)}")
                
                # Best and worst
                best = kernel_data.loc[kernel_data['failure_rate'].idxmin()]
                worst = kernel_data.loc[kernel_data['failure_rate'].idxmax()]
                
                print(f"  Best format: {best['format_name']} ({best['failure_rate']:.3f}%)")
                print(f"  Worst format: {worst['format_name']} ({worst['failure_rate']:.3f}%)")
                print(f"  Average failure rate: {kernel_data['failure_rate'].mean():.3f}%")
    
    # Cross-kernel comparison on same formats
    print(f"\nCross-kernel comparison:")
    common_formats = set(df['format_name'])
    for kernel in kernels:
        kernel_formats = set(df[df['kernel_base'] == kernel]['format_name'])
        common_formats = common_formats.intersection(kernel_formats)
    
    if common_formats:
        print(f"Common formats tested: {len(common_formats)}")
        
        comparison_data = []
        for fmt in sorted(common_formats):
            fmt_data = df[df['format_name'] == fmt]
            comparison_data.append({
                'format': fmt,
                'kernels': {row['kernel_base']: row['failure_rate'] 
                           for _, row in fmt_data.iterrows()}
            })
        
        # Show best performing kernel for each format
        print(f"Best performing kernel by format:")
        for item in comparison_data[:10]:  # Show top 10
            fmt = item['format']
            kernel_rates = item['kernels']
            best_kernel = min(kernel_rates.keys(), key=lambda k: kernel_rates[k])
            best_rate = kernel_rates[best_kernel]
            print(f"  {fmt}: {best_kernel} ({best_rate:.3f}%)")

def main():
    """Main execution function."""
    parser = argparse.ArgumentParser(description='Generate multi-kernel Q-format comparison visualizations')
    parser.add_argument('--input', '-i', default='multi_kernel_qformat_sweep_results.csv',
                       help='Input CSV file with multi-kernel test results')
    parser.add_argument('--output-prefix', '-o', default='multi_kernel_',
                       help='Output file prefix for visualizations')
    
    args = parser.parse_args()
    
    # Check if input file exists
    if not Path(args.input).exists():
        print(f"Error: Input file '{args.input}' not found!")
        print("Run the comprehensive multi-kernel sweep first to generate the CSV data.")
        sys.exit(1)
    
    # Load and validate data
    df = load_and_validate_data(args.input)
    
    # Generate visualizations
    outputs = []
    
    success = create_kernel_comparison_chart(df, args.output_prefix + 'comparison.png')
    if success:
        outputs.append(f"  - Kernel comparison: {args.output_prefix}comparison.png")
    
    if outputs:
        print("Multi-kernel visualization generation completed successfully!")
        
        # Print summary statistics
        generate_summary_statistics(df)
        
        print(f"\nGenerated outputs:")
        for output in outputs:
            print(output)
        
    else:
        print("Failed to generate visualizations!")
        sys.exit(1)

if __name__ == '__main__':
    main() 