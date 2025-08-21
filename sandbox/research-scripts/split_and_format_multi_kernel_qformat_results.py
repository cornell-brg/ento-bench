"""
Multi-Kernel Q-Format Results Splitter and Formatter

This script processes the output CSV from comprehensive_multi_kernel_sweep.cc.

Features:
- Splits the results into one CSV per kernel (e.g., madgwick_qformat_results.csv, mahoney_qformat_results.csv, ...)
- Optionally outputs a unified, plotting-friendly CSV with only the most relevant columns for heatmap/summary plotting.

Usage:
    python split_and_format_multi_kernel_qformat_results.py --input multi_kernel_qformat_sweep_results.csv [--output-dir outdir] [--unified unified.csv]

Arguments:
    --input        Path to the multi-kernel sweep results CSV
    --output-dir   Directory to write per-kernel CSVs (default: current directory)
    --unified      (Optional) Path to write a unified, plotting-friendly CSV

Example:
    python split_and_format_multi_kernel_qformat_results.py --input multi_kernel_qformat_sweep_results.csv --output-dir kernel_csvs --unified unified_for_plotting.csv
"""
import os
import argparse
import pandas as pd

def main():
    parser = argparse.ArgumentParser(description="Split and format multi-kernel Q-format sweep results.")
    parser.add_argument('--input', required=True, help='Input CSV from comprehensive_multi_kernel_sweep.cc')
    parser.add_argument('--output-dir', default='.', help='Directory to write per-kernel CSVs')
    parser.add_argument('--unified', default=None, help='(Optional) Output path for unified plotting-friendly CSV')
    args = parser.parse_args()

    df = pd.read_csv(args.input)
    os.makedirs(args.output_dir, exist_ok=True)

    # Split by kernel
    for kernel in df['Kernel'].unique():
        kernel_df = df[df['Kernel'] == kernel]
        outname = os.path.join(args.output_dir, f"{kernel.lower()}_qformat_results.csv")
        kernel_df.to_csv(outname, index=False)
        print(f"Wrote {outname} ({len(kernel_df)} rows)")

    # Optionally write unified plotting-friendly CSV
    if args.unified:
        keep_cols = [
            'Kernel', 'Format', 'IntegerBits', 'FractionalBits', 'TotalBits', 'IsFloat',
            'Mode', 'FailureRate', 'TestSucceeded',
            'OverflowCount', 'BadNormCount', 'NearZeroDivCount', 'ExcessiveErrCount',
            'TotalSamples'
        ]
        # Rename columns for plotting script compatibility
        rename_map = {
            'Kernel': 'kernel_name',
            'Format': 'format_name',
            'IntegerBits': 'integer_bits',
            'FractionalBits': 'fractional_bits',
            'TotalBits': 'total_bits',
            'IsFloat': 'is_float',
            'Mode': 'mode',
            'FailureRate': 'failure_rate',
            'TestSucceeded': 'test_succeeded',
            'OverflowCount': 'overflow',
            'BadNormCount': 'bad_norm',
            'NearZeroDivCount': 'near_zero',
            'ExcessiveErrCount': 'excess_err',
            'TotalSamples': 'total_samples',
        }
        plot_df = df[keep_cols].rename(columns=rename_map)
        plot_df.to_csv(args.unified, index=False)
        print(f"Wrote unified plotting CSV: {args.unified} ({len(plot_df)} rows)")

if __name__ == "__main__":
    main() 