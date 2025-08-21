#!/usr/bin/env python3

import re
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from pathlib import Path

def parse_results(filename):
    """Parse the comprehensive sweep results file."""
    results = []
    
    with open(filename, 'r') as f:
        content = f.read()
    
    # Find all format sections
    format_sections = re.findall(r'Testing format: ([^(]+) \(([^)]+)\)\n(.*?)(?=Testing format:|$)', 
                                content, re.DOTALL)
    
    for format_name, format_desc, section in format_sections:
        # Extract format info
        if '32-bit' in format_desc:
            bits = 32
            # Extract Q-format like Q3.29
            q_match = re.search(r'Q(\d+)\.(\d+)', format_name)
            if q_match:
                int_bits = int(q_match.group(1))
                frac_bits = int(q_match.group(2))
                q_format = f"Q{int_bits}.{frac_bits}"
            else:
                continue
        elif '16-bit' in format_desc:
            bits = 16
            q_match = re.search(r'Q(\d+)\.(\d+)', format_name)
            if q_match:
                int_bits = int(q_match.group(1))
                frac_bits = int(q_match.group(2))
                q_format = f"Q{int_bits}.{frac_bits}"
            else:
                continue
        elif 'Float' in format_name:
            bits = 32
            q_format = 'Float'
        elif 'Double' in format_name:
            bits = 64
            q_format = 'Double'
        else:
            continue
        
        # Parse algorithm results
        madgwick_match = re.search(r'Madgwick.*?(\d+\.\d+)%', section)
        mahoney_match = re.search(r'Mahoney.*?(\d+\.\d+)%', section)
        fourati_match = re.search(r'Fourati.*?(\d+\.\d+)%', section)
        
        if madgwick_match:
            madgwick_rate = float(madgwick_match.group(1))
        else:
            madgwick_rate = None
            
        if mahoney_match:
            mahoney_rate = float(mahoney_match.group(1))
        else:
            mahoney_rate = None
            
        if fourati_match:
            fourati_rate = float(fourati_match.group(1))
        else:
            fourati_rate = None
        
        # Determine mode (IMU vs MARG)
        mode = 'MARG' if 'MARG' in section else 'IMU'
        
        results.append({
            'format': q_format,
            'bits': bits,
            'mode': mode,
            'madgwick': madgwick_rate,
            'mahoney': mahoney_rate,
            'fourati': fourati_rate
        })
    
    return results

def create_plots(results_list, dataset_names):
    """Create comparison plots for all three algorithms."""
    
    # Filter for 32-bit formats only
    filtered_results = []
    for results in results_list:
        filtered_results.append([r for r in results if r['bits'] == 32])
    
    # Create figure with subplots
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    
    # Colors and styles
    colors = {'Madgwick': '#1f77b4', 'Mahoney': '#ff7f0e', 'Fourati': '#d62728'}
    line_styles = {'icm': '-', 'steering': '--', 'straight': ':'}
    
    for mode_idx, mode in enumerate(['IMU', 'MARG']):
        ax = axes[mode_idx]
        
        for dataset_idx, (dataset_name, results) in enumerate(zip(dataset_names, filtered_results)):
            # Filter for current mode
            mode_results = [r for r in results if r['mode'] == mode]
            
            # Sort by Q-format (extract numbers for sorting)
            def sort_key(r):
                if r['format'] == 'Float':
                    return (0, 0)
                elif r['format'] == 'Double':
                    return (0, 1)
                else:
                    q_match = re.match(r'Q(\d+)\.(\d+)', r['format'])
                    if q_match:
                        return (1, int(q_match.group(1)), int(q_match.group(2)))
                    return (2, 0, 0)
            
            mode_results.sort(key=sort_key)
            
            # Extract data
            formats = [r['format'] for r in mode_results]
            madgwick_rates = [r['madgwick'] if r['madgwick'] is not None else np.nan for r in mode_results]
            mahoney_rates = [r['mahoney'] if r['mahoney'] is not None else np.nan for r in mode_results]
            fourati_rates = [r['fourati'] if r['fourati'] is not None else np.nan for r in mode_results]
            
            # Plot each algorithm
            ax.plot(formats, madgwick_rates, color=colors['Madgwick'], 
                   linestyle=line_styles[dataset_name], linewidth=1.5, 
                   label=f'Madgwick ({dataset_name})' if mode_idx == 0 else "")
            ax.plot(formats, mahoney_rates, color=colors['Mahoney'], 
                   linestyle=line_styles[dataset_name], linewidth=1.5,
                   label=f'Mahoney ({dataset_name})' if mode_idx == 0 else "")
            ax.plot(formats, fourati_rates, color=colors['Fourati'], 
                   linestyle=line_styles[dataset_name], linewidth=1.5,
                   label=f'Fourati ({dataset_name})' if mode_idx == 0 else "")
        
        # Customize plot
        ax.set_title(f'{mode} Mode', fontsize=14, fontweight='bold')
        ax.set_xlabel('Q-Format', fontsize=12)
        ax.set_ylabel('Failure Rate (%)', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0, 100)
        
        # Rotate x-axis labels for better readability
        plt.setp(ax.get_xticklabels(), rotation=45, ha='right')
        
        # Add legend only to first subplot
        if mode_idx == 0:
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=10)
    
    plt.tight_layout()
    plt.savefig('all_three_algorithms_comparison.png', dpi=300, bbox_inches='tight')
    plt.show()
    
    # Also create individual dataset plots
    for dataset_idx, (dataset_name, results) in enumerate(zip(dataset_names, filtered_results)):
        fig, axes = plt.subplots(1, 2, figsize=(15, 6))
        
        for mode_idx, mode in enumerate(['IMU', 'MARG']):
            ax = axes[mode_idx]
            
            # Filter for current mode
            mode_results = [r for r in results if r['mode'] == mode]
            
            # Sort by Q-format
            def sort_key(r):
                if r['format'] == 'Float':
                    return (0, 0)
                elif r['format'] == 'Double':
                    return (0, 1)
                else:
                    q_match = re.match(r'Q(\d+)\.(\d+)', r['format'])
                    if q_match:
                        return (1, int(q_match.group(1)), int(q_match.group(2)))
                    return (2, 0, 0)
            
            mode_results.sort(key=sort_key)
            
            # Extract data
            formats = [r['format'] for r in mode_results]
            madgwick_rates = [r['madgwick'] if r['madgwick'] is not None else np.nan for r in mode_results]
            mahoney_rates = [r['mahoney'] if r['mahoney'] is not None else np.nan for r in mode_results]
            fourati_rates = [r['fourati'] if r['fourati'] is not None else np.nan for r in mode_results]
            
            # Plot each algorithm
            ax.plot(formats, madgwick_rates, color=colors['Madgwick'], 
                   linewidth=2, label='Madgwick')
            ax.plot(formats, mahoney_rates, color=colors['Mahoney'], 
                   linewidth=2, label='Mahoney')
            ax.plot(formats, fourati_rates, color=colors['Fourati'], 
                   linewidth=2, label='Fourati')
            
            # Customize plot
            ax.set_title(f'{dataset_name.capitalize()} Dataset - {mode} Mode', fontsize=14, fontweight='bold')
            ax.set_xlabel('Q-Format', fontsize=12)
            ax.set_ylabel('Failure Rate (%)', fontsize=12)
            ax.grid(True, alpha=0.3)
            ax.set_ylim(0, 100)
            ax.legend()
            
            # Rotate x-axis labels
            plt.setp(ax.get_xticklabels(), rotation=45, ha='right')
        
        plt.tight_layout()
        plt.savefig(f'{dataset_name}_algorithms_comparison.png', dpi=300, bbox_inches='tight')
        plt.show()

def main():
    # Parse results from all datasets
    datasets = ['icm', 'steering', 'straight']
    results_list = []
    
    for dataset in datasets:
        filename = f'{dataset}_results.txt'
        if Path(filename).exists():
            results = parse_results(filename)
            results_list.append(results)
            print(f"Parsed {len(results)} results from {dataset}")
        else:
            print(f"Warning: {filename} not found")
            results_list.append([])
    
    # Create plots
    create_plots(results_list, datasets)
    
    # Print summary statistics
    print("\n=== Summary Statistics ===")
    for dataset_idx, (dataset_name, results) in enumerate(zip(datasets, results_list)):
        if not results:
            continue
            
        print(f"\n{dataset_name.upper()} Dataset:")
        
        for mode in ['IMU', 'MARG']:
            mode_results = [r for r in results if r['mode'] == mode and r['bits'] == 32]
            
            if not mode_results:
                continue
                
            print(f"  {mode} Mode:")
            
            for algo in ['madgwick', 'mahoney', 'fourati']:
                rates = [r[algo] for r in mode_results if r[algo] is not None]
                if rates:
                    avg_rate = np.mean(rates)
                    min_rate = np.min(rates)
                    max_rate = np.max(rates)
                    print(f"    {algo.capitalize()}: avg={avg_rate:.2f}%, min={min_rate:.2f}%, max={max_rate:.2f}%")

if __name__ == "__main__":
    main() 