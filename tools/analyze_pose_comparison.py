#!/usr/bin/env python3
"""
Enhanced Pose Solver Comparison Analysis
Generates grouped bar charts comparing pose solvers for MCU applications
Supports both absolute and relative pose estimation
"""

import os
import re
import sys
import argparse
from pathlib import Path
from collections import defaultdict
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

class PoseAnalyzer:
    """Reusable analyzer for pose estimation benchmark results"""
    
    def __init__(self, results_dir, pose_type="absolute"):
        """
        Initialize analyzer
        
        Args:
            results_dir: Path to benchmark results directory
            pose_type: "absolute" or "relative"
        """
        self.results_dir = Path(results_dir)
        self.pose_type = pose_type
        self.data = defaultdict(lambda: defaultdict(dict))
        
    def get_solver_configs(self):
        """Return solver configurations based on pose type"""
        if self.pose_type == "absolute":
            return ["UP2P", "P3P", "DLT-6", "DLT-8", "DLT-16", "DLT-32", "DLT-64"]
        else:  # relative
            return ["8pt", "5pt", "upright-3pt", "planar-3pt", "planar-2pt"]
    
    def parse_logs(self):
        """Parse all log files and extract statistics"""
        logs_dir = self.results_dir / "logs"
        if not logs_dir.exists():
            print(f"Error: Logs directory not found: {logs_dir}")
            return False
            
        log_files = list(logs_dir.glob("*.log"))
        if not log_files:
            print(f"Error: No log files found in: {logs_dir}")
            return False
            
        print(f"Parsing {len(log_files)} log files...")
        
        for log_file in log_files:
            self._parse_single_log(log_file)
            
        if not self.data:
            print("Error: No data extracted from log files")
            return False
            
        print(f"Successfully parsed data for {len(self.data)} configurations")
        return True
    
    def _parse_single_log(self, log_file):
        """Parse a single log file"""
        try:
            with open(log_file, 'r') as f:
                content = f.read()
                
            # Extract configuration from filename
            config = self._parse_filename(log_file.name)
            if not config:
                return
                
            # Extract statistics for each noise level
            # Look for solver statistics sections with noise levels
            stats_pattern = r'=== (\w+) Statistics \(noise=([\d.]+)\) ===.*?Successful solves: (\d+) \(([^)]+)%\).*?Combined Error Statistics:\s+Mean ± Std Dev: ([\d.]+) ± ([\d.]+)'
            
            matches = re.findall(stats_pattern, content, re.DOTALL | re.IGNORECASE)
            
            for solver_name, noise_str, successes, success_rate_str, mean_error, std_error in matches:
                noise = float(noise_str)
                
                # Store the data
                key = f"{config['solver']}_{config['precision']}_{config['data_mode']}"
                if config['dlt_min']:
                    key += f"_dlt{config['dlt_min']}"
                
                self.data[key][noise] = {
                    'mean_error': float(mean_error),
                    'std_error': float(std_error),
                    'success_rate': float(success_rate_str),
                    'solver': config['solver'],
                    'precision': config['precision'],
                    'data_mode': config['data_mode'],
                    'dlt_min': config['dlt_min']
                }
                
        except Exception as e:
            print(f"Warning: Failed to parse {log_file.name}: {e}")
    
    def _parse_filename(self, filename):
        """Parse configuration from filename"""
        # Expected format: solver_precision_datamode_problems[_dltN].log
        # e.g., p3p_float_traditional_p1000.log or dlt_double_realistic_p1000_dlt16.log
        
        parts = filename.replace('.log', '').split('_')
        if len(parts) < 4:
            return None
            
        config = {
            'solver': parts[0],
            'precision': parts[1], 
            'data_mode': parts[2],
            'dlt_min': None
        }
        
        # Check for DLT minimum points specification
        for part in parts:
            if part.startswith('dlt') and part[3:].isdigit():
                config['dlt_min'] = int(part[3:])
                break
                
        return config
    
    def create_grouped_comparison_plot(self, noise_levels=None, data_mode="traditional", 
                                     output_file=None, figsize=(16, 8)):
        """
        Create grouped bar chart comparing all solvers
        
        Args:
            noise_levels: List of noise levels to include (default: [0.0, 0.01, 0.05])
            data_mode: "traditional" or "realistic" 
            output_file: Output filename (default: auto-generated)
            figsize: Figure size tuple
        """
        if noise_levels is None:
            noise_levels = [0.0, 0.01, 0.05]
            
        if output_file is None:
            output_file = f"{self.pose_type}_pose_comparison_{data_mode}.png"
            
        # Prepare data for plotting
        solver_configs = self._get_plottable_solver_configs()
        plot_data = self._prepare_plot_data(solver_configs, noise_levels, data_mode)
        
        if not plot_data:
            print("Error: No data available for plotting")
            return False
            
        # Create the plot
        fig, ax = plt.subplots(figsize=figsize)
        
        # Plot configuration
        n_solvers = len(solver_configs)
        n_noise = len(noise_levels)
        n_precision = 2  # float, double
        
        # Bar width and positioning
        bar_width = 0.35
        group_width = bar_width * n_precision
        group_spacing = 0.3
        total_group_width = group_width + group_spacing
        
        # Colors for noise levels (blue to red gradient)
        noise_colors = plt.cm.viridis(np.linspace(0, 1, n_noise))
        
        # X positions
        x_base = np.arange(n_solvers) * (total_group_width * n_noise + 1.0)
        
        # Track all bars for legend
        legend_handles = []
        legend_labels = []
        
        # Plot each noise level
        for noise_idx, noise in enumerate(noise_levels):
            noise_color = noise_colors[noise_idx]
            
            # Offset for this noise level group
            noise_offset = noise_idx * total_group_width
            
            # Float and double bars for this noise level
            float_positions = x_base + noise_offset
            double_positions = x_base + noise_offset + bar_width
            
            float_values = []
            float_errors = []
            double_values = []
            double_errors = []
            
            # Collect data for each solver
            for solver_config in solver_configs:
                float_key = f"{solver_config}_float_{data_mode}"
                double_key = f"{solver_config}_double_{data_mode}"
                
                # Get float data
                if float_key in plot_data and noise in plot_data[float_key]:
                    data = plot_data[float_key][noise]
                    float_values.append(data['mean_error'])
                    float_errors.append(data['std_error'])
                else:
                    float_values.append(0)
                    float_errors.append(0)
                    
                # Get double data  
                if double_key in plot_data and noise in plot_data[double_key]:
                    data = plot_data[double_key][noise]
                    double_values.append(data['mean_error'])
                    double_errors.append(data['std_error'])
                else:
                    double_values.append(0)
                    double_errors.append(0)
            
            # Plot bars
            float_bars = ax.bar(float_positions, float_values, bar_width, 
                              yerr=float_errors, capsize=3,
                              color=noise_color, alpha=0.7, 
                              label=f'Float (noise={noise})',
                              edgecolor='black', linewidth=0.5)
                              
            double_bars = ax.bar(double_positions, double_values, bar_width,
                               yerr=double_errors, capsize=3, 
                               color=noise_color, alpha=1.0,
                               label=f'Double (noise={noise})',
                               edgecolor='black', linewidth=0.5,
                               hatch='///')
            
            # Add to legend (only for first noise level to avoid clutter)
            if noise_idx == 0:
                legend_handles.extend([float_bars[0], double_bars[0]])
                legend_labels.extend(['Float', 'Double'])
        
        # Customize plot
        ax.set_xlabel('Solver Configuration', fontsize=12, fontweight='bold')
        ax.set_ylabel('Mean Pose Error (degrees)', fontsize=12, fontweight='bold')
        ax.set_title(f'{self.pose_type.title()} Pose Solver Comparison ({data_mode.title()} Data)', 
                    fontsize=14, fontweight='bold')
        
        # X-axis labels and positioning
        x_centers = x_base + (total_group_width * (n_noise - 1)) / 2
        ax.set_xticks(x_centers)
        ax.set_xticklabels([self._format_solver_label(config) for config in solver_configs], 
                          rotation=45, ha='right')
        
        # Add noise level labels at the top
        for noise_idx, noise in enumerate(noise_levels):
            noise_center = np.mean(x_base + noise_idx * total_group_width + bar_width/2)
            ax.text(noise_center, ax.get_ylim()[1] * 0.95, f'noise={noise}', 
                   ha='center', va='top', fontsize=10, fontweight='bold',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgray', alpha=0.7))
        
        # Legend
        precision_legend = ax.legend(legend_handles, legend_labels, 
                                   loc='upper left', title='Precision Type')
        ax.add_artist(precision_legend)
        
        # Noise level color legend
        noise_patches = [mpatches.Patch(color=noise_colors[i], label=f'noise={noise}') 
                        for i, noise in enumerate(noise_levels)]
        ax.legend(handles=noise_patches, loc='upper right', title='Noise Level')
        
        # Grid and styling
        ax.grid(True, alpha=0.3, axis='y')
        ax.set_axisbelow(True)
        
        # Layout and save
        plt.tight_layout()
        
        output_path = self.results_dir / output_file
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved plot: {output_path}")
        
        # Also save to current directory for convenience
        local_output = Path(output_file)
        plt.savefig(local_output, dpi=300, bbox_inches='tight')
        print(f"Saved plot: {local_output}")
        
        plt.show()
        return True
    
    def _get_plottable_solver_configs(self):
        """Get solver configurations that have data available"""
        available_configs = set()
        
        for key in self.data.keys():
            parts = key.split('_')
            solver = parts[0]
            
            if solver == 'dlt' and len(parts) >= 4:
                # Extract DLT configuration
                for part in parts:
                    if part.startswith('dlt') and part[3:].isdigit():
                        available_configs.add(f"DLT-{part[3:]}")
                        break
            else:
                available_configs.add(solver.upper())
        
        # Return in desired order
        all_configs = self.get_solver_configs()
        return [config for config in all_configs if config in available_configs]
    
    def _prepare_plot_data(self, solver_configs, noise_levels, data_mode):
        """Prepare data dictionary for plotting"""
        plot_data = {}
        
        for key, noise_data in self.data.items():
            parts = key.split('_')
            if len(parts) < 3:
                continue
                
            solver = parts[0]
            precision = parts[1]
            mode = parts[2]
            
            if mode != data_mode:
                continue
                
            # Create standardized key
            if solver == 'dlt':
                dlt_min = None
                for part in parts:
                    if part.startswith('dlt') and part[3:].isdigit():
                        dlt_min = part[3:]
                        break
                if dlt_min:
                    plot_key = f"DLT-{dlt_min}_{precision}_{data_mode}"
                else:
                    continue
            else:
                plot_key = f"{solver.upper()}_{precision}_{data_mode}"
            
            plot_data[plot_key] = noise_data
            
        return plot_data
    
    def _format_solver_label(self, config):
        """Format solver configuration for display"""
        if config.startswith('DLT-'):
            return config  # Already formatted
        return config
    
    def print_summary(self):
        """Print summary of parsed data"""
        print(f"\n=== {self.pose_type.title()} Pose Analysis Summary ===")
        print(f"Total configurations: {len(self.data)}")
        
        # Group by solver type
        solver_counts = defaultdict(int)
        for key in self.data.keys():
            solver = key.split('_')[0]
            solver_counts[solver] += 1
            
        for solver, count in sorted(solver_counts.items()):
            print(f"  {solver.upper()}: {count} configurations")
            
        # Show available noise levels
        all_noise_levels = set()
        for noise_data in self.data.values():
            all_noise_levels.update(noise_data.keys())
            
        print(f"Noise levels: {sorted(all_noise_levels)}")


def main():
    parser = argparse.ArgumentParser(description='Analyze pose estimation benchmark results')
    parser.add_argument('results_dir', help='Path to benchmark results directory')
    parser.add_argument('--pose-type', choices=['absolute', 'relative'], default='absolute',
                       help='Type of pose estimation to analyze')
    parser.add_argument('--noise-levels', nargs='+', type=float, default=[0.0, 0.01, 0.05],
                       help='Noise levels to include in plot')
    parser.add_argument('--data-mode', choices=['traditional', 'realistic'], default='realistic',
                       help='Data generation mode to analyze')
    parser.add_argument('--output', help='Output filename (default: auto-generated)')
    parser.add_argument('--figsize', nargs=2, type=int, default=[16, 8],
                       help='Figure size as width height')
    
    args = parser.parse_args()
    
    # Create analyzer
    analyzer = PoseAnalyzer(args.results_dir, args.pose_type)
    
    # Parse data
    if not analyzer.parse_logs():
        print("Failed to parse log files")
        return 1
        
    # Print summary
    analyzer.print_summary()
    
    # Create plot
    success = analyzer.create_grouped_comparison_plot(
        noise_levels=args.noise_levels,
        data_mode=args.data_mode,
        output_file=args.output,
        figsize=tuple(args.figsize)
    )
    
    if not success:
        print("Failed to create plot")
        return 1
        
    print("Analysis complete!")
    return 0


if __name__ == '__main__':
    sys.exit(main()) 