#!/usr/bin/env python3
"""
Enhanced Absolute Pose Benchmark Analysis Script
Parses log files and generates comprehensive plots for performance analysis
Updated for dual statistics format (successful-only vs all-solutions)
"""

import os
import re
import sys
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import argparse
import matplotlib.cm as cm
import matplotlib.colors as colors

@dataclass
class BenchmarkResult:
    """Container for parsed benchmark results with dual statistics support"""
    solver: str
    precision: str
    data_mode: str
    problems: int
    dlt_min_points: Optional[int]
    noise_level: float
    total_problems: int
    successful_solves: int
    failed_solves: int
    success_rate: float
    
    # Successful-only statistics (threshold-based)
    success_rotation_error_mean: float
    success_rotation_error_std: float
    success_rotation_error_min: float
    success_rotation_error_max: float
    success_translation_error_mean: float
    success_translation_error_std: float
    success_translation_error_min: float
    success_translation_error_max: float
    success_reprojection_error_mean: float
    success_reprojection_error_std: float
    
    # All-solutions statistics (complete performance)
    all_rotation_error_mean: float
    all_rotation_error_std: float
    all_rotation_error_min: float
    all_rotation_error_max: float
    all_translation_error_mean: float
    all_translation_error_std: float
    all_translation_error_min: float
    all_translation_error_max: float
    all_reprojection_error_mean: float
    all_reprojection_error_std: float
    
    # Solution statistics
    avg_solutions_per_problem: float
    problems_with_multiple_solutions: int
    multiple_solutions_percentage: float

class BenchmarkAnalyzer:
    """Analyzes benchmark results and generates plots"""
    
    def __init__(self, results_dir: str):
        self.results_dir = Path(results_dir)
        self.logs_dir = self.results_dir / "logs"
        self.plots_dir = self.results_dir / "plots"
        self.plots_dir.mkdir(exist_ok=True)
        
        self.results: List[BenchmarkResult] = []
        
    def parse_log_file(self, log_file: Path) -> List[BenchmarkResult]:
        """Parse a single log file and extract benchmark results"""
        results = []
        
        # Parse run configuration from filename
        # Format: solver_precision_datamode_pN_noiseX.Y[_dltM].log
        filename = log_file.stem
        parts = filename.split('_')
        
        solver = parts[0]
        precision = parts[1] 
        data_mode = parts[2]
        problems = int(parts[3][1:])  # Remove 'p' prefix
        dlt_min_points = None
        
        # Extract DLT point count from filename (look for pattern like "dltN" at the end)
        for part in reversed(parts):
            if part.startswith('dlt') and len(part) > 3:
                try:
                    dlt_min_points = int(part[3:])  # Remove 'dlt' prefix
                    break
                except ValueError:
                    continue
            
        try:
            with open(log_file, 'r') as f:
                content = f.read()
                
            # Extract statistics for each noise level - updated for dual format
            stats_pattern = r'=== .* Statistics \(noise=([\d.]+)\) ===\s*\n(.*?)\n================================'
            stats_matches = re.findall(stats_pattern, content, re.DOTALL)
            
            for noise_str, stats_content in stats_matches:
                noise_level = float(noise_str)
                
                # Parse statistics content
                result = self._parse_statistics_section(
                    stats_content, solver, precision, data_mode, 
                    problems, dlt_min_points, noise_level
                )
                
                if result:
                    results.append(result)
                    
        except Exception as e:
            print(f"Error parsing {log_file}: {e}")
            
        return results
    
    def _parse_statistics_section(self, stats_content: str, solver: str, precision: str,
                                data_mode: str, problems: int, dlt_min_points: Optional[int],
                                noise_level: float) -> Optional[BenchmarkResult]:
        """Parse the statistics section with dual format support"""
        
        try:
            # Extract basic stats
            total_problems = int(re.search(r'Total problems: (\d+)', stats_content).group(1))
            successful_match = re.search(r'Successful solves: (\d+) \(([\d.]+)%\)', stats_content)
            successful_solves = int(successful_match.group(1))
            success_rate = float(successful_match.group(2))
            
            failed_match = re.search(r'Failed solves: (\d+)', stats_content)
            failed_solves = int(failed_match.group(1))
            
            # Parse SUCCESSFUL CASES ONLY section
            success_section_match = re.search(
                r'=== SUCCESSFUL CASES ONLY \(threshold-based\) ===\s*\n(.*?)(?=\n\s*=== ALL SOLUTIONS|$)',
                stats_content, re.DOTALL
            )
            
            if success_section_match:
                success_section = success_section_match.group(1)
                
                # Parse successful-only rotation errors
                success_rot_match = re.search(
                    r'Rotation Error Statistics:\s*\n\s*Mean ± Std Dev: ([\d.]+) ± ([\d.]+) degrees\s*\n\s*Min: ([\d.]+) degrees\s*\n\s*Max: ([\d.]+) degrees',
                    success_section
                )
                if success_rot_match:
                    success_rotation_error_mean = float(success_rot_match.group(1))
                    success_rotation_error_std = float(success_rot_match.group(2))
                    success_rotation_error_min = float(success_rot_match.group(3))
                    success_rotation_error_max = float(success_rot_match.group(4))
                else:
                    success_rotation_error_mean = success_rotation_error_std = success_rotation_error_min = success_rotation_error_max = 0.0
                
                # Parse successful-only translation errors  
                success_trans_match = re.search(
                    r'Translation Error Statistics:\s*\n\s*Mean ± Std Dev: ([\d.]+) ± ([\d.]+) meters\s*\n\s*Min: ([\d.]+) meters\s*\n\s*Max: ([\d.]+) meters',
                    success_section
                )
                if success_trans_match:
                    success_translation_error_mean = float(success_trans_match.group(1))
                    success_translation_error_std = float(success_trans_match.group(2))
                    success_translation_error_min = float(success_trans_match.group(3))
                    success_translation_error_max = float(success_trans_match.group(4))
                else:
                    success_translation_error_mean = success_translation_error_std = success_translation_error_min = success_translation_error_max = 0.0
                
                # Parse successful-only reprojection errors
                success_reproj_match = re.search(
                    r'Reprojection Error Statistics:\s*\n\s*Mean ± Std Dev: ([\d.]+) ± ([\d.]+) pixels',
                    success_section
                )
                if success_reproj_match:
                    success_reprojection_error_mean = float(success_reproj_match.group(1))
                    success_reprojection_error_std = float(success_reproj_match.group(2))
                else:
                    success_reprojection_error_mean = success_reprojection_error_std = 0.0
            else:
                # No successful cases - set all to zero
                success_rotation_error_mean = success_rotation_error_std = success_rotation_error_min = success_rotation_error_max = 0.0
                success_translation_error_mean = success_translation_error_std = success_translation_error_min = success_translation_error_max = 0.0
                success_reprojection_error_mean = success_reprojection_error_std = 0.0
            
            # Parse ALL SOLUTIONS section
            all_section_match = re.search(
                r'=== ALL SOLUTIONS \(complete performance\) ===\s*\n(.*?)(?=\n================================|$)',
                stats_content, re.DOTALL
            )
            
            if all_section_match:
                all_section = all_section_match.group(1)
                
                # Parse all-solutions rotation errors
                all_rot_match = re.search(
                    r'Rotation Error Statistics:\s*\n\s*Mean ± Std Dev: ([\d.]+) ± ([\d.]+) degrees\s*\n\s*Min: ([\d.]+) degrees\s*\n\s*Max: ([\d.]+) degrees',
                    all_section
                )
                if all_rot_match:
                    all_rotation_error_mean = float(all_rot_match.group(1))
                    all_rotation_error_std = float(all_rot_match.group(2))
                    all_rotation_error_min = float(all_rot_match.group(3))
                    all_rotation_error_max = float(all_rot_match.group(4))
                else:
                    all_rotation_error_mean = all_rotation_error_std = all_rotation_error_min = all_rotation_error_max = 0.0
                
                # Parse all-solutions translation errors
                all_trans_match = re.search(
                    r'Translation Error Statistics:\s*\n\s*Mean ± Std Dev: ([\d.]+) ± ([\d.]+) meters\s*\n\s*Min: ([\d.]+) meters\s*\n\s*Max: ([\d.]+) meters',
                    all_section
                )
                if all_trans_match:
                    all_translation_error_mean = float(all_trans_match.group(1))
                    all_translation_error_std = float(all_trans_match.group(2))
                    all_translation_error_min = float(all_trans_match.group(3))
                    all_translation_error_max = float(all_trans_match.group(4))
                else:
                    all_translation_error_mean = all_translation_error_std = all_translation_error_min = all_translation_error_max = 0.0
                
                # Parse all-solutions reprojection errors
                all_reproj_match = re.search(
                    r'Reprojection Error Statistics:\s*\n\s*Mean ± Std Dev: ([\d.]+) ± ([\d.]+) pixels',
                    all_section
                )
                if all_reproj_match:
                    all_reprojection_error_mean = float(all_reproj_match.group(1))
                    all_reprojection_error_std = float(all_reproj_match.group(2))
                else:
                    all_reprojection_error_mean = all_reprojection_error_std = 0.0
            else:
                # Fallback: use successful-only data for all-solutions if available, otherwise zero
                all_rotation_error_mean = success_rotation_error_mean
                all_rotation_error_std = success_rotation_error_std
                all_rotation_error_min = success_rotation_error_min
                all_rotation_error_max = success_rotation_error_max
                all_translation_error_mean = success_translation_error_mean
                all_translation_error_std = success_translation_error_std
                all_translation_error_min = success_translation_error_min
                all_translation_error_max = success_translation_error_max
                all_reprojection_error_mean = success_reprojection_error_mean
                all_reprojection_error_std = success_reprojection_error_std
            
            # Extract solution statistics
            avg_solutions_match = re.search(r'Average solutions per problem: ([\d.]+)', stats_content)
            avg_solutions_per_problem = float(avg_solutions_match.group(1)) if avg_solutions_match else 1.0
            
            multiple_solutions_match = re.search(r'Problems with multiple solutions: (\d+) \(([\d.]+)%\)', stats_content)
            if multiple_solutions_match:
                problems_with_multiple_solutions = int(multiple_solutions_match.group(1))
                multiple_solutions_percentage = float(multiple_solutions_match.group(2))
            else:
                problems_with_multiple_solutions = 0
                multiple_solutions_percentage = 0.0
                
            return BenchmarkResult(
                solver=solver,
                precision=precision,
                data_mode=data_mode,
                problems=problems,
                dlt_min_points=dlt_min_points,
                noise_level=noise_level,
                total_problems=total_problems,
                successful_solves=successful_solves,
                failed_solves=failed_solves,
                success_rate=success_rate,
                
                # Successful-only statistics
                success_rotation_error_mean=success_rotation_error_mean,
                success_rotation_error_std=success_rotation_error_std,
                success_rotation_error_min=success_rotation_error_min,
                success_rotation_error_max=success_rotation_error_max,
                success_translation_error_mean=success_translation_error_mean,
                success_translation_error_std=success_translation_error_std,
                success_translation_error_min=success_translation_error_min,
                success_translation_error_max=success_translation_error_max,
                success_reprojection_error_mean=success_reprojection_error_mean,
                success_reprojection_error_std=success_reprojection_error_std,
                
                # All-solutions statistics
                all_rotation_error_mean=all_rotation_error_mean,
                all_rotation_error_std=all_rotation_error_std,
                all_rotation_error_min=all_rotation_error_min,
                all_rotation_error_max=all_rotation_error_max,
                all_translation_error_mean=all_translation_error_mean,
                all_translation_error_std=all_translation_error_std,
                all_translation_error_min=all_translation_error_min,
                all_translation_error_max=all_translation_error_max,
                all_reprojection_error_mean=all_reprojection_error_mean,
                all_reprojection_error_std=all_reprojection_error_std,
                
                # Solution statistics
                avg_solutions_per_problem=avg_solutions_per_problem,
                problems_with_multiple_solutions=problems_with_multiple_solutions,
                multiple_solutions_percentage=multiple_solutions_percentage
            )
            
        except Exception as e:
            print(f"Error parsing statistics section: {e}")
            return None
    
    def load_all_results(self):
        """Load results from all log files"""
        if not self.logs_dir.exists():
            print(f"Logs directory not found: {self.logs_dir}")
            return
            
        for log_file in self.logs_dir.glob("*.log"):
            results = self.parse_log_file(log_file)
            self.results.extend(results)
            
        print(f"Loaded {len(self.results)} benchmark results from {len(list(self.logs_dir.glob('*.log')))} log files")
    
    def create_dataframe(self) -> pd.DataFrame:
        """Convert results to pandas DataFrame for analysis"""
        data = []
        for result in self.results:
            data.append({
                'solver': result.solver,
                'precision': result.precision,
                'data_mode': result.data_mode,
                'problems': result.problems,
                'dlt_min_points': result.dlt_min_points,
                'noise_level': result.noise_level,
                'total_problems': result.total_problems,
                'successful_solves': result.successful_solves,
                'failed_solves': result.failed_solves,
                'success_rate': result.success_rate,
                
                # Successful-only statistics
                'success_rotation_error_mean': result.success_rotation_error_mean,
                'success_rotation_error_std': result.success_rotation_error_std,
                'success_translation_error_mean': result.success_translation_error_mean,
                'success_translation_error_std': result.success_translation_error_std,
                'success_reprojection_error_mean': result.success_reprojection_error_mean,
                'success_reprojection_error_std': result.success_reprojection_error_std,
                
                # All-solutions statistics  
                'all_rotation_error_mean': result.all_rotation_error_mean,
                'all_rotation_error_std': result.all_rotation_error_std,
                'all_translation_error_mean': result.all_translation_error_mean,
                'all_translation_error_std': result.all_translation_error_std,
                'all_reprojection_error_mean': result.all_reprojection_error_mean,
                'all_reprojection_error_std': result.all_reprojection_error_std,
                
                # Solution statistics
                'avg_solutions_per_problem': result.avg_solutions_per_problem,
                'problems_with_multiple_solutions': result.problems_with_multiple_solutions,
                'multiple_solutions_percentage': result.multiple_solutions_percentage,
            })
        
        return pd.DataFrame(data)
    
    def plot_success_rates(self, df: pd.DataFrame):
        """Plot success rates across different configurations"""
        plt.figure(figsize=(12, 8))
        
        # Success rate vs noise level
        plt.subplot(2, 2, 1)
        for solver in df['solver'].unique():
            solver_data = df[df['solver'] == solver]
            plt.plot(solver_data['noise_level'], solver_data['success_rate'], 
                    marker='o', label=solver.upper(), linewidth=2)
        plt.xlabel('Noise Level')
        plt.ylabel('Success Rate (%)')
        plt.title('Success Rate vs Noise Level')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Success rate by solver type
        plt.subplot(2, 2, 2)
        success_by_solver = df.groupby('solver')['success_rate'].agg(['mean', 'std']).reset_index()
        plt.bar(success_by_solver['solver'], success_by_solver['mean'], 
                yerr=success_by_solver['std'], capsize=5, alpha=0.7)
        plt.xlabel('Solver')
        plt.ylabel('Average Success Rate (%)')
        plt.title('Average Success Rate by Solver')
        plt.xticks(rotation=45)
        
        # Success rate by precision type
        plt.subplot(2, 2, 3)
        precision_data = df.groupby(['solver', 'precision', 'noise_level'])['success_rate'].mean().reset_index()
        for solver in precision_data['solver'].unique():
            solver_data = precision_data[precision_data['solver'] == solver]
            for precision in solver_data['precision'].unique():
                subset = solver_data[solver_data['precision'] == precision]
                plt.plot(subset['noise_level'], subset['success_rate'], 
                        marker='o', label=f"{solver.upper()}-{precision}")
        plt.xlabel('Noise Level')
        plt.ylabel('Success Rate (%)')
        plt.title('Precision Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Success rate by data mode
        plt.subplot(2, 2, 4)
        mode_data = df.groupby(['solver', 'data_mode', 'noise_level'])['success_rate'].mean().reset_index()
        for solver in mode_data['solver'].unique():
            solver_data = mode_data[mode_data['solver'] == solver]
            for mode in solver_data['data_mode'].unique():
                subset = solver_data[solver_data['data_mode'] == mode]
                plt.plot(subset['noise_level'], subset['success_rate'], 
                        marker='o', label=f"{solver.upper()}-{mode}")
        plt.xlabel('Noise Level')
        plt.ylabel('Success Rate (%)')
        plt.title('Data Mode Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'success_rates_vs_noise.png', dpi=300, bbox_inches='tight')
        plt.close()

    def plot_error_analysis(self, df: pd.DataFrame):
        """Generate error analysis plots grouped by solver, then noise levels with float/double pairs"""
        
        # Create clean 2x1 subplot (stacked vertically) for better space
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(20, 12))
        
        # Filter to specific noise levels (simplified set, excluding perfect zero noise)
        target_noise_levels = [0.01, 0.02, 0.04]
        df_filtered = df[df['noise_level'].isin(target_noise_levels)].copy()
        
        if df_filtered.empty:
            print("Warning: No data found for target noise levels")
            return
        
        # Define colors for precision types and noise levels
        precision_colors = {'float': '#87CEEB', 'double': '#4682B4'}  # Light blue, dark blue
        
        # Create colormap for noise levels (heat map effect)
        noise_cmap = cm.get_cmap('YlOrRd')  # Yellow to Orange to Red
        noise_norm = colors.Normalize(vmin=min(target_noise_levels), vmax=max(target_noise_levels))
        
        def get_noise_color(noise_level, precision):
            base_color = noise_cmap(noise_norm(noise_level))
            # Adjust brightness based on precision (double = darker, float = lighter)
            if precision == 'double':
                return tuple(c * 0.8 for c in base_color[:3]) + (0.9,)  # Darker
            else:
                return tuple(min(1.0, c * 1.2) for c in base_color[:3]) + (0.7,)  # Lighter
        
        # Define solver groups - P3P, UP2P, and multiple DLT point counts
        solver_groups = []
        
        # Add P3P and UP2P
        for solver in ['p3p', 'up2p']:
            if solver in df_filtered['solver'].values:
                solver_groups.append((solver, None, solver.upper()))
        
        # Add DLT groups based on actual point counts in data (limit to ≤64 points)
        dlt_data = df_filtered[df_filtered['solver'] == 'dlt']
        dlt_point_counts = sorted([int(x) for x in dlt_data['dlt_min_points'].unique() 
                                 if x is not None and x <= 64])
        
        for point_count in dlt_point_counts:
            solver_groups.append(('dlt', point_count, f'DLT-{point_count}pt'))
        
        # If no DLT point counts found, add generic DLT
        if not dlt_point_counts and 'dlt' in df_filtered['solver'].values:
            solver_groups.append(('dlt', None, 'DLT'))
        
        # Calculate positions
        n_noise_levels = len(target_noise_levels)
        n_precisions = 2  # float, double
        noise_group_width = n_precisions * 0.35  # Width for float+double pair
        noise_spacing = 0.1  # Space between noise levels
        solver_spacing = 1.5  # Space between solver groups
        
        x_positions = []
        x_labels = []
        
        # Plot rotation errors (top subplot)
        ax1.set_title('Rotation Error Analysis by Solver and Noise Level', fontsize=16, fontweight='bold', pad=20)
        
        current_x = 0
        
        for solver_key, point_count, solver_label in solver_groups:
            # Get data for this solver (and specific point count for DLT)
            if solver_key == 'dlt' and point_count is not None:
                solver_data = df_filtered[(df_filtered['solver'] == solver_key) & 
                                        (df_filtered['dlt_min_points'] == point_count)]
            else:
                solver_data = df_filtered[df_filtered['solver'] == solver_key]
            
            if solver_data.empty:
                continue
            
            solver_x_positions = []
            
            # For each noise level, create float/double pair
            for noise_idx, noise_level in enumerate(target_noise_levels):
                noise_data = solver_data[solver_data['noise_level'] == noise_level]
                
                # Position for this noise level group
                noise_x = current_x + noise_idx * (noise_group_width + noise_spacing)
                
                # Plot float and double bars for this noise level
                for prec_idx, precision in enumerate(['float', 'double']):
                    prec_data = noise_data[noise_data['precision'] == precision]
                    
                    if not prec_data.empty:
                        # Average across data modes (traditional/realistic)
                        mean_rot = prec_data['all_rotation_error_mean'].mean()
                        
                        # Handle zero values for log scale plotting
                        if mean_rot <= 0:
                            mean_rot = 1e-6  # Small epsilon for log scale
                        
                        bar_x = noise_x + prec_idx * 0.35
                        ax1.bar(bar_x, mean_rot, 0.3, 
                               color=get_noise_color(noise_level, precision), 
                               alpha=0.8, 
                               edgecolor='black', 
                               linewidth=0.5)
                
                # Store center position for noise level label
                center_x = noise_x + (n_precisions - 1) * 0.35 / 2
                solver_x_positions.append(center_x)
            
            # Store solver group center for main x-axis label
            solver_center = current_x + (n_noise_levels - 1) * (noise_group_width + noise_spacing) / 2
            x_positions.append(solver_center)
            x_labels.append(solver_label)
            
            # Move to next solver group
            current_x += n_noise_levels * (noise_group_width + noise_spacing) + solver_spacing
        
        ax1.set_xlabel('Solver Type and Noise Level', fontsize=14)
        ax1.set_ylabel('Rotation Error (degrees)', fontsize=14)
        ax1.set_yscale('log')
        
        # Custom formatter for small numbers with appropriate precision
        from matplotlib.ticker import FuncFormatter
        def custom_log_formatter(x, pos):
            if x >= 1:
                return f'{x:.1f}'
            elif x >= 0.1:
                return f'{x:.2f}'
            elif x >= 0.01:
                return f'{x:.3f}'
            elif x >= 0.001:
                return f'{x:.4f}'
            else:
                return f'{x:.1e}'
        
        ax1.yaxis.set_major_formatter(FuncFormatter(custom_log_formatter))
        ax1.tick_params(axis='y', which='minor', labelsize=9)
        
        ax1.set_xticks(x_positions)
        ax1.set_xticklabels(x_labels, fontsize=12)
        
        # Removed legend for cleaner look
        # ax1.legend(handles=legend_elements, bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=10)
        ax1.grid(True, alpha=0.3, axis='y')
        
        # Plot translation errors (bottom subplot) - same structure
        ax2.set_title('Translation Error Analysis by Solver and Noise Level', fontsize=16, fontweight='bold', pad=20)
        
        current_x = 0
        
        for solver_key, point_count, solver_label in solver_groups:
            # Get data for this solver (and specific point count for DLT)
            if solver_key == 'dlt' and point_count is not None:
                solver_data = df_filtered[(df_filtered['solver'] == solver_key) & 
                                        (df_filtered['dlt_min_points'] == point_count)]
            else:
                solver_data = df_filtered[df_filtered['solver'] == solver_key]
            
            if solver_data.empty:
                continue
            
            # For each noise level, create float/double pair
            for noise_idx, noise_level in enumerate(target_noise_levels):
                noise_data = solver_data[solver_data['noise_level'] == noise_level]
                
                # Position for this noise level group
                noise_x = current_x + noise_idx * (noise_group_width + noise_spacing)
                
                # Plot float and double bars for this noise level
                for prec_idx, precision in enumerate(['float', 'double']):
                    prec_data = noise_data[noise_data['precision'] == precision]
                    
                    if not prec_data.empty:
                        # Average across data modes (traditional/realistic)
                        mean_trans = prec_data['all_translation_error_mean'].mean()
                        
                        # Handle zero values for log scale plotting
                        if mean_trans <= 0:
                            mean_trans = 1e-6  # Small epsilon for log scale
                        
                        bar_x = noise_x + prec_idx * 0.35
                        ax2.bar(bar_x, mean_trans, 0.3,
                               color=get_noise_color(noise_level, precision), 
                               alpha=0.8, 
                               edgecolor='black', 
                               linewidth=0.5)
            
            # Move to next solver group
            current_x += n_noise_levels * (noise_group_width + noise_spacing) + solver_spacing
        
        ax2.set_xlabel('Solver Type and Noise Level', fontsize=14)
        ax2.set_ylabel('Translation Error (meters)', fontsize=14)
        ax2.set_yscale('log')
        
        # Custom formatter for small numbers with appropriate precision
        ax2.yaxis.set_major_formatter(FuncFormatter(custom_log_formatter))
        ax2.tick_params(axis='y', which='minor', labelsize=9)
        
        ax2.set_xticks(x_positions)
        ax2.set_xticklabels(x_labels, fontsize=12)
        ax2.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        plt.savefig(self.results_dir / 'error_analysis_by_noise_and_solver.png', 
                   dpi=300, bbox_inches='tight')
        plt.show()
        
        print(f"Error analysis plot saved to {self.results_dir / 'error_analysis_by_noise_and_solver.png'}")
        
        # Print summary
        print(f"\nPlotted data summary:")
        print(f"  Noise levels: {target_noise_levels}")
        print(f"  Solver groups: {[label for _, _, label in solver_groups]}")
        print(f"  Precision types: float (light blue), double (dark blue)")

    def plot_success_vs_all_comparison(self, df: pd.DataFrame):
        """Plot comparison between successful-only and all-solutions statistics"""
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        colors = {'p3p': '#1f77b4', 'up2p': '#ff7f0e', 'dlt': '#2ca02c'}
        
        # Plot 1: Rotation Error Comparison
        ax1.set_title('Rotation Error: Successful-Only vs All-Solutions', fontweight='bold')
        for solver in df['solver'].unique():
            solver_data = df[df['solver'] == solver]
            ax1.plot(solver_data['noise_level'], solver_data['success_rotation_error_mean'], 
                    color=colors.get(solver, '#666666'), marker='o', linestyle='-',
                    label=f"{solver.upper()}-successful", linewidth=2)
            ax1.plot(solver_data['noise_level'], solver_data['all_rotation_error_mean'], 
                    color=colors.get(solver, '#666666'), marker='s', linestyle='--',
                    label=f"{solver.upper()}-all", linewidth=2, alpha=0.7)
        ax1.set_xlabel('Noise Level')
        ax1.set_ylabel('Rotation Error (degrees)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.set_yscale('log')
        
        # Plot 2: Translation Error Comparison
        ax2.set_title('Translation Error: Successful-Only vs All-Solutions', fontweight='bold')
        for solver in df['solver'].unique():
            solver_data = df[df['solver'] == solver]
            ax2.plot(solver_data['noise_level'], solver_data['success_translation_error_mean'], 
                    color=colors.get(solver, '#666666'), marker='o', linestyle='-',
                    label=f"{solver.upper()}-successful", linewidth=2)
            ax2.plot(solver_data['noise_level'], solver_data['all_translation_error_mean'], 
                    color=colors.get(solver, '#666666'), marker='s', linestyle='--',
                    label=f"{solver.upper()}-all", linewidth=2, alpha=0.7)
        ax2.set_xlabel('Noise Level')
        ax2.set_ylabel('Translation Error (meters)')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        ax2.set_yscale('log')
        
        # Plot 3: Success Rate vs All-Solutions Quality
        ax3.set_title('Success Rate vs All-Solutions Error Quality', fontweight='bold')
        for solver in df['solver'].unique():
            solver_data = df[df['solver'] == solver]
            # Plot success rate vs combined error (rotation + translation)
            combined_error = solver_data['all_rotation_error_mean'] + solver_data['all_translation_error_mean']
            scatter = ax3.scatter(solver_data['success_rate'], combined_error, 
                                 c=solver_data['noise_level'], cmap='viridis',
                                 label=solver.upper(), alpha=0.7, s=60)
        ax3.set_xlabel('Success Rate (%)')
        ax3.set_ylabel('Combined Error (degrees + meters)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        plt.colorbar(scatter, ax=ax3, label='Noise Level')
        
        # Plot 4: Reprojection Error Comparison
        ax4.set_title('Reprojection Error: Successful-Only vs All-Solutions', fontweight='bold')
        for solver in df['solver'].unique():
            solver_data = df[df['solver'] == solver]
            ax4.plot(solver_data['noise_level'], solver_data['success_reprojection_error_mean'], 
                    color=colors.get(solver, '#666666'), marker='o', linestyle='-',
                    label=f"{solver.upper()}-successful", linewidth=2)
            ax4.plot(solver_data['noise_level'], solver_data['all_reprojection_error_mean'], 
                    color=colors.get(solver, '#666666'), marker='s', linestyle='--',
                    label=f"{solver.upper()}-all", linewidth=2, alpha=0.7)
        ax4.set_xlabel('Noise Level')
        ax4.set_ylabel('Reprojection Error (pixels)')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        ax4.set_yscale('log')
        
        plt.tight_layout()
        plt.savefig(self.results_dir / 'success_vs_all_comparison.png', dpi=300, bbox_inches='tight')
        plt.show()

    def plot_dlt_analysis(self, df: pd.DataFrame):
        """Special analysis for DLT configurations"""
        dlt_data = df[df['solver'] == 'dlt'].copy()
        
        if dlt_data.empty:
            print("No DLT data found for analysis")
            return
            
        plt.figure(figsize=(15, 10))
        
        # Success rate by DLT minimum points
        plt.subplot(2, 3, 1)
        for noise in sorted(dlt_data['noise_level'].unique()):
            subset = dlt_data[dlt_data['noise_level'] == noise]
            plt.plot(subset['dlt_min_points'], subset['success_rate'], 
                    marker='o', label=f'noise={noise}')
        plt.xlabel('DLT Minimum Points')
        plt.ylabel('Success Rate (%)')
        plt.title('DLT Success Rate vs Minimum Points')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Rotation error by DLT minimum points (all-solutions)
        plt.subplot(2, 3, 2)
        for noise in sorted(dlt_data['noise_level'].unique()):
            subset = dlt_data[dlt_data['noise_level'] == noise]
            plt.errorbar(subset['dlt_min_points'], subset['all_rotation_error_mean'], 
                        yerr=subset['all_rotation_error_std'], label=f'noise={noise}', 
                        marker='o', capsize=5)
        plt.xlabel('DLT Minimum Points')
        plt.ylabel('Rotation Error (degrees)')
        plt.title('DLT Rotation Error vs Minimum Points (All Solutions)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Translation error by DLT minimum points (all-solutions)
        plt.subplot(2, 3, 3)
        for noise in sorted(dlt_data['noise_level'].unique()):
            subset = dlt_data[dlt_data['noise_level'] == noise]
            plt.errorbar(subset['dlt_min_points'], subset['all_translation_error_mean'], 
                        yerr=subset['all_translation_error_std'], label=f'noise={noise}', 
                        marker='o', capsize=5)
        plt.xlabel('DLT Minimum Points')
        plt.ylabel('Translation Error (meters)')
        plt.title('DLT Translation Error vs Minimum Points (All Solutions)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Heatmap of success rates
        plt.subplot(2, 3, 4)
        pivot_data = dlt_data.pivot_table(
            values='success_rate', 
            index='dlt_min_points', 
            columns='noise_level', 
            aggfunc='mean'
        )
        if not pivot_data.empty and pivot_data.size > 0:
            sns.heatmap(pivot_data, annot=True, cmap='RdYlGn', fmt='.1f')
            plt.title('DLT Success Rate Heatmap')
            plt.xlabel('Noise Level')
            plt.ylabel('DLT Minimum Points')
        else:
            plt.text(0.5, 0.5, 'No data available', ha='center', va='center', transform=plt.gca().transAxes)
            plt.title('DLT Success Rate Heatmap (No Data)')
        
        # Heatmap of rotation errors
        plt.subplot(2, 3, 5)
        pivot_rot_data = dlt_data.pivot_table(
            values='all_rotation_error_mean', 
            index='dlt_min_points', 
            columns='noise_level', 
            aggfunc='mean'
        )
        if not pivot_rot_data.empty and pivot_rot_data.size > 0:
            sns.heatmap(pivot_rot_data, annot=True, cmap='YlOrRd', fmt='.2f')
            plt.title('DLT Rotation Error Heatmap (degrees)')
            plt.xlabel('Noise Level')
            plt.ylabel('DLT Minimum Points')
        else:
            plt.text(0.5, 0.5, 'No data available', ha='center', va='center', transform=plt.gca().transAxes)
            plt.title('DLT Rotation Error Heatmap (No Data)')
        
        # Heatmap of translation errors
        plt.subplot(2, 3, 6)
        pivot_trans_data = dlt_data.pivot_table(
            values='all_translation_error_mean', 
            index='dlt_min_points', 
            columns='noise_level', 
            aggfunc='mean'
        )
        if not pivot_trans_data.empty and pivot_trans_data.size > 0:
            sns.heatmap(pivot_trans_data, annot=True, cmap='YlOrRd', fmt='.3f')
            plt.title('DLT Translation Error Heatmap (meters)')
            plt.xlabel('Noise Level')
            plt.ylabel('DLT Minimum Points')
        else:
            plt.text(0.5, 0.5, 'No data available', ha='center', va='center', transform=plt.gca().transAxes)
            plt.title('DLT Translation Error Heatmap (No Data)')
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'dlt_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()

    def generate_summary_report(self, df: pd.DataFrame):
        """Generate a comprehensive summary report with dual statistics"""
        report_path = self.results_dir / 'analysis_report.txt'
        
        with open(report_path, 'w') as f:
            f.write("=== Enhanced Absolute Pose Benchmark Analysis Report ===\n")
            f.write(f"Generated: {pd.Timestamp.now()}\n")
            f.write("Updated for dual statistics format (successful-only vs all-solutions)\n\n")
            
            f.write(f"Total benchmark results analyzed: {len(df)}\n")
            f.write(f"Solvers tested: {', '.join(df['solver'].unique())}\n")
            f.write(f"Precision types: {', '.join(df['precision'].unique())}\n")
            f.write(f"Data modes: {', '.join(df['data_mode'].unique())}\n")
            f.write(f"Noise levels tested: {', '.join(map(str, sorted(df['noise_level'].unique())))}\n\n")
            
            # Overall statistics - successful-only
            f.write("=== Overall Performance (Successful Cases Only) ===\n")
            success_stats = df.groupby('solver').agg({
                'success_rate': ['mean', 'std', 'min', 'max'],
                'success_rotation_error_mean': ['mean', 'std'],
                'success_translation_error_mean': ['mean', 'std']
            }).round(4)
            f.write(str(success_stats))
            f.write("\n\n")
            
            # Overall statistics - all solutions
            f.write("=== Overall Performance (All Solutions) ===\n")
            all_stats = df.groupby('solver').agg({
                'success_rate': ['mean', 'std', 'min', 'max'],
                'all_rotation_error_mean': ['mean', 'std'],
                'all_translation_error_mean': ['mean', 'std']
            }).round(4)
            f.write(str(all_stats))
            f.write("\n\n")
            
            # Best performing configurations
            f.write("=== Best Performing Configurations ===\n")
            best_success = df.loc[df['success_rate'].idxmax()]
            f.write(f"Highest success rate: {best_success['success_rate']:.1f}% ")
            f.write(f"({best_success['solver'].upper()}, {best_success['precision']}, {best_success['data_mode']}, noise={best_success['noise_level']})\n")
            
            # Best rotation error (all-solutions)
            lowest_rot_error = df.loc[df['all_rotation_error_mean'].idxmin()]
            f.write(f"Lowest rotation error (all): {lowest_rot_error['all_rotation_error_mean']:.4f}° ")
            f.write(f"({lowest_rot_error['solver'].upper()}, {lowest_rot_error['precision']}, {lowest_rot_error['data_mode']}, noise={lowest_rot_error['noise_level']})\n")
            
            # Best translation error (all-solutions)
            lowest_trans_error = df.loc[df['all_translation_error_mean'].idxmin()]
            f.write(f"Lowest translation error (all): {lowest_trans_error['all_translation_error_mean']:.4f} m ")
            f.write(f"({lowest_trans_error['solver'].upper()}, {lowest_trans_error['precision']}, {lowest_trans_error['data_mode']}, noise={lowest_trans_error['noise_level']})\n")
            
            # Comparison between successful-only and all-solutions
            f.write(f"\n=== Successful-Only vs All-Solutions Comparison ===\n")
            for solver in df['solver'].unique():
                solver_data = df[df['solver'] == solver]
                avg_success_rot = solver_data['success_rotation_error_mean'].mean()
                avg_all_rot = solver_data['all_rotation_error_mean'].mean()
                avg_success_trans = solver_data['success_translation_error_mean'].mean()
                avg_all_trans = solver_data['all_translation_error_mean'].mean()
                
                f.write(f"{solver.upper()}:\n")
                f.write(f"  Rotation error - Successful: {avg_success_rot:.4f}°, All: {avg_all_rot:.4f}° (ratio: {avg_all_rot/avg_success_rot if avg_success_rot > 0 else float('inf'):.2f})\n")
                f.write(f"  Translation error - Successful: {avg_success_trans:.4f}m, All: {avg_all_trans:.4f}m (ratio: {avg_all_trans/avg_success_trans if avg_success_trans > 0 else float('inf'):.2f})\n")
            
        print(f"Analysis report saved to: {report_path}")
        
    def run_analysis(self):
        """Run complete analysis pipeline"""
        print("Loading benchmark results...")
        self.load_all_results()
        
        if not self.results:
            print("No results found to analyze!")
            return
            
        print("Creating DataFrame...")
        df = self.create_dataframe()
        
        # Filter for traditional data only
        print(f"Original data points: {len(df)}")
        print(f"Available data modes: {df['data_mode'].unique()}")
        df = df[df['data_mode'] == 'traditional']
        print(f"Traditional data points: {len(df)}")
        
        if df.empty:
            print("No traditional data found!")
            return
        
        print("Generating plots...")
        self.plot_success_rates(df)
        self.plot_error_analysis(df)
        self.plot_success_vs_all_comparison(df)
        self.plot_dlt_analysis(df)
        
        print("Generating summary report...")
        self.generate_summary_report(df)
        
        print(f"Analysis complete! Results saved to: {self.plots_dir}")

def main():
    parser = argparse.ArgumentParser(description='Analyze absolute pose benchmark results')
    parser.add_argument('results_dir', help='Directory containing benchmark results')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.results_dir):
        print(f"Results directory does not exist: {args.results_dir}")
        sys.exit(1)
    
    analyzer = BenchmarkAnalyzer(args.results_dir)
    analyzer.run_analysis()

if __name__ == "__main__":
    main() 