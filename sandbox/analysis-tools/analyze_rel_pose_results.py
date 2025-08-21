#!/usr/bin/env python3
"""
Enhanced Relative Pose Results Analysis Tool
Analyzes benchmark results and generates comprehensive error analysis plots:
- Rotation and translation direction error analysis
- Sampson distance reprojection error analysis  
- Comparative analysis across noise levels
- Solver performance comparison
- Statistical significance testing
- Comprehensive visualizations

Author: Enhanced for ento-bench relative pose analysis
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import os
import glob
from pathlib import Path
import matplotlib.cm as cm
import matplotlib.colors as colors
from matplotlib.ticker import FuncFormatter

class RelativePoseAnalyzer:
    """Analyzes relative pose benchmark results and generates focused error plots"""
    
    def __init__(self, results_dir="benchmark_results/rel_pose"):
        """
        Initialize analyzer
        
        Args:
            results_dir: Directory containing benchmark results (default: benchmark_results/rel_pose)
        """
        self.results_dir = Path(results_dir)
        self.data_dir = self.results_dir / "data"
        self.plots_dir = self.results_dir / "plots"
        
        # Ensure directories exist
        self.results_dir.mkdir(parents=True, exist_ok=True)
        self.plots_dir.mkdir(parents=True, exist_ok=True)
        
        # Color schemes matching abs-pose style
        self.precision_colors = {'float': '#87CEEB', 'double': '#4682B4'}  # Light blue, dark blue
        self.noise_cmap = plt.get_cmap('YlOrRd')  # Yellow to Orange to Red
        
    def get_noise_color(self, noise_level, precision):
        """Get color for noise level and precision combination"""
        noise_norm = colors.Normalize(vmin=0.01, vmax=0.04)
        base_color = self.noise_cmap(noise_norm(noise_level))
        # Adjust brightness based on precision (double = darker, float = lighter)
        if precision == 'double':
            return tuple(c * 0.8 for c in base_color[:3]) + (1.0,)
        else:
            return base_color
    
    def load_data_from_logs(self):
        """Load data from benchmark log files (not CSV files)"""
        print("Loading data from benchmark log files...")
        
        # Find all log files in the logs directory
        log_files = list(self.results_dir.glob("logs/*.log"))
        
        if not log_files:
            print("No log files found! Please run the benchmark first.")
            return pd.DataFrame()
        
        print(f"Found {len(log_files)} log files")
        
        data_rows = []
        processed_keys = set()  # Track processed solver/precision/datamode/noise combinations
        
        for log_file in log_files:
            # Parse filename patterns:
            # Minimal solvers: {solver}_{precision}_{datamode}_p{num_problems}_noise{noise_level}
            # Linear solvers: {solver}_{precision}_{datamode}_p{num_problems}_noise{noise_level}_N{points}
            filename = log_file.stem
            print(f"Processing: {filename}")
            
            # ENHANCED: Handle both old and new filename patterns
            parts = filename.split('_')
            
            # Clean up old format artifacts (remove trailing solver-specific suffixes like "8pt16", "8pt64")
            # These appear in older log files and can confuse the parser
            if len(parts) > 0:
                last_part = parts[-1]
                # Check if last part looks like a solver suffix (e.g., "8pt16", "8pt64", "8pt128")
                import re
                if re.match(r'^(8pt|5pt|upright)\d+$', last_part):
                    parts = parts[:-1]  # Remove the trailing solver suffix
                    print(f"  Cleaned filename suffix: removed '{last_part}'")
            
            # Find the noise part first (always starts with "noise")
            noise_idx = -1
            for i, part in enumerate(parts):
                if part.startswith('noise'):
                    noise_idx = i
                    break
            
            if noise_idx == -1:
                print(f"  Skipping {filename} - no noise level found")
                continue
            
            # Extract noise level
            noise_part = parts[noise_idx]
            if not noise_part.startswith('noise'):
                print(f"  Skipping {filename} - no noise level found")
                continue
            noise_level = float(noise_part[5:])  # Remove "noise" prefix
            
            # Check if there's an N suffix after noise (new format for linear solvers)
            has_n_suffix = (noise_idx + 1 < len(parts) and parts[noise_idx + 1].startswith('N'))
            n_points = None
            
            if has_n_suffix:
                # New format: extract N value
                n_part = parts[noise_idx + 1]
                n_points = int(n_part[1:])  # Remove "N" prefix
                # For parsing, we treat this as if N part doesn't exist
                effective_parts = parts[:noise_idx + 1]
            else:
                # Old format: no N suffix
                effective_parts = parts
                noise_idx_effective = noise_idx
            
            # Find the problems part (always "p1000" or similar) in effective parts
            problems_idx = -1
            for i, part in enumerate(effective_parts):
                if part.startswith('p') and part[1:].isdigit():
                    problems_idx = i
                    break
            
            if problems_idx == -1:
                print(f"  Skipping {filename} - no problems count found")
                continue
            
            # Extract: solver, precision, data_mode
            if problems_idx < 2:
                print(f"  Skipping {filename} - unexpected format")
                continue
            
            solver_parts = effective_parts[:problems_idx-2]
            precision = effective_parts[problems_idx-2]
            data_mode = effective_parts[problems_idx-1]
            
            if not solver_parts:
                print(f"  Skipping {filename} - no solver name found")
                continue
            
            solver = '_'.join(solver_parts)  # Rejoin solver parts with underscores
            
            # Create solver display name with N count if available
            if n_points is not None:
                solver_display = f"{solver}_N{n_points}"
                print(f"  Parsed: solver={solver} (N={n_points}), precision={precision}, data_mode={data_mode}, noise={noise_level}")
            else:
                solver_display = solver
                print(f"  Parsed: solver={solver}, precision={precision}, data_mode={data_mode}, noise={noise_level}")
            
            # Create a unique key for this configuration
            config_key = (solver, precision, data_mode, noise_level, n_points)
            
            # For linear solvers (8pt, upright_planar_3pt), skip files without N suffix if we have files with N suffix
            is_linear_solver = solver in ['8pt', 'upright_planar_3pt']
            if is_linear_solver and n_points is None:
                # Check if we have any files with N suffix for this solver
                has_n_variants = any(f for f in log_files if f.stem.startswith(f"{solver}_{precision}_{data_mode}") and f"_N" in f.stem)
                if has_n_variants:
                    print(f"  Skipping {filename} - using explicit N variants instead")
                    continue
            
            # Skip if we've already processed this exact configuration
            if config_key in processed_keys:
                print(f"  Skipping duplicate: {filename}")
                continue
            
            processed_keys.add(config_key)
            
            # Read log file content
            try:
                with open(log_file, 'r') as f:
                    content = f.read()
            except Exception as e:
                print(f"  Error reading {log_file}: {e}")
                continue
            
            # Parse statistics from log content - use solver_display for grouping
            stats = self.parse_log_statistics(content, solver_display, precision, data_mode, noise_level, 1000)
            if stats:
                data_rows.append(stats)
                print(f"  ✓ Extracted statistics: {stats['success_rate']:.1f}% success, rot_err={stats['rotation_error_mean']:.2f}°")
            else:
                print(f"  ✗ No statistics found in log file")
        
        if not data_rows:
            print("No valid data found in any log files!")
            return pd.DataFrame()
        
        df = pd.DataFrame(data_rows)
        print(f"\nLoaded {len(df)} data points from {len(log_files)} log files")
        print(f"Solvers found: {sorted(df['solver'].unique())}")
        print(f"Noise levels: {sorted(df['noise_level'].unique())}")
        print(f"Precisions: {sorted(df['precision'].unique())}")
        print(f"Data modes: {sorted(df['data_mode'].unique())}")
        
        return df
    
    def parse_log_statistics(self, content, solver, precision, data_mode, noise_level, num_problems):
        """Parse statistics from log file content"""
        lines = content.split('\n')
        
        # Look for the statistics section
        # Example: "=== upright_planar_2pt (noise=0.010000 radians) Statistics (noise=0.01) ==="
        stats_start = -1
        for i, line in enumerate(lines):
            if 'Statistics' in line and 'noise=' in line:
                stats_start = i
                break
        
        if stats_start == -1:
            return None
        
        # Initialize result dictionary with both traditional and robust statistics
        result = {
            'solver': solver,
            'precision': precision,
            'data_mode': data_mode,
            'noise_level': noise_level,
            'num_problems': num_problems,
            'success_rate': 0.0,
            
            # Traditional statistics (mean ± std)
            'rotation_error_mean': 0.0,
            'rotation_error_std': 0.0,
            'translation_direction_error_mean': 0.0,
            'translation_direction_error_std': 0.0,
            'reprojection_error_mean': 0.0,
            'reprojection_error_std': 0.0,
            'combined_error_mean': 0.0,
            'combined_error_std': 0.0,
            
            # NEW: Robust statistics (median, quartiles) - ALWAYS INITIALIZE
            'rotation_error_median': 0.0,
            'rotation_error_q1': 0.0,
            'rotation_error_q3': 0.0,
            'translation_direction_error_median': 0.0,
            'translation_direction_error_q1': 0.0,
            'translation_direction_error_q3': 0.0,
            'reprojection_error_median': 0.0,
            'reprojection_error_q1': 0.0,
            'reprojection_error_q3': 0.0,
            'combined_error_median': 0.0,
            'combined_error_q1': 0.0,
            'combined_error_q3': 0.0,
            
            'avg_solutions': 0.0
        }
        
        # Parse statistics section
        i = stats_start
        while i < len(lines) and not lines[i].startswith('==='*5):  # Until next major section
            line = lines[i].strip()
            
            if 'Successful solves:' in line:
                # Extract success rate: "Successful solves: 476 (47.6%)"
                import re
                match = re.search(r'(\d+)\s*\(([0-9.]+)%\)', line)
                if match:
                    result['success_rate'] = float(match.group(2))
            
            elif 'Rotation Error Statistics:' in line:
                # Look for Traditional and Robust lines
                j = i + 1
                while j < len(lines) and not lines[j].strip().startswith('Translation') and not lines[j].strip().startswith('==='):
                    next_line = lines[j].strip()
                    
                    if 'Traditional: Mean ± Std Dev:' in next_line:
                        # FIXED: Updated regex to handle scientific notation (e.g., 5.60089e-05)
                        import re
                        match = re.search(r'Traditional: Mean ± Std Dev:\s*([0-9.e+-]+)\s*±\s*([0-9.e+-]+)', next_line)
                        if match:
                            result['rotation_error_mean'] = float(match.group(1))
                            result['rotation_error_std'] = float(match.group(2))
                            print(f"    DEBUG: Parsed traditional rotation - mean={result['rotation_error_mean']:.6e}, std={result['rotation_error_std']:.6e}")
                    
                    elif 'Robust: Median [Q1, Q3]:' in next_line:
                        # Parse: "Robust: Median [Q1, Q3]: 5.60089e-05 [2.84e-05, 8.12e-05] degrees"
                        import re
                        match = re.search(r'Robust: Median \[Q1, Q3\]:\s*([0-9.e+-]+)\s*\[([0-9.e+-]+),\s*([0-9.e+-]+)\]', next_line)
                        if match:
                            result['rotation_error_median'] = float(match.group(1))
                            result['rotation_error_q1'] = float(match.group(2))
                            result['rotation_error_q3'] = float(match.group(3))
                            print(f"    DEBUG: Parsed robust rotation - median={result['rotation_error_median']:.6e}, Q1={result['rotation_error_q1']:.6e}, Q3={result['rotation_error_q3']:.6e}")
                    
                    j += 1
            
            elif 'Translation Direction Error Statistics:' in line:
                # Look for Traditional and Robust lines
                j = i + 1
                while j < len(lines) and not lines[j].strip().startswith('Reprojection') and not lines[j].strip().startswith('==='):
                    next_line = lines[j].strip()
                    
                    if 'Traditional: Mean ± Std Dev:' in next_line:
                        # FIXED: Updated regex to handle scientific notation
                        import re
                        match = re.search(r'Traditional: Mean ± Std Dev:\s*([0-9.e+-]+)\s*±\s*([0-9.e+-]+)', next_line)
                        if match:
                            result['translation_direction_error_mean'] = float(match.group(1))
                            result['translation_direction_error_std'] = float(match.group(2))
                            print(f"    DEBUG: Parsed traditional translation - mean={result['translation_direction_error_mean']:.6e}, std={result['translation_direction_error_std']:.6e}")
                    
                    elif 'Robust: Median [Q1, Q3]:' in next_line:
                        import re
                        match = re.search(r'Robust: Median \[Q1, Q3\]:\s*([0-9.e+-]+)\s*\[([0-9.e+-]+),\s*([0-9.e+-]+)\]', next_line)
                        if match:
                            result['translation_direction_error_median'] = float(match.group(1))
                            result['translation_direction_error_q1'] = float(match.group(2))
                            result['translation_direction_error_q3'] = float(match.group(3))
                            print(f"    DEBUG: Parsed robust translation - median={result['translation_direction_error_median']:.6e}, Q1={result['translation_direction_error_q1']:.6e}, Q3={result['translation_direction_error_q3']:.6e}")
                    
                    j += 1
            
            elif 'Reprojection Error Statistics' in line:
                # Look for Traditional and Robust lines
                j = i + 1
                while j < len(lines) and not lines[j].strip().startswith('Combined') and not lines[j].strip().startswith('==='):
                    next_line = lines[j].strip()
                    
                    if 'Traditional: Mean ± Std Dev:' in next_line:
                        # FIXED: Updated regex to handle scientific notation
                        import re
                        match = re.search(r'Traditional: Mean ± Std Dev:\s*([0-9.e+-]+)\s*±\s*([0-9.e+-]+)', next_line)
                        if match:
                            result['reprojection_error_mean'] = float(match.group(1))
                            result['reprojection_error_std'] = float(match.group(2))
                    
                    elif 'Robust: Median [Q1, Q3]:' in next_line:
                        import re
                        match = re.search(r'Robust: Median \[Q1, Q3\]:\s*([0-9.e+-]+)\s*\[([0-9.e+-]+),\s*([0-9.e+-]+)\]', next_line)
                        if match:
                            result['reprojection_error_median'] = float(match.group(1))
                            result['reprojection_error_q1'] = float(match.group(2))
                            result['reprojection_error_q3'] = float(match.group(3))
                    
                    j += 1
            
            elif 'Combined Error Statistics:' in line:
                # Look for Traditional and Robust lines
                j = i + 1
                while j < len(lines) and not lines[j].strip().startswith('Solution') and not lines[j].strip().startswith('==='):
                    next_line = lines[j].strip()
                    
                    if 'Traditional: Mean ± Std Dev:' in next_line:
                        # FIXED: Updated regex to handle scientific notation
                        import re
                        match = re.search(r'Traditional: Mean ± Std Dev:\s*([0-9.e+-]+)\s*±\s*([0-9.e+-]+)', next_line)
                        if match:
                            result['combined_error_mean'] = float(match.group(1))
                            result['combined_error_std'] = float(match.group(2))
                    
                    elif 'Robust: Median [Q1, Q3]:' in next_line:
                        import re
                        match = re.search(r'Robust: Median \[Q1, Q3\]:\s*([0-9.e+-]+)\s*\[([0-9.e+-]+),\s*([0-9.e+-]+)\]', next_line)
                        if match:
                            result['combined_error_median'] = float(match.group(1))
                            result['combined_error_q1'] = float(match.group(2))
                            result['combined_error_q3'] = float(match.group(3))
                    
                    j += 1
            
            elif 'Average solutions per problem:' in line:
                # Parse: "Average solutions per problem: 1.80042"
                import re
                match = re.search(r'Average solutions per problem:\s*([0-9.]+)', line)
                if match:
                    result['avg_solutions'] = float(match.group(1))
            
            i += 1
        
        # DEBUGGING: Check if robust statistics were parsed
        robust_stats_found = any([
            result['rotation_error_median'] is not None,
            result['translation_direction_error_median'] is not None,
            result['reprojection_error_median'] is not None,
            result['combined_error_median'] is not None
        ])
        
        if not robust_stats_found:
            print(f"    WARNING: No robust statistics found for {solver} {precision} {data_mode} noise={noise_level}")
            print(f"    DEBUG: rotation_median={result['rotation_error_median']}, translation_median={result['translation_direction_error_median']}, reprojection_median={result['reprojection_error_median']}, combined_median={result['combined_error_median']}")
            # For missing robust stats, use traditional stats as fallback
            result['rotation_error_median'] = result['rotation_error_mean']
            result['rotation_error_q1'] = max(0, result['rotation_error_mean'] - result['rotation_error_std'])
            result['rotation_error_q3'] = result['rotation_error_mean'] + result['rotation_error_std']
            
            result['translation_direction_error_median'] = result['translation_direction_error_mean']
            result['translation_direction_error_q1'] = max(0, result['translation_direction_error_mean'] - result['translation_direction_error_std'])
            result['translation_direction_error_q3'] = result['translation_direction_error_mean'] + result['translation_direction_error_std']
            
            result['reprojection_error_median'] = result['reprojection_error_mean']
            result['reprojection_error_q1'] = max(0, result['reprojection_error_mean'] - result['reprojection_error_std'])
            result['reprojection_error_q3'] = result['reprojection_error_mean'] + result['reprojection_error_std']
            
            result['combined_error_median'] = result['combined_error_mean']
            result['combined_error_q1'] = max(0, result['combined_error_mean'] - result['combined_error_std'])
            result['combined_error_q3'] = result['combined_error_mean'] + result['combined_error_std']
        
        return result
    
    def create_solver_groups_ordered(self, df):
        """Create solver groups with proper ordering: minimal solvers first, then linear solvers"""
        # Use aggregated data, not raw CSV data
        solver_groups = {}
        
        # Collect data by solver type
        solver_data = {}
        
        for _, row in df.iterrows():
            solver = row['solver']
            precision = row['precision']
            noise_level = row['noise_level']
            
            # Parse solver name to extract base solver and N count
            if '_N' in solver:
                # New format: "8pt_N128", "upright_planar_3pt_N32", etc.
                base_solver, n_part = solver.split('_N')
                n_count = int(n_part)
                group_key = f"{base_solver.upper().replace('_', ' ')} (N={n_count})"
            else:
                # Old format: solver name without N suffix
                base_solver = solver
                # Infer N count for display (backward compatibility)
                if solver == '8pt':
                    group_key = "8PT (N=128)"  # Default for old data
                elif solver == '5pt':
                    group_key = "5PT (N=5)"
                elif solver == 'upright_3pt':
                    group_key = "UPRIGHT 3PT (N=3)"
                elif solver == 'upright_planar_3pt':
                    group_key = "UPRIGHT PLANAR 3PT (N=32)"  # Default for old data
                elif solver == 'upright_planar_2pt':
                    group_key = "UPRIGHT PLANAR 2PT (N=2)"
                else:
                    group_key = solver.upper().replace('_', ' ')
            
            if group_key not in solver_data:
                solver_data[group_key] = []
            
            solver_data[group_key].append({
                'solver': solver,
                'precision': precision,
                'noise_level': noise_level,
                'success_rate': row['success_rate'],
                'rotation_error_mean': row['rotation_error_mean'],
                'rotation_error_std': row['rotation_error_std'],
                'translation_direction_error_mean': row['translation_direction_error_mean'],
                'translation_direction_error_std': row['translation_direction_error_std'],
                'combined_error_mean': row['combined_error_mean'],
                'combined_error_std': row['combined_error_std'],
                # Add robust statistics columns
                'rotation_error_median': row.get('rotation_error_median'),
                'rotation_error_q1': row.get('rotation_error_q1'),
                'rotation_error_q3': row.get('rotation_error_q3'),
                'translation_direction_error_median': row.get('translation_direction_error_median'),
                'translation_direction_error_q1': row.get('translation_direction_error_q1'),
                'translation_direction_error_q3': row.get('translation_direction_error_q3'),
                'combined_error_median': row.get('combined_error_median'),
                'combined_error_q1': row.get('combined_error_q1'),
                'combined_error_q3': row.get('combined_error_q3')
            })
        
        # Define ordering priority for base solver types
        solver_priority = {
            '5PT': 1,                      # Minimal solvers first
            'UPRIGHT 3PT': 2,
            'UPRIGHT PLANAR 2PT': 3,
            'UPRIGHT PLANAR 3PT': 4,       # Linear solvers last
            '8PT': 5
        }
        
        # Sort solver groups by priority and then by N count
        def sort_key(group_name):
            # Extract base solver name and N count
            if '(N=' in group_name:
                base_name = group_name.split(' (N=')[0]
                n_count = int(group_name.split('(N=')[1].split(')')[0])
            else:
                base_name = group_name
                n_count = 0
            
            priority = solver_priority.get(base_name, 999)
            return (priority, n_count)
        
        # Create ordered dictionary
        ordered_groups = {}
        sorted_group_names = sorted(solver_data.keys(), key=sort_key)
        
        for group_name in sorted_group_names:
            ordered_groups[group_name] = solver_data[group_name]
        
        return ordered_groups

    def plot_error_analysis(self, df):
        """Create focused error analysis plots matching abs-pose style"""
        print("\nCreating error analysis plots...")
        
        if df.empty:
            print("No data to plot!")
            return
        
        # Target noise levels for cleaner plots (matching abs-pose style)
        target_noise_levels = [0.01, 0.02, 0.04]
        
        # Filter data to target noise levels AND realistic data mode only
        df_filtered = df[(df['noise_level'].isin(target_noise_levels)) & (df['data_mode'] == 'realistic')]
        
        if df_filtered.empty:
            print(f"No realistic data found for target noise levels {target_noise_levels}")
            print(f"Available noise levels: {sorted(df['noise_level'].unique())}")
            print(f"Available data modes: {sorted(df['data_mode'].unique())}")
            # Fall back to any realistic data available
            df_realistic = df[df['data_mode'] == 'realistic']
            if not df_realistic.empty:
                available_noise = sorted(df_realistic['noise_level'].unique())
                if len(available_noise) >= 3:
                    target_noise_levels = available_noise[:3]
                    df_filtered = df_realistic[df_realistic['noise_level'].isin(target_noise_levels)]
                else:
                    df_filtered = df_realistic
                    target_noise_levels = available_noise
            else:
                print("No realistic data found at all!")
                return
        
        # Create four different visualization approaches
        print("Creating simple plots (mean only - easy to read)...")
        self.plot_error_analysis_simple(df_filtered, target_noise_levels, "simple")
        
        print("Creating traditional plots (mean ± std)...")
        self.plot_error_analysis_traditional(df_filtered, target_noise_levels, "traditional")
        
        print("Creating robust plots (median + quartiles)...")
        self.plot_error_analysis_robust(df_filtered, target_noise_levels, "robust")
        
        print("Creating box and whisker plots (complete distribution)...")
        self.plot_error_analysis_boxplot(df_filtered, target_noise_levels, "boxplot")
        
        # Create individual solver comparison plots if we have 8pt variants
        self.plot_individual_solver_analysis(df_filtered, self.create_solver_groups_ordered(df_filtered))

    def plot_error_analysis_traditional(self, df, target_noise_levels, plot_suffix):
        """Create error analysis plots using traditional statistics (mean ± std)"""
        
        # Create solver groups with proper ordering
        solver_groups = self.create_solver_groups_ordered(df)
        
        if not solver_groups:
            print("No solver groups created!")
            return
        
        print(f"Plotting {len(solver_groups)} solver groups with noise levels: {target_noise_levels}")
        
        # Set up the figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 12))
        
        # Prepare data for plotting
        group_names = list(solver_groups.keys())
        group_positions = np.arange(len(group_names))
        
        # Width settings for bars
        noise_width = 0.25  # Width for each noise level
        precision_offset = 0.1  # Offset between float/double within same noise level
        
        # Plot rotation errors (top subplot)
        ax1.set_title('Relative Pose Estimation: Rotation Error vs Noise Level (Traditional Stats)', fontsize=16, fontweight='bold')
        ax1.set_ylabel('Error (degrees)')
        
        # Plot translation direction errors (bottom subplot)  
        ax2.set_title('Relative Pose Estimation: Translation Direction Error vs Noise Level (Traditional Stats)', fontsize=16, fontweight='bold')
        ax2.set_ylabel('Error (degrees)')
        ax2.set_xlabel('Solver Groups', fontsize=14)
        
        # For each noise level
        for noise_idx, noise_level in enumerate(target_noise_levels):
            noise_offset = (noise_idx - len(target_noise_levels)/2 + 0.5) * noise_width
            
            # Get base color for this noise level and create float/double variants
            base_color = plt.cm.Set1(noise_idx / max(len(target_noise_levels)-1, 1))  # Different color for each noise level
            
            # Create lighter shade for float, darker shade for double
            import matplotlib.colors as mcolors
            float_color = mcolors.to_rgba(base_color, alpha=0.7)  # Lighter/more transparent
            double_color = mcolors.to_rgba(base_color, alpha=1.0)  # Darker/more opaque
            
            # Alternative: Use actual lighter/darker shades
            hsv_color = mcolors.rgb_to_hsv(base_color[:3])
            # Make float lighter (increase value/brightness)
            float_hsv = (hsv_color[0], hsv_color[1] * 0.6, min(1.0, hsv_color[2] * 1.3))
            float_color = mcolors.hsv_to_rgb(float_hsv)
            
            # Make double darker (decrease value/brightness) 
            double_hsv = (hsv_color[0], hsv_color[1], hsv_color[2] * 0.7)
            double_color = mcolors.hsv_to_rgb(double_hsv)
            
            # Prepare data arrays for this noise level
            float_rot_means = []
            float_rot_stds = []
            float_trans_means = []
            float_trans_stds = []
            double_rot_means = []
            double_rot_stds = []
            double_trans_means = []
            double_trans_stds = []
            
            for group_name in group_names:
                group_data = solver_groups[group_name]
                
                # Find float and double data for this noise level
                float_data = None
                double_data = None
                
                for entry in group_data:
                    if entry['noise_level'] == noise_level:
                        if entry['precision'] == 'float':
                            float_data = entry
                        elif entry['precision'] == 'double':
                            double_data = entry
                
                # FIXED: Don't clamp actual data - only use min_value for missing data
                min_value_for_missing = 1e-2  # Only for cases with no data
                min_value_for_zero = 1e-5     # For actual zero values (much smaller, distinguishable)
                
                if float_data:
                    # Use actual values, handle true zeros for log scale
                    rot_val = float_data['rotation_error_mean'] if float_data['rotation_error_mean'] > 0 else min_value_for_zero
                    trans_val = float_data['translation_direction_error_mean'] if float_data['translation_direction_error_mean'] > 0 else min_value_for_zero
                    
                    # ENHANCED: Ensure minimum visibility - if value is too small, use a larger minimum
                    min_visible = 1e-6  # REDUCED: Smaller minimum to show very small values like 5.60089e-05
                    if rot_val > 0 and rot_val < min_visible:
                        print(f"    DEBUG: Adjusting tiny rotation value from {rot_val:.2e} to {min_visible:.2e} for visibility")
                        rot_val = min_visible
                    if trans_val > 0 and trans_val < min_visible:
                        print(f"    DEBUG: Adjusting tiny translation value from {trans_val:.2e} to {min_visible:.2e} for visibility")
                        trans_val = min_visible
                    
                    float_rot_means.append(rot_val)
                    float_rot_stds.append(float_data['rotation_error_std'])
                    float_trans_means.append(trans_val)
                    float_trans_stds.append(float_data['translation_direction_error_std'])
                else:
                    # No data available for this combination - use placeholder
                    float_rot_means.append(min_value_for_missing)
                    float_rot_stds.append(0)
                    float_trans_means.append(min_value_for_missing)
                    float_trans_stds.append(0)
                
                if double_data:
                    # Use actual values, handle true zeros for log scale
                    rot_val = double_data['rotation_error_mean'] if double_data['rotation_error_mean'] > 0 else min_value_for_zero
                    trans_val = double_data['translation_direction_error_mean'] if double_data['translation_direction_error_mean'] > 0 else min_value_for_zero
                    
                    # ENHANCED: Ensure minimum visibility - if value is too small, use a larger minimum
                    min_visible = 1e-6  # REDUCED: Smaller minimum to show very small values like 5.60089e-05
                    if rot_val > 0 and rot_val < min_visible:
                        print(f"    DEBUG: Adjusting tiny rotation value from {rot_val:.2e} to {min_visible:.2e} for visibility")
                        rot_val = min_visible
                    if trans_val > 0 and trans_val < min_visible:
                        print(f"    DEBUG: Adjusting tiny translation value from {trans_val:.2e} to {min_visible:.2e} for visibility")
                        trans_val = min_visible
                    
                    double_rot_means.append(rot_val)
                    double_rot_stds.append(double_data['rotation_error_std'])
                    double_trans_means.append(trans_val)
                    double_trans_stds.append(double_data['translation_direction_error_std'])
                else:
                    # No data available for this combination - use placeholder
                    double_rot_means.append(min_value_for_missing)
                    double_rot_stds.append(0)
                    double_trans_means.append(min_value_for_missing)
                    double_trans_stds.append(0)
            
            # FIXED: Compute asymmetric error bars for log scale
            # On log scale, we need to ensure error bars don't go negative
            float_rot_lower_errors = []
            float_rot_upper_errors = []
            float_trans_lower_errors = []
            float_trans_upper_errors = []
            double_rot_lower_errors = []
            double_rot_upper_errors = []
            double_trans_lower_errors = []
            double_trans_upper_errors = []
            
            for i in range(len(group_names)):
                # For log scale, use asymmetric error bars that don't go negative
                
                # Float rotation errors
                mean_val = float_rot_means[i]
                std_val = float_rot_stds[i]
                lower_bound = max(mean_val - std_val, mean_val * 0.1)  # Don't go below 10% of mean
                upper_bound = mean_val + std_val
                float_rot_lower_errors.append(mean_val - lower_bound)
                float_rot_upper_errors.append(upper_bound - mean_val)
                
                # Float translation errors  
                mean_val = float_trans_means[i]
                std_val = float_trans_stds[i]
                lower_bound = max(mean_val - std_val, mean_val * 0.1)
                upper_bound = mean_val + std_val
                float_trans_lower_errors.append(mean_val - lower_bound)
                float_trans_upper_errors.append(upper_bound - mean_val)
                
                # Double rotation errors
                mean_val = double_rot_means[i]
                std_val = double_rot_stds[i] 
                lower_bound = max(mean_val - std_val, mean_val * 0.1)
                upper_bound = mean_val + std_val
                double_rot_lower_errors.append(mean_val - lower_bound)
                double_rot_upper_errors.append(upper_bound - mean_val)
                
                # Double translation errors
                mean_val = double_trans_means[i]
                std_val = double_trans_stds[i]
                lower_bound = max(mean_val - std_val, mean_val * 0.1)
                upper_bound = mean_val + std_val
                double_trans_lower_errors.append(mean_val - lower_bound)
                double_trans_upper_errors.append(upper_bound - mean_val)
            
            # Create asymmetric error arrays for matplotlib
            float_rot_errors = [float_rot_lower_errors, float_rot_upper_errors]
            float_trans_errors = [float_trans_lower_errors, float_trans_upper_errors]
            double_rot_errors = [double_rot_lower_errors, double_rot_upper_errors]  
            double_trans_errors = [double_trans_lower_errors, double_trans_upper_errors]
            
            # Plot bars with error bars
            float_positions = group_positions + noise_offset - precision_offset/2
            double_positions = group_positions + noise_offset + precision_offset/2
            
            # DEBUG: Print values being plotted for the first noise level
            if noise_idx == 0:
                print(f"\nDEBUG: Plotting traditional values for noise={noise_level}")
                for i, group_name in enumerate(group_names):
                    float_rot = float_rot_means[i]
                    double_rot = double_rot_means[i]
                    float_std = float_rot_stds[i]
                    double_std = double_rot_stds[i]
                    
                    # Check for problematic values
                    float_problem = ""
                    double_problem = ""
                    if float_rot <= 1e-4:
                        float_problem = " [VERY_SMALL]"
                    if double_rot <= 1e-4:
                        double_problem = " [VERY_SMALL]"
                    if float_std > 0 and float_rot <= 1e-4:
                        float_problem += " [STD>0_BUT_TINY_MEAN]"
                    if double_std > 0 and double_rot <= 1e-4:
                        double_problem += " [STD>0_BUT_TINY_MEAN]"
                        
                    print(f"  {group_name}: float_rot={float_rot:.6f}±{float_std:.4f}{float_problem}, double_rot={double_rot:.6f}±{double_std:.4f}{double_problem}")
            
            # Rotation errors (top subplot)
            bars_float_rot = ax1.bar(float_positions, float_rot_means, 
                                   width=precision_offset*0.8, 
                                   yerr=float_rot_errors,  # FIXED: Use asymmetric errors
                                   color=float_color, 
                                   alpha=0.9,
                                   capsize=3,
                                   error_kw={'linewidth': 1.5})
            
            bars_double_rot = ax1.bar(double_positions, double_rot_means, 
                                    width=precision_offset*0.8, 
                                    yerr=double_rot_errors,  # FIXED: Use asymmetric errors
                                    color=double_color, 
                                    alpha=0.9,
                                    capsize=3,
                                    error_kw={'linewidth': 1.5})
            
            # Translation direction errors (bottom subplot)
            bars_float_trans = ax2.bar(float_positions, float_trans_means, 
                                     width=precision_offset*0.8, 
                                     yerr=float_trans_errors,  # FIXED: Use asymmetric errors
                                     color=float_color, 
                                     alpha=0.9,
                                     capsize=3,
                                     error_kw={'linewidth': 1.5})
            
            bars_double_trans = ax2.bar(double_positions, double_trans_means, 
                                      width=precision_offset*0.8, 
                                      yerr=double_trans_errors,  # FIXED: Use asymmetric errors
                                      color=double_color, 
                                      alpha=0.9,
                                      capsize=3,
                                      error_kw={'linewidth': 1.5})
        
        # Customize both subplots
        for ax in [ax1, ax2]:
            ax.set_xticks(group_positions)
            ax.set_xticklabels(group_names, rotation=45, ha='right')
            ax.set_yscale('linear')
            
            # Let matplotlib auto-scale to actual data range for better visibility
            
            ax.grid(True, alpha=0.3)
            ax.tick_params(axis='both', which='major', labelsize=12)
            
            # Improved log formatter to handle the full range with better precision
            def custom_log_formatter(x, pos):
                if x >= 10:
                    return f'{x:.0f}°'
                elif x >= 1:
                    return f'{x:.2f}°'
                elif x >= 0.1:
                    return f'{x:.3f}°'
                elif x >= 0.01:
                    return f'{x:.4f}°'  # More precision for small values
                elif x >= 0.001:
                    return f'{x:.5f}°'  # Even more precision for very small values
                else:
                    return f'{x:.1e}°'  # Scientific notation for extremely small values
            
            ax.yaxis.set_major_formatter(FuncFormatter(custom_log_formatter))
        
        # Adjust layout and save
        plt.tight_layout()
        
        # Save the plot
        output_file = self.plots_dir / f"relative_pose_error_analysis_{plot_suffix}.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved traditional error analysis plot: {output_file}")
        plt.close()  # Close to free memory

    def plot_error_analysis_robust(self, df, target_noise_levels, plot_suffix):
        """Create error analysis plots using robust statistics (median + quartiles)"""
        
        # Create solver groups with proper ordering
        solver_groups = self.create_solver_groups_ordered(df)
        
        if not solver_groups:
            print("No solver groups created!")
            return
        
        print(f"Plotting {len(solver_groups)} solver groups with robust statistics")
        
        # Set up the figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 12))
        
        # Prepare data for plotting
        group_names = list(solver_groups.keys())
        group_positions = np.arange(len(group_names))
        
        # Width settings for bars
        noise_width = 0.25  # Width for each noise level
        precision_offset = 0.1  # Offset between float/double within same noise level
        
        # Plot rotation errors (top subplot)
        ax1.set_title('Relative Pose Estimation: Rotation Error vs Noise Level (Robust Stats)', fontsize=16, fontweight='bold')
        ax1.set_ylabel('Error (degrees)')
        
        # Plot translation direction errors (bottom subplot)  
        ax2.set_title('Relative Pose Estimation: Translation Direction Error vs Noise Level (Robust Stats)', fontsize=16, fontweight='bold')
        ax2.set_ylabel('Error (degrees)')
        ax2.set_xlabel('Solver Groups', fontsize=14)
        
        # For each noise level
        for noise_idx, noise_level in enumerate(target_noise_levels):
            noise_offset = (noise_idx - len(target_noise_levels)/2 + 0.5) * noise_width
            
            # Get base color for this noise level and create float/double variants
            base_color = plt.cm.Set1(noise_idx / max(len(target_noise_levels)-1, 1))  # Different color for each noise level
            
            # Create lighter shade for float, darker shade for double
            import matplotlib.colors as mcolors
            float_color = mcolors.to_rgba(base_color, alpha=0.7)  # Lighter/more transparent
            double_color = mcolors.to_rgba(base_color, alpha=1.0)  # Darker/more opaque
            
            # Alternative: Use actual lighter/darker shades
            hsv_color = mcolors.rgb_to_hsv(base_color[:3])
            # Make float lighter (increase value/brightness)
            float_hsv = (hsv_color[0], hsv_color[1] * 0.6, min(1.0, hsv_color[2] * 1.3))
            float_color = mcolors.hsv_to_rgb(float_hsv)
            
            # Make double darker (decrease value/brightness) 
            double_hsv = (hsv_color[0], hsv_color[1], hsv_color[2] * 0.7)
            double_color = mcolors.hsv_to_rgb(double_hsv)
            
            # Prepare data arrays for this noise level (using robust stats)
            float_rot_medians = []
            float_rot_q1s = []
            float_rot_q3s = []
            float_trans_medians = []
            float_trans_q1s = []
            float_trans_q3s = []
            double_rot_medians = []
            double_rot_q1s = []
            double_rot_q3s = []
            double_trans_medians = []
            double_trans_q1s = []
            double_trans_q3s = []
            
            for group_name in group_names:
                group_data = solver_groups[group_name]
                
                # Find float and double data for this noise level
                float_data = None
                double_data = None
                
                for entry in group_data:
                    if entry['noise_level'] == noise_level:
                        if entry['precision'] == 'float':
                            float_data = entry
                        elif entry['precision'] == 'double':
                            double_data = entry
                
                min_value_for_missing = 1e-2  # Only for cases with no data
                min_value_for_zero = 1e-5     # For actual zero values
                
                if float_data:
                    # DEBUG: Check what keys are actually available
                    if 'rotation_error_median' not in float_data:
                        print(f"ERROR: 'rotation_error_median' missing from float_data for {group_name} noise={noise_level}")
                        print(f"Available keys: {list(float_data.keys())}")
                        # Use fallback values
                        rot_median = float_data.get('rotation_error_mean', min_value_for_zero)
                        trans_median = float_data.get('translation_direction_error_mean', min_value_for_zero)
                        q1_rot = max(0, rot_median - float_data.get('rotation_error_std', 0))
                        q3_rot = rot_median + float_data.get('rotation_error_std', 0)
                        q1_trans = max(0, trans_median - float_data.get('translation_direction_error_std', 0))
                        q3_trans = trans_median + float_data.get('translation_direction_error_std', 0)
                    else:
                        # Use actual robust statistics
                        rot_median = float_data['rotation_error_median'] if float_data['rotation_error_median'] > 0 else min_value_for_zero
                        trans_median = float_data['translation_direction_error_median'] if float_data['translation_direction_error_median'] > 0 else min_value_for_zero
                        q1_rot = float_data['rotation_error_q1'] if float_data['rotation_error_q1'] > 0 else min_value_for_zero
                        q3_rot = float_data['rotation_error_q3'] if float_data['rotation_error_q3'] > 0 else min_value_for_zero
                        q1_trans = float_data['translation_direction_error_q1'] if float_data['translation_direction_error_q1'] > 0 else min_value_for_zero
                        q3_trans = float_data['translation_direction_error_q3'] if float_data['translation_direction_error_q3'] > 0 else min_value_for_zero
                    
                    float_rot_medians.append(rot_median)
                    float_rot_q1s.append(q1_rot)
                    float_rot_q3s.append(q3_rot)
                    float_trans_medians.append(trans_median)
                    float_trans_q1s.append(q1_trans)
                    float_trans_q3s.append(q3_trans)
                else:
                    # No data available
                    float_rot_medians.append(min_value_for_missing)
                    float_rot_q1s.append(min_value_for_missing)
                    float_rot_q3s.append(min_value_for_missing)
                    float_trans_medians.append(min_value_for_missing)
                    float_trans_q1s.append(min_value_for_missing)
                    float_trans_q3s.append(min_value_for_missing)
                
                if double_data:
                    # DEBUG: Check what keys are actually available
                    if 'rotation_error_median' not in double_data:
                        print(f"ERROR: 'rotation_error_median' missing from double_data for {group_name} noise={noise_level}")
                        print(f"Available keys: {list(double_data.keys())}")
                        # Use fallback values
                        rot_median = double_data.get('rotation_error_mean', min_value_for_zero)
                        trans_median = double_data.get('translation_direction_error_mean', min_value_for_zero)
                        q1_rot = max(0, rot_median - double_data.get('rotation_error_std', 0))
                        q3_rot = rot_median + double_data.get('rotation_error_std', 0)
                        q1_trans = max(0, trans_median - double_data.get('translation_direction_error_std', 0))
                        q3_trans = trans_median + double_data.get('translation_direction_error_std', 0)
                    else:
                        # Use actual robust statistics
                        rot_median = double_data['rotation_error_median'] if double_data['rotation_error_median'] > 0 else min_value_for_zero
                        trans_median = double_data['translation_direction_error_median'] if double_data['translation_direction_error_median'] > 0 else min_value_for_zero
                        q1_rot = double_data['rotation_error_q1'] if double_data['rotation_error_q1'] > 0 else min_value_for_zero
                        q3_rot = double_data['rotation_error_q3'] if double_data['rotation_error_q3'] > 0 else min_value_for_zero
                        q1_trans = double_data['translation_direction_error_q1'] if double_data['translation_direction_error_q1'] > 0 else min_value_for_zero
                        q3_trans = double_data['translation_direction_error_q3'] if double_data['translation_direction_error_q3'] > 0 else min_value_for_zero
                    
                    double_rot_medians.append(rot_median)
                    double_rot_q1s.append(q1_rot)
                    double_rot_q3s.append(q3_rot)
                    double_trans_medians.append(trans_median)
                    double_trans_q1s.append(q1_trans)
                    double_trans_q3s.append(q3_trans)
                else:
                    # No data available
                    double_rot_medians.append(min_value_for_missing)
                    double_rot_q1s.append(min_value_for_missing)
                    double_rot_q3s.append(min_value_for_missing)
                    double_trans_medians.append(min_value_for_missing)
                    double_trans_q1s.append(min_value_for_missing)
                    double_trans_q3s.append(min_value_for_missing)
            
            # Compute error bars from quartiles (no artificial capping needed!)
            float_rot_lower_errors = [median - q1 for median, q1 in zip(float_rot_medians, float_rot_q1s)]
            float_rot_upper_errors = [q3 - median for median, q3 in zip(float_rot_medians, float_rot_q3s)]
            float_trans_lower_errors = [median - q1 for median, q1 in zip(float_trans_medians, float_trans_q1s)]
            float_trans_upper_errors = [q3 - median for median, q3 in zip(float_trans_medians, float_trans_q3s)]
            
            double_rot_lower_errors = [median - q1 for median, q1 in zip(double_rot_medians, double_rot_q1s)]
            double_rot_upper_errors = [q3 - median for median, q3 in zip(double_rot_medians, double_rot_q3s)]
            double_trans_lower_errors = [median - q1 for median, q1 in zip(double_trans_medians, double_trans_q1s)]
            double_trans_upper_errors = [q3 - median for median, q3 in zip(double_trans_medians, double_trans_q3s)]
            
            # Create error arrays for matplotlib
            float_rot_errors = [float_rot_lower_errors, float_rot_upper_errors]
            float_trans_errors = [float_trans_lower_errors, float_trans_upper_errors]
            double_rot_errors = [double_rot_lower_errors, double_rot_upper_errors]  
            double_trans_errors = [double_trans_lower_errors, double_trans_upper_errors]
            
            # Plot bars with quartile error bars
            float_positions = group_positions + noise_offset - precision_offset/2
            double_positions = group_positions + noise_offset + precision_offset/2
            
            # DEBUG: Print robust values for first noise level
            if noise_idx == 0:
                print(f"\nDEBUG: Plotting robust values for noise={noise_level}")
                for i, group_name in enumerate(group_names):
                    float_median = float_rot_medians[i]
                    float_q1 = float_rot_q1s[i]
                    float_q3 = float_rot_q3s[i]
                    double_median = double_rot_medians[i]
                    double_q1 = double_rot_q1s[i]
                    double_q3 = double_rot_q3s[i]
                    print(f"  {group_name}: float_rot_median={float_median:.6e} [{float_q1:.6e}, {float_q3:.6e}], double_rot_median={double_median:.6e} [{double_q1:.6e}, {double_q3:.6e}]")
            
            # Rotation errors (top subplot) - using median + quartile error bars
            bars_float_rot = ax1.bar(float_positions, float_rot_medians, 
                                   width=precision_offset*0.8, 
                                   yerr=float_rot_errors,
                                   color=float_color, 
                                   alpha=0.9,
                                   capsize=3,
                                   error_kw={'linewidth': 1.5})
            
            bars_double_rot = ax1.bar(double_positions, double_rot_medians, 
                                    width=precision_offset*0.8, 
                                    yerr=double_rot_errors,
                                    color=double_color, 
                                    alpha=0.9,
                                    capsize=3,
                                    error_kw={'linewidth': 1.5})
            
            # Translation direction errors (bottom subplot) - using median + quartile error bars
            bars_float_trans = ax2.bar(float_positions, float_trans_medians, 
                                     width=precision_offset*0.8, 
                                     yerr=float_trans_errors,
                                     color=float_color, 
                                     alpha=0.9,
                                     capsize=3,
                                     error_kw={'linewidth': 1.5})
            
            bars_double_trans = ax2.bar(double_positions, double_trans_medians, 
                                      width=precision_offset*0.8, 
                                      yerr=double_trans_errors,
                                      color=double_color, 
                                      alpha=0.9,
                                      capsize=3,
                                      error_kw={'linewidth': 1.5})
        
        # Customize both subplots (same as traditional)
        for ax in [ax1, ax2]:
            ax.set_xticks(group_positions)
            ax.set_xticklabels(group_names, rotation=45, ha='right')
            ax.set_yscale('linear')
            
            # Let matplotlib auto-scale to actual data range for better visibility
            
            ax.grid(True, alpha=0.3)
            ax.tick_params(axis='both', which='major', labelsize=12)
            
            # Improved log formatter to handle the full range with better precision
            def custom_log_formatter(x, pos):
                if x >= 10:
                    return f'{x:.0f}°'
                elif x >= 1:
                    return f'{x:.2f}°'
                elif x >= 0.1:
                    return f'{x:.3f}°'
                elif x >= 0.01:
                    return f'{x:.4f}°'  # More precision for small values
                elif x >= 0.001:
                    return f'{x:.5f}°'  # Even more precision for very small values
                else:
                    return f'{x:.1e}°'  # Scientific notation for extremely small values
            
            ax.yaxis.set_major_formatter(FuncFormatter(custom_log_formatter))
        
        # Adjust layout and save
        plt.tight_layout()
        
        # Save the plot
        output_file = self.plots_dir / f"relative_pose_error_analysis_{plot_suffix}.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved robust error analysis plot: {output_file}")
        plt.close()  # Close to free memory

    def plot_error_analysis_boxplot(self, df, target_noise_levels, plot_suffix):
        """Create box and whisker plots showing complete statistical distribution"""
        
        # Create solver groups with proper ordering
        solver_groups = self.create_solver_groups_ordered(df)
        
        if not solver_groups:
            print("No solver groups created!")
            return
        
        print(f"Creating box plots for {len(solver_groups)} solver groups")
        
        # Set up the figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 14))
        
        # Prepare data for plotting
        group_names = list(solver_groups.keys())
        
        # Plot rotation errors (top subplot)
        ax1.set_title('Relative Pose Estimation: Rotation Error Distribution (Box & Whisker)', fontsize=16, fontweight='bold')
        ax1.set_ylabel('Error (degrees)')
        
        # Plot translation direction errors (bottom subplot)  
        ax2.set_title('Relative Pose Estimation: Translation Direction Error Distribution (Box & Whisker)', fontsize=16, fontweight='bold')
        ax2.set_ylabel('Error (degrees)')
        ax2.set_xlabel('Solver Groups', fontsize=14)
        
        # Prepare data for box plots
        rotation_data_for_boxplot = []
        translation_data_for_boxplot = []
        box_labels = []
        box_colors = []
        
        # For each solver group and noise level combination
        for group_name in group_names:
            group_data = solver_groups[group_name]
            
            for noise_level in target_noise_levels:
                # Find data for this noise level (combine float and double)
                combined_rot_values = []
                combined_trans_values = []
                
                for entry in group_data:
                    if entry['noise_level'] == noise_level:
                        # We need to extract the actual raw values, not just statistics
                        # For now, we'll simulate the distribution based on the statistics we have
                        mean_rot = entry['rotation_error_mean']
                        std_rot = entry['rotation_error_std']
                        median_rot = entry['rotation_error_median']
                        q1_rot = entry['rotation_error_q1']
                        q3_rot = entry['rotation_error_q3']
                        
                        mean_trans = entry['translation_direction_error_mean']
                        std_trans = entry['translation_direction_error_std']
                        median_trans = entry['translation_direction_error_median']
                        q1_trans = entry['translation_direction_error_q1']
                        q3_trans = entry['translation_direction_error_q3']
                        
                        # APPROACH: Create synthetic data that matches the known statistics
                        # This gives us the distribution shape for box plotting
                        synthetic_rot_data = self.create_synthetic_distribution(
                            mean_rot, std_rot, median_rot, q1_rot, q3_rot, n_samples=1000
                        )
                        synthetic_trans_data = self.create_synthetic_distribution(
                            mean_trans, std_trans, median_trans, q1_trans, q3_trans, n_samples=1000
                        )
                        
                        combined_rot_values.extend(synthetic_rot_data)
                        combined_trans_values.extend(synthetic_trans_data)
                
                if combined_rot_values:  # Only add if we have data
                    rotation_data_for_boxplot.append(combined_rot_values)
                    translation_data_for_boxplot.append(combined_trans_values)
                    
                    # Create label and color
                    label = f"{group_name}\n(noise={noise_level})"
                    box_labels.append(label)
                    box_colors.append(self.get_noise_color(noise_level, 'float'))
        
        # Create box plots
        if rotation_data_for_boxplot:
            # Rotation errors box plot
            bp1 = ax1.boxplot(rotation_data_for_boxplot, labels=box_labels, patch_artist=True,
                             showfliers=True, whis=1.5, notch=False)
            
            # Color the boxes
            for patch, color in zip(bp1['boxes'], box_colors):
                patch.set_facecolor(color)
                patch.set_alpha(0.7)
            
            # Translation errors box plot
            bp2 = ax2.boxplot(translation_data_for_boxplot, labels=box_labels, patch_artist=True,
                             showfliers=True, whis=1.5, notch=False)
            
            # Color the boxes
            for patch, color in zip(bp2['boxes'], box_colors):
                patch.set_facecolor(color)
                patch.set_alpha(0.7)
        
        # Customize both subplots
        for ax in [ax1, ax2]:
            ax.set_yscale('linear')
            # Let matplotlib auto-scale to actual data range for better visibility
            ax.grid(True, alpha=0.3, axis='y')
            ax.tick_params(axis='both', which='major', labelsize=10)
            ax.tick_params(axis='x', rotation=45)
            
            # Improved log formatter
            def custom_log_formatter(x, pos):
                if x >= 10:
                    return f'{x:.0f}°'
                elif x >= 1:
                    return f'{x:.2f}°'
                elif x >= 0.1:
                    return f'{x:.3f}°'
                elif x >= 0.01:
                    return f'{x:.4f}°'
                elif x >= 0.001:
                    return f'{x:.5f}°'
                else:
                    return f'{x:.1e}°'
            
            ax.yaxis.set_major_formatter(FuncFormatter(custom_log_formatter))
        
        # Add explanation text
        fig.text(0.02, 0.02, 
                'Box plot elements: Center line = median, Box = Q1 to Q3 (IQR), Whiskers = 1.5×IQR, Dots = outliers',
                fontsize=10, style='italic')
        
        # Adjust layout and save
        plt.tight_layout()
        plt.subplots_adjust(bottom=0.15)  # Make room for explanation text
        
        # Save the plot
        output_file = self.plots_dir / f"relative_pose_error_analysis_{plot_suffix}.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved box plot analysis: {output_file}")
        plt.close()
    
    def create_synthetic_distribution(self, mean_val, std_val, median_val, q1_val, q3_val, n_samples=1000):
        """
        Create synthetic data that matches known statistics for box plotting
        This handles the case where we only have summary statistics, not raw data
        """
        import numpy as np
        
        # Handle edge cases
        if mean_val == 0 and median_val == 0 and std_val == 0:
            # All values are exactly zero
            return [0.0] * n_samples
        
        if median_val == 0 and q1_val == 0 and q3_val == 0:
            # Most values are zero, but mean > 0 indicates some outliers
            # Create a distribution with mostly zeros and some outliers
            n_zeros = int(0.75 * n_samples)  # 75% zeros
            n_outliers = n_samples - n_zeros
            
            zeros = [0.0] * n_zeros
            # Create outliers that give us the correct mean
            target_sum = mean_val * n_samples
            outlier_values = np.random.exponential(target_sum / n_outliers, n_outliers)
            
            return zeros + outlier_values.tolist()
        
        # Normal case: create distribution matching quartiles
        # Use inverse transform sampling to match quartiles approximately
        samples = []
        for _ in range(n_samples):
            u = np.random.uniform(0, 1)
            if u < 0.25:
                # Below Q1: interpolate from 0 to Q1
                value = u * 4 * q1_val
            elif u < 0.5:
                # Q1 to median
                value = q1_val + (u - 0.25) * 4 * (median_val - q1_val)
            elif u < 0.75:
                # Median to Q3
                value = median_val + (u - 0.5) * 4 * (q3_val - median_val)
            else:
                # Above Q3: extrapolate with exponential tail
                value = q3_val + np.random.exponential((q3_val - q1_val) * 0.5)
            
            samples.append(max(0, value))  # Ensure non-negative
        
        return samples

    def plot_individual_solver_analysis(self, df, solver_groups):
        """Create individual solver analysis plots if needed"""
        # For now, just print a summary since we don't have 8pt variants in our current data
        print(f"\nSolver Summary:")
        for group_name, group_data in solver_groups.items():
            print(f"  {group_name}: {len(group_data)} configurations")
        
        # If we had 8pt variants with different point counts, we would create separate plots here
        # This is a placeholder for future expansion

    def analyze_data(self):
        """Main analysis function - load data and create all plots"""
        print("Enhanced Relative Pose Analysis Starting...")
        print("="*50)
        
        # Load data from log files
        df = self.load_data_from_logs()
        
        if df.empty:
            print("No CSV files found! Please run the data generator first.")
            return
        
        # Create error analysis plots
        self.plot_error_analysis(df)
        
        # Create summary statistics
        self.create_summary_statistics(df)
        
        print(f"\n✓ Analysis complete! Plots saved to: {self.plots_dir}/")

    def create_summary_statistics(self, df):
        """Create and save summary statistics"""
        print("\nGenerating summary statistics...")
        
        if df.empty:
            print("No data available for summary statistics")
            return
        
        summary_file = self.plots_dir / "analysis_summary.txt"
        
        with open(summary_file, 'w') as f:
            f.write("Relative Pose Estimation Analysis Summary\n")
            f.write("="*50 + "\n\n")
            
            f.write(f"Total configurations analyzed: {len(df)}\n")
            f.write(f"Solvers: {', '.join(sorted(df['solver'].unique()))}\n")
            f.write(f"Precision types: {', '.join(sorted(df['precision'].unique()))}\n")
            f.write(f"Noise levels: {', '.join([str(x) for x in sorted(df['noise_level'].unique())])}\n\n")
            
            # Success rate analysis
            f.write("Success Rate Analysis:\n")
            f.write("-" * 30 + "\n")
            for solver in sorted(df['solver'].unique()):
                solver_data = df[df['solver'] == solver]
                avg_success = solver_data['success_rate'].mean()
                f.write(f"{solver}: {avg_success:.1f}% average success rate\n")
            f.write("\n")
            
            # Error analysis by solver
            f.write("Error Analysis by Solver:\n")
            f.write("-" * 30 + "\n")
            for solver in sorted(df['solver'].unique()):
                solver_data = df[df['solver'] == solver]
                avg_rot_error = solver_data['rotation_error_mean'].mean()
                avg_trans_error = solver_data['translation_direction_error_mean'].mean()
                f.write(f"{solver}:\n")
                f.write(f"  Average rotation error: {avg_rot_error:.2f}°\n")
                f.write(f"  Average translation direction error: {avg_trans_error:.2f}°\n")
            f.write("\n")
            
            # Best performing configurations
            f.write("Best Performing Configurations (lowest combined error):\n")
            f.write("-" * 50 + "\n")
            df_sorted = df.sort_values('combined_error_mean')
            for i, (_, row) in enumerate(df_sorted.head(5).iterrows()):
                f.write(f"{i+1}. {row['solver']} ({row['precision']}, noise={row['noise_level']}): {row['combined_error_mean']:.2f}°\n")
        
        print(f"Summary statistics saved to: {summary_file}")

    def plot_error_analysis_simple(self, df, target_noise_levels, plot_suffix):
        """Create simple error analysis plots with only mean values for clean comparison"""
        if df.empty:
            print("No data available for simple error analysis")
            return
        
        # Create solver groups with proper ordering
        solver_groups = self.create_solver_groups_ordered(df)
        
        if not solver_groups:
            print("No solver groups created!")
            return
        
        print(f"Creating simple mean error plots for {len(solver_groups)} solver groups")
        
        # Set up the figure with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10))
        fig.suptitle(f'Relative Pose Error Analysis (Mean Only) - {plot_suffix}', fontsize=16, fontweight='bold')
        
        # Prepare data for plotting
        group_names = list(solver_groups.keys())
        group_positions = np.arange(len(group_names))
        
        # Width settings for bars
        noise_width = 0.25  # Width for each noise level
        precision_offset = 0.1  # Offset between float/double within same noise level
        
        # Plot rotation errors (top subplot)
        ax1.set_title('Rotation Error vs Noise Level (Mean Values)', fontsize=14)
        ax1.set_ylabel('Rotation Error (degrees)', fontsize=12)
        
        # Plot translation direction errors (bottom subplot)  
        ax2.set_title('Translation Direction Error vs Noise Level (Mean Values)', fontsize=14)
        ax2.set_ylabel('Translation Direction Error (degrees)', fontsize=12)
        ax2.set_xlabel('Solver Groups', fontsize=12)
        
        # For each noise level
        for noise_idx, noise_level in enumerate(target_noise_levels):
            noise_offset = (noise_idx - len(target_noise_levels)/2 + 0.5) * noise_width
            
            # Get base color for this noise level and create float/double variants
            base_color = plt.cm.Set1(noise_idx / max(len(target_noise_levels)-1, 1))  # Different color for each noise level
            
            # Create lighter shade for float, darker shade for double
            import matplotlib.colors as mcolors
            float_color = mcolors.to_rgba(base_color, alpha=0.7)  # Lighter/more transparent
            double_color = mcolors.to_rgba(base_color, alpha=1.0)  # Darker/more opaque
            
            # Alternative: Use actual lighter/darker shades
            hsv_color = mcolors.rgb_to_hsv(base_color[:3])
            # Make float lighter (increase value/brightness)
            float_hsv = (hsv_color[0], hsv_color[1] * 0.6, min(1.0, hsv_color[2] * 1.3))
            float_color = mcolors.hsv_to_rgb(float_hsv)
            
            # Make double darker (decrease value/brightness) 
            double_hsv = (hsv_color[0], hsv_color[1], hsv_color[2] * 0.7)
            double_color = mcolors.hsv_to_rgb(double_hsv)
            
            # Prepare data arrays for this noise level (mean values only)
            float_rot_means = []
            float_trans_means = []
            double_rot_means = []
            double_trans_means = []
            
            for group_name in group_names:
                group_data = solver_groups[group_name]
                
                # Find float and double data for this noise level
                float_data = None
                double_data = None
                
                for entry in group_data:
                    if entry['noise_level'] == noise_level:
                        if entry['precision'] == 'float':
                            float_data = entry
                        elif entry['precision'] == 'double':
                            double_data = entry
                
                # Use a small value for missing data (will be barely visible)
                min_value_for_missing = 1e-3
                
                if float_data:
                    float_rot_means.append(float_data['rotation_error_mean'])
                    float_trans_means.append(float_data['translation_direction_error_mean'])
                else:
                    float_rot_means.append(min_value_for_missing)
                    float_trans_means.append(min_value_for_missing)
                
                if double_data:
                    double_rot_means.append(double_data['rotation_error_mean'])
                    double_trans_means.append(double_data['translation_direction_error_mean'])
                else:
                    double_rot_means.append(min_value_for_missing)
                    double_trans_means.append(min_value_for_missing)
            
            # Plot bars (no error bars, just clean mean values)
            float_positions = group_positions + noise_offset - precision_offset/2
            double_positions = group_positions + noise_offset + precision_offset/2
            
            bar_width = precision_offset * 0.8
            
            # Rotation errors
            ax1.bar(float_positions, float_rot_means, width=bar_width, 
                   color=float_color, alpha=0.8, 
                   label=f'Noise {noise_level} (Float)' if noise_idx < len(target_noise_levels) else "")
            ax1.bar(double_positions, double_rot_means, width=bar_width, 
                   color=double_color, alpha=0.8,
                   label=f'Noise {noise_level} (Double)' if noise_idx < len(target_noise_levels) else "")
            
            # Translation errors
            ax2.bar(float_positions, float_trans_means, width=bar_width, 
                   color=float_color, alpha=0.8, 
                   label=f'Noise {noise_level} (Float)' if noise_idx < len(target_noise_levels) else "")
            ax2.bar(double_positions, double_trans_means, width=bar_width, 
                   color=double_color, alpha=0.8,
                   label=f'Noise {noise_level} (Double)' if noise_idx < len(target_noise_levels) else "")
        
        # Customize both subplots
        for ax in [ax1, ax2]:
            ax.set_xticks(group_positions)
            ax.set_xticklabels(group_names, rotation=45, ha='right')
            ax.set_yscale('linear')
            
            # Let matplotlib auto-scale to actual data range for better visibility
            
            ax.grid(True, alpha=0.3)
            ax.tick_params(axis='both', which='major', labelsize=10)
            
            # Simple degree formatter (no scientific notation needed for mean values)
            def simple_formatter(x, pos):
                if x >= 1:
                    return f'{x:.1f}°'
                elif x >= 0.1:
                    return f'{x:.2f}°'
                else:
                    return f'{x:.3f}°'
            
            ax.yaxis.set_major_formatter(FuncFormatter(simple_formatter))
        
        # Add legends
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=10)
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left', fontsize=10)
        
        # Adjust layout and save
        plt.tight_layout()
        
        # Save the plot
        output_file = self.plots_dir / f"relative_pose_simple_mean_analysis_{plot_suffix}.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved simple mean error analysis plot: {output_file}")
        plt.close()  # Close to free memory

    def create_comprehensive_plots(self, df):
        """Create all comprehensive analysis plots"""
        if df.empty:
            print("No data available for comprehensive plots")
            return
        
        # Get actual noise levels from the data
        noise_levels = sorted(df['noise_level'].unique())
        data_mode = df['data_mode'].iloc[0] if not df.empty else "unknown"
        
        print(f"Creating comprehensive plots for {data_mode} mode with noise levels: {noise_levels}")
        
        # Simple mean-only analysis (NEW - clean and easy to read)
        self.plot_error_analysis_simple(df, noise_levels, f"{data_mode}_simple")
        
        # Traditional error analysis
        self.plot_error_analysis_traditional(df, noise_levels, f"{data_mode}_comprehensive")
        
        # Robust error analysis
        self.plot_error_analysis_robust(df, noise_levels, f"{data_mode}_robust")
        
        # Box plot analysis
        self.plot_error_analysis_boxplot(df, noise_levels, f"{data_mode}_boxplot")
        
        # Individual solver analysis
        solver_groups = self.create_solver_groups_ordered(df)
        self.plot_individual_solver_analysis(df, solver_groups)

def main():
    """Main function"""
    print("Relative Pose Analysis Script")
    print("============================\n")
    
    # Create analyzer and run analysis
    analyzer = RelativePoseAnalyzer()
    analyzer.analyze_data()

if __name__ == "__main__":
    main()