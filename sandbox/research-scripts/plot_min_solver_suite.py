#!/usr/bin/env python3
"""
Plot minimal solver suite: rotation error, cycles, and peak power (3x1 subplot).
- Top: Mean rotation error (deg) from benchmark_results.csv
- Middle: Cycles (log scale) from cs4-min-solver.csv
- Bottom: Peak power (mW) from cs4-min-solver.csv
Grouping: solver (x), MCU (color), precision (hatch)
"""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
from matplotlib.patches import Patch
import argparse
from matplotlib import cm
from matplotlib.colors import to_hex
import matplotlib as mpl

# --- File paths ---
parser = argparse.ArgumentParser(description='Plot minimal solver suite: rotation error, cycles, and peak power.')
parser.add_argument('--rot-csv', type=str, default='benchmark_results.csv', help='Path to rotation error CSV (from minimal solver benchmark)')
parser.add_argument('--mcu-csv', type=str, default='benchmark_results/rel-pose-mcu/cs4-min-solver.csv', help='Path to MCU cycles/power CSV')
args = parser.parse_args()

rot_csv = args.rot_csv
mcu_csv = args.mcu_csv
plot_dir = 'benchmark_results/plots'
os.makedirs(plot_dir, exist_ok=True)

# --- Solver order and labels ---
solver_order = [
    ('5pt', 5), ('u3pt', 3), ('up2pt', 2),
    ('8pt', 8), ('8pt', 16), ('8pt', 32), ('8pt', 64),
    ('up3pt', 3), ('up3pt', 8), ('up3pt', 16), ('up3pt', 32)
]
precision_order = ['float', 'double']
mcu_order = ['M4', 'M33', 'M7']
label_map = {'u3pt': 'u3pt', 'up2pt': 'up2pt', '5pt': '5pt', '8pt': '8pt', 'up3pt': 'up3pt'}
def make_xtick_label(solver, N):
    if solver in ['5pt', 'u3pt', 'up2pt']:
        return solver
    else:
        return f'{solver}-{N}'
group_labels = [make_xtick_label(s, N) for s, N in solver_order]

# --- Load rotation error data ---
df_rot = pd.read_csv(rot_csv)
df_rot = df_rot[df_rot['noise_level'] != 0]
noise_levels = sorted(df_rot['noise_level'].unique())
cb_noise_colors = [to_hex(c) for c in cm.get_cmap('Set1').colors[:len(noise_levels)]]

# --- Load MCU data ---
df_mcu = pd.read_csv(mcu_csv)
# Normalize solver names for linear solvers
import re
def norm_solver(s):
    if re.match(r'8pt-\d+', s): return '8pt'
    if re.match(r'up3pt-\d+', s): return 'up3pt'
    return s

def extract_N(s):
    m = re.match(r'(8pt|up3pt)-(\d+)', s)
    if m: return int(m.group(2))
    if s == '5pt': return 5
    if s == 'u3pt': return 3
    if s == 'up2pt': return 2
    return np.nan

df_mcu['solver'] = df_mcu['Solver'].apply(norm_solver)
df_mcu['N'] = df_mcu['Solver'].apply(extract_N)
df_mcu['precision'] = df_mcu['Scalar']

# --- Melt MCU data ---
cycles_long = df_mcu.melt(id_vars=['solver', 'N', 'precision'],
                         value_vars=['M4 Cycles', 'M33 Cycles', 'M7 Cycles'],
                         var_name='mcu', value_name='cycles')
cycles_long['mcu'] = cycles_long['mcu'].str.extract(r'(M4|M33|M7)')
peak_long = df_mcu.melt(id_vars=['solver', 'N', 'precision'],
                       value_vars=['M4 Peak Power', 'M33 Peak Power', 'M7 Peak Power'],
                       var_name='mcu', value_name='peak_power')
peak_long['mcu'] = peak_long['mcu'].str.extract(r'(M4|M33|M7)')

# --- Geometry constants (recomputed now that we know noise count) ---
BAR_WIDTH = 0.09
NUM_NOISE = len(noise_levels)
NUM_PREC  = len(precision_order)
BARS_PER_GROUP = NUM_NOISE * NUM_PREC
TOTAL_GROUP_SPAN = BAR_WIDTH * BARS_PER_GROUP
HALF_SPAN = TOTAL_GROUP_SPAN / 2
# Gaps further tightened between major kernel groups
GAP_VARIANT = TOTAL_GROUP_SPAN * 1.05   # within same solver (N-variants)
GAP_MINIMAL = TOTAL_GROUP_SPAN * 1.3    # between standalone minimal solvers
GAP_SOLVER  = TOTAL_GROUP_SPAN * 1.5    # between 8pt family and up3pt family

# Recompute x-centre positions ------------------------------------------------
centres = []
cur_x = 0.0
for idx, (solver, N) in enumerate(solver_order):
    centres.append(cur_x)
    if idx == len(solver_order) - 1:
        break
    next_solver = solver_order[idx + 1][0]
    if solver in ['5pt', 'u3pt', 'up2pt']:
        cur_x += GAP_MINIMAL
    elif solver == '8pt' and next_solver == '8pt':
        cur_x += GAP_VARIANT
    elif solver == '8pt' and next_solver != '8pt':
        cur_x += GAP_SOLVER
    elif solver == 'up3pt' and next_solver == 'up3pt':
        cur_x += GAP_VARIANT
    else:
        cur_x += GAP_MINIMAL
bar_pos = np.array(centres)

# --- Colors and hatches ---
# Use Set1 for noise (rotation error), Set2 for MCUs (cycles/power)
# (cb_noise_colors defined after noise_levels)
cb_colors = [to_hex(c) for c in cm.get_cmap('Set2').colors[:len(mcu_order)]]  # M4, M33, M7
hatch_map = {'float': '', 'double': '////'}

# --- Figure setup ---
# Global font settings for the paper (larger & bold)
mpl.rcParams.update({
    'font.size': 26,
    'axes.titlesize': 28,
    'axes.labelsize': 26,
    'xtick.labelsize': 24,
    'ytick.labelsize': 24,
    'legend.fontsize': 24,
    'font.weight': 'bold',
    'axes.labelweight': 'bold',
    'axes.titleweight': 'bold',
})
fig, axes = plt.subplots(3, 1, figsize=(20, 12), sharex=True)

# --- Panel 1: Rotation error (all noise levels, color) ---
ax = axes[0]
width = BAR_WIDTH  # use unified width constant
n_noise = len(noise_levels)
n_prec = len(precision_order)
n_groups = len(group_labels)
for i, noise in enumerate(noise_levels):
    for j, precision in enumerate(precision_order):
        for idx, (solver, N) in enumerate(solver_order):
            row = df_rot[(df_rot['solver'] == solver) & (df_rot['N'] == int(N)) &
                         (df_rot['noise_level'] == noise) & (df_rot['precision'] == precision)]
            if not row.empty:
                val = row['mean_rot_error_deg'].values[0]
                if not np.isnan(val):
                    k = i * n_prec + j  # flattened bar index within the group (0..5)
                    pos = bar_pos[idx] + (k - (n_noise * n_prec - 1) / 2) * width
                    hatch = hatch_map[precision]
                    color = cb_noise_colors[i]
                    ax.bar(pos, val, width=width, color=color, edgecolor='black',
                           hatch=hatch, alpha=0.9, label=None)
ax.set_ylabel('Rot. Error (deg)', fontsize=26, fontweight='bold')
ax.set_xticks(bar_pos)
ax.set_xticklabels(group_labels, rotation=45, ha='right', fontdict={'fontsize': 24, 'fontweight': 'bold'})
ax.yaxis.grid(True, linestyle='--', which='major', color='gray', alpha=0.5)
ax.set_axisbelow(True)
# Noise legend inside panel 1 (top left)
noise_patches = [Patch(facecolor=cb_noise_colors[i], edgecolor='black', label=f'{noise_levels[i]}') for i in range(len(noise_levels))]
ax.legend(handles=noise_patches, title='Noise (pix)', loc='upper left', frameon=True, fontsize=15, title_fontsize=16)

# --- Panel 2: Cycles (log scale, grouped bars at bar_pos) ---
ax = axes[1]
bar_width = BAR_WIDTH
n_mcu = len(mcu_order)
n_prec = len(precision_order)
BARS_CYC = n_mcu * n_prec
for idx, (solver, N) in enumerate(solver_order):
    center = bar_pos[idx]
    for m_idx, mcu in enumerate(mcu_order):
        for j, precision in enumerate(precision_order):
            row = cycles_long[(cycles_long['solver'] == solver) & (cycles_long['N'] == int(N)) &
                             (cycles_long['mcu'] == mcu) & (cycles_long['precision'] == precision)]
            if not row.empty:
                val = row['cycles'].values[0]
                k = m_idx * n_prec + j  # flattened bar index (0..5)
                x = center + (k - (BARS_CYC - 1) / 2) * bar_width
                hatch = hatch_map[precision]
                ax.bar(x, val, width=bar_width, color=cb_colors[m_idx], edgecolor='black',
                       hatch=hatch, alpha=0.9, label=None)
ax.set_ylabel('Cycles', fontsize=26, fontweight='bold')
ax.set_yscale('log')
ax.set_xticks(bar_pos)
ax.set_xticklabels(group_labels, rotation=45, ha='right', fontdict={'fontsize': 24, 'fontweight': 'bold'})
ax.tick_params(axis='x', labelbottom=True)  # ensure x-ticks visible
ax.yaxis.grid(True, linestyle='--', which='major', color='gray', alpha=0.5)
ax.set_axisbelow(True)

# --- Panel 3: Peak Power (grouped bars at bar_pos) ---
ax = axes[2]
for idx, (solver, N) in enumerate(solver_order):
    center = bar_pos[idx]
    for m_idx, mcu in enumerate(mcu_order):
        for j, precision in enumerate(precision_order):
            row = peak_long[(peak_long['solver'] == solver) & (peak_long['N'] == int(N)) &
                            (peak_long['mcu'] == mcu) & (peak_long['precision'] == precision)]
            if not row.empty:
                val = row['peak_power'].values[0]
                k = m_idx * n_prec + j  # 0..5
                x = center + (k - (BARS_CYC - 1) / 2) * bar_width
                hatch = hatch_map[precision]
                ax.bar(x, val, width=bar_width, color=cb_colors[m_idx], edgecolor='black',
                       hatch=hatch, alpha=0.9, label=None)
ax.set_ylabel('Peak Power (mW)', fontsize=26, fontweight='bold')
ax.set_xticks(bar_pos)
ax.set_xticklabels(group_labels, rotation=45, ha='right', fontdict={'fontsize': 24, 'fontweight': 'bold'})
ax.yaxis.grid(True, linestyle='--', which='major', color='gray', alpha=0.5)
ax.set_axisbelow(True)

# ensure x-ticks visible on the top panel too
axes[0].tick_params(axis='x', labelbottom=True)

# --- Legends ---
float_patch = Patch(facecolor='white', edgecolor='black', hatch='', label='float')
double_patch = Patch(facecolor='white', edgecolor='black', hatch='////', label='double')
mcu_patches = [Patch(facecolor=cb_colors[i], edgecolor='black', label=mcu_order[i]) for i in range(3)]

fig.legend(handles=[float_patch, double_patch] + mcu_patches, loc='lower center', ncol=5, frameon=False,
           fontsize=24, prop={'weight': 'bold'}, bbox_to_anchor=(0.5, -0.05))

plt.tight_layout(rect=[0, 0, 1, 0.95])
plt.savefig(f'{plot_dir}/min_solver_suite.pdf', bbox_inches='tight')
print(f"Saved plot → {plot_dir}/min_solver_suite.pdf") 