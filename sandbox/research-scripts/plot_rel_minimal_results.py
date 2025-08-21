import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse
import matplotlib as mpl

parser = argparse.ArgumentParser(description='Plot minimal solver benchmark results.')
parser.add_argument('--csv', type=str, default='benchmark_results.csv', help='Path to input CSV file')
args = parser.parse_args()
df = pd.read_csv(args.csv)

# Exclude zero noise
df = df[df['noise_level'] != 0]

# Define solver/N order
solver_order = [
    ('5pt', 5), ('u3pt', 3), ('up2pt', 2),
    ('8pt', 8), ('8pt', 16), ('8pt', 32), ('8pt', 64),
    ('up3pt', 3), ('up3pt', 8), ('up3pt', 16), ('up3pt', 32)
]
precision_order = ['float', 'double']
noise_levels = sorted(df['noise_level'].unique())

# Global plotting style tweaks for publication
mpl.rcParams.update({
    'font.weight': 'bold',
    'axes.labelweight': 'bold',
    'axes.titleweight': 'bold',
})

def make_xtick_label(solver, N):
    if solver in ['5pt', 'u3pt', 'up2pt']:
        return solver
    else:
        return f'{solver}-{N}'

group_labels = [make_xtick_label(solver, N) for solver, N in solver_order]

# Custom x positions for group labels
x = []
xpos = 0
for solver, N in solver_order:
    x.append(xpos)
    if solver in ['5pt', 'u3pt', 'up2pt']:
        xpos += 1.2  # larger gap after minimal solvers
    elif solver == '8pt':
        xpos += 0.7  # smaller gap between 8pt-N
    elif solver == 'up3pt':
        xpos += 0.7  # smaller gap between up3pt-N
x = np.array(x)

# Output directory
plot_dir = 'benchmark_results/plots'
os.makedirs(plot_dir, exist_ok=True)

# Use colorblind-friendly colors
from matplotlib import cm
from matplotlib.colors import to_hex
cb_colors = [to_hex(c) for c in cm.get_cmap('Set2').colors[:len(noise_levels)]]

def plot_rotation_error():
    fig, ax = plt.subplots(figsize=(12, 6))
    width = 0.16
    n_noise = len(noise_levels)
    n_prec = len(precision_order)
    n_groups = len(group_labels)

    for i, noise in enumerate(noise_levels):
        for j, precision in enumerate(precision_order):
            bar_vals = []
            for idx, (solver, N) in enumerate(solver_order):
                row = df[(df['solver'] == solver) & (df['N'] == int(N)) &
                         (df['noise_level'] == noise) & (df['precision'] == precision)]
                if not row.empty:
                    bar_vals.append(row['mean_rot_error_deg'].values[0])
                else:
                    bar_vals.append(np.nan)
            bar_pos = x + (i - n_noise/2)*n_prec*width + (j - n_prec/2)*width + width/2
            hatch = '' if precision == 'float' else '////'
            color = cb_colors[i]
            ax.bar(bar_pos, bar_vals, width=width, color=color, edgecolor='black',
                   hatch=hatch, alpha=0.9, label=None)

    ax.set_xticks(x)
    ax.set_xticklabels(group_labels, rotation=45, ha='right', fontdict={'fontsize': 18, 'fontweight': 'bold'})
    ax.set_ylabel('Mean Rotation Error (deg)', fontsize=20, fontweight='bold')
    # No plot title

    # Add horizontal grid lines for y-ticks
    ax.yaxis.grid(True, linestyle='--', which='major', color='gray', alpha=0.5)
    ax.set_axisbelow(True)
    ax.tick_params(axis='y', labelsize=18, width=1.5)

    # Explicitly bold all tick labels (sometimes PDF rendering ignores set_xticklabels kwargs)
    for tick in ax.get_xticklabels():
        tick.set_fontweight('bold')
    for tick in ax.get_yticklabels():
        tick.set_fontweight('bold')

    # Custom legend: noise (color), float/double (hatch)
    noise_patches = [plt.Rectangle((0,0),1,1, color=cb_colors[i], edgecolor='black') for i in range(n_noise)]
    noise_labels = [f'{noise}' for noise in noise_levels]
    float_patch = plt.Rectangle((0,0),1,1, facecolor='white', edgecolor='black', hatch='', label='float')
    double_patch = plt.Rectangle((0,0),1,1, facecolor='white', edgecolor='black', hatch='////', label='double')

    legend1 = ax.legend(noise_patches, noise_labels, title='Noise (pix)', loc='upper right', frameon=True, fontsize=17, title_fontsize=18)
    ax.add_artist(legend1)
    ax.legend([float_patch, double_patch], ['float', 'double'], title='Precision', loc='upper left', frameon=True, fontsize=17, title_fontsize=18)
    plt.tight_layout()
    plt.savefig(f'{plot_dir}/rot_error_grouped.pdf', dpi=200, bbox_inches='tight')
    plt.close()

plot_rotation_error()
print(f"Rotation error plot saved to {plot_dir}/rot_error_grouped.pdf") 