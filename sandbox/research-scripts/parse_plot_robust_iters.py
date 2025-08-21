#!/usr/bin/env python3
"""Parse robust‐relative‐pose RANSAC logs and plot iteration statistics.

The script expects the log files produced by `scripts/run_robust_relpose.sh`
under `logs/robust-relpose/` (one file per solver/precision).

It produces
  • CSV  : logs/robust-relpose/iteration_stats.csv
  • Plot : benchmark_results/plots/iter_counts_grouped.png

Usage (run from repo root):
  python tools/parse_plot_robust_iters.py           # uses defaults
  python tools/parse_plot_robust_iters.py --help    # options
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path
from typing import List

try:
    import pandas as pd  # type: ignore
    import matplotlib.pyplot as plt  # type: ignore
    import seaborn as sns  # type: ignore
    import matplotlib as mpl
except ModuleNotFoundError as e:
    print("ERROR: Missing required Python packages. Install with:\n"
          "   pip install pandas matplotlib seaborn", file=sys.stderr)
    raise

# -------------------------------------------------------------
# Regex patterns for the stdout blocks we need to capture
# -------------------------------------------------------------
PAT_HEAD = re.compile(r"Testing (\S+) robust solver.*")
PAT_ITER = re.compile(
    r"Iteration Statistics .*?\n\s*Mean:\s*([0-9.]+).*?\n"  # mean
    r"\s*Std Dev:\s*([0-9.]+).*?\n"                          # std
    r"\s*Min:\s*([0-9]+).*?\n"                               # min
    r"\s*Max:\s*([0-9]+).*?\n"                               # max
    r"\s*Median:\s*([0-9.]+)",                               # median
    re.S)
PAT_SUCCESS = re.compile(r"Success rate:\s*([0-9.]+)%")

DEFAULT_ORDER = [
    "5pt",
    "upright_3pt",
    "upright_planar_2pt",
    "8pt",
    "upright_planar_3pt",
]

# -------------------------------------------------------------
# Helper functions
# -------------------------------------------------------------

def parse_log(path: Path) -> dict | None:
    """Extract statistics from one log file."""
    text = path.read_text(errors="replace")
    m_head = PAT_HEAD.search(text)
    m_iter = PAT_ITER.search(text)
    m_succ = PAT_SUCCESS.search(text)
    if not (m_head and m_iter and m_succ):
        print(f"[WARN] could not parse {path.name}", file=sys.stderr)
        return None

    solver = m_head.group(1)
    mean, std, minv, maxv, median = m_iter.groups()
    success = m_succ.group(1)

    precision = "double" if "_double" in path.stem else "float"

    return {
        "solver": solver,
        "precision": precision,
        "mean_iters": float(mean),
        "std_iters": float(std),
        "min_iters": int(minv),
        "max_iters": int(maxv),
        "median_iters": float(median),
        "success_rate": float(success),
    }

# -------------------------------------------------------------
# Main
# -------------------------------------------------------------

def main(argv: List[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Parse robust‐relpose logs and plot iteration statistics.")
    ap.add_argument("--log-dir", default="logs/robust-relpose", help="Directory containing .log files")
    ap.add_argument("--csv-out", default=None, help="CSV output path (default: <log-dir>/iteration_stats.csv)")
    ap.add_argument("--cycles-csv", default="benchmark_results/rel-pose-mcu/cs4-rel-loransac.csv",
                    help="CSV containing MCU cycle & power measurements (solver,Scalar,cycles,power)")
    ap.add_argument("--plot-out", default="benchmark_results/plots/iter_cycles_power_grouped.png",
                    help="Path to save the multi-panel PNG")
    args = ap.parse_args(argv)

    log_dir = Path(args.log_dir)
    if not log_dir.is_dir():
        ap.error(f"Log directory '{log_dir}' does not exist")

    log_files = sorted(log_dir.glob("*.log"))
    if not log_files:
        ap.error(f"No .log files found in {log_dir}")

    rows = []
    for lf in log_files:
        row = parse_log(lf)
        if row:
            rows.append(row)

    if not rows:
        sys.exit("No rows parsed – aborting")

    # DataFrame for convenience
    df = pd.DataFrame(rows)

    # CSV output
    csv_path = Path(args.csv_out) if args.csv_out else (log_dir / "iteration_stats.csv")
    df.to_csv(csv_path, index=False)
    print(f"Saved CSV with {len(df)} rows to {csv_path}")

    # -------------------------------------------------------------
    # Load MCU latency / power CSV and merge
    # -------------------------------------------------------------
    mcu_path = Path(args.cycles_csv)
    if not mcu_path.is_file():
        ap.error(f"Cycles/power CSV not found: {mcu_path}")

    mcu_df_raw = pd.read_csv(mcu_path)
    # Normalise column names
    mcu_df = mcu_df_raw.rename(columns={"Scalar": "precision"})

    # Normalise solver names to match iteration logs
    name_map = {
        "u3pt": "upright_3pt",
        "up3pt": "upright_planar_3pt",
        "up2pt": "upright_planar_2pt",
    }

    solver_col = mcu_df["Solver"].astype(str)
    # Collapse variants like "8pt-8" → "8pt"
    solver_norm = solver_col.str.replace(r"8pt-\d+", "8pt", regex=True)
    solver_norm = solver_norm.replace(name_map)
    mcu_df.insert(0, "solver", solver_norm)

    # For merging with iteration stats, use only M7 columns
    if "M7 Cycles" not in mcu_df.columns or "M7 Peak Power" not in mcu_df.columns:
        ap.error("Expected 'M7 Cycles' and 'M7 Peak Power' columns in cycles CSV")

    mcu_merge = mcu_df[["solver", "precision", "M7 Cycles", "M7 Peak Power"]].copy()
    mcu_merge.columns = ["solver", "precision", "cycles", "peak_power"]

    # Merge with iteration stats to ensure consistent ordering / presence
    merged = pd.merge(df, mcu_merge, on=["solver", "precision"], how="inner")
    if merged.empty:
        ap.error("No matching rows between iteration logs and cycles CSV")

    # -------------------------------------------------------------
    # Plotting – 1×3 subplots (special handling for 8pt)
    # -------------------------------------------------------------
    sns.set(style="whitegrid", font_scale=2.0)
    # Ensure larger default matplotlib sizes and bold weight
    mpl.rcParams.update({
        'font.size': 22,
        'axes.titlesize': 24,
        'axes.labelsize': 22,
        'xtick.labelsize': 20,
        'ytick.labelsize': 20,
        'legend.fontsize': 20,
        'font.weight': 'bold',
        'axes.labelweight': 'bold',
        'axes.titleweight': 'bold',
    })
    fig, axes = plt.subplots(1, 3, figsize=(20, 7), sharex=False)

    hue_order_custom = [p for p in ["float", "double"] if p in merged.precision.unique()]
    mcu_order = ["M4", "M33", "M7"]
    display_order = ["8pt", "5pt", "upright_3pt", "upright_planar_3pt", "upright_planar_2pt"]
    label_map = {"upright_3pt": "u3pt", "upright_planar_3pt": "up3pt", "upright_planar_2pt": "up2pt"}

    # --- Panel 1: Mean RANSAC iters (all solvers, including 8pt) ---
    iter_df = df.copy()
    iter_df["solver"] = iter_df["solver"].replace(label_map)
    iter_df["solver"] = pd.Categorical(iter_df["solver"], categories=[label_map.get(s, s) for s in display_order], ordered=True)
    ax = axes[0]
    sns.barplot(
        data=iter_df,
        x="solver",
        y="mean_iters",
        hue="precision",
        order=[label_map.get(s, s) for s in display_order],
        hue_order=hue_order_custom,
        palette="pastel",
        edgecolor="black",
        ax=ax,
    )
    ax.set_ylabel("Mean RANSAC iters", fontweight="bold")
    ax.set_xlabel("")
    ax.set_xticklabels([label_map.get(s, s) for s in display_order], rotation=0)
    ax.set_title("RANSAC Iterations", fontweight="bold")

    # --- Panel 2/3: Cycles and Peak Power (exclude 8pt), color=MCU, hatch=precision ---
    mcu_long = mcu_df.copy()
    mcu_long = mcu_long[~mcu_long["solver"].str.startswith("8pt")].copy()
    mcu_long = mcu_long.melt(
        id_vars=["solver", "precision"],
        value_vars=["M4 Cycles", "M33 Cycles", "M7 Cycles", "M4 Peak Power", "M33 Peak Power", "M7 Peak Power"],
        var_name="metric",
        value_name="value"
    )
    # Split metric into MCU and type
    mcu_long["mcu"] = mcu_long["metric"].str.extract(r"(M4|M33|M7)")
    mcu_long["type"] = mcu_long["metric"].str.contains("Cycles").map({True: "cycles", False: "peak_power"})
    mcu_long = mcu_long.drop(columns=["metric"])

    # Only keep solvers in display_order (except 8pt)
    mcu_long = mcu_long[mcu_long["solver"].isin(display_order[1:])]
    mcu_long["solver"] = mcu_long["solver"].replace(label_map)
    mcu_long["solver"] = pd.Categorical(mcu_long["solver"], categories=[label_map.get(s, s) for s in display_order[1:]], ordered=True)
    mcu_long["mcu"] = pd.Categorical(mcu_long["mcu"], categories=mcu_order, ordered=True)

    # Color palette for MCUs
    mcu_palette = {"M4": "#66c2a5", "M33": "#fc8d62", "M7": "#8da0cb"}
    hatch_map = {"float": "", "double": "///"}

    for i, (typ, ylabel, title) in enumerate(zip(["cycles", "peak_power"], ["Cycles", "Peak Power (mW)"], ["MCU Cycles", "MCU Peak Power"])):
        ax = axes[i+1]
        plot_df = mcu_long[mcu_long["type"] == typ]
        # Sort for consistent bar order
        plot_df = plot_df.sort_values(["solver", "mcu", "precision"])
        # Bar positions: for each solver, 3 MCUs × 2 precisions
        solvers = [label_map.get(s, s) for s in display_order[1:]]
        n_mcu = len(mcu_order)
        n_prec = len(hue_order_custom)
        bar_width = 0.8 / (n_mcu * n_prec)
        x_ticks = []
        x_ticklabels = []
        for s_idx, solver in enumerate(solvers):
            for m_idx, mcu in enumerate(mcu_order):
                for p_idx, precision in enumerate(hue_order_custom):
                    x = s_idx + (m_idx - 1) * bar_width * n_prec + (p_idx - 0.5) * bar_width
                    row = plot_df[(plot_df["solver"] == solver) & (plot_df["mcu"] == mcu) & (plot_df["precision"] == precision)]
                    if not row.empty:
                        val = row["value"].values[0]
                        bar = ax.bar(
                            x,
                            val,
                            width=bar_width,
                            color=mcu_palette[mcu],
                            label=f"{mcu} {precision}",
                            hatch=hatch_map[precision],
                            edgecolor="black",
                        )
                    if p_idx == 0:
                        x_ticks.append(x + bar_width / 2)
                        x_ticklabels.append(solver)
        ax.set_xticks(range(len(solvers)))
        ax.set_xticklabels(solvers, rotation=0)
        ax.set_ylabel(ylabel, fontweight="bold")
        ax.set_xlabel("")
        ax.set_title(title, fontweight="bold")
        if typ == "cycles":
            ax.set_yscale("log")

    # Custom legend for MCU (color) and precision (hatch)
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor=mcu_palette[mcu], edgecolor="black", label=mcu) for mcu in mcu_order
    ] + [
        Patch(facecolor="white", edgecolor="black", hatch=hatch_map[prec], label=prec) for prec in hue_order_custom
    ]
    fig.legend(handles=legend_elements, loc="upper center", ncol=5, frameon=False,
               prop={"weight": "bold", "size": 20})
    for ax in axes:
        leg = ax.get_legend()
        if leg is not None:
            leg.remove()

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    # Save as PDF for publication quality
    plot_out = Path(args.plot_out)
    plot_out = plot_out.with_suffix('.pdf')
    plot_out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(plot_out, bbox_inches='tight')
    print(f"Saved plot → {plot_out}")

    # Console summary
    print("\nSummary (mean ± std [min,max]):")
    prec_cat = pd.CategoricalDtype(categories=["float", "double"], ordered=True)
    df_sorted_summary = df.copy()
    df_sorted_summary["precision"] = df_sorted_summary["precision"].astype(prec_cat)
    for _, r in df_sorted_summary.sort_values(["solver", "precision"]).iterrows():
        print(f"  {r.solver:18s} {r.precision:6s}  {r.mean_iters:.1f} ± {r.std_iters:.1f}  [{int(r.min_iters)}, {int(r.max_iters)}]  success={r.success_rate:.1f}%")


if __name__ == "__main__":
    main() 