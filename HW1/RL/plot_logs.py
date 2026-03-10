"""
Plot training metrics from all .csv files in the RL/logs/ directory.
Each CSV is a separate training session, plotted together for comparison.

Usage:
    python plot_logs.py
    python plot_logs.py --log_dir logs/       # custom log directory
    python plot_logs.py --smooth 10           # rolling average window size
"""

import argparse
import glob
import os
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np


def load_csvs(log_dir):
    pattern = os.path.join(log_dir, "*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"No .csv files found in '{log_dir}'")
        return []
    sessions = []
    for f in files:
        try:
            df = pd.read_csv(f)
            # Use just the filename stem as the session label
            label = os.path.splitext(os.path.basename(f))[0]
            sessions.append((label, df))
            print(f"  Loaded: {os.path.basename(f)} ({len(df)} rows)")
        except Exception as e:
            print(f"  Could not load {f}: {e}")
    return sessions


def smooth(series, window):
    """Rolling mean with min_periods=1 so the start is not NaN."""
    return series.rolling(window=window, min_periods=1).mean()


def plot_sessions(sessions, smooth_window=5, save_path="logs/training_comparison.png"):

    metrics = [
        ("Avg_Reward",    "Average Training Reward"),
        ("Actor_Loss",    "Actor Loss"),
        ("Critic_Loss",   "Critic Loss"),
    ]
    eval_col = "Eval_Reward"

    # Color palette — one color per session
    colors = cm.tab10(np.linspace(0, 1, max(len(sessions), 1)))

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    axes = axes.flatten()
    fig.suptitle("PPO Training Session Comparison", fontsize=16, fontweight="bold")

    for ax_idx, (col, title) in enumerate(metrics):
        ax = axes[ax_idx]
        for (label, df), color in zip(sessions, colors):
            if col not in df.columns:
                continue
            x = df["Timestep"]
            y = smooth(df[col], smooth_window)
            ax.plot(x, y, label=label, color=color, linewidth=1.5)
            # Faint raw values in background
            ax.plot(x, df[col], color=color, alpha=0.15, linewidth=0.8)
        ax.set_title(title)
        ax.set_xlabel("Timestep")
        ax.set_ylabel(col)
        ax.legend(fontsize=7, loc="best")
        ax.grid(True, alpha=0.3)

    # 4th panel: Evaluation Reward (only rows where Eval_Reward is present)
    ax = axes[3]
    for (label, df), color in zip(sessions, colors):
        if eval_col not in df.columns:
            continue
        eval_df = df[df[eval_col].notna() & (df[eval_col] != "")]
        if eval_df.empty:
            continue
        x = eval_df["Timestep"]
        y = pd.to_numeric(eval_df[eval_col], errors="coerce")
        ax.plot(x, y, marker="o", markersize=4, label=label, color=color, linewidth=1.5)
    ax.set_title("Periodic Evaluation Reward")
    ax.set_xlabel("Timestep")
    ax.set_ylabel("Eval Reward")
    ax.legend(fontsize=7, loc="best")
    ax.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    os.makedirs(os.path.dirname(save_path) if os.path.dirname(save_path) else ".", exist_ok=True)
    plt.savefig(save_path, dpi=150)
    print(f"\nSaved plot to: {save_path}")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot RL training logs.")
    parser.add_argument("--log_dir", type=str, default="logs",
                        help="Directory containing .csv log files (default: logs/)")
    parser.add_argument("--smooth", type=int, default=5,
                        help="Rolling average window size (default: 5)")
    parser.add_argument("--out", type=str, default="logs/training_comparison.png",
                        help="Output image path (default: logs/training_comparison.png)")
    args = parser.parse_args()

    print(f"Loading sessions from '{args.log_dir}/'...")
    sessions = load_csvs(args.log_dir)
    if sessions:
        plot_sessions(sessions, smooth_window=args.smooth, save_path=args.out)
