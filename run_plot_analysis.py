#!/usr/bin/env python3
"""Pose log analysis plot — distance distribution, cumulative, and trajectory."""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

sns.set_theme(style="whitegrid", palette="muted", context="notebook")
matplotlib.rcParams.update({
    "figure.dpi": 150,
    "font.size": 11,
    "axes.titlesize": 13,
    "axes.labelsize": 11,
})

STD_DISTANCE = 0.10  # standard step distance in meters


def parse_log(filepath: str) -> np.ndarray:
    """Parse pose log into (N, 4) array: [timestamp, x, y, z]."""
    entries = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.replace("x=", " ").replace("y=", " ").replace("z=", " ").split()
            if len(parts) >= 4:
                try:
                    entries.append([float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])])
                except ValueError:
                    continue
    return np.array(entries)


def compute_distances(poses: np.ndarray) -> np.ndarray:
    """Euclidean distance between consecutive poses."""
    diffs = np.diff(poses[:, 1:4], axis=0)
    return np.sqrt(np.sum(diffs**2, axis=1))


def plot_analysis(
    multi_path: str = "pose_log_multi.txt",
    single_path: str = "pose_log_single.txt",
    out_path: str | None = None,
) -> None:
    """Generate analysis plot."""
    have_multi = Path(multi_path).is_file()
    have_single = Path(single_path).is_file()

    fig = plt.figure(figsize=(14, 8))
    fig.suptitle("TagSLAM Pose Log Analysis", fontsize=16, fontweight="bold", y=0.98)

    # ── Panel 1: step distances ──
    ax1 = fig.add_subplot(2, 3, (1, 2))
    colors = sns.color_palette("Set2", 2)

    max_pairs = 0
    offset = 0.0

    if have_multi:
        m = parse_log(multi_path)
        if len(m) >= 2:
            d_multi = compute_distances(m)
            x = np.arange(1, len(d_multi) + 1)
            bars = ax1.bar(x - 0.15, d_multi, width=0.3, color=colors[0], alpha=0.85, label="Multi-tag (odom)")
            ax1.bar_label(bars, fmt="%.3f", fontsize=7, rotation=90, padding=2)
            max_pairs = max(max_pairs, len(d_multi))

    if have_single:
        s = parse_log(single_path)
        if len(s) >= 2:
            d_single = compute_distances(s)
            x = np.arange(1, len(d_single) + 1)
            bars = ax1.bar(x + 0.15, d_single, width=0.3, color=colors[1], alpha=0.85, label="Single-tag (Tag0 PnP)")
            ax1.bar_label(bars, fmt="%.3f", fontsize=7, rotation=90, padding=2)
            max_pairs = max(max_pairs, len(d_single))

    ax1.axhline(y=STD_DISTANCE, color="red", linestyle="--", linewidth=1.5, alpha=0.7, label=f"Standard ({STD_DISTANCE*100:.0f} cm)")
    ax1.set_xlabel("Step")
    ax1.set_ylabel("Distance (m)")
    ax1.set_title("Step Distance vs. Standard")
    ax1.legend(loc="upper right", fontsize=9)
    ax1.set_xticks(np.arange(1, max_pairs + 1, max(1, max_pairs // 10)))

    # ── Panel 2: cumulative distance ──
    ax2 = fig.add_subplot(2, 3, 3)

    if have_multi and len(m) >= 2:
        cumul_multi = np.cumsum(compute_distances(m))
        ax2.plot(np.arange(1, len(cumul_multi) + 1), cumul_multi,
                 "o-", color=colors[0], markersize=5, linewidth=2,
                 label=f"Multi-tag (total: {cumul_multi[-1]:.3f} m)")

    if have_single and len(s) >= 2:
        cumul_single = np.cumsum(compute_distances(s))
        ax2.plot(np.arange(1, len(cumul_single) + 1), cumul_single,
                 "s--", color=colors[1], markersize=5, linewidth=2,
                 label=f"Single-tag (total: {cumul_single[-1]:.3f} m)")

    ax2.set_xlabel("Step")
    ax2.set_ylabel("Cumulative Distance (m)")
    ax2.set_title("Cumulative Distance")
    ax2.legend(loc="upper left", fontsize=9)

    # ── Panel 3: trajectory (XY) ──
    ax3 = fig.add_subplot(2, 3, 4)

    if have_multi and len(m) >= 2:
        ax3.plot(m[:, 1], m[:, 2], "o-", color=colors[0], markersize=4, linewidth=1.5, label="Multi-tag")
        ax3.scatter(m[0, 1], m[0, 2], c="green", s=80, zorder=5, marker="s", label="Start", edgecolors="black", linewidths=0.5)
        ax3.scatter(m[-1, 1], m[-1, 2], c="red", s=80, zorder=5, marker="X", label="End", edgecolors="black", linewidths=0.5)

    if have_single and len(s) >= 2:
        ax3.plot(s[:, 1], s[:, 2], "s--", color=colors[1], markersize=3, linewidth=1.5, label="Single-tag")
        ax3.scatter(s[0, 1], s[0, 2], c="green", s=60, zorder=5, marker="s")
        ax3.scatter(s[-1, 1], s[-1, 2], c="red", s=60, zorder=5, marker="X")

    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_title("Trajectory (XY plane)")
    ax3.legend(loc="best", fontsize=8)
    ax3.axis("equal")

    # ── Panel 4: error vs standard ──
    ax4 = fig.add_subplot(2, 3, 5)

    if have_multi and len(m) >= 2:
        d_m = compute_distances(m)
        err_m = np.abs(d_m - STD_DISTANCE) * 100  # cm
        ax4.bar(np.arange(1, len(err_m) + 1) - 0.15, err_m, width=0.3, color=colors[0], alpha=0.85, label="Multi-tag")
        ax4.axhline(y=np.mean(err_m), color=colors[0], linestyle=":", linewidth=1.2,
                    label=f"Mean: {np.mean(err_m):.2f} cm")

    if have_single and len(s) >= 2:
        d_s = compute_distances(s)
        err_s = np.abs(d_s - STD_DISTANCE) * 100
        ax4.bar(np.arange(1, len(err_s) + 1) + 0.15, err_s, width=0.3, color=colors[1], alpha=0.85, label="Single-tag")
        ax4.axhline(y=np.mean(err_s), color=colors[1], linestyle=":", linewidth=1.2,
                    label=f"Mean: {np.mean(err_s):.2f} cm")

    ax4.set_xlabel("Step")
    ax4.set_ylabel("|Error| (cm)")
    ax4.set_title("Absolute Error vs. Standard 10 cm")
    ax4.legend(loc="upper right", fontsize=8)

    # ── Panel 5: statistics text ──
    ax5 = fig.add_subplot(2, 3, 6)
    ax5.axis("off")

    lines = []
    for label, path, c in [("Multi-tag", multi_path, colors[0]), ("Single-tag", single_path, colors[1])]:
        if Path(path).is_file():
            data = parse_log(path)
            if len(data) >= 2:
                d = compute_distances(data)
                lines.append(f"── {label} ──")
                lines.append(f"  Pairs       : {len(d)}")
                lines.append(f"  Mean dist.  : {np.mean(d):.4f} m")
                lines.append(f"  Std dev.    : {np.std(d):.4f} m")
                lines.append(f"  Min / Max   : {np.min(d):.4f} / {np.max(d):.4f} m")
                lines.append(f"  Total       : {np.sum(d):.4f} m")
                lines.append(f"  Mean error  : {np.mean(np.abs(d - STD_DISTANCE))*100:.2f} cm")
                lines.append("")

    for i, line in enumerate(lines):
        y = 0.95 - i * 0.045
        ax5.text(0.05, y, line, transform=ax5.transAxes, fontfamily="monospace", fontsize=10,
                 verticalalignment="top", color="dimgray")

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    if out_path:
        plt.savefig(out_path, dpi=200, bbox_inches="tight")
        print(f"Saved: {out_path}")
    else:
        plt.show()


def main() -> None:
    multi_path = sys.argv[1] if len(sys.argv) > 1 else "pose_log_multi.txt"
    single_path = sys.argv[2] if len(sys.argv) > 2 else "pose_log_single.txt"
    out_path = sys.argv[3] if len(sys.argv) > 3 else None
    plot_analysis(multi_path, single_path, out_path)


if __name__ == "__main__":
    main()
