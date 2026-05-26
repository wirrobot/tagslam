"""Pose analysis command — compute distances from log file."""

from __future__ import annotations

import logging
from typing import Annotated

import typer

from tagslam_tools.pose_analysis import analyze_pose_log
from tagslam_tools.utils import console, heading, section

logger = logging.getLogger(__name__)

analysis_app = typer.Typer(help="Pose log analysis")


@analysis_app.command()
def distances(
    filepath: Annotated[
        str,
        typer.Option("-f", "--file", help="Path to pose_log.txt"),
    ] = "pose_log.txt",
) -> None:
    """Compute Euclidean distances between consecutive poses in a log file."""
    heading("Analyze — pose distances")
    section()

    result = analyze_pose_log(filepath)
    if result is None:
        console.print("  Need at least 2 entries to compute distances.", style="warn")
        section()
        raise typer.Exit(1)

    console.print(f"  File    : {result['filepath']}", style="item")
    console.print(f"  Entries : {result['num_entries']}", style="item")
    console.print(f"  Pairs   : {result['num_pairs']}", style="item")
    section()

    console.print(f"  {'Pair':<8} {'Distance (m)':<14} {'Cumulative (m)'}", style="heading")
    cumulative = 0.0
    for i, d in enumerate(result["distances"]):
        cumulative += d
        console.print(f"  {i + 1:<8} {d:<14.6f} {cumulative:<.6f}", style="item")

    section()
    console.print(f"  Min   : {result['min']:.6f} m", style="item")
    console.print(f"  Max   : {result['max']:.6f} m", style="item")
    console.print(f"  Mean  : {result['mean']:.6f} m", style="item")
    console.print(f"  Total : {result['total']:.6f} m", style="success")
    section()

    logger.info(
        "Pose analysis: %d pairs, total=%.4f m, mean=%.4f m",
        result["num_pairs"],
        result["total"],
        result["mean"],
    )
