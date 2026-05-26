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
        section()
        raise typer.Exit(1)

    section()
    logger.info(
        "Pose analysis: %d pairs, total=%.4f m, mean=%.4f m",
        result["num_pairs"],
        result["total"],
        result["mean"],
    )
    console.print("  Press Enter to exit...", style="dim")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
