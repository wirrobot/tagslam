"""Pose analysis command — compute distances from log files."""

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
    multi: Annotated[
        str,
        typer.Option("-m", "--multi", help="Multi-tag pose log file"),
    ] = "pose_log_multi.txt",
    single: Annotated[
        str,
        typer.Option("-s", "--single", help="Single-tag pose log file"),
    ] = "pose_log_single.txt",
) -> None:
    """Compute distances for multi-tag and single-tag pose logs side by side."""
    heading("Analyze — pose distances")
    section()

    for label, filepath in [("Multi-tag (odom)", multi), ("Single-tag (Tag0 PnP)", single)]:
        console.print(f"  {label}", style="heading")
        console.print(f"  File: {filepath}", style="item")
        section()
        result = analyze_pose_log(filepath)
        if result is None:
            console.print("  (no data)", style="warn")
        else:
            logger.info(
                "%s: %d pairs, total=%.4f m, mean=%.4f m",
                label,
                result["num_pairs"],
                result["total"],
                result["mean"],
            )
        section()

    console.print("  Press Enter to exit...", style="dim")
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
