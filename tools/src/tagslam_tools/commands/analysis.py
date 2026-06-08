"""Pose analysis command — compute distances from log files and point-cloud loss."""

from __future__ import annotations

import logging
from typing import Annotated

import typer

from tagslam_tools.point_analysis import run_point_analysis
from tagslam_tools.pose_analysis import analyze_pose_log
from tagslam_tools.utils import console, heading, section

logger = logging.getLogger(__name__)

analysis_app = typer.Typer(help="Pose log and point-cloud analysis")


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


@analysis_app.command()
def point_loss(
    photo_dir: Annotated[
        str,
        typer.Option("-p", "--photo-dir", help="Directory of point photos"),
    ] = "data/point",
    video_dir: Annotated[
        str,
        typer.Option("-v", "--video-dir", help="Directory of videos"),
    ] = "data/video",
    output_dir: Annotated[
        str,
        typer.Option("-o", "--output-dir", help="Directory for output CSV"),
    ] = "data/loss",
) -> None:
    """Analyze point-cloud photos vs video frames through TagSLAM, export loss CSV."""
    heading("Analyze — point-cloud loss")
    section()
    console.print(f"  Photo dir : {photo_dir}", style="item")
    console.print(f"  Video dir : {video_dir}", style="item")
    console.print(f"  Output    : {output_dir}/point_loss.csv", style="item")
    section()
    logger.info("Starting point-cloud loss analysis")
    run_point_analysis(
        photo_dir=photo_dir,
        video_dir=video_dir,
        output_dir=output_dir,
    )
    section()
    console.print(f"  Results saved to {output_dir}/point_loss.csv", style="success")
