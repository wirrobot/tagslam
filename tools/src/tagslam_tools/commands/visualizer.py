"""Visualizer command."""

from __future__ import annotations

import logging
from typing import Annotated

import typer

from tagslam_tools.utils import console, heading, section
from tagslam_tools.visualizer import run_visualizer

logger = logging.getLogger(__name__)

visualizer_app = typer.Typer(help="SLAM visualization operations")


@visualizer_app.command()
def visualize(
    image_topic: Annotated[
        str,
        typer.Option("-i", "--image", help="Camera image topic"),
    ] = "camera/image_raw",
    odom_topic: Annotated[
        str,
        typer.Option("-o", "--odom", help="SLAM odometry topic"),
    ] = "/odom/body_rig",
) -> None:
    """Show camera feed with overlaid SLAM pose (requires ROS2 env)."""
    heading("Visualizer — live pose overlay")
    section()
    console.print(f"  Image topic : {image_topic}", style="item")
    console.print(f"  Odom topic  : {odom_topic}", style="item")
    console.print("  Press Q or Esc in window to quit", style="dim")
    section()
    logger.info("Starting visualizer: image=%s odom=%s", image_topic, odom_topic)
    run_visualizer(image_topic, odom_topic)
