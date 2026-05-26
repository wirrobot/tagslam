"""Camera commands — capture and publish."""

from __future__ import annotations

import logging
from typing import Annotated

import typer

from tagslam_tools.camera import interactive_capture, publish_camera_loop
from tagslam_tools.utils import console, heading, section

logger = logging.getLogger(__name__)

camera_app = typer.Typer(help="Camera capture and publishing operations")


@camera_app.command()
def capture(
    save_dir: Annotated[
        str,
        typer.Option("-d", "--dir", help="Directory to save captured images"),
    ] = "pic",
) -> None:
    """Open camera preview.  SPACE to save frame, ESC/Q to quit."""
    heading("Camera — capture")
    section()
    logger.info("Starting interactive capture, saving to %s", save_dir)
    count = interactive_capture(save_dir)
    section()
    console.print(f"  Captured {count} images to {save_dir}/", style="item")
    logger.info("Captured %d images", count)


@camera_app.command()
def publish(
    topic: Annotated[
        str,
        typer.Option("-t", "--topic", help="ROS2 image topic to publish to"),
    ] = "camera/image_raw",
) -> None:
    """Publish camera frames to a ROS2 topic (requires ROS2 env sourced)."""
    heading("Camera — publish")
    section()
    console.print(f"  Topic : {topic}", style="item")
    console.print("  Press Ctrl+C to stop", style="dim")
    section()
    logger.info("Publishing camera to topic: %s", topic)
    publish_camera_loop(topic)
