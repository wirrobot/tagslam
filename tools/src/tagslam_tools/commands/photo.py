"""Photo commands — point cloud photo capture."""

from __future__ import annotations

import logging
from typing import Annotated

import typer

from tagslam_tools.photo import interactive_photo_capture
from tagslam_tools.utils import console, heading, section

logger = logging.getLogger(__name__)

photo_app = typer.Typer(help="Point cloud photo capture operations")


@photo_app.command()
def capture(
    save_dir: Annotated[
        str,
        typer.Option("-d", "--dir", help="Directory to save photos"),
    ] = "data/point",
    device: Annotated[
        int,
        typer.Option("--device", help="Camera device ID (e.g. 0 → /dev/video0)"),
    ] = 0,
) -> None:
    """Open camera preview.  SPACE to save photo, ESC/Q to quit."""
    heading("Photo — capture")
    section()
    console.print(f"  Device : /dev/video{device}", style="item")
    logger.info("Starting photo capture, saving to %s (device=%d)", save_dir, device)
    count = interactive_photo_capture(save_dir, device=device)
    section()
    console.print(f"  Captured {count} photos to {save_dir}/", style="item")
    logger.info("Captured %d photos", count)
