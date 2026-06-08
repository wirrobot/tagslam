"""Video commands — video recording."""

from __future__ import annotations

import logging
from typing import Annotated

import typer

from tagslam_tools.utils import console, heading, section
from tagslam_tools.video import interactive_video_record

logger = logging.getLogger(__name__)

video_app = typer.Typer(help="Video recording operations")


@video_app.command()
def record(
    save_dir: Annotated[
        str,
        typer.Option("-d", "--dir", help="Directory to save videos"),
    ] = "data/video",
    fps: Annotated[
        float,
        typer.Option("-f", "--fps", help="Recording frames per second"),
    ] = 30.0,
) -> None:
    """Open camera preview.  SPACE to start/stop recording, ESC/Q to quit."""
    heading("Video — record")
    section()
    console.print(f"  Save dir : {save_dir}", style="item")
    console.print(f"  FPS      : {fps}", style="item")
    section()
    logger.info("Starting video recording, saving to %s @ %.1f fps", save_dir, fps)
    count = interactive_video_record(save_dir, fps)
    section()
    console.print(f"  Recorded {count} videos to {save_dir}/", style="item")
    logger.info("Recorded %d videos", count)
