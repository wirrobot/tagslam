"""Calibration command."""

from __future__ import annotations

import logging
from typing import Annotated

import typer

from tagslam_tools.calibrate import calibrate_from_images
from tagslam_tools.utils import console, heading, section

logger = logging.getLogger(__name__)

calibrate_app = typer.Typer(help="Camera calibration operations")


@calibrate_app.command()
def calibrate(
    image_dir: Annotated[
        str,
        typer.Option("-d", "--dir", help="Directory containing chessboard images"),
    ] = "pic",
    cols: Annotated[int, typer.Option("-c", "--cols", help="Chessboard inner corner columns")] = 11,
    rows: Annotated[int, typer.Option("-r", "--rows", help="Chessboard inner corner rows")] = 8,
    square: Annotated[
        float,
        typer.Option("-s", "--square", help="Chessboard square size in meters"),
    ] = 0.015,
) -> None:
    """Calibrate camera from chessboard images."""
    heading("Calibrate — from images")
    section()
    console.print(f"  Image dir    : {image_dir}", style="item")
    console.print(f"  Chessboard   : {cols} x {rows}  ({square:.4f} m)", style="item")
    section()

    logger.info(
        "Calibrating: dir=%s size=(%d,%d) square=%.4fm",
        image_dir,
        cols,
        rows,
        square,
    )
    result = calibrate_from_images(image_dir, chessboard_size=(cols, rows), square_size=square)
    if result is None:
        section()
        console.print("  Calibration failed.", style="error")
        raise typer.Exit(1)

    section()
    console.print(f"  RMS reprojection : {result['rms']:.4f} px", style="success")
    console.print(
        f"  Used / total     : {result['num_used']} / {result['num_total']}",
        style="item",
    )
    logger.info("Calibration RMS=%.4f", result["rms"])
