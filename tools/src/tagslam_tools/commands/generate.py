"""Generate AprilTag markers on A4 PDF."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

import typer

from tagslam_tools.generate_tags import generate
from tagslam_tools.utils import console, heading, section

generate_app = typer.Typer(help="Generate AprilTag marker sheets")


@generate_app.command()
def sheet(
    out: Annotated[
        str,
        typer.Option("-o", "--out", help="Output PDF file path"),
    ] = "apriltags_a4.pdf",
) -> None:
    """Generate an A4 PDF with two AprilTags (large + small, vertically centered)."""
    heading("Generate — AprilTag sheet")
    section()
    result = generate(out)
    section()
    console.print(f"  Output : {Path(result).absolute()}", style="item")
