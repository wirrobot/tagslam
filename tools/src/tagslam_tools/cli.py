"""TagSLAM Tools CLI — camera, calibration, visualization, launch."""

from __future__ import annotations

import logging
from typing import Annotated

import questionary
import typer

from tagslam_tools import __version__
from tagslam_tools.calibrate import calibrate_from_images
from tagslam_tools.camera import interactive_capture, publish_camera_loop
from tagslam_tools.commands.analysis import analysis_app
from tagslam_tools.commands.calibrate import calibrate_app
from tagslam_tools.commands.camera import camera_app
from tagslam_tools.commands.generate import generate_app
from tagslam_tools.commands.launch import _launch_all, launch_app
from tagslam_tools.commands.visualizer import visualizer_app
from tagslam_tools.pose_analysis import analyze_pose_log
from tagslam_tools.utils import (
    PALETTE,
    console,
    heading,
    make_choice,
    questionary_style,
    section,
    setup_logging,
)
from tagslam_tools.visualizer import run_visualizer

logger = logging.getLogger(__name__)

app = typer.Typer(
    name="tagslam-tools",
    help="TagSLAM auxiliary tools — camera, calibration, visualization, SLAM launch.",
    invoke_without_command=True,
)

app.add_typer(camera_app, name="camera", help="Camera capture and publishing")
app.add_typer(calibrate_app, name="calibrate", help="Camera calibration")
app.add_typer(analysis_app, name="analyze", help="Pose log analysis")
app.add_typer(generate_app, name="generate", help="AprilTag marker generation")
app.add_typer(visualizer_app, name="visualize", help="SLAM visualization")
app.add_typer(launch_app, name="launch", help="Launch the full pipeline")


@app.callback()
def main(
    ctx: typer.Context,
    menu: Annotated[bool, typer.Option("--menu", help="Launch interactive menu mode")] = False,
    verbose: Annotated[bool, typer.Option("--verbose", "-v", help="Enable debug logging")] = False,
    version: Annotated[
        bool | None,
        typer.Option("--version", is_eager=True, help="Show version"),
    ] = None,
) -> None:
    """TagSLAM Tools — camera capture, calibration, visualization, and SLAM launch."""
    setup_logging(verbose=verbose)
    if version:
        console.print(f"tagslam-tools v{__version__}", style="heading")
        raise typer.Exit()
    if menu or ctx.invoked_subcommand is None:
        _show_banner()
        _interactive_menu()


# ── Banner ─────────────────────────────────────────────────────


def _show_banner() -> None:
    section()
    heading("  TagSLAM Tools")
    section()


# ── Interactive menu ───────────────────────────────────────────


def _interactive_menu() -> None:
    while True:
        action = questionary.select(
            "What would you like to do?",
            style=questionary_style,
            choices=[
                questionary.Separator("Camera"),
                make_choice("capture", "Preview camera, SPACE to save frames"),
                make_choice("publish", "Publish camera frames to ROS2 topic"),
                questionary.Separator("Calibration"),
                make_choice("calibrate", "Calibrate camera from chessboard images"),
                questionary.Separator("SLAM"),
                make_choice("launch --viz", "Start full pipeline with visualizer"),
                make_choice("launch", "Start full pipeline only"),
                make_choice("visualize", "Live SLAM pose overlay on camera feed"),
                questionary.Separator("Analysis"),
                make_choice("analyze", "Compute distances from pose log file"),
                questionary.Separator(""),
                questionary.Choice(
                    title=[(f"fg:{PALETTE['muted']}", "  exit")],
                    value="exit",
                ),
            ],
        ).ask()

        if action is None or action == "exit":
            break

        logger.info("Menu selected: %s", action)
        match action:
            case "capture":
                interactive_capture("pic")
            case "publish":
                publish_camera_loop()
            case "calibrate":
                calibrate_from_images("pic")
            case "launch --viz":
                _launch_all(viz=True)
            case "launch":
                _launch_all(viz=False)
            case "visualize":
                run_visualizer()
            case "analyze":
                analyze_pose_log("pose_log.txt")
            case _:
                logger.warning("Unknown menu action: %s", action)

    console.print("Goodbye!", style="dim")
