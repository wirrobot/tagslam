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
from tagslam_tools.commands.photo import photo_app
from tagslam_tools.commands.video import video_app
from tagslam_tools.commands.visualizer import visualizer_app
from tagslam_tools.photo import interactive_photo_capture
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
from tagslam_tools.video import interactive_video_record
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
app.add_typer(photo_app, name="photo", help="Point cloud photo capture")
app.add_typer(video_app, name="video", help="Video recording")


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


def _capture_with_device(func=None, default_dir: str = "pic") -> None:
    """Ask for camera device before starting capture.\n    \n    Args:\n        func: capture function to call (interactive_capture, interactive_photo_capture, etc.)\n        default_dir: default save directory\n    """
    import os

    if func is None:
        func = interactive_capture  # type: ignore[assignment]

    available = sorted(
        int(f[5:]) for f in os.listdir("/dev") if f.startswith("video") and f[5:].isdigit()
    )
    device = questionary.select(
        "Select camera device:",
        style=questionary_style,
        choices=[
            make_choice(str(d), f"/dev/video{d}") for d in available
        ],
    ).ask()
    if device and device.isdigit():
        func(default_dir, device=int(device))
    else:
        func(default_dir)


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
                questionary.Separator("Point Cloud"),
                make_choice("photo", "Preview camera, SPACE to save photos to data/point/"),
                questionary.Separator("Video"),
                make_choice("video", "Preview camera, SPACE to toggle recording"),
                questionary.Separator("SLAM"),
                make_choice("launch --viz", "Start full pipeline with visualizer"),
                make_choice("launch", "Start full pipeline only"),
                make_choice("visualize", "Live SLAM pose overlay on camera feed"),
                questionary.Separator("Analysis"),
                make_choice("analyze", "Compute distances from pose log file"),
                make_choice("point-loss", "Point-cloud vs video TagSLAM loss analysis"),
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
                _capture_with_device()
            case "publish":
                publish_camera_loop()
            case "calibrate":
                calibrate_from_images("pic")
            case "photo":
                _capture_with_device(func=interactive_photo_capture, default_dir="data/point")
            case "video":
                _capture_with_device(func=interactive_video_record, default_dir="data/video")
            case "launch --viz":
                _launch_all(viz=True)
            case "launch":
                _launch_all(viz=False)
            case "visualize":
                run_visualizer()
            case "analyze":
                analyze_pose_log("pose_log_multi.txt")
                console.print()
                analyze_pose_log("pose_log_single.txt")
                console.print("  Press Enter to exit...", style="dim")
                try:
                    input()
                except (EOFError, KeyboardInterrupt):
                    pass
            case "point-loss":
                from tagslam_tools.point_analysis import run_point_analysis

                run_point_analysis("data/point", "data/video", "data/loss")
                console.print()
                console.print("  Press Enter to exit...", style="dim")
                try:
                    input()
                except (EOFError, KeyboardInterrupt):
                    pass
            case _:
                logger.warning("Unknown menu action: %s", action)

    console.print("Goodbye!", style="dim")
