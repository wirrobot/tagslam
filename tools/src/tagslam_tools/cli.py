"""TagSLAM Tools CLI — camera, calibration, visualization, launch."""

from __future__ import annotations

import logging
import os
import subprocess
import sys
from pathlib import Path

import questionary
import typer
from rich.console import Console
from rich.theme import Theme

from tagslam_tools.calibrate import calibrate_from_images
from tagslam_tools.camera import interactive_capture, publish_camera_loop
from tagslam_tools.visualizer import run_visualizer

app = typer.Typer()

THEME = Theme(
    {
        "divider": "bold cyan",
        "heading": "bold green",
        "success": "bold green",
        "warn": "yellow",
        "error": "bold red",
    }
)
console = Console(theme=THEME)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.FileHandler("tagslam_tools.log"), logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent.parent
CONFIG_DIR = PROJECT_ROOT / "config"
TOOLS_DIR = PROJECT_ROOT / "tools"


def _divider() -> None:
    console.print("=" * 56, style="divider")


# ── CLI commands ──────────────────────────────────────────────


@app.command()
def capture(save_dir: str = "pic") -> None:
    """Open camera preview.  SPACE to save frame, ESC/Q to quit."""
    logger.info("Starting interactive capture, saving to %s", save_dir)
    count = interactive_capture(save_dir)
    logger.info("Captured %d images", count)


@app.command()
def calibrate(
    image_dir: str = "pic",
    chessboard_cols: int = 11,
    chessboard_rows: int = 8,
    square_size: float = 0.015,
) -> None:
    """Calibrate camera from chessboard images."""
    logger.info(
        "Calibrating: dir=%s size=(%d,%d) square=%.4fm",
        image_dir,
        chessboard_cols,
        chessboard_rows,
        square_size,
    )
    result = calibrate_from_images(
        image_dir,
        chessboard_size=(chessboard_cols, chessboard_rows),
        square_size=square_size,
    )
    if result is None:
        console.print("Calibration failed.", style="error")
        raise typer.Exit(1)
    console.print("Calibration succeeded!", style="success")
    logger.info("Calibration RMS=%.4f", result["rms"])


@app.command()
def publish(topic: str = "camera/image_raw") -> None:
    """Publish camera frames to a ROS2 topic (requires ROS2 env sourced)."""
    logger.info("Publishing camera to topic: %s", topic)
    console.print("Camera publisher starting. Press Ctrl+C to stop.", style="heading")
    publish_camera_loop(topic)


@app.command()
def visualize(
    image_topic: str = "camera/image_raw",
    odom_topic: str = "/odom/body_rig",
) -> None:
    """Show camera feed with overlaid SLAM pose (requires ROS2 env)."""
    logger.info("Starting visualizer: image=%s odom=%s", image_topic, odom_topic)
    run_visualizer(image_topic, odom_topic)


@app.command()
def launch(
    visualize: bool = typer.Option(False, "--viz", help="Also start the visualizer window"),
) -> None:
    """One-click launch: camera + sync_and_detect + tagslam."""
    _launch_all(visualize)


# ── Interactive menu ──────────────────────────────────────────


@app.command()
def menu() -> None:
    """Launch interactive menu for all operations."""
    _divider()
    console.print("  TagSLAM Tools", style="heading")
    _divider()

    while True:
        action = questionary.select(
            "What would you like to do?",
            choices=[
                questionary.Separator("── Camera ──"),
                "Capture calibration images",
                "Publish camera to ROS2 topic",
                questionary.Separator("── Calibration ──"),
                "Calibrate camera from images",
                questionary.Separator("── SLAM ──"),
                "Launch full SLAM pipeline",
                "Launch SLAM + visualizer",
                "Run visualizer only",
                questionary.Separator("──"),
                "Exit",
            ],
        ).ask()

        if action is None or action == "Exit":
            break

        logger.info("Menu: %s", action)
        match action:
            case "Capture calibration images":
                interactive_capture("pic")
            case "Publish camera to ROS2 topic":
                publish_camera_loop()
            case "Calibrate camera from images":
                calibrate_from_images("pic")
            case "Launch full SLAM pipeline":
                _launch_all(viz=False)
            case "Launch SLAM + visualizer":
                _launch_all(viz=True)
            case "Run visualizer only":
                run_visualizer()

    console.print("Goodbye!", style="heading")


# ── Helpers ───────────────────────────────────────────────────


def _launch_all(viz: bool = False) -> None:
    """Run the full pipeline via bash helper."""
    config_dir = str(CONFIG_DIR)

    _divider()
    console.print("  TagSLAM Pipeline", style="heading")
    _divider()

    try:
        logger.info("Starting camera publisher...")
        cmd = "from tagslam_tools.camera import publish_camera_loop; publish_camera_loop()"
        proc_cam = subprocess.Popen(
            [sys.executable, "-c", cmd],
            env={
                **os.environ,
                "PYTHONPATH": str(TOOLS_DIR / "src") + ":" + os.environ.get("PYTHONPATH", ""),
            },
        )
    except Exception:
        logger.exception("Failed to start camera publisher")
        return

    try:
        logger.info("Starting sync_and_detect...")
        proc_sad = subprocess.Popen(
            [
                "ros2",
                "launch",
                "tagslam",
                "sync_and_detect.launch.py",
                f"cameras:={config_dir}/cameras.yaml",
                f"tagslam_config:={config_dir}/tagslam.yaml",
                "use_approximate_sync:=True",
            ],
        )
    except FileNotFoundError:
        logger.error("ros2 not found — is ROS2 sourced?")
        proc_cam.terminate()
        return

    try:
        logger.info("Starting tagslam...")
        proc_slam = subprocess.Popen(
            [
                "ros2",
                "launch",
                "tagslam",
                "tagslam.launch.py",
                f"cameras:={config_dir}/cameras.yaml",
                f"camera_poses:={config_dir}/camera_poses.yaml",
                f"tagslam_config:={config_dir}/tagslam.yaml",
                "use_approximate_sync:=True",
            ],
        )
    except FileNotFoundError:
        logger.error("ros2 not found")
        proc_cam.terminate()
        proc_sad.terminate()
        return

    if viz:
        logger.info("Starting visualizer...")
        cmd = "from tagslam_tools.visualizer import run_visualizer; run_visualizer()"
        proc_viz = subprocess.Popen([sys.executable, "-c", cmd])

    console.print("All nodes running. Press Ctrl+C to stop.", style="success")
    console.print(f"  {'Camera':<22} camera/image_raw", style="warn")
    console.print(f"  {'Tag detections':<22} /detector/tags", style="warn")
    console.print(f"  {'SLAM odometry':<22} /odom/body_rig", style="warn")

    try:
        proc_cam.wait()
    except KeyboardInterrupt:
        pass
    finally:
        for p in [proc_cam, proc_sad, proc_slam]:
            p.terminate()
        if viz:
            proc_viz.terminate()  # type: ignore[possibly-used]


if __name__ == "__main__":
    app()
