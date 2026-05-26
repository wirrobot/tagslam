"""Launch command — one-click pipeline starter."""

from __future__ import annotations

import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Annotated

import typer

from tagslam_tools.utils import console, heading, section

logger = logging.getLogger(__name__)

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent.parent.parent
CONFIG_DIR = PROJECT_ROOT / "config"
TOOLS_DIR = PROJECT_ROOT / "tools"

launch_app = typer.Typer(help="Launch the full TagSLAM pipeline")


@launch_app.command()
def launch(
    viz: Annotated[bool, typer.Option("--viz", help="Also start the visualizer window")] = False,
) -> None:
    """One-click launch: camera publisher + sync_and_detect + tagslam."""
    _launch_all(viz=viz)


def _launch_all(viz: bool = False) -> None:
    """Core pipeline launcher — reusable from CLI and interactive menu."""
    heading("Launch — full pipeline")
    section()
    console.print(f"  Config : {CONFIG_DIR}", style="item")
    if viz:
        console.print("  Visualizer : enabled", style="item")
    section()

    config_dir = str(CONFIG_DIR)
    procs: list[subprocess.Popen[bytes]] = []
    penv = {
        **os.environ,
        "PYTHONPATH": f"{TOOLS_DIR / 'src'}:{os.environ.get('PYTHONPATH', '')}",
    }

    logger.info("Starting camera publisher...")
    procs.append(
        subprocess.Popen(
            [
                sys.executable,
                "-c",
                "from tagslam_tools.camera import publish_camera_loop; publish_camera_loop()",
            ],
            env=penv,
        )
    )
    console.print(f"  {'[1/4] camera publisher':<30} started", style="success")

    logger.info("Starting sync_and_detect...")
    procs.append(
        subprocess.Popen(
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
    )
    console.print(f"  {'[2/4] sync_and_detect':<30} started", style="success")

    logger.info("Starting tagslam...")
    procs.append(
        subprocess.Popen(
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
    )
    console.print(f"  {'[3/4] tagslam':<30} started", style="success")

    if viz:
        logger.info("Starting visualizer...")
        procs.append(
            subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    "from tagslam_tools.visualizer import run_visualizer; run_visualizer()",
                ],
                env=penv,
            )
        )
        console.print(f"  {'[4/4] visualizer':<30} started", style="success")

    section()
    console.print(f"  {'Topics':<22} {'Status'}", style="heading")
    console.print(f"  {'camera/image_raw':<22} camera feed", style="item")
    console.print(f"  {'/detector/tags':<22} tag detections", style="item")
    console.print(f"  {'/odom/body_rig':<22} SLAM odometry", style="item")
    section()
    console.print("  Press Ctrl+C to stop all nodes.", style="dim")

    try:
        procs[0].wait()
    except KeyboardInterrupt:
        pass
    finally:
        for p in procs:
            p.terminate()
        section()
        console.print("  All nodes stopped.", style="warn")
        console.print("  Press Enter to exit...", style="dim")
        try:
            input()
        except (EOFError, KeyboardInterrupt):
            pass
