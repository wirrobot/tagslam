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

launch_app = typer.Typer(
    help="Launch the full TagSLAM pipeline",
    invoke_without_command=True,
)


@launch_app.callback()
def launch(
    viz: Annotated[bool, typer.Option("--viz", help="Also start the visualizer window")] = False,
    device: Annotated[
        int,
        typer.Option("--device", help="Camera device ID (e.g. 0 → /dev/video0)"),
    ] = 0,
    width: Annotated[
        int | None,
        typer.Option("--width", help="Capture width (default: from config/cameras.yaml)"),
    ] = None,
    height: Annotated[
        int | None,
        typer.Option("--height", help="Capture height (default: from config/cameras.yaml)"),
    ] = None,
) -> None:
    """One-click launch: camera publisher + sync_and_detect + tagslam."""
    _launch_all(viz=viz, device=device, width=width, height=height)


def _kill_stray_camera_publishers() -> None:
    """Kill any existing camera publisher subprocesses left from previous runs."""
    import signal

    killed = 0
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            cmdline = open(f"/proc/{entry}/cmdline", "rb").read()
            if b"publish_camera_loop" in cmdline:
                os.kill(int(entry), signal.SIGTERM)
                killed += 1
        except (OSError, ProcessLookupError):
            continue
    if killed:
        logger.info("Cleaned up %d stray camera publisher(s)", killed)


def _read_config_resolution() -> tuple[int, int]:
    """Return (width, height) from config/cameras.yaml."""
    try:
        from yaml import safe_load

        with open(CONFIG_DIR / "cameras.yaml") as f:
            cfg = safe_load(f)
        for cam in cfg.values():
            if isinstance(cam, dict) and "resolution" in cam:
                w, h = cam["resolution"]
                return (int(w), int(h))
    except Exception:
        logger.debug("Could not read resolution from config")
    return (1280, 720)


def _launch_all(
    viz: bool = False, device: int = 0, width: int | None = None, height: int | None = None
) -> None:
    """Core pipeline launcher — reusable from CLI and interactive menu."""
    heading("Launch — full pipeline")
    section()

    _kill_stray_camera_publishers()

    res_w, res_h = _read_config_resolution()
    if width is not None and height is not None:
        res_w, res_h = width, height
        resolution_label = f"{res_w}x{res_h} (overridden)"
    else:
        resolution_label = f"{res_w}x{res_h}"

    console.print(f"  Config : {CONFIG_DIR}", style="item")
    console.print(f"  Camera : /dev/video{device}  @  {resolution_label}", style="item")
    if viz:
        console.print("  Visualizer : enabled", style="item")
    section()

    config_dir = str(CONFIG_DIR)
    procs: list[subprocess.Popen[bytes]] = []
    penv = {
        **os.environ,
        "PYTHONPATH": ":".join(
            [
                str(TOOLS_DIR / "src"),
                str(PROJECT_ROOT / ".venv" / "lib" / "python3.10" / "site-packages"),
                os.environ.get("PYTHONPATH", ""),
            ]
        ),
    }

    logger.info("Starting camera publisher (device=%d, %dx%d)...", device, res_w, res_h)
    cam_code = (
        f"from tagslam_tools.camera import publish_camera_loop; "
        f"publish_camera_loop(device={device}, width={res_w}, height={res_h})"
    )
    procs.append(
        subprocess.Popen(
            [sys.executable, "-c", cam_code],
            env=penv,
        )
    )
    console.print(f"  {'[1/4] camera publisher':<30} started (device={device}, {res_w}x{res_h})", style="success")

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
        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        for p in procs:
            p.terminate()
        for p in procs:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        section()
        console.print("  All nodes stopped.", style="warn")
        console.print("  Data saved to pose_log_multi.txt / pose_log_single.txt", style="item")
