"""Command sub-apps for TagSLAM Tools."""

from __future__ import annotations

from tagslam_tools.commands.analysis import analysis_app
from tagslam_tools.commands.calibrate import calibrate_app
from tagslam_tools.commands.camera import camera_app
from tagslam_tools.commands.generate import generate_app
from tagslam_tools.commands.launch import launch_app
from tagslam_tools.commands.photo import photo_app
from tagslam_tools.commands.video import video_app
from tagslam_tools.commands.visualizer import visualizer_app

__all__ = [
    "analysis_app",
    "calibrate_app",
    "camera_app",
    "generate_app",
    "launch_app",
    "photo_app",
    "video_app",
    "visualizer_app",
]
