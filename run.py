"""TagSLAM Tools launcher — run.py at project root.

Usage:
    uv run run.py --menu           # interactive menu mode
    uv run run.py launch --viz     # one-click SLAM + visualizer
    uv run run.py camera capture   # capture calibration images
    uv run run.py --version        # show version
"""

from __future__ import annotations

from tagslam_tools.cli import app

if __name__ == "__main__":
    app()
