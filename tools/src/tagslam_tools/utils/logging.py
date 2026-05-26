"""Structured logging with dual handlers (console + file)."""

from __future__ import annotations

import logging
import sys


def setup_logging(*, verbose: bool = False) -> None:
    """Configure root logger with console (INFO+) and file (DEBUG+) handlers."""
    level = logging.DEBUG if verbose else logging.INFO
    root = logging.getLogger()
    root.setLevel(logging.DEBUG)
    root.handlers.clear()

    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")

    sh = logging.StreamHandler(sys.stdout)
    sh.setLevel(level)
    sh.setFormatter(fmt)
    root.addHandler(sh)

    fh = logging.FileHandler("tagslam_tools.log", mode="a")
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(fmt)
    root.addHandler(fh)
