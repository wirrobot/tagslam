"""Compute distances between consecutive pose entries in a log file."""

from __future__ import annotations

import math
import re
from typing import Any


def parse_pose_log(filepath: str) -> list[dict[str, Any]]:
    """Parse pose_log.txt into a list of dicts with keys: ts, x, y, z, qx, qy, qz, qw."""
    entries: list[dict[str, Any]] = []
    pattern = re.compile(
        r"^(?P<ts>[\d.]+)\s+"
        r"x=(?P<x>[-\d.]+)\s+y=(?P<y>[-\d.]+)\s+z=(?P<z>[-\d.]+)\s+"
        r"qx=(?P<qx>[-\d.]+)\s+qy=(?P<qy>[-\d.]+)\s+qz=(?P<qz>[-\d.]+)\s+qw=(?P<qw>[-\d.]+)"
    )
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = pattern.match(line)
            if m:
                entries.append(
                    {
                        "ts": float(m.group("ts")),
                        "x": float(m.group("x")),
                        "y": float(m.group("y")),
                        "z": float(m.group("z")),
                        "qx": float(m.group("qx")),
                        "qy": float(m.group("qy")),
                        "qz": float(m.group("qz")),
                        "qw": float(m.group("qw")),
                    }
                )
    return entries


def compute_distances(entries: list[dict[str, Any]]) -> list[float]:
    """Return Euclidean distances between consecutive entries."""
    dists: list[float] = []
    for i in range(1, len(entries)):
        a = entries[i - 1]
        b = entries[i]
        d = math.sqrt((b["x"] - a["x"]) ** 2 + (b["y"] - a["y"]) ** 2 + (b["z"] - a["z"]) ** 2)
        dists.append(d)
    return dists


def analyze_pose_log(filepath: str = "pose_log.txt") -> dict[str, Any] | None:
    """Analyze a pose log file and return summary statistics."""
    entries = parse_pose_log(filepath)
    if len(entries) < 2:
        return None

    dists = compute_distances(entries)
    total = sum(dists)
    return {
        "filepath": filepath,
        "num_entries": len(entries),
        "num_pairs": len(dists),
        "distances": dists,
        "total": total,
        "min": min(dists),
        "max": max(dists),
        "mean": total / len(dists),
    }
