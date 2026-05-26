"""Compute distances between consecutive pose entries in a log file."""

from __future__ import annotations

import math
import re
from typing import Any


def parse_pose_log(filepath: str) -> list[dict[str, Any]]:
    """Parse a pose log file into a list of dicts with keys: ts, x, y, z."""
    entries: list[dict[str, Any]] = []
    # Match either full format (with quaternion) or simple format (xyz only)
    pattern_full = re.compile(
        r"^(?P<ts>[\d.]+)\s+"
        r"x=(?P<x>[-\d.]+)\s+y=(?P<y>[-\d.]+)\s+z=(?P<z>[-\d.]+)\s+"
        r"qx=(?P<qx>[-\d.]+)\s+qy=(?P<qy>[-\d.]+)\s+qz=(?P<qz>[-\d.]+)\s+qw=(?P<qw>[-\d.]+)"
    )
    pattern_simple = re.compile(
        r"^(?P<ts>[\d.]+)\s+"
        r"x=(?P<x>[-\d.]+)\s+y=(?P<y>[-\d.]+)\s+z=(?P<z>[-\d.]+)"
    )
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            m = pattern_full.match(line) or pattern_simple.match(line)
            if m:
                entries.append(
                    {
                        "ts": float(m.group("ts")),
                        "x": float(m.group("x")),
                        "y": float(m.group("y")),
                        "z": float(m.group("z")),
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
    """Analyze a pose log file and return summary statistics.

    Prints results to stdout and returns the data dict.
    """
    entries = parse_pose_log(filepath)
    if len(entries) < 2:
        print(f"  Need at least 2 entries, found {len(entries)}")
        return None

    dists = compute_distances(entries)
    total = sum(dists)
    result = {
        "filepath": filepath,
        "num_entries": len(entries),
        "num_pairs": len(dists),
        "distances": dists,
        "total": total,
        "min": min(dists),
        "max": max(dists),
        "mean": total / len(dists),
    }

    print(f"  File    : {result['filepath']}")
    print(f"  Entries : {result['num_entries']}")
    print(f"  Pairs   : {result['num_pairs']}")
    print(f"  {'Pair':<8} {'Distance (m)':<14} {'Cumulative (m)'}")
    cumulative = 0.0
    for i, d in enumerate(result["distances"]):  # type: ignore[var-annotated,arg-type]
        cumulative += d
        print(f"  {i + 1:<8} {d:<14.6f} {cumulative:<.6f}")
    print(f"  Min   : {result['min']:.6f} m")
    print(f"  Max   : {result['max']:.6f} m")
    print(f"  Mean  : {result['mean']:.6f} m")
    print(f"  Total : {result['total']:.6f} m")
    return result
