"""Camera calibration from a directory of chessboard images."""

from __future__ import annotations

import glob
import logging
import os
from typing import Any

import cv2
import numpy as np
from rich.console import Console

console = Console()
logger = logging.getLogger(__name__)


def calibrate_from_images(
    image_dir: str,
    chessboard_size: tuple[int, int] = (11, 8),
    square_size: float = 0.015,
) -> dict[str, Any] | None:
    """Run OpenCV camera calibration on all .jpg images in *image_dir*."""
    images = sorted(glob.glob(os.path.join(image_dir, "*.jpg")))
    if len(images) < 5:
        logger.error("Need at least 5 images, found %d", len(images))
        return None

    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : chessboard_size[0], 0 : chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    obj_points: list[np.ndarray] = []
    img_points: list[np.ndarray] = []
    img_size: tuple[int, int] | None = None

    for fpath in images:
        img = cv2.imread(fpath)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if img_size is None:
            h, w = gray.shape[:2]
            img_size = (w, h)

        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        if ret:
            obj_points.append(objp)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            img_points.append(corners2)
            console.print(f"  [green]OK[/]  {os.path.basename(fpath)}")
        else:
            console.print(f"  [yellow]SKIP[/] {os.path.basename(fpath)}")

    num_total = len(images)
    num_used = len(obj_points)
    console.print()
    console.print(f"Found chessboard in {num_used} / {num_total} images")

    if num_used < 5 or img_size is None:
        logger.error("Not enough valid images (%d)", num_used)
        return None

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(  # type: ignore[call-overload]
        obj_points, img_points, img_size, None, None
    )

    mean_err = 0.0
    for i in range(num_used):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        mean_err += cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
    mean_err /= num_used

    console.print()
    console.print("=" * 50, style="bold cyan")
    console.print(f"  RMS reprojection error : {mean_err:.4f} px")
    console.print(f"  Image size             : {img_size[0]} x {img_size[1]}")
    console.print()
    console.print(f"  fx={K[0, 0]:.6f}  fy={K[1, 1]:.6f}", style="bold")
    console.print(f"  cx={K[0, 2]:.6f}  cy={K[1, 2]:.6f}", style="bold")
    console.print()
    console.print(f"  K = [[{K[0, 0]:.6f}, 0.0, {K[0, 2]:.6f}],")
    console.print(f"       [0.0, {K[1, 1]:.6f}, {K[1, 2]:.6f}],")
    console.print("       [0.0, 0.0, 1.0]]")
    console.print()
    console.print(f"  Distortion ({len(dist.ravel())} coeffs):")
    console.print(f"  {np.array2string(dist.ravel(), separator=', ', precision=8)}")
    console.print("=" * 50, style="bold cyan")

    return {
        "K": K,
        "dist": dist,
        "img_size": img_size,
        "rms": mean_err,
        "num_used": num_used,
        "num_total": num_total,
    }
