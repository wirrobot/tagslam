"""Tests for AprilTag detection and pose estimation core logic.

Requires ``test.jpg`` placed in ``tools/tests/``.
Camera intrinsics and tag sizes are read from ``config/`` YAML files.

Usage:
    uv run pytest tools/tests/test_tag_detection.py -sv
    uv run pytest tools/tests/ -k tag_detection -sv
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, NamedTuple

import numpy as np
import pytest

try:
    import cv2
except ImportError:
    cv2 = None  # type: ignore[assignment]

try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None  # type: ignore[assignment]


class PoseResult(NamedTuple):
    tag_id: int
    tag_size: float
    pose_R: np.ndarray
    pose_t: np.ndarray
    pose_err: float
    corners: np.ndarray
    center: np.ndarray
    family: str
    hamming: int
    decision_margin: float


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

SEP = "─" * 56


def _print_sep(title: str = "") -> None:
    print(f"\n{SEP}")
    if title:
        print(f"  {title}")
        print(SEP)


def _format_matrix(mat: np.ndarray, precision: int = 4) -> str:
    return np.array2string(mat, precision=precision, suppress_small=True)


def _print_corners(corners: np.ndarray, cx: float, cy: float) -> None:
    print("  Corners (px):")
    for i in range(corners.shape[0]):
        print(f"    [{corners[i, 0]:8.2f}, {corners[i, 1]:8.2f}]")


def _print_pose(R: np.ndarray, t: np.ndarray, err: float, tag_size: float) -> None:
    dist = np.linalg.norm(t)
    print(f"  Pose  (tag_size={tag_size:.4f} m):")
    print(f"    R = {_format_matrix(R)}")
    print(f"    t = {_format_matrix(t.ravel(), 4)}  m")
    print(f"    Distance: {dist:.4f} m")
    print(f"    Error:    {err:.6f}")


def _tag_family_str(det: Any) -> str:
    f = det.tag_family
    return f.decode() if isinstance(f, bytes) else str(f)


def _print_detection_no_pose(det: Any) -> None:
    _print_sep(f"Tag ID={det.tag_id}  (detection only)")
    print(f"  Family:          {_tag_family_str(det)}")
    print(f"  Hamming:         {det.hamming}")
    print(f"  Decision margin: {det.decision_margin:.2f}")
    _print_corners(det.corners, det.center[0], det.center[1])


def _print_detection_pose(r: PoseResult) -> None:
    _print_sep(f"Tag ID={r.tag_id}  (with pose)")
    print(f"  Family:          {r.family}")
    print(f"  Hamming:         {r.hamming}")
    print(f"  Decision margin: {r.decision_margin:.2f}")
    _print_corners(r.corners, r.center[0], r.center[1])
    _print_pose(r.pose_R, r.pose_t, r.pose_err, r.tag_size)


# ---------------------------------------------------------------------------
# default fallback values (used when config YAMLs are missing)
# ---------------------------------------------------------------------------

_DEFAULT_CAMERA_PARAMS = (1653.275628, 1654.027585, 648.592971, 355.735559)
_DEFAULT_TAG_SIZES: dict[int, float] = {0: 0.1283, 1: 0.06416}


# ---------------------------------------------------------------------------
# fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def fx() -> float:
    return _DEFAULT_CAMERA_PARAMS[0]


@pytest.fixture
def fy() -> float:
    return _DEFAULT_CAMERA_PARAMS[1]


@pytest.fixture
def cx() -> float:
    return _DEFAULT_CAMERA_PARAMS[2]


@pytest.fixture
def cy() -> float:
    return _DEFAULT_CAMERA_PARAMS[3]


@pytest.fixture
def cam_params(
    camera_params: tuple[float, float, float, float] | None,
) -> tuple[float, float, float, float]:
    return camera_params if camera_params is not None else _DEFAULT_CAMERA_PARAMS


@pytest.fixture
def sizes(tag_sizes: dict[int, float] | None) -> dict[int, float]:
    return tag_sizes if (tag_sizes is not None and len(tag_sizes) > 0) else _DEFAULT_TAG_SIZES


@pytest.fixture
def gray_img(test_image_path: str) -> np.ndarray:
    if cv2 is None:
        pytest.skip("opencv-python not installed")
    if not Path(test_image_path).is_file():
        pytest.skip(f"test.jpg not found at {test_image_path}")
    img = cv2.imread(test_image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        pytest.fail(f"Failed to read {test_image_path}")
    return img


@pytest.fixture
def raw_detections(gray_img: np.ndarray) -> list[Any]:
    """Run detection without pose estimation."""
    if Detector is None:
        pytest.skip("pupil_apriltags not installed")
    detector = Detector(families="tag36h11", quad_decimate=1.0)
    return detector.detect(gray_img)


@pytest.fixture
def pose_results(
    gray_img: np.ndarray,
    cam_params: tuple[float, float, float, float],
    sizes: dict[int, float],
) -> list[PoseResult]:
    """Run detection with pose estimation for each configured tag_size pair.

    Returns a list of PoseResult namedtuples, one per (detected_tag, tag_size).
    """
    if Detector is None:
        pytest.skip("pupil_apriltags not installed")
    results: list[PoseResult] = []
    seen: set[tuple[int, float]] = set()
    for _tag_id, size in sizes.items():
        detector = Detector(families="tag36h11", quad_decimate=1.0)
        dets = detector.detect(
            gray_img, estimate_tag_pose=True, camera_params=cam_params, tag_size=size
        )
        for det in dets:
            key = (det.tag_id, size)
            if key not in seen:
                seen.add(key)
                results.append(
                    PoseResult(
                        tag_id=det.tag_id,
                        tag_size=size,
                        pose_R=det.pose_R.copy(),
                        pose_t=det.pose_t.copy(),
                        pose_err=det.pose_err,
                        corners=det.corners.copy(),
                        center=det.center.copy(),
                        family=_tag_family_str(det),
                        hamming=det.hamming,
                        decision_margin=det.decision_margin,
                    )
                )
    return results


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------


def test_load_image(gray_img: np.ndarray, test_image_path: str) -> None:
    """Verify test.jpg is a valid grayscale image."""
    h, w = gray_img.shape
    assert h > 0 and w > 0

    _print_sep("Image Info")
    print(f"  Path: {test_image_path}")
    print(f"  Size: {w} x {h}")


def test_detect_tags(raw_detections: list[Any], gray_img: np.ndarray) -> None:
    """Detect AprilTags and print each tag's pixel coordinates."""
    assert len(raw_detections) >= 1, (
        f"Expected at least one tag detected in {gray_img.shape[1]}x{gray_img.shape[0]} image"
    )

    _print_sep(f"Detection Results  ({len(raw_detections)} tag(s) found)")
    for det in raw_detections:
        corners = det.corners
        assert corners.shape == (4, 2), f"Expected corners shape (4,2), got {corners.shape}"
        assert not np.any(np.isnan(corners)), "Corners contain NaN"
        assert isinstance(det.tag_id, int)
        assert isinstance(det.tag_family, (str, bytes))
        _print_detection_no_pose(det)


def test_corners_valid(raw_detections: list[Any], gray_img: np.ndarray) -> None:
    """Verify all detected corners are within image bounds and form valid quads."""
    h, w = gray_img.shape
    for det in raw_detections:
        corners = det.corners
        for i in range(corners.shape[0]):
            x, y = float(corners[i, 0]), float(corners[i, 1])
            assert 0 <= x < w, f"Tag ID={det.tag_id} corner[{i}] x={x:.1f} out of [0, {w})"
            assert 0 <= y < h, f"Tag ID={det.tag_id} corner[{i}] y={y:.1f} out of [0, {h})"
        area = float(cv2.contourArea(corners.astype(np.float32)))
        assert area > 0, f"Tag ID={det.tag_id} quad area={area:.1f} must be positive"


def test_pose_estimation(pose_results: list[PoseResult]) -> None:
    """Estimate 6-DOF pose for each detected tag and print results.

    Uses the tag_size matching the tag_id from the config.
    """
    assert len(pose_results) >= 1, "Expected at least one pose result"

    _print_sep(f"Pose Estimation Results  ({len(pose_results)} result(s))")
    for r in pose_results:
        _print_detection_pose(r)

    for r in pose_results:
        det_r = np.linalg.det(r.pose_R)
        assert 0.9 < det_r < 1.1, (
            f"Tag ID={r.tag_id}: det(R)={det_r:.4f}, expected ~1.0 "
            f"(tag_size={r.tag_size:.4f})"
        )
        distance = np.linalg.norm(r.pose_t)
        assert 0.01 < distance < 20.0, (
            f"Tag ID={r.tag_id}: distance={distance:.3f}m "
            f"out of [0.01, 20.0] (tag_size={r.tag_size:.4f})"
        )
        assert r.pose_err < 2.0, (
            f"Tag ID={r.tag_id}: pose_err={r.pose_err:.4f} too large "
            f"(tag_size={r.tag_size:.4f})"
        )


def test_pose_consistency(
    gray_img: np.ndarray,
    cam_params: tuple[float, float, float, float],
    sizes: dict[int, float],
    raw_detections: list[Any],
) -> None:
    """Verify that estimated distance ratio matches tag_size ratio.

    When the same detected tag is processed with two different tag_size values,
    the estimated distance should scale proportionally.
    """
    if Detector is None:
        pytest.skip("pupil_apriltags not installed")
    if len(sizes) < 2:
        pytest.skip("Need at least 2 tag sizes in config for consistency test")

    sizes_sorted = sorted(sizes.values())
    s_small, s_large = sizes_sorted[0], sizes_sorted[-1]
    expected_ratio = s_large / s_small

    common_ids = sorted({d.tag_id for d in raw_detections} & set(sizes.keys()))
    if not common_ids:
        pytest.skip("No detected tag matches any configured tag ID")

    test_id = common_ids[0]
    detector = Detector(families="tag36h11", quad_decimate=1.0)

    def _dist(size: float) -> float:
        dets = detector.detect(
            gray_img, estimate_tag_pose=True, camera_params=cam_params, tag_size=size
        )
        for d in dets:
            if d.tag_id == test_id:
                return float(np.linalg.norm(d.pose_t))
        return 0.0

    d_small = _dist(s_small)
    d_large = _dist(s_large)
    actual_ratio = d_large / d_small if d_small > 0 else 0.0

    _print_sep("Pose Consistency Check")
    print(f"  Test tag ID:      {test_id}")
    print(f"  Tag sizes:        {s_small:.4f} m  →  {s_large:.4f} m")
    print(f"  Expected ratio:   {expected_ratio:.4f}")
    print(f"  Distance (size={s_small:.4f}): {d_small:.4f} m")
    print(f"  Distance (size={s_large:.4f}): {d_large:.4f} m")
    print(f"  Actual ratio:     {actual_ratio:.4f}")

    assert d_small > 0, f"Tag ID={test_id}: pose failed for size={s_small:.4f}m"
    assert d_large > 0, f"Tag ID={test_id}: pose failed for size={s_large:.4f}m"

    rel_error = abs(actual_ratio - expected_ratio) / expected_ratio
    assert rel_error < 0.2, (
        f"Distance ratio {actual_ratio:.4f} deviates from expected {expected_ratio:.4f} "
        f"(relative error {rel_error:.3%})"
    )
