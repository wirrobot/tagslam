"""Point-cloud to tag-map analysis — compute camera position, fit line, measure loss."""

from __future__ import annotations

import csv
import logging
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

logger = logging.getLogger(__name__)

def _import_apriltag():
    """Import pupil_apriltags Detector, bypassing ROS2 path conflicts."""
    saved = list(sys.path)
    sys.path = [p for p in sys.path if "ros" not in p.lower() and "opt/ros" not in p]
    extra_paths = [
        os.path.expanduser("~/.local/lib/python3.10/site-packages"),
        os.path.expanduser("~/miniconda3/lib/python3.13/site-packages"),
        os.path.expanduser("~/miniconda3/lib/python3.10/site-packages"),
        "/usr/lib/python3/dist-packages",
    ]
    for p in extra_paths:
        if os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)
    try:
        from pupil_apriltags import Detector  # type: ignore[import-untyped]

        return Detector
    finally:
        sys.path = saved


def _make_detector(quad_decimate: float = 2.0):
    """Create a pupil_apriltags Detector using system libapriltag (3.4.5), not bundled 3.1.0."""
    Detector = _import_apriltag()
    system_lib = "/opt/ros/humble/lib/x86_64-linux-gnu"
    searchpath = (
        (Path(system_lib),)
        if os.path.isdir(system_lib)
        else ()
    )
    return Detector(families="tag36h11", quad_decimate=quad_decimate, searchpath=searchpath)


def _axis_angle_to_rot(rx: float, ry: float, rz: float) -> np.ndarray:
    angle = np.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-12:
        return np.eye(3)
    axis = np.array([rx, ry, rz]) / angle
    c = np.cos(angle)
    s = np.sin(angle)
    v = 1.0 - c
    x, y, z = axis
    return np.array(
        [
            [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
            [y * x * v + z * s, y * y * v + c, y * z * v - x * s],
            [z * x * v - y * s, z * y * v + x * s, z * z * v + c],
        ],
        dtype=np.float64,
    )


def _load_camera_params(
    config_path: str,
) -> tuple[np.ndarray, np.ndarray, tuple[int, int], np.ndarray]:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    cam = cfg["cam0"]
    intr = cam["intrinsics"]
    fx, fy, cx, cy = float(intr[0]), float(intr[1]), float(intr[2]), float(intr[3])
    K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    dist = np.array(cam.get("distortion_coeffs", [0, 0, 0, 0, 0]), dtype=np.float64)
    res = tuple(cam.get("resolution", [1280, 720]))
    return K, dist, (int(res[0]), int(res[1])), np.array(intr, dtype=np.float64)


def _load_tag_config(config_path: str) -> list[dict]:
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    bodies = cfg.get("bodies", [])
    tags: list[dict] = []
    for body in bodies:
        for _name, bdata in body.items():
            body_tags = bdata.get("tags", [])
            default_size = bdata.get("default_tag_size", 0.1283)
            body_pose = bdata.get("pose", {})
            bp_pos = body_pose.get("position", {"x": 0, "y": 0, "z": 0})
            bp_rot = body_pose.get("rotation", {"x": 0, "y": 0, "z": 0})
            R_body = _axis_angle_to_rot(
                float(bp_rot["x"]), float(bp_rot["y"]), float(bp_rot["z"])
            )
            t_body = np.array(
                [float(bp_pos["x"]), float(bp_pos["y"]), float(bp_pos["z"])]
            )
            for tag in body_tags:
                tid = tag["id"]
                tsize = tag.get("size", default_size)
                tpose = tag.get("pose", {})
                tpos = tpose.get("position", {"x": 0, "y": 0, "z": 0})
                trot = tpose.get("rotation", {"x": 0, "y": 0, "z": 0})
                R_tag_body = _axis_angle_to_rot(
                    float(trot["x"]), float(trot["y"]), float(trot["z"])
                )
                t_tag_body = np.array(
                    [float(tpos["x"]), float(tpos["y"]), float(tpos["z"])]
                )
                R_tag_world = R_body @ R_tag_body
                t_tag_world = R_body @ t_tag_body + t_body
                tags.append(
                    {
                        "id": tid,
                        "size": tsize,
                        "R_tag_to_world": R_tag_world,
                        "t_tag_world": t_tag_world,
                    }
                )
    logger.info("Loaded %d tag(s): %s", len(tags), [t["id"] for t in tags])
    return tags



def _compute_camera_world_pos(
    image: np.ndarray,
    K: np.ndarray,
    dist_coeffs: np.ndarray,
    tag_configs: list[dict],
    detections: list | None = None,
    detector=None,
) -> tuple[float, float, float] | None:
    gray_full = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h_full, w_full = gray_full.shape

    if detections is not None:
        dets = detections
    else:
        if detector is None:
            detector = _make_detector()
        scale = min(1.0, 1920.0 / w_full)
        if scale < 1.0:
            gray = cv2.resize(gray_full, (int(w_full * scale), int(h_full * scale)), interpolation=cv2.INTER_AREA)
        else:
            gray = gray_full
        raw_dets = detector.detect(gray)
        dets = []
        for d in raw_dets:
            if scale < 1.0:
                d.corners[:] /= scale
                d.center[:] /= scale
            dets.append(d)

    if not dets:
        return None

    tag_map = {t["id"]: t for t in tag_configs}
    cam_positions: list[np.ndarray] = []

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    subpix_win = (5, 5) if w_full > 1920 else (3, 3)

    for d in dets:
        tag_id = d.tag_id
        if tag_id not in tag_map:
            continue
        tcfg = tag_map[tag_id]

        corners = d.corners.astype(np.float32).reshape(4, 2)
        cv2.cornerSubPix(gray_full, corners, subpix_win, (-1, -1), criteria)

        s = tcfg["size"]
        obj_pts = np.array(
            [[-s / 2, -s / 2, 0], [s / 2, -s / 2, 0], [s / 2, s / 2, 0], [-s / 2, s / 2, 0]],
            dtype=np.float32,
        )
        ret, rvec, tvec = cv2.solvePnP(obj_pts, corners, K, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ret:
            continue
        R_tc, _ = cv2.Rodrigues(rvec)
        t_tc = tvec.ravel()
        cam_in_tag = -R_tc.T @ t_tc
        cam_in_world = tcfg["R_tag_to_world"] @ cam_in_tag + tcfg["t_tag_world"]
        cam_positions.append(cam_in_world)

    if not cam_positions:
        return None

    avg = np.mean(cam_positions, axis=0)
    return float(avg[0]), float(avg[1]), float(avg[2])


def _fit_line_2sigma(
    points: np.ndarray, max_iter: int = 5
) -> tuple[np.ndarray, np.ndarray, list[int]]:
    mask = np.ones(len(points), dtype=bool)
    for _iter in range(max_iter):
        pts = points[mask]
        if len(pts) < 2:
            break
        centroid = np.mean(pts, axis=0)
        centered = pts - centroid
        cov = centered.T @ centered / len(pts)
        eigvals, eigvecs = np.linalg.eigh(cov)
        direction = eigvecs[:, -1]
        direction /= np.linalg.norm(direction)

        vecs = points - centroid
        dists = np.linalg.norm(np.cross(vecs, direction), axis=1)

        mu = np.mean(dists[mask])
        sigma = np.std(dists[mask])
        threshold = mu + 2.0 * sigma
        new_mask = dists <= threshold
        if np.array_equal(mask, new_mask):
            break
        mask = new_mask

    final_pts = points[mask]
    if len(final_pts) < 2:
        return np.mean(points, axis=0), np.array([1.0, 0.0, 0.0]), list(range(len(points)))

    centroid = np.mean(final_pts, axis=0)
    centered = final_pts - centroid
    cov = centered.T @ centered / len(final_pts)
    _, eigvecs = np.linalg.eigh(cov)
    direction = eigvecs[:, -1]
    direction /= np.linalg.norm(direction)

    inliers = [int(i) for i in np.where(mask)[0]]
    logger.info(
        "Line fit: %d inliers / %d points, dir=[%.4f %.4f %.4f]",
        len(inliers),
        len(points),
        direction[0],
        direction[1],
        direction[2],
    )
    return centroid, direction, inliers


def _point_to_line_dist(
    point: np.ndarray, origin: np.ndarray, direction: np.ndarray
) -> float:
    return float(np.linalg.norm(np.cross(point - origin, direction)))


def _extract_tag_frames(
    video_path: str,
    tag_configs: list[dict],
    K: np.ndarray,
    dist_coeffs: np.ndarray,
    stride: int = 5,
):
    det = _make_detector()

    tag_ids = {t["id"] for t in tag_configs}

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        logger.error("Cannot open video: %s", video_path)
        return

    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps <= 0:
        fps = 30.0
    frame_idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        timestamp = frame_idx / fps
        if frame_idx % stride == 0:
            gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h_full, w_full = gray_full.shape
            scale = min(1.0, 1920.0 / w_full)
            if scale < 1.0:
                gray = cv2.resize(gray_full, (int(w_full * scale), int(h_full * scale)), interpolation=cv2.INTER_AREA)
            else:
                gray = gray_full
            raw_dets = det.detect(gray)
            if any(d.tag_id in tag_ids for d in raw_dets):
                for d in raw_dets:
                    if scale < 1.0:
                        d.corners[:] /= scale
                        d.center[:] /= scale
                pos = _compute_camera_world_pos(frame, K, dist_coeffs, tag_configs, detections=raw_dets)
                if pos is not None:
                    yield frame_idx, timestamp, pos
        frame_idx += 1
    cap.release()
    try:
        det.tag_detector_ptr = None
    except Exception:
        pass


def run_point_analysis(
    photo_dir: str = "data/point",
    video_dir: str = "data/video",
    output_dir: str = "data/loss",
    camera_config: str = "config/cameras.yaml",
    tagslam_config: str = "config/tagslam.yaml",
) -> None:
    import sys as _sys

    _sys.stdout.reconfigure(line_buffering=True) if hasattr(_sys.stdout, "reconfigure") else None

    print("═" * 60)
    print("  Point Cloud → TagSLAM Loss Analysis")
    print("═" * 60)

    K, dist_coeffs, resolution, _fourcc = _load_camera_params(camera_config)
    print(f"\n  Camera  : {resolution[0]}x{resolution[1]}")
    print(f"  K       : fx={K[0, 0]:.2f} fy={K[1, 1]:.2f} cx={K[0, 2]:.2f} cy={K[1, 2]:.2f}")

    tag_configs = _load_tag_config(tagslam_config)
    for t in tag_configs:
        tw = t["t_tag_world"]
        print(
            f"  Tag {t['id']:<4}: size={t['size']:.4f}m"
            f"  pos=({tw[0]:.4f}, {tw[1]:.4f}, {tw[2]:.4f})"
        )

    # ── Phase 1: Process photos ──
    print("\n── Phase 1: Processing photos ──")
    photo_path = Path(photo_dir)
    if not photo_path.exists():
        print(f"  Directory not found: {photo_dir}")
        return
    image_files = sorted(
        p for p in photo_path.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png")
    )
    if not image_files:
        print(f"  No images found in {photo_dir}")
        return
    print(f"  Found {len(image_files)} image(s)")

    photo_points: list[np.ndarray] = []
    photo_names: list[str] = []
    photo_total = len(image_files)

    photo_detector = _make_detector()

    for i, img_path in enumerate(image_files):
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"  [{i + 1}/{photo_total}] SKIP {img_path.name} (unreadable)")
            continue
        pos = _compute_camera_world_pos(img, K, dist_coeffs, tag_configs, detector=photo_detector)
        if pos is None:
            print(f"  [{i + 1}/{photo_total}] SKIP {img_path.name} (no tag detected)")
            continue
        photo_points.append(np.array(pos))
        photo_names.append(img_path.name)
        print(
            f"  [{i + 1}/{photo_total}] {img_path.name}"
            f"  → x={pos[0]:.4f} y={pos[1]:.4f} z={pos[2]:.4f}"
        )

    if len(photo_points) < 2:
        print("  Need at least 2 points to fit a line. Aborting.")
        return

    points_arr = np.array(photo_points)
    origin, direction, inliers = _fit_line_2sigma(points_arr)
    n_in = len(inliers)
    n_out = len(photo_points) - n_in

    print("\n  Line fit result:")
    print(f"    Origin     : ({origin[0]:.4f}, {origin[1]:.4f}, {origin[2]:.4f})")
    print(f"    Direction  : ({direction[0]:.4f}, {direction[1]:.4f}, {direction[2]:.4f})")
    print(f"    Inliers    : {n_in} / {len(photo_points)}  ({n_out} outliers removed)")

    # ── Phase 2: Process videos ──
    print("\n── Phase 2: Processing videos ──")
    video_path = Path(video_dir)
    if not video_path.exists():
        print(f"  Directory not found: {video_dir}")
        return

    video_files = sorted(
        p for p in video_path.iterdir() if p.suffix.lower() in (".mp4", ".avi", ".mov", ".mkv")
    )
    if not video_files:
        print(f"  No videos found in {video_dir}")
        return
    print(f"  Found {len(video_files)} video(s)")

    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, "point_loss.csv")
    fieldnames = ["filename", "timestamp", "x", "y", "z", "distance"]
    csv_file = open(csv_path, "w", newline="")  # noqa: SIM115
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()
    csv_file.flush()
    total_rows = 0

    for vi, vpath in enumerate(video_files):
        vname = vpath.name
        print(f"\n  [{vi + 1}/{len(video_files)}] {vname}")
        tag_frames = 0
        for frame_idx, timestamp, pos in _extract_tag_frames(
            str(vpath), tag_configs, K, dist_coeffs
        ):
            dist = _point_to_line_dist(np.array(pos), origin, direction)
            writer.writerow(
                {
                    "filename": f"{Path(vname).stem}_f{frame_idx:05d}",
                    "timestamp": f"{timestamp:.3f}",
                    "x": f"{pos[0]:.6f}",
                    "y": f"{pos[1]:.6f}",
                    "z": f"{pos[2]:.6f}",
                    "distance": f"{dist:.6f}",
                }
            )
            csv_file.flush()
            total_rows += 1
            tag_frames += 1
            if tag_frames % 50 == 0:
                print(f"    ... {tag_frames} tag frames processed")
        print(f"    Tag frames: {tag_frames}")

    csv_file.close()

    # ── Phase 3: Summary ──
    print(f"\n── Phase 3: Results ──")
    print(f"  Exported {total_rows} rows to {csv_path}")
    print(f"  Photo points   : {len(photo_points)}")
    print(f"  Line inliers   : {n_in}")
    print(f"  Video frames   : {total_rows}")

    if total_rows > 0:
        dists = []
        with open(csv_path) as f:
            reader = csv.DictReader(f)
            for row in reader:
                dists.append(float(row["distance"]))
        dists_arr = np.array(dists)
        print(f"  Distance mean  : {np.mean(dists_arr):.6f} m")
        print(f"  Distance std   : {np.std(dists_arr):.6f} m")
        print(f"  Distance max   : {np.max(dists_arr):.6f} m")
        print(f"  Distance min   : {np.min(dists_arr):.6f} m")

    print("\n  Done.")

    # Suppress exit-time libapriltag cleanup crash (data already saved)
    detectors = [d for d in [photo_detector] if d is not None]
    for d in detectors:
        try:
            d.tag_detector_ptr = None
        except Exception:
            pass
