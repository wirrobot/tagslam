"""Real-time SLAM visualization — overlays camera XYZ on the image feed."""

from __future__ import annotations

import logging
import time

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# Camera intrinsics — read from config/cameras.yaml on first use
_CAM_PARAMS: list[float] | None = None
_TAG0_SIZE = 0.1283


def _read_camera_params() -> list[float]:
    """Read camera intrinsics from config/cameras.yaml."""
    import os
    from pathlib import Path

    try:
        from yaml import safe_load

        config_dir = Path(os.environ.get("TAGSLAM_CONFIG_DIR", str(Path(__file__).resolve().parent.parent.parent.parent / "config")))
        with open(config_dir / "cameras.yaml") as f:
            cfg = safe_load(f)
        for cam in cfg.values():
            if isinstance(cam, dict) and "intrinsics" in cam:
                return [float(v) for v in cam["intrinsics"]]
    except Exception:
        logger.debug("Could not read intrinsics from config, using fallback")
    return [799.8855, 800.2863, 636.4467, 353.9479]


def _read_distortion_coeffs() -> np.ndarray:
    """Read distortion coefficients from config/cameras.yaml."""
    import os
    from pathlib import Path

    try:
        from yaml import safe_load

        config_dir = Path(os.environ.get("TAGSLAM_CONFIG_DIR", str(Path(__file__).resolve().parent.parent.parent.parent / "config")))
        with open(config_dir / "cameras.yaml") as f:
            cfg = safe_load(f)
        for cam in cfg.values():
            if isinstance(cam, dict) and "distortion_coeffs" in cam:
                return np.array([float(v) for v in cam["distortion_coeffs"]], dtype=np.float64)
    except Exception:
        logger.debug("Could not read distortion from config, using zeros")
    return np.zeros(5, dtype=np.float64)


_CAM_K: np.ndarray | None = None
_CAM_DIST: np.ndarray | None = None


def _get_camera_params() -> list[float]:
    global _CAM_PARAMS
    if _CAM_PARAMS is None:
        _CAM_PARAMS = _read_camera_params()
    return _CAM_PARAMS


def _get_camera_K() -> np.ndarray:
    global _CAM_K
    if _CAM_K is None:
        intr = _get_camera_params()
        fx, fy, cx, cy = intr
        _CAM_K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
    return _CAM_K


def _get_camera_dist() -> np.ndarray:
    global _CAM_DIST
    if _CAM_DIST is None:
        _CAM_DIST = _read_distortion_coeffs()
    return _CAM_DIST

# Rotation: {x:0, y:1.5708, z:0} maps tag-z → world-x, tag-y → world-y, tag-x → world-z
_R_TAG_TO_WORLD = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)


_MAX_DISPLAY_SIZE = (480, 270)


def _resize_display(img: np.ndarray) -> np.ndarray:
    """Downscale image for display if it exceeds max dimensions."""
    h, w = img.shape[:2]
    max_w, max_h = _MAX_DISPLAY_SIZE
    if w <= max_w and h <= max_h:
        return img
    scale = min(max_w / w, max_h / h)
    new_w, new_h = int(w * scale), int(h * scale)
    return cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)


def run_visualizer(
    image_topic: str = "camera/image_raw",
    odom_topic: str = "/odom/body_rig",
    tag_topic: str = "/detector/tags",
    pose_log_multi: str = "pose_log_multi.txt",
    pose_log_single: str = "pose_log_single.txt",
) -> None:
    """Visualize SLAM pose with dual recording (multi-tag vs single-tag).

    Press SPACE to save BOTH poses simultaneously:
      - multi-tag  → *pose_log_multi* (from /odom/body_rig)
      - single-tag → *pose_log_single* (Tag 0 PnP via apriltag library)

    Press Q / Esc to quit.
    Requires ROS2 environment sourced.
    """
    import rclpy
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    _log_multi = open(pose_log_multi, "a")  # noqa: SIM115
    _log_single = open(pose_log_single, "a")  # noqa: SIM115

    def _save_line(fh, label: str, x: float, y: float, z: float) -> None:
        ts = time.time()
        fh.write(f"{ts:.6f} x={x:.6f} y={y:.6f} z={z:.6f}\n")
        fh.flush()
        logger.info("%s saved: x=%.3f y=%.3f z=%.3f", label, x, y, z)

    class Visualizer(Node):
        def __init__(self) -> None:
            super().__init__("tagslam_visualizer")
            self._bridge = CvBridge()
            self._latest_odom = None
            self._latest_tag0: tuple[float, float, float] | None = None
            self._latest_image: np.ndarray | None = None
            self._latest_gray: np.ndarray | None = None

            self._image_sub = self.create_subscription(Image, image_topic, self._image_callback, 10)
            self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
            self._timer = self.create_timer(0.033, self._render)
            logger.info("Visualizer started (multi + single tag recording)")

        def _image_callback(self, msg: Image) -> None:
            try:
                bgr = self._bridge.imgmsg_to_cv2(msg, "bgr8")
                self._latest_image = bgr
                self._latest_gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            except Exception:
                logger.exception("Image conversion failed")

        def _odom_callback(self, msg: Odometry) -> None:
            self._latest_odom = msg.pose.pose

        def _compute_single_tag_pose(self) -> tuple[float, float, float] | None:
            """Run apriltag detection on current frame, return Tag 0 camera pos in world frame."""
            gray = self._latest_gray
            if gray is None:
                return None
            try:
                import os as _os
                import sys as _sys

                _saved = list(_sys.path)
                _sys.path = [p for p in _sys.path if "ros" not in p.lower() and "opt/ros" not in p]
                for _p in [
                    _os.path.expanduser("~/miniconda3/lib/python3.13/site-packages"),
                    _os.path.expanduser("~/miniconda3/lib/python3.10/site-packages"),
                    "/usr/lib/python3/dist-packages",
                ]:
                    if _os.path.isdir(_p) and _p not in _sys.path:
                        _sys.path.insert(0, _p)
                try:
                    from pupil_apriltags import Detector

                    det = Detector(
                        families="tag36h11",
                        quad_decimate=2.0,
                        searchpath=(Path("/opt/ros/humble/lib/x86_64-linux-gnu"),),
                    )
                finally:
                    _sys.path = _saved
                detections = det.detect(gray)
                for d in detections:
                    if d.tag_id == 0:
                        corners = d.corners.astype(np.float32).reshape(4, 2)
                        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
                        cv2.cornerSubPix(gray, corners, (3, 3), (-1, -1), criteria)

                        s = _TAG0_SIZE
                        obj_pts = np.array(
                            [[-s / 2, -s / 2, 0], [s / 2, -s / 2, 0], [s / 2, s / 2, 0], [-s / 2, s / 2, 0]],
                            dtype=np.float32,
                        )
                        ret, rvec, tvec = cv2.solvePnP(
                            obj_pts, corners, _get_camera_K(), _get_camera_dist(), flags=cv2.SOLVEPNP_ITERATIVE
                        )
                        if not ret:
                            continue
                        R_tc, _ = cv2.Rodrigues(rvec)
                        t_tc = tvec.ravel()
                        cam_in_tag = -R_tc.T @ t_tc
                        cam_in_world = _R_TAG_TO_WORLD @ cam_in_tag
                        return (
                            float(cam_in_world[0]),
                            float(cam_in_world[1]),
                            float(cam_in_world[2]),
                        )
            except Exception:
                logger.exception("Single-tag detection failed")
            return None

        def _render(self) -> None:
            img = self._latest_image
            if img is None:
                return
            display = img.copy()

            h, w = display.shape[:2]
            scale = max(1.0, min(w / 1280, h / 720))

            panel_w, panel_h = int(380 * scale), int(180 * scale)
            overlay = display.copy()
            cv2.rectangle(overlay, (8, 8), (8 + panel_w, 8 + panel_h), (0, 0, 0), -1)
            display = cv2.addWeighted(overlay, 0.55, display, 0.45, 0)

            line_y = int(36 * scale)
            cv2.putText(
                display,
                f"{w}x{h}",
                (int(20 * scale), line_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45 * scale,
                (180, 180, 180),
                int(max(1, scale)),
            )
            line_y += int(22 * scale)
            if self._latest_odom is not None:
                p = self._latest_odom.position
                cv2.putText(
                    display,
                    "Multi-tag (odom)",
                    (int(20 * scale), line_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50 * scale,
                    (200, 200, 200),
                    int(max(1, scale)),
                )
                line_y += int(20 * scale)
                cv2.putText(
                    display,
                    f"X:{p.x:+.4f} Y:{p.y:+.4f} Z:{p.z:+.4f}",
                    (int(20 * scale), line_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55 * scale,
                    (0, 255, 0),
                    int(max(1, 2 * scale)),
                )
                line_y += int(28 * scale)

            if self._latest_tag0 is not None:
                tx, ty, tz = self._latest_tag0
                cv2.putText(
                    display,
                    "Single-tag (Tag0 PnP)",
                    (int(20 * scale), line_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50 * scale,
                    (200, 200, 200),
                    int(max(1, scale)),
                )
                line_y += int(20 * scale)
                cv2.putText(
                    display,
                    f"X:{tx:+.4f} Y:{ty:+.4f} Z:{tz:+.4f}",
                    (int(20 * scale), line_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55 * scale,
                    (255, 200, 0),
                    int(max(1, 2 * scale)),
                )
                line_y += int(28 * scale)

            if self._latest_odom is None and self._latest_tag0 is None:
                cv2.putText(
                    display,
                    "Waiting for pose...",
                    (int(20 * scale), int(55 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7 * scale,
                    (100, 100, 255),
                    int(max(1, 2 * scale)),
                )

            cv2.putText(
                display,
                "SPACE: save both poses",
                (int(20 * scale), line_y + int(4 * scale)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45 * scale,
                (180, 180, 180),
                int(max(1, scale)),
            )

            cv2.imshow("TagSLAM Visualizer", _resize_display(display))
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                raise KeyboardInterrupt
            if key == 32:
                if self._latest_odom is not None:
                    p = self._latest_odom.position
                    _save_line(_log_multi, "multi", p.x, p.y, p.z)
                single = self._compute_single_tag_pose()
                if single is not None:
                    self._latest_tag0 = single
                    _save_line(_log_single, "single", single[0], single[1], single[2])

        def destroy_node(self) -> None:
            cv2.destroyAllWindows()
            super().destroy_node()

    rclpy.init()
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        logger.exception("Visualizer error")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        _log_multi.close()
        _log_single.close()
