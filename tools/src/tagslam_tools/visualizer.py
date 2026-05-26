"""Real-time SLAM visualization — overlays camera XYZ on the image feed."""

from __future__ import annotations

import logging
import time

import cv2
import numpy as np

logger = logging.getLogger(__name__)

_CAMERA_K = np.array(
    [[1653.275628, 0.0, 648.592971], [0.0, 1654.027585, 355.735559], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
_CAMERA_D = np.zeros(5, dtype=np.float64)  # no distortion assumed for PnP

# Rotation: {x:0, y:1.5708, z:0} → maps tag-local coords to world coords
#  tag z (out of tag) → world x (perpendicular to wall)
_R_TAG_TO_WORLD, _ = cv2.Rodrigues(np.array([0.0, 1.5708, 0.0], dtype=np.float64))

_TAG0_OBJECT_POINTS = np.array(
    [
        [-0.06415, -0.06415, 0.0],
        [0.06415, -0.06415, 0.0],
        [0.06415, 0.06415, 0.0],
        [-0.06415, 0.06415, 0.0],
    ],
    dtype=np.float64,
)


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
      - single-tag → *pose_log_single* (PnP from Tag 0 only)

    Press Q / Esc to quit.
    Requires ROS2 environment sourced.
    """
    import rclpy
    from apriltag_msgs.msg import AprilTagDetectionArray
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
            self._latest_tag0: tuple[np.ndarray, np.ndarray] | None = None
            self._latest_image: np.ndarray | None = None

            self._image_sub = self.create_subscription(Image, image_topic, self._image_callback, 10)
            self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
            self._tag_sub = self.create_subscription(
                AprilTagDetectionArray, tag_topic, self._tag_callback, 10
            )
            self._timer = self.create_timer(0.033, self._render)
            logger.info("Visualizer started (multi + single tag recording)")

        def _image_callback(self, msg: Image) -> None:
            try:
                self._latest_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception:
                logger.exception("Image conversion failed")

        def _odom_callback(self, msg: Odometry) -> None:
            self._latest_odom = msg.pose.pose

        def _tag_callback(self, msg: AprilTagDetectionArray) -> None:
            for det in msg.detections:
                if det.id == 0 and det.family == "tf36h11":
                    corners = np.array([[c.x, c.y] for c in det.corners], dtype=np.float64)
                    ret, rvec, tvec = cv2.solvePnP(  # type: ignore[call-overload]
                        _TAG0_OBJECT_POINTS,
                        corners,
                        _CAMERA_K,
                        _CAMERA_D,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    if ret:
                        # Transform PnP result to world coordinates
                        # tvec = Tag 0 origin in camera frame
                        # Camera in tag frame = -R_cam_tag^T * tvec
                        # Camera in world = R_tag_world * cam_in_tag
                        R_cam_tag, _ = cv2.Rodrigues(rvec)
                        cam_in_tag = -R_cam_tag.T @ tvec.ravel()
                        cam_in_world = (_R_TAG_TO_WORLD @ cam_in_tag).ravel()
                        self._latest_tag0 = (
                            cam_in_world.astype(np.float64),
                            rvec.ravel().astype(np.float64),  # type: ignore[union-attr]
                        )
                    return

        def _render(self) -> None:
            img = self._latest_image
            if img is None:
                return
            display = img.copy()

            panel_w, panel_h = 380, 150
            overlay = display.copy()
            cv2.rectangle(overlay, (8, 8), (8 + panel_w, 8 + panel_h), (0, 0, 0), -1)
            display = cv2.addWeighted(overlay, 0.55, display, 0.45, 0)

            line_y = 36
            if self._latest_odom is not None:
                p = self._latest_odom.position
                cv2.putText(
                    display,
                    "Multi-tag (odom)",
                    (20, line_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (200, 200, 200),
                    1,
                )
                line_y += 20
                cv2.putText(
                    display,
                    f"X:{p.x:+.4f} Y:{p.y:+.4f} Z:{p.z:+.4f}",
                    (20, line_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0),
                    2,
                )
                line_y += 28

            if self._latest_tag0 is not None:
                tvec, _ = self._latest_tag0
                cv2.putText(
                    display,
                    "Single-tag (Tag0 PnP)",
                    (20, line_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (200, 200, 200),
                    1,
                )
                line_y += 20
                cv2.putText(
                    display,
                    f"X:{tvec[0]:+.4f} Y:{tvec[1]:+.4f} Z:{tvec[2]:+.4f}",
                    (20, line_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 200, 0),
                    2,
                )
                line_y += 28

            if self._latest_odom is None and self._latest_tag0 is None:
                cv2.putText(
                    display,
                    "Waiting for pose...",
                    (20, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (100, 100, 255),
                    2,
                )

            cv2.putText(
                display,
                "SPACE: save both poses",
                (20, line_y + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (180, 180, 180),
                1,
            )

            cv2.imshow("TagSLAM Visualizer", display)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord("q"):
                raise KeyboardInterrupt
            if key == 32:
                if self._latest_odom is not None:
                    p = self._latest_odom.position
                    _save_line(_log_multi, "multi", p.x, p.y, p.z)
                if self._latest_tag0 is not None:
                    tvec, _ = self._latest_tag0
                    _save_line(
                        _log_single, "single", float(tvec[0]), float(tvec[1]), float(tvec[2])
                    )

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
