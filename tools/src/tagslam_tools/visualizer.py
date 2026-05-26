"""Real-time SLAM visualization — overlays camera XYZ on the image feed."""

from __future__ import annotations

import logging
import time

import cv2
import numpy as np

logger = logging.getLogger(__name__)

# Camera intrinsics (from calibration)
_CAM_PARAMS = [1653.275628, 1654.027585, 648.592971, 355.735559]
_TAG0_SIZE = 0.1283


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
            """Run apriltag detection on current frame, return Tag 0 pose_t."""
            gray = self._latest_gray
            if gray is None:
                return None
            try:
                from apriltag import Detector, DetectorOptions

                opt = DetectorOptions(families="tag36h11")
                det = Detector(options=opt)
                dets = det.detect(gray)
                for d in dets:
                    if d.tag_id == 0:
                        pose, _, _ = det.detection_pose(
                            d, camera_params=_CAM_PARAMS, tag_size=_TAG0_SIZE
                        )
                        t = pose[:3, 3]
                        return (float(t[0]), float(t[1]), float(t[2]))
            except Exception:
                logger.exception("Single-tag detection failed")
            return None

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
                tx, ty, tz = self._latest_tag0
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
                    f"X:{tx:+.4f} Y:{ty:+.4f} Z:{tz:+.4f}",
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
