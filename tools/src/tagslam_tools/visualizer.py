"""Real-time SLAM visualization — overlays camera XYZ on the image feed."""

from __future__ import annotations

import logging

import cv2
import numpy as np

logger = logging.getLogger(__name__)


def run_visualizer(
    image_topic: str = "camera/image_raw",
    odom_topic: str = "/odom/body_rig",
) -> None:
    """Launch a window displaying the camera feed with overlaid pose coordinates.

    Requires ROS2 environment sourced.
    """
    import rclpy
    from cv_bridge import CvBridge
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class Visualizer(Node):
        def __init__(self) -> None:
            super().__init__("tagslam_visualizer")
            self._bridge = CvBridge()
            self._latest_pose = None
            self._latest_image: np.ndarray | None = None

            self._image_sub = self.create_subscription(Image, image_topic, self._image_callback, 10)
            self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_callback, 10)
            self._timer = self.create_timer(0.033, self._render)
            logger.info("Visualizer started, waiting for image & odom...")

        def _image_callback(self, msg: Image) -> None:
            try:
                self._latest_image = self._bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception:
                logger.exception("Image conversion failed")

        def _odom_callback(self, msg: Odometry) -> None:
            self._latest_pose = msg.pose.pose

        def _render(self) -> None:
            img = self._latest_image
            if img is None:
                return
            display = img.copy()

            panel_w, panel_h = 360, 100
            overlay = display.copy()
            cv2.rectangle(overlay, (8, 8), (8 + panel_w, 8 + panel_h), (0, 0, 0), -1)
            display = cv2.addWeighted(overlay, 0.55, display, 0.45, 0)

            if self._latest_pose is not None:
                p = self._latest_pose.position
                x, y, z = p.x, p.y, p.z
                cv2.putText(
                    display,
                    "Camera XYZ (world frame)",
                    (20, 36),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (200, 200, 200),
                    1,
                )
                cv2.putText(
                    display,
                    f"X: {x:+.4f} m",
                    (20, 62),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    display,
                    f"Y: {y:+.4f} m",
                    (20, 82),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    display,
                    f"Z: {z:+.4f} m",
                    (20, 102),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.60,
                    (255, 100, 100),
                    2,
                )
            else:
                cv2.putText(
                    display,
                    "Waiting for pose...",
                    (20, 55),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (100, 100, 255),
                    2,
                )

            cv2.imshow("TagSLAM Visualizer", display)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                raise KeyboardInterrupt

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
