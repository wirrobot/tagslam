#!/usr/bin/env python3
"""
TagSLAM 可视化调试工具
订阅摄像头画面和 SLAM 位姿结果，在图像左上角叠加显示 xyz 坐标
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


class Visualizer(Node):
    def __init__(self):
        super().__init__("tagslam_visualizer")
        self.bridge = CvBridge()
        self.latest_pose = None
        self.latest_image = None

        self.image_sub = self.create_subscription(
            Image, "camera/image_raw", self.image_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/tagslam/odom/body_rig", self.odom_callback, 10
        )

        self.timer = self.create_timer(0.033, self.render)  # ~30 fps
        self.get_logger().info("Visualizer started, waiting for image & odom...")

    def image_callback(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def odom_callback(self, msg: Odometry):
        self.latest_pose = msg.pose.pose

    def render(self):
        img = self.latest_image
        if img is None:
            return

        display = img.copy()
        h, w = display.shape[:2]

        # ── 半透明左上角信息面板 ──
        panel_w, panel_h = 360, 100
        overlay = display.copy()
        cv2.rectangle(overlay, (8, 8), (8 + panel_w, 8 + panel_h), (0, 0, 0), -1)
        display = cv2.addWeighted(overlay, 0.55, display, 0.45, 0)

        if self.latest_pose is not None:
            p = self.latest_pose.position
            x, y, z = p.x, p.y, p.z

            cv2.putText(display, f"Camera XYZ (world frame)",
                        (20, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
            cv2.putText(display, f"X: {x:+.4f} m", (20, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2)
            cv2.putText(display, f"Y: {y:+.4f} m", (20, 82),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 255), 2)
            cv2.putText(display, f"Z: {z:+.4f} m", (20, 102),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 100, 100), 2)
        else:
            cv2.putText(display, "Waiting for pose...",
                        (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 100, 255), 2)

        cv2.imshow("TagSLAM Visualizer", display)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            raise KeyboardInterrupt

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
