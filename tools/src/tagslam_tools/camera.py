"""Camera utilities — ROS2 publisher and interactive capture tool."""

from __future__ import annotations

import logging
import os
import time

import cv2

logger = logging.getLogger(__name__)


def create_capture(resolution: tuple[int, int] = (1280, 720)) -> cv2.VideoCapture | None:
    """Open the camera at requested resolution. Returns None on failure."""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logger.error("Failed to open camera /dev/video0")
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))  # type: ignore[attr-defined]
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    logger.info("Camera opened: %dx%d", w, h)
    return cap


def publish_camera_loop(image_topic: str = "camera/image_raw", fps: float = 30.0) -> None:
    """Publish camera frames to a ROS2 topic (requires ROS2 env sourced)."""
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    class CameraPublisher(Node):
        def __init__(self) -> None:
            super().__init__("camera_publisher")
            self._bridge = CvBridge()
            self._pub = self.create_publisher(Image, image_topic, 10)
            self._cap = create_capture()
            period = 1.0 / fps if fps > 0 else 0.033
            self._timer = self.create_timer(period, self._publish_frame)

        def _publish_frame(self) -> None:
            if self._cap is None:
                return
            ret, frame = self._cap.read()
            if not ret:
                logger.warning("Failed to read frame", extra={"throttle_sec": 3})
                return
            msg = self._bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            self._pub.publish(msg)

        def destroy_node(self) -> None:
            if self._cap is not None:
                self._cap.release()
            super().destroy_node()

    rclpy.init()
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        logger.exception("Camera publisher error")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


def interactive_capture(save_dir: str = "pic") -> int:
    """Show live preview.  Press SPACE to save frame, ESC/Q to quit."""
    os.makedirs(save_dir, exist_ok=True)
    cap = create_capture()
    if cap is None:
        return 0

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera: {w}x{h}")
    print("Press SPACE to capture, ESC or Q to quit")
    print(f"Saving to: {os.path.abspath(save_dir)}/")

    count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            logger.warning("Frame read failed")
            time.sleep(0.1)
            continue

        display = frame.copy()
        cv2.putText(
            display,
            f"Saved: {count} | SPACE=Capture  ESC=Quit",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.imshow("Camera - Press SPACE to capture", display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
        if key == 32:  # Space
            filename = os.path.join(save_dir, f"capture_{count:04d}_{int(time.time())}.jpg")
            cv2.imwrite(filename, frame)
            count += 1
            print(f"[{count}] Saved: {filename}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"Done. {count} images saved.")
    return count
