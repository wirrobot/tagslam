"""Camera utilities — ROS2 publisher and interactive capture tool."""

from __future__ import annotations

import logging
import os
import time
from pathlib import Path

import cv2
import numpy as np

logger = logging.getLogger(__name__)

_DEFAULT_RESOLUTION = (1280, 720)
_MAX_DISPLAY_SIZE = (960, 540)


def _resize_display(img: np.ndarray) -> np.ndarray:
    """Downscale image for display if it exceeds max dimensions."""
    h, w = img.shape[:2]
    max_w, max_h = _MAX_DISPLAY_SIZE
    if w <= max_w and h <= max_h:
        return img
    scale = min(max_w / w, max_h / h)
    new_w, new_h = int(w * scale), int(h * scale)
    return cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)


def _read_config_resolution() -> tuple[int, int]:
    """Read camera resolution from config/cameras.yaml, fall back to default."""
    try:
        from yaml import safe_load

        config_path = Path(os.environ.get("TAGSLAM_CONFIG_DIR", str(Path(__file__).resolve().parent.parent.parent.parent / "config")))
        with open(config_path / "cameras.yaml") as f:
            cfg = safe_load(f)
        for cam in cfg.values():
            if isinstance(cam, dict) and "resolution" in cam:
                w, h = cam["resolution"]
                return (int(w), int(h))
    except Exception:
        logger.debug("Could not read resolution from config, using default")
    return _DEFAULT_RESOLUTION


def create_capture(
    resolution: tuple[int, int] | None = None, device: int = 0
) -> cv2.VideoCapture | None:
    """Open the camera at requested resolution. Uses config if resolution not given."""
    if resolution is None:
        resolution = _read_config_resolution()
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        available = sorted(int(f[5:]) for f in os.listdir("/dev") if f.startswith("video") and f[5:].isdigit())
        logger.error("Failed to open /dev/video%d", device)
        logger.error("Available devices: %s", [f"/dev/video{i}" for i in available] if available else "(none)")
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*"MJPG"))  # type: ignore[attr-defined]
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    logger.info("Camera opened: %dx%d", w, h)
    return cap


def publish_camera_loop(
    image_topic: str = "camera/image_raw",
    fps: float = 30.0,
    device: int = 0,
    width: int | None = None,
    height: int | None = None,
) -> None:
    """Publish camera frames to a ROS2 topic (requires ROS2 env sourced)."""
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    if width is not None and height is not None:
        resolution = (width, height)
    else:
        resolution = None

    class CameraPublisher(Node):
        def __init__(self) -> None:
            super().__init__("camera_publisher")
            self._bridge = CvBridge()
            self._pub = self.create_publisher(Image, image_topic, 10)
            self._cap = create_capture(resolution=resolution, device=device)
            period = 1.0 / fps if fps > 0 else 0.033
            self._timer = self.create_timer(period, self._publish_frame)

        def _publish_frame(self) -> None:
            if self._cap is None:
                return
            try:
                ret, frame = self._cap.read()
            except cv2.error:
                logger.warning("Corrupt frame, skipping", extra={"throttle_sec": 3})
                return
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


def interactive_capture(
    save_dir: str = "pic", device: int = 0, width: int | None = None, height: int | None = None
) -> int:
    """Show live preview.  Press SPACE to save frame, ESC/Q to quit."""
    os.makedirs(save_dir, exist_ok=True)
    resolution = (width, height) if (width is not None and height is not None) else None
    cap = create_capture(resolution=resolution, device=device)
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
        cv2.imshow("Camera - Press SPACE to capture", _resize_display(display))

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
