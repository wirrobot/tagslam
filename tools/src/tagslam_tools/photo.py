"""Point cloud photo capture — open camera, SPACE to save frames to data/point/."""

from __future__ import annotations

import logging
import os
import time

import cv2

from tagslam_tools.camera import _resize_display, create_capture

logger = logging.getLogger(__name__)


def interactive_photo_capture(save_dir: str = "data/point", device: int = 0) -> int:
    os.makedirs(save_dir, exist_ok=True)
    cap = create_capture(device=device)
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
        cv2.imshow("Point Photo - SPACE to capture", _resize_display(display))

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break
        if key == 32:
            count += 1
            filename = os.path.join(save_dir, f"point_{count:04d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"[{count}] Saved: {filename}")

    cap.release()
    cv2.destroyAllWindows()
    print(f"Done. {count} photos saved.")
    return count
