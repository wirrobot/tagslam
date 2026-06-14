"""Video recording — open camera, SPACE to start/stop recording, saves to data/video/."""

from __future__ import annotations

import logging
import os
import time

import cv2

from tagslam_tools.camera import _resize_display, create_capture

logger = logging.getLogger(__name__)


def _find_video_writer(
    filename: str, fps: float, width: int, height: int
) -> cv2.VideoWriter:
    """Try H.264 encoders in order, fall back to MJPG."""
    encoders = [
        (cv2.VideoWriter.fourcc(*"avc1"), ".mp4"),  # type: ignore[attr-defined]
        (cv2.VideoWriter.fourcc(*"h264"), ".mp4"),  # type: ignore[attr-defined]
        (cv2.VideoWriter.fourcc(*"mp4v"), ".mp4"),  # type: ignore[attr-defined]
        (cv2.VideoWriter.fourcc(*"XVID"), ".avi"),  # type: ignore[attr-defined]
        (cv2.VideoWriter.fourcc(*"MJPG"), ".avi"),  # type: ignore[attr-defined]
    ]
    for fourcc, ext in encoders:
        out_path = filename.rsplit(".", 1)[0] + ext
        writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
        if writer.isOpened():
            return writer
    raise RuntimeError("No suitable video encoder found")


def interactive_video_record(
    save_dir: str = "data/video", fps: float = 30.0, device: int = 0
) -> int:
    os.makedirs(save_dir, exist_ok=True)
    cap = create_capture(device=device)
    if cap is None:
        return 0

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Camera: {width}x{height} @ {fps} fps")
    print(f"Saving to: {os.path.abspath(save_dir)}/")
    print("SPACE = Start/Stop recording  |  ESC/Q = Quit")

    recording = False
    writer: cv2.VideoWriter | None = None
    current_filename = ""
    count = 0
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.warning("Frame read failed")
            time.sleep(0.1)
            continue

        display = frame.copy()
        status_text = ""
        if recording:
            status_text = f"REC {frame_count}"
            cv2.circle(display, (width - 30, 30), 10, (0, 0, 255), -1)
            if writer is not None:
                writer.write(frame)
                frame_count += 1
        else:
            status_text = "IDLE"
            cv2.circle(display, (width - 30, 30), 10, (0, 255, 0), -1)

        cv2.putText(
            display,
            f"Saved: {count} | {status_text} | SPACE=Toggle  ESC=Quit",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        cv2.imshow("Video Record - SPACE to start/stop", _resize_display(display))

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            if recording and writer is not None:
                writer.release()
                count += 1
                print(f"[{count}] Saved: {current_filename}")
            break
        if key == 32:
            if recording:
                if writer is not None:
                    writer.release()
                    count += 1
                    print(f"[{count}] Saved: {current_filename}")
                recording = False
                writer = None
                frame_count = 0
            else:
                current_filename = os.path.join(save_dir, f"video_{count + 1:04d}.mp4")
                try:
                    writer = _find_video_writer(
                        current_filename, fps, width, height
                    )
                    recording = True
                    frame_count = 0
                    print(f"[{count + 1}] Recording: {current_filename}")
                except RuntimeError as e:
                    logger.error("Failed to create video writer: %s", e)

    cap.release()
    cv2.destroyAllWindows()
    print(f"Done. {count} videos saved.")
    return count
