#!/usr/bin/env python3
"""Camera viewer — scan devices, select resolution, live preview with info overlay."""

from __future__ import annotations

import os
import subprocess
import sys

import cv2

SUPPORTED_RES_COMMON = [
    (3840, 2160),
    (1920, 1080),
    (1280, 720),
    (640, 480),
    (640, 360),
]


def scan_cameras() -> dict[int, tuple[int, int]]:
    """Return {device_id: (width, height)} for working cameras."""
    cameras: dict[int, tuple[int, int]] = {}
    files = sorted(
        f for f in os.listdir("/dev") if f.startswith("video") and f[5:].isdigit()
    )
    for f in files:
        device = int(f[5:])
        cap = cv2.VideoCapture(device)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if w > 0 and h > 0:
                cameras[device] = (w, h)
        cap.release()
    return cameras


def get_supported_resolutions(device: int) -> list[tuple[int, int]]:
    """Return supported resolutions via v4l2-ctl or OpenCV caps."""
    seen: set[tuple[int, int]] = set()
    # try OpenCV backend first
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if cap.isOpened():
        for w, h in SUPPORTED_RES_COMMON:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            rw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            rh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if rw > 0 and rh > 0:
                seen.add((rw, rh))
        cap.release()

    if not seen:
        # fallback: ask v4l2-ctl
        try:
            out = subprocess.check_output(
                ["v4l2-ctl", "-d", f"/dev/video{device}", "--list-formats-ext"],
                stderr=subprocess.DEVNULL,
                text=True,
            )
            for line in out.splitlines():
                if "Size: Discrete" in line:
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        try:
                            w, h = int(parts[2]), int(parts[3])
                            seen.add((w, h))
                        except ValueError:
                            pass
        except Exception:
            pass

    return sorted(seen, key=lambda r: r[0] * r[1], reverse=True)


def preview(device: int, width: int | None = None, height: int | None = None) -> None:
    """Show live preview with resolution overlay. Press ESC/Q to quit."""
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"Failed to open /dev/video{device}")
        return

    if width is not None and height is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"\n  Camera /dev/video{device}: {w} x {h} @ {fps:.1f} fps")
    print("  Press ESC or Q to quit\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        display = frame.copy()
        overlay = display.copy()
        panel_h = 72 if w < 3840 else 120  # scale up text for hi-res
        cv2.rectangle(overlay, (8, 8), (280, 8 + panel_h), (0, 0, 0), -1)
        display = cv2.addWeighted(overlay, 0.55, display, 0.45, 0)

        font_scale = 0.55 if w < 3840 else 1.0
        thickness = 2 if w < 3840 else 3
        cv2.putText(
            display, f"/dev/video{device}  {w}x{h}",
            (16, 34 if w < 3840 else 48),
            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 255, 0), thickness,
        )
        cv2.putText(
            display, f"FPS: {fps:.1f}",
            (16, 60 if w < 3840 else 88),
            cv2.FONT_HERSHEY_SIMPLEX, font_scale, (200, 200, 200), thickness - 1,
        )

        if w > 1920:
            display_small = cv2.resize(display, (w // 8, h // 8))
            cv2.imshow("Camera Preview", display_small)
        else:
            cv2.imshow("Camera Preview", display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main() -> None:
    cameras = scan_cameras()
    if not cameras:
        print("No cameras found.")
        sys.exit(1)

    # parse args: [device] [WxH]
    device = None
    target_w, target_h = None, None

    for arg in sys.argv[1:]:
        if arg.isdigit():
            device = int(arg)
        elif "x" in arg.lower():
            parts = arg.lower().split("x")
            if len(parts) == 2 and parts[0].isdigit() and parts[1].isdigit():
                target_w, target_h = int(parts[0]), int(parts[1])

    if device is not None and target_w is not None:
        # direct: ./run_cam_view.sh 1 1920x1080
        preview(device, target_w, target_h)
        return

    # interactive
    print("Available cameras:\n")
    for dev, (w, h) in sorted(cameras.items()):
        print(f"  {dev}  →  /dev/video{dev}  ({w} x {h})")

    print("\nEnter camera ID to preview (or q to quit): ", end="")
    choice = input().strip()
    if choice.lower() == "q":
        return
    if not choice.isdigit() or int(choice) not in cameras:
        print(f"Invalid choice: {choice}")
        return

    device = int(choice)

    # show supported resolutions
    resolutions = get_supported_resolutions(device)
    if resolutions:
        print(f"\nSupported resolutions for /dev/video{device}:\n")
        for i, (w, h) in enumerate(resolutions):
            print(f"  {i + 1}. {w} x {h}")
        print("\nEnter # to pick resolution (or Enter for default): ", end="")
        res_choice = input().strip()
        if res_choice.isdigit() and 1 <= int(res_choice) <= len(resolutions):
            target_w, target_h = resolutions[int(res_choice) - 1]

    preview(device, target_w, target_h)


if __name__ == "__main__":
    main()
