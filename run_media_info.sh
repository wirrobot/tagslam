#!/usr/bin/env python3
"""Media info viewer — show resolution of images and videos, plus video frame count."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import cv2


def scan_images(directory: str) -> list[dict]:
    """Return [{path, width, height}] for all images in directory."""
    results = []
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif"}
    for p in sorted(Path(directory).iterdir()):
        if p.suffix.lower() not in exts:
            continue
        img = cv2.imread(str(p))
        if img is not None:
            h, w = img.shape[:2]
            results.append({"path": p.name, "width": w, "height": h, "size_mb": p.stat().st_size / 1e6})
    return results


def scan_videos(directory: str) -> list[dict]:
    """Return [{path, width, height, frames, fps, duration_s}] for all videos."""
    results = []
    exts = {".mp4", ".avi", ".mov", ".mkv", ".webm"}
    for p in sorted(Path(directory).iterdir()):
        if p.suffix.lower() not in exts:
            continue
        cap = cv2.VideoCapture(str(p))
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            fps = cap.get(cv2.CAP_PROP_FPS)
            duration = frames / fps if fps > 0 else 0
            cap.release()
            results.append({
                "path": p.name,
                "width": w,
                "height": h,
                "frames": frames,
                "fps": fps,
                "duration_s": duration,
                "size_mb": p.stat().st_size / 1e6,
            })
    return results


def main() -> None:
    img_dir = sys.argv[1] if len(sys.argv) > 1 else "data/point"
    vid_dir = sys.argv[2] if len(sys.argv) > 2 else "data/video"

    sep = "─" * 64

    # ── Images ──
    if Path(img_dir).is_dir():
        imgs = scan_images(img_dir)
        print(f"\n{sep}")
        print(f"  Images in {img_dir}/  ({len(imgs)} files)")
        print(sep)
        if imgs:
            print(f"  {'File':<30} {'Resolution':<14} {'Size'}")
            for r in imgs:
                print(f"  {r['path']:<30} {r['width']} x {r['height']:<8} {r['size_mb']:.2f} MB")
        else:
            print("  (no images found)")
    else:
        print(f"\n  Directory not found: {img_dir}")

    # ── Videos ──
    if Path(vid_dir).is_dir():
        vids = scan_videos(vid_dir)
        print(f"\n{sep}")
        print(f"  Videos in {vid_dir}/  ({len(vids)} files)")
        print(sep)
        if vids:
            print(f"  {'File':<30} {'Resolution':<14} {'Frames':<8} {'FPS':<7} {'Dur (s)':<9} {'Size'}")
            for r in vids:
                print(
                    f"  {r['path']:<30} {r['width']} x {r['height']:<8} "
                    f"{r['frames']:<8} {r['fps']:<6.1f} {r['duration_s']:<8.1f} {r['size_mb']:.2f} MB"
                )
            if vids:
                total_frames = sum(r["frames"] for r in vids)
                total_dur = sum(r["duration_s"] for r in vids)
                print(f"  {'':-<30} {'':-<14} {'':-<8} {'':-<7} {'':-<9} {'':-<6}")
                print(f"  {'TOTAL':<30} {'':<14} {total_frames:<8} {'':<7} {total_dur:<9.1f} {'':<6}")
        else:
            print("  (no videos found)")
    else:
        print(f"\n  Directory not found: {vid_dir}")

    print()


if __name__ == "__main__":
    main()
