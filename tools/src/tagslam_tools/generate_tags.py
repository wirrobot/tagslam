#!/usr/bin/env python3
"""Generate an A4 PDF with two AprilTag markers (tag36h11).

Layout (portrait A4, 210mm × 297mm):
  ┌──────────────────────────┐
  │        margin            │
  │   ┌──────────────────┐   │
  │   │   Tag 0 (large)  │   │  ← ID=0, side = L
  │   └──────────────────┘   │
  │           gap             │  ← gap = S (small tag side)
  │   ┌──────────┐           │
  │   │ Tag 1    │           │  ← ID=1, side = S = 0.5*L
  │   │ (small)  │           │
  │   └──────────┘           │
  │        margin            │
  └──────────────────────────┘

Both tags horizontally and vertically centered.
Maximizes tag size while keeping a white margin.
"""

from __future__ import annotations

import ctypes
import os

import numpy as np
from PIL import Image

# ---------------------------------------------------------------------------
# AprilTag 36h11 pattern extraction via ctypes
# ---------------------------------------------------------------------------

_APRILTAG_LIB = ctypes.CDLL("/opt/ros/humble/lib/x86_64-linux-gnu/libapriltag.so")


class _ApriltagFamily(ctypes.Structure):
    _fields_ = [
        ("ncodes", ctypes.c_uint32),
        ("codes", ctypes.POINTER(ctypes.c_uint64)),
        ("width_at_border", ctypes.c_int),
        ("total_width", ctypes.c_int),
        ("reversed_border", ctypes.c_bool),
        ("nbits", ctypes.c_uint32),
        ("bit_x", ctypes.POINTER(ctypes.c_uint32)),
        ("bit_y", ctypes.POINTER(ctypes.c_uint32)),
        ("h", ctypes.c_int),
        ("name", ctypes.c_char_p),
        ("black_border", ctypes.c_int),
    ]


_APRILTAG_LIB.tag36h11_create.restype = ctypes.POINTER(_ApriltagFamily)


def _get_tag_bits(tag_id: int, family_name: str = "tag36h11") -> np.ndarray:
    """Return a boolean (8,8) array for the given AprilTag ID.

    The tag36h11 family uses an 8×8 grid: 1-pixel black border + 6×6 data bits.
    True = white, False = black.
    """
    if family_name == "tag36h11":
        family = _APRILTAG_LIB.tag36h11_create()
    else:
        raise ValueError(f"Unknown family: {family_name}")

    if tag_id < 0 or tag_id >= family.contents.ncodes:
        raise ValueError(f"Tag ID {tag_id} out of range [0, {family.contents.ncodes})")

    code = family.contents.codes[tag_id]
    bw = int(family.contents.black_border)
    w = int(family.contents.width_at_border)
    bits = np.zeros((w, w), dtype=bool)

    nbits = int(family.contents.nbits)
    for i in range(nbits):
        bit_val = (code >> (nbits - 1 - i)) & 1
        x = int(family.contents.bit_x[i])
        y = int(family.contents.bit_y[i])
        bits[y, x] = bool(bit_val)

    full = np.ones((w + 2 * bw, w + 2 * bw), dtype=bool)
    full[bw : bw + w, bw : bw + w] = bits

    return full


# ---------------------------------------------------------------------------
# A4 PDF generation
# ---------------------------------------------------------------------------

A4_MM = (210, 297)
DPI = 300
MM_TO_INCH = 1 / 25.4
INCH_TO_PX = DPI
MM_TO_PX = MM_TO_INCH * INCH_TO_PX


def _bits_to_image(bits: np.ndarray, pixel_size: int) -> Image.Image:
    """Scale up a boolean array to an image with given pixel_size per cell."""
    h, w = bits.shape
    img = Image.new("L", (w * pixel_size, h * pixel_size), 255)
    for y in range(h):
        for x in range(w):
            if not bits[y, x]:
                for dy in range(pixel_size):
                    for dx in range(pixel_size):
                        img.putpixel((x * pixel_size + dx, y * pixel_size + dy), 0)
    return img


def generate(out_path: str = "apriltags_a4.pdf") -> str:
    margin_mm = 15.0  # white margin on all sides

    usable_w_mm = A4_MM[0] - 2 * margin_mm  # 180mm
    usable_h_mm = A4_MM[1] - 2 * margin_mm  # 267mm

    # L = large tag side, S = small tag side = 0.5*L
    # total block height = L + S (gap) + S = L + 0.5L + 0.5L = 2L
    # constraints: L <= usable_w, 2L <= usable_h
    large_mm = min(usable_w_mm, usable_h_mm / 2)
    small_mm = large_mm / 2
    gap_mm = small_mm

    page_w_px = int(A4_MM[0] * MM_TO_PX)
    page_h_px = int(A4_MM[1] * MM_TO_PX)

    canvas = Image.new("RGB", (page_w_px, page_h_px), "white")

    large_px = int(large_mm * MM_TO_PX)
    small_px = int(small_mm * MM_TO_PX)
    gap_px = int(gap_mm * MM_TO_PX)

    block_h_px = large_px + gap_px + small_px
    block_top_px = (page_h_px - block_h_px) // 2

    # Tag 0 (large, ID=0) — top
    bits0 = _get_tag_bits(0)
    tag0_img = _bits_to_image(bits0, max(1, large_px // 8))
    tag0_resized = tag0_img.resize((large_px, large_px), Image.Resampling.NEAREST)
    x0 = (page_w_px - large_px) // 2
    y0 = block_top_px
    canvas.paste(tag0_resized, (x0, y0))

    # Tag 1 (small, ID=1) — bottom
    bits1 = _get_tag_bits(1)
    tag1_img = _bits_to_image(bits1, max(1, small_px // 8))
    tag1_resized = tag1_img.resize((small_px, small_px), Image.Resampling.NEAREST)
    x1 = (page_w_px - small_px) // 2
    y1 = block_top_px + large_px + gap_px
    canvas.paste(tag1_resized, (x1, y1))

    out_path = os.path.abspath(
        out_path if os.path.isabs(out_path) else os.path.join(os.getcwd(), out_path)
    )
    canvas.save(out_path, "PDF", resolution=DPI)
    print(f"Saved: {out_path}")
    print(f"  Page      : {A4_MM[0]}×{A4_MM[1]} mm (A4 portrait)")
    print(f"  Large tag : ID=0, {large_mm:.1f}×{large_mm:.1f} mm")
    print(f"  Small tag : ID=1, {small_mm:.1f}×{small_mm:.1f} mm")
    print(f"  Gap       : {gap_mm:.1f} mm")
    print(f"  Margin    : {margin_mm:.1f} mm")
    block_top_mm = margin_mm + (usable_h_mm - large_mm - gap_mm - small_mm) / 2
    print(f"  Block top : {block_top_mm:.1f} mm from top")
    return out_path


if __name__ == "__main__":
    generate()
