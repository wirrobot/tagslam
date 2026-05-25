#!/usr/bin/env python3
"""棋盘格标定脚本 — 从 pic/ 目录读取图片，计算内参和畸变系数"""
import os
import glob
import cv2
import numpy as np

CHESSBOARD = (11, 8)     # 内角点数 (cols, rows)
SQUARE_SIZE = 0.015      # 每格边长(米)，请根据实际棋盘格修改

PIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pic")
images = sorted(glob.glob(os.path.join(PIC_DIR, "*.jpg")))

if len(images) < 10:
    print(f"ERROR: Need at least 10 images, found {len(images)}")
    exit(1)

objp = np.zeros((CHESSBOARD[0] * CHESSBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD[0], 0:CHESSBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

obj_points = []
img_points = []
img_size = None

for f in images:
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if img_size is None:
        img_size = gray.shape[::-1]

    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD, None)
    if ret:
        obj_points.append(objp)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        img_points.append(corners2)
        print(f"  OK  {os.path.basename(f)}")
    else:
        print(f"  SKIP {os.path.basename(f)} (no chessboard found)")

print(f"\nFound chessboard in {len(obj_points)} / {len(images)} images")

if len(obj_points) < 5:
    print("ERROR: Not enough valid images for calibration")
    exit(1)

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, img_size, None, None)

mean_err = 0
for i in range(len(obj_points)):
    proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
    mean_err += cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
mean_err /= len(obj_points)

print(f"\n{'='*50}")
print(f"RMS reprojection error: {mean_err:.4f} px")
print(f"Image size: {img_size[0]} x {img_size[1]}")
print(f"\nCamera Matrix:")
print(f"  fx={K[0,0]:.6f}  fy={K[1,1]:.6f}")
print(f"  cx={K[0,2]:.6f}  cy={K[1,2]:.6f}")
print(f"\n  K = [[{K[0,0]:.6f}, 0.0, {K[0,2]:.6f}],")
print(f"       [0.0, {K[1,1]:.6f}, {K[1,2]:.6f}],")
print(f"       [0.0, 0.0, 1.0]]")
print(f"\nDistortion ({len(dist.ravel())} coeffs):")
print(f"  {np.array2string(dist.ravel(), separator=', ', precision=8)}")
print(f"{'='*50}")
