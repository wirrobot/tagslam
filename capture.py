#!/usr/bin/env python3
"""摄像头实时预览 + 空格键拍照保存到 pic/ 目录"""
import os
import cv2
import time

PIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pic")
os.makedirs(PIC_DIR, exist_ok=True)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print(f"Camera: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
print("Press SPACE to capture, ESC or Q to quit")
print(f"Saving to: {PIC_DIR}/")

count = 0
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame read failed")
        time.sleep(0.1)
        continue

    display = frame.copy()
    cv2.putText(display, f"Saved: {count} | SPACE=Capture  ESC=Quit",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("Camera - Press SPACE to capture", display)

    key = cv2.waitKey(1) & 0xFF
    if key == 27 or key == ord("q"):
        break
    elif key == 32:  # Space
        filename = os.path.join(PIC_DIR, f"capture_{count:04d}_{int(time.time())}.jpg")
        cv2.imwrite(filename, frame)
        count += 1
        print(f"[{count}] Saved: {filename}")

cap.release()
cv2.destroyAllWindows()
print(f"Done. {count} images saved.")
