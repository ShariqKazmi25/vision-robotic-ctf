\#!/usr/bin/env python3
"""
detect\_robot\_cube\_depth\_bearing\_relaxed.py

Detects the red acrylic “cube” on your RoboMaster with some tolerance and smoothing:

1. Masks red via HSV
2. Finds all red contours, uses rotated minAreaRect for best shape match
3. Computes real‐world size, distance & bearing
4. Allows a dynamic tolerance that grows with distance
5. Smooths the output over several frames for stability

Dependencies:
pip install pyrealsense2 opencv-python numpy
"""

import pyrealsense2 as rs
import numpy as np
import cv2
from collections import deque

# ─── CONFIG ─────────────────────────────────────────────────────────────────────

# HSV thresholds for red (looser)

LOWER\_RED1 = np.array(\[0, 100, 100])
UPPER\_RED1 = np.array(\[10, 255, 255])
LOWER\_RED2 = np.array(\[160, 100, 100])
UPPER\_RED2 = np.array(\[180, 255, 255])

# Morphology

KERNEL\_OPEN  = cv2.getStructuringElement(cv2.MORPH\_RECT, (5,5))
KERNEL\_CLOSE = cv2.getStructuringElement(cv2.MORPH\_RECT, (7,7))

# Stream settings

WIDTH, HEIGHT, FPS = 640, 480, 30

# Real‐world target square face size (meters)

TARGET\_SIZE = 0.14    # 14 cm

# Camera horizontal FOV (degrees) & focal length (px)

H\_FOV\_DEG    = 87.0
FOCAL\_LENGTH = WIDTH / (2 \* np.tan(np.deg2rad(H\_FOV\_DEG/2)))
ANGLE\_PER\_PX = H\_FOV\_DEG / WIDTH

# Minimum contour area (px²)

MIN\_AREA = 1000

# Smoothing window

SMOOTH\_W = 5

# ────────────────────────────────────────────────────────────────────────────────

def dynamic\_tolerance(dist):
"""
Returns an allowed size‐error tolerance based on distance (meters).
Starts at 3cm and grows by 2cm per meter of distance.
"""
return 0.03 + 0.02 \* dist  # meters

def main():
\# frame‐to‐frame smoothing
dist\_buf  = deque(maxlen=SMOOTH\_W)
bear\_buf  = deque(maxlen=SMOOTH\_W)


# 1) Start RealSense
pipeline = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
pipeline.start(cfg)

# 2) Align depth to color
align = rs.align(rs.stream.color)

try:
    print("Streaming… press Ctrl+C to stop")
    while True:
        frames      = pipeline.wait_for_frames()
        aligned     = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # 3) Acquire and preprocess
        color = np.asanyarray(color_frame.get_data())
        hsv   = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        # Red mask
        m1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
        m2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
        mask = cv2.bitwise_or(m1, m2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  KERNEL_OPEN)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL_CLOSE)

        # 4) Find contours
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_err = float("inf")

        for c in cnts:
            area = cv2.contourArea(c)
            if area < MIN_AREA:
                continue

            # 5) Rotated rectangle fit
            rect = cv2.minAreaRect(c)
            ((cx, cy), (pw, ph), ang) = rect
            # pixel dims
            w_px, h_px = max(pw, ph), min(pw, ph)
            if w_px == 0 or h_px == 0:
                continue

            # 6) Depth at center
            dist = depth_frame.get_distance(int(cx), int(cy))
            if dist <= 0:
                continue

            # 7) Real‐world dims
            real_w = (w_px * dist) / FOCAL_LENGTH
            real_h = (h_px * dist) / FOCAL_LENGTH

            # 8) Compute error vs target
            err = abs(real_w - TARGET_SIZE) + abs(real_h - TARGET_SIZE)
            tol = dynamic_tolerance(dist) * 2  # allow 2× tol across two dims
            if err < best_err:
                best_err = err
                best = (rect, dist, real_w, real_h)

        # 9) If best match within tol, calculate bearing & annotate
        if best is not None and best_err < tol:
            (rect, dist, real_w, real_h) = best
            (cx, cy), (pw, ph), ang = rect
            # bearing
            dx = cx - (WIDTH/2)
            bearing = dx * ANGLE_PER_PX

            # smoothing
            dist_buf.append(dist)
            bear_buf.append(bearing)
            sdist = sum(dist_buf) / len(dist_buf)
            sbear = sum(bear_buf) / len(bear_buf)

            # draw rotated rect
            box = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(color, [box], -1, (0,255,0), 2)
            cv2.circle(color, (int(cx), int(cy)), 4, (255,0,0), -1)
            cv2.putText(color, f"D:{sdist:.2f}m", (int(cx), int(cy)-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            cv2.putText(color, f"B:{sbear:+.1f}°", (int(cx), int(cy)-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            print(f"[Smoothed] Dist={sdist:.2f} m, Bearing={sbear:+.1f}°,"
                  f" w={real_w:.2f}m,h={real_h:.2f}m, err={best_err:.3f}")
        else:
            print("No cube face matched within tolerance")

        # 10) Display
        cv2.imshow("Color", color)
        cv2.imshow("Mask", mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nInterrupted")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()


if *name* == "*main*":
main()
