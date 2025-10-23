
# Air Hockey Puck Tracker (Classical CV + Kalman + ROI)
# with interactive sliders for quick tuning (HSV, ROI, Radius)
# ROI: Region Of Interest
# pip install opencv-python numpy

import numpy as np
import time
import cv2
from statistics import mean
import tkinter as tk
from tkinter import messagebox

# -------------------- USER SETTINGS --------------------
CAM_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
TARGET_FPS = 90
TRACK_LIGHT = True  # Track light-colored puck if True, dark if False
MIN_SOLIDITY = 0.85  # Minimum solidity (area/convex hull area ratio)
MAX_ASPECT_RATIO = 1.2  # Maximum allowed aspect ratio (width/height)
MAX_VELOCITY = 1000  # Maximum allowed velocity in pixels/second
MIN_DETECTION_SCORE = 0.7  # Minimum score to consider a detection valid

# Initial HSV bounds (looser for slightly brighter puck)
PUCK_HSV_LOW  = [0, 0, 0]
PUCK_HSV_HIGH = [180, 120, 150]

# ROI defaults
ROI_MARGINS = dict(top=60, bottom=60, left=200, right=450)


# -----------------------------------------------------
# Helper to create ROI mask
def make_roi_mask(h, w, margins):
    """Build a uint8 mask: 255 inside ROI, 0 outside."""
    # Convert dimensions to integers
    h = int(h)
    w = int(w)
    
    # Create mask with correct dimensions
    mask = np.zeros((h, w), dtype=np.uint8)
    
    # Convert margin values to integers
    t = int(margins["top"])
    b = int(margins["bottom"])
    l = int(margins["left"])
    r = int(margins["right"])
    
    # Draw rectangle on mask (everything outside will be black)
    # Note: points are (x,y) format
    roi_points = np.array([
        [l, t],  # Top left
        [w-r, t],  # Top right
        [w-r, h-b],  # Bottom right
        [l, h-b]  # Bottom left
    ], dtype=np.int32)
    
    # Fill the ROI region with white
    cv2.fillPoly(mask, [roi_points], 255)
    
    return mask

# Simple 2D constant-velocity Kalman filter
class ConstantVelocityKF:
    """4-state Kalman filter: [x, y, vx, vy], measurement: [x, y]."""
    def __init__(self, init_xy=None, process_noise_pos=1e-2,
                 process_noise_vel=5e-1, meas_noise=3.0):
        self.kf = cv2.KalmanFilter(4, 2, 0, cv2.CV_32F)
        self.kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)

        q = np.diag([process_noise_pos, process_noise_pos,
                     process_noise_vel, process_noise_vel]).astype(np.float32)
        self.kf.processNoiseCov = q
        self.kf.measurementNoiseCov = (meas_noise * np.eye(2, dtype=np.float32))
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

        if init_xy is not None:
            x, y = init_xy
        else:
            x, y = 0.0, 0.0
        self.kf.statePost = np.array([[x], [y], [0.0], [0.0]], dtype=np.float32)
        self._last_t = time.time()

    def _update_dt(self):
        now = time.time()
        dt = max(1e-3, now - self._last_t)
        self._last_t = now
        self.kf.transitionMatrix[0, 2] = dt
        self.kf.transitionMatrix[1, 3] = dt

    def predict(self):
        self._update_dt()
        pred = self.kf.predict()
        return [float(pred[i, 0]) for i in range(4)]

    def correct(self, meas_x, meas_y):
        measurement = np.array([[np.float32(meas_x)], [np.float32(meas_y)]],
                               dtype=np.float32)
        est = self.kf.correct(measurement)
        return [float(est[i, 0]) for i in range(4)]


# ------------------ END OF KF CLASS -------------------
def nothing(x):
    pass

# Set up trackbars for HSV, ROI, radius
def setup_trackbars(h, w):
    cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)
    cv2.namedWindow("HSV Debug", cv2.WINDOW_NORMAL)

    # Add tracking mode switch
    cv2.createTrackbar("Track Light/Dark", "Controls", 1, 1, nothing)

    # HSV thresholds
    cv2.createTrackbar("LowH", "Controls", PUCK_HSV_LOW[0], 180, nothing)
    cv2.createTrackbar("LowS", "Controls", PUCK_HSV_LOW[1], 255, nothing)
    cv2.createTrackbar("LowV", "Controls", PUCK_HSV_LOW[2], 255, nothing)
    cv2.createTrackbar("HighH", "Controls", PUCK_HSV_HIGH[0], 180, nothing)
    cv2.createTrackbar("HighS", "Controls", PUCK_HSV_HIGH[1], 255, nothing)
    cv2.createTrackbar("HighV", "Controls", PUCK_HSV_HIGH[2], 255, nothing)

    # ROI sliders
    cv2.createTrackbar("ROI_Top", "Controls", ROI_MARGINS["top"], h//2, nothing)
    cv2.createTrackbar("ROI_Bottom", "Controls", ROI_MARGINS["bottom"], h//2, nothing)
    cv2.createTrackbar("ROI_Left", "Controls", ROI_MARGINS["left"], w//2, nothing)
    cv2.createTrackbar("ROI_Right", "Controls", ROI_MARGINS["right"], w//2, nothing)

    # Radius filtering sliders
    cv2.createTrackbar("MinRadius", "Controls", 10, 100, nothing)
    cv2.createTrackbar("MaxRadius", "Controls", 40, 100, nothing)

# Read trackbar positions
def update_params(h, w):
    track_light = cv2.getTrackbarPos("Track Light/Dark", "Controls") == 1
    lowH = cv2.getTrackbarPos("LowH", "Controls")
    lowS = cv2.getTrackbarPos("LowS", "Controls")
    lowV = cv2.getTrackbarPos("LowV", "Controls")
    highH = cv2.getTrackbarPos("HighH", "Controls")
    highS = cv2.getTrackbarPos("HighS", "Controls")
    highV = cv2.getTrackbarPos("HighV", "Controls")

    roi_top = cv2.getTrackbarPos("ROI_Top", "Controls")
    roi_bottom = cv2.getTrackbarPos("ROI_Bottom", "Controls")
    roi_left = cv2.getTrackbarPos("ROI_Left", "Controls")
    roi_right = cv2.getTrackbarPos("ROI_Right", "Controls")

    min_radius = cv2.getTrackbarPos("MinRadius", "Controls")
    max_radius = cv2.getTrackbarPos("MaxRadius", "Controls")

    margins = dict(top=roi_top, bottom=roi_bottom,
                   left=roi_left, right=roi_right)

    return (np.array([lowH, lowS, lowV]),
            np.array([highH, highS, highV]),
            margins,
            min_radius,
            max_radius,
            track_light)


def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

    last_valid_pos = None
    last_valid_time = time.time()

    if not cap.isOpened():
        print("Error: cannot open camera")
        return

    ret, frame = cap.read()
    if not ret:
        print("Error: cannot read from camera")
        return

    h, w = frame.shape[:2]
    kf = ConstantVelocityKF(init_xy=(w / 2, h / 2))

    # if combined.shape[:2] != roi_mask.shape[:2]:
    #     roi_mask = cv2.resize(roi_mask, (combined.shape[1], combined.shape[0]))

    fps_t = time.time()
    last_print_t = 0.0

    # Setup sliders
    setup_trackbars(h, w)


    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Read sliders each loop
        lowHSV, highHSV, margins, min_r, max_r, track_light = update_params(h, w)

        roi_mask = make_roi_mask(h, w, margins)

        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # --- Color threshold ---
        mask_color = cv2.inRange(hsv, lowHSV, highHSV)
        if not track_light:
            mask_color = cv2.bitwise_not(mask_color)

        # --- Adaptive threshold fallback ---
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        adaptive = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 5
        )

        # Combine (color OR adaptive), then ROI
        combined = cv2.bitwise_or(mask_color, adaptive)
        roi_mask = make_roi_mask(frame.shape[0], frame.shape[1], margins)

        if combined.shape[:2] != roi_mask.shape[:2]:
            roi_mask = cv2.resize(roi_mask, (combined.shape[1], combined.shape[0]))
        combined = cv2.bitwise_and(combined, roi_mask)

        # Morphological cleanup
        kernel_open = np.ones((3, 3), np.uint8)  # Smaller kernel for opening
        kernel_close = np.ones((5, 5), np.uint8)  # Larger kernel for closing
        
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel_open)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel_close)
        combined = cv2.morphologyEx(combined, cv2.MORPH_ERODE, kernel_open)

        # --- Find puck candidate ---
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        meas = None
        best_score = -1.0

        # Evaluate contours, pick best circular one within size range
        for cnt in contours:
            (x, y), rad = cv2.minEnclosingCircle(cnt)
            if rad < min_r or rad > max_r:
                continue

            area = cv2.contourArea(cnt)
            perimeter = max(1.0, cv2.arcLength(cnt, True))
            circularity = 4.0 * np.pi * (area / (perimeter * perimeter))
            score = circularity
            if score > best_score:
                best_score = score
                meas = (float(x), float(y), float(rad))

            rect = cv2.minAreaRect(cnt)
            w, h = rect[1]
            if w == 0 or h == 0:
                continue
                
            # Check aspect ratio
            aspect_ratio = max(w, h) / min(w, h)
            if aspect_ratio > MAX_ASPECT_RATIO:
                continue
                
            # Check solidity
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            if hull_area > 0:
                solidity = area / hull_area
                if solidity < MIN_SOLIDITY:
                    continue
            
            # Adjust scoring to include these metrics
            circularity = 4.0 * np.pi * (area / (perimeter * perimeter))
            score = circularity * (1.0 - (aspect_ratio - 1.0)) * solidity
            
            if score > best_score:
                best_score = score
                meas = (float(x), float(y), float(rad))

        # Kalman predict + correct
        if meas is not None and best_score > MIN_DETECTION_SCORE:
            current_time = time.time()
            if last_valid_pos is not None:
                # Check if movement is physically possible
                dx = meas[0] - last_valid_pos[0]
                dy = meas[1] - last_valid_pos[1]
                dt = current_time - last_valid_time
                velocity = np.sqrt(dx*dx + dy*dy) / dt
                
                if velocity > MAX_VELOCITY:
                    meas = None  # Reject physically impossible movements
                else:
                    last_valid_pos = (meas[0], meas[1])
                    last_valid_time = current_time
                    fx, fy, vx, vy = kf.correct(meas[0], meas[1])
            else:
                last_valid_pos = (meas[0], meas[1])
                last_valid_time = current_time
                fx, fy, vx, vy = kf.correct(meas[0], meas[1])
        else:
            fx, fy, vx, vy = kf.predict()

        # Visualization
        vis = frame.copy()
        cv2.circle(vis, (int(fx), int(fy)), int(max_r), (0, 255, 0), 2)
        cv2.rectangle(vis,
                     pt1=(int(margins["left"]), int(margins["top"])),
                     pt2=(int(w - margins["right"]), int(h - margins["bottom"])),
                     color=(255, 0, 0),
                     thickness=2)
        cv2.putText(vis, f"KF: ({int(fx)}, {int(fy)})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        hsv_channels = cv2.split(hsv)
        hsv_vis = np.hstack([
        cv2.cvtColor(hsv_channels[0], cv2.COLOR_GRAY2BGR),  # H channel
        cv2.cvtColor(hsv_channels[1], cv2.COLOR_GRAY2BGR),  # S channel
        cv2.cvtColor(hsv_channels[2], cv2.COLOR_GRAY2BGR)   # V channel
    ])
        
        cv2.imshow("HSV Debug", hsv_vis)
        cv2.imshow("Mask", combined)
        cv2.imshow("Puck Tracking", vis)

        now = time.time()
        # Print at most TARGET_FPS lines per second
        if now - last_print_t >= 1.0 / max(1, TARGET_FPS):
            print(f"{int(fx)},{int(fy)}", flush=True)
            last_print_t = now

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

        fps_t = now

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
