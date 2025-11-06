# Air Hockey Puck Tracker (fast + low-lag)
# Classical CV + Kalman + dynamic ROI + line suppression
# pip install opencv-python numpy

import numpy as np
import time
import cv2

# -------------------- USER SETTINGS --------------------
CAM_INDEX   = 0
FRAME_WIDTH = 960
FRAME_HEIGHT = 540
TARGET_FPS  = 120          # try 120; camera will clamp if unsupported

# Initial HSV bounds (kept to allow future colored puck)
PUCK_HSV_LOW  = [0, 0, 0]
PUCK_HSV_HIGH = [180, 120, 150]

# Table boundary (global ROI) defaults (tune with sliders)
ROI_MARGINS = dict(top=60, bottom=60, left=200, right=450)

# Processing speed / latency controls
PROC_SCALE        = 0.5     # process at half-res (0.4–0.6 are good)
ROI_HALF_W        = 220     # dynamic crop half-width (full-res px)
ROI_HALF_H        = 140     # dynamic crop half-height (full-res px)
FULLFRAME_EVERY   = 12      # run a full-frame scan every N frames

# KF noise (tuned to reduce lag on fast motion)
KF_NOISE_POS  = 0.05
KF_NOISE_VEL  = 4.00
KF_MEAS_NOISE = 2.00


# -----------------------------------------------------
# Helper to create a full-res table boundary mask
def make_roi_mask(h, w, margins):
    """Build a uint8 mask: 255 inside table ROI, 0 outside (full resolution)."""
    mask = np.zeros((h, w), dtype=np.uint8)
    t, b, l, r = margins["top"], margins["bottom"], margins["left"], margins["right"]
    cv2.rectangle(mask, (l, t), (w - r, h - b), 255, thickness=-1)
    return mask

# Simple 2D constant-velocity Kalman filter
class ConstantVelocityKF:
    """4-state Kalman filter: [x, y, vx, vy], measurement: [x, y]."""
    def __init__(self, init_xy=None,
                 process_noise_pos=KF_NOISE_POS,
                 process_noise_vel=KF_NOISE_VEL,
                 meas_noise=KF_MEAS_NOISE):
        self.kf = cv2.KalmanFilter(4, 2)
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


# ------------------ EXTRA HELPERS -------------------
def suppress_table_lines(hsv):
    """Return a mask (255 = keep) that removes red/blue rink markings."""
    # Red lives near 0 and 180 in HSV; build two bands and OR them.
    red1 = cv2.inRange(hsv, (0, 80, 40), (10, 255, 255))
    red2 = cv2.inRange(hsv, (170, 80, 40), (180, 255, 255))
    red = cv2.bitwise_or(red1, red2)

    # Blue is roughly 100–140 hue (tune if your lighting shifts)
    blue = cv2.inRange(hsv, (100, 70, 40), (140, 255, 255))

    # Lines to suppress:
    lines = cv2.bitwise_or(red, blue)

    # Keep = not lines
    keep = cv2.bitwise_not(lines)
    return keep

def make_puck_darklow_mask(hsv, low_hsv, high_hsv, s_max, v_max):
    """
    Build a puck mask that prioritizes 'dark, low-saturation' targets.
    We combine:
      (A) A dedicated 'dark & low-S' mask (robust to gray/black)
      (B) Optional HSV band from sliders (useful if you later paint the puck)
    """
    dark_lowS = cv2.inRange(hsv, (0, 0, 0), (180, s_max, v_max))
    color_mask = cv2.inRange(hsv, low_hsv, high_hsv)
    return cv2.bitwise_or(dark_lowS, color_mask)

def best_circular_candidate(contours, min_r, max_r, s_channel, kf_pred_xy=None):
    """
    Score each contour by circularity and proximity to KF prediction,
    and penalize high saturation (paint/lines).
    Returns (x,y,rad,score) or None.
    """
    best = None
    H, W = s_channel.shape[:2]
    for cnt in contours:
        (x, y), rad = cv2.minEnclosingCircle(cnt)
        if rad < min_r or rad > max_r:
            continue

        area = cv2.contourArea(cnt)
        if area < 5:
            continue
        per = max(1.0, cv2.arcLength(cnt, True))
        circularity = 4.0 * np.pi * (area / (per * per))  # 0..1, higher is rounder

        # Mean saturation inside the contour
        mask = np.zeros((H, W), dtype=np.uint8)
        cv2.drawContours(mask, [cnt], -1, 255, thickness=-1)
        meanS = cv2.mean(s_channel, mask=mask)[0]  # 0..255

        # Proximity to KF prediction (if available)
        prox_penalty = 0.0
        if kf_pred_xy is not None:
            px, py = kf_pred_xy
            d = np.hypot(x - px, y - py)
            prox_penalty = 0.002 * d  # tune if needed

        # Saturation penalty (prefer low-S to avoid colored lines)
        sat_penalty = (meanS / 255.0) * 0.3

        score = circularity - prox_penalty - sat_penalty

        if (best is None) or (score > best[3]):
            best = (float(x), float(y), float(rad), float(score))
    return best

def crop_roi_around(x, y, w, h, half_w, half_h):
    """Integer crop box around (x,y) clamped to image rect."""
    x1 = max(0, int(x - half_w)); y1 = max(0, int(y - half_h))
    x2 = min(w, int(x + half_w)); y2 = min(h, int(y + half_h))
    # Ensure non-empty
    if x2 <= x1: x2 = min(w, x1 + 2*half_w)
    if y2 <= y1: y2 = min(h, y1 + 2*half_h)
    return x1, y1, x2, y2


# ------------------ UI TRACKBARS -------------------
def nothing(x):
    pass

def setup_trackbars(h, w):
    cv2.namedWindow("Controls", cv2.WINDOW_NORMAL)

    # HSV thresholds (for optional colored puck)
    cv2.createTrackbar("LowH", "Controls", PUCK_HSV_LOW[0], 180, nothing)
    cv2.createTrackbar("LowS", "Controls", PUCK_HSV_LOW[1], 255, nothing)
    cv2.createTrackbar("LowV", "Controls", PUCK_HSV_LOW[2], 255, nothing)
    cv2.createTrackbar("HighH", "Controls", PUCK_HSV_HIGH[0], 180, nothing)
    cv2.createTrackbar("HighS", "Controls", PUCK_HSV_HIGH[1], 255, nothing)
    cv2.createTrackbar("HighV", "Controls", PUCK_HSV_HIGH[2], 255, nothing)

    # Table boundary (global ROI) sliders (full-res)
    cv2.createTrackbar("ROI_Top", "Controls", ROI_MARGINS["top"], h//2, nothing)
    cv2.createTrackbar("ROI_Bottom", "Controls", ROI_MARGINS["bottom"], h//2, nothing)
    cv2.createTrackbar("ROI_Left", "Controls", ROI_MARGINS["left"], w//2, nothing)
    cv2.createTrackbar("ROI_Right", "Controls", ROI_MARGINS["right"], w//2, nothing)

    # Radius filtering sliders (full-res radii)
    cv2.createTrackbar("MinRadius", "Controls", 10, 150, nothing)
    cv2.createTrackbar("MaxRadius", "Controls", 40, 300, nothing)

    # Dark & Low-S ceilings (for gray/black puck)
    cv2.createTrackbar("DarkSMax", "Controls", 70, 255, nothing)
    cv2.createTrackbar("DarkVMax", "Controls", 90, 255, nothing)

def update_params(h, w):
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

    dark_s_max = cv2.getTrackbarPos("DarkSMax", "Controls")
    dark_v_max = cv2.getTrackbarPos("DarkVMax", "Controls")

    margins = dict(top=roi_top, bottom=roi_bottom,
                   left=roi_left, right=roi_right)

    return (np.array([lowH, lowS, lowV]),
            np.array([highH, highS, highV]),
            margins,
            min_radius,
            max_radius,
            dark_s_max,
            dark_v_max)


# ------------------ MAIN -------------------
def main():
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

    # Try to minimize camera buffering and motion blur
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)         # some backends ignore this
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # UVC: 0.25 -> manual, 0.75 -> auto (varies by driver)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)          # shorter exposure (increase lighting)
    cap.set(cv2.CAP_PROP_GAIN, 0)

    if not cap.isOpened():
        print("Error: cannot open camera")
        return

    # Prime one frame (grab newest)
    if not cap.grab():
        print("Error: cannot grab frame")
        return
    ret, frame = cap.retrieve()
    if not ret:
        print("Error: cannot retrieve frame")
        return

    h, w = frame.shape[:2]
    kf = ConstantVelocityKF(init_xy=(w / 2, h / 2))

    # Setup sliders
    setup_trackbars(h, w)

    last_print_t = 0.0
    frame_count = 0

    while True:
        # Always take the newest frame (drop backlog)
        if not cap.grab():
            break
        ret, frame = cap.retrieve()
        if not ret:
            break

        # Read sliders each loop (full-res params)
        lowHSV, highHSV, margins, min_r, max_r, s_max, v_max = update_params(h, w)

        # Build full-res table mask (global boundary), then we'll crop and scale it
        table_mask_full = make_roi_mask(h, w, margins)

        # KF prediction (full-res coords)
        pred_x, pred_y, _, _ = kf.predict()

        # Choose processing region: dynamic crop around prediction, or full frame occasionally
        use_full = (frame_count % FULLFRAME_EVERY == 0)
        if use_full:
            roi_x1, roi_y1, roi_x2, roi_y2 = 0, 0, w, h
        else:
            roi_x1, roi_y1, roi_x2, roi_y2 = crop_roi_around(
                pred_x, pred_y, w, h, ROI_HALF_W, ROI_HALF_H
            )

        # Crop full-res frame and mask to our region
        roi_bgr  = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        roi_mask = table_mask_full[roi_y1:roi_y2, roi_x1:roi_x2]

        # Downscale region for fast processing
        proc     = cv2.resize(roi_bgr, (0, 0), fx=PROC_SCALE, fy=PROC_SCALE, interpolation=cv2.INTER_AREA)
        procmask = cv2.resize(roi_mask, (int(roi_bgr.shape[1]*PROC_SCALE), int(roi_bgr.shape[0]*PROC_SCALE)),
                              interpolation=cv2.INTER_NEAREST)

        ph, pw = proc.shape[:2]

        # Light blur, HSV, and line suppression in the small image
        blurred = cv2.GaussianBlur(proc, (3, 3), 0)
        hsv     = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        keep_mask = suppress_table_lines(hsv)

        # Puck-first mask (dark & low-S with optional color band)
        puck_mask = make_puck_darklow_mask(hsv, lowHSV, highHSV, s_max, v_max)

        # Combine: (puck) AND (not lines) AND (table boundary)
        mask = cv2.bitwise_and(puck_mask, keep_mask)
        mask = cv2.bitwise_and(mask, procmask)

        # Morphology: open (remove specks) then close (solidify puck)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Fallback: if empty, try adaptive threshold (still within proc ROI)
        if not np.any(mask):
            gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
            adaptive = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 5
            )
            adaptive = cv2.bitwise_and(adaptive, keep_mask)
            adaptive = cv2.bitwise_and(adaptive, procmask)
            adaptive = cv2.morphologyEx(adaptive, cv2.MORPH_OPEN, kernel)
            adaptive = cv2.morphologyEx(adaptive, cv2.MORPH_CLOSE, kernel)
            mask = adaptive

        # Contours in the small image
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Convert KF prediction into local, scaled coords for scoring
        pred_local_x = (pred_x - roi_x1) * PROC_SCALE
        pred_local_y = (pred_y - roi_y1) * PROC_SCALE

        # Scale radii for small image
        min_r_s = int(max(1, min_r * PROC_SCALE))
        max_r_s = int(max(2, max_r * PROC_SCALE))

        # Score candidates
        hch, sch, vch = cv2.split(hsv)
        best = best_circular_candidate(contours, min_r_s, max_r_s, sch, (pred_local_x, pred_local_y))

        got_measurement = False
        if best is not None:
            bx, by, br, _ = best
            # Map back to full-frame coords
            gx = bx / PROC_SCALE + roi_x1
            gy = by / PROC_SCALE + roi_y1
            fx, fy, vx, vy = kf.correct(gx, gy)
            got_measurement = True
        else:
            # Optional Hough: only on full-frame sweeps to save time
            if use_full:
                gray_small = cv2.cvtColor(proc, cv2.COLOR_BGR2GRAY)
                hc = cv2.HoughCircles(
                    gray_small, cv2.HOUGH_GRADIENT, dp=1.2, minDist=max(10, int(min_r_s*1.5)),
                    param1=120, param2=22, minRadius=int(min_r_s), maxRadius=int(max_r_s)
                )
                if hc is not None:
                    cand = min(hc[0], key=lambda c: np.hypot(c[0]-pred_local_x, c[1]-pred_local_y))
                    gx = cand[0] / PROC_SCALE + roi_x1
                    gy = cand[1] / PROC_SCALE + roi_y1
                    fx, fy, vx, vy = kf.correct(gx, gy)
                    got_measurement = True

            if not got_measurement:
                # Pure prediction path (zero extra lag in processing)
                fx, fy = pred_x, pred_y
                vx = vy = 0.0

        # --------------- Visualization ---------------
        vis = frame.copy()
        # Draw global table boundary
        cv2.rectangle(vis,
                      (margins["left"], margins["top"]),
                      (w - margins["right"], h - margins["bottom"]),
                      (255, 0, 0), 2)
        # Draw dynamic ROI (processing window)
        cv2.rectangle(vis, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 128, 255), 1)

        # KF position
        cv2.circle(vis, (int(fx), int(fy)), max(5, min_r), (0, 255, 0), 2)
        cv2.putText(vis, f"KF: ({int(fx)}, {int(fy)}) {'M' if got_measurement else 'P'}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Show mask (scaled up to full-res window for convenience)
        mask_show = cv2.resize(mask, (roi_bgr.shape[1], roi_bgr.shape[0]), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("Mask (proc ROI)", mask_show)
        cv2.imshow("Puck Tracking", vis)

        # Timed printout (CSV fx,fy)
        now = time.time()
        if now - last_print_t >= 1.0 / max(1, TARGET_FPS):
            print(f"{int(fx)},{int(fy)}", flush=True)
            last_print_t = now

        frame_count += 1

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
