# Advanced Hockey Puck Tracker
# Features: GUI color selection, ROI adjustment, velocity tracking (m/s), 90fps support
# Requirements: pip install opencv-python numpy

import numpy as np
import time
import cv2
import tkinter as tk
from tkinter import ttk, messagebox
from threading import Thread, Lock
from collections import deque

# -------------------- CONFIGURATION --------------------
# Camera Settings
# CAM_INDEX: 0 = laptop webcam, 1 = first USB camera (ELP), 2 = second USB, etc.
# Change this value if you have multiple cameras connected
CAM_INDEX = 0  # Changed to 1 for ELP USB camera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 90

# Performance optimization
SKIP_LINE_SUPPRESSION = True  # Skip table line removal for speed (enable if needed)
MORPH_KERNEL_SIZE = 3  # Smaller kernel = faster processing
VISUAL_UPDATE_SKIP = 0  # Skip N frames between visual updates (0=update every frame)
GUI_UPDATE_MS = 100  # GUI refresh rate in milliseconds

# Puck color presets (HSV ranges)
COLOR_PRESETS = {
    "Black/Dark": {"low": [0, 0, 0], "high": [180, 70, 90]},
    "Orange": {"low": [5, 100, 100], "high": [25, 255, 255]},
    "Green": {"low": [40, 50, 50], "high": [80, 255, 255]},
    "Blue": {"low": [100, 50, 50], "high": [130, 255, 255]},
    "Red": {"low": [0, 100, 100], "high": [10, 255, 255]},
    "Yellow": {"low": [20, 100, 100], "high": [40, 255, 255]},
    "Custom": {"low": [0, 0, 0], "high": [180, 255, 255]}
}

# Default physical calibration (meters per pixel)
# User should calibrate this based on known distance in the scene
DEFAULT_METERS_PER_PIXEL = 0.001  # 1mm per pixel as default

# Tracking parameters
DEFAULT_ROI_MARGINS = {"top": 50, "bottom": 50, "left": 50, "right": 50}
DEFAULT_MIN_RADIUS = 5
DEFAULT_MAX_RADIUS = 50
VELOCITY_HISTORY_SIZE = 10  # frames to average for velocity calculation

# Table boundary for trajectory prediction (physics simulation)
DEFAULT_TABLE_BOUNDS = {"top": 100, "bottom": 100, "left": 100, "right": 100}
TRAJECTORY_PREDICTION_TIME = 2.0  # seconds to predict ahead
TRAJECTORY_DAMPING = 0.95  # velocity reduction per bounce (0.9 = 10% loss)

# Kalman filter tuning
KF_PROCESS_NOISE_POS = 0.05
KF_PROCESS_NOISE_VEL = 4.0
KF_MEAS_NOISE = 2.0

# -------------------- KALMAN FILTER --------------------
class ConstantVelocityKF:
    """2D Kalman filter: state [x, y, vx, vy], measurement [x, y]"""
    def __init__(self, init_xy=None):
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

        q = np.diag([KF_PROCESS_NOISE_POS, KF_PROCESS_NOISE_POS,
                     KF_PROCESS_NOISE_VEL, KF_PROCESS_NOISE_VEL]).astype(np.float32)
        self.kf.processNoiseCov = q
        self.kf.measurementNoiseCov = KF_MEAS_NOISE * np.eye(2, dtype=np.float32)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

        x, y = init_xy if init_xy else (0.0, 0.0)
        self.kf.statePost = np.array([[x], [y], [0.0], [0.0]], dtype=np.float32)
        self._last_t = time.time()

    def _update_dt(self):
        now = time.time()
        dt = max(1e-3, now - self._last_t)
        self._last_t = now
        self.kf.transitionMatrix[0, 2] = dt
        self.kf.transitionMatrix[1, 3] = dt
        return dt

    def predict(self):
        self._update_dt()
        pred = self.kf.predict()
        return [float(pred[i, 0]) for i in range(4)]

    def correct(self, meas_x, meas_y):
        measurement = np.array([[np.float32(meas_x)], [np.float32(meas_y)]], dtype=np.float32)
        est = self.kf.correct(measurement)
        return [float(est[i, 0]) for i in range(4)]

# -------------------- TRACKER STATE --------------------
class TrackerState:
    """Thread-safe state for tracker parameters"""
    def __init__(self):
        self.lock = Lock()
        
        # Color tracking
        self.selected_color = "Black/Dark"
        self.hsv_low = np.array(COLOR_PRESETS["Black/Dark"]["low"])
        self.hsv_high = np.array(COLOR_PRESETS["Black/Dark"]["high"])
        
        # ROI
        self.roi_margins = DEFAULT_ROI_MARGINS.copy()
        
        # Table bounds for trajectory prediction
        self.table_bounds = DEFAULT_TABLE_BOUNDS.copy()
        
        # Radius
        self.min_radius = DEFAULT_MIN_RADIUS
        self.max_radius = DEFAULT_MAX_RADIUS
        
        # Calibration
        self.meters_per_pixel = DEFAULT_METERS_PER_PIXEL
        self.calibration_distance_m = 0.5  # 50cm for calibration
        self.calibration_distance_px = 500  # pixels
        
        # Tracking mode
        self.tracking_enabled = True
        self.show_mask = True
        self.show_velocity_vector = True
        self.show_trajectory = True
        
        # Results
        self.current_x = 0.0
        self.current_y = 0.0
        self.velocity_x_ms = 0.0
        self.velocity_y_ms = 0.0
        self.speed_ms = 0.0
        self.fps = 0.0
        
    def get_color_range(self):
        with self.lock:
            return self.hsv_low.copy(), self.hsv_high.copy()
    
    def set_color_preset(self, color_name):
        with self.lock:
            self.selected_color = color_name
            self.hsv_low = np.array(COLOR_PRESETS[color_name]["low"])
            self.hsv_high = np.array(COLOR_PRESETS[color_name]["high"])
    
    def set_custom_color(self, hsv_low, hsv_high):
        with self.lock:
            self.selected_color = "Custom"
            self.hsv_low = np.array(hsv_low)
            self.hsv_high = np.array(hsv_high)
    
    def get_roi_margins(self):
        with self.lock:
            return self.roi_margins.copy()
    
    def set_roi_margins(self, margins):
        with self.lock:
            self.roi_margins = margins.copy()
    
    def get_table_bounds(self):
        with self.lock:
            return self.table_bounds.copy()
    
    def set_table_bounds(self, bounds):
        with self.lock:
            self.table_bounds = bounds.copy()
    
    def get_radius_range(self):
        with self.lock:
            return self.min_radius, self.max_radius
    
    def set_radius_range(self, min_r, max_r):
        with self.lock:
            self.min_radius = min_r
            self.max_radius = max_r
    
    def get_meters_per_pixel(self):
        with self.lock:
            return self.meters_per_pixel
    
    def set_meters_per_pixel(self, mpp):
        with self.lock:
            self.meters_per_pixel = mpp
    
    def update_calibration(self, distance_m, distance_px):
        with self.lock:
            self.calibration_distance_m = distance_m
            self.calibration_distance_px = distance_px
            if distance_px > 0:
                self.meters_per_pixel = distance_m / distance_px
    
    def update_results(self, x, y, vx_ms, vy_ms, speed_ms, fps):
        with self.lock:
            self.current_x = x
            self.current_y = y
            self.velocity_x_ms = vx_ms
            self.velocity_y_ms = vy_ms
            self.speed_ms = speed_ms
            self.fps = fps

# -------------------- HELPER FUNCTIONS --------------------
def make_roi_mask(h, w, margins):
    """Create ROI mask"""
    mask = np.zeros((h, w), dtype=np.uint8)
    t, b, l, r = margins["top"], margins["bottom"], margins["left"], margins["right"]
    cv2.rectangle(mask, (l, t), (w - r, h - b), 255, thickness=-1)
    return mask

def suppress_table_lines(hsv):
    """Remove red/blue rink markings from mask"""
    red1 = cv2.inRange(hsv, (0, 80, 40), (10, 255, 255))
    red2 = cv2.inRange(hsv, (170, 80, 40), (180, 255, 255))
    red = cv2.bitwise_or(red1, red2)
    blue = cv2.inRange(hsv, (100, 70, 40), (140, 255, 255))
    lines = cv2.bitwise_or(red, blue)
    return cv2.bitwise_not(lines)

def find_best_puck_candidate(contours, min_r, max_r, kf_pred_xy=None):
    """Find best circular candidate from contours"""
    best = None
    for cnt in contours:
        (x, y), rad = cv2.minEnclosingCircle(cnt)
        if rad < min_r or rad > max_r:
            continue
        
        area = cv2.contourArea(cnt)
        if area < 5:
            continue
        
        per = max(1.0, cv2.arcLength(cnt, True))
        circularity = 4.0 * np.pi * (area / (per * per))
        
        # Proximity to prediction
        prox_penalty = 0.0
        if kf_pred_xy is not None:
            px, py = kf_pred_xy
            d = np.hypot(x - px, y - py)
            prox_penalty = 0.002 * d
        
        score = circularity - prox_penalty
        
        if (best is None) or (score > best[3]):
            best = (float(x), float(y), float(rad), float(score))
    
    return best

def predict_trajectory_with_bounces(x, y, vx_px, vy_px, bounds, max_time=2.0, dt=0.016, damping=0.95):
    """Predict puck trajectory with wall bounces
    
    Args:
        x, y: Current position (pixels)
        vx_px, vy_px: Current velocity (pixels/second)
        bounds: dict with 'top', 'bottom', 'left', 'right' margins
        max_time: How far ahead to predict (seconds)
        dt: Time step for simulation (seconds)
        damping: Velocity retention after bounce (0.95 = 5% loss)
    
    Returns:
        List of (x, y) points along predicted path
    """
    # Calculate table boundaries
    left = bounds['left']
    right = bounds['right']  # This is actually margin from right edge
    top = bounds['top']
    bottom = bounds['bottom']  # This is margin from bottom edge
    
    # Simulate trajectory
    points = [(x, y)]
    px, py = x, y
    vx, vy = vx_px, vy_px
    
    time = 0
    while time < max_time:
        # Update position
        px += vx * dt
        py += vy * dt
        
        # Check for wall collisions and bounce
        # Note: 'right' and 'bottom' are margins, need frame dimensions for actual bounds
        # We'll handle this in the calling function with proper frame dimensions
        
        points.append((px, py))
        time += dt
        
        # Stop if velocity is too low
        if abs(vx) < 1 and abs(vy) < 1:
            break
    
    return points

def predict_trajectory_bounded(x, y, vx_px, vy_px, frame_w, frame_h, bounds, max_time=2.0, dt=0.016, damping=0.95):
    """Predict trajectory with proper bounds checking"""
    # Calculate actual boundaries
    left = bounds['left']
    right = frame_w - bounds['right']
    top = bounds['top']
    bottom = frame_h - bounds['bottom']
    
    points = [(x, y)]
    px, py = x, y
    vx, vy = vx_px, vy_px
    
    time = 0
    max_bounces = 20  # Prevent infinite loops
    bounces = 0
    
    while time < max_time and bounces < max_bounces:
        # Update position
        px += vx * dt
        py += vy * dt
        
        # Check boundaries and bounce
        if px <= left:
            px = left
            vx = abs(vx) * damping
            bounces += 1
        elif px >= right:
            px = right
            vx = -abs(vx) * damping
            bounces += 1
        
        if py <= top:
            py = top
            vy = abs(vy) * damping
            bounces += 1
        elif py >= bottom:
            py = bottom
            vy = -abs(vy) * damping
            bounces += 1
        
        points.append((int(px), int(py)))
        time += dt
        
        # Stop if velocity is too low
        if abs(vx) < 5 and abs(vy) < 5:
            break
    
    return points

# -------------------- TRACKING THREAD --------------------
def tracking_thread(state, stop_event):
    """Main tracking loop running in separate thread"""
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    # Try to reduce motion blur
    # cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    # cap.set(cv2.CAP_PROP_EXPOSURE, -3) # Adjust as needed
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return
    
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read from camera")
        return
    
    h, w = frame.shape[:2]
    kf = ConstantVelocityKF(init_xy=(w / 2, h / 2))
    
    fps_history = deque(maxlen=30)
    last_time = time.time()
    frame_counter = 0
    
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            break
        
        # Get current parameters
        hsv_low, hsv_high = state.get_color_range()
        margins = state.get_roi_margins()
        table_bounds = state.get_table_bounds()
        min_r, max_r = state.get_radius_range()
        meters_per_pixel = state.get_meters_per_pixel()
        
        # Create ROI mask (reuse if margins haven't changed - optimization)
        roi_mask = make_roi_mask(h, w, margins)
        
        # Preprocessing - use smaller blur kernel for speed
        blurred = cv2.GaussianBlur(frame, (3, 3), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Color threshold
        mask_color = cv2.inRange(hsv, hsv_low, hsv_high)
        
        # Suppress table lines (optional for speed)
        if not SKIP_LINE_SUPPRESSION:
            keep_mask = suppress_table_lines(hsv)
            mask_color = cv2.bitwise_and(mask_color, keep_mask)
        
        # Apply ROI
        combined = cv2.bitwise_and(mask_color, roi_mask)
        
        # Morphological cleanup - use smaller kernel for speed
        kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_OPEN, kernel)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(combined, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Kalman prediction
        pred_x, pred_y, pred_vx, pred_vy = kf.predict()
        
        # Find best candidate
        best = find_best_puck_candidate(contours, min_r, max_r, (pred_x, pred_y))
        
        got_measurement = False
        if best is not None:
            bx, by, br, _ = best
            fx, fy, vx_px, vy_px = kf.correct(bx, by)
            got_measurement = True
        else:
            fx, fy = pred_x, pred_y
            vx_px, vy_px = pred_vx, pred_vy
        
        # Convert velocity to m/s
        vx_ms = vx_px * meters_per_pixel
        vy_ms = vy_px * meters_per_pixel
        speed_ms = np.hypot(vx_ms, vy_ms)
        
        # Calculate FPS
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        if dt > 0:
            fps_history.append(1.0 / dt)
        fps = np.mean(fps_history) if fps_history else 0
        
        # Update state
        state.update_results(fx, fy, vx_ms, vy_ms, speed_ms, fps)
        
        # Console output (every frame for data logging)
        print(f"{int(fx)},{int(fy)},{vx_ms:.4f},{vy_ms:.4f},{speed_ms:.4f}", flush=True)
        
        # Visualization (skip frames for performance if needed)
        frame_counter += 1
        if frame_counter % (VISUAL_UPDATE_SKIP + 1) != 0:
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                stop_event.set()
                break
            continue
        
        vis = frame.copy()
        
        # Draw ROI
        cv2.rectangle(vis,
                     (margins["left"], margins["top"]),
                     (w - margins["right"], h - margins["bottom"]),
                     (255, 0, 0), 2)
        
        # Draw table bounds (trajectory boundary)
        cv2.rectangle(vis,
                     (table_bounds["left"], table_bounds["top"]),
                     (w - table_bounds["right"], h - table_bounds["bottom"]),
                     (255, 165, 0), 2)  # Orange for table bounds
        
        # Draw trajectory prediction
        if state.show_trajectory and speed_ms > 0.1:  # Only show if moving
            traj_points = predict_trajectory_bounded(
                fx, fy, vx_px, vy_px, w, h, table_bounds,
                max_time=TRAJECTORY_PREDICTION_TIME,
                damping=TRAJECTORY_DAMPING
            )
            if len(traj_points) > 1:
                # Draw predicted path (ensure integer coordinates)
                for i in range(len(traj_points) - 1):
                    pt1 = (int(traj_points[i][0]), int(traj_points[i][1]))
                    pt2 = (int(traj_points[i+1][0]), int(traj_points[i+1][1]))
                    cv2.line(vis, pt1, pt2, (255, 0, 255), 2)
                # Draw predicted endpoint
                if len(traj_points) > 0:
                    end_pt = (int(traj_points[-1][0]), int(traj_points[-1][1]))
                    cv2.circle(vis, end_pt, 8, (255, 0, 255), -1)
        
        # Draw puck position
        cv2.circle(vis, (int(fx), int(fy)), max(5, int(max_r)), (0, 255, 0), 2)
        
        # Draw velocity vector
        if state.show_velocity_vector and speed_ms > 0.01:
            vector_scale = 50  # pixels per m/s
            end_x = int(fx + vx_ms * vector_scale)
            end_y = int(fy + vy_ms * vector_scale)
            cv2.arrowedLine(vis, (int(fx), int(fy)), (end_x, end_y), (0, 255, 255), 2, tipLength=0.3)
        
        # Draw info text
        info_y = 30
        cv2.putText(vis, f"Pos: ({int(fx)}, {int(fy)}) px", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        info_y += 25
        cv2.putText(vis, f"Speed: {speed_ms:.3f} m/s", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        info_y += 25
        cv2.putText(vis, f"Vel: ({vx_ms:.3f}, {vy_ms:.3f}) m/s", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        info_y += 25
        cv2.putText(vis, f"FPS: {fps:.1f}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        info_y += 25
        cv2.putText(vis, f"Status: {'M' if got_measurement else 'P'}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Show windows
        cv2.imshow("Puck Tracker", vis)
        if state.show_mask:
            cv2.imshow("Mask", combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            stop_event.set()
            break
    
    cap.release()
    cv2.destroyAllWindows()

# -------------------- GUI --------------------
class TrackerGUI:
    def __init__(self, state):
        self.state = state
        self.root = tk.Tk()
        self.root.title("Hockey Puck Tracker Control Panel")
        self.root.geometry("600x1200")  # Increased height to fit all controls
        
        self._create_widgets()
        self._start_update_loop()
    
    def _create_widgets(self):
        # Camera Selection Frame
        cam_frame = ttk.LabelFrame(self.root, text="Camera Settings", padding=10)
        cam_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(cam_frame, text="Camera Index:").grid(row=0, column=0, sticky="w")
        ttk.Label(cam_frame, text="(0=laptop, 1=USB, etc. Restart required)").grid(row=0, column=1, sticky="w")
        ttk.Label(cam_frame, text=f"Current: {CAM_INDEX}", font=("Arial", 10, "bold")).grid(row=1, column=0, columnspan=2, pady=5)
        
        # Color Selection Frame
        color_frame = ttk.LabelFrame(self.root, text="Puck Color Selection", padding=10)
        color_frame.pack(fill="x", padx=10, pady=5)
        
        self.color_var = tk.StringVar(value="Black/Dark")
        for color in COLOR_PRESETS.keys():
            rb = ttk.Radiobutton(color_frame, text=color, variable=self.color_var,
                                value=color, command=self._on_color_change)
            rb.pack(anchor="w")
        
        # Custom HSV Frame
        hsv_frame = ttk.LabelFrame(self.root, text="Custom HSV Range (for Custom preset)", padding=10)
        hsv_frame.pack(fill="x", padx=10, pady=5)
        
        # Low HSV
        ttk.Label(hsv_frame, text="Low H (0-180):").grid(row=0, column=0, sticky="w")
        self.low_h = ttk.Scale(hsv_frame, from_=0, to=180, orient="horizontal")
        self.low_h.set(0)
        self.low_h.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(hsv_frame, text="Low S (0-255):").grid(row=1, column=0, sticky="w")
        self.low_s = ttk.Scale(hsv_frame, from_=0, to=255, orient="horizontal")
        self.low_s.set(0)
        self.low_s.grid(row=1, column=1, sticky="ew")
        
        ttk.Label(hsv_frame, text="Low V (0-255):").grid(row=2, column=0, sticky="w")
        self.low_v = ttk.Scale(hsv_frame, from_=0, to=255, orient="horizontal")
        self.low_v.set(0)
        self.low_v.grid(row=2, column=1, sticky="ew")
        
        # High HSV
        ttk.Label(hsv_frame, text="High H (0-180):").grid(row=3, column=0, sticky="w")
        self.high_h = ttk.Scale(hsv_frame, from_=0, to=180, orient="horizontal")
        self.high_h.set(180)
        self.high_h.grid(row=3, column=1, sticky="ew")
        
        ttk.Label(hsv_frame, text="High S (0-255):").grid(row=4, column=0, sticky="w")
        self.high_s = ttk.Scale(hsv_frame, from_=0, to=255, orient="horizontal")
        self.high_s.set(255)
        self.high_s.grid(row=4, column=1, sticky="ew")
        
        ttk.Label(hsv_frame, text="High V (0-255):").grid(row=5, column=0, sticky="w")
        self.high_v = ttk.Scale(hsv_frame, from_=0, to=255, orient="horizontal")
        self.high_v.set(255)
        self.high_v.grid(row=5, column=1, sticky="ew")
        
        ttk.Button(hsv_frame, text="Apply Custom HSV", command=self._apply_custom_hsv).grid(row=6, column=0, columnspan=2, pady=5)
        
        hsv_frame.columnconfigure(1, weight=1)
        
        # ROI Frame
        roi_frame = ttk.LabelFrame(self.root, text="Region of Interest (ROI) Margins", padding=10)
        roi_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(roi_frame, text="Top:").grid(row=0, column=0, sticky="w")
        self.roi_top = ttk.Scale(roi_frame, from_=0, to=FRAME_HEIGHT//2, orient="horizontal")
        self.roi_top.set(DEFAULT_ROI_MARGINS["top"])
        self.roi_top.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(roi_frame, text="Bottom:").grid(row=1, column=0, sticky="w")
        self.roi_bottom = ttk.Scale(roi_frame, from_=0, to=FRAME_HEIGHT//2, orient="horizontal")
        self.roi_bottom.set(DEFAULT_ROI_MARGINS["bottom"])
        self.roi_bottom.grid(row=1, column=1, sticky="ew")
        
        ttk.Label(roi_frame, text="Left:").grid(row=2, column=0, sticky="w")
        self.roi_left = ttk.Scale(roi_frame, from_=0, to=FRAME_WIDTH//2, orient="horizontal")
        self.roi_left.set(DEFAULT_ROI_MARGINS["left"])
        self.roi_left.grid(row=2, column=1, sticky="ew")
        
        ttk.Label(roi_frame, text="Right:").grid(row=3, column=0, sticky="w")
        self.roi_right = ttk.Scale(roi_frame, from_=0, to=FRAME_WIDTH//2, orient="horizontal")
        self.roi_right.set(DEFAULT_ROI_MARGINS["right"])
        self.roi_right.grid(row=3, column=1, sticky="ew")
        
        # Bind command after all widgets are created
        self.roi_top.configure(command=self._update_roi)
        self.roi_bottom.configure(command=self._update_roi)
        self.roi_left.configure(command=self._update_roi)
        self.roi_right.configure(command=self._update_roi)
        
        roi_frame.columnconfigure(1, weight=1)
        
        # Table Bounds Frame (for trajectory prediction)
        table_frame = ttk.LabelFrame(self.root, text="Table Bounds (for Trajectory Prediction)", padding=10)
        table_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(table_frame, text="Top:").grid(row=0, column=0, sticky="w")
        self.table_top = ttk.Scale(table_frame, from_=0, to=FRAME_HEIGHT//2, orient="horizontal")
        self.table_top.set(DEFAULT_TABLE_BOUNDS["top"])
        self.table_top.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(table_frame, text="Bottom:").grid(row=1, column=0, sticky="w")
        self.table_bottom = ttk.Scale(table_frame, from_=0, to=FRAME_HEIGHT//2, orient="horizontal")
        self.table_bottom.set(DEFAULT_TABLE_BOUNDS["bottom"])
        self.table_bottom.grid(row=1, column=1, sticky="ew")
        
        ttk.Label(table_frame, text="Left:").grid(row=2, column=0, sticky="w")
        self.table_left = ttk.Scale(table_frame, from_=0, to=FRAME_WIDTH//2, orient="horizontal")
        self.table_left.set(DEFAULT_TABLE_BOUNDS["left"])
        self.table_left.grid(row=2, column=1, sticky="ew")
        
        ttk.Label(table_frame, text="Right:").grid(row=3, column=0, sticky="w")
        self.table_right = ttk.Scale(table_frame, from_=0, to=FRAME_WIDTH//2, orient="horizontal")
        self.table_right.set(DEFAULT_TABLE_BOUNDS["right"])
        self.table_right.grid(row=3, column=1, sticky="ew")
        
        # Bind command after all widgets are created
        self.table_top.configure(command=self._update_table_bounds)
        self.table_bottom.configure(command=self._update_table_bounds)
        self.table_left.configure(command=self._update_table_bounds)
        self.table_right.configure(command=self._update_table_bounds)
        
        table_frame.columnconfigure(1, weight=1)
        
        # Radius Frame
        radius_frame = ttk.LabelFrame(self.root, text="Puck Radius Range (pixels)", padding=10)
        radius_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(radius_frame, text="Min Radius:").grid(row=0, column=0, sticky="w")
        self.min_radius = ttk.Scale(radius_frame, from_=1, to=100, orient="horizontal")
        self.min_radius.set(DEFAULT_MIN_RADIUS)
        self.min_radius.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(radius_frame, text="Max Radius:").grid(row=1, column=0, sticky="w")
        self.max_radius = ttk.Scale(radius_frame, from_=1, to=150, orient="horizontal")
        self.max_radius.set(DEFAULT_MAX_RADIUS)
        self.max_radius.grid(row=1, column=1, sticky="ew")
        
        # Bind command after all widgets are created
        self.min_radius.configure(command=self._update_radius)
        self.max_radius.configure(command=self._update_radius)
        
        radius_frame.columnconfigure(1, weight=1)
        
        # Calibration Frame
        cal_frame = ttk.LabelFrame(self.root, text="Physical Calibration", padding=10)
        cal_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(cal_frame, text="Known Distance (meters):").grid(row=0, column=0, sticky="w")
        self.cal_distance_m = ttk.Entry(cal_frame)
        self.cal_distance_m.insert(0, "0.5")
        self.cal_distance_m.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(cal_frame, text="Distance in Pixels:").grid(row=1, column=0, sticky="w")
        self.cal_distance_px = ttk.Entry(cal_frame)
        self.cal_distance_px.insert(0, "500")
        self.cal_distance_px.grid(row=1, column=1, sticky="ew")
        
        ttk.Button(cal_frame, text="Calibrate", command=self._calibrate).grid(row=2, column=0, columnspan=2, pady=5)
        
        ttk.Label(cal_frame, text="Current Scale:").grid(row=3, column=0, sticky="w")
        self.cal_label = ttk.Label(cal_frame, text=f"{DEFAULT_METERS_PER_PIXEL:.6f} m/px")
        self.cal_label.grid(row=3, column=1, sticky="w")
        
        cal_frame.columnconfigure(1, weight=1)
        
        # Display Options Frame
        display_frame = ttk.LabelFrame(self.root, text="Display Options", padding=10)
        display_frame.pack(fill="x", padx=10, pady=5)
        
        self.show_mask_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Mask Window", variable=self.show_mask_var,
                       command=self._toggle_mask).pack(anchor="w")
        
        self.show_vector_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Velocity Vector", variable=self.show_vector_var,
                       command=self._toggle_vector).pack(anchor="w")
        
        self.show_trajectory_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Trajectory Prediction", variable=self.show_trajectory_var,
                       command=self._toggle_trajectory).pack(anchor="w")
        
        # Results Frame
        results_frame = ttk.LabelFrame(self.root, text="Current Tracking Results", padding=10)
        results_frame.pack(fill="x", padx=10, pady=5)
        
        self.result_labels = {}
        labels = ["Position (x, y):", "Velocity (vx, vy) m/s:", "Speed m/s:", "FPS:"]
        for i, label in enumerate(labels):
            ttk.Label(results_frame, text=label).grid(row=i, column=0, sticky="w")
            val_label = ttk.Label(results_frame, text="--", font=("Courier", 10))
            val_label.grid(row=i, column=1, sticky="w")
            self.result_labels[label] = val_label
    
    def _on_color_change(self):
        color = self.color_var.get()
        self.state.set_color_preset(color)
    
    def _apply_custom_hsv(self):
        hsv_low = [int(self.low_h.get()), int(self.low_s.get()), int(self.low_v.get())]
        hsv_high = [int(self.high_h.get()), int(self.high_s.get()), int(self.high_v.get())]
        self.state.set_custom_color(hsv_low, hsv_high)
        self.color_var.set("Custom")
        messagebox.showinfo("Custom HSV", "Custom HSV range applied!")
    
    def _update_roi(self, *args):
        margins = {
            "top": int(self.roi_top.get()),
            "bottom": int(self.roi_bottom.get()),
            "left": int(self.roi_left.get()),
            "right": int(self.roi_right.get())
        }
        self.state.set_roi_margins(margins)
    
    def _update_radius(self, *args):
        min_r = int(self.min_radius.get())
        max_r = int(self.max_radius.get())
        self.state.set_radius_range(min_r, max_r)
    
    def _update_table_bounds(self, *args):
        bounds = {
            "top": int(self.table_top.get()),
            "bottom": int(self.table_bottom.get()),
            "left": int(self.table_left.get()),
            "right": int(self.table_right.get())
        }
        self.state.set_table_bounds(bounds)
    
    def _calibrate(self):
        try:
            dist_m = float(self.cal_distance_m.get())
            dist_px = float(self.cal_distance_px.get())
            if dist_px <= 0:
                messagebox.showerror("Error", "Pixel distance must be > 0")
                return
            self.state.update_calibration(dist_m, dist_px)
            mpp = self.state.get_meters_per_pixel()
            self.cal_label.config(text=f"{mpp:.6f} m/px")
            messagebox.showinfo("Calibration", f"Calibrated: {mpp:.6f} meters per pixel")
        except ValueError:
            messagebox.showerror("Error", "Invalid calibration values")
    
    def _toggle_mask(self):
        self.state.show_mask = self.show_mask_var.get()
        if not self.state.show_mask:
            cv2.destroyWindow("Mask")
    
    def _toggle_vector(self):
        self.state.show_velocity_vector = self.show_vector_var.get()
    
    def _toggle_trajectory(self):
        self.state.show_trajectory = self.show_trajectory_var.get()
    
    def _start_update_loop(self):
        """Periodically update result labels"""
        def update():
            self.result_labels["Position (x, y):"].config(
                text=f"({self.state.current_x:.1f}, {self.state.current_y:.1f}) px"
            )
            self.result_labels["Velocity (vx, vy) m/s:"].config(
                text=f"({self.state.velocity_x_ms:.4f}, {self.state.velocity_y_ms:.4f})"
            )
            self.result_labels["Speed m/s:"].config(
                text=f"{self.state.speed_ms:.4f}"
            )
            self.result_labels["FPS:"].config(
                text=f"{self.state.fps:.1f}"
            )
            self.root.after(GUI_UPDATE_MS, update)
        
        update()
    
    def run(self):
        self.root.mainloop()

# -------------------- MAIN --------------------
def main():
    print("=" * 60)
    print("Hockey Puck Tracker - Advanced Edition")
    print("=" * 60)
    print("\nFeatures:")
    print("  - Optimized for 90 FPS at 480p (640x480)")
    print("  - GUI for color selection and parameter tuning")
    print("  - ROI adjustment for tracking region")
    print("  - Velocity tracking with m/s output")
    print("  - Physical calibration")
    print("  - Trajectory prediction with bounce physics")
    print("  - Adjustable table boundaries")
    print("\nPerformance:")
    print(f"  - Resolution: {FRAME_WIDTH}x{FRAME_HEIGHT}")
    print(f"  - Target FPS: {TARGET_FPS}")
    print(f"  - Camera Index: {CAM_INDEX}")
    print("\nVisual Elements:")
    print("  - Blue box: ROI (tracking region)")
    print("  - Orange box: Table bounds (trajectory boundary)")
    print("  - Green circle: Puck position")
    print("  - Cyan arrow: Velocity vector")
    print("  - Magenta line: Predicted trajectory with bounces")
    print("\nControls:")
    print("  - Use GUI to adjust all parameters")
    print("  - Press 'q' or ESC in video window to quit")
    print("\nOutput Format (console):")
    print("  x_px, y_px, vx_m/s, vy_m/s, speed_m/s")
    print("=" * 60)
    print()
    
    # Create shared state
    state = TrackerState()
    
    # Create stop event for threading
    import threading
    stop_event = threading.Event()
    
    # Start tracking thread
    tracker = Thread(target=tracking_thread, args=(state, stop_event), daemon=True)
    tracker.start()
    
    # Start GUI (blocking)
    gui = TrackerGUI(state)
    gui.run()
    
    # Cleanup
    stop_event.set()
    tracker.join(timeout=2)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
