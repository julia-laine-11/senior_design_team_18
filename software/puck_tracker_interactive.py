# Interactive Hockey Puck Tracker with Drag-and-Drop Boundaries
# Features: GUI color selection, interactive ROI/table bounds, velocity tracking (m/s), 90fps support
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
CAM_INDEX = 1  # 0=laptop, 1=USB camera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 90

# Performance optimization
SKIP_LINE_SUPPRESSION = True
MORPH_KERNEL_SIZE = 3
VISUAL_UPDATE_SKIP = 0
GUI_UPDATE_MS = 100

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

DEFAULT_METERS_PER_PIXEL = 0.001
DEFAULT_ROI_MARGINS = {"top": 50, "bottom": 50, "left": 50, "right": 50}
DEFAULT_MIN_RADIUS = 5
DEFAULT_MAX_RADIUS = 50
DEFAULT_TABLE_BOUNDS = {"top": 100, "bottom": 100, "left": 100, "right": 100}
TRAJECTORY_PREDICTION_TIME = 2.0
TRAJECTORY_DAMPING = 0.95

# Kalman filter tuning
KF_PROCESS_NOISE_POS = 0.05
KF_PROCESS_NOISE_VEL = 4.0
KF_MEAS_NOISE = 2.0

# -------------------- INTERACTIVE MOUSE HANDLER --------------------
class InteractiveBoundaryEditor:
    """Handle mouse interactions for dragging boundaries"""
    def __init__(self):
        self.mode = "setup"  # "setup" or "tracking"
        self.dragging = None  # None, "roi_top", "roi_bottom", etc.
        self.mouse_x = 0
        self.mouse_y = 0
        self.snap_distance = 20  # pixels for edge detection
        
    def mouse_callback(self, event, x, y, flags, param):
        """OpenCV mouse callback"""
        self.mouse_x = x
        self.mouse_y = y
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # Check if near any boundary edge
            self.dragging = self._check_near_edge(x, y, param)
        
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = None
        
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            # Update the boundary being dragged
            self._update_boundary(x, y, param)
    
    def _check_near_edge(self, x, y, state):
        """Check if mouse is near any boundary edge"""
        if self.mode == "tracking":
            return None  # Don't allow dragging during tracking
        
        frame_w = FRAME_WIDTH
        frame_h = FRAME_HEIGHT
        
        # Get current boundaries
        roi = state.get_roi_margins()
        table = state.get_table_bounds()
        
        # Check ROI edges
        if abs(y - roi["top"]) < self.snap_distance and roi["left"] < x < frame_w - roi["right"]:
            return ("roi", "top")
        if abs(y - (frame_h - roi["bottom"])) < self.snap_distance and roi["left"] < x < frame_w - roi["right"]:
            return ("roi", "bottom")
        if abs(x - roi["left"]) < self.snap_distance and roi["top"] < y < frame_h - roi["bottom"]:
            return ("roi", "left")
        if abs(x - (frame_w - roi["right"])) < self.snap_distance and roi["top"] < y < frame_h - roi["bottom"]:
            return ("roi", "right")
        
        # Check table bounds edges
        if abs(y - table["top"]) < self.snap_distance and table["left"] < x < frame_w - table["right"]:
            return ("table", "top")
        if abs(y - (frame_h - table["bottom"])) < self.snap_distance and table["left"] < x < frame_w - table["right"]:
            return ("table", "bottom")
        if abs(x - table["left"]) < self.snap_distance and table["top"] < y < frame_h - table["bottom"]:
            return ("table", "left")
        if abs(x - (frame_w - table["right"])) < self.snap_distance and table["top"] < y < frame_h - table["bottom"]:
            return ("table", "right")
        
        return None
    
    def _update_boundary(self, x, y, state):
        """Update boundary position based on drag"""
        if not self.dragging:
            return
        
        boundary_type, edge = self.dragging
        frame_w = FRAME_WIDTH
        frame_h = FRAME_HEIGHT
        
        if boundary_type == "roi":
            margins = state.get_roi_margins()
            if edge == "top":
                margins["top"] = max(0, min(y, frame_h // 2))
            elif edge == "bottom":
                margins["bottom"] = max(0, min(frame_h - y, frame_h // 2))
            elif edge == "left":
                margins["left"] = max(0, min(x, frame_w // 2))
            elif edge == "right":
                margins["right"] = max(0, min(frame_w - x, frame_w // 2))
            state.set_roi_margins(margins)
        
        elif boundary_type == "table":
            bounds = state.get_table_bounds()
            if edge == "top":
                bounds["top"] = max(0, min(y, frame_h // 2))
            elif edge == "bottom":
                bounds["bottom"] = max(0, min(frame_h - y, frame_h // 2))
            elif edge == "left":
                bounds["left"] = max(0, min(x, frame_w // 2))
            elif edge == "right":
                bounds["right"] = max(0, min(frame_w - x, frame_w // 2))
            state.set_table_bounds(bounds)
    
    def draw_hover_indicator(self, vis, state):
        """Draw visual feedback for draggable edges"""
        if self.mode == "tracking":
            return vis
        
        near_edge = self._check_near_edge(self.mouse_x, self.mouse_y, state)
        
        if near_edge:
            boundary_type, edge = near_edge
            frame_w = FRAME_WIDTH
            frame_h = FRAME_HEIGHT
            
            if boundary_type == "roi":
                margins = state.get_roi_margins()
                color = (255, 255, 0)  # Cyan for ROI
                
                if edge == "top":
                    cv2.line(vis, (margins["left"], margins["top"]), 
                            (frame_w - margins["right"], margins["top"]), color, 4)
                elif edge == "bottom":
                    cv2.line(vis, (margins["left"], frame_h - margins["bottom"]), 
                            (frame_w - margins["right"], frame_h - margins["bottom"]), color, 4)
                elif edge == "left":
                    cv2.line(vis, (margins["left"], margins["top"]), 
                            (margins["left"], frame_h - margins["bottom"]), color, 4)
                elif edge == "right":
                    cv2.line(vis, (frame_w - margins["right"], margins["top"]), 
                            (frame_w - margins["right"], frame_h - margins["bottom"]), color, 4)
            
            elif boundary_type == "table":
                bounds = state.get_table_bounds()
                color = (0, 255, 255)  # Yellow for table
                
                if edge == "top":
                    cv2.line(vis, (bounds["left"], bounds["top"]), 
                            (frame_w - bounds["right"], bounds["top"]), color, 4)
                elif edge == "bottom":
                    cv2.line(vis, (bounds["left"], frame_h - bounds["bottom"]), 
                            (frame_w - bounds["right"], frame_h - bounds["bottom"]), color, 4)
                elif edge == "left":
                    cv2.line(vis, (bounds["left"], bounds["top"]), 
                            (bounds["left"], frame_h - bounds["bottom"]), color, 4)
                elif edge == "right":
                    cv2.line(vis, (frame_w - bounds["right"], bounds["top"]), 
                            (frame_w - bounds["right"], frame_h - bounds["bottom"]), color, 4)
        
        return vis

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
        self.selected_color = "Black/Dark"
        self.hsv_low = np.array(COLOR_PRESETS["Black/Dark"]["low"])
        self.hsv_high = np.array(COLOR_PRESETS["Black/Dark"]["high"])
        self.roi_margins = DEFAULT_ROI_MARGINS.copy()
        self.table_bounds = DEFAULT_TABLE_BOUNDS.copy()
        self.min_radius = DEFAULT_MIN_RADIUS
        self.max_radius = DEFAULT_MAX_RADIUS
        self.meters_per_pixel = DEFAULT_METERS_PER_PIXEL
        self.calibration_distance_m = 0.5
        self.calibration_distance_px = 500
        self.tracking_enabled = True
        self.show_mask = True
        self.show_velocity_vector = True
        self.show_trajectory = True
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
        
        prox_penalty = 0.0
        if kf_pred_xy is not None:
            px, py = kf_pred_xy
            d = np.hypot(x - px, y - py)
            prox_penalty = 0.002 * d
        
        score = circularity - prox_penalty
        
        if (best is None) or (score > best[3]):
            best = (float(x), float(y), float(rad), float(score))
    
    return best

def predict_trajectory_bounded(x, y, vx_px, vy_px, frame_w, frame_h, bounds, max_time=2.0, dt=0.016, damping=0.95):
    """Predict trajectory with proper bounds checking"""
    left = bounds['left']
    right = frame_w - bounds['right']
    top = bounds['top']
    bottom = frame_h - bounds['bottom']
    
    points = [(x, y)]
    px, py = x, y
    vx, vy = vx_px, vy_px
    
    time_elapsed = 0
    max_bounces = 20
    bounces = 0
    
    while time_elapsed < max_time and bounces < max_bounces:
        px += vx * dt
        py += vy * dt
        
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
        time_elapsed += dt
        
        if abs(vx) < 5 and abs(vy) < 5:
            break
    
    return points

# -------------------- TRACKING THREAD --------------------
def tracking_thread(state, stop_event, mouse_handler):
    """Main tracking loop running in separate thread"""
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    
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
    
    # Set up mouse callback
    cv2.namedWindow("Puck Tracker")
    cv2.setMouseCallback("Puck Tracker", mouse_handler.mouse_callback, state)
    
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
        
        # Create ROI mask
        roi_mask = make_roi_mask(h, w, margins)
        
        # Preprocessing
        blurred = cv2.GaussianBlur(frame, (3, 3), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Color threshold
        mask_color = cv2.inRange(hsv, hsv_low, hsv_high)
        
        # Suppress table lines (optional)
        if not SKIP_LINE_SUPPRESSION:
            keep_mask = suppress_table_lines(hsv)
            mask_color = cv2.bitwise_and(mask_color, keep_mask)
        
        # Apply ROI
        combined = cv2.bitwise_and(mask_color, roi_mask)
        
        # Morphological cleanup
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
        
        # Console output
        print(f"{int(fx)},{int(fy)},{vx_ms:.4f},{vy_ms:.4f},{speed_ms:.4f}", flush=True)
        
        # Visualization
        frame_counter += 1
        if frame_counter % (VISUAL_UPDATE_SKIP + 1) != 0:
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                stop_event.set()
                break
            elif key == ord('s'):
                mouse_handler.mode = "setup" if mouse_handler.mode == "tracking" else "tracking"
                print(f"Mode: {mouse_handler.mode}")
            continue
        
        vis = frame.copy()
        
        # Draw ROI
        cv2.rectangle(vis,
                     (margins["left"], margins["top"]),
                     (w - margins["right"], h - margins["bottom"]),
                     (255, 0, 0), 2)
        
        # Draw table bounds
        cv2.rectangle(vis,
                     (table_bounds["left"], table_bounds["top"]),
                     (w - table_bounds["right"], h - table_bounds["bottom"]),
                     (255, 165, 0), 2)
        
        # Draw trajectory prediction
        if state.show_trajectory and speed_ms > 0.1 and mouse_handler.mode == "tracking":
            traj_points = predict_trajectory_bounded(
                fx, fy, vx_px, vy_px, w, h, table_bounds,
                max_time=TRAJECTORY_PREDICTION_TIME,
                damping=TRAJECTORY_DAMPING
            )
            if len(traj_points) > 1:
                for i in range(len(traj_points) - 1):
                    pt1 = (int(traj_points[i][0]), int(traj_points[i][1]))
                    pt2 = (int(traj_points[i+1][0]), int(traj_points[i+1][1]))
                    cv2.line(vis, pt1, pt2, (255, 0, 255), 2)
                if len(traj_points) > 0:
                    end_pt = (int(traj_points[-1][0]), int(traj_points[-1][1]))
                    cv2.circle(vis, end_pt, 8, (255, 0, 255), -1)
        
        # Draw puck position
        if mouse_handler.mode == "tracking":
            cv2.circle(vis, (int(fx), int(fy)), max(5, int(max_r)), (0, 255, 0), 2)
            
            # Draw velocity vector
            if state.show_velocity_vector and speed_ms > 0.01:
                vector_scale = 50
                end_x = int(fx + vx_ms * vector_scale)
                end_y = int(fy + vy_ms * vector_scale)
                cv2.arrowedLine(vis, (int(fx), int(fy)), (end_x, end_y), (0, 255, 255), 2, tipLength=0.3)
        
        # Draw hover indicator for draggable edges
        vis = mouse_handler.draw_hover_indicator(vis, state)
        
        # Draw info text
        info_y = 30
        mode_color = (0, 255, 0) if mouse_handler.mode == "tracking" else (0, 165, 255)
        cv2.putText(vis, f"Mode: {mouse_handler.mode.upper()} (press 'S' to toggle)", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2)
        info_y += 25
        
        if mouse_handler.mode == "tracking":
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
        else:
            cv2.putText(vis, "Drag edges to adjust boundaries", (10, info_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            info_y += 25
            cv2.putText(vis, "Blue = ROI (tracking area)", (10, info_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            info_y += 20
            cv2.putText(vis, "Orange = Table bounds (trajectory)", (10, info_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 2)
        
        # Show windows
        cv2.imshow("Puck Tracker", vis)
        if state.show_mask:
            cv2.imshow("Mask", combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
            stop_event.set()
            break
        elif key == ord('s'):
            mouse_handler.mode = "setup" if mouse_handler.mode == "tracking" else "tracking"
            print(f"Mode: {mouse_handler.mode}")
    
    cap.release()
    cv2.destroyAllWindows()

# -------------------- GUI (SIMPLIFIED) --------------------
class TrackerGUI:
    def __init__(self, state):
        self.state = state
        self.root = tk.Tk()
        self.root.title("Interactive Puck Tracker Control Panel")
        self.root.geometry("500x600")
        
        self._create_widgets()
        self._start_update_loop()
    
    def _create_widgets(self):
        # Instructions
        info_frame = ttk.LabelFrame(self.root, text="Interactive Mode", padding=10)
        info_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(info_frame, text="Press 'S' in video window to toggle Setup/Tracking mode", 
                 font=("Arial", 10, "bold")).pack()
        ttk.Label(info_frame, text="In Setup mode: Drag boundary edges with mouse").pack()
        ttk.Label(info_frame, text="In Tracking mode: Watch puck trajectory").pack()
        
        # Color Selection
        color_frame = ttk.LabelFrame(self.root, text="Puck Color", padding=10)
        color_frame.pack(fill="x", padx=10, pady=5)
        
        self.color_var = tk.StringVar(value="Black/Dark")
        for color in list(COLOR_PRESETS.keys())[:4]:  # Show first 4
            rb = ttk.Radiobutton(color_frame, text=color, variable=self.color_var,
                                value=color, command=self._on_color_change)
            rb.pack(anchor="w")
        
        # Radius Range
        radius_frame = ttk.LabelFrame(self.root, text="Puck Radius (pixels)", padding=10)
        radius_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(radius_frame, text="Min:").grid(row=0, column=0)
        self.min_radius = ttk.Scale(radius_frame, from_=1, to=50, orient="horizontal")
        self.min_radius.set(DEFAULT_MIN_RADIUS)
        self.min_radius.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(radius_frame, text="Max:").grid(row=1, column=0)
        self.max_radius = ttk.Scale(radius_frame, from_=10, to=100, orient="horizontal")
        self.max_radius.set(DEFAULT_MAX_RADIUS)
        self.max_radius.grid(row=1, column=1, sticky="ew")
        
        self.min_radius.configure(command=self._update_radius)
        self.max_radius.configure(command=self._update_radius)
        radius_frame.columnconfigure(1, weight=1)
        
        # Calibration
        cal_frame = ttk.LabelFrame(self.root, text="Calibration", padding=10)
        cal_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(cal_frame, text="Distance (m):").grid(row=0, column=0)
        self.cal_distance_m = ttk.Entry(cal_frame, width=10)
        self.cal_distance_m.insert(0, "0.5")
        self.cal_distance_m.grid(row=0, column=1)
        
        ttk.Label(cal_frame, text="Pixels:").grid(row=1, column=0)
        self.cal_distance_px = ttk.Entry(cal_frame, width=10)
        self.cal_distance_px.insert(0, "500")
        self.cal_distance_px.grid(row=1, column=1)
        
        ttk.Button(cal_frame, text="Calibrate", command=self._calibrate).grid(row=2, column=0, columnspan=2, pady=5)
        self.cal_label = ttk.Label(cal_frame, text=f"{DEFAULT_METERS_PER_PIXEL:.6f} m/px")
        self.cal_label.grid(row=3, column=0, columnspan=2)
        
        # Display Options
        display_frame = ttk.LabelFrame(self.root, text="Display", padding=10)
        display_frame.pack(fill="x", padx=10, pady=5)
        
        self.show_mask_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Mask", variable=self.show_mask_var,
                       command=self._toggle_mask).pack(anchor="w")
        
        self.show_vector_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Velocity", variable=self.show_vector_var,
                       command=self._toggle_vector).pack(anchor="w")
        
        self.show_trajectory_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Trajectory", variable=self.show_trajectory_var,
                       command=self._toggle_trajectory).pack(anchor="w")
        
        # Results
        results_frame = ttk.LabelFrame(self.root, text="Tracking Results", padding=10)
        results_frame.pack(fill="x", padx=10, pady=5)
        
        self.result_labels = {}
        labels = ["Position:", "Speed:", "FPS:"]
        for i, label in enumerate(labels):
            ttk.Label(results_frame, text=label).grid(row=i, column=0, sticky="w")
            val_label = ttk.Label(results_frame, text="--", font=("Courier", 10))
            val_label.grid(row=i, column=1, sticky="w")
            self.result_labels[label] = val_label
    
    def _on_color_change(self):
        self.state.set_color_preset(self.color_var.get())
    
    def _update_radius(self, *args):
        self.state.set_radius_range(int(self.min_radius.get()), int(self.max_radius.get()))
    
    def _calibrate(self):
        try:
            dist_m = float(self.cal_distance_m.get())
            dist_px = float(self.cal_distance_px.get())
            if dist_px <= 0:
                messagebox.showerror("Error", "Pixels must be > 0")
                return
            self.state.update_calibration(dist_m, dist_px)
            mpp = self.state.get_meters_per_pixel()
            self.cal_label.config(text=f"{mpp:.6f} m/px")
            messagebox.showinfo("Success", f"Calibrated: {mpp:.6f} m/px")
        except ValueError:
            messagebox.showerror("Error", "Invalid values")
    
    def _toggle_mask(self):
        self.state.show_mask = self.show_mask_var.get()
    
    def _toggle_vector(self):
        self.state.show_velocity_vector = self.show_vector_var.get()
    
    def _toggle_trajectory(self):
        self.state.show_trajectory = self.show_trajectory_var.get()
    
    def _start_update_loop(self):
        def update():
            self.result_labels["Position:"].config(
                text=f"({self.state.current_x:.0f}, {self.state.current_y:.0f}) px"
            )
            self.result_labels["Speed:"].config(
                text=f"{self.state.speed_ms:.3f} m/s"
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
    print("Interactive Hockey Puck Tracker")
    print("=" * 60)
    print("\nFeatures:")
    print("  - Drag-and-drop boundary adjustment")
    print("  - Press 'S' to toggle Setup/Tracking mode")
    print("  - Optimized for 90 FPS at 480p")
    print("\nControls:")
    print("  - Setup Mode: Drag blue/orange edges to adjust")
    print("  - Tracking Mode: Watch puck and trajectory")
    print("  - 'S' key: Toggle modes")
    print("  - 'Q' or ESC: Quit")
    print("=" * 60)
    print()
    
    state = TrackerState()
    mouse_handler = InteractiveBoundaryEditor()
    
    import threading
    stop_event = threading.Event()
    
    tracker = Thread(target=tracking_thread, args=(state, stop_event, mouse_handler), daemon=True)
    tracker.start()
    
    gui = TrackerGUI(state)
    gui.run()
    
    stop_event.set()
    tracker.join(timeout=2)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
