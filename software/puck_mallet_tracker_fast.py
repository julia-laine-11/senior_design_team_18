# High-Performance Hockey Puck & Mallet Tracker
# OpenCV display + Tkinter control panel
# Optimized for multi-core CPU with SIMD (AVX2) support

import numpy as np
import time
import cv2
import tkinter as tk
from tkinter import ttk, messagebox
from threading import Thread, Lock, Event

# -------------------- CONFIGURATION --------------------
CAM_INDEX = 2  # Global Shutter USB Camera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Processing scale (0.5 = half resolution for detection, faster)
PROCESSING_SCALE = 0.5

# HSV ranges for detection
PUCK_HSV_LOW = np.array([40, 50, 50], dtype=np.uint8)    # Green
PUCK_HSV_HIGH = np.array([80, 255, 255], dtype=np.uint8)
MALLET_HSV_LOW = np.array([5, 100, 100], dtype=np.uint8)  # Orange
MALLET_HSV_HIGH = np.array([25, 255, 255], dtype=np.uint8)

# Detection parameters
PUCK_MIN_RADIUS = 5
PUCK_MAX_RADIUS = 30
MALLET_MIN_RADIUS = 15
MALLET_MAX_RADIUS = 60

# Trajectory prediction
TRAJECTORY_TIME = 2.0
TRAJECTORY_DAMPING = 0.95

# Table bounds (pixels from edge)
TABLE_BOUNDS = {"top": 50, "bottom": 50, "left": 50, "right": 50}

# ROI margins (pixels from edge)
ROI_MARGINS = {"top": 20, "bottom": 20, "left": 20, "right": 20}

# Display options (controlled by GUI)
SHOW_MASK = True
SHOW_ROI = True
GUI_UPDATE_MS = 100

# -------------------- SETUP OPENCV FOR MAX PERFORMANCE --------------------
print("=" * 60)
print("High-Performance Puck & Mallet Tracker")
print("=" * 60)

# Use all CPU threads
num_threads = cv2.getNumThreads()
print(f"OpenCV threads: {num_threads}")

# Try to enable OpenCL
use_opencl = False
try:
    cv2.ocl.setUseOpenCL(True)
    if cv2.ocl.haveOpenCL() and cv2.ocl.useOpenCL():
        use_opencl = True
        print("OpenCL: ENABLED")
    else:
        print("OpenCL: Not available")
except:
    print("OpenCL: Failed to enable")

print(f"Processing scale: {PROCESSING_SCALE}")
print("=" * 60)

# -------------------- THREAD-SAFE STATE --------------------
class TrackerState:
    """Thread-safe state shared between tracker and GUI"""
    def __init__(self):
        self.lock = Lock()
        self.roi_margins = ROI_MARGINS.copy()
        self.table_bounds = TABLE_BOUNDS.copy()
        self.show_mask = SHOW_MASK
        self.show_roi = SHOW_ROI
        self.show_trajectory = True
        
        # Tracking results
        self.puck_x = 0.0
        self.puck_y = 0.0
        self.puck_speed = 0.0
        self.puck_detected = False
        self.mallet_x = 0.0
        self.mallet_y = 0.0
        self.mallet_detected = False
        self.fps = 0.0
        self.cap_ms = 0.0
        self.proc_ms = 0.0
    
    def get_roi(self):
        with self.lock:
            return self.roi_margins.copy()
    
    def set_roi(self, margins):
        with self.lock:
            self.roi_margins = margins.copy()
    
    def get_table_bounds(self):
        with self.lock:
            return self.table_bounds.copy()
    
    def set_table_bounds(self, bounds):
        with self.lock:
            self.table_bounds = bounds.copy()
    
    def update_tracking(self, px, py, pspeed, pdet, mx, my, mdet, fps, cap_ms, proc_ms):
        with self.lock:
            self.puck_x, self.puck_y = px, py
            self.puck_speed = pspeed
            self.puck_detected = pdet
            self.mallet_x, self.mallet_y = mx, my
            self.mallet_detected = mdet
            self.fps = fps
            self.cap_ms = cap_ms
            self.proc_ms = proc_ms

# -------------------- KALMAN FILTER --------------------
class KalmanTracker:
    def __init__(self, x=0, y=0):
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1]], dtype=np.float32)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]], dtype=np.float32)
        self.kf.processNoiseCov = np.diag([0.05, 0.05, 4.0, 4.0]).astype(np.float32)
        self.kf.measurementNoiseCov = 2.0 * np.eye(2, dtype=np.float32)
        self.kf.statePost = np.array([[x], [y], [0], [0]], dtype=np.float32)
        self._last_t = time.perf_counter()
        self._meas = np.zeros((2, 1), dtype=np.float32)
    
    def predict(self):
        now = time.perf_counter()
        dt = max(0.001, now - self._last_t)
        self._last_t = now
        self.kf.transitionMatrix[0, 2] = dt
        self.kf.transitionMatrix[1, 3] = dt
        p = self.kf.predict()
        return p[0, 0], p[1, 0], p[2, 0], p[3, 0]
    
    def correct(self, x, y):
        self._meas[0, 0], self._meas[1, 0] = x, y
        c = self.kf.correct(self._meas)
        return c[0, 0], c[1, 0], c[2, 0], c[3, 0]

# -------------------- DETECTION --------------------
def find_circle(contours, min_r, max_r, pred_xy=None):
    best = None
    best_score = -1
    px, py = pred_xy if pred_xy else (None, None)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 10:
            continue
        (x, y), r = cv2.minEnclosingCircle(cnt)
        if r < min_r or r > max_r:
            continue
        per = cv2.arcLength(cnt, True)
        if per < 1:
            continue
        circ = 4 * 3.14159 * area / (per * per)
        score = circ
        if px is not None:
            score -= 0.000004 * ((x - px)**2 + (y - py)**2)
        if score > best_score:
            best_score = score
            best = (x, y, r)
    return best

def predict_trajectory(x, y, vx, vy, w, h, bounds, max_t=2.0, dt=0.02):
    points = [(int(x), int(y))]
    left, right = bounds['left'], w - bounds['right']
    top, bottom = bounds['top'], h - bounds['bottom']
    
    for _ in range(int(max_t / dt)):
        x += vx * dt
        y += vy * dt
        
        if x <= left:
            x, vx = left, abs(vx) * TRAJECTORY_DAMPING
        elif x >= right:
            x, vx = right, -abs(vx) * TRAJECTORY_DAMPING
        if y <= top:
            y, vy = top, abs(vy) * TRAJECTORY_DAMPING
        elif y >= bottom:
            y, vy = bottom, -abs(vy) * TRAJECTORY_DAMPING
        
        points.append((int(x), int(y)))
        if vx*vx + vy*vy < 25:
            break
    return points

# -------------------- TKINTER GUI --------------------
class ControlGUI:
    def __init__(self, state):
        self.state = state
        self.root = tk.Tk()
        self.root.title("Tracker Controls")
        self.root.geometry("400x600")
        self._create_widgets()
        self._start_update()
    
    def _create_widgets(self):
        # Info
        info = ttk.LabelFrame(self.root, text="Info", padding=10)
        info.pack(fill="x", padx=10, pady=5)
        ttk.Label(info, text="GREEN=Puck | ORANGE=Mallet", font=("Arial", 9, "bold")).pack()
        self.fps_label = ttk.Label(info, text="FPS: --", font=("Courier", 10))
        self.fps_label.pack()
        
        # ROI Margins
        roi = ttk.LabelFrame(self.root, text="ROI Margins", padding=10)
        roi.pack(fill="x", padx=10, pady=5)
        
        self.roi_sliders = {}
        for i, (name, default) in enumerate([("top", ROI_MARGINS["top"]), 
                                              ("bottom", ROI_MARGINS["bottom"]),
                                              ("left", ROI_MARGINS["left"]), 
                                              ("right", ROI_MARGINS["right"])]):
            ttk.Label(roi, text=f"{name.title()}:").grid(row=i, column=0, sticky="w")
            s = ttk.Scale(roi, from_=0, to=200, orient="horizontal")
            s.set(default)
            s.grid(row=i, column=1, sticky="ew")
            s.configure(command=lambda v, n=name: self._update_roi(n, v))
            self.roi_sliders[name] = s
        roi.columnconfigure(1, weight=1)
        
        # Table Bounds
        table = ttk.LabelFrame(self.root, text="Table Bounds", padding=10)
        table.pack(fill="x", padx=10, pady=5)
        
        self.table_sliders = {}
        for i, (name, default) in enumerate([("top", TABLE_BOUNDS["top"]), 
                                              ("bottom", TABLE_BOUNDS["bottom"]),
                                              ("left", TABLE_BOUNDS["left"]), 
                                              ("right", TABLE_BOUNDS["right"])]):
            ttk.Label(table, text=f"{name.title()}:").grid(row=i, column=0, sticky="w")
            s = ttk.Scale(table, from_=0, to=200, orient="horizontal")
            s.set(default)
            s.grid(row=i, column=1, sticky="ew")
            s.configure(command=lambda v, n=name: self._update_table(n, v))
            self.table_sliders[name] = s
        table.columnconfigure(1, weight=1)
        
        # Display Options
        disp = ttk.LabelFrame(self.root, text="Display", padding=10)
        disp.pack(fill="x", padx=10, pady=5)
        
        self.show_mask_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(disp, text="Show Mask Window", variable=self.show_mask_var,
                       command=self._toggle_mask).pack(anchor="w")
        
        self.show_roi_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(disp, text="Show ROI Rectangle", variable=self.show_roi_var,
                       command=self._toggle_roi).pack(anchor="w")
        
        self.show_traj_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(disp, text="Show Trajectory", variable=self.show_traj_var,
                       command=self._toggle_traj).pack(anchor="w")
        
        # Results
        results = ttk.LabelFrame(self.root, text="Tracking Results", padding=10)
        results.pack(fill="x", padx=10, pady=5)
        
        self.puck_label = ttk.Label(results, text="Puck: --", font=("Courier", 9))
        self.puck_label.pack(anchor="w")
        self.mallet_label = ttk.Label(results, text="Mallet: --", font=("Courier", 9))
        self.mallet_label.pack(anchor="w")
        self.perf_label = ttk.Label(results, text="Cap: -- | Proc: --", font=("Courier", 9))
        self.perf_label.pack(anchor="w")
    
    def _update_roi(self, name, val):
        margins = self.state.get_roi()
        margins[name] = int(float(val))
        self.state.set_roi(margins)
    
    def _update_table(self, name, val):
        bounds = self.state.get_table_bounds()
        bounds[name] = int(float(val))
        self.state.set_table_bounds(bounds)
    
    def _toggle_mask(self):
        self.state.show_mask = self.show_mask_var.get()
    
    def _toggle_roi(self):
        self.state.show_roi = self.show_roi_var.get()
    
    def _toggle_traj(self):
        self.state.show_trajectory = self.show_traj_var.get()
    
    def _start_update(self):
        def update():
            s = self.state
            pstat = "YES" if s.puck_detected else "NO"
            mstat = "YES" if s.mallet_detected else "NO"
            self.fps_label.config(text=f"FPS: {s.fps:.0f}")
            self.puck_label.config(text=f"Puck: ({int(s.puck_x)}, {int(s.puck_y)}) {s.puck_speed:.2f}m/s [{pstat}]")
            self.mallet_label.config(text=f"Mallet: ({int(s.mallet_x)}, {int(s.mallet_y)}) [{mstat}]")
            self.perf_label.config(text=f"Cap: {s.cap_ms:.1f}ms | Proc: {s.proc_ms:.1f}ms")
            self.root.after(GUI_UPDATE_MS, update)
        update()
    
    def run(self):
        self.root.mainloop()

# -------------------- TRACKING THREAD --------------------
def tracking_thread(state, stop_event):
    """Main tracking loop - runs in background thread"""
    print(f"\nOpening camera {CAM_INDEX}...")
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 90)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    # CRITICAL: Enable auto-exposure to fix 1 FPS issue on Global Shutter cameras
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)  # 3 = auto, 1 = manual
    
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAM_INDEX)
    
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read camera")
        return
    
    h, w = frame.shape[:2]
    print(f"Camera: {w}x{h} @ {cap.get(cv2.CAP_PROP_FPS):.0f} FPS")
    
    # Processing dimensions
    proc_w, proc_h = int(w * PROCESSING_SCALE), int(h * PROCESSING_SCALE)
    scale_inv = 1.0 / PROCESSING_SCALE
    print(f"Processing: {proc_w}x{proc_h}")
    
    # Pre-allocate buffers
    hsv_buf = np.empty((proc_h, proc_w, 3), dtype=np.uint8)
    puck_mask = np.empty((proc_h, proc_w), dtype=np.uint8)
    mallet_mask = np.empty((proc_h, proc_w), dtype=np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    
    # Create ROI mask for processing resolution
    roi_mask = np.zeros((proc_h, proc_w), dtype=np.uint8)
    
    def update_roi_mask(margins):
        roi_mask.fill(0)
        roi_t = int(margins['top'] * PROCESSING_SCALE)
        roi_b = int(margins['bottom'] * PROCESSING_SCALE)
        roi_l = int(margins['left'] * PROCESSING_SCALE)
        roi_r = int(margins['right'] * PROCESSING_SCALE)
        cv2.rectangle(roi_mask, (roi_l, roi_t), (proc_w - roi_r, proc_h - roi_b), 255, -1)
    
    update_roi_mask(state.get_roi())
    
    # Kalman filters
    puck_kf = KalmanTracker(w // 2, h // 2)
    mallet_kf = KalmanTracker(w // 2, h // 4)
    
    # Lost tracking counters (frames since last detection)
    puck_lost_frames = 0
    mallet_lost_frames = 0
    LOST_THRESHOLD = 15  # After this many frames, ignore Kalman prediction for search
    
    # FPS tracking
    fps_alpha = 0.1
    fps = 0
    last_time = time.perf_counter()
    frame_count = 0
    
    print("\nPress 'Q' to quit, 'S' to save screenshot")
    print("-" * 60)
    
    # Cache for ROI/table changes
    last_roi = state.get_roi()
    
    while not stop_event.is_set():
        t0 = time.perf_counter()
        
        ret, frame = cap.read()
        if not ret:
            break
        t_cap = time.perf_counter()
        
        # Check for ROI changes from GUI
        current_roi = state.get_roi()
        if current_roi != last_roi:
            update_roi_mask(current_roi)
            last_roi = current_roi
        
        # Get current table bounds
        table_bounds = state.get_table_bounds()
        
        # Resize for processing
        small = cv2.resize(frame, (proc_w, proc_h), interpolation=cv2.INTER_LINEAR)
        
        # Use UMat for GPU acceleration if OpenCL available
        if use_opencl:
            small_gpu = cv2.UMat(small)
            roi_gpu = cv2.UMat(roi_mask)
            blurred = cv2.GaussianBlur(small_gpu, (3, 3), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            puck_m = cv2.inRange(hsv, PUCK_HSV_LOW, PUCK_HSV_HIGH)
            puck_m = cv2.bitwise_and(puck_m, roi_gpu)
            puck_m = cv2.morphologyEx(puck_m, cv2.MORPH_OPEN, kernel)
            puck_m = cv2.morphologyEx(puck_m, cv2.MORPH_CLOSE, kernel)
            mallet_m = cv2.inRange(hsv, MALLET_HSV_LOW, MALLET_HSV_HIGH)
            mallet_m = cv2.bitwise_and(mallet_m, roi_gpu)
            mallet_m = cv2.morphologyEx(mallet_m, cv2.MORPH_OPEN, kernel)
            mallet_m = cv2.morphologyEx(mallet_m, cv2.MORPH_CLOSE, kernel)
            puck_mask = puck_m.get()
            mallet_mask = mallet_m.get()
        else:
            cv2.GaussianBlur(small, (3, 3), 0, dst=small)
            cv2.cvtColor(small, cv2.COLOR_BGR2HSV, dst=hsv_buf)
            cv2.inRange(hsv_buf, PUCK_HSV_LOW, PUCK_HSV_HIGH, dst=puck_mask)
            cv2.bitwise_and(puck_mask, roi_mask, dst=puck_mask)
            cv2.morphologyEx(puck_mask, cv2.MORPH_OPEN, kernel, dst=puck_mask)
            cv2.morphologyEx(puck_mask, cv2.MORPH_CLOSE, kernel, dst=puck_mask)
            cv2.inRange(hsv_buf, MALLET_HSV_LOW, MALLET_HSV_HIGH, dst=mallet_mask)
            cv2.bitwise_and(mallet_mask, roi_mask, dst=mallet_mask)
            cv2.morphologyEx(mallet_mask, cv2.MORPH_OPEN, kernel, dst=mallet_mask)
            cv2.morphologyEx(mallet_mask, cv2.MORPH_CLOSE, kernel, dst=mallet_mask)
        
        # Find contours
        puck_cnts, _ = cv2.findContours(puck_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mallet_cnts, _ = cv2.findContours(mallet_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        t_proc = time.perf_counter()
        
        # ROI limits for clamping positions
        roi_left = current_roi['left']
        roi_right = w - current_roi['right']
        roi_top = current_roi['top']
        roi_bottom = h - current_roi['bottom']
        
        # Track puck
        px, py, pvx, pvy = puck_kf.predict()
        
        # If lost for too long, search without prediction bias (search whole ROI)
        if puck_lost_frames >= LOST_THRESHOLD:
            pred_hint = None  # Search everywhere
        else:
            pred_hint = (px * PROCESSING_SCALE, py * PROCESSING_SCALE)
        
        puck = find_circle(puck_cnts, PUCK_MIN_RADIUS * PROCESSING_SCALE, 
                          PUCK_MAX_RADIUS * PROCESSING_SCALE, pred_hint)
        puck_det = puck is not None
        
        if puck_det:
            px, py, pr = puck[0] * scale_inv, puck[1] * scale_inv, puck[2] * scale_inv
            
            # If we were lost, reset Kalman filter to new position
            if puck_lost_frames >= LOST_THRESHOLD:
                puck_kf = KalmanTracker(px, py)
                pvx, pvy = 0, 0
            else:
                px, py, pvx, pvy = puck_kf.correct(px, py)
            
            puck_lost_frames = 0
        else:
            puck_lost_frames += 1
        
        # Clamp puck position to ROI
        px = max(roi_left, min(roi_right, px))
        py = max(roi_top, min(roi_bottom, py))
        
        # Track mallet
        mx, my, mvx, mvy = mallet_kf.predict()
        
        # Same lost-tracking logic for mallet
        if mallet_lost_frames >= LOST_THRESHOLD:
            mallet_pred_hint = None
        else:
            mallet_pred_hint = (mx * PROCESSING_SCALE, my * PROCESSING_SCALE)
        
        mallet = find_circle(mallet_cnts, MALLET_MIN_RADIUS * PROCESSING_SCALE,
                            MALLET_MAX_RADIUS * PROCESSING_SCALE, mallet_pred_hint)
        mallet_det = mallet is not None
        
        if mallet_det:
            mx, my, mr = mallet[0] * scale_inv, mallet[1] * scale_inv, mallet[2] * scale_inv
            
            if mallet_lost_frames >= LOST_THRESHOLD:
                mallet_kf = KalmanTracker(mx, my)
            else:
                mx, my, _, _ = mallet_kf.correct(mx, my)
            
            mallet_lost_frames = 0
        else:
            mallet_lost_frames += 1
            mr = MALLET_MIN_RADIUS
        
        # Clamp mallet position to ROI
        mx = max(roi_left, min(roi_right, mx))
        my = max(roi_top, min(roi_bottom, my))
        
        # FPS and timing
        now = time.perf_counter()
        dt = now - last_time
        last_time = now
        if dt > 0:
            fps = fps_alpha * (1.0 / dt) + (1 - fps_alpha) * fps
        
        cap_ms = (t_cap - t0) * 1000
        proc_ms = (t_proc - t_cap) * 1000
        puck_speed = (pvx*pvx + pvy*pvy)**0.5 * 0.001
        
        # Update shared state for GUI
        state.update_tracking(px, py, puck_speed, puck_det, mx, my, mallet_det, fps, cap_ms, proc_ms)
        
        # === VISUALIZATION ===
        vis = frame.copy()
        
        # Draw ROI rectangle (blue)
        if state.show_roi:
            cv2.rectangle(vis, (current_roi['left'], current_roi['top']),
                         (w - current_roi['right'], h - current_roi['bottom']),
                         (255, 0, 0), 2)
        
        # Draw table bounds (orange)
        cv2.rectangle(vis, (table_bounds['left'], table_bounds['top']),
                     (w - table_bounds['right'], h - table_bounds['bottom']),
                     (0, 165, 255), 1)
        
        # Draw trajectory
        if state.show_trajectory and puck_det and (pvx*pvx + pvy*pvy) > 100:
            traj = predict_trajectory(px, py, pvx, pvy, w, h, table_bounds)
            if len(traj) > 1:
                pts = np.array(traj, dtype=np.int32)
                cv2.polylines(vis, [pts], False, (255, 0, 255), 2)
                cv2.circle(vis, traj[-1], 6, (255, 0, 255), -1)
        
        # Draw puck
        if puck_det:
            cv2.circle(vis, (int(px), int(py)), int(pr) if puck else 10, (0, 255, 0), 3)
            cv2.putText(vis, f"PUCK {puck_speed:.2f}m/s", (int(px)+15, int(py)-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw mallet
        if mallet_det:
            cv2.circle(vis, (int(mx), int(my)), int(mr), (0, 165, 255), 3)
            cv2.putText(vis, "MALLET", (int(mx)+15, int(my)-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        
        # Info overlay
        info = f"FPS: {fps:.0f} | Cap: {cap_ms:.1f}ms | Proc: {proc_ms:.1f}ms"
        if use_opencl:
            info += " [OpenCL]"
        cv2.putText(vis, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, f"Puck: ({int(px)}, {int(py)})", (10, 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(vis, f"Mallet: ({int(mx)}, {int(my)})", (10, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        
        # Show main tracker
        cv2.imshow("Tracker", vis)
        
        # Show masks (scaled back to full resolution for display)
        if state.show_mask:
            # Resize masks to full resolution
            puck_mask_full = cv2.resize(puck_mask, (w, h), interpolation=cv2.INTER_NEAREST)
            mallet_mask_full = cv2.resize(mallet_mask, (w, h), interpolation=cv2.INTER_NEAREST)
            
            # Create color-coded combined mask (Green=Puck, Red/Orange=Mallet)
            mask_vis = np.zeros((h, w, 3), dtype=np.uint8)
            mask_vis[:, :, 1] = puck_mask_full      # Green channel = puck
            mask_vis[:, :, 2] = mallet_mask_full    # Red channel = mallet
            mask_vis[:, :, 0] = mallet_mask_full // 2  # Some blue for orange tint
            
            # Draw ROI on mask view
            cv2.rectangle(mask_vis, (current_roi['left'], current_roi['top']),
                         (w - current_roi['right'], h - current_roi['bottom']),
                         (255, 255, 255), 1)
            
            cv2.putText(mask_vis, "GREEN=Puck  ORANGE=Mallet", (10, 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            cv2.imshow("Mask (ROI)", mask_vis)
        
        # Handle keys
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('s'):
            cv2.imwrite(f"screenshot_{int(time.time())}.png", vis)
            print("Screenshot saved!")
        
        frame_count += 1
        if frame_count % 60 == 0:
            print(f"\rFPS: {fps:.0f} | Cap: {cap_ms:.1f}ms | Proc: {proc_ms:.1f}ms | "
                  f"Puck: {'YES' if puck_det else 'NO'} | Mallet: {'YES' if mallet_det else 'NO'}   ", end='')
    
    print("\n\nCleaning up...")
    cap.release()
    cv2.destroyAllWindows()

# -------------------- MAIN --------------------
def main():
    print("\nStarting tracker with GUI control panel...")
    print("Close the control window or press Q in OpenCV window to quit.\n")
    
    state = TrackerState()
    stop_event = Event()
    
    # Start tracking in background thread
    tracker = Thread(target=tracking_thread, args=(state, stop_event), daemon=True)
    tracker.start()
    
    # Run GUI in main thread (Tkinter requirement)
    gui = ControlGUI(state)
    gui.run()
    
    # Cleanup
    stop_event.set()
    tracker.join(timeout=2)
    cv2.destroyAllWindows()
    print("Done.")

if __name__ == "__main__":
    main()
