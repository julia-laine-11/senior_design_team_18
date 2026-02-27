# Optimized Hockey Puck & Mallet Tracker with GPU Acceleration
# Features: UMat-based GPU acceleration (OpenCL/CUDA), pre-allocated buffers, performance optimizations
# Requirements: pip install opencv-python numpy
# For GPU: Install OpenCV with CUDA support or ensure OpenCL drivers are available

import os
# Disable OpenCV GUI completely to avoid Qt/Tkinter conflicts
os.environ['OPENCV_VIDEOIO_PRIORITY_GSTREAMER'] = '0'

import numpy as np
import time
import cv2

# Disable OpenCV's GUI module threading
cv2.setNumThreads(4)

import tkinter as tk
from tkinter import ttk, messagebox
from threading import Thread, Lock
from collections import deque

# -------------------- CONFIGURATION --------------------
CAM_INDEX = 2 # video0 works at ~30 FPS (video2 has hardware issues)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 120  # Higher target for optimized version

# Performance optimization
SKIP_LINE_SUPPRESSION = True
MORPH_KERNEL_SIZE = 3
VISUAL_UPDATE_SKIP = 0
GUI_UPDATE_MS = 100

# Resolution scaling (1.0 = full resolution, 0.5 = half resolution for processing)
PROCESSING_SCALE = .5  # Set to 0.5 or 0.75 for higher FPS at cost of accuracy

# Use GPU acceleration (UMat) when available
# NOTE: Disabled because OpenCL conflicts with camera capture on this system
USE_GPU_ACCELERATION = False

# Fixed colors for game objects
PUCK_HSV_LOW = np.array([40, 50, 50], dtype=np.uint8)    # Green puck
PUCK_HSV_HIGH = np.array([80, 255, 255], dtype=np.uint8)

MALLET_HSV_LOW = np.array([5, 100, 100], dtype=np.uint8)   # Orange mallet
MALLET_HSV_HIGH = np.array([25, 255, 255], dtype=np.uint8)

# Tracking parameters
DEFAULT_METERS_PER_PIXEL = 0.001
DEFAULT_ROI_MARGINS = {"top": 50, "bottom": 50, "left": 50, "right": 50}
DEFAULT_TABLE_BOUNDS = {"top": 100, "bottom": 100, "left": 100, "right": 100}

# Puck detection
PUCK_MIN_RADIUS = 5
PUCK_MAX_RADIUS = 30

# Mallet detection (typically larger than puck)
MALLET_MIN_RADIUS = 15
MALLET_MAX_RADIUS = 60

# Trajectory prediction
TRAJECTORY_PREDICTION_TIME = 2.0
TRAJECTORY_DAMPING = 0.95

# Robot control parameters
GOAL_DEFENSE_ZONE_Y = 50
MALLET_STOW_POSITION = (320, 50)
INTERCEPTION_LEAD_TIME = 0.5

# Kalman filter tuning
KF_PROCESS_NOISE_POS = 0.05
KF_PROCESS_NOISE_VEL = 4.0
KF_MEAS_NOISE = 2.0

# -------------------- GPU UTILITIES --------------------
class GPUAccelerator:
    """Manages GPU acceleration via OpenCV UMat"""
    
    def __init__(self):
        self.use_gpu = USE_GPU_ACCELERATION
        self.gpu_available = False
        self.backend_name = "CPU"
        
        # Check GPU availability
        if self.use_gpu:
            try:
                # Try to enable OpenCL
                cv2.ocl.setUseOpenCL(True)
                if cv2.ocl.useOpenCL() and cv2.ocl.haveOpenCL():
                    self.gpu_available = True
                    self.backend_name = "OpenCL"
                    print(f"[GPU] OpenCL acceleration enabled")
                else:
                    # Check for CUDA
                    if hasattr(cv2, 'cuda') and cv2.cuda.getCudaEnabledDeviceCount() > 0:
                        self.gpu_available = True
                        self.backend_name = "CUDA"
                        print(f"[GPU] CUDA acceleration enabled with {cv2.cuda.getCudaEnabledDeviceCount()} device(s)")
                    else:
                        print("[GPU] No GPU acceleration available, using optimized CPU path")
            except Exception as e:
                print(f"[GPU] Error checking GPU: {e}, using CPU")
        else:
            print("[GPU] GPU acceleration disabled by config")
    
    def to_gpu(self, mat):
        """Upload Mat to GPU (UMat) if GPU is available"""
        if self.gpu_available and self.use_gpu:
            return cv2.UMat(mat)
        return mat
    
    def to_cpu(self, umat):
        """Download UMat to CPU Mat"""
        if isinstance(umat, cv2.UMat):
            return umat.get()
        return umat
    
    def is_umat(self, mat):
        """Check if matrix is on GPU"""
        return isinstance(mat, cv2.UMat)

# Global GPU accelerator
gpu = GPUAccelerator()

# -------------------- PRE-ALLOCATED BUFFERS --------------------
class ProcessingBuffers:
    """Pre-allocated buffers for reduced memory allocation overhead"""
    
    def __init__(self, height, width, scale=1.0):
        self.scale = scale
        self.proc_h = int(height * scale)
        self.proc_w = int(width * scale)
        self.full_h = height
        self.full_w = width
        
        # Pre-allocate CPU buffers
        self.blurred = np.empty((self.proc_h, self.proc_w, 3), dtype=np.uint8)
        self.hsv = np.empty((self.proc_h, self.proc_w, 3), dtype=np.uint8)
        self.puck_mask = np.empty((self.proc_h, self.proc_w), dtype=np.uint8)
        self.mallet_mask = np.empty((self.proc_h, self.proc_w), dtype=np.uint8)
        self.roi_mask = np.zeros((self.proc_h, self.proc_w), dtype=np.uint8)
        
        # Morphology kernel (pre-allocated)
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
        )
        
        # Upload kernel to GPU if available
        if gpu.gpu_available:
            self.morph_kernel_gpu = gpu.to_gpu(self.morph_kernel)
        
        print(f"[Buffers] Initialized for {self.proc_w}x{self.proc_h} (scale={scale})")
    
    def update_roi_mask(self, margins):
        """Update ROI mask with new margins (scaled)"""
        self.roi_mask.fill(0)
        t = int(margins["top"] * self.scale)
        b = int(margins["bottom"] * self.scale)
        l = int(margins["left"] * self.scale)
        r = int(margins["right"] * self.scale)
        cv2.rectangle(self.roi_mask, (l, t), 
                     (self.proc_w - r, self.proc_h - b), 255, thickness=-1)

# -------------------- KALMAN FILTER (OPTIMIZED) --------------------
class ConstantVelocityKF:
    """2D Kalman filter: state [x, y, vx, vy], measurement [x, y]"""
    
    __slots__ = ['kf', '_last_t', '_transition', '_measurement']
    
    def __init__(self, init_xy=None):
        self.kf = cv2.KalmanFilter(4, 2)
        
        # Pre-allocate transition matrix
        self._transition = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        self.kf.transitionMatrix = self._transition.copy()
        
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
        self._last_t = time.perf_counter()  # Use perf_counter for higher precision
        
        # Pre-allocate measurement array
        self._measurement = np.zeros((2, 1), dtype=np.float32)

    def _update_dt(self):
        now = time.perf_counter()
        dt = max(1e-4, now - self._last_t)
        self._last_t = now
        self.kf.transitionMatrix[0, 2] = dt
        self.kf.transitionMatrix[1, 3] = dt
        return dt

    def predict(self):
        self._update_dt()
        pred = self.kf.predict()
        return pred[0, 0], pred[1, 0], pred[2, 0], pred[3, 0]

    def correct(self, meas_x, meas_y):
        self._measurement[0, 0] = meas_x
        self._measurement[1, 0] = meas_y
        est = self.kf.correct(self._measurement)
        return est[0, 0], est[1, 0], est[2, 0], est[3, 0]

# -------------------- TRACKER STATE --------------------
class TrackerState:
    """Thread-safe state for tracker parameters"""
    
    __slots__ = ['lock', 'roi_margins', 'table_bounds', 'meters_per_pixel',
                 'calibration_distance_m', 'calibration_distance_px',
                 'show_mask', 'show_velocity_vector', 'show_trajectory', 'show_mallet',
                 'puck_x', 'puck_y', 'puck_vx_ms', 'puck_vy_ms', 'puck_speed_ms', 'puck_detected',
                 'mallet_x', 'mallet_y', 'mallet_detected',
                 'robot_target_x', 'robot_target_y', 'robot_action', 'fps', 'gpu_backend',
                 'frame_width', 'frame_height']
    
    def __init__(self):
        self.lock = Lock()
        
        # Actual frame dimensions (updated by tracker thread)
        self.frame_width = FRAME_WIDTH
        self.frame_height = FRAME_HEIGHT
        
        # ROI
        self.roi_margins = DEFAULT_ROI_MARGINS.copy()
        self.table_bounds = DEFAULT_TABLE_BOUNDS.copy()
        
        # Calibration
        self.meters_per_pixel = DEFAULT_METERS_PER_PIXEL
        self.calibration_distance_m = 0.5
        self.calibration_distance_px = 500
        
        # Display options
        self.show_mask = True
        self.show_velocity_vector = True
        self.show_trajectory = True
        self.show_mallet = True
        
        # Puck tracking results
        self.puck_x = 0.0
        self.puck_y = 0.0
        self.puck_vx_ms = 0.0
        self.puck_vy_ms = 0.0
        self.puck_speed_ms = 0.0
        self.puck_detected = False
        
        # Mallet tracking results
        self.mallet_x = 0.0
        self.mallet_y = 0.0
        self.mallet_detected = False
        
        # Robot control state
        self.robot_target_x = MALLET_STOW_POSITION[0]
        self.robot_target_y = MALLET_STOW_POSITION[1]
        self.robot_action = "STOW"
        
        self.fps = 0.0
        self.gpu_backend = gpu.backend_name
    
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
    
    def get_meters_per_pixel(self):
        with self.lock:
            return self.meters_per_pixel
    
    def update_calibration(self, distance_m, distance_px):
        with self.lock:
            self.calibration_distance_m = distance_m
            self.calibration_distance_px = distance_px
            if distance_px > 0:
                self.meters_per_pixel = distance_m / distance_px
    
    def update_puck(self, x, y, vx_ms, vy_ms, speed_ms, detected):
        with self.lock:
            self.puck_x = x
            self.puck_y = y
            self.puck_vx_ms = vx_ms
            self.puck_vy_ms = vy_ms
            self.puck_speed_ms = speed_ms
            self.puck_detected = detected
    
    def update_mallet(self, x, y, detected):
        with self.lock:
            self.mallet_x = x
            self.mallet_y = y
            self.mallet_detected = detected
    
    def update_robot_command(self, target_x, target_y, action):
        with self.lock:
            self.robot_target_x = target_x
            self.robot_target_y = target_y
            self.robot_action = action
    
    def update_fps(self, fps):
        with self.lock:
            self.fps = fps
    
    def set_frame_dimensions(self, width, height):
        with self.lock:
            self.frame_width = width
            self.frame_height = height
    
    def get_frame_dimensions(self):
        with self.lock:
            return self.frame_width, self.frame_height

# -------------------- OPTIMIZED DETECTION FUNCTIONS --------------------
def find_best_circular_candidate_fast(contours, min_r, max_r, kf_pred_xy=None):
    """Optimized circular candidate detection with early exit"""
    best = None
    best_score = -float('inf')
    
    # Pre-compute prediction values if available
    pred_x, pred_y = kf_pred_xy if kf_pred_xy else (None, None)
    
    for cnt in contours:
        # Quick area check first (cheaper than minEnclosingCircle)
        area = cv2.contourArea(cnt)
        if area < 5:
            continue
        
        (x, y), rad = cv2.minEnclosingCircle(cnt)
        
        # Radius check
        if rad < min_r or rad > max_r:
            continue
        
        # Circularity calculation
        per = cv2.arcLength(cnt, True)
        if per < 1.0:
            continue
        
        circularity = 4.0 * 3.14159 * area / (per * per)
        
        # Proximity penalty
        score = circularity
        if pred_x is not None:
            dx = x - pred_x
            dy = y - pred_y
            # Use squared distance to avoid sqrt
            d_sq = dx * dx + dy * dy
            score -= 0.000004 * d_sq  # Adjusted for squared distance
        
        if score > best_score:
            best_score = score
            best = (float(x), float(y), float(rad), float(score))
    
    return best

def predict_trajectory_fast(puck_x, puck_y, vx_px, vy_px, frame_w, frame_h, 
                            bounds, mallet_pos=None, mallet_radius=25,
                            max_time=2.0, dt=0.02, damping=0.95):
    """Optimized trajectory prediction with reduced iterations"""
    left = bounds['left']
    right = frame_w - bounds['right']
    top = bounds['top']
    bottom = frame_h - bounds['bottom']
    
    # Pre-allocate with estimated size
    max_points = int(max_time / dt) + 1
    points = [(puck_x, puck_y)]
    
    px, py = puck_x, puck_y
    vx, vy = vx_px, vy_px
    
    collision_detected = False
    puck_radius = 10
    combined_radius_sq = (puck_radius + mallet_radius) ** 2 if mallet_pos else 0
    mx, my = mallet_pos if mallet_pos else (0, 0)
    
    time_elapsed = 0
    bounces = 0
    max_bounces = 10  # Reduced for performance
    min_velocity_sq = 25  # 5^2
    
    while time_elapsed < max_time and bounces < max_bounces and len(points) < max_points:
        px += vx * dt
        py += vy * dt
        
        # Mallet collision check (squared distance)
        if mallet_pos and not collision_detected:
            dx = px - mx
            dy = py - my
            if dx * dx + dy * dy < combined_radius_sq:
                collision_detected = True
                break
        
        # Wall bounces
        bounced = False
        if px <= left:
            px = left
            vx = abs(vx) * damping
            bounced = True
        elif px >= right:
            px = right
            vx = -abs(vx) * damping
            bounced = True
        
        if py <= top:
            py = top
            vy = abs(vy) * damping
            bounced = True
        elif py >= bottom:
            py = bottom
            vy = -abs(vy) * damping
            bounced = True
        
        if bounced:
            bounces += 1
        
        points.append((int(px), int(py)))
        time_elapsed += dt
        
        # Early exit if velocity is too low
        if vx * vx + vy * vy < min_velocity_sq:
            break
    
    return points, collision_detected

# -------------------- ROBOT CONTROL LOGIC (STUBS) --------------------
def calculate_robot_action(state):
    """Calculate robot action based on puck and mallet positions"""
    if not state.puck_detected:
        return MALLET_STOW_POSITION[0], MALLET_STOW_POSITION[1], "STOW"
    
    puck_x = state.puck_x
    puck_y = state.puck_y
    puck_vy = state.puck_vy_ms / state.meters_per_pixel
    
    if puck_y < GOAL_DEFENSE_ZONE_Y and puck_vy < 0:
        return puck_x, GOAL_DEFENSE_ZONE_Y, "DEFEND"
    elif abs(puck_y - MALLET_STOW_POSITION[1]) < 100:
        return puck_x, puck_y, "STRIKE"
    
    return MALLET_STOW_POSITION[0], MALLET_STOW_POSITION[1], "STOW"

def send_robot_command(target_x, target_y, action):
    """Placeholder for hardware interface"""
    pass

# -------------------- OPTIMIZED TRACKING THREAD --------------------
def tracking_thread(state, stop_event):
    """Main tracking loop with GPU acceleration and optimizations"""
    
    # Try multiple methods to open camera at desired resolution
    cap = None
    
    # Method 1: V4L2 with MJPEG (fastest)
    print(f"[Camera] Attempting to open camera {CAM_INDEX} at {FRAME_WIDTH}x{FRAME_HEIGHT}...")
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if cap.isOpened():
        # Set MJPEG FIRST (before resolution) - this is critical
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    # Method 2: Try GStreamer pipeline for forced resolution
    if not cap or not cap.isOpened():
        gst_pipeline = (
            f"v4l2src device=/dev/video{CAM_INDEX} ! "
            f"image/jpeg,width={FRAME_WIDTH},height={FRAME_HEIGHT},framerate=30/1 ! "
            f"jpegdec ! videoconvert ! appsink"
        )
        print(f"[Camera] Trying GStreamer: {gst_pipeline}")
        cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
    
    # Method 3: Default backend
    if not cap or not cap.isOpened():
        print("[Camera] Falling back to default backend...")
        cap = cv2.VideoCapture(CAM_INDEX)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return
    
    # Try to disable auto exposure
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read from camera")
        return
    
    cam_h, cam_w = frame.shape[:2]
    cam_fps = cap.get(cv2.CAP_PROP_FPS)
    
    # Check if we need to resize (camera didn't accept our resolution)
    needs_resize = (cam_w != FRAME_WIDTH or cam_h != FRAME_HEIGHT)
    if needs_resize:
        print(f"[Camera] WARNING: Camera returned {cam_w}x{cam_h} @ {cam_fps} FPS")
        print(f"[Camera] Will resize to {FRAME_WIDTH}x{FRAME_HEIGHT} for display")
    else:
        print(f"[Camera] Opened at {cam_w}x{cam_h} @ {cam_fps} FPS")
    
    # Use configured dimensions for processing, not camera dimensions
    h, w = FRAME_HEIGHT, FRAME_WIDTH
    scale = PROCESSING_SCALE
    
    # Initialize buffers
    buffers = ProcessingBuffers(h, w, scale)
    
    # Kalman filters
    puck_kf = ConstantVelocityKF(init_xy=(w / 2, h / 2))
    mallet_kf = ConstantVelocityKF(init_xy=(w / 2, h / 4))
    
    # FPS tracking with exponential moving average
    fps_alpha = 0.1
    fps_ema = 0.0
    last_time = time.perf_counter()
    frame_counter = 0
    
    # Cache for HSV bounds (convert to GPU if available)
    if gpu.gpu_available:
        puck_low_gpu = gpu.to_gpu(PUCK_HSV_LOW.reshape(1, 1, 3))
        puck_high_gpu = gpu.to_gpu(PUCK_HSV_HIGH.reshape(1, 1, 3))
        mallet_low_gpu = gpu.to_gpu(MALLET_HSV_LOW.reshape(1, 1, 3))
        mallet_high_gpu = gpu.to_gpu(MALLET_HSV_HIGH.reshape(1, 1, 3))
    
    # Get initial margins
    margins = state.get_roi_margins()
    buffers.update_roi_mask(margins)
    last_margins = margins.copy()
    
    print(f"[Tracker] Started - Output: {w}x{h}, Processing: {buffers.proc_w}x{buffers.proc_h}, Backend: {gpu.backend_name}")
    
    # Update state with output frame dimensions
    state.set_frame_dimensions(w, h)
    
    # Performance tracking
    perf_capture_time = 0
    perf_process_time = 0
    perf_display_time = 0
    perf_count = 0
    
    while not stop_event.is_set():
        t_start = time.perf_counter()
        
        ret, frame = cap.read()
        if not ret:
            break
        
        t_capture = time.perf_counter()
        perf_capture_time += t_capture - t_start
        
        # Resize frame if camera returned different resolution
        if needs_resize:
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_LINEAR)
        
        # Get parameters (with change detection)
        margins = state.get_roi_margins()
        if margins != last_margins:
            buffers.update_roi_mask(margins)
            last_margins = margins.copy()
        
        table_bounds = state.get_table_bounds()
        meters_per_pixel = state.get_meters_per_pixel()
        
        # Scale frame if needed
        if scale != 1.0:
            proc_frame = cv2.resize(frame, (buffers.proc_w, buffers.proc_h), 
                                   interpolation=cv2.INTER_LINEAR)
        else:
            proc_frame = frame
        
        # GPU-accelerated or CPU processing
        if gpu.gpu_available:
            # Upload to GPU
            frame_gpu = gpu.to_gpu(proc_frame)
            
            # Gaussian blur
            blurred_gpu = cv2.GaussianBlur(frame_gpu, (3, 3), 0)
            
            # Convert to HSV
            hsv_gpu = cv2.cvtColor(blurred_gpu, cv2.COLOR_BGR2HSV)
            
            # Puck mask (green)
            puck_mask_gpu = cv2.inRange(hsv_gpu, PUCK_HSV_LOW, PUCK_HSV_HIGH)
            
            # Apply ROI
            roi_mask_gpu = gpu.to_gpu(buffers.roi_mask)
            puck_mask_gpu = cv2.bitwise_and(puck_mask_gpu, roi_mask_gpu)
            
            # Morphology
            puck_mask_gpu = cv2.morphologyEx(puck_mask_gpu, cv2.MORPH_OPEN, buffers.morph_kernel)
            puck_mask_gpu = cv2.morphologyEx(puck_mask_gpu, cv2.MORPH_CLOSE, buffers.morph_kernel)
            
            # Mallet mask (orange)
            mallet_mask_gpu = cv2.inRange(hsv_gpu, MALLET_HSV_LOW, MALLET_HSV_HIGH)
            mallet_mask_gpu = cv2.bitwise_and(mallet_mask_gpu, roi_mask_gpu)
            mallet_mask_gpu = cv2.morphologyEx(mallet_mask_gpu, cv2.MORPH_OPEN, buffers.morph_kernel)
            mallet_mask_gpu = cv2.morphologyEx(mallet_mask_gpu, cv2.MORPH_CLOSE, buffers.morph_kernel)
            
            # Download for contour finding (CPU-only in OpenCV)
            puck_mask = gpu.to_cpu(puck_mask_gpu)
            mallet_mask = gpu.to_cpu(mallet_mask_gpu)
        else:
            # Optimized CPU path
            cv2.GaussianBlur(proc_frame, (3, 3), 0, dst=buffers.blurred)
            cv2.cvtColor(buffers.blurred, cv2.COLOR_BGR2HSV, dst=buffers.hsv)
            
            # Puck mask
            cv2.inRange(buffers.hsv, PUCK_HSV_LOW, PUCK_HSV_HIGH, dst=buffers.puck_mask)
            cv2.bitwise_and(buffers.puck_mask, buffers.roi_mask, dst=buffers.puck_mask)
            cv2.morphologyEx(buffers.puck_mask, cv2.MORPH_OPEN, buffers.morph_kernel, dst=buffers.puck_mask)
            cv2.morphologyEx(buffers.puck_mask, cv2.MORPH_CLOSE, buffers.morph_kernel, dst=buffers.puck_mask)
            
            # Mallet mask
            cv2.inRange(buffers.hsv, MALLET_HSV_LOW, MALLET_HSV_HIGH, dst=buffers.mallet_mask)
            cv2.bitwise_and(buffers.mallet_mask, buffers.roi_mask, dst=buffers.mallet_mask)
            cv2.morphologyEx(buffers.mallet_mask, cv2.MORPH_OPEN, buffers.morph_kernel, dst=buffers.mallet_mask)
            cv2.morphologyEx(buffers.mallet_mask, cv2.MORPH_CLOSE, buffers.morph_kernel, dst=buffers.mallet_mask)
            
            puck_mask = buffers.puck_mask
            mallet_mask = buffers.mallet_mask
        
        # Find contours (CPU operation)
        puck_contours, _ = cv2.findContours(puck_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mallet_contours, _ = cv2.findContours(mallet_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Scale factors for coordinate conversion
        inv_scale = 1.0 / scale if scale != 1.0 else 1.0
        
        # Track puck
        puck_pred_x, puck_pred_y, puck_pred_vx, puck_pred_vy = puck_kf.predict()
        scaled_pred = (puck_pred_x * scale, puck_pred_y * scale) if scale != 1.0 else (puck_pred_x, puck_pred_y)
        
        best_puck = find_best_circular_candidate_fast(
            puck_contours, 
            PUCK_MIN_RADIUS * scale, 
            PUCK_MAX_RADIUS * scale,
            scaled_pred
        )
        
        puck_detected = False
        puck_radius = PUCK_MIN_RADIUS
        if best_puck is not None:
            px, py, puck_radius, _ = best_puck
            # Scale back to full resolution
            px *= inv_scale
            py *= inv_scale
            puck_radius *= inv_scale
            puck_x, puck_y, puck_vx_px, puck_vy_px = puck_kf.correct(px, py)
            puck_detected = True
        else:
            puck_x, puck_y = puck_pred_x, puck_pred_y
            puck_vx_px, puck_vy_px = puck_pred_vx, puck_pred_vy
        
        puck_vx_ms = puck_vx_px * meters_per_pixel
        puck_vy_ms = puck_vy_px * meters_per_pixel
        puck_speed_ms = (puck_vx_ms * puck_vx_ms + puck_vy_ms * puck_vy_ms) ** 0.5
        
        # Track mallet
        mallet_pred_x, mallet_pred_y, _, _ = mallet_kf.predict()
        scaled_mallet_pred = (mallet_pred_x * scale, mallet_pred_y * scale) if scale != 1.0 else (mallet_pred_x, mallet_pred_y)
        
        best_mallet = find_best_circular_candidate_fast(
            mallet_contours,
            MALLET_MIN_RADIUS * scale,
            MALLET_MAX_RADIUS * scale,
            scaled_mallet_pred
        )
        
        mallet_detected = False
        mallet_radius = MALLET_MIN_RADIUS
        if best_mallet is not None:
            mx, my, mallet_radius, _ = best_mallet
            mx *= inv_scale
            my *= inv_scale
            mallet_radius *= inv_scale
            mallet_x, mallet_y, _, _ = mallet_kf.correct(mx, my)
            mallet_detected = True
        else:
            mallet_x, mallet_y = mallet_pred_x, mallet_pred_y
        
        # FPS calculation (exponential moving average)
        current_time = time.perf_counter()
        dt = current_time - last_time
        last_time = current_time
        if dt > 0:
            instant_fps = 1.0 / dt
            fps_ema = fps_alpha * instant_fps + (1 - fps_alpha) * fps_ema
        fps = fps_ema
        
        # Update state
        state.update_puck(puck_x, puck_y, puck_vx_ms, puck_vy_ms, puck_speed_ms, puck_detected)
        state.update_mallet(mallet_x, mallet_y, mallet_detected)
        state.update_fps(fps)
        
        # Robot control
        target_x, target_y, action = calculate_robot_action(state)
        state.update_robot_command(target_x, target_y, action)
        
        t_process = time.perf_counter()
        perf_process_time += t_process - t_capture
        perf_count += 1
        
        # Reduced console output (every 30th frame) with performance breakdown
        if frame_counter % 30 == 0 and perf_count > 0:
            avg_cap = 1000 * perf_capture_time / perf_count
            avg_proc = 1000 * perf_process_time / perf_count
            print(f"\rFPS: {fps:.0f} | Cap: {avg_cap:.1f}ms | Proc: {avg_proc:.1f}ms | "
                  f"Puck: ({int(puck_x):3d},{int(puck_y):3d}) {puck_speed_ms:.2f}m/s   ", end='', flush=True)
            perf_capture_time = perf_process_time = perf_count = 0
        
        # No OpenCV GUI - just increment counter
        frame_counter += 1
    
    print()  # Newline after carriage return output
    cap.release()

# -------------------- GUI --------------------
class TrackerGUI:
    def __init__(self, state):
        self.state = state
        self.root = tk.Tk()
        self.root.title("Puck & Mallet Tracker (Optimized)")
        self.root.geometry("600x800")
        
        self._create_widgets()
        self._start_update_loop()
    
    def _create_widgets(self):
        # Get actual frame dimensions (use defaults initially, will update)
        fw, fh = self.state.get_frame_dimensions()
        
        # Info
        info_frame = ttk.LabelFrame(self.root, text="System Info", padding=10)
        info_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(info_frame, text=f"Backend: {gpu.backend_name} | Scale: {PROCESSING_SCALE}",
                 font=("Arial", 10, "bold")).pack()
        self.resolution_label = ttk.Label(info_frame, text=f"Resolution: {fw}x{fh}", font=("Arial", 9))
        self.resolution_label.pack()
        ttk.Label(info_frame, text="[GREEN] Puck | [ORANGE] Mallet | [YELLOW] Robot Target",
                 font=("Arial", 9)).pack()
        
        # ROI
        roi_frame = ttk.LabelFrame(self.root, text="ROI Margins", padding=10)
        roi_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(roi_frame, text="Top:").grid(row=0, column=0)
        self.roi_top = ttk.Scale(roi_frame, from_=0, to=fh//2, orient="horizontal")
        self.roi_top.set(DEFAULT_ROI_MARGINS["top"])
        self.roi_top.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(roi_frame, text="Bottom:").grid(row=1, column=0)
        self.roi_bottom = ttk.Scale(roi_frame, from_=0, to=fh//2, orient="horizontal")
        self.roi_bottom.set(DEFAULT_ROI_MARGINS["bottom"])
        self.roi_bottom.grid(row=1, column=1, sticky="ew")
        
        ttk.Label(roi_frame, text="Left:").grid(row=2, column=0)
        self.roi_left = ttk.Scale(roi_frame, from_=0, to=fw//2, orient="horizontal")
        self.roi_left.set(DEFAULT_ROI_MARGINS["left"])
        self.roi_left.grid(row=2, column=1, sticky="ew")
        
        ttk.Label(roi_frame, text="Right:").grid(row=3, column=0)
        self.roi_right = ttk.Scale(roi_frame, from_=0, to=fw//2, orient="horizontal")
        self.roi_right.set(DEFAULT_ROI_MARGINS["right"])
        self.roi_right.grid(row=3, column=1, sticky="ew")
        
        self.roi_top.configure(command=self._update_roi)
        self.roi_bottom.configure(command=self._update_roi)
        self.roi_left.configure(command=self._update_roi)
        self.roi_right.configure(command=self._update_roi)
        roi_frame.columnconfigure(1, weight=1)
        
        # Table Bounds
        table_frame = ttk.LabelFrame(self.root, text="Table Bounds", padding=10)
        table_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(table_frame, text="Top:").grid(row=0, column=0)
        self.table_top = ttk.Scale(table_frame, from_=0, to=fh//2, orient="horizontal")
        self.table_top.set(DEFAULT_TABLE_BOUNDS["top"])
        self.table_top.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(table_frame, text="Bottom:").grid(row=1, column=0)
        self.table_bottom = ttk.Scale(table_frame, from_=0, to=fh//2, orient="horizontal")
        self.table_bottom.set(DEFAULT_TABLE_BOUNDS["bottom"])
        self.table_bottom.grid(row=1, column=1, sticky="ew")
        
        ttk.Label(table_frame, text="Left:").grid(row=2, column=0)
        self.table_left = ttk.Scale(table_frame, from_=0, to=fw//2, orient="horizontal")
        self.table_left.set(DEFAULT_TABLE_BOUNDS["left"])
        self.table_left.grid(row=2, column=1, sticky="ew")
        
        ttk.Label(table_frame, text="Right:").grid(row=3, column=0)
        self.table_right = ttk.Scale(table_frame, from_=0, to=fw//2, orient="horizontal")
        self.table_right.set(DEFAULT_TABLE_BOUNDS["right"])
        self.table_right.grid(row=3, column=1, sticky="ew")
        
        self.table_top.configure(command=self._update_table)
        self.table_bottom.configure(command=self._update_table)
        self.table_left.configure(command=self._update_table)
        self.table_right.configure(command=self._update_table)
        table_frame.columnconfigure(1, weight=1)
        
        # Calibration
        cal_frame = ttk.LabelFrame(self.root, text="Calibration", padding=10)
        cal_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(cal_frame, text="Distance (m):").grid(row=0, column=0)
        self.cal_m = ttk.Entry(cal_frame, width=10)
        self.cal_m.insert(0, "0.5")
        self.cal_m.grid(row=0, column=1)
        
        ttk.Label(cal_frame, text="Pixels:").grid(row=1, column=0)
        self.cal_px = ttk.Entry(cal_frame, width=10)
        self.cal_px.insert(0, "500")
        self.cal_px.grid(row=1, column=1)
        
        ttk.Button(cal_frame, text="Calibrate", command=self._calibrate).grid(row=2, column=0, columnspan=2)
        self.cal_label = ttk.Label(cal_frame, text=f"{DEFAULT_METERS_PER_PIXEL:.6f} m/px")
        self.cal_label.grid(row=3, column=0, columnspan=2)
        
        # Display
        display_frame = ttk.LabelFrame(self.root, text="Display", padding=10)
        display_frame.pack(fill="x", padx=10, pady=5)
        
        self.show_mask_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Mask", variable=self.show_mask_var,
                       command=self._toggle_mask).pack(anchor="w")
        
        self.show_vector_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Velocity", variable=self.show_vector_var,
                       command=self._toggle_vector).pack(anchor="w")
        
        self.show_traj_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Trajectory", variable=self.show_traj_var,
                       command=self._toggle_traj).pack(anchor="w")
        
        self.show_mallet_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(display_frame, text="Show Mallet", variable=self.show_mallet_var,
                       command=self._toggle_mallet).pack(anchor="w")
        
        # Results
        results_frame = ttk.LabelFrame(self.root, text="Tracking Results", padding=10)
        results_frame.pack(fill="x", padx=10, pady=5)
        
        self.result_labels = {}
        labels = ["Puck:", "Mallet:", "Robot Action:", "FPS:", "Backend:"]
        for i, label in enumerate(labels):
            ttk.Label(results_frame, text=label).grid(row=i, column=0, sticky="w")
            val_label = ttk.Label(results_frame, text="--", font=("Courier", 9))
            val_label.grid(row=i, column=1, sticky="w")
            self.result_labels[label] = val_label
    
    def _update_roi(self, *args):
        margins = {
            "top": int(self.roi_top.get()),
            "bottom": int(self.roi_bottom.get()),
            "left": int(self.roi_left.get()),
            "right": int(self.roi_right.get())
        }
        self.state.set_roi_margins(margins)
    
    def _update_table(self, *args):
        bounds = {
            "top": int(self.table_top.get()),
            "bottom": int(self.table_bottom.get()),
            "left": int(self.table_left.get()),
            "right": int(self.table_right.get())
        }
        self.state.set_table_bounds(bounds)
    
    def _calibrate(self):
        try:
            dist_m = float(self.cal_m.get())
            dist_px = float(self.cal_px.get())
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
    
    def _toggle_traj(self):
        self.state.show_trajectory = self.show_traj_var.get()
    
    def _toggle_mallet(self):
        self.state.show_mallet = self.show_mallet_var.get()
    
    def _start_update_loop(self):
        self._last_dims = (0, 0)
        
        def update():
            puck_status = "DETECTED" if self.state.puck_detected else "SEARCHING"
            self.result_labels["Puck:"].config(
                text=f"({self.state.puck_x:.0f},{self.state.puck_y:.0f}) {self.state.puck_speed_ms:.2f}m/s [{puck_status}]"
            )
            
            mallet_status = "DETECTED" if self.state.mallet_detected else "SEARCHING"
            self.result_labels["Mallet:"].config(
                text=f"({self.state.mallet_x:.0f},{self.state.mallet_y:.0f}) [{mallet_status}]"
            )
            
            self.result_labels["Robot Action:"].config(
                text=f"{self.state.robot_action} -> ({self.state.robot_target_x:.0f},{self.state.robot_target_y:.0f})"
            )
            
            self.result_labels["FPS:"].config(
                text=f"{self.state.fps:.1f}"
            )
            
            self.result_labels["Backend:"].config(
                text=f"{gpu.backend_name}"
            )
            
            # Update resolution display and slider ranges if dimensions changed
            fw, fh = self.state.get_frame_dimensions()
            if (fw, fh) != self._last_dims and fw > 0 and fh > 0:
                self._last_dims = (fw, fh)
                self.resolution_label.config(text=f"Resolution: {fw}x{fh}")
                # Update slider ranges
                self.roi_top.configure(to=fh//2)
                self.roi_bottom.configure(to=fh//2)
                self.roi_left.configure(to=fw//2)
                self.roi_right.configure(to=fw//2)
                self.table_top.configure(to=fh//2)
                self.table_bottom.configure(to=fh//2)
                self.table_left.configure(to=fw//2)
                self.table_right.configure(to=fw//2)
            
            self.root.after(GUI_UPDATE_MS, update)
        update()
    
    def run(self):
        self.root.mainloop()

# -------------------- MAIN --------------------
def main():
    print("=" * 70)
    print("Hockey Puck & Mallet Tracker (OPTIMIZED)")
    print("=" * 70)
    print(f"\nGPU Acceleration: {gpu.backend_name}")
    print(f"Processing Scale: {PROCESSING_SCALE}")
    print("\nOptimizations:")
    print("  - UMat-based GPU acceleration (when available)")
    print("  - Pre-allocated buffers")
    print("  - Optimized contour detection")
    print("  - Reduced memory allocations")
    print("  - V4L2 camera backend (Linux)")
    print("  - MJPEG codec for higher FPS")
    print("\nColor Configuration:")
    print("  GREEN PUCK: HSV 40-80")
    print("  ORANGE MALLET: HSV 5-25")
    print("\nControls:")
    print("  - 'Q' or ESC: Quit")
    print("=" * 70)
    print()
    
    state = TrackerState()
    
    import threading
    stop_event = threading.Event()
    
    tracker = Thread(target=tracking_thread, args=(state, stop_event), daemon=True)
    tracker.start()
    
    gui = TrackerGUI(state)
    gui.run()
    
    stop_event.set()
    tracker.join(timeout=2)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
