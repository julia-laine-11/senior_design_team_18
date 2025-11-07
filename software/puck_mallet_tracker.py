# Advanced Hockey Puck & Mallet Tracker with Robot Control Framework
# Features: Green puck detection, Orange mallet detection, trajectory with collision prediction
# Requirements: pip install opencv-python numpy

import numpy as np
import time
import cv2
import tkinter as tk
from tkinter import ttk, messagebox
from threading import Thread, Lock
from collections import deque

# -------------------- CONFIGURATION --------------------
CAM_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
TARGET_FPS = 90

# Performance optimization
SKIP_LINE_SUPPRESSION = True
MORPH_KERNEL_SIZE = 3
VISUAL_UPDATE_SKIP = 0
GUI_UPDATE_MS = 100

# Fixed colors for game objects
PUCK_HSV_LOW = np.array([40, 50, 50])    # Green puck
PUCK_HSV_HIGH = np.array([80, 255, 255])

MALLET_HSV_LOW = np.array([5, 100, 100])   # Orange mallet
MALLET_HSV_HIGH = np.array([25, 255, 255])

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

# Robot control parameters (for future implementation)
GOAL_DEFENSE_ZONE_Y = 50  # Y-coordinate of goal defense line (pixels from top)
MALLET_STOW_POSITION = (320, 50)  # Default stow position in front of goal (x, y)
INTERCEPTION_LEAD_TIME = 0.5  # Seconds ahead to position mallet for interception

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
        
        # Robot control state (for future implementation)
        self.robot_target_x = MALLET_STOW_POSITION[0]
        self.robot_target_y = MALLET_STOW_POSITION[1]
        self.robot_action = "STOW"  # STOW, INTERCEPT, STRIKE, DEFEND
        
        self.fps = 0.0
    
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
        """Update robot control target (for future implementation)"""
        with self.lock:
            self.robot_target_x = target_x
            self.robot_target_y = target_y
            self.robot_action = action
    
    def update_fps(self, fps):
        with self.lock:
            self.fps = fps

# -------------------- HELPER FUNCTIONS --------------------
def make_roi_mask(h, w, margins):
    """Create ROI mask"""
    mask = np.zeros((h, w), dtype=np.uint8)
    t, b, l, r = margins["top"], margins["bottom"], margins["left"], margins["right"]
    cv2.rectangle(mask, (l, t), (w - r, h - b), 255, thickness=-1)
    return mask

def find_best_circular_candidate(contours, min_r, max_r, kf_pred_xy=None):
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

def check_puck_mallet_collision(puck_x, puck_y, puck_r, mallet_x, mallet_y, mallet_r):
    """
    Check if puck and mallet are colliding
    Returns True if collision detected
    """
    distance = np.hypot(puck_x - mallet_x, puck_y - mallet_y)
    return distance < (puck_r + mallet_r)

def predict_trajectory_with_mallet(puck_x, puck_y, vx_px, vy_px, frame_w, frame_h, 
                                   bounds, mallet_pos=None, mallet_radius=25,
                                   max_time=2.0, dt=0.016, damping=0.95):
    """
    Predict puck trajectory with wall bounces AND mallet collision detection
    
    Args:
        puck_x, puck_y: Current puck position
        vx_px, vy_px: Current velocity (pixels/second)
        frame_w, frame_h: Frame dimensions
        bounds: Table boundaries dict
        mallet_pos: (x, y) tuple of mallet position, or None
        mallet_radius: Mallet radius for collision detection
        max_time: Prediction time (seconds)
        dt: Time step
        damping: Energy retention after bounce
    
    Returns:
        List of (x, y) trajectory points, collision_detected (bool)
    """
    left = bounds['left']
    right = frame_w - bounds['right']
    top = bounds['top']
    bottom = frame_h - bounds['bottom']
    
    points = [(puck_x, puck_y)]
    px, py = puck_x, puck_y
    vx, vy = vx_px, vy_px
    
    time_elapsed = 0
    max_bounces = 20
    bounces = 0
    collision_detected = False
    puck_radius = 10  # Approximate puck radius
    
    while time_elapsed < max_time and bounces < max_bounces:
        px += vx * dt
        py += vy * dt
        
        # Check mallet collision
        if mallet_pos is not None and not collision_detected:
            mx, my = mallet_pos
            distance = np.hypot(px - mx, py - my)
            if distance < (puck_radius + mallet_radius):
                collision_detected = True
                # Stop trajectory prediction after mallet collision
                # (actual deflection would require mallet velocity)
                break
        
        # Wall bounces
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
    
    return points, collision_detected

# -------------------- ROBOT CONTROL LOGIC (STUBS) --------------------
def calculate_robot_action(state):
    """
    Calculate what action the robot should take based on puck and mallet positions
    
    This is a framework for future robot control implementation.
    Returns: (target_x, target_y, action_string)
    
    Future implementation will:
    1. Determine if puck is heading toward goal
    2. Calculate optimal interception point
    3. Generate motor commands for mallet positioning
    4. Execute strike or defensive maneuvers
    """
    
    # Get current state
    puck_detected = state.puck_detected
    mallet_detected = state.mallet_detected
    
    if not puck_detected:
        # No puck visible - return to stow position
        return MALLET_STOW_POSITION[0], MALLET_STOW_POSITION[1], "STOW"
    
    puck_x = state.puck_x
    puck_y = state.puck_y
    puck_vx = state.puck_vx_ms / state.meters_per_pixel  # Convert to px/s
    puck_vy = state.puck_vy_ms / state.meters_per_pixel
    
    # TODO: Implement trajectory prediction to goal
    # - Check if puck trajectory intersects goal zone
    # - Calculate time to reach goal
    # - Determine if interception is needed
    
    # TODO: Calculate optimal interception point
    # - Factor in robot mallet movement speed
    # - Account for lead time
    # - Avoid table boundaries
    
    # TODO: Determine strike opportunity
    # - Check if puck is close to robot mallet
    # - Evaluate strike angle toward opponent goal
    # - Calculate strike timing
    
    # Placeholder logic for demonstration
    if puck_y < GOAL_DEFENSE_ZONE_Y and puck_vy < 0:
        # Puck moving toward goal - DEFEND
        target_x = puck_x  # Move to block
        target_y = GOAL_DEFENSE_ZONE_Y
        action = "DEFEND"
    elif abs(puck_y - MALLET_STOW_POSITION[1]) < 100:
        # Puck near striking zone - evaluate strike
        target_x = puck_x
        target_y = puck_y
        action = "STRIKE"
    else:
        # Default: stow position
        target_x, target_y = MALLET_STOW_POSITION
        action = "STOW"
    
    return target_x, target_y, action

def send_robot_command(target_x, target_y, action):
    """
    Send movement command to robot hardware
    
    Future implementation will:
    1. Convert pixel coordinates to motor coordinates
    2. Calculate motor velocities and accelerations
    3. Send serial/network commands to robot controller
    4. Handle safety limits and collision avoidance
    
    Args:
        target_x, target_y: Target position in pixels
        action: Action string ("STOW", "INTERCEPT", "STRIKE", "DEFEND")
    """
    # TODO: Implement hardware interface
    # Example structure:
    # - Convert pixels to robot coordinate system
    # - Apply coordinate transforms
    # - Send to motor controller via serial/USB/network
    # - Monitor feedback for position confirmation
    
    pass  # Placeholder for future implementation

# -------------------- TRACKING THREAD --------------------
def tracking_thread(state, stop_event):
    """Main tracking loop with puck and mallet detection"""
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
    
    # Separate Kalman filters for puck and mallet
    puck_kf = ConstantVelocityKF(init_xy=(w / 2, h / 2))
    mallet_kf = ConstantVelocityKF(init_xy=(w / 2, h / 4))
    
    fps_history = deque(maxlen=30)
    last_time = time.time()
    frame_counter = 0
    
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            break
        
        # Get parameters
        margins = state.get_roi_margins()
        table_bounds = state.get_table_bounds()
        meters_per_pixel = state.get_meters_per_pixel()
        
        # Create ROI mask
        roi_mask = make_roi_mask(h, w, margins)
        
        # Preprocessing
        blurred = cv2.GaussianBlur(frame, (3, 3), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # PUCK DETECTION (Green)
        puck_mask = cv2.inRange(hsv, PUCK_HSV_LOW, PUCK_HSV_HIGH)
        puck_mask = cv2.bitwise_and(puck_mask, roi_mask)
        
        kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
        puck_mask = cv2.morphologyEx(puck_mask, cv2.MORPH_OPEN, kernel)
        puck_mask = cv2.morphologyEx(puck_mask, cv2.MORPH_CLOSE, kernel)
        
        puck_contours, _ = cv2.findContours(puck_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Track puck
        puck_pred_x, puck_pred_y, puck_pred_vx, puck_pred_vy = puck_kf.predict()
        best_puck = find_best_circular_candidate(puck_contours, PUCK_MIN_RADIUS, PUCK_MAX_RADIUS, 
                                                  (puck_pred_x, puck_pred_y))
        
        puck_detected = False
        puck_radius = PUCK_MIN_RADIUS
        if best_puck is not None:
            px, py, puck_radius, _ = best_puck
            puck_x, puck_y, puck_vx_px, puck_vy_px = puck_kf.correct(px, py)
            puck_detected = True
        else:
            puck_x, puck_y = puck_pred_x, puck_pred_y
            puck_vx_px, puck_vy_px = puck_pred_vx, puck_pred_vy
        
        puck_vx_ms = puck_vx_px * meters_per_pixel
        puck_vy_ms = puck_vy_px * meters_per_pixel
        puck_speed_ms = np.hypot(puck_vx_ms, puck_vy_ms)
        
        # MALLET DETECTION (Orange)
        mallet_mask = cv2.inRange(hsv, MALLET_HSV_LOW, MALLET_HSV_HIGH)
        mallet_mask = cv2.bitwise_and(mallet_mask, roi_mask)
        
        mallet_mask = cv2.morphologyEx(mallet_mask, cv2.MORPH_OPEN, kernel)
        mallet_mask = cv2.morphologyEx(mallet_mask, cv2.MORPH_CLOSE, kernel)
        
        mallet_contours, _ = cv2.findContours(mallet_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Track mallet
        mallet_pred_x, mallet_pred_y, _, _ = mallet_kf.predict()
        best_mallet = find_best_circular_candidate(mallet_contours, MALLET_MIN_RADIUS, MALLET_MAX_RADIUS,
                                                    (mallet_pred_x, mallet_pred_y))
        
        mallet_detected = False
        mallet_radius = MALLET_MIN_RADIUS
        if best_mallet is not None:
            mx, my, mallet_radius, _ = best_mallet
            mallet_x, mallet_y, _, _ = mallet_kf.correct(mx, my)
            mallet_detected = True
        else:
            mallet_x, mallet_y = mallet_pred_x, mallet_pred_y
        
        # Calculate FPS
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time
        if dt > 0:
            fps_history.append(1.0 / dt)
        fps = np.mean(fps_history) if fps_history else 0
        
        # Update state
        state.update_puck(puck_x, puck_y, puck_vx_ms, puck_vy_ms, puck_speed_ms, puck_detected)
        state.update_mallet(mallet_x, mallet_y, mallet_detected)
        state.update_fps(fps)
        
        # Calculate robot action (framework for future implementation)
        target_x, target_y, action = calculate_robot_action(state)
        state.update_robot_command(target_x, target_y, action)
        
        # TODO: Send command to robot hardware
        # send_robot_command(target_x, target_y, action)
        
        # Console output
        print(f"PUCK: ({int(puck_x)},{int(puck_y)}) {puck_speed_ms:.3f}m/s | "
              f"MALLET: ({int(mallet_x)},{int(mallet_y)}) | "
              f"ROBOT: {action} -> ({int(target_x)},{int(target_y)})", 
              flush=True)
        
        # Visualization
        frame_counter += 1
        if frame_counter % (VISUAL_UPDATE_SKIP + 1) != 0:
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                stop_event.set()
                break
            continue
        
        vis = frame.copy()
        
        # Draw ROI
        cv2.rectangle(vis, (margins["left"], margins["top"]),
                     (w - margins["right"], h - margins["bottom"]),
                     (255, 0, 0), 2)
        
        # Draw table bounds
        cv2.rectangle(vis, (table_bounds["left"], table_bounds["top"]),
                     (w - table_bounds["right"], h - table_bounds["bottom"]),
                     (255, 165, 0), 2)
        
        # Draw goal defense line (for visualization)
        cv2.line(vis, (0, GOAL_DEFENSE_ZONE_Y), (w, GOAL_DEFENSE_ZONE_Y), (255, 0, 0), 2)
        cv2.putText(vis, "GOAL ZONE", (10, GOAL_DEFENSE_ZONE_Y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        
        # Check collision
        collision = False
        if puck_detected and mallet_detected:
            collision = check_puck_mallet_collision(puck_x, puck_y, puck_radius,
                                                    mallet_x, mallet_y, mallet_radius)
        
        # Draw trajectory with mallet collision prediction
        if state.show_trajectory and puck_detected and puck_speed_ms > 0.1:
            mallet_pos = (mallet_x, mallet_y) if mallet_detected else None
            traj_points, will_collide = predict_trajectory_with_mallet(
                puck_x, puck_y, puck_vx_px, puck_vy_px, w, h, table_bounds,
                mallet_pos=mallet_pos, mallet_radius=mallet_radius,
                max_time=TRAJECTORY_PREDICTION_TIME, damping=TRAJECTORY_DAMPING
            )
            
            # Color trajectory based on collision prediction
            traj_color = (0, 255, 255) if will_collide else (255, 0, 255)  # Yellow if collision, Magenta otherwise
            
            if len(traj_points) > 1:
                for i in range(len(traj_points) - 1):
                    pt1 = (int(traj_points[i][0]), int(traj_points[i][1]))
                    pt2 = (int(traj_points[i+1][0]), int(traj_points[i+1][1]))
                    cv2.line(vis, pt1, pt2, traj_color, 2)
                
                if len(traj_points) > 0:
                    end_pt = (int(traj_points[-1][0]), int(traj_points[-1][1]))
                    cv2.circle(vis, end_pt, 8, traj_color, -1)
                    
                    if will_collide:
                        cv2.putText(vis, "COLLISION!", (end_pt[0] + 10, end_pt[1]),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        
        # Draw puck (green circle)
        if puck_detected:
            cv2.circle(vis, (int(puck_x), int(puck_y)), max(5, int(puck_radius)), (0, 255, 0), 3)
            cv2.putText(vis, "PUCK", (int(puck_x) + 10, int(puck_y) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw velocity vector
            if state.show_velocity_vector and puck_speed_ms > 0.01:
                vector_scale = 50
                end_x = int(puck_x + puck_vx_ms * vector_scale)
                end_y = int(puck_y + puck_vy_ms * vector_scale)
                cv2.arrowedLine(vis, (int(puck_x), int(puck_y)), (end_x, end_y),
                               (0, 255, 0), 2, tipLength=0.3)
        
        # Draw mallet (orange circle)
        if state.show_mallet and mallet_detected:
            cv2.circle(vis, (int(mallet_x), int(mallet_y)), max(5, int(mallet_radius)), (0, 165, 255), 3)
            cv2.putText(vis, "MALLET", (int(mallet_x) + 10, int(mallet_y) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
            
            # Draw collision indicator
            if collision:
                cv2.circle(vis, (int(puck_x), int(puck_y)), int(puck_radius + mallet_radius), 
                          (0, 0, 255), 3)
                cv2.putText(vis, "CONTACT!", (int(puck_x), int(puck_y) - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Draw robot target (for visualization)
        cv2.circle(vis, (int(target_x), int(target_y)), 10, (255, 255, 0), 2)
        cv2.line(vis, (int(mallet_x), int(mallet_y)), (int(target_x), int(target_y)),
                (255, 255, 0), 1)
        
        # Draw info text
        info_y = 30
        cv2.putText(vis, f"FPS: {fps:.1f}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        info_y += 25
        cv2.putText(vis, f"Puck: ({int(puck_x)},{int(puck_y)}) {puck_speed_ms:.2f}m/s", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        info_y += 20
        cv2.putText(vis, f"Mallet: ({int(mallet_x)},{int(mallet_y)})", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        info_y += 20
        cv2.putText(vis, f"Robot: {action}", (10, info_y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # Show windows
        cv2.imshow("Puck & Mallet Tracker", vis)
        if state.show_mask:
            # Show combined mask
            combined_mask = cv2.bitwise_or(puck_mask, mallet_mask)
            # Color code: green = puck, orange = mallet
            mask_vis = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
            mask_vis[:, :, 1] = np.maximum(mask_vis[:, :, 1], puck_mask)  # Green channel
            mask_vis[:, :, 2] = np.maximum(mask_vis[:, :, 2], mallet_mask)  # Red channel
            cv2.imshow("Mask (Green=Puck, Orange=Mallet)", mask_vis)
        
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
        self.root.title("Puck & Mallet Tracker with Robot Control")
        self.root.geometry("600x750")
        
        self._create_widgets()
        self._start_update_loop()
    
    def _create_widgets(self):
        # Info
        info_frame = ttk.LabelFrame(self.root, text="System Info", padding=10)
        info_frame.pack(fill="x", padx=10, pady=5)
        ttk.Label(info_frame, text="[GREEN] Puck | [ORANGE] Mallet | [YELLOW] Robot Target",
                 font=("Arial", 10, "bold")).pack()
        
        # ROI
        roi_frame = ttk.LabelFrame(self.root, text="ROI Margins", padding=10)
        roi_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(roi_frame, text="Top:").grid(row=0, column=0)
        self.roi_top = ttk.Scale(roi_frame, from_=0, to=FRAME_HEIGHT//2, orient="horizontal")
        self.roi_top.set(DEFAULT_ROI_MARGINS["top"])
        self.roi_top.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(roi_frame, text="Bottom:").grid(row=1, column=0)
        self.roi_bottom = ttk.Scale(roi_frame, from_=0, to=FRAME_HEIGHT//2, orient="horizontal")
        self.roi_bottom.set(DEFAULT_ROI_MARGINS["bottom"])
        self.roi_bottom.grid(row=1, column=1, sticky="ew")
        
        ttk.Label(roi_frame, text="Left:").grid(row=2, column=0)
        self.roi_left = ttk.Scale(roi_frame, from_=0, to=FRAME_WIDTH//2, orient="horizontal")
        self.roi_left.set(DEFAULT_ROI_MARGINS["left"])
        self.roi_left.grid(row=2, column=1, sticky="ew")
        
        ttk.Label(roi_frame, text="Right:").grid(row=3, column=0)
        self.roi_right = ttk.Scale(roi_frame, from_=0, to=FRAME_WIDTH//2, orient="horizontal")
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
        self.table_top = ttk.Scale(table_frame, from_=0, to=FRAME_HEIGHT//2, orient="horizontal")
        self.table_top.set(DEFAULT_TABLE_BOUNDS["top"])
        self.table_top.grid(row=0, column=1, sticky="ew")
        
        ttk.Label(table_frame, text="Bottom:").grid(row=1, column=0)
        self.table_bottom = ttk.Scale(table_frame, from_=0, to=FRAME_HEIGHT//2, orient="horizontal")
        self.table_bottom.set(DEFAULT_TABLE_BOUNDS["bottom"])
        self.table_bottom.grid(row=1, column=1, sticky="ew")
        
        ttk.Label(table_frame, text="Left:").grid(row=2, column=0)
        self.table_left = ttk.Scale(table_frame, from_=0, to=FRAME_WIDTH//2, orient="horizontal")
        self.table_left.set(DEFAULT_TABLE_BOUNDS["left"])
        self.table_left.grid(row=2, column=1, sticky="ew")
        
        ttk.Label(table_frame, text="Right:").grid(row=3, column=0)
        self.table_right = ttk.Scale(table_frame, from_=0, to=FRAME_WIDTH//2, orient="horizontal")
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
        labels = ["Puck:", "Mallet:", "Robot Action:", "FPS:"]
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
            
            self.root.after(GUI_UPDATE_MS, update)
        update()
    
    def run(self):
        self.root.mainloop()

# -------------------- MAIN --------------------
def main():
    print("=" * 70)
    print("Hockey Puck & Mallet Tracker with Robot Control Framework")
    print("=" * 70)
    print("\nColor Configuration:")
    print("  GREEN PUCK: HSV 40-80")
    print("  ORANGE MALLET: HSV 5-25")
    print("\nRobot Control:")
    print("  - Framework included for future hardware integration")
    print("  - Trajectory prediction includes mallet collision detection")
    print("  - Action planning: STOW, DEFEND, STRIKE, INTERCEPT")
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
