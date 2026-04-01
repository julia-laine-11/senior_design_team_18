# Defensive Mode – Autonomous Air Hockey Defense
# Combines puck/mallet vision tracking with CoreXY motor control.
#
# G = toggle game mode (autonomous defense / manual control)
#
# Safety (always active):
#   - Mallet not detected → STOP all motors
#   - Red zone boundary enforcement (CoreXYController)
#
# Game mode ON:
#   - Puck not detected → move to home position
#   - Puck heading toward goal → intercept at predicted crossing
#   - Puck heading away → track puck laterally along goal line
#
# Game mode OFF:
#   - Manual numpad / WASD / arrow control
#
# Dependencies:
#   pip install opencv-python numpy pyserial

import numpy as np
import time
import cv2
import tkinter as tk
from tkinter import ttk
from threading import Thread, Lock, Event

from corexy_controller import CoreXYController, MAX_MOTOR_PCT

# ==================== CONFIGURATION ====================

# Camera
CAM_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Processing scale (0.5 = half-res for detection, faster)
PROCESSING_SCALE = 0.5

# HSV ranges (calibrated defaults)
PUCK_HSV_LOW = np.array([40, 50, 50], dtype=np.uint8)      # Green
PUCK_HSV_HIGH = np.array([80, 255, 255], dtype=np.uint8)
MALLET_HSV_LOW = np.array([5, 127, 100], dtype=np.uint8)   # Orange
MALLET_HSV_HIGH = np.array([25, 209, 255], dtype=np.uint8)

# Detection radii (full-res pixels)
PUCK_MIN_RADIUS = 5
PUCK_MAX_RADIUS = 30
MALLET_MIN_RADIUS = 15
MALLET_MAX_RADIUS = 60

# Table ROI (margins from frame edge) – used for detection masks
TABLE_ROI = {"top": 50, "bottom": 50, "left": 50, "right": 50}
TABLE_CORNER_RADIUS = 30   # semi-large rounded corners

# Motor UART
MOTOR_PORT = 'COM4'
MOTOR_BAUD = 115200
MOTOR_ENABLED = True

# Mallet box (margins from frame edge, motor boundary)
MALLET_BOX_MARGINS = {"top": 20, "bottom": 20, "left": 20, "right": 20}

# Red zone (margins inward from mallet box edge)
RED_ZONE_MARGINS = {"top": 30, "bottom": 30, "left": 30, "right": 30}

# Speed
DEFAULT_SPEED = 15

# Home position (pixels)
DEFAULT_HOME_X = 60
DEFAULT_HOME_Y = FRAME_HEIGHT // 2
HOME_THRESHOLD = 15

# Goal definition – vertical line on the LEFT side of the table.
# Puck attacks from right → left.  The mallet patrols this line.
DEFAULT_GOAL_X = 50               # X position of the defense line
DEFAULT_GOAL_Y = FRAME_HEIGHT // 2 # centre Y of the goal opening
DEFAULT_GOAL_LENGTH = 200          # vertical extent of the goal opening

# Trajectory / interception
TRAJECTORY_DAMPING = 0.95
TRAJECTORY_TIME = 2.0

# Tracking
LOST_THRESHOLD = 15
PUCK_LOST_TIMEOUT = 0.25

# Manual drive
KEY_RELEASE_TIMEOUT = 0.20

# GUI
GUI_UPDATE_MS = 100

# ==================== KEY MAPPINGS (manual mode) ====================

_NUM_DIRS = {
    ord('7'): (-1, -1), ord('8'): (0, -1), ord('9'): (1, -1),
    ord('4'): (-1,  0), ord('5'): (0,  0), ord('6'): (1,  0),
    ord('1'): (-1,  1), ord('2'): (0,  1), ord('3'): (1,  1),
}
_WASD_DIRS = {
    ord('w'): (0, -1), ord('a'): (-1, 0),
    ord('s'): (0,  1), ord('d'): (1,  0),
    ord(' '): (0,  0),
}
_ARROW_DIRS = {
    2490368: (0, -1), 2621440: (0, 1),
    2424832: (-1, 0), 2555904: (1, 0),
}
KEY_MAP = {**_NUM_DIRS, **_WASD_DIRS, **_ARROW_DIRS}

DIR_NAMES = {
    (-1, -1): "UP-LEFT",   (0, -1): "UP",      (1, -1): "UP-RIGHT",
    (-1,  0): "LEFT",      (0,  0): "STOP",    (1,  0): "RIGHT",
    (-1,  1): "DOWN-LEFT", (0,  1): "DOWN",    (1,  1): "DOWN-RIGHT",
}

# ==================== HELPER FUNCTIONS ====================

def make_rounded_rect_mask(h, w, x1, y1, x2, y2, radius):
    """Create a filled rounded-rectangle mask (uint8, 0/255)."""
    mask = np.zeros((h, w), dtype=np.uint8)
    r = max(0, min(radius, (x2 - x1) // 2, (y2 - y1) // 2))
    if r == 0:
        cv2.rectangle(mask, (x1, y1), (x2, y2), 255, -1)
        return mask
    # Overlapping rectangles to fill the body
    cv2.rectangle(mask, (x1 + r, y1), (x2 - r, y2), 255, -1)
    cv2.rectangle(mask, (x1, y1 + r), (x2, y2 - r), 255, -1)
    # Four corner circles
    cv2.circle(mask, (x1 + r, y1 + r), r, 255, -1)
    cv2.circle(mask, (x2 - r, y1 + r), r, 255, -1)
    cv2.circle(mask, (x1 + r, y2 - r), r, 255, -1)
    cv2.circle(mask, (x2 - r, y2 - r), r, 255, -1)
    return mask


def draw_rounded_rect(img, pt1, pt2, color, thickness, radius):
    """Draw a rounded-rectangle outline on an image."""
    x1, y1 = pt1
    x2, y2 = pt2
    r = max(0, min(radius, (x2 - x1) // 2, (y2 - y1) // 2))
    if r == 0:
        cv2.rectangle(img, pt1, pt2, color, thickness)
        return
    # Straight edges
    cv2.line(img, (x1 + r, y1), (x2 - r, y1), color, thickness)
    cv2.line(img, (x1 + r, y2), (x2 - r, y2), color, thickness)
    cv2.line(img, (x1, y1 + r), (x1, y2 - r), color, thickness)
    cv2.line(img, (x2, y1 + r), (x2, y2 - r), color, thickness)
    # Corner arcs
    cv2.ellipse(img, (x1 + r, y1 + r), (r, r), 180, 0, 90, color, thickness)
    cv2.ellipse(img, (x2 - r, y1 + r), (r, r), 270, 0, 90, color, thickness)
    cv2.ellipse(img, (x1 + r, y2 - r), (r, r), 90, 0, 90, color, thickness)
    cv2.ellipse(img, (x2 - r, y2 - r), (r, r), 0, 0, 90, color, thickness)


def find_circle(contours, min_r, max_r, pred_xy=None):
    """Find the most circular contour within radius range."""
    best, best_score = None, -1
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
            score -= 0.000004 * ((x - px) ** 2 + (y - py) ** 2)
        if score > best_score:
            best_score = score
            best = (x, y, r)
    return best


def predict_trajectory(x, y, vx, vy, w, h, bounds, max_t=2.0, dt=0.02):
    """Predict puck trajectory with wall bounces. Returns list of (x,y)."""
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
        if vx * vx + vy * vy < 25:
            break
    return points


def predict_intercept(px, py, pvx, pvy, line_x, goal_y, goal_len,
                      table_left, table_right, table_top, table_bottom,
                      max_t=2.0, dt=0.02, max_bounces=1):
    """Predict where the puck crosses a vertical line (x = line_x).

    Extends the puck's current direction with at most *max_bounces*
    wall reflections (default 1).  No damping or speed thresholds –
    purely geometric.

    Only considers crossings whose Y falls within
    [goal_y - goal_len/2, goal_y + goal_len/2].

    Returns (line_x, intercept_y) or None.
    """
    x, y = float(px), float(py)
    vx, vy = float(pvx), float(pvy)
    goal_top = goal_y - goal_len / 2.0
    goal_bot = goal_y + goal_len / 2.0
    bounces = 0

    for _ in range(int(max_t / dt)):
        prev_x, prev_y = x, y
        x += vx * dt
        y += vy * dt

        # --- Check line crossing BEFORE wall bounces ---
        crossed_left = (prev_x > line_x and x <= line_x)
        crossed_right = (prev_x < line_x and x >= line_x)

        if crossed_left or crossed_right:
            dx = x - prev_x
            if abs(dx) > 0.001:
                t_frac = (line_x - prev_x) / dx
                cross_y = prev_y + vy * dt * t_frac
            else:
                cross_y = y
            if goal_top <= cross_y <= goal_bot:
                return (float(line_x), cross_y)

        # --- Wall bounces (no damping – pure reflection) ---
        if y <= table_top:
            y, vy = table_top, abs(vy)
            bounces += 1
        elif y >= table_bottom:
            y, vy = table_bottom, -abs(vy)
            bounces += 1

        if x <= table_left:
            x, vx = table_left, abs(vx)
            bounces += 1
        elif x >= table_right:
            x, vx = table_right, -abs(vx)
            bounces += 1

        if bounces > max_bounces:
            break

    return None


# ==================== KALMAN FILTER ====================

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
        self.kf.processNoiseCov = np.diag(
            [0.05, 0.05, 4.0, 4.0]).astype(np.float32)
        self.kf.measurementNoiseCov = 2.0 * np.eye(2, dtype=np.float32)
        self.kf.statePost = np.array(
            [[x], [y], [0], [0]], dtype=np.float32)
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


# ==================== THREAD-SAFE STATE ====================

class GameState:
    """Thread-safe state shared between tracking thread and GUI."""

    def __init__(self):
        self.lock = Lock()

        # Table ROI
        self.table_roi = TABLE_ROI.copy()
        self.corner_radius = TABLE_CORNER_RADIUS

        # Mallet box & red zone
        self.box_margins = MALLET_BOX_MARGINS.copy()
        self.red_margins = RED_ZONE_MARGINS.copy()

        # HSV
        self.puck_hsv_low = list(PUCK_HSV_LOW)
        self.puck_hsv_high = list(PUCK_HSV_HIGH)
        self.mallet_hsv_low = list(MALLET_HSV_LOW)
        self.mallet_hsv_high = list(MALLET_HSV_HIGH)

        # Game / defense
        self.game_enabled = False
        self.speed = DEFAULT_SPEED
        self.home_x = DEFAULT_HOME_X
        self.home_y = DEFAULT_HOME_Y
        self.goal_x = DEFAULT_GOAL_X
        self.goal_y = DEFAULT_GOAL_Y
        self.goal_length = DEFAULT_GOAL_LENGTH

        # Display toggles
        self.show_mask = False
        self.show_roi = True
        self.show_trajectory = True

        # Tracking results (written by thread, read by GUI)
        self.puck_x = 0.0
        self.puck_y = 0.0
        self.puck_speed = 0.0
        self.puck_detected = False
        self.mallet_x = 0.0
        self.mallet_y = 0.0
        self.mallet_r = 0.0
        self.mallet_detected = False
        self.fps = 0.0
        self.cap_ms = 0.0
        self.proc_ms = 0.0
        self.in_red = False
        self.intercept_x = None
        self.intercept_y = None
        self.motor_info = "A:0% B:0%"
        self.defense_state = "IDLE"

    # ---- Getters / setters (lock-protected) ----

    def get_table_roi(self):
        with self.lock:
            return self.table_roi.copy(), self.corner_radius

    def set_table_roi_margin(self, key, val):
        with self.lock:
            self.table_roi[key] = int(float(val))

    def set_corner_radius(self, val):
        with self.lock:
            self.corner_radius = int(float(val))

    def get_box_margins(self):
        with self.lock:
            return self.box_margins.copy()

    def set_box_margin(self, key, val):
        with self.lock:
            self.box_margins[key] = int(float(val))

    def get_red_margins(self):
        with self.lock:
            return self.red_margins.copy()

    def set_red_margin(self, key, val):
        with self.lock:
            self.red_margins[key] = int(float(val))

    def get_puck_hsv(self):
        with self.lock:
            return (np.array(self.puck_hsv_low, dtype=np.uint8),
                    np.array(self.puck_hsv_high, dtype=np.uint8))

    def set_puck_hsv(self, low, high):
        with self.lock:
            self.puck_hsv_low = list(low)
            self.puck_hsv_high = list(high)

    def get_mallet_hsv(self):
        with self.lock:
            return (np.array(self.mallet_hsv_low, dtype=np.uint8),
                    np.array(self.mallet_hsv_high, dtype=np.uint8))

    def set_mallet_hsv(self, low, high):
        with self.lock:
            self.mallet_hsv_low = list(low)
            self.mallet_hsv_high = list(high)

    def get_speed(self):
        with self.lock:
            return self.speed

    def set_speed(self, val):
        with self.lock:
            self.speed = int(float(val))

    def get_goal(self):
        with self.lock:
            return self.goal_x, self.goal_y, self.goal_length

    def set_goal_param(self, param, val):
        with self.lock:
            setattr(self, f"goal_{param}", int(float(val)))

    def get_home(self):
        with self.lock:
            return self.home_x, self.home_y

    def set_home_x(self, val):
        with self.lock:
            self.home_x = int(float(val))

    def set_home_y(self, val):
        with self.lock:
            self.home_y = int(float(val))

    def toggle_game(self):
        with self.lock:
            self.game_enabled = not self.game_enabled
            return self.game_enabled

    def is_game_enabled(self):
        with self.lock:
            return self.game_enabled

    def update_tracking(self, puck_x, puck_y, puck_speed, puck_detected,
                        mallet_x, mallet_y, mallet_r, mallet_detected,
                        fps, cap_ms, proc_ms, in_red,
                        intercept_x, intercept_y,
                        motor_info, defense_state):
        with self.lock:
            self.puck_x = puck_x
            self.puck_y = puck_y
            self.puck_speed = puck_speed
            self.puck_detected = puck_detected
            self.mallet_x = mallet_x
            self.mallet_y = mallet_y
            self.mallet_r = mallet_r
            self.mallet_detected = mallet_detected
            self.fps = fps
            self.cap_ms = cap_ms
            self.proc_ms = proc_ms
            self.in_red = in_red
            self.intercept_x = intercept_x
            self.intercept_y = intercept_y
            self.motor_info = motor_info
            self.defense_state = defense_state


# ==================== TKINTER GUI ====================

class ControlGUI:
    """Tabbed control panel.  Runs in the main thread (Tkinter requirement)."""

    def __init__(self, state, frame_w=FRAME_WIDTH, frame_h=FRAME_HEIGHT):
        self.state = state
        self.fw = frame_w
        self.fh = frame_h
        self.root = tk.Tk()
        self.root.title("Defense Mode Controls")
        self.root.geometry("470x720")
        self._build()
        self._start_update()

    # ----------------------------------------------------------------
    # Widget creation
    # ----------------------------------------------------------------

    def _build(self):
        s = self.state
        nb = ttk.Notebook(self.root)
        nb.pack(fill="both", expand=True, padx=5, pady=5)

        # ===== TAB 1: GAME =====
        game_tab = ttk.Frame(nb)
        nb.add(game_tab, text="Game")

        gf = ttk.LabelFrame(game_tab, text="Game Mode", padding=8)
        gf.pack(fill="x", padx=8, pady=4)
        self.game_label = ttk.Label(
            gf, text="OFF  (press G in camera window)",
            font=("Courier", 11, "bold"))
        self.game_label.pack()
        self.defense_label = ttk.Label(
            gf, text="State: IDLE", font=("Courier", 9))
        self.defense_label.pack()

        # Max speed
        spd_f = ttk.LabelFrame(
            game_tab, text=f"Max Speed (cap {MAX_MOTOR_PCT}%)", padding=8)
        spd_f.pack(fill="x", padx=8, pady=4)
        self.speed_slider = ttk.Scale(
            spd_f, from_=1, to=MAX_MOTOR_PCT, orient="horizontal")
        self.speed_slider.set(DEFAULT_SPEED)
        self.speed_slider.pack(fill="x")
        self.speed_slider.configure(command=lambda v: s.set_speed(v))
        self.speed_val = ttk.Label(
            spd_f, text=f"{DEFAULT_SPEED}%", font=("Courier", 10))
        self.speed_val.pack()

        # Goal (vertical line on the left side)
        goal_f = ttk.LabelFrame(game_tab, text="Goal (Yellow, Vertical)", padding=5)
        goal_f.pack(fill="x", padx=8, pady=4)
        self.goal_sliders = {}
        for i, (label, param, default, mx) in enumerate([
            ("X", "x", DEFAULT_GOAL_X, self.fw),
            ("Y", "y", DEFAULT_GOAL_Y, self.fh),
            ("Length", "length", DEFAULT_GOAL_LENGTH, self.fh),
        ]):
            ttk.Label(goal_f, text=f"{label}:").grid(
                row=i, column=0, sticky="w")
            sl = ttk.Scale(goal_f, from_=0, to=mx,
                           orient="horizontal", length=140)
            sl.set(default)
            sl.grid(row=i, column=1, sticky="ew", padx=2)
            sl.configure(
                command=lambda v, p=param: s.set_goal_param(p, v))
            self.goal_sliders[param] = sl
        goal_f.columnconfigure(1, weight=1)

        # Home position
        home_f = ttk.LabelFrame(
            game_tab, text="Home Position (Green Cross)", padding=5)
        home_f.pack(fill="x", padx=8, pady=4)
        ttk.Label(home_f, text="X:").grid(row=0, column=0, sticky="w")
        self.home_x_sl = ttk.Scale(
            home_f, from_=0, to=self.fw, orient="horizontal", length=140)
        self.home_x_sl.set(DEFAULT_HOME_X)
        self.home_x_sl.grid(row=0, column=1, sticky="ew", padx=2)
        self.home_x_sl.configure(command=lambda v: s.set_home_x(v))
        ttk.Label(home_f, text="Y:").grid(row=1, column=0, sticky="w")
        self.home_y_sl = ttk.Scale(
            home_f, from_=0, to=self.fh, orient="horizontal", length=140)
        self.home_y_sl.set(DEFAULT_HOME_Y)
        self.home_y_sl.grid(row=1, column=1, sticky="ew", padx=2)
        self.home_y_sl.configure(command=lambda v: s.set_home_y(v))
        home_f.columnconfigure(1, weight=1)

        # ===== TAB 2: BOUNDS =====
        bounds_tab = ttk.Frame(nb)
        nb.add(bounds_tab, text="Bounds")

        # Table ROI
        roi_f = ttk.LabelFrame(
            bounds_tab, text="Table ROI (Orange, Rounded)", padding=5)
        roi_f.pack(fill="x", padx=8, pady=4)
        self.roi_sliders = {}
        for i, (name, default) in enumerate([
            ("top", TABLE_ROI["top"]), ("bottom", TABLE_ROI["bottom"]),
            ("left", TABLE_ROI["left"]), ("right", TABLE_ROI["right"]),
        ]):
            ttk.Label(roi_f, text=f"{name.title()}:").grid(
                row=i // 2, column=(i % 2) * 2, sticky="w")
            sl = ttk.Scale(roi_f, from_=0, to=300,
                           orient="horizontal", length=80)
            sl.set(default)
            sl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="ew", padx=2)
            sl.configure(
                command=lambda v, n=name: s.set_table_roi_margin(n, v))
            self.roi_sliders[name] = sl
        ttk.Label(roi_f, text="Radius:").grid(row=2, column=0, sticky="w")
        self.radius_sl = ttk.Scale(
            roi_f, from_=0, to=150, orient="horizontal", length=80)
        self.radius_sl.set(TABLE_CORNER_RADIUS)
        self.radius_sl.grid(row=2, column=1, sticky="ew", padx=2)
        self.radius_sl.configure(command=lambda v: s.set_corner_radius(v))
        roi_f.columnconfigure(1, weight=1)
        roi_f.columnconfigure(3, weight=1)

        # Mallet box
        box_f = ttk.LabelFrame(
            bounds_tab, text="Mallet Box (Cyan)", padding=5)
        box_f.pack(fill="x", padx=8, pady=4)
        self.box_sliders = {}
        for i, (name, default) in enumerate([
            ("top", MALLET_BOX_MARGINS["top"]),
            ("bottom", MALLET_BOX_MARGINS["bottom"]),
            ("left", MALLET_BOX_MARGINS["left"]),
            ("right", MALLET_BOX_MARGINS["right"]),
        ]):
            ttk.Label(box_f, text=f"{name.title()}:").grid(
                row=i // 2, column=(i % 2) * 2, sticky="w")
            sl = ttk.Scale(box_f, from_=0, to=300,
                           orient="horizontal", length=80)
            sl.set(default)
            sl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="ew", padx=2)
            sl.configure(
                command=lambda v, n=name: s.set_box_margin(n, v))
            self.box_sliders[name] = sl
        box_f.columnconfigure(1, weight=1)
        box_f.columnconfigure(3, weight=1)

        # Red zone
        red_f = ttk.LabelFrame(
            bounds_tab, text="Red Zone (Red)", padding=5)
        red_f.pack(fill="x", padx=8, pady=4)
        self.red_sliders = {}
        for i, (name, default) in enumerate([
            ("top", RED_ZONE_MARGINS["top"]),
            ("bottom", RED_ZONE_MARGINS["bottom"]),
            ("left", RED_ZONE_MARGINS["left"]),
            ("right", RED_ZONE_MARGINS["right"]),
        ]):
            ttk.Label(red_f, text=f"{name.title()}:").grid(
                row=i // 2, column=(i % 2) * 2, sticky="w")
            sl = ttk.Scale(red_f, from_=0, to=300,
                           orient="horizontal", length=80)
            sl.set(default)
            sl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="ew", padx=2)
            sl.configure(
                command=lambda v, n=name: s.set_red_margin(n, v))
            self.red_sliders[name] = sl
        red_f.columnconfigure(1, weight=1)
        red_f.columnconfigure(3, weight=1)

        # ===== TAB 3: VISION =====
        vision_tab = ttk.Frame(nb)
        nb.add(vision_tab, text="Vision")

        # Puck HSV
        phsv = ttk.LabelFrame(
            vision_tab, text="Puck HSV (Green)", padding=5)
        phsv.pack(fill="x", padx=8, pady=4)
        self.puck_sliders = {}
        for i, (name, default, mx) in enumerate([
            ("H Low", PUCK_HSV_LOW[0], 179),
            ("H High", PUCK_HSV_HIGH[0], 179),
            ("S Low", PUCK_HSV_LOW[1], 255),
            ("S High", PUCK_HSV_HIGH[1], 255),
            ("V Low", PUCK_HSV_LOW[2], 255),
            ("V High", PUCK_HSV_HIGH[2], 255),
        ]):
            ttk.Label(phsv, text=f"{name}:", width=6).grid(
                row=i // 2, column=(i % 2) * 2, sticky="w")
            sl = ttk.Scale(phsv, from_=0, to=mx,
                           orient="horizontal", length=80)
            sl.set(default)
            sl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="ew", padx=2)
            sl.configure(command=lambda v, n=name: self._upd_puck_hsv())
            self.puck_sliders[name] = sl
        phsv.columnconfigure(1, weight=1)
        phsv.columnconfigure(3, weight=1)

        # Mallet HSV
        mhsv = ttk.LabelFrame(
            vision_tab, text="Mallet HSV (Orange)", padding=5)
        mhsv.pack(fill="x", padx=8, pady=4)
        self.mallet_sliders = {}
        for i, (name, default, mx) in enumerate([
            ("H Low", MALLET_HSV_LOW[0], 179),
            ("H High", MALLET_HSV_HIGH[0], 179),
            ("S Low", MALLET_HSV_LOW[1], 255),
            ("S High", MALLET_HSV_HIGH[1], 255),
            ("V Low", MALLET_HSV_LOW[2], 255),
            ("V High", MALLET_HSV_HIGH[2], 255),
        ]):
            ttk.Label(mhsv, text=f"{name}:", width=6).grid(
                row=i // 2, column=(i % 2) * 2, sticky="w")
            sl = ttk.Scale(mhsv, from_=0, to=mx,
                           orient="horizontal", length=80)
            sl.set(default)
            sl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="ew", padx=2)
            sl.configure(command=lambda v, n=name: self._upd_mallet_hsv())
            self.mallet_sliders[name] = sl
        mhsv.columnconfigure(1, weight=1)
        mhsv.columnconfigure(3, weight=1)

        # Display options
        disp = ttk.LabelFrame(vision_tab, text="Display", padding=5)
        disp.pack(fill="x", padx=8, pady=4)
        self.show_mask_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            disp, text="Show Masks", variable=self.show_mask_var,
            command=lambda: setattr(s, 'show_mask',
                                    self.show_mask_var.get())).pack(anchor="w")
        self.show_roi_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            disp, text="Show ROI / Bounds", variable=self.show_roi_var,
            command=lambda: setattr(s, 'show_roi',
                                    self.show_roi_var.get())).pack(anchor="w")
        self.show_traj_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(
            disp, text="Show Trajectory", variable=self.show_traj_var,
            command=lambda: setattr(s, 'show_trajectory',
                                    self.show_traj_var.get())).pack(anchor="w")

        # ===== TAB 4: STATUS =====
        status_tab = ttk.Frame(nb)
        nb.add(status_tab, text="Status")

        # Tracking results
        tf = ttk.LabelFrame(status_tab, text="Tracking", padding=5)
        tf.pack(fill="x", padx=8, pady=4)
        self.fps_label = ttk.Label(tf, text="FPS: --", font=("Courier", 9))
        self.fps_label.pack(anchor="w")
        self.puck_label = ttk.Label(tf, text="Puck: --", font=("Courier", 9))
        self.puck_label.pack(anchor="w")
        self.mallet_label = ttk.Label(
            tf, text="Mallet: --", font=("Courier", 9))
        self.mallet_label.pack(anchor="w")
        self.motor_label = ttk.Label(
            tf, text="Motors: --", font=("Courier", 9))
        self.motor_label.pack(anchor="w")
        self.zone_label = ttk.Label(
            tf, text="Zone: OK", font=("Courier", 9))
        self.zone_label.pack(anchor="w")

        # Box readouts
        ro = ttk.LabelFrame(status_tab, text="Box Readouts (px)", padding=5)
        ro.pack(fill="x", padx=8, pady=4)
        self.roi_ro = ttk.Label(ro, text="ROI: --", font=("Courier", 8))
        self.roi_ro.pack(anchor="w")
        self.box_ro = ttk.Label(ro, text="Box: --", font=("Courier", 8))
        self.box_ro.pack(anchor="w")
        self.red_ro = ttk.Label(ro, text="Red: --", font=("Courier", 8))
        self.red_ro.pack(anchor="w")
        self.goal_ro = ttk.Label(ro, text="Goal: --", font=("Courier", 8))
        self.goal_ro.pack(anchor="w")
        self.home_ro = ttk.Label(ro, text="Home: --", font=("Courier", 8))
        self.home_ro.pack(anchor="w")

        # Print button
        ttk.Button(status_tab, text="Print All Values to Console",
                   command=self._print_all).pack(pady=8)

    # ---- HSV helpers ----

    def _upd_puck_hsv(self):
        low = [int(self.puck_sliders["H Low"].get()),
               int(self.puck_sliders["S Low"].get()),
               int(self.puck_sliders["V Low"].get())]
        high = [int(self.puck_sliders["H High"].get()),
                int(self.puck_sliders["S High"].get()),
                int(self.puck_sliders["V High"].get())]
        self.state.set_puck_hsv(low, high)

    def _upd_mallet_hsv(self):
        low = [int(self.mallet_sliders["H Low"].get()),
               int(self.mallet_sliders["S Low"].get()),
               int(self.mallet_sliders["V Low"].get())]
        high = [int(self.mallet_sliders["H High"].get()),
                int(self.mallet_sliders["S High"].get()),
                int(self.mallet_sliders["V High"].get())]
        self.state.set_mallet_hsv(low, high)

    # ---- Print all values ----

    def _print_all(self):
        s = self.state
        w, h = self.fw, self.fh
        roi, rad = s.get_table_roi()
        box = s.get_box_margins()
        red = s.get_red_margins()
        pl, ph = s.get_puck_hsv()
        ml, mh = s.get_mallet_hsv()
        gx, gy, gl = s.get_goal()
        hx, hy = s.get_home()
        spd = s.get_speed()

        print("\n" + "=" * 65)
        print("CURRENT VALUES – Paste into defensive_mode.py as new defaults")
        print("=" * 65)
        print(f'TABLE_ROI = {{"top": {roi["top"]}, "bottom": {roi["bottom"]}, '
              f'"left": {roi["left"]}, "right": {roi["right"]}}}')
        print(f'TABLE_CORNER_RADIUS = {rad}')
        print(f'MALLET_BOX_MARGINS = {{"top": {box["top"]}, '
              f'"bottom": {box["bottom"]}, '
              f'"left": {box["left"]}, "right": {box["right"]}}}')
        print(f'RED_ZONE_MARGINS = {{"top": {red["top"]}, '
              f'"bottom": {red["bottom"]}, '
              f'"left": {red["left"]}, "right": {red["right"]}}}')
        print(f'DEFAULT_SPEED = {spd}')
        print(f'DEFAULT_HOME_X = {hx}')
        print(f'DEFAULT_HOME_Y = {hy}')
        print(f'DEFAULT_GOAL_X = {gx}')
        print(f'DEFAULT_GOAL_Y = {gy}')
        print(f'DEFAULT_GOAL_LENGTH = {gl}')
        print(f'PUCK_HSV_LOW = np.array([{pl[0]}, {pl[1]}, {pl[2]}], '
              f'dtype=np.uint8)')
        print(f'PUCK_HSV_HIGH = np.array([{ph[0]}, {ph[1]}, {ph[2]}], '
              f'dtype=np.uint8)')
        print(f'MALLET_HSV_LOW = np.array([{ml[0]}, {ml[1]}, {ml[2]}], '
              f'dtype=np.uint8)')
        print(f'MALLET_HSV_HIGH = np.array([{mh[0]}, {mh[1]}, {mh[2]}], '
              f'dtype=np.uint8)')
        # Computed positions
        rx1, ry1 = roi["left"], roi["top"]
        rx2, ry2 = w - roi["right"], h - roi["bottom"]
        bx1, by1 = box["left"], box["top"]
        bx2, by2 = w - box["right"], h - box["bottom"]
        rdx1, rdy1 = bx1 + red["left"], by1 + red["top"]
        rdx2, rdy2 = bx2 - red["right"], by2 - red["bottom"]
        print(f'\n# Computed pixel rectangles:')
        print(f'# Table ROI : ({rx1},{ry1})-({rx2},{ry2}) '
              f'{rx2-rx1}x{ry2-ry1} R={rad}')
        print(f'# Mallet Box: ({bx1},{by1})-({bx2},{by2}) '
              f'{bx2-bx1}x{by2-by1}')
        print(f'# Red Zone  : ({rdx1},{rdy1})-({rdx2},{rdy2}) '
              f'{rdx2-rdx1}x{rdy2-rdy1}')
        print(f'# Goal      : x={gx} y={gy-gl//2}..{gy+gl//2} len={gl}')
        print(f'# Home      : ({hx},{hy})')
        print("=" * 65 + "\n")

    # ---- Periodic GUI refresh ----

    def _start_update(self):
        def tick():
            s = self.state
            w, h = self.fw, self.fh

            with s.lock:
                fps = s.fps
                px, py = s.puck_x, s.puck_y
                pspd = s.puck_speed
                pdet = s.puck_detected
                mx, my = s.mallet_x, s.mallet_y
                mr = s.mallet_r
                mdet = s.mallet_detected
                in_red = s.in_red
                minfo = s.motor_info
                game = s.game_enabled
                dstate = s.defense_state
                spd = s.speed

            self.fps_label.config(text=f"FPS: {fps:.0f}")
            ps = "Y" if pdet else "N"
            ms = "Y" if mdet else "N"
            self.puck_label.config(
                text=f"Puck: ({int(px)},{int(py)}) "
                     f"{pspd:.2f}m/s [{ps}]")
            self.mallet_label.config(
                text=f"Mallet: ({int(mx)},{int(my)}) "
                     f"r={int(mr)} [{ms}]")
            self.motor_label.config(text=f"Motors: {minfo}")
            self.zone_label.config(
                text="Zone: !! RED !!" if in_red else "Zone: OK")
            self.speed_val.config(text=f"{spd}%")

            # Game
            self.game_label.config(
                text="GAME ON  (G to toggle)" if game
                else "OFF  (G to toggle)")
            self.defense_label.config(text=f"State: {dstate}")

            # Readouts
            roi, rad = s.get_table_roi()
            box = s.get_box_margins()
            red = s.get_red_margins()
            gx, gy, gl = s.get_goal()
            hx, hy = s.get_home()

            rx1, ry1 = roi["left"], roi["top"]
            rx2, ry2 = w - roi["right"], h - roi["bottom"]
            self.roi_ro.config(
                text=f"ROI: ({rx1},{ry1})-({rx2},{ry2}) "
                     f"{rx2-rx1}x{ry2-ry1} R={rad}")

            bx1, by1 = box["left"], box["top"]
            bx2, by2 = w - box["right"], h - box["bottom"]
            self.box_ro.config(
                text=f"Box: ({bx1},{by1})-({bx2},{by2}) "
                     f"{bx2-bx1}x{by2-by1}")

            rdx1 = bx1 + red["left"]
            rdy1 = by1 + red["top"]
            rdx2 = bx2 - red["right"]
            rdy2 = by2 - red["bottom"]
            self.red_ro.config(
                text=f"Red: ({rdx1},{rdy1})-({rdx2},{rdy2}) "
                     f"{rdx2-rdx1}x{rdy2-rdy1}")

            self.goal_ro.config(
                text=f"Goal: x={gx} y={gy-gl//2}..{gy+gl//2} "
                     f"len={gl}")
            self.home_ro.config(text=f"Home: ({hx},{hy})")

            self.root.after(GUI_UPDATE_MS, tick)
        tick()

    def run(self):
        self.root.mainloop()


# ==================== TRACKING / DEFENSE THREAD ====================

def tracking_thread(state, stop_event):
    """Main loop: camera → detection → defense → motor → display."""
    print(f"\nOpening camera {CAM_INDEX}...")
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 90)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)

    if not cap.isOpened():
        print("Error: Cannot open camera")
        return

    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read camera")
        return

    h, w = frame.shape[:2]
    print(f"Camera: {w}x{h} @ {cap.get(cv2.CAP_PROP_FPS):.0f} FPS")

    # ---- Motor controller ----
    ctrl = None
    if MOTOR_ENABLED:
        try:
            ctrl = CoreXYController(
                MOTOR_PORT, MOTOR_BAUD, state.get_speed())
        except Exception as e:
            print(f"[Motor] Init failed ({e}) – vision-only mode")

    # ---- Processing buffers ----
    proc_w = int(w * PROCESSING_SCALE)
    proc_h = int(h * PROCESSING_SCALE)
    scale_inv = 1.0 / PROCESSING_SCALE
    hsv_buf = np.empty((proc_h, proc_w, 3), dtype=np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    print(f"Processing: {proc_w}x{proc_h}")

    # ---- Initial ROI mask (rounded) ----
    roi, radius = state.get_table_roi()
    roi_mask = _build_roi_mask(proc_w, proc_h, w, h, roi, radius)
    last_roi_key = _roi_key(roi, radius)

    # ---- Kalman filters ----
    puck_kf = KalmanTracker(w // 2, h // 2)
    mallet_kf = KalmanTracker(w // 2, h // 4)

    print("\nControls:")
    print("  G = Toggle game mode")
    print("  Numpad / WASD / Arrows = Manual drive (game OFF)")
    print("  Q / ESC = Quit  |  S = Screenshot")
    print("-" * 60)

    try:
        _inner_loop(state, stop_event, ctrl, cap,
                    w, h, proc_w, proc_h, scale_inv,
                    hsv_buf, kernel, roi_mask, last_roi_key,
                    puck_kf, mallet_kf)
    except Exception as e:
        print(f"\n!! Loop error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nStopping motors...")
        if ctrl:
            ctrl.close()
        cap.release()
        cv2.destroyAllWindows()


def _roi_key(roi, radius):
    return (roi['top'], roi['bottom'], roi['left'], roi['right'], radius)


def _build_roi_mask(proc_w, proc_h, w, h, roi, radius):
    rx1 = int(roi['left'] * PROCESSING_SCALE)
    ry1 = int(roi['top'] * PROCESSING_SCALE)
    rx2 = int((w - roi['right']) * PROCESSING_SCALE)
    ry2 = int((h - roi['bottom']) * PROCESSING_SCALE)
    return make_rounded_rect_mask(
        proc_h, proc_w, rx1, ry1, rx2, ry2,
        int(radius * PROCESSING_SCALE))


def _inner_loop(state, stop_event, ctrl, cap,
                w, h, proc_w, proc_h, scale_inv,
                hsv_buf, kernel, roi_mask, last_roi_key,
                puck_kf, mallet_kf):
    """Inner loop – separated so the outer try/finally always cleans up."""

    puck_lost_frames = 0
    mallet_lost_frames = 0
    puck_last_seen = time.perf_counter()

    # Manual drive
    dx, dy = 0, 0
    last_key_time = 0.0

    # FPS
    fps_alpha = 0.1
    fps = 0.0
    last_time = time.perf_counter()
    frame_count = 0

    while not stop_event.is_set():
        t0 = time.perf_counter()

        ret, frame = cap.read()
        if not ret:
            break
        t_cap = time.perf_counter()

        # ---- Read GUI parameters ----
        cur_speed = state.get_speed()
        roi, radius = state.get_table_roi()
        box_margins = state.get_box_margins()
        red_margins = state.get_red_margins()
        puck_hsv_low, puck_hsv_high = state.get_puck_hsv()
        mallet_hsv_low, mallet_hsv_high = state.get_mallet_hsv()
        goal_x, goal_y, goal_len = state.get_goal()
        home_x, home_y = state.get_home()
        game_on = state.is_game_enabled()

        # Push params to motor controller
        if ctrl:
            ctrl.speed_pct = cur_speed
            ctrl.mallet_box = [
                box_margins["left"], box_margins["top"],
                w - box_margins["right"], h - box_margins["bottom"]]
            ctrl.red_zone_margins = red_margins

        # ---- Rebuild ROI mask if changed ----
        cur_key = _roi_key(roi, radius)
        if cur_key != last_roi_key:
            roi_mask = _build_roi_mask(proc_w, proc_h, w, h, roi, radius)
            last_roi_key = cur_key

        # ---- Vision pipeline ----
        small = cv2.resize(frame, (proc_w, proc_h),
                           interpolation=cv2.INTER_LINEAR)
        cv2.GaussianBlur(small, (3, 3), 0, dst=small)
        cv2.cvtColor(small, cv2.COLOR_BGR2HSV, dst=hsv_buf)

        puck_mask = cv2.inRange(hsv_buf, puck_hsv_low, puck_hsv_high)
        cv2.bitwise_and(puck_mask, roi_mask, dst=puck_mask)
        cv2.morphologyEx(puck_mask, cv2.MORPH_OPEN, kernel, dst=puck_mask)
        cv2.morphologyEx(puck_mask, cv2.MORPH_CLOSE, kernel, dst=puck_mask)

        mallet_mask = cv2.inRange(hsv_buf, mallet_hsv_low, mallet_hsv_high)
        cv2.bitwise_and(mallet_mask, roi_mask, dst=mallet_mask)
        cv2.morphologyEx(mallet_mask, cv2.MORPH_OPEN, kernel,
                         dst=mallet_mask)
        cv2.morphologyEx(mallet_mask, cv2.MORPH_CLOSE, kernel,
                         dst=mallet_mask)

        puck_cnts, _ = cv2.findContours(
            puck_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mallet_cnts, _ = cv2.findContours(
            mallet_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        t_proc = time.perf_counter()

        # ---- Track puck (Kalman) ----
        px, py, pvx, pvy = puck_kf.predict()
        pred_hint = (None if puck_lost_frames >= LOST_THRESHOLD
                     else (px * PROCESSING_SCALE, py * PROCESSING_SCALE))
        puck = find_circle(puck_cnts,
                           PUCK_MIN_RADIUS * PROCESSING_SCALE,
                           PUCK_MAX_RADIUS * PROCESSING_SCALE, pred_hint)
        puck_det = puck is not None
        pr = PUCK_MIN_RADIUS

        if puck_det:
            px = puck[0] * scale_inv
            py = puck[1] * scale_inv
            pr = puck[2] * scale_inv
            if puck_lost_frames >= LOST_THRESHOLD:
                puck_kf = KalmanTracker(px, py)
                pvx, pvy = 0.0, 0.0
            else:
                px, py, pvx, pvy = puck_kf.correct(px, py)
            puck_lost_frames = 0
            puck_last_seen = time.perf_counter()
        else:
            puck_lost_frames += 1
            if time.perf_counter() - puck_last_seen >= PUCK_LOST_TIMEOUT:
                px, py = -1.0, -1.0
                pvx, pvy = 0.0, 0.0

        # ---- Track mallet (Kalman) ----
        mx, my, _, _ = mallet_kf.predict()
        mpred = (None if mallet_lost_frames >= LOST_THRESHOLD
                 else (mx * PROCESSING_SCALE, my * PROCESSING_SCALE))
        mallet = find_circle(mallet_cnts,
                             MALLET_MIN_RADIUS * PROCESSING_SCALE,
                             MALLET_MAX_RADIUS * PROCESSING_SCALE, mpred)
        mallet_det = mallet is not None
        mr = float(MALLET_MIN_RADIUS)

        if mallet_det:
            mx = mallet[0] * scale_inv
            my = mallet[1] * scale_inv
            mr = mallet[2] * scale_inv
            if mallet_lost_frames >= LOST_THRESHOLD:
                mallet_kf = KalmanTracker(mx, my)
            else:
                mx, my, _, _ = mallet_kf.correct(mx, my)
            mallet_lost_frames = 0
        else:
            mallet_lost_frames += 1

        # ---- FPS / timing ----
        now = time.perf_counter()
        dt = now - last_time
        last_time = now
        if dt > 0:
            fps = fps_alpha * (1.0 / dt) + (1 - fps_alpha) * fps
        cap_ms = (t_cap - t0) * 1000
        proc_ms = (t_proc - t_cap) * 1000
        puck_speed = (pvx * pvx + pvy * pvy) ** 0.5 * 0.001

        # ---- Key handling ----
        key = cv2.waitKeyEx(1)
        now_key = time.perf_counter()

        if key != -1:
            masked = key & 0xFF
            if masked == ord('q') or masked == 27:
                break
            elif masked == ord('g'):
                game_on = state.toggle_game()
                last_key_time = now_key
            elif masked == ord('s'):
                cv2.imwrite(f"screenshot_{int(time.time())}.png", frame)
                print("Screenshot saved!")
            elif not game_on:
                if key in KEY_MAP:
                    dx, dy = KEY_MAP[key]
                    last_key_time = now_key
                elif masked in KEY_MAP:
                    dx, dy = KEY_MAP[masked]
                    last_key_time = now_key

        if now_key - last_key_time > KEY_RELEASE_TIMEOUT:
            dx, dy = 0, 0

        # ============================================================
        #                      DEFENSE LOGIC
        # ============================================================
        vx, vy = 0.0, 0.0
        defense_state = "IDLE"
        intercept_x, intercept_y = None, None
        in_red = False

        table_left = roi['left']
        table_right = w - roi['right']
        table_top = roi['top']
        table_bottom = h - roi['bottom']

        # SAFETY: mallet not detected → STOP
        if not mallet_det:
            defense_state = "SAFETY STOP"
            vx, vy = 0.0, 0.0

        elif game_on:
            # ---- Autonomous defense ----
            # Red zone left edge – the closest X the mallet can safely reach.
            # The CoreXY boundary enforcement stops the mallet edge here.
            rz_left = box_margins['left'] + red_margins['left']

            if not puck_det or (px < 0 and py < 0):
                # Puck lost → go home
                defense_state = "HOMING"
                diff_x = home_x - mx
                diff_y = home_y - my
                dist = (diff_x ** 2 + diff_y ** 2) ** 0.5
                if dist > HOME_THRESHOLD:
                    vx = float(diff_y)        # camera→cmd 90° rotation
                    vy = float(-diff_x)
                else:
                    defense_state = "HOME"

            else:
                # Puck visible – check if it will reach the goal (≤1 bounce)
                goal_result = predict_intercept(
                    px, py, pvx, pvy,
                    goal_x, goal_y, goal_len,
                    table_left, table_right, table_top, table_bottom)

                if goal_result is not None:
                    # Puck WILL cross the goal line.
                    intercept_x, intercept_y = goal_result

                    # Find where the path crosses the red-zone edge
                    # (the safe X where the mallet can actually wait).
                    rz_result = predict_intercept(
                        px, py, pvx, pvy,
                        rz_left, goal_y, goal_len,
                        table_left, table_right, table_top, table_bottom)

                    target_x = float(rz_left)
                    if rz_result is not None:
                        target_y = rz_result[1]
                    else:
                        target_y = intercept_y
                    target_y = max(goal_y - goal_len // 2,
                                   min(goal_y + goal_len // 2, target_y))
                    defense_state = "INTERCEPT"

                    diff_x = target_x - mx
                    diff_y = target_y - my
                    dist = (diff_x ** 2 + diff_y ** 2) ** 0.5
                    if dist > HOME_THRESHOLD / 2:
                        vx = float(diff_y)
                        vy = float(-diff_x)
                else:
                    # No collision predicted – sit still
                    defense_state = "WAITING"

        else:
            # ---- Manual control ----
            defense_state = "MANUAL"
            vx, vy = float(dx), float(dy)

        # ---- Drive motor ----
        actual_vx, actual_vy = 0.0, 0.0
        if ctrl:
            if not mallet_det:
                ctrl.stop()
            else:
                actual_vx, actual_vy, in_red = ctrl.drive(
                    vx, vy, mx, my, mr)

        # ---- Motor info string ----
        if actual_vx != 0 or actual_vy != 0:
            m_vx, m_vy = -actual_vx, -actual_vy
            ra = m_vx + m_vy
            rb = m_vx - m_vy
            pk = max(abs(ra), abs(rb))
            cap_s = min(cur_speed, MAX_MOTOR_PCT)
            a_pct = abs(ra) / pk * cap_s if pk else 0
            b_pct = abs(rb) / pk * cap_s if pk else 0
            motor_info = (f"A:{a_pct:.0f}%{'R' if ra < 0 else 'F'} "
                          f"B:{b_pct:.0f}%{'R' if rb < 0 else 'F'}")
        else:
            motor_info = "A:0% B:0%"

        # ---- Update shared state ----
        state.update_tracking(
            px, py, puck_speed, puck_det,
            mx, my, mr, mallet_det,
            fps, cap_ms, proc_ms, in_red,
            intercept_x, intercept_y,
            motor_info, defense_state)

        # ============================================================
        #                      VISUALIZATION
        # ============================================================
        vis = frame.copy()

        if state.show_roi:
            # Table ROI (orange, rounded)
            draw_rounded_rect(
                vis,
                (roi['left'], roi['top']),
                (w - roi['right'], h - roi['bottom']),
                (0, 165, 255), 2, radius)

            # Mallet box (cyan)
            bx = [box_margins['left'], box_margins['top'],
                  w - box_margins['right'], h - box_margins['bottom']]
            cv2.rectangle(vis, (bx[0], bx[1]), (bx[2], bx[3]),
                          (255, 255, 0), 2)

            # Red zone
            rz = [bx[0] + red_margins['left'],
                  bx[1] + red_margins['top'],
                  bx[2] - red_margins['right'],
                  bx[3] - red_margins['bottom']]
            rc = (0, 0, 255) if in_red else (0, 0, 140)
            cv2.rectangle(vis, (rz[0], rz[1]), (rz[2], rz[3]), rc, 2)

            # Red overlay when mallet in zone
            if in_red:
                ov = vis.copy()
                cv2.rectangle(ov, (bx[0], bx[1]),
                              (bx[2], rz[1]), (0, 0, 200), -1)
                cv2.rectangle(ov, (bx[0], rz[3]),
                              (bx[2], bx[3]), (0, 0, 200), -1)
                cv2.rectangle(ov, (bx[0], rz[1]),
                              (rz[0], rz[3]), (0, 0, 200), -1)
                cv2.rectangle(ov, (rz[2], rz[1]),
                              (bx[2], rz[3]), (0, 0, 200), -1)
                cv2.addWeighted(ov, 0.3, vis, 0.7, 0, vis)

        # Goal (yellow vertical line)
        gy1 = goal_y - goal_len // 2
        gy2 = goal_y + goal_len // 2
        cv2.line(vis, (goal_x, gy1), (goal_x, gy2), (0, 255, 255), 3)

        # Home (green crosshair)
        cv2.drawMarker(vis, (home_x, home_y), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)

        # Trajectory
        if (state.show_trajectory and puck_det
                and (pvx * pvx + pvy * pvy) > 100):
            t_bounds = {'left': roi['left'], 'right': roi['right'],
                        'top': roi['top'], 'bottom': roi['bottom']}
            traj = predict_trajectory(px, py, pvx, pvy, w, h, t_bounds)
            if len(traj) > 1:
                pts = np.array(traj, dtype=np.int32)
                cv2.polylines(vis, [pts], False, (255, 0, 255), 2)
                cv2.circle(vis, traj[-1], 6, (255, 0, 255), -1)

        # Intercept marker (yellow dot + line from mallet)
        if intercept_x is not None:
            ix, iy = int(intercept_x), int(intercept_y)
            cv2.circle(vis, (ix, iy), 10, (0, 255, 255), 3)
            if mallet_det:
                cv2.line(vis, (int(mx), int(my)), (ix, iy),
                         (0, 255, 255), 1, cv2.LINE_AA)

        # Puck circle
        if puck_det:
            cv2.circle(vis, (int(px), int(py)), int(pr), (0, 255, 0), 3)
            cv2.putText(vis, f"PUCK {puck_speed:.2f}m/s",
                        (int(px) + 15, int(py) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Mallet circle
        if mallet_det:
            mc = (0, 0, 255) if in_red else (0, 165, 255)
            cv2.circle(vis, (int(mx), int(my)), int(mr), mc, 3)
            cv2.circle(vis, (int(mx), int(my)), 3, mc, -1)

        # Homing line
        if game_on and defense_state == "HOMING" and mallet_det:
            cv2.line(vis, (int(mx), int(my)), (home_x, home_y),
                     (0, 255, 0), 1, cv2.LINE_AA)

        # ---- Text overlay ----
        game_tag = " [GAME]" if game_on else ""
        cv2.putText(vis,
                    f"FPS:{fps:.0f} | {defense_state}{game_tag}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 2)
        cv2.putText(vis,
                    f"Puck:({int(px)},{int(py)}) "
                    f"Mallet:({int(mx)},{int(my)})",
                    (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    (200, 200, 200), 1)
        cv2.putText(vis,
                    f"Motors: {motor_info}  Speed: {cur_speed}%",
                    (10, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    (200, 200, 200), 1)

        if in_red:
            cv2.putText(vis, "!! RED ZONE !!",
                        (w // 2 - 80, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if not mallet_det:
            cv2.putText(vis, "!! SAFETY STOP - NO MALLET !!",
                        (w // 2 - 170, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Key legend
        cv2.putText(vis, "G=Game  WASD/Numpad=Manual  Q=Quit",
                    (w - 300, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (180, 180, 180), 1)

        cv2.imshow("Defense Mode", vis)

        # ---- Show masks ----
        if state.show_mask:
            pm_full = cv2.resize(puck_mask, (w, h),
                                 interpolation=cv2.INTER_NEAREST)
            mm_full = cv2.resize(mallet_mask, (w, h),
                                 interpolation=cv2.INTER_NEAREST)
            mv = np.zeros((h, w, 3), dtype=np.uint8)
            mv[:, :, 1] = pm_full
            mv[:, :, 2] = mm_full
            mv[:, :, 0] = mm_full // 2
            cv2.putText(mv, "GREEN=Puck  ORANGE=Mallet", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1)
            cv2.imshow("Masks", mv)
        else:
            try:
                cv2.destroyWindow("Masks")
            except cv2.error:
                pass

        # Console log
        frame_count += 1
        if frame_count % 90 == 0:
            gt = "GAME" if game_on else "MAN"
            print(f"\rFPS:{fps:.0f} P:{'Y' if puck_det else 'N'} "
                  f"M:{'Y' if mallet_det else 'N'} "
                  f"[{gt}] {defense_state}   ", end='')


# ==================== MAIN ====================

def main():
    print("=" * 60)
    print("  Air Hockey Defense Mode")
    print("=" * 60)
    print(f"  Motor : {MOTOR_PORT} @ {MOTOR_BAUD}")
    print(f"  Camera: {CAM_INDEX}")
    print(f"  G = Toggle game mode | WASD/Numpad = Manual")
    print("=" * 60)

    state = GameState()
    stop_event = Event()

    tracker = Thread(target=tracking_thread,
                     args=(state, stop_event), daemon=True)
    tracker.start()

    gui = ControlGUI(state)
    gui.run()

    stop_event.set()
    tracker.join(timeout=2)
    cv2.destroyAllWindows()
    print("\nDone.")


if __name__ == "__main__":
    main()
