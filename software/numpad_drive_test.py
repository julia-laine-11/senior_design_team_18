# Numpad Drive Test – CoreXY Manual Control
#
# Tkinter control panel + OpenCV camera window.
# Hold a direction key to move; release to stop.
#
# ┌─────────────────────────────────┐
# │  Numpad (NumLock ON):           │
# │    7 (↖)   8 (↑)   9 (↗)      │
# │    4 (←)   5 (⏹)   6 (→)      │
# │    1 (↙)   2 (↓)   3 (↘)      │
# │                                 │
# │  Also: W/A/S/D, Arrow keys     │
# │  Q or ESC = Quit               │
# └─────────────────────────────────┘
#
# Speed byte controls STEPPING FREQUENCY (50% duty-cycle is
# maintained by the motor driver hardware).

import numpy as np
import cv2
import time
import tkinter as tk
from tkinter import ttk
from threading import Thread, Lock, Event

from corexy_controller import CoreXYController

# ===================== CONFIGURATION =====================

# Camera
CAM_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Motor UART
MOTOR_PORT = 'COM4'
MOTOR_BAUD = 115200

# Defaults
DEFAULT_SPEED = 5
DEFAULT_BOX_MARGINS = {"top": 20, "bottom": 20, "left": 20, "right": 20}
DEFAULT_RED_MARGINS = {"top": 30, "bottom": 30, "left": 30, "right": 30}

# Mallet HSV (same defaults as puck_mallet_tracker_fast.py)
MALLET_HSV_LOW  = np.array([5, 127, 100], dtype=np.uint8)
MALLET_HSV_HIGH = np.array([25, 209, 255], dtype=np.uint8)
MALLET_MIN_RADIUS = 15
MALLET_MAX_RADIUS = 60

# Hold-to-move: stop motors if no key event for this long
KEY_RELEASE_TIMEOUT = 0.20   # seconds

GUI_UPDATE_MS = 100

# ===================== KEY MAPPINGS ======================

_NUM_DIRS = {
    ord('7'): (-1, -1),  ord('8'): (0, -1),  ord('9'): (1, -1),
    ord('4'): (-1,  0),  ord('5'): (0,  0),  ord('6'): (1,  0),
    ord('1'): (-1,  1),  ord('2'): (0,  1),  ord('3'): (1,  1),
}
_WASD_DIRS = {
    ord('w'): (0, -1),  ord('a'): (-1, 0),
    ord('s'): (0,  1),  ord('d'): ( 1, 0),
    ord(' '): (0,  0),
}
_ARROW_DIRS = {
    2490368: (0, -1),   # Up
    2621440: (0,  1),   # Down
    2424832: (-1, 0),   # Left
    2555904: ( 1, 0),   # Right
}
KEY_MAP = {**_NUM_DIRS, **_WASD_DIRS, **_ARROW_DIRS}

DIR_NAMES = {
    (-1, -1): "UP-LEFT",    (0, -1): "UP",       (1, -1): "UP-RIGHT",
    (-1,  0): "LEFT",       (0,  0): "STOP",     (1,  0): "RIGHT",
    (-1,  1): "DOWN-LEFT",  (0,  1): "DOWN",     (1,  1): "DOWN-RIGHT",
}

# ===================== SHARED STATE ======================

class DriveState:
    """Thread-safe state shared between drive loop and GUI."""

    def __init__(self):
        self.lock = Lock()

        # --- GUI-controlled parameters ---
        self.box_margins = DEFAULT_BOX_MARGINS.copy()
        self.red_margins = DEFAULT_RED_MARGINS.copy()
        self.speed = DEFAULT_SPEED

        self.mallet_hsv_low = list(MALLET_HSV_LOW)
        self.mallet_hsv_high = list(MALLET_HSV_HIGH)

        self.show_mask = False

        # --- Status (written by drive thread, read by GUI) ---
        self.cmd_dir = (0, 0)
        self.actual_dir = (0, 0)
        self.in_red = False
        self.mallet_pos = None        # (x, y, r) or None
        self.motor_info = "A: 0%  B: 0%"
        self.fps = 0.0

    # ---- getters / setters (all lock-protected) ----

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

    def get_speed(self):
        with self.lock:
            return self.speed

    def set_speed(self, val):
        with self.lock:
            self.speed = int(float(val))

    def get_mallet_hsv(self):
        with self.lock:
            return (np.array(self.mallet_hsv_low, dtype=np.uint8),
                    np.array(self.mallet_hsv_high, dtype=np.uint8))

    def set_mallet_hsv(self, low, high):
        with self.lock:
            self.mallet_hsv_low = list(low)
            self.mallet_hsv_high = list(high)

    def update_status(self, cmd_dir, actual_dir, in_red, mallet_pos,
                      motor_info, fps):
        with self.lock:
            self.cmd_dir = cmd_dir
            self.actual_dir = actual_dir
            self.in_red = in_red
            self.mallet_pos = mallet_pos
            self.motor_info = motor_info
            self.fps = fps

# ===================== TKINTER GUI =======================

class DriveGUI:
    """Tkinter control panel (runs in main thread)."""

    def __init__(self, state):
        self.state = state
        self.root = tk.Tk()
        self.root.title("CoreXY Drive Controls")
        self.root.geometry("420x750")
        self._create_widgets()
        self._start_update()

    # ---- widget creation ----

    def _create_widgets(self):
        s = self.state

        # --- Info ---
        info = ttk.LabelFrame(self.root, text="Info", padding=8)
        info.pack(fill="x", padx=10, pady=4)
        ttk.Label(info, text="Hold key to move, release to stop",
                  font=("Arial", 9, "bold")).pack()
        self.fps_label = ttk.Label(info, text="FPS: --", font=("Courier", 10))
        self.fps_label.pack()

        # --- Speed ---
        spd = ttk.LabelFrame(self.root, text="Speed (stepping freq level)",
                              padding=8)
        spd.pack(fill="x", padx=10, pady=4)
        self.speed_slider = ttk.Scale(spd, from_=1, to=100,
                                      orient="horizontal")
        self.speed_slider.set(DEFAULT_SPEED)
        self.speed_slider.pack(fill="x")
        self.speed_slider.configure(
            command=lambda v: s.set_speed(v))
        self.speed_val_label = ttk.Label(spd, text=f"{DEFAULT_SPEED}",
                                         font=("Courier", 10))
        self.speed_val_label.pack()

        # --- Mallet Box Margins ---
        box_frame = ttk.LabelFrame(self.root, text="Mallet Box Margins (Cyan)",
                                   padding=5)
        box_frame.pack(fill="x", padx=10, pady=4)
        self.box_sliders = {}
        for i, (name, default) in enumerate([
            ("top", DEFAULT_BOX_MARGINS["top"]),
            ("bottom", DEFAULT_BOX_MARGINS["bottom"]),
            ("left", DEFAULT_BOX_MARGINS["left"]),
            ("right", DEFAULT_BOX_MARGINS["right"]),
        ]):
            ttk.Label(box_frame, text=f"{name.title()}:").grid(
                row=i // 2, column=(i % 2) * 2, sticky="w")
            sl = ttk.Scale(box_frame, from_=0, to=300,
                           orient="horizontal", length=80)
            sl.set(default)
            sl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="ew", padx=2)
            sl.configure(command=lambda v, n=name: s.set_box_margin(n, v))
            self.box_sliders[name] = sl
        box_frame.columnconfigure(1, weight=1)
        box_frame.columnconfigure(3, weight=1)

        # --- Red Zone Margins ---
        red_frame = ttk.LabelFrame(self.root, text="Red Zone Margins (Red)",
                                   padding=5)
        red_frame.pack(fill="x", padx=10, pady=4)
        self.red_sliders = {}
        for i, (name, default) in enumerate([
            ("top", DEFAULT_RED_MARGINS["top"]),
            ("bottom", DEFAULT_RED_MARGINS["bottom"]),
            ("left", DEFAULT_RED_MARGINS["left"]),
            ("right", DEFAULT_RED_MARGINS["right"]),
        ]):
            ttk.Label(red_frame, text=f"{name.title()}:").grid(
                row=i // 2, column=(i % 2) * 2, sticky="w")
            sl = ttk.Scale(red_frame, from_=0, to=150,
                           orient="horizontal", length=80)
            sl.set(default)
            sl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="ew", padx=2)
            sl.configure(command=lambda v, n=name: s.set_red_margin(n, v))
            self.red_sliders[name] = sl
        red_frame.columnconfigure(1, weight=1)
        red_frame.columnconfigure(3, weight=1)

        # --- Mallet HSV Calibration ---
        hsv_frame = ttk.LabelFrame(self.root, text="Mallet HSV (Orange)",
                                   padding=5)
        hsv_frame.pack(fill="x", padx=10, pady=4)
        self.hsv_sliders = {}
        hsv_defaults = [
            ("H Low", MALLET_HSV_LOW[0], 179),
            ("H High", MALLET_HSV_HIGH[0], 179),
            ("S Low", MALLET_HSV_LOW[1], 255),
            ("S High", MALLET_HSV_HIGH[1], 255),
            ("V Low", MALLET_HSV_LOW[2], 255),
            ("V High", MALLET_HSV_HIGH[2], 255),
        ]
        for i, (name, default, max_val) in enumerate(hsv_defaults):
            ttk.Label(hsv_frame, text=f"{name}:", width=6).grid(
                row=i // 2, column=(i % 2) * 2, sticky="w")
            sl = ttk.Scale(hsv_frame, from_=0, to=max_val,
                           orient="horizontal", length=80)
            sl.set(default)
            sl.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="ew", padx=2)
            sl.configure(command=lambda v, n=name: self._update_hsv())
            self.hsv_sliders[name] = sl
        hsv_frame.columnconfigure(1, weight=1)
        hsv_frame.columnconfigure(3, weight=1)

        # --- Display ---
        disp = ttk.LabelFrame(self.root, text="Display", padding=5)
        disp.pack(fill="x", padx=10, pady=4)
        self.show_mask_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(disp, text="Show Mallet Mask",
                        variable=self.show_mask_var,
                        command=self._toggle_mask).pack(anchor="w")

        # --- Status ---
        status = ttk.LabelFrame(self.root, text="Status", padding=8)
        status.pack(fill="x", padx=10, pady=4)
        self.dir_label = ttk.Label(status, text="Dir: STOP",
                                   font=("Courier", 9))
        self.dir_label.pack(anchor="w")
        self.motor_label = ttk.Label(status, text="Motors: A:0% B:0%",
                                     font=("Courier", 9))
        self.motor_label.pack(anchor="w")
        self.mallet_label = ttk.Label(status, text="Mallet: --",
                                      font=("Courier", 9))
        self.mallet_label.pack(anchor="w")
        self.zone_label = ttk.Label(status, text="Zone: OK",
                                    font=("Courier", 9))
        self.zone_label.pack(anchor="w")

    # ---- callbacks ----

    def _update_hsv(self):
        low = [int(self.hsv_sliders["H Low"].get()),
               int(self.hsv_sliders["S Low"].get()),
               int(self.hsv_sliders["V Low"].get())]
        high = [int(self.hsv_sliders["H High"].get()),
                int(self.hsv_sliders["S High"].get()),
                int(self.hsv_sliders["V High"].get())]
        self.state.set_mallet_hsv(low, high)

    def _toggle_mask(self):
        self.state.show_mask = self.show_mask_var.get()

    # ---- periodic GUI update ----

    def _start_update(self):
        def tick():
            s = self.state
            with s.lock:
                cmd = s.cmd_dir
                act = s.actual_dir
                red = s.in_red
                mpos = s.mallet_pos
                minfo = s.motor_info
                fps = s.fps
                spd = s.speed

            self.fps_label.config(text=f"FPS: {fps:.0f}")
            self.speed_val_label.config(text=f"{spd}")

            act_name = DIR_NAMES.get(act, "?")
            self.dir_label.config(text=f"Dir: {act_name}")
            self.motor_label.config(text=f"Motors: {minfo}")

            if mpos:
                self.mallet_label.config(
                    text=f"Mallet: ({int(mpos[0])},{int(mpos[1])}) "
                         f"r={int(mpos[2])}")
            else:
                self.mallet_label.config(text="Mallet: NOT DETECTED")

            self.zone_label.config(
                text="Zone: !! RED ZONE !!" if red else "Zone: OK")

            self.root.after(GUI_UPDATE_MS, tick)
        tick()

    def run(self):
        self.root.mainloop()

# ===================== MALLET DETECTION ==================

def find_mallet(contours, min_r, max_r):
    """Return (x, y, radius) of the most circular contour, or None."""
    best, best_score = None, -1
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
        if circ > best_score:
            best_score = circ
            best = (x, y, r)
    return best

# ===================== DRIVE LOOP ========================

def drive_loop(state, stop_event):
    """Background thread: camera + key input + motor drive."""

    # ---- Camera ----
    has_camera = False
    cap = cv2.VideoCapture(CAM_INDEX)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            has_camera = True
            print(f"  Camera: {w}x{h}")
        else:
            print("  WARNING: Camera read failed.")
    else:
        print("  WARNING: No camera – no boundary enforcement.")

    if not has_camera:
        w, h = FRAME_WIDTH, FRAME_HEIGHT

    # ---- Motor controller ----
    ctrl = None
    try:
        ctrl = CoreXYController(MOTOR_PORT, MOTOR_BAUD, state.get_speed())
    except Exception as e:
        print(f"  Motor init failed ({e}) – vision only mode.")

    # ---- Loop state ----
    dx, dy = 0, 0
    last_key_time = 0.0
    mallet_x, mallet_y, mallet_r = None, None, 0
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    fps_alpha = 0.1
    fps = 0.0
    last_t = time.perf_counter()

    print("  Drive loop running.\n")

    while not stop_event.is_set():
        t0 = time.perf_counter()

        # ---- Read params from GUI state ----
        cur_speed = state.get_speed()
        margins = state.get_box_margins()
        red_margins = state.get_red_margins()
        hsv_low, hsv_high = state.get_mallet_hsv()

        if ctrl:
            ctrl.speed_pct = cur_speed
            ctrl.mallet_box = [
                margins["left"], margins["top"],
                w - margins["right"], h - margins["bottom"]]
            ctrl.red_zone_margins = red_margins

        # ---- Camera / detection ----
        if has_camera:
            ret, frame = cap.read()
            if not ret:
                break
            vis = frame.copy()

            hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_img, hsv_low, hsv_high)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

            mallet = find_mallet(cnts, MALLET_MIN_RADIUS, MALLET_MAX_RADIUS)
            if mallet:
                mallet_x, mallet_y, mallet_r = mallet
            else:
                mallet_x, mallet_y, mallet_r = None, None, 0
        else:
            vis = np.full((h, w, 3), 40, dtype=np.uint8)
            mask = None

        # ---- Key handling (hold-to-move) ----
        key = cv2.waitKeyEx(1)
        now = time.perf_counter()

        if key != -1:
            masked = key & 0xFF
            if masked == ord('q') or masked == 27:
                break
            if key in KEY_MAP:
                dx, dy = KEY_MAP[key]
                last_key_time = now
            elif masked in KEY_MAP:
                dx, dy = KEY_MAP[masked]
                last_key_time = now

        # Auto-stop when key released
        if now - last_key_time > KEY_RELEASE_TIMEOUT:
            dx, dy = 0, 0

        # ---- Drive ----
        actual_dx, actual_dy, in_red = 0, 0, False
        if ctrl:
            actual_dx, actual_dy, in_red = ctrl.drive(
                dx, dy, mallet_x, mallet_y, mallet_r)

        # ---- Compute motor info string ----
        if actual_dx != 0 or actual_dy != 0:
            ra = actual_dx + actual_dy
            rb = actual_dx - actual_dy
            pk = max(abs(ra), abs(rb))
            a_pct = abs(ra) / pk * cur_speed if pk else 0
            b_pct = abs(rb) / pk * cur_speed if pk else 0
            a_dir = "REV" if ra < 0 else "FWD"
            b_dir = "REV" if rb < 0 else "FWD"
            motor_info = f"A:{a_pct:.0f}% {a_dir}  B:{b_pct:.0f}% {b_dir}"
        else:
            motor_info = "A: 0%  B: 0%"

        # ---- FPS ----
        dt = now - last_t
        last_t = now
        if dt > 0:
            fps = fps_alpha * (1.0 / dt) + (1 - fps_alpha) * fps

        # ---- Update shared status ----
        state.update_status(
            cmd_dir=(dx, dy),
            actual_dir=(actual_dx, actual_dy),
            in_red=in_red,
            mallet_pos=(mallet_x, mallet_y, mallet_r)
                       if mallet_x is not None else None,
            motor_info=motor_info,
            fps=fps,
        )

        # ---- Visualization ----
        box = ctrl.mallet_box if ctrl else [
            margins["left"], margins["top"],
            w - margins["right"], h - margins["bottom"]]
        red_rect = [box[0] + red_margins["left"], box[1] + red_margins["top"],
                    box[2] - red_margins["right"], box[3] - red_margins["bottom"]]

        # Mallet box (cyan)
        cv2.rectangle(vis, (box[0], box[1]), (box[2], box[3]),
                      (255, 255, 0), 2)

        # Red stop-zone
        rc = (0, 0, 255) if in_red else (0, 0, 140)
        rt = 3 if in_red else 2
        cv2.rectangle(vis, (red_rect[0], red_rect[1]),
                      (red_rect[2], red_rect[3]), rc, rt)

        # Semi-transparent red overlay when mallet is in zone
        if in_red:
            ov = vis.copy()
            cv2.rectangle(ov, (box[0], box[1]),
                          (box[2], red_rect[1]), (0, 0, 200), -1)
            cv2.rectangle(ov, (box[0], red_rect[3]),
                          (box[2], box[3]), (0, 0, 200), -1)
            cv2.rectangle(ov, (box[0], red_rect[1]),
                          (red_rect[0], red_rect[3]), (0, 0, 200), -1)
            cv2.rectangle(ov, (red_rect[2], red_rect[1]),
                          (box[2], red_rect[3]), (0, 0, 200), -1)
            cv2.addWeighted(ov, 0.3, vis, 0.7, 0, vis)

        # Mallet circle
        if mallet_x is not None:
            mc = (0, 0, 255) if in_red else (0, 165, 255)
            cv2.circle(vis, (int(mallet_x), int(mallet_y)),
                       int(mallet_r), mc, 3)
            cv2.circle(vis, (int(mallet_x), int(mallet_y)), 3, mc, -1)

        # Direction arrow
        cx, cy = w // 2, 35
        if actual_dx != 0 or actual_dy != 0:
            cv2.arrowedLine(vis, (cx, cy),
                            (cx + actual_dx * 20, cy + actual_dy * 20),
                            (0, 255, 0), 3, tipLength=0.4)
        else:
            cv2.circle(vis, (cx, cy), 6, (128, 128, 128), -1)

        # Text overlay
        y0 = h - 70
        act_name = DIR_NAMES.get((actual_dx, actual_dy), "?")
        cv2.putText(vis, f"Dir: {act_name}  Motors: {motor_info}",
                    (10, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1)
        cv2.putText(vis, f"Speed: {cur_speed}%   FPS: {fps:.0f}",
                    (10, y0 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1)
        if mallet_x is not None:
            cv2.putText(vis,
                        f"Mallet: ({int(mallet_x)},{int(mallet_y)}) "
                        f"r={int(mallet_r)}",
                        (10, y0 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                        (0, 165, 255), 1)
        else:
            cv2.putText(vis, "Mallet: NOT DETECTED",
                        (10, y0 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                        (0, 0, 255), 1)

        if in_red:
            cv2.putText(vis, "!! RED ZONE !!",
                        (w // 2 - 90, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255), 2)

        # Key legend
        cv2.putText(vis, "7 8 9 / 4 5 6 / 1 2 3  WASD  Arrows",
                    (w - 310, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.33,
                    (200, 200, 200), 1)

        cv2.imshow("Numpad Drive Test", vis)

        # Show mask
        if has_camera and state.show_mask and mask is not None:
            cv2.imshow("Mallet Mask", mask)
        elif not state.show_mask:
            try:
                cv2.destroyWindow("Mallet Mask")
            except cv2.error:
                pass

    # ---- Cleanup ----
    print("\n  Stopping motors...")
    if ctrl:
        ctrl.close()
    cap.release()
    cv2.destroyAllWindows()

# ===================== MAIN ==============================

def main():
    print("=" * 55)
    print("  Numpad Drive Test – CoreXY Differential Drive")
    print("=" * 55)
    print(f"  Port  : {MOTOR_PORT}")
    print("  Keys  : Numpad 1-9, WASD, Arrows (hold to move)")
    print("  Speed controls STEPPING FREQUENCY (50% duty-cycle)")
    print("=" * 55)

    state = DriveState()
    stop_event = Event()

    # Drive loop in background thread
    thread = Thread(target=drive_loop, args=(state, stop_event), daemon=True)
    thread.start()

    # Tkinter GUI in main thread
    gui = DriveGUI(state)
    gui.run()

    # Cleanup
    stop_event.set()
    thread.join(timeout=2)
    cv2.destroyAllWindows()
    print("  Done.")


if __name__ == "__main__":
    main()
