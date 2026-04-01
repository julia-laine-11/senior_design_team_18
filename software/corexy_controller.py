# CoreXY Velocity-Based Differential Drive Controller
# Two stepper motors, same gearing, same direction – single motor = diagonal,
# both together = cardinal axis (like an XY 3D-printer gantry).
#
# CoreXY kinematics:
#   Motor A = Vx + Vy   (both same dir → +X, opposite dir → +Y)
#   Motor B = Vx - Vy
#
# Physical parameters:
#   Step angle  : 2°  → 180 steps/rev
#   Gear radius : 1.5"  → circumference ≈ 9.42"
#   Belt travel per step ≈ 0.0524"
#
# Uses the same 2-byte UART packet as defense_controller.py:
#   Byte 1 (Control):  0x80 | [motor B: 0x40] | [reverse: 0x20]
#   Byte 2 (Payload):  0-100  speed level (stepping frequency)
#   50 % duty-cycle is maintained by the driver hardware.
#
# Velocity control:
#   Caller provides a continuous velocity vector (vx, vy).
#   The controller normalises so the fastest motor = min(speed_pct, 65%).
#
# Red-zone boundary:
#   Movement INTO the red zone is blocked per-axis.
#   Movement AWAY (escaping) is always allowed.

import math
import serial

# --- Physical constants ---
STEP_ANGLE_DEG = 2.0
GEAR_RADIUS_IN = 1.5
STEPS_PER_REV = 360.0 / STEP_ANGLE_DEG            # 180
GEAR_CIRC_IN = 2.0 * math.pi * GEAR_RADIUS_IN     # ~9.42"
DIST_PER_STEP_IN = GEAR_CIRC_IN / STEPS_PER_REV   # ~0.0524"
MAX_MOTOR_PCT = 65                                 # hard cap per motor


class CoreXYController:
    """CoreXY differential drive with boundary-aware motor commands."""

    def __init__(self, port, baud_rate=115200, speed_pct=5):
        self.ser = serial.Serial(port, baud_rate)
        self.speed_pct = speed_pct
        self._last_a = None
        self._last_b = None

        # Mallet box – absolute pixel coords [left, top, right, bottom]
        # Calibrate these to match your camera view of the physical play area.
        self.mallet_box = [20, 20, 620, 460]

        # Red zone: pixels INWARD from mallet box edge (per-side).
        # If the mallet edge enters this band, ALL motion is stopped.
        self.red_zone_margins = {"top": 30, "bottom": 30, "left": 30, "right": 30}

        print(f"[CoreXY] Link on {port} @ {speed_pct}% speed")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def drive(self, vx, vy, mallet_x=None, mallet_y=None, mallet_r=0):
        """Drive with a continuous velocity vector.

        vx, vy : any float – direction AND ratio matter; magnitude is
                 normalised so the fastest motor = min(speed_pct, 65 %).
        mallet_x, mallet_y : mallet centre in pixels (for boundary check)
        mallet_r : mallet radius in pixels

        Returns (actual_vx, actual_vy, in_red_zone).
        """
        in_red = False

        # Boundary enforcement (may zero individual components)
        if mallet_x is not None and mallet_y is not None:
            vx, vy, in_red = self._enforce_bounds(
                mallet_x, mallet_y, mallet_r, vx, vy
            )

        if vx == 0 and vy == 0:
            self._motor('A', 0, False)
            self._motor('B', 0, False)
            return 0.0, 0.0, in_red

        # Negate for physical motor wiring (flip both axes)
        mvx, mvy = -vx, -vy

        # CoreXY: A = Vx + Vy,  B = Vx - Vy
        raw_a = mvx + mvy
        raw_b = mvx - mvy

        # Normalise so fastest motor = min(speed_pct, MAX_MOTOR_PCT)
        peak = max(abs(raw_a), abs(raw_b))
        cap = min(self.speed_pct, MAX_MOTOR_PCT)
        scale = cap / peak if peak else 0

        a_pct = abs(raw_a) * scale
        b_pct = abs(raw_b) * scale

        self._motor('A', int(round(a_pct)), raw_a < 0)
        self._motor('B', int(round(b_pct)), raw_b < 0)

        return vx, vy, in_red

    def stop(self):
        """Immediately stop both motors."""
        self._motor('A', 0, False)
        self._motor('B', 0, False)

    def close(self):
        self.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[CoreXY] Port closed.")

    # ------------------------------------------------------------------
    # Boundary helpers
    # ------------------------------------------------------------------

    def get_red_zone_rect(self):
        """Inner rectangle of the red stop-zone [left, top, right, bottom]."""
        rm = self.red_zone_margins
        return [
            self.mallet_box[0] + rm["left"],
            self.mallet_box[1] + rm["top"],
            self.mallet_box[2] - rm["right"],
            self.mallet_box[3] - rm["bottom"],
        ]

    def _enforce_bounds(self, mx, my, mr, vx, vy):
        """Block velocity components that push further into the red zone.
        Movement AWAY from the zone is always allowed (escape)."""
        bx0, by0, bx1, by1 = self.mallet_box
        rm = self.red_zone_margins
        in_red = False

        # Mallet edges
        edge_l = mx - mr
        edge_r = mx + mr
        edge_t = my - mr
        edge_b = my + mr

        # Left red zone – block leftward
        if edge_l <= bx0 + rm["left"]:
            in_red = True
            if vx < 0:
                vx = 0

        # Right red zone – block rightward
        if edge_r >= bx1 - rm["right"]:
            in_red = True
            if vx > 0:
                vx = 0

        # Top red zone – block upward
        if edge_t <= by0 + rm["top"]:
            in_red = True
            if vy < 0:
                vy = 0

        # Bottom red zone – block downward
        if edge_b >= by1 - rm["bottom"]:
            in_red = True
            if vy > 0:
                vy = 0

        return vx, vy, in_red

    # ------------------------------------------------------------------
    # UART helpers  (same protocol as defense_controller.py)
    # ------------------------------------------------------------------

    def _motor(self, which, speed, reverse):
        """Send 2-byte command.  `speed` = stepping frequency level 0-100.
        50 % duty-cycle is maintained by the driver hardware."""
        speed = max(0, min(100, int(speed)))

        b1 = 0x80
        if which == 'B':
            b1 |= 0x40
        if reverse:
            b1 |= 0x20

        b2 = speed
        pair = (b1, b2)

        if which == 'A':
            if pair == self._last_a:
                return
            self._last_a = pair
        else:
            if pair == self._last_b:
                return
            self._last_b = pair

        self.ser.write(bytes([b1, b2]))
