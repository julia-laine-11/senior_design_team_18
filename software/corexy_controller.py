# CoreXY Differential Drive Controller
# Two stepper motors geared the same direction – single motor = diagonal,
# both together = cardinal axis (like an XY 3D-printer gantry).
#
# CoreXY kinematics:
#   Motor A = X + Y   (both same dir → +X, opposite dir → +Y)
#   Motor B = X - Y
#
# Uses the same 2-byte UART packet as defense_controller.py:
#   Byte 1 (Control):  0x80 | [motor B: 0x40] | [reverse: 0x20]
#   Byte 2 (Payload):  0-100  speed level (stepping frequency)
#
# IMPORTANT: The motor driver hardware maintains a fixed 50 % duty-cycle.
# Byte 2 controls the STEPPING FREQUENCY, not the duty-cycle width.
# Lower values = slower stepping, higher = faster.  0 = stopped.
#
# Includes mallet-box boundary enforcement with a red "stop zone."
# When the mallet EDGE (center ± radius) enters the red zone, movement
# toward that boundary is blocked, but movement AWAY is still allowed
# so the mallet can always escape.

import serial


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

        # Red zone: pixels INWARD from mallet box edge.
        # If the mallet edge enters this band, motion toward that wall is blocked.
        self.red_zone_margin = 30

        print(f"[CoreXY] Link on {port} @ {speed_pct}% speed")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def drive(self, dx, dy, mallet_x=None, mallet_y=None, mallet_r=0):
        """Send a direction command through CoreXY kinematics.

        dx, dy : each in {-1, 0, 1}  (screen coordinates – right/down positive)
        mallet_x, mallet_y : mallet centre in pixels (for boundary check)
        mallet_r : mallet radius in pixels

        Returns (actual_dx, actual_dy, in_red_zone).
        """
        in_red = False

        # Boundary enforcement (modifies dx/dy if needed)
        if mallet_x is not None and mallet_y is not None:
            dx, dy, in_red = self._enforce_bounds(
                mallet_x, mallet_y, mallet_r, dx, dy
            )

        if dx == 0 and dy == 0:
            self._motor('A', 0, False)
            self._motor('B', 0, False)
            return 0, 0, in_red

        # CoreXY: A = X + Y,  B = X - Y
        raw_a = dx + dy
        raw_b = dx - dy

        peak = max(abs(raw_a), abs(raw_b))
        a_pct = abs(raw_a) / peak * self.speed_pct if peak else 0
        b_pct = abs(raw_b) / peak * self.speed_pct if peak else 0

        self._motor('A', int(round(a_pct)), raw_a < 0)
        self._motor('B', int(round(b_pct)), raw_b < 0)

        return dx, dy, in_red

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
        m = self.red_zone_margin
        return [
            self.mallet_box[0] + m,
            self.mallet_box[1] + m,
            self.mallet_box[2] - m,
            self.mallet_box[3] - m,
        ]

    def _enforce_bounds(self, mx, my, mr, dx, dy):
        """Block directions that would push the mallet edge further into
        the red zone.  Movement AWAY from the wall is always allowed."""
        bx0, by0, bx1, by1 = self.mallet_box
        m = self.red_zone_margin
        in_red = False

        # Mallet edges
        edge_l = mx - mr
        edge_r = mx + mr
        edge_t = my - mr
        edge_b = my + mr

        # Left red zone
        if edge_l <= bx0 + m:
            in_red = True
            if dx < 0:
                dx = 0

        # Right red zone
        if edge_r >= bx1 - m:
            in_red = True
            if dx > 0:
                dx = 0

        # Top red zone
        if edge_t <= by0 + m:
            in_red = True
            if dy < 0:
                dy = 0

        # Bottom red zone
        if edge_b >= by1 - m:
            in_red = True
            if dy > 0:
                dy = 0

        return dx, dy, in_red

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
