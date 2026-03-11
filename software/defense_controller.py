# Defense Controller – Dual Motor (2-Byte Protocol)
# Drives two motors over a single UART link using the same 2-byte
# packet format as embedded/src/uart.py.
#
# Packet format (per motor command):
#   Byte 1 (Control):  0x80 | [motor B: 0x40] | [reverse: 0x20]
#   Byte 2 (Payload):  0-100  (duty-cycle percentage, 0 = OFF)
#
# Behavior (both motors):
#   - Mallet leaves its ROI  -> STOP both  (0%)
#   - Puck leaves its ROI    -> reverse both at minimum percent
#   - Both visible:
#       Motor A: closer puck = faster  (percent toward 100)
#       Motor B: closer puck = slower  (percent toward 0)  (inverse of A)

import serial
import math


class DefenseController:
    """Maps vision tracking state to two inversely-proportional motors."""

    # --- tunables ---
    MAX_DISTANCE_PX = 500   # beyond this distance -> 0 % for A
    MIN_DISTANCE_PX = 50    # closer than this     -> 100 % for A
    LOST_PUCK_PCT   = 10    # reverse % when puck leaves ROI

    def __init__(self, port, baud_rate=115200):
        self.ser = serial.Serial(port, baud_rate)
        self._last_a = None   # de-duplicate (ctrl, payload) pairs
        self._last_b = None
        print(f"[Defense] Dual-motor link on {port}")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def update(self, puck_x, puck_y, puck_detected,
               mallet_x, mallet_y, mallet_detected):
        """Call once per frame with the latest tracking results."""
        # Priority 1 – mallet not in its ROI -> STOP both
        if not mallet_detected:
            self._send('A', 0, reverse=False)
            self._send('B', 0, reverse=False)
            return

        # Priority 2 – puck not in its ROI -> reverse both at low speed
        if not puck_detected or (puck_x < 0 and puck_y < 0):
            self._send('A', self.LOST_PUCK_PCT, reverse=True)
            self._send('B', self.LOST_PUCK_PCT, reverse=True)
            return

        # Both objects visible – inversely proportional speeds
        distance = math.hypot(puck_x - mallet_x, puck_y - mallet_y)
        pct_a = self._distance_to_percent(distance)
        pct_b = 100 - pct_a  # inverse
        self._send('A', pct_a, reverse=False)
        self._send('B', pct_b, reverse=False)

    def stop(self):
        self._send('A', 0, reverse=False)
        self._send('B', 0, reverse=False)

    def close(self):
        self.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[Defense] Port closed.")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _distance_to_percent(self, distance):
        """Map pixel distance to duty-cycle percent 0-100.

        Closer  -> higher percent (faster)
        Farther -> lower  percent (slower)
        """
        clamped = max(self.MIN_DISTANCE_PX, min(self.MAX_DISTANCE_PX, distance))
        t = (clamped - self.MIN_DISTANCE_PX) / (self.MAX_DISTANCE_PX - self.MIN_DISTANCE_PX)
        return int(100 - t * 100)

    def _send(self, motor, percent, reverse=False):
        """Build and transmit a 2-byte packet (uart.py protocol)."""
        percent = max(0, min(100, percent))

        byte_1 = 0x80
        if motor == 'B':
            byte_1 |= 0x40
        if reverse:
            byte_1 |= 0x20

        byte_2 = percent
        pair = (byte_1, byte_2)

        # De-duplicate per motor
        if motor == 'A':
            if pair == self._last_a:
                return
            self._last_a = pair
        else:
            if pair == self._last_b:
                return
            self._last_b = pair

        self.ser.write(bytes([byte_1, byte_2]))
