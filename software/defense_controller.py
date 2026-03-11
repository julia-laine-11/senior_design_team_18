# Defense Controller – Dual Motor
# Drives two motors with inversely proportional speeds based on
# puck-to-mallet distance, both via UART.
# Uses the same single-byte encoding protocol as embedded/src/uart.py.
#
# Encoding:
#   freq 100-200 forward  -> byte = (freq - 100)
#   freq 100-200 reverse  -> byte = (freq - 100) | 0x80
#   stop (freq 0)         -> byte = 0x7F
#
# Behavior (both motors):
#   - Mallet leaves its ROI  -> STOP both
#   - Puck leaves its ROI    -> reverse both at minimum speed (-100)
#   - Both visible:
#       Motor A: closer puck = faster  (freq toward 200)
#       Motor B: closer puck = slower  (freq toward 100)  (inverse of A)

import serial
import math


class _MotorLink:
    """Manages a single serial connection to one motor controller."""

    def __init__(self, port, baud_rate, label):
        self.ser = serial.Serial(port, baud_rate)
        self.label = label
        self._last_byte = None
        print(f"[Defense] {label} connected on {port}")

    def write(self, byte_val):
        if byte_val == self._last_byte:
            return
        self.ser.write(bytes([byte_val]))
        self._last_byte = byte_val

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"[Defense] {self.label} port closed.")


class DefenseController:
    """Maps vision tracking state to two inversely-proportional motors."""

    # --- tunables ---
    MIN_FREQ = 100          # slowest motor frequency (kHz)
    MAX_FREQ = 200          # fastest motor frequency (kHz)
    MAX_DISTANCE_PX = 500   # beyond this distance -> minimum speed for A
    MIN_DISTANCE_PX = 50    # closer than this     -> maximum speed for A

    def __init__(self, port_a, port_b, baud_rate=115200):
        self._motor_a = _MotorLink(port_a, baud_rate, "Motor A")
        self._motor_b = _MotorLink(port_b, baud_rate, "Motor B")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def update(self, puck_x, puck_y, puck_detected,
               mallet_x, mallet_y, mallet_detected):
        """Call once per frame with the latest tracking results."""
        # Priority 1 – mallet not in its ROI -> STOP both
        if not mallet_detected:
            self._send_both_stop()
            return

        # Priority 2 – puck not in its ROI -> reverse both at slowest
        if not puck_detected or (puck_x < 0 and puck_y < 0):
            self._send_both_freq(-self.MIN_FREQ, -self.MIN_FREQ)
            return

        # Both objects visible – inversely proportional speeds
        distance = math.hypot(puck_x - mallet_x, puck_y - mallet_y)
        freq_a = self._distance_to_freq(distance)
        freq_b = self.MIN_FREQ + self.MAX_FREQ - freq_a  # inverse
        self._send_both_freq(freq_a, freq_b)

    def stop(self):
        self._send_both_stop()

    def close(self):
        self.stop()
        self._motor_a.close()
        self._motor_b.close()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _distance_to_freq(self, distance):
        """Map pixel distance to a forward frequency in [MIN_FREQ, MAX_FREQ].

        Closer  -> higher frequency (faster)
        Farther -> lower  frequency (slower)
        """
        clamped = max(self.MIN_DISTANCE_PX, min(self.MAX_DISTANCE_PX, distance))
        t = (clamped - self.MIN_DISTANCE_PX) / (self.MAX_DISTANCE_PX - self.MIN_DISTANCE_PX)
        return int(self.MAX_FREQ - t * (self.MAX_FREQ - self.MIN_FREQ))

    def _encode(self, freq_value):
        """Encode a frequency into a single byte (uart.py protocol)."""
        mag = abs(freq_value)
        if mag == 0:
            return 0x7F
        if 100 <= mag <= 200:
            encoded = mag - 100
            if freq_value < 0:
                encoded |= 0x80
            return encoded
        return None  # out of range

    def _send_both_freq(self, freq_a, freq_b):
        byte_a = self._encode(freq_a)
        byte_b = self._encode(freq_b)
        if byte_a is not None:
            self._motor_a.write(byte_a)
        if byte_b is not None:
            self._motor_b.write(byte_b)

    def _send_both_stop(self):
        self._motor_a.write(0x7F)
        self._motor_b.write(0x7F)
