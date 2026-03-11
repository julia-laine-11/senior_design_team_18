# Defense Controller
# Drives motor speed based on puck proximity to mallet via UART.
# Uses the same single-byte encoding protocol as embedded/src/uart.py.
#
# Encoding:
#   freq 100-200 forward  -> byte = (freq - 100)
#   freq 100-200 reverse  -> byte = (freq - 100) | 0x80
#   stop (freq 0)         -> byte = 0x7F
#
# Behavior:
#   - Puck closer to mallet  -> motor spins faster  (freq toward 200)
#   - Puck farther from mallet -> motor spins slower (freq toward 100)
#   - Mallet leaves its ROI  -> STOP
#   - Puck leaves its ROI    -> reverse at minimum speed (-100)

import serial
import math


class DefenseController:
    """Maps vision tracking state to motor commands over UART."""

    # --- tunables ---
    MIN_FREQ = 100          # slowest motor frequency (kHz)
    MAX_FREQ = 200          # fastest motor frequency (kHz)
    MAX_DISTANCE_PX = 500   # beyond this distance -> minimum speed
    MIN_DISTANCE_PX = 50    # closer than this     -> maximum speed

    def __init__(self, port='/dev/ttyUSB1', baud_rate=115200):
        self.ser = serial.Serial(port, baud_rate)
        self._last_byte = None          # de-duplicate identical commands
        print(f"[Defense] Motor connected on {port}")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def update(self, puck_x, puck_y, puck_detected,
               mallet_x, mallet_y, mallet_detected):
        """Call once per frame with the latest tracking results.

        Parameters match the fields on TrackerState after update_tracking().
        """
        # Priority 1 – mallet not in its ROI -> STOP
        if not mallet_detected:
            self._send_stop()
            return

        # Priority 2 – puck not in its ROI -> reverse at slowest speed
        if not puck_detected or (puck_x < 0 and puck_y < 0):
            self._send_freq(-self.MIN_FREQ)
            return

        # Both objects visible – scale speed by proximity
        distance = math.hypot(puck_x - mallet_x, puck_y - mallet_y)
        freq = self._distance_to_freq(distance)
        self._send_freq(freq)

    def stop(self):
        """Convenience: immediately stop the motor."""
        self._send_stop()

    def close(self):
        """Stop the motor and release the serial port."""
        self.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("[Defense] Port closed.")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _distance_to_freq(self, distance):
        """Map pixel distance to a forward frequency in [MIN_FREQ, MAX_FREQ].

        Closer  -> higher frequency (faster motor)
        Farther -> lower  frequency (slower motor)
        """
        clamped = max(self.MIN_DISTANCE_PX, min(self.MAX_DISTANCE_PX, distance))
        # t = 0 when close, 1 when far
        t = (clamped - self.MIN_DISTANCE_PX) / (self.MAX_DISTANCE_PX - self.MIN_DISTANCE_PX)
        # invert: close -> MAX_FREQ, far -> MIN_FREQ
        return int(self.MAX_FREQ - t * (self.MAX_FREQ - self.MIN_FREQ))

    def _send_freq(self, freq_value):
        """Encode and transmit a frequency using the uart.py protocol."""
        mag = abs(freq_value)

        if mag == 0:
            encoded = 0x7F                       # OFF
        elif 100 <= mag <= 200:
            encoded = mag - 100
            if freq_value < 0:
                encoded |= 0x80                  # direction bit
        else:
            return                               # out of range – ignore

        self._write(encoded)

    def _send_stop(self):
        self._write(0x7F)

    def _write(self, byte_val):
        """Write a single byte, skipping if it matches the last command."""
        if byte_val == self._last_byte:
            return
        self.ser.write(bytes([byte_val]))
        self._last_byte = byte_val
