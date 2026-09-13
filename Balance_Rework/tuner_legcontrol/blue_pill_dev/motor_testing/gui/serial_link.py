"""
serial_link.py
Background engine for motor_testing wiring-check GUI (blue_pill_dev).
Wired USB link to Serial1 on the Blue Pill, 115200 baud.
"""

import threading
import time
try:
    import serial
except ImportError:
    serial = None


class SerialLink:
    def __init__(self, port="COM13", baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()

        self.encoder_left = 0
        self.encoder_right = 0
        self.raw_log = []

    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial not installed. Run: pip install pyserial")
        self.ser = serial.Serial(self.port, self.baud, timeout=None)
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _read_loop(self):
        buf = b""
        while self._running and self.ser and self.ser.is_open:
            try:
                waiting = self.ser.in_waiting
                chunk = self.ser.read(waiting if waiting > 0 else 1)
                if not chunk:
                    continue
                buf += chunk
                while b"|" in buf:
                    raw_frame, buf = buf.split(b"|", 1)
                    line = raw_frame.decode("utf-8", errors="ignore").strip()
                    if line:
                        self._process_line(line)
            except Exception as e:
                print(f"[SerialLink] Read error: {e}")
                buf = b""
                time.sleep(0.05)

    def _process_line(self, line: str):
        with self._lock:
            self.raw_log.append(line)
            if len(self.raw_log) > 500:
                self.raw_log.pop(0)

        if line.startswith("EL"):
            try:
                el_str, er_str = line.split()
                with self._lock:
                    self.encoder_left = int(el_str[2:])
                    self.encoder_right = int(er_str[2:])
            except (ValueError, IndexError):
                pass

    def snapshot(self):
        with self._lock:
            return self.encoder_left, self.encoder_right

    def recent_lines(self, limit=50):
        with self._lock:
            return list(self.raw_log[-limit:])

    # ── COMMAND API ──────────────────────────────────────────────────────────
    def _send(self, text):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text + "|").encode("utf-8"))
            except Exception as e:
                print(f"[SerialLink] Write error: {e}")

    def forward(self):    self._send("F")
    def backward(self):   self._send("B")
    def turn_left(self):  self._send("L")
    def turn_right(self): self._send("R")
    def stop(self):       self._send("S")
    def set_pwm(self, val): self._send(f"SP{int(val)}")
