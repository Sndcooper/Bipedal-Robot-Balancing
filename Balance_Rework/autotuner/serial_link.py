"""
serial_link.py — Serial transport + telemetry parsing for the Balance_Rework autotuner.

Talks to the reworked STM32 firmware (Balance_Rework/firmware) over Serial1 @ 115200.

Command protocol (single letter + value, newline-terminated):
    P<kp>  I<ki>  D<kd>      PID gains
    V<kd_vel>                 encoder velocity-damping gain   (NEW)
    A<alpha>                  complementary-filter blend      (NEW)
    T<deg>                    safety tilt limit               (NEW)
    O<offset> S<target>       pitch offset / target angle
    C                          calibrate IMU (hold upright)
    M                          toggle motors on/off

Telemetry (20 Hz), comma-separated key:value — extended vs. the original firmware:
    PITCH:<v>, PID_OUT:<v>, ENC_L:<v>, ENC_R:<v>, VEL:<v>, KDVEL:<v>, ALPHA:<v>, TILT:<v>

The critical safety line we watch for:
    SAFETY CUTOFF TRIGGERED
"""

import serial
import threading
import time
from collections import deque


class SerialLink:
    """Thread-safe serial connection + background telemetry reader/parser."""

    def __init__(self, port, baud=115200, max_samples=4000):
        self.port = port
        self.baud = baud
        self.ser = None
        self._write_lock = threading.Lock()

        # --- Rolling telemetry buffers (guarded by _data_lock) ---
        self._data_lock = threading.Lock()
        self._max = max_samples
        self._clear_buffers()
        self._log = deque(maxlen=self._max)

        # --- Firmware-reported state ---
        self.motors_on = False
        self.fw = {  # last "Updated ->" echo from firmware
            "Kp": None, "Ki": None, "Kd": None,
            "Kd_vel": None, "alpha": None, "tilt": None,
            "offset": None, "target": None,
        }

        # --- Safety event flag: set True the instant the firmware prints the
        #     cutoff message. Caller inspects/clears it via cutoff_since(). ---
        self._cutoff_event = False
        self._cutoff_lock = threading.Lock()

        # --- Reader thread ---
        self._running = False
        self._thread = None

    # ------------------------------------------------------------------ #
    #  Connection lifecycle
    # ------------------------------------------------------------------ #
    def connect(self):
        print(f"[serial] opening {self.port} @ {self.baud} ...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2.0)  # STM32 reboots when the port opens
        self.ser.reset_input_buffer()
        self._running = True
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()
        print("[serial] connected, reader thread started")

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[serial] closed")

    # ------------------------------------------------------------------ #
    #  Writing commands
    # ------------------------------------------------------------------ #
    def _send(self, cmd):
        with self._write_lock:
            if self.ser and self.ser.is_open:
                self.ser.write(f"{cmd}\n".encode())
                self.ser.flush()
                time.sleep(0.03)

    def set_gains(self, kp, ki, kd, kd_vel, alpha):
        """Push all five tunable control parameters."""
        self._send(f"P{kp:.4f}")
        self._send(f"I{ki:.4f}")
        self._send(f"D{kd:.4f}")
        self._send(f"V{kd_vel:.4f}")
        self._send(f"A{alpha:.4f}")

    def set_kp(self, kp): self._send(f"P{kp:.4f}")
    def set_ki(self, ki): self._send(f"I{ki:.4f}")
    def set_kd(self, kd): self._send(f"D{kd:.4f}")
    def set_kd_vel(self, v): self._send(f"V{v:.4f}")
    def set_alpha(self, a): self._send(f"A{a:.4f}")

    def set_pid(self, kp, ki, kd):
        self._send(f"P{kp:.4f}")
        self._send(f"I{ki:.4f}")
        self._send(f"D{kd:.4f}")

    def set_tilt(self, t):     self._send(f"T{t:.2f}")
    def set_target(self, a):   self._send(f"S{a:.2f}")
    def set_offset(self, o):   self._send(f"O{o:.2f}")

    def calibrate(self):
        self._send("C")
        time.sleep(1.5)  # firmware takes 100 samples * 10 ms

    def toggle_motors(self):
        self._send("M")
        time.sleep(0.15)

    def ensure_motors(self, want_on):
        """Force motors into the desired state (idempotent)."""
        if self.motors_on != want_on:
            self.toggle_motors()
            time.sleep(0.15)
        return self.motors_on == want_on

    # ------------------------------------------------------------------ #
    #  Safety event
    # ------------------------------------------------------------------ #
    def arm_cutoff_watch(self):
        """Clear any pending cutoff flag before starting a trial."""
        with self._cutoff_lock:
            self._cutoff_event = False

    def cutoff_since(self):
        """True if the firmware printed SAFETY CUTOFF TRIGGERED since arming."""
        with self._cutoff_lock:
            return self._cutoff_event

    # ------------------------------------------------------------------ #
    #  Telemetry access
    # ------------------------------------------------------------------ #
    def _clear_buffers(self):
        self.t = deque(maxlen=self._max)
        self.pitch = deque(maxlen=self._max)
        self.pid_out = deque(maxlen=self._max)
        self.enc_l = deque(maxlen=self._max)
        self.enc_r = deque(maxlen=self._max)
        self.vel = deque(maxlen=self._max)

    def clear(self):
        with self._data_lock:
            self._clear_buffers()

    def clear_log(self):
        with self._data_lock:
            self._log.clear()

    def recent_lines(self, limit=200):
        with self._data_lock:
            if limit is None or limit >= len(self._log):
                return list(self._log)
            return list(self._log)[-limit:]

    def snapshot(self):
        """Return a dict of lists (copy) of everything currently buffered."""
        with self._data_lock:
            return {
                "t": list(self.t),
                "pitch": list(self.pitch),
                "pid_out": list(self.pid_out),
                "enc_l": list(self.enc_l),
                "enc_r": list(self.enc_r),
                "vel": list(self.vel),
            }

    @property
    def latest_pitch(self):
        with self._data_lock:
            return self.pitch[-1] if self.pitch else 0.0

    @property
    def sample_count(self):
        with self._data_lock:
            return len(self.pitch)

    # ------------------------------------------------------------------ #
    #  Background reader
    # ------------------------------------------------------------------ #
    def _reader_loop(self):
        while self._running:
            try:
                raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception:
                continue
            if raw:
                self._parse(raw)

    def _parse(self, line):
        with self._data_lock:
            self._log.append(line)

        # --- Safety cutoff is the highest-priority line to catch ---
        if "SAFETY CUTOFF TRIGGERED" in line:
            with self._cutoff_lock:
                self._cutoff_event = True
            self.motors_on = False
            return

        if line.startswith("PITCH:"):
            self._parse_telemetry(line)
            return

        if "Motors ENABLED" in line:
            self.motors_on = True
            return
        if "Motors DISABLED" in line:
            self.motors_on = False
            return

        if line.startswith("Updated ->"):
            self._parse_updated(line)
            return

        if "Calibration complete" in line:
            try:
                self.fw["offset"] = float(line.split(":")[-1].strip())
            except ValueError:
                pass
            return

    def _parse_telemetry(self, line):
        """Robust key:value comma parse; tolerates extra/added fields."""
        kv = {}
        for part in line.split(","):
            part = part.strip()
            if ":" in part:
                k, v = part.split(":", 1)
                kv[k.strip().upper()] = v.strip()
        try:
            pitch = float(kv["PITCH"])
            pid_out = float(kv.get("PID_OUT", "0"))
            enc_l = int(float(kv.get("ENC_L", "0")))
            enc_r = int(float(kv.get("ENC_R", "0")))
            vel = float(kv.get("VEL", "0"))
        except (KeyError, ValueError):
            return
        with self._data_lock:
            self.t.append(time.time())
            self.pitch.append(pitch)
            self.pid_out.append(pid_out)
            self.enc_l.append(enc_l)
            self.enc_r.append(enc_r)
            self.vel.append(vel)

    def _parse_updated(self, line):
        # "Updated -> P:.. I:.. D:.. Offset:.. Target:.. Vel:.. Alpha:.. Tilt:.."
        mapping = {
            "P": "Kp", "I": "Ki", "D": "Kd",
            "OFFSET": "offset", "TARGET": "target",
            "VEL": "Kd_vel", "ALPHA": "alpha", "TILT": "tilt",
        }
        for tok in line.replace("Updated ->", "").split():
            if ":" in tok:
                k, v = tok.split(":", 1)
                key = mapping.get(k.strip().upper())
                if key:
                    try:
                        self.fw[key] = float(v)
                    except ValueError:
                        pass
