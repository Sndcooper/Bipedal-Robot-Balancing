"""
serial_link.py
Background engine handling all 3DR telemetry radio communications for mcu_pos_wireless.
"""

import threading
import time
try:
    import serial
except ImportError:
    serial = None


class SerialLink:
    def __init__(self, port="COM13", baud=115200):
        self.port     = port
        self.baud     = baud
        self.ser      = None
        self._thread  = None
        self._running = False
        self._lock    = threading.Lock()

        # Historical telemetry (Tab 1)
        self.history = {
            "t": [], "pitch": [], "pid_out": [],
            "vel": [], "integral": [],
            "tilt_bias": [], "pos_err": [], "vel_err": [],
            "pos_offset": [], "turn_bias": []
        }
        self.start_time = time.time()

        # MCU state mirror
        self.fw       = {}
        self.motors_on    = False
        self._cutoff_time = None

        # Serial monitor log
        self.raw_log = []

        # Servo health (Tab 2)
        self.servo_health = {
            6:  {"temp": 0, "load": 0.0},
            0:  {"temp": 0, "load": 0.0},
            14: {"temp": 0, "load": 0.0},
            1:  {"temp": 0, "load": 0.0},
        }

    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial not installed. Run: pip install pyserial")
        self.ser = serial.Serial(self.port, self.baud, timeout=None)
        self._running   = True
        self.start_time = time.time()
        self._thread    = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ── BACKGROUND READER ────────────────────────────────────────────────────
    def _read_loop(self):
        buf = b""
        while self._running and self.ser and self.ser.is_open:
            try:
                waiting = self.ser.in_waiting
                chunk = self.ser.read(waiting if waiting > 0 else 1)
                if not chunk:
                    continue

                buf += chunk

                for terminator in (b"|", b"\n"):
                    while terminator in buf:
                        raw_frame, buf = buf.split(terminator, 1)
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
            if len(self.raw_log) > 1000:
                self.raw_log.pop(0)

        if len(line) > 1 and line[0] == 'S' and line[1].isdigit():
            self._parse_telemetry(line)
        elif line.startswith("SRV "):
            self._parse_servo_health(line)
        elif line.startswith("Updated"):
            self._parse_fw_update(line)
        elif "SAFETY" in line:
            with self._lock:
                self._cutoff_time = time.time()
                self.motors_on    = False
        elif "Motors ENABLED" in line:
            with self._lock:
                self.motors_on    = True
                self._cutoff_time = None
        elif "Motors DISABLED" in line:
            with self._lock:
                self.motors_on = False
        elif line.startswith("CAL DONE"):
            try:
                idx = line.index("OFFSET")
                with self._lock:
                    self.fw["pitchOffset"] = float(line[idx + 6:].split()[0])
            except (ValueError, IndexError):
                pass

    def _parse_telemetry(self, line):
        data = {}
        for tok in line.split():
            j = 0
            while j < len(tok) and tok[j].isalpha():
                j += 1
            if 0 < j < len(tok):
                try:
                    data[tok[:j]] = float(tok[j:])
                except ValueError:
                    pass

        # Keep only the telemetry fields that the firmware actually transmits.
        with self._lock:
            self.history["t"].append(time.time() - self.start_time)
            self.history["pitch"].append(data.get("P", 0.0))
            self.history["pid_out"].append(data.get("O", 0.0))
            self.history["vel"].append(data.get("V", 0.0))
            self.history["integral"].append(data.get("I", 0.0))
            self.history["tilt_bias"].append(data.get("TB", 0.0))
            self.history["pos_err"].append(data.get("EP", 0.0))
            self.history["vel_err"].append(data.get("EV", 0.0))
            self.history["pos_offset"].append(data.get("PO", 0.0))
            self.history["turn_bias"].append(data.get("TR", 0.0))

            mot = data.get("M")
            if mot is not None:
                self.motors_on = bool(int(mot))

            if len(self.history["t"]) > 500:
                for k in self.history:
                    self.history[k].pop(0)

        encoder_fields = ["V", "EP", "EV", "PO", "VR"]
        if any(key in data for key in encoder_fields):
            print(
                "ENC " + " ".join(
                    f"{key}={data.get(key, 0.0):g}" for key in encoder_fields if key in data
                )
            )

    def _parse_servo_health(self, line):
        try:
            parts = line.split()
            sid  = int(parts[1])
            temp = int(parts[2])
            load = float(parts[3])
            with self._lock:
                if sid in self.servo_health:
                    self.servo_health[sid]["temp"] = temp
                    self.servo_health[sid]["load"] = load
        except Exception:
            pass

    def _parse_fw_update(self, line):
        KEY_MAP = {
            "P": "Kp", "I": "Ki", "D": "Kd",
            "VP": "Kp_vel", "VI": "Ki_vel", "VA": "vel_alpha", "PP": "Kp_pos",
            "PO": "pos_offset", "TR": "turn_bias",
            "Offset": "pitchOffset",
            "Target": "targetAngle", "Alpha": "alpha", "Tilt": "maxSafeTilt",
        }
        try:
            tokens = line.split()
            new_fw = {}
            for tok in tokens[1:]:
                j = 0
                while j < len(tok) and tok[j].isalpha():
                    j += 1
                if 0 < j < len(tok):
                    raw_key = tok[:j]
                    mapped  = KEY_MAP.get(raw_key, raw_key)
                    try:
                        new_fw[mapped] = float(tok[j:])
                    except ValueError:
                        pass
            with self._lock:
                self.fw.update(new_fw)
        except Exception:
            pass

    # ── DATA ACCESS ──────────────────────────────────────────────────────────
    def snapshot(self):
        with self._lock:
            return {k: list(v) for k, v in self.history.items()}

    def recent_lines(self, limit=100):
        with self._lock:
            return list(self.raw_log[-limit:])

    def cutoff_since(self):
        with self._lock:
            return self._cutoff_time is not None

    def get_servo_health(self):
        with self._lock:
            return {k: dict(v) for k, v in self.servo_health.items()}

    # ── COMMAND API ──────────────────────────────────────────────────────────
    def _send(self, text):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text + "|").encode("utf-8"))
            except Exception as e:
                print(f"[SerialLink] Write error: {e}")

    # PID & Balance Tuning
    def set_kp(self, val):        self._send(f"P{val}")
    def set_ki(self, val):        self._send(f"I{val}")
    def set_kd(self, val):        self._send(f"D{val}")
    def set_kp_vel(self, val):    self._send(f"VP{val}")
    def set_ki_vel(self, val):    self._send(f"VI{val}")
    def set_vel_alpha(self, val): self._send(f"VA{val}")
    def set_kp_pos(self, val):    self._send(f"PP{val}")
    def set_pos_offset(self, val):self._send(f"PO{val:.1f}")
    def set_turn_bias(self, val): self._send(f"TR{val:.1f}")
    def set_alpha(self, val):     self._send(f"A{val}")
    def set_target(self, val):    self._send(f"S{val}")
    def set_offset(self, val):    self._send(f"O{val}")
    def set_tilt(self, val):      self._send(f"T{val}")

    def calibrate(self):      self._send("C")
    def toggle_motors(self):  self._send("M")
    def reset_integral(self): self._send("R")
    def latch_home(self):     self._send("H")

    def arm_cutoff_watch(self):
        with self._lock:
            self._cutoff_time = None

    # Leg / IK controls
    def send_leg_position(self, servo_id, pos):
        self._send(f"POS {servo_id} {int(pos)}")

    def send_ik1(self, fx, fy):   self._send(f"IK1 {fx:.2f} {fy:.2f}")
    def send_ik2(self, fx, fy):   self._send(f"IK2 {fx:.2f} {fy:.2f}")
    def send_ikd(self, dist):     self._send(f"IKD {dist:.2f}")
    def send_ikl(self, lean):     self._send(f"IKL {lean:.2f}")

    def send_torque_limit(self, servo_id, limit):
        self._send(f"TRQ {servo_id} {int(limit)}")

    def send_compliance(self, servo_id, margin, slope):
        self._send(f"CMP {servo_id} {int(margin)} {int(slope)}")

    def send_torque_enable(self, state):
        self._send(f"TRQE {int(state)}")
