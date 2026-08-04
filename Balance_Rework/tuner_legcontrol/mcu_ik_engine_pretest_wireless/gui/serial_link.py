"""
serial_link.py
Background engine handling all 3DR telemetry radio communications.
"""

import threading
import time
try:
    import serial
except ImportError:
    serial = None


class SerialLink:
    def __init__(self, port="COM3", baud=115200):
        self.port     = port
        self.baud     = baud
        self.ser      = None
        self._thread  = None
        self._running = False
        self._lock    = threading.Lock()

        # Historical telemetry (Tab 1)
        self.history = {
            "t": [], "pitch": [], "pid_out": [],
            "vel": [], "enc_l": [], "enc_r": [], "integral": []
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
        # timeout=None: blocking read — the accumulator controls all line assembly.
        # Do NOT use readline() with a short timeout on a radio link; it returns
        # partial lines when the radio pauses between chunks.
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
        """Byte-accumulator loop — never returns a partial line.

        The 3DR radio sends data in chunks that may not align with newlines.
        Using readline() with a timeout causes partial lines to be returned
        when the radio pauses between chunks (e.g. 'PITCH:,PID_OUT:,' with
        empty values). Instead, we accumulate raw bytes and only dispatch a
        line once the terminating '\n' has arrived.
        """
        buf = b""
        while self._running and self.ser and self.ser.is_open:
            try:
                # Block until at least 1 byte is available (timeout=None on port).
                # Read all currently-available bytes in one syscall to minimise
                # loop overhead, but always read at least 1.
                waiting = self.ser.in_waiting
                chunk = self.ser.read(waiting if waiting > 0 else 1)
                if not chunk:
                    continue

                buf += chunk

                # Dispatch every complete line in the buffer.
                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="ignore").strip()
                    if line:
                        self._process_line(line)

            except Exception as e:
                print(f"[SerialLink] Read error: {e}")
                buf = b""          # discard corrupted buffer on error
                time.sleep(0.05)

    def _process_line(self, line: str):
        """Route a complete, stripped line to the correct parser."""
        with self._lock:
            self.raw_log.append(line)
            if len(self.raw_log) > 1000:
                self.raw_log.pop(0)

        if line.startswith("PITCH:"):
            self._parse_telemetry(line)
        elif line.startswith("SRV:"):
            self._parse_servo_health(line)
        elif line.startswith("Updated ->"):
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
        elif line.startswith("CAL:DONE"):
            try:
                offset_str = line.split("OFFSET:")[1]
                with self._lock:
                    self.fw["pitchOffset"] = float(offset_str)
            except (IndexError, ValueError):
                pass

    def _parse_telemetry(self, line):
        """Parses: PITCH:1.23,PID_OUT:-4.5,INT:0.01,EL:100,ER:105,..."""
        data = {}
        for part in line.split(","):
            if ":" in part:
                k, v = part.split(":", 1)
                try:
                    data[k.strip()] = float(v.strip())
                except ValueError:
                    pass

        with self._lock:
            self.history["t"].append(time.time() - self.start_time)
            self.history["pitch"].append(data.get("PITCH", 0.0))
            self.history["pid_out"].append(data.get("PID_OUT", 0.0))
            self.history["vel"].append(data.get("VEL", 0.0))
            self.history["enc_l"].append(data.get("EL", 0.0))
            self.history["enc_r"].append(data.get("ER", 0.0))
            self.history["integral"].append(data.get("INT", 0.0))

            # Sync motor/latch state from embedded flags
            if "MOT" in data:
                self.motors_on = bool(int(data["MOT"]))

            # Cap history to 500 samples
            if len(self.history["t"]) > 500:
                for k in self.history:
                    self.history[k].pop(0)

    def _parse_servo_health(self, line):
        """Parses: SRV:<id>,<temp>,<load%>  e.g. SRV:6,45,12.5"""
        try:
            _, payload = line.split(":", 1)
            sid_s, temp_s, load_s = payload.split(",")
            sid = int(sid_s.strip())
            with self._lock:
                if sid in self.servo_health:
                    self.servo_health[sid]["temp"] = int(temp_s.strip())
                    self.servo_health[sid]["load"] = float(load_s.strip())
        except Exception:
            pass

    def _parse_fw_update(self, line):
        """Parses: Updated -> P:11.2 I:0.0 D:0.0 Offset:0.0 Target:0.0 Alpha:0.96 STR:0.0 Tilt:25.0"""
        KEY_MAP = {
            "P": "Kp", "I": "Ki", "D": "Kd",
            "STR": "Kp_straight", "Offset": "pitchOffset",
            "Target": "targetAngle", "Alpha": "alpha", "Tilt": "maxSafeTilt",
        }
        try:
            _, payload = line.split("->", 1)
            new_fw = {}
            for part in payload.split():
                if ":" in part:
                    k, v = part.split(":", 1)
                    mapped = KEY_MAP.get(k.strip(), k.strip())
                    new_fw[mapped] = float(v.strip())
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

    # ── COMMAND API ───────────────────────────────────────────────────────────
    def _send(self, text):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text + "\n").encode("utf-8"))
            except Exception as e:
                print(f"[SerialLink] Write error: {e}")

    # PID & balance tuning
    def set_kp(self, val):          self._send(f"P{val}")
    def set_ki(self, val):          self._send(f"I{val}")
    def set_kd(self, val):          self._send(f"D{val}")
    def set_kp_straight(self, val): self._send(f"STR{val}")
    def set_alpha(self, val):       self._send(f"A{val}")
    def set_target(self, val):      self._send(f"S{val}")
    def set_offset(self, val):      self._send(f"O{val}")
    def set_tilt(self, val):        self._send(f"T{val}")

    def calibrate(self):      self._send("C")
    def toggle_motors(self):  self._send("M")
    def reset_integral(self): self._send("R")

    def arm_cutoff_watch(self):
        with self._lock:
            self._cutoff_time = None

    # Leg / IK controls
    def send_leg_position(self, servo_id, pos):
        self._send(f"POS,{servo_id},{int(pos)}")

    def send_ik1(self, fx, fy):   self._send(f"IK1,{fx:.2f},{fy:.2f}")
    def send_ik2(self, fx, fy):   self._send(f"IK2,{fx:.2f},{fy:.2f}")
    def send_ikd(self, dist):     self._send(f"IKD,{dist:.2f}")
    def send_ikl(self, lean):     self._send(f"IKL,{lean:.2f}")

    def send_torque_limit(self, servo_id, limit):
        self._send(f"TRQ,{servo_id},{int(limit)}")

    def send_compliance(self, servo_id, margin, slope):
        self._send(f"CMP,{servo_id},{int(margin)},{int(slope)}")