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
            "vel": [], "enc_l": [], "enc_r": [], "integral": [], "trim": []
        }
        self.start_time = time.time()

        # MCU state mirror
        self.fw       = {}
        self.motors_on    = False
        self.auto_trim_on = False
        self._cutoff_time = None
        self._last_trim_commit = None   # (committed_deg, new_target, time.time())

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
        elif line.startswith("ACK:AUTOTRIM_"):
            with self._lock:
                self.auto_trim_on = line.endswith("ON")
        elif line.startswith("TRIM:DONE"):
            # "TRIM:DONE COMMITTED:0.812 TARGET:1.234"
            try:
                parts = dict(p.split(":", 1) for p in line.split()[1:])
                committed = float(parts["COMMITTED"])
                new_target = float(parts["TARGET"])
                with self._lock:
                    self.fw["targetAngle"]  = new_target
                    self._last_trim_commit  = (committed, new_target, time.time())
            except (KeyError, ValueError):
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
            self.history["trim"].append(data.get("TRIM", 0.0))

            # Sync motor/latch/auto-trim state from embedded flags
            if "MOT" in data:
                self.motors_on = bool(int(data["MOT"]))
            if "ATE" in data:
                self.auto_trim_on = bool(int(data["ATE"]))

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
        """Parses: Updated -> P:11.2 I:0.0 D:0.0 Offset:0.0 Target:0.0 Alpha:0.96 Tilt:25.0"""
        KEY_MAP = {
            "P": "Kp", "I": "Ki", "D": "Kd",
            "Offset": "pitchOffset",
            "Target": "targetAngle", "Alpha": "alpha", "Tilt": "maxSafeTilt",
            "TrimGain": "Ki_trim", "Crouch": "crouchOffset",
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

    # PID & balance tuning (single control loop)
    def set_kp(self, val):     self._send(f"P{val}")
    def set_ki(self, val):     self._send(f"I{val}")
    def set_kd(self, val):     self._send(f"D{val}")
    def set_alpha(self, val):  self._send(f"A{val}")
    def set_target(self, val): self._send(f"S{val}")
    def set_offset(self, val): self._send(f"O{val}")
    def set_tilt(self, val):   self._send(f"T{val}")

    # Auto-trim (drift-cancelling bias) — see firmware AUTO-TRIM block
    def set_trim_gain(self, val):      self._send(f"TG{val}")
    def set_auto_trim(self, enabled):  self._send(f"TE{1 if enabled else 0}")
    def commit_trim(self):             self._send("TC")

    # Crouch bar — stands the legs tall or crouches them, independent of
    # motorsEnabled (works with the wheel motors disarmed on the bench).
    def set_crouch(self, val):         self._send(f"CR{val}")

    # Raw per-servo position — bypasses the crouch IK entirely, moves exactly
    # one AX-12 joint. servo_id in {6, 0, 14, 1}; pos is a raw AX-12 unit (0-1023).
    def set_servo_position(self, servo_id, pos):
        self._send(f"PS{int(servo_id)} {int(pos)}")

    def calibrate(self):      self._send("C")
    def toggle_motors(self):  self._send("M")
    def reset_integral(self): self._send("R")

    def arm_cutoff_watch(self):
        with self._lock:
            self._cutoff_time = None