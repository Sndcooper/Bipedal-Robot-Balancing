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
            "vel": [], "enc_l": [], "enc_r": [], "integral": [],
            "tilt_bias": [], "pos_err": [], "vel_err": [], "cascade_state": []
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
        """Byte-accumulator loop — never returns a partial frame.

        The 3DR radio sends data in chunks that may not align with the pipe
        terminator. Using readline() with a timeout causes partial lines to be
        returned when the radio pauses between chunks. Instead, we accumulate
        raw bytes and only dispatch a frame once the terminating '|' has arrived.
        '\n' is also accepted as a fallback terminator for plain serial-monitor
        use during development.
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

                # Dispatch every complete pipe-terminated frame in the buffer.
                # Also split on \n for backward-compat with plain serial monitors.
                for terminator in (b"|", b"\n"):
                    while terminator in buf:
                        raw_frame, buf = buf.split(terminator, 1)
                        line = raw_frame.decode("utf-8", errors="ignore").strip()
                        if line:
                            self._process_line(line)

            except Exception as e:
                print(f"[SerialLink] Read error: {e}")
                buf = b""          # discard corrupted buffer on error
                time.sleep(0.05)

    def _process_line(self, line: str):
        """Route a complete, stripped frame to the correct parser.

        New compact protocol (pipe-terminated, space-delimited, no colons):
          Telemetry : "S42 DT10012 P1.23 O-4.5 I0.0001 V12.3 A0.96 T25.0 M1 L0"
          Servo hlth: "SRV <id> <temp> <load%>"
          FW ack    : "Updated P11.200 I670.000 D1.900 ..."
          One-shots : "BOOT OK", "SAFETY CUTOFF", "Motors ENABLED", "CAL DONE OFFSET..."
        """
        with self._lock:
            self.raw_log.append(line)
            if len(self.raw_log) > 1000:
                self.raw_log.pop(0)

        # Telemetry frame: starts with 'S' followed by a digit (sequence number)
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
            # Format: "CAL DONE OFFSET-1.2345"
            try:
                idx = line.index("OFFSET")
                with self._lock:
                    self.fw["pitchOffset"] = float(line[idx + 6:])
            except (ValueError, IndexError):
                pass

    def _parse_telemetry(self, line):
        """Parses compact space-delimited telemetry frame.

        Format: "S42 DT10012 P1.23 O-4.5 I0.0001 V12.3 A0.96 T25.0 M1 L0"
        Each token is an alpha prefix immediately followed by a numeric value,
        e.g. "P1.23" → key="P", value=1.23.
        """
        data = {}
        for tok in line.split():
            # Find where the alpha prefix ends and the numeric value begins.
            # Handles signed values: prefix is all alpha chars at the start.
            j = 0
            while j < len(tok) and tok[j].isalpha():
                j += 1
            if 0 < j < len(tok):
                try:
                    data[tok[:j]] = float(tok[j:])
                except ValueError:
                    pass

        with self._lock:
            self.history["t"].append(time.time() - self.start_time)
            self.history["pitch"].append(data.get("P", 0.0))
            self.history["pid_out"].append(data.get("O", 0.0))
            self.history["vel"].append(data.get("V", 0.0))
            self.history["enc_l"].append(data.get("EL", 0.0))
            self.history["enc_r"].append(data.get("ER", 0.0))
            self.history["integral"].append(data.get("I", 0.0))
            self.history["tilt_bias"].append(data.get("TB", 0.0))
            self.history["pos_err"].append(data.get("EP", 0.0))
            self.history["vel_err"].append(data.get("EV", 0.0))
            self.history["cascade_state"].append(data.get("ST", 2.0))  # default HOLDING

            # Sync motor/latch state from embedded flags
            mot = data.get("M")
            if mot is not None:
                self.motors_on = bool(int(mot))

            # Cap history to 500 samples
            if len(self.history["t"]) > 500:
                for k in self.history:
                    self.history[k].pop(0)

    def _parse_servo_health(self, line):
        """Parses compact space-delimited servo health frame.

        Format: "SRV <id> <temp> <load%>"  e.g. "SRV 6 45 12.5"
        """
        try:
            parts = line.split()
            # parts[0] == "SRV", parts[1] == id, parts[2] == temp, parts[3] == load
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
        """Parses compact space-delimited PID ack frame.

        Format: "Updated P11.200 I670.000 D1.900 Offset0.0000 Target0.000 Alpha0.9600 STR0.0000 Tilt25.00"
        Each token after "Updated" is an alpha-prefix key immediately followed
        by the numeric value, mirroring the telemetry token format.
        """
        KEY_MAP = {
            "P": "Kp", "I": "Ki", "D": "Kd",
            "VP": "Kp_vel", "VI": "Ki_vel", "VA": "vel_alpha", "PP": "Kp_pos",
            "Offset": "pitchOffset",
            "Target": "targetAngle", "Alpha": "alpha", "Tilt": "maxSafeTilt",
        }
        try:
            tokens = line.split()
            # tokens[0] == "Updated", remaining are key-value tokens
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

    # ── COMMAND API ───────────────────────────────────────────────────────────────────
    def _send(self, text):
        """Transmit a pipe-terminated command frame over the 3DR radio.

        The pipe character '|' is the canonical frame terminator for the new
        compact protocol.  STM32 parseCommand() also accepts '\n'/'\r' as
        a fallback, so plain serial-monitor usage still works.
        """
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text + "|").encode("utf-8"))
            except Exception as e:
                print(f"[SerialLink] Write error: {e}")

    # PID & balance tuning — single-char prefix, value appended, no delimiter
    def set_kp(self, val):      self._send(f"P{val}")
    def set_ki(self, val):      self._send(f"I{val}")
    def set_kd(self, val):      self._send(f"D{val}")
    def set_kp_vel(self, val):  self._send(f"VP{val}")
    def set_ki_vel(self, val):  self._send(f"VI{val}")
    def set_vel_alpha(self, val): self._send(f"VA{val}")
    def set_kp_pos(self, val):  self._send(f"PP{val}")
    def set_alpha(self, val):   self._send(f"A{val}")
    def set_target(self, val):  self._send(f"S{val}")
    def set_offset(self, val):  self._send(f"O{val}")
    def set_tilt(self, val):    self._send(f"T{val}")

    def calibrate(self):      self._send("C")
    def toggle_motors(self):  self._send("M")
    def reset_integral(self): self._send("R")

    def arm_cutoff_watch(self):
        with self._lock:
            self._cutoff_time = None

    # Leg / IK controls — space-delimited, no commas
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