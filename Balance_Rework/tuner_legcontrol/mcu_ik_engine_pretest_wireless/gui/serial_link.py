"""
serial_link.py
Background engine handling all 3DR telemetry radio communications.
Extended for the unified firmware: parses both the single-loop balance PITCH
telemetry and the new AX12 leg-subsystem state/health lines.
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

        # Historical telemetry (Tab 1 — balance loop)
        self.history = {
            "t": [], "pitch": [], "pid_out": [],
            "vel": [], "enc_l": [], "enc_r": [], "integral": [], "trim": []
        }
        self.start_time = time.time()

        # MCU state mirror — balance loop
        self.fw       = {}
        self.motors_on    = False
        self.auto_trim_on = False
        self._cutoff_time = None
        self._last_trim_commit = None   # (committed_deg, new_target, time.time())

        # MCU state mirror — AX-12 leg subsystem
        self.ax12 = {
            "mode":         0,       # 0=CROUCH, 1=IK
            "torque_on":    True,
            "torque_limit": 1023,
            "comp_margin":  1,
            "comp_slope":   4,
            "moving_speed": 0,
            "move_time_ms": 800,
            "crouch":       0.0,
            "fx1": 1.0,   "fy1": -151.1,
            "fx2": -6.0,  "fy2": -149.6,
            "ik1_valid":    True,
            "ik2_valid":    True,
            "move_active":  False,
        }

        # Serial monitor log
        self.raw_log = []

        # Servo health (Tab 3)
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
        # Request a full state resync so the GUI sliders populate immediately
        self._send("RB")

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ── BACKGROUND READER ────────────────────────────────────────────────────
    def _read_loop(self):
        """Byte-accumulator loop — never returns a partial line."""
        buf = b""
        while self._running and self.ser and self.ser.is_open:
            try:
                waiting = self.ser.in_waiting
                chunk = self.ser.read(waiting if waiting > 0 else 1)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="ignore").strip(" \t\r\n\x00")
                    if line:
                        self._process_line(line)
            except Exception as e:
                print(f"[SerialLink] Read error: {e}")
                buf = b""
                time.sleep(0.05)

    def _process_line(self, line: str):
        """Route a complete, stripped line to the correct parser."""
        with self._lock:
            self.raw_log.append(line)
            if len(self.raw_log) > 1000:
                self.raw_log.pop(0)

        for prefix in ("PITCH:", "AX12:", "SRV:", "Updated ->", "BOOT:", "CAL:",
                       "ACK:", "TRIM:", "SAFETY:", "Motors ", "FT:", "FA:"):
            idx = line.find(prefix)
            if idx != -1:
                line = line[idx:]
                break

        if line.startswith("PITCH:"):
            self._parse_telemetry(line)
        elif line.startswith("AX12:"):
            self._parse_ax12_state(line)
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
        elif line.startswith("ACK:TORQUE_"):
            with self._lock:
                self.ax12["torque_on"] = line.endswith("ON")
        elif line.startswith("TRIM:DONE"):
            try:
                parts = dict(p.split(":", 1) for p in line.split()[1:])
                committed  = float(parts["COMMITTED"])
                new_target = float(parts["TARGET"])
                with self._lock:
                    self.fw["targetAngle"]  = new_target
                    self._last_trim_commit  = (committed, new_target, time.time())
            except (KeyError, ValueError):
                pass

    def _parse_telemetry(self, line):
        """Parses: PITCH:1.23,PID_OUT:-4.5,INT:0.01,EL:100,...
        Also carries leg subsystem fields: TORQ/CR/FX1/FY1/FX2/FY2/IK1/IK2/MOVE."""
        data = {}
        for part in line.split(","):
            if ":" in part:
                k, v = part.split(":", 1)
                val_str = v.strip()
                if val_str:
                    try:
                        data[k.strip()] = float(val_str)
                    except ValueError:
                        pass

        with self._lock:
            self.history["t"].append(time.time() - self.start_time)
            self.history["pitch"].append(data.get("PITCH",   0.0))
            self.history["pid_out"].append(data.get("PID_OUT", 0.0))
            self.history["vel"].append(data.get("VEL",     0.0))
            self.history["enc_l"].append(data.get("EL",      0.0))
            self.history["enc_r"].append(data.get("ER",      0.0))
            self.history["integral"].append(data.get("INT",     0.0))
            self.history["trim"].append(data.get("TRIM",    0.0))

            if "MOT" in data:
                self.motors_on = bool(int(data["MOT"]))
            if "ATE" in data:
                self.auto_trim_on = bool(int(data["ATE"]))

            # AX-12 leg subsystem fields (extended telemetry)
            if "TORQ" in data:
                self.ax12["torque_on"]   = bool(int(data["TORQ"]))
            if "CR"   in data:
                self.ax12["crouch"]      = data["CR"]
            if "FX1"  in data:
                self.ax12["fx1"]         = data["FX1"]
            if "FY1"  in data:
                self.ax12["fy1"]         = data["FY1"]
            if "FX2"  in data:
                self.ax12["fx2"]         = data["FX2"]
            if "FY2"  in data:
                self.ax12["fy2"]         = data["FY2"]
            if "IK1"  in data:
                self.ax12["ik1_valid"]   = bool(int(data["IK1"]))
            if "IK2"  in data:
                self.ax12["ik2_valid"]   = bool(int(data["IK2"]))
            if "MOVE" in data:
                self.ax12["move_active"] = bool(int(data["MOVE"]))

            if len(self.history["t"]) > 500:
                for k in self.history:
                    self.history[k].pop(0)

    def _parse_ax12_state(self, line):
        """Parses: AX12:MODE:0,TQ:1,TL:1023,CM:1,CS:4,MS:0,MT:800"""
        payload = line[5:] if line.startswith("AX12:") else line
        data = {}
        for part in payload.split(","):
            if ":" in part:
                k, v = part.split(":", 1)
                try:
                    data[k.strip()] = float(v.strip())
                except ValueError:
                    pass
        with self._lock:
            if "MODE" in data:
                self.ax12["mode"]          = int(data["MODE"])
            if "TQ"   in data:
                self.ax12["torque_on"]     = bool(int(data["TQ"]))
            if "TL"   in data:
                self.ax12["torque_limit"]  = int(data["TL"])
            if "CM"   in data:
                self.ax12["comp_margin"]   = int(data["CM"])
            if "CS"   in data:
                self.ax12["comp_slope"]    = int(data["CS"])
            if "MS"   in data:
                self.ax12["moving_speed"]  = int(data["MS"])
            if "MT"   in data:
                self.ax12["move_time_ms"]  = int(data["MT"])
            if "CR"   in data:
                self.ax12["crouch"]        = data["CR"]
            if "FX1"  in data:
                self.ax12["fx1"]           = data["FX1"]
            if "FY1"  in data:
                self.ax12["fy1"]           = data["FY1"]
            if "FX2"  in data:
                self.ax12["fx2"]           = data["FX2"]
            if "FY2"  in data:
                self.ax12["fy2"]           = data["FY2"]
            if "IK1"  in data:
                self.ax12["ik1_valid"]     = bool(int(data["IK1"]))
            if "IK2"  in data:
                self.ax12["ik2_valid"]     = bool(int(data["IK2"]))
            if "MOVE" in data:
                self.ax12["move_active"]   = bool(int(data["MOVE"]))

    def _parse_servo_health(self, line):
        """Parses: SRV:<id>,<temp>,<load%>  e.g. SRV:6,45,12.5"""
        try:
            _, payload = line.split(":", 1)
            parts = payload.split(",")
            if len(parts) >= 2:
                sid  = int(parts[0].strip())
                temp = int(parts[1].strip())
                load_str = parts[2].strip() if len(parts) > 2 else ""
                try:
                    load = float(load_str) if load_str else 0.0
                except ValueError:
                    load = 0.0
                with self._lock:
                    if sid in self.servo_health:
                        self.servo_health[sid]["temp"] = temp
                        self.servo_health[sid]["load"] = load
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
                    val_str = v.strip()
                    if val_str:
                        try:
                            mapped = KEY_MAP.get(k.strip(), k.strip())
                            new_fw[mapped] = float(val_str)
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

    def get_ax12_state(self):
        with self._lock:
            return dict(self.ax12)

    # ── COMMAND API — balance loop ────────────────────────────────────────────
    def _send(self, text):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text + "\n").encode("utf-8"))
            except Exception as e:
                print(f"[SerialLink] Write error: {e}")

    def set_kp(self, val):     self._send(f"P{val}")
    def set_ki(self, val):     self._send(f"I{val}")
    def set_kd(self, val):     self._send(f"D{val}")
    def set_alpha(self, val):  self._send(f"A{val}")
    def set_target(self, val): self._send(f"S{val}")
    def set_offset(self, val): self._send(f"O{val}")
    def set_tilt(self, val):   self._send(f"T{val}")

    def set_trim_gain(self, val):      self._send(f"TG{val}")
    def set_auto_trim(self, enabled):  self._send(f"TE{1 if enabled else 0}")
    def commit_trim(self):             self._send("TC")

    def set_crouch(self, val):         self._send(f"CR{val}")
    def set_servo_position(self, servo_id, pos):
        self._send(f"PS{int(servo_id)} {int(pos)}")

    def calibrate(self):      self._send("C")
    def toggle_motors(self):  self._send("M")
    def reset_integral(self): self._send("R")

    def arm_cutoff_watch(self):
        with self._lock:
            self._cutoff_time = None

    # ── COMMAND API — AX-12 leg subsystem ────────────────────────────────────
    def set_torque(self, enabled: bool):
        """TQ0 = limp (legs go compliant), TQ1 = re-grip with FK re-seed."""
        self._send(f"TQ{1 if enabled else 0}")

    def set_torque_limit(self, val: int):
        self._send(f"TL{int(val)}")

    def set_comp_margin(self, val: int):
        self._send(f"CM{int(val)}")

    def set_comp_slope(self, val: int):
        self._send(f"CS{int(val)}")

    def set_moving_speed(self, val: int):
        self._send(f"MS{int(val)}")

    def set_move_time(self, val: int):
        self._send(f"MT{int(val)}")

    def set_foot_target(self, leg: int, x: float, y: float):
        """leg=1 or 2; x,y in mm. Triggers an interpolated move."""
        self._send(f"FT{leg} {x:.2f} {y:.2f}")

    def set_both_feet(self, x1: float, y1: float, x2: float, y2: float):
        """Send both leg targets atomically — single command avoids 2-FT stagger."""
        self._send(f"FA {x1:.2f} {y1:.2f} {x2:.2f} {y2:.2f}")

    def home_pose(self):
        self._send("HM")

    def servo_reset(self):
        """Re-init bus registers and snap to standing pose."""
        self._send("SR")

    def set_pose_mode(self, ik_mode: bool):
        """MD0=CROUCH, MD1=IK. Firmware seeds the new mode from the current pose."""
        self._send(f"MD{1 if ik_mode else 0}")

    def request_state(self):
        """Ask firmware to broadcast a full AX12 state line (GUI resync)."""
        self._send("RB")
