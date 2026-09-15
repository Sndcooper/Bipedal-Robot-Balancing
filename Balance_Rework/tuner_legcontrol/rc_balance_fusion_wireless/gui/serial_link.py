"""
serial_link.py
Background engine handling all 3DR telemetry radio communications.

Matched to rc_balance_fusion_wireless/firmware. Parses the balance PITCH
telemetry, the AX12 leg-subsystem state/health lines, and the RC receiver
state (link/arm/stick values + the raw channel line).

PROTOCOL NOTE — this file is NOT interchangeable with the other variants'
serial_link.py. This family uses "key:value" pairs, comma-separated, with a
"\n" terminator. RC_mcu_IK_wireless uses "key<value>" space-separated with a
"|" terminator. Copying a send method between the two fails silently.
"""

import threading
import time
import os
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

        # Serial Monitor Backup Logger (COM13 / active port)
        self.auto_backup_enabled    = True
        self.backup_active          = False
        self.backup_file            = None
        self.backup_filepath        = ""
        self.backup_filename        = ""
        self.backup_lines_count     = 0
        self.last_cal_offset        = None
        self.last_cal_time          = None
        self.last_backup_saved_path = ""
        self._backup_start_time     = 0.0
        self._log_file_lock         = threading.Lock()

        # MCU state mirror — AX-12 leg subsystem
        self.ax12 = {
            "mode":         0,       # 0=CROUCH, 1=IK
            "torque_on":    True,
            "torque_limit": 1023,
            "comp_margin":  1,
            "comp_slope":   4,
            "moving_speed": 0,
            "move_time_ms": 800,
            "crouch_l":     0.0,
            "crouch_r":     0.0,
            "fx1": 1.0,   "fy1": -151.1,
            "fx2": -6.0,  "fy2": -149.6,
            "ik1_valid":    True,
            "ik2_valid":    True,
            "move_active":  False,
        }

        # MCU state mirror — RC receiver (FlySky iBUS)
        # link_ok mirrors the firmware failsafe: False means no valid iBUS
        # frames for RC_TIMEOUT_MS, which disarms the motors on the MCU side.
        # rc_enabled is the RE0/RE1 authority switch, NOT the link state.
        self.rc = {
            "link_ok":   False,
            "rc_enabled": True,
            "armed":     False,
            "drive":     0.0,    # -1..+1 decoded drive stick
            "steer":     0.0,    # -1..+1 decoded steer stick
            "target_vel": 0.0,   # counts/sec commanded to the outer loop
            "target_ang": 0.0,   # targetAngle in deg (shared with the GUI bar)
            "ch8_raw":    0,     # raw Ch8 width, to spot a resting offset
            "channels":  [0] * 10,  # raw microseconds, Ch1..Ch10
            "max_crouch_rate": 3.0,
            "idle_states": {},   # RCCEN IDLE8/IDLE10, boot-resting switch positions
            "max_vel":    400.0,
            "max_steer":  40.0,
            "max_crouch": 40.0,
        }

        # Serial monitor log
        self.raw_log = []

        # Servo health (Tab 3)
        # err  = AX-12 status ERROR byte (bit2 overheat, bit5 overload, ...)
        # fail = consecutive reads with no valid reply. Either being non-zero
        #        means the servo is NOT obeying goal writes, which previously
        #        showed up only as a temperature that quietly stopped changing.
        self.servo_health = {
            6:  {"temp": 0, "load": 0.0, "err": 0, "fail": 0},
            0:  {"temp": 0, "load": 0.0, "err": 0, "fail": 0},
            14: {"temp": 0, "load": 0.0, "err": 0, "fail": 0},
            1:  {"temp": 0, "load": 0.0, "err": 0, "fail": 0},
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
        if self.backup_active:
            self.stop_backup_session(reason="Port Disconnected / Closed")
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ── MOTOR STATE & BACKUP SESSION TRIGGERS ────────────────────────────────
    def _set_motors_on(self, enabled: bool, reason: str = ""):
        with self._lock:
            prev = self.motors_on
            self.motors_on = enabled
            if enabled:
                self._cutoff_time = None
            elif "SAFETY" in reason:
                self._cutoff_time = time.time()

        if enabled and not prev:
            if self.auto_backup_enabled:
                self.start_backup_session(reason=reason or "Motors ENABLED")
        elif not enabled and prev:
            if self.backup_active:
                self.stop_backup_session(reason=reason or "Motors DISABLED")

    def start_backup_session(self, reason="Motors ON"):
        with self._log_file_lock:
            if self.backup_active and self.backup_file:
                return  # already recording
            try:
                gui_dir = os.path.dirname(os.path.abspath(__file__))
                logs_dir = os.path.join(gui_dir, "logs")
                os.makedirs(logs_dir, exist_ok=True)
                t_stamp = time.strftime("%Y%m%d_%H%M%S")
                clean_port = str(self.port).replace("/", "_").replace("\\", "_").replace(":", "")
                filename = f"serial_{clean_port}_{t_stamp}.log"
                filepath = os.path.join(logs_dir, filename)

                self.backup_file = open(filepath, "w", encoding="utf-8")
                self.backup_filepath = filepath
                self.backup_filename = filename
                self.backup_active = True
                self.backup_lines_count = 0
                self._backup_start_time = time.time()

                now_str = time.strftime("%Y-%m-%d %H:%M:%S")
                cal_info = (
                    f"Offset: {self.last_cal_offset:.4f}° (calibrated at {self.last_cal_time})"
                    if self.last_cal_offset is not None
                    else "No calibration recorded in this session"
                )
                with self._lock:
                    fw_copy = dict(self.fw)

                fw_summary = ", ".join(f"{k}={v}" for k, v in fw_copy.items()) if fw_copy else "Default / Awaiting sync"

                header = (
                    "================================================================================\n"
                    "SERIAL MONITOR BACKUP LOG\n"
                    f"Port: {self.port} | Baud: {self.baud}\n"
                    f"Session Started: {now_str}\n"
                    f"Trigger Reason: {reason}\n"
                    f"Last IMU Calibration: {cal_info}\n"
                    f"Active FW Parameters: {fw_summary}\n"
                    "================================================================================\n"
                    f"{'[Timestamp]':<26} {'[Dir]':<6} Payload\n"
                    "--------------------------------------------------------------------------------\n"
                )
                self.backup_file.write(header)
                self.backup_file.flush()
                print(f"[SerialLink] Backup session started: {filepath}")
            except Exception as e:
                print(f"[SerialLink] Failed to start backup log: {e}")
                self.backup_active = False
                self.backup_file = None

    def stop_backup_session(self, reason="Motors OFF"):
        with self._log_file_lock:
            if not self.backup_active or not self.backup_file:
                return
            try:
                now = time.time()
                ms = int((now % 1.0) * 1000)
                t_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
                duration = now - getattr(self, "_backup_start_time", now)
                footer = (
                    "--------------------------------------------------------------------------------\n"
                    f"[{t_str}.{ms:03d}] [SYS] Session Stopped: {reason}\n"
                    f"Total Duration: {duration:.2f}s | Total Lines Captured: {self.backup_lines_count}\n"
                    "================================================================================\n"
                )
                self.backup_file.write(footer)
                self.backup_file.flush()
                self.backup_file.close()
                self.last_backup_saved_path = self.backup_filepath
                print(f"[SerialLink] Backup session ended: {self.backup_filepath} ({self.backup_lines_count} lines)")
            except Exception as e:
                print(f"[SerialLink] Error closing backup log: {e}")
            finally:
                self.backup_file = None
                self.backup_active = False

    def _log_entry(self, direction: str, text: str):
        with self._log_file_lock:
            if not self.backup_active or not self.backup_file:
                return
            try:
                now = time.time()
                ms = int((now % 1.0) * 1000)
                t_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
                line_entry = f"[{t_str}.{ms:03d}] [{direction}] {text}\n"
                self.backup_file.write(line_entry)
                self.backup_file.flush()
                self.backup_lines_count += 1
            except Exception as e:
                print(f"[SerialLink] Backup write error: {e}")

    def get_backup_status(self):
        with self._log_file_lock:
            return {
                "active": self.backup_active,
                "filepath": self.backup_filepath,
                "filename": getattr(self, "backup_filename", ""),
                "lines": self.backup_lines_count,
                "last_saved": getattr(self, "last_backup_saved_path", ""),
                "auto_enabled": self.auto_backup_enabled,
            }

    def set_auto_backup(self, enabled: bool):
        self.auto_backup_enabled = bool(enabled)

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

        clean_line = line
        for prefix in ("PITCH:", "AX12:", "SRV:", "RC:", "Updated ->", "BOOT:",
                       "CAL:", "ACK:", "TRIM:", "SAFETY:", "Motors ", "FT:",
                       "FA:"):
            idx = line.find(prefix)
            if idx != -1:
                clean_line = line[idx:]
                break

        # Handle motor state transitions
        if "Motors ENABLED" in clean_line:
            self._set_motors_on(True, "Motors ENABLED")
            self._log_entry("RX", line)
            return
        elif "Motors DISABLED" in clean_line:
            self._log_entry("RX", line)
            self._set_motors_on(False, "Motors DISABLED")
            return
        elif "SAFETY" in clean_line:
            self._log_entry("RX", line)
            self._set_motors_on(False, "SAFETY CUTOFF")
            return

        # Log incoming communication if backup is active
        self._log_entry("RX", line)

        if clean_line.startswith("PITCH:"):
            self._parse_telemetry(clean_line)
        elif clean_line.startswith("AX12:"):
            self._parse_ax12_state(clean_line)
        elif clean_line.startswith("SRV:"):
            self._parse_servo_health(clean_line)
        elif clean_line.startswith("RCCEN:"):
            # One-shot boot capture report:
            #   RCCEN:<Ch3 centre>,<Ch4 centre>,IDLE8:<-1|0|1>,IDLE10:<-1|0|1>
            # Ch3/Ch4 are proportional sticks, so a resting WIDTH is captured.
            # Ch8/Ch10 are switches on this transmitter, so what is captured is
            # which position they were resting in -- that position becomes idle.
            try:
                payload = clean_line.split(":", 1)[1]
                parts = payload.split(",")
                vals, idle = [], {}
                for part in parts:
                    if part.startswith("IDLE"):
                        k, v = part.split(":", 1)
                        idle[k] = int(float(v))
                    elif len(vals) < 2:
                        vals.append(int(float(part)))
                with self._lock:
                    if vals:
                        self.rc["centres"] = vals
                    if idle:
                        self.rc["idle_states"] = idle
            except (ValueError, IndexError):
                pass
        elif clean_line.startswith("RC:"):
            self._parse_rc_channels(clean_line)
        elif clean_line.startswith("Updated ->"):
            self._parse_fw_update(clean_line)
        elif clean_line.startswith("CAL:DONE"):
            try:
                offset_str = clean_line.split("OFFSET:")[1].split(",")[0].strip()
                val = float(offset_str)
                with self._lock:
                    self.fw["pitchOffset"] = val
                    self.last_cal_offset = val
                    self.last_cal_time = time.strftime("%Y-%m-%d %H:%M:%S")
            except (IndexError, ValueError):
                pass
        elif clean_line.startswith("CAL:START"):
            with self._lock:
                self.last_cal_time = time.strftime("%Y-%m-%d %H:%M:%S")
        elif clean_line.startswith("ACK:AUTOTRIM_"):
            with self._lock:
                self.auto_trim_on = clean_line.endswith("ON")
        elif clean_line.startswith("ACK:RC_"):
            with self._lock:
                self.rc["rc_enabled"] = clean_line.endswith("ON")
        elif clean_line.startswith("ACK:TORQUE_"):
            with self._lock:
                self.ax12["torque_on"] = clean_line.endswith("ON")
        elif clean_line.startswith("TRIM:DONE"):
            try:
                parts = dict(p.split(":", 1) for p in clean_line.split()[1:])
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

            if "ATE" in data:
                self.auto_trim_on = bool(int(data["ATE"]))

            # AX-12 leg subsystem fields (extended telemetry)
            if "TORQ" in data:
                self.ax12["torque_on"]   = bool(int(data["TORQ"]))
            if "CRL"  in data:
                self.ax12["crouch_l"]    = data["CRL"]
            if "CRR"  in data:
                self.ax12["crouch_r"]    = data["CRR"]
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

            # RC receiver fields
            if "RCL"  in data:
                self.rc["link_ok"]    = bool(int(data["RCL"]))
            if "RCE"  in data:
                self.rc["rc_enabled"] = bool(int(data["RCE"]))
            if "RCA"  in data:
                self.rc["armed"]      = bool(int(data["RCA"]))
            if "RCD"  in data:
                self.rc["drive"]      = data["RCD"]
            if "RCS"  in data:
                self.rc["steer"]      = data["RCS"]
            if "TVEL" in data:
                self.rc["target_vel"] = data["TVEL"]
            if "RCT"  in data:
                # targetAngle. Mirrored into fw so the GUI "Target" slider
                # follows it -- this field exists because targetAngle was
                # previously invisible in telemetry, which let Ch8 silently
                # slew it to the clamp with nothing on screen to show it.
                self.rc["target_ang"]  = data["RCT"]
                self.fw["targetAngle"] = data["RCT"]
            if "RC8"  in data:
                self.rc["ch8_raw"]     = int(data["RC8"])

            if len(self.history["t"]) > 500:
                for k in self.history:
                    self.history[k].pop(0)

        if "MOT" in data:
            self._set_motors_on(bool(int(data["MOT"])), f"Telemetry MOT:{int(data['MOT'])}")

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
            if "CRL"  in data:
                self.ax12["crouch_l"]      = data["CRL"]
            if "CRR"  in data:
                self.ax12["crouch_r"]      = data["CRR"]
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

    def _parse_rc_channels(self, line):
        """Parses: RC:1500,1500,1000,1000,2000,1000,1000,1000,1000,1500,LINK:1

        TEN raw channel widths in microseconds (Ch1..Ch10) then the link flag.
        Ch9/Ch10 were added when RC crouch became reachable in this variant.
        A channel the receiver has never populated reads 0 — kept as 0 rather
        than coerced to a midpoint, because "no data" and "centred stick" must
        stay distinguishable in the UI. This line exists specifically so the
        channel map can be verified against the physical transmitter.
        """
        try:
            _, payload = line.split(":", 1)
            parts = payload.split(",")
            chans = []
            for i in range(10):
                try:
                    chans.append(int(float(parts[i])))
                except (IndexError, ValueError):
                    chans.append(0)
            link = None
            # Scanned by prefix rather than by index so an older firmware that
            # still emits only eight channels keeps parsing.
            for part in parts[8:]:
                if part.startswith("LINK:"):
                    try:
                        link = bool(int(part.split(":", 1)[1]))
                    except ValueError:
                        link = None
            with self._lock:
                self.rc["channels"] = chans
                if link is not None:
                    self.rc["link_ok"] = link
        except (ValueError, IndexError):
            pass

    def _parse_servo_health(self, line):
        """Parses: SRV:<id>,<temp>,<load%>[,<err>,<fail>]

        The trailing err/fail fields are optional so an older firmware build
        still parses; they default to 0.
        """
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
                def _int_at(idx):
                    try:
                        return int(parts[idx].strip()) if len(parts) > idx else 0
                    except ValueError:
                        return 0
                err  = _int_at(3)
                fail = _int_at(4)
                with self._lock:
                    if sid in self.servo_health:
                        self.servo_health[sid]["temp"] = temp
                        self.servo_health[sid]["load"] = load
                        self.servo_health[sid]["err"]  = err
                        self.servo_health[sid]["fail"] = fail
        except Exception:
            pass

    def _parse_fw_update(self, line):
        """Parses: Updated -> P:11.2 I:0.0 D:0.0 Offset:0.0 Target:0.0 Alpha:0.96 Tilt:25.0"""
        KEY_MAP = {
            "P": "Kp", "I": "Ki", "D": "Kd",
            "Offset": "pitchOffset",
            "Target": "targetAngle", "Alpha": "alpha", "Tilt": "maxSafeTilt",
            "TrimGain": "Ki_trim", "CrouchL": "crouchOffsetL", "CrouchR": "crouchOffsetR",
            "VP": "Kp_vel",
            "RV": "RC_MAX_VEL", "RS": "RC_MAX_STEER", "RCM": "RC_MAX_CROUCH",
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
                # Mirror the RC limits into self.rc as well, so a UI reading RC
                # state has one place to look instead of straddling two dicts.
                if "RC_MAX_VEL"    in new_fw:
                    self.rc["max_vel"]    = new_fw["RC_MAX_VEL"]
                if "RC_MAX_STEER"  in new_fw:
                    self.rc["max_steer"]  = new_fw["RC_MAX_STEER"]
                if "RC_MAX_CROUCH" in new_fw:
                    self.rc["max_crouch"] = new_fw["RC_MAX_CROUCH"]
        except Exception:
            pass

    # ── DATA ACCESS ──────────────────────────────────────────────────────────
    def get_rc_state(self):
        with self._lock:
            state = dict(self.rc)
            state["channels"] = list(self.rc["channels"])
            return state

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
        self._log_entry("TX", text)
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

    def set_crouch(self, val):         self._send(f"CR{val}")     # both legs (mirrored)
    def set_crouch_l(self, val):       self._send(f"CRL{val}")
    def set_crouch_r(self, val):       self._send(f"CRR{val}")
    # Outer velocity-loop P gain: deg of lean per (count/s) of velocity error.
    def set_vel_p(self, val):          self._send(f"VP{val}")

    # ── RC control ───────────────────────────────────────────────────────────
    # RE0 parks the transmitter's authority so GUI sliders can be used on the
    # bench with a live TX nearby; it does NOT disarm (pulling authority from a
    # balancing robot would drop it) and it does NOT stop the failsafe.
    def set_rc_enabled(self, enabled):  self._send(f"RE{1 if enabled else 0}")
    def set_rc_max_vel(self, val):      self._send(f"RV{val}")
    def set_rc_max_steer(self, val):    self._send(f"RS{val}")
    def set_rc_max_crouch(self, val):   self._send(f"RCM{val}")
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
