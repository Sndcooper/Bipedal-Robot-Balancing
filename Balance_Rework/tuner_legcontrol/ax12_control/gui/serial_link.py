"""
serial_link.py — ax12_control
Background engine handling all 3DR telemetry radio communications.

PROTOCOL WARNING
----------------
This wire format belongs to ax12_control ALONE. It is not interchangeable with
any other tuner_legcontrol variant, and the collisions are deliberate traps to
be aware of rather than accidents:

    "TE1"  -> auto-trim ENABLE      in mcu_ik_engine_pretest_wireless
              (this variant uses "TQ1" for the torque kill so the two can
               never be confused if a port is opened against the wrong build)
    "CR40" -> crouch                in BOTH -- same meaning, same units
    "PS6 750" -> raw servo position in BOTH -- same meaning, same units

Downlink lines this parser understands:
    SRV:<id>,POS:<n>,GOAL:<n>,SPD:<n>,LOAD:<f>,TEMP:<n>,VOLT:<f>
    STATE:MODE:<0|1>,TQ:<0|1>,TL:<n>,CM:<n>,CS:<n>,MS:<n>,MT:<n>,CROUCH:<f>,
          FX1:<f>,FY1:<f>,FX2:<f>,FY2:<f>,IK1:<0|1>,IK2:<0|1>
    ACK:TORQUE_ON | ACK:TORQUE_LIMP | ACK:SERVOS_RESET
    PS:OK ... | PS:ERR ... | FT:ERR ...
    BOOT:OK AX12_CONTROL
    PONG<token>
"""

import threading
import time

try:
    import serial
except ImportError:
    serial = None


SERVO_IDS = (6, 14, 0, 1)


class SerialLink:
    def __init__(self, port="COM13", baud=115200):
        self.port     = port
        self.baud     = baud
        self.ser      = None
        self._thread  = None
        self._running = False
        self._lock    = threading.Lock()

        # MCU state mirror, seeded with the firmware's own power-on defaults so
        # the GUI renders a sane pose before the first STATE line lands.
        self.fw = {
            "mode": 0, "torqueOn": 1,
            "torqueLimit": 1023, "compMargin": 1, "compSlope": 4, "movingSpeed": 0,
            "moveTime": 800,
            "crouch": 0.0,
            "fx1": 1.0, "fy1": -151.1, "fx2": -6.0, "fy2": -149.6,
            "ik1": 1, "ik2": 1,
        }
        self._state_seen = False

        # Per-servo live readback. present_pos drives the FK "ghost" linkage;
        # goal is what the firmware currently commands. Their difference IS the
        # compliance droop the margin/slope sliders are tuning.
        self.servos = {
            sid: {"pos": None, "goal": None, "spd": 0,
                  "load": 0.0, "temp": 0, "volt": 0.0, "stamp": 0.0}
            for sid in SERVO_IDS
        }

        self.raw_log = []

    # ── CONNECTION ───────────────────────────────────────────────────────────
    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial not installed. Run: pip install pyserial")
        # timeout=None: blocking read — the accumulator controls all line
        # assembly. Do NOT use readline() with a short timeout on a radio link;
        # it returns partial lines when the radio pauses between chunks.
        self.ser = serial.Serial(self.port, self.baud, timeout=None)
        self._running = True
        self._thread  = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        self.request_state()   # pull the firmware's real settings immediately

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ── BACKGROUND READER ────────────────────────────────────────────────────
    def _read_loop(self):
        """Byte-accumulator loop — never dispatches a partial line.

        The 3DR radio sends data in chunks that need not align with newlines.
        readline() with a timeout returns half a line whenever the radio pauses
        mid-chunk, which here would surface as a servo momentarily reporting a
        truncated position. Accumulate and only dispatch on a real '\\n'.
        """
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
                    line = raw_line.decode("utf-8", errors="ignore").strip()
                    if line:
                        self._process_line(line)
            except Exception as e:
                print(f"[SerialLink] Read error: {e}")
                buf = b""          # discard corrupted buffer on error
                time.sleep(0.05)

    def _process_line(self, line):
        with self._lock:
            self.raw_log.append(line)
            if len(self.raw_log) > 1000:
                self.raw_log.pop(0)

        if line.startswith("SRV:"):
            self._parse_servo(line)
        elif line.startswith("STATE:"):
            self._parse_state(line)
        elif line.startswith("ACK:TORQUE_"):
            with self._lock:
                self.fw["torqueOn"] = 1 if line.endswith("_ON") else 0

    @staticmethod
    def _kv(payload):
        """'A:1,B:2.5' -> {'A': '1', 'B': '2.5'} — the shared comma/colon form."""
        out = {}
        for part in payload.split(","):
            if ":" in part:
                k, v = part.split(":", 1)
                out[k.strip()] = v.strip()
        return out

    def _parse_servo(self, line):
        d = self._kv(line)
        try:
            sid = int(d["SRV"])
        except (KeyError, ValueError):
            return
        with self._lock:
            if sid not in self.servos:
                return
            s = self.servos[sid]
            for key, field, cast in (("POS", "pos", int), ("GOAL", "goal", int),
                                     ("SPD", "spd", int), ("LOAD", "load", float),
                                     ("TEMP", "temp", int), ("VOLT", "volt", float)):
                if key in d:
                    try:
                        s[field] = cast(float(d[key])) if cast is int else cast(d[key])
                    except ValueError:
                        pass
            s["stamp"] = time.time()

    def _parse_state(self, line):
        # "STATE:MODE:0,TQ:1,..." — the first token carries two colons, so strip
        # the STATE: prefix before the generic comma/colon split.
        d = self._kv(line[len("STATE:"):])
        KEYS = {
            "MODE": ("mode", int), "TQ": ("torqueOn", int),
            "TL": ("torqueLimit", int), "CM": ("compMargin", int),
            "CS": ("compSlope", int),   "MS": ("movingSpeed", int),
            "MT": ("moveTime", int),
            "CROUCH": ("crouch", float),
            "FX1": ("fx1", float), "FY1": ("fy1", float),
            "FX2": ("fx2", float), "FY2": ("fy2", float),
            "IK1": ("ik1", int),   "IK2": ("ik2", int),
        }
        new = {}
        for k, raw in d.items():
            if k in KEYS:
                field, cast = KEYS[k]
                try:
                    new[field] = cast(float(raw)) if cast is int else cast(raw)
                except ValueError:
                    pass
        with self._lock:
            self.fw.update(new)
            self._state_seen = True

    # ── DATA ACCESS ──────────────────────────────────────────────────────────
    def state(self):
        with self._lock:
            return dict(self.fw)

    def servo_snapshot(self):
        with self._lock:
            return {k: dict(v) for k, v in self.servos.items()}

    def recent_lines(self, limit=100):
        with self._lock:
            return list(self.raw_log[-limit:])

    def state_seen(self):
        with self._lock:
            return self._state_seen

    # ── COMMAND API ──────────────────────────────────────────────────────────
    def _send(self, text):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text + "\n").encode("utf-8"))
            except Exception as e:
                print(f"[SerialLink] Write error: {e}")

    # Global servo settings — applied identically to all four by the firmware.
    def set_torque_limit(self, v):  self._send(f"TL{int(v)}")
    def set_comp_margin(self, v):   self._send(f"CM{int(v)}")
    def set_comp_slope(self, v):    self._send(f"CS{int(v)}")
    def set_moving_speed(self, v):  self._send(f"MS{int(v)}")

    # Duration of an interpolated pose move, ms. Fixed time, not fixed speed.
    def set_move_time(self, v):     self._send(f"MT{int(v)}")

    # Master torque kill. False => addr 24 = 0 on all four and the firmware
    # stops re-asserting goal position, so the legs go completely limp.
    def set_torque(self, on):       self._send(f"TQ{1 if on else 0}")

    # Pose mode: 0 = CROUCH (one vertical knob), 1 = IK (dragged foot target).
    # The two are mutually exclusive in firmware; the inactive one is ignored.
    def set_mode(self, mode):       self._send(f"MD{int(mode)}")
    def set_crouch(self, v):        self._send(f"CR{float(v):.2f}")
    def set_foot_target(self, leg, x, y):
        self._send(f"FT{int(leg)} {float(x):.2f} {float(y):.2f}")

    # Both legs in ONE command, so they start their interpolated move on the
    # same tick. Two back-to-back FT commands would start leg 2 about 60 ms
    # after leg 1 (the RX budget is ~3.4 bytes/tick) and restart the timer.
    def set_foot_all(self, x1, y1, x2, y2):
        self._send(f"FA {float(x1):.2f} {float(y1):.2f} "
                   f"{float(x2):.2f} {float(y2):.2f}")

    # Raw per-servo goal position, bypassing IK entirely. Firmware clamps 0-1023.
    def set_servo_position(self, servo_id, pos):
        self._send(f"PS{int(servo_id)} {int(pos)}")

    def home(self):          self._send("HM")
    def servo_reset(self):   self._send("SR")
    def request_state(self): self._send("RB")
