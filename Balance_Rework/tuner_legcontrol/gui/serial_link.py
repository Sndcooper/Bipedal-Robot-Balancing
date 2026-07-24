"""
serial_link.py 
The background engine (The Waiter) handling all STM32 communications.
"""

import threading
import time
try:
    import serial
except ImportError:
    serial = None

class SerialLink:
    def __init__(self, port="COM3", baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()
        
        # 1. The Data Vault (Historical Telemetry for Tab 1)
        self.history = {
            "t": [], "pitch": [], "pid_out": [], 
            "vel": [], "enc_l": [], "enc_r": []
        }
        self.start_time = time.time()
        
        # 2. MCU State
        self.fw = {}
        self.motors_on = False
        self._cutoff_time = None
        
        # 3. Serial Monitor Log
        self.raw_log = []
        
        # 4. NEW: Servo Health Tracking (For Tab 2)
        self.servo_health = {
            6:  {"temp": 0, "load": 0.0},
            0:  {"temp": 0, "load": 0.0},
            14: {"temp": 0, "load": 0.0},
            1:  {"temp": 0, "load": 0.0}
        }

    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial is not installed. Run 'pip install pyserial'")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        self._running = True
        self.start_time = time.time()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def close(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            self.ser.close()

    # ── THE ROUTER (Background Thread) ───────────────────────────────────────
    def _read_loop(self):
        while self._running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                with self._lock:
                    self.raw_log.append(line)
                    if len(self.raw_log) > 1000:
                        self.raw_log.pop(0)

                # Route 1: High-Speed Balance Telemetry
                if line.startswith("PITCH:"):
                    self._parse_telemetry(line)
                
                # Route 2: NEW Servo Health Data (Temp & Load)
                elif line.startswith("SRV:"):
                    self._parse_servo_health(line)
                
                # Route 3: Firmware State Updates
                elif line.startswith("Updated ->"):
                    self._parse_fw_update(line)
                
                # Route 4: Safety & Motor Events
                elif "SAFETY CUTOFF TRIGGERED" in line:
                    with self._lock:
                        self._cutoff_time = time.time()
                        self.motors_on = False
                elif "Motors ENABLED" in line:
                    with self._lock:
                        self.motors_on = True
                        self._cutoff_time = None
                elif "Motors DISABLED" in line:
                    with self._lock:
                        self.motors_on = False
            except Exception as e:
                print(f"[SerialLink] Read error: {e}")
                time.sleep(0.1)

    def _parse_telemetry(self, line):
        """Parses: PITCH:1.23, PID_OUT:-4.5, ENC_L:100, ENC_R:105, VEL:0.5 ..."""
        parts = line.split(',')
        data = {}
        for p in parts:
            if ':' in p:
                k, v = p.split(':', 1)
                try:
                    data[k.strip()] = float(v.strip())
                except ValueError:
                    pass
        
        with self._lock:
            self.history["t"].append(time.time() - self.start_time)
            self.history["pitch"].append(data.get("PITCH", 0.0))
            self.history["pid_out"].append(data.get("PID_OUT", 0.0))
            self.history["vel"].append(data.get("VEL", 0.0))
            self.history["enc_l"].append(data.get("ENC_L", 0.0))
            self.history["enc_r"].append(data.get("ENC_R", 0.0))
            
            # Keep arrays from eating all the PC's RAM
            if len(self.history["t"]) > 500:
                for k in self.history:
                    self.history[k].pop(0)

    def _parse_servo_health(self, line):
        """Parses: SRV:<id>,<temp>,<load%> (e.g. SRV:6,45,12.5)"""
        try:
            _, payload = line.split(':', 1)
            sid, temp, load = payload.split(',')
            sid = int(sid.strip())
            
            with self._lock:
                if sid in self.servo_health:
                    self.servo_health[sid]["temp"] = int(temp.strip())
                    self.servo_health[sid]["load"] = float(load.strip())
        except Exception:
            pass # Ignore corrupted lines

    def _parse_fw_update(self, line):
        """Parses: Updated -> P:11.2 I:0.0 ..."""
        try:
            _, payload = line.split("->", 1)
            parts = payload.split()
            new_fw = {}
            for p in parts:
                if ':' in p:
                    k, v = p.split(':', 1)
                    new_fw[k.strip()] = float(v.strip())
            with self._lock:
                self.fw.update(new_fw)
        except Exception:
            pass

    # ── DATA ACCESS API (For the GUI) ────────────────────────────────────────
    def snapshot(self):
        with self._lock:
            return {k: list(v) for k, v in self.history.items()}

    def recent_lines(self, limit=100):
        with self._lock:
            return list(self.raw_log[-limit:])

    def clear_log(self):
        with self._lock:
            self.raw_log.clear()

    def cutoff_since(self):
        with self._lock:
            return self._cutoff_time is not None

    def get_servo_health(self):
        """Returns a safe copy of the servo temperatures and loads."""
        with self._lock:
            # Deep copy to prevent race conditions
            return {k: dict(v) for k, v in self.servo_health.items()}

    # ── COMMAND API (Sending Orders to the Kitchen) ──────────────────────────
    def _send(self, text):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text + "\n").encode('utf-8'))
                self.ser.flush()
            except Exception as e:
                print(f"[SerialLink] Write error: {e}")

    # Legacy Tuning Commands
    def set_kp(self, val): self._send(f"P{val}")
    def set_ki(self, val): self._send(f"I{val}")
    def set_kd(self, val): self._send(f"D{val}")
    def set_kd_vel(self, val): self._send(f"V{val}")
    def set_alpha(self, val): self._send(f"A{val}")
    def set_target(self, val): self._send(f"S{val}")
    def set_offset(self, val): pass # Offset is handled via Calibration
    def set_tilt(self, val): self._send(f"T{val}")
    
    def set_gains(self, p, i, d, v, a):
        self.set_kp(p)
        self.set_ki(i)
        self.set_kd(d)
        self.set_kd_vel(v)
        self.set_alpha(a)

    def calibrate(self): self._send("C")
    def toggle_motors(self): self._send("M")
    def reset_integral(self): self._send("R")
    def arm_cutoff_watch(self):
        with self._lock:
            self._cutoff_time = None

    # NEW: Leg Controls
    def send_leg_position(self, servo_id, pos):
        """Sends: POS,6,717"""
        self._send(f"POS,{servo_id},{int(pos)}")

    def send_torque_limit(self, servo_id, limit):
        """Sends: TRQ,6,1023"""
        self._send(f"TRQ,{servo_id},{int(limit)}")

    def send_compliance(self, servo_id, margin, slope):
        """Sends: CMP,6,1,4"""
        self._send(f"CMP,{servo_id},{int(margin)},{int(slope)}")