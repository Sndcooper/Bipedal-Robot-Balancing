"""
╔══════════════════════════════════════════════════════════════════════╗
║  AUTONOMOUS PID AUTOTUNER — Self-Balancing Robot                   ║
║  Twiddle + Heuristic AI Advisor + Safety Monitor                   ║
║                                                                     ║
║  Firmware: STM32 Bluepill, MPU6050, Encoders, Serial1 @ 115200     ║
║  Protocol: P<kp> I<ki> D<kd> O<ofs> S<set> C(calibrate) M(toggle) ║
║  Telemetry: PITCH:<v>, PID_OUT:<v>, ENC_L:<v>, <enc_r>  @ 20 Hz   ║
╚══════════════════════════════════════════════════════════════════════╝

Usage:
    python autotune.py                    # Use defaults (COM10)
    python autotune.py --port COM5        # Specify port
    python autotune.py --port COM10 --plot  # With live plot
"""

import serial
import time
import threading
import numpy as np
from collections import deque
from datetime import datetime
import json
import os
import sys
import argparse

# ════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ════════════════════════════════════════════════════════════════════

DEFAULT_PORT = 'COM10'
BAUD = 115200

# Safety
SAFE_PITCH_LIMIT = 30.0        # |pitch| > this → EMERGENCY STOP
WARN_PITCH_LIMIT = 20.0        # |pitch| > this → caution flag
INTEGRAL_WINDUP_LIMIT = 15.0   # Max Ki to prevent runaway

# Timing
EVAL_WINDOW = 2.0              # Seconds of data per cost evaluation
SETTLE_TIME = 0.5              # Wait after param change before measuring
STABLE_THRESHOLD = 2.5         # Mean |pitch| < this → "stable"
STABLE_DURATION = 8.0          # Seconds of stability → declare success
MAX_TUNING_TIME = 180.0        # Hard timeout for entire tuning session

# Twiddle
MAX_ITERATIONS = 60
TWIDDLE_TOLERANCE = 0.05       # Sum(dp) below this → converged
DP_DECAY = 0.7                 # dp shrink factor on fail
DP_GROW = 1.1                  # dp grow factor on success

# PID limits (hard clamps)
KP_RANGE = (0.0, 80.0)
KI_RANGE = (0.0, 10.0)
KD_RANGE = (0.0, 15.0)

# Initial gains — deliberately conservative
INITIAL_PID = [8.0, 0.0, 0.8]              # [Kp, Ki, Kd]
INITIAL_DP  = [3.0, 0.3, 0.4]              # [dp_kp, dp_ki, dp_kd]

# Cost function weights
W_MAE       = 1.0     # Mean absolute pitch error
W_VARIANCE  = 0.5     # Pitch variance (oscillation)
W_MAX_PITCH = 0.3     # Peak |pitch| penalty
W_MOTOR     = 0.05    # Motor effort (RMS of PID output / 255)
W_DERIVATIVE = 0.2    # Mean |d(pitch)/dt| — jitter penalty
FALL_PENALTY = 500.0  # Added if robot fell during evaluation


# ════════════════════════════════════════════════════════════════════
#  SERIAL INTERFACE
# ════════════════════════════════════════════════════════════════════

class SerialInterface:
    """Thread-safe serial connection to the STM32."""

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self._lock = threading.Lock()

    def connect(self):
        print(f"\n📡 Connecting to {self.port} @ {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2.0)  # STM32 resets on serial connect
        self.ser.reset_input_buffer()
        print(f"   ✅ Connected")

    def send(self, cmd):
        """Send a command string (adds newline)."""
        with self._lock:
            if self.ser and self.ser.is_open:
                self.ser.write(f"{cmd}\n".encode())
                time.sleep(0.03)

    def readline(self):
        """Read one line (blocking up to timeout)."""
        if self.ser and self.ser.is_open:
            try:
                return self.ser.readline().decode('utf-8', errors='ignore').strip()
            except Exception:
                return ""
        return ""

    def set_pid(self, kp, ki, kd):
        """Send all three PID gains."""
        self.send(f"P{kp:.4f}")
        self.send(f"I{ki:.4f}")
        self.send(f"D{kd:.4f}")

    def set_target(self, angle):
        self.send(f"S{angle:.2f}")

    def set_offset(self, offset):
        self.send(f"O{offset:.2f}")

    def calibrate(self):
        print("   📐 Calibrating IMU (hold robot upright & still)...")
        self.send("C")
        time.sleep(2.0)  # Firmware takes 100 samples × 10ms = 1s + margin
        print("   ✅ Calibration sent")

    def toggle_motors(self):
        self.send("M")
        time.sleep(0.15)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


# ════════════════════════════════════════════════════════════════════
#  DATA BUFFER — Rolling window of telemetry
# ════════════════════════════════════════════════════════════════════

class DataBuffer:
    """Thread-safe rolling buffer of telemetry data."""

    def __init__(self, max_size=2000):
        self.max_size = max_size
        self._lock = threading.Lock()
        self.clear()

    def clear(self):
        with self._lock:
            self.timestamps = deque(maxlen=self.max_size)
            self.pitch = deque(maxlen=self.max_size)
            self.pid_out = deque(maxlen=self.max_size)
            self.enc_l = deque(maxlen=self.max_size)
            self.enc_r = deque(maxlen=self.max_size)

    def append(self, t, pitch, pid_out, enc_l, enc_r):
        with self._lock:
            self.timestamps.append(t)
            self.pitch.append(pitch)
            self.pid_out.append(pid_out)
            self.enc_l.append(enc_l)
            self.enc_r.append(enc_r)

    def get_window(self, seconds):
        """Get data from the last N seconds."""
        with self._lock:
            if not self.timestamps:
                return None
            cutoff = time.time() - seconds
            idx = 0
            for i, t in enumerate(self.timestamps):
                if t >= cutoff:
                    idx = i
                    break
            return {
                'time': list(self.timestamps)[idx:],
                'pitch': list(self.pitch)[idx:],
                'pid_out': list(self.pid_out)[idx:],
                'enc_l': list(self.enc_l)[idx:],
                'enc_r': list(self.enc_r)[idx:],
            }

    @property
    def latest_pitch(self):
        with self._lock:
            return self.pitch[-1] if self.pitch else 0.0

    @property
    def latest_time(self):
        with self._lock:
            return self.timestamps[-1] if self.timestamps else 0.0

    @property
    def count(self):
        with self._lock:
            return len(self.pitch)


# ════════════════════════════════════════════════════════════════════
#  TELEMETRY READER — Background thread
# ════════════════════════════════════════════════════════════════════

class TelemetryReader:
    """Background thread that continuously reads and parses serial data."""

    def __init__(self, serial_iface, data_buffer):
        self.serial = serial_iface
        self.buffer = data_buffer
        self.running = False
        self.motors_on = False
        self.firmware_kp = 0.0
        self.firmware_ki = 0.0
        self.firmware_kd = 0.0
        self.firmware_offset = 0.0
        self._thread = None

    def start(self):
        self.running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self):
        while self.running:
            line = self.serial.readline()
            if not line:
                continue
            self._parse(line)

    def _parse(self, line):
        # Telemetry: PITCH:<val>, PID_OUT:<val>, ENC_L:<val>, <enc_r>
        if line.startswith('PITCH:'):
            try:
                parts = line.split(',')
                pitch = float(parts[0].split(':')[1])
                pid_out = float(parts[1].strip().split(':')[1])
                enc_l = int(parts[2].strip().split(':')[1])
                enc_r = int(parts[3].strip())
                self.buffer.append(time.time(), pitch, pid_out, enc_l, enc_r)
            except (ValueError, IndexError):
                pass
        elif 'Motors ENABLED' in line:
            self.motors_on = True
        elif 'Motors DISABLED' in line:
            self.motors_on = False
        elif 'Updated' in line:
            try:
                for tok in line.split():
                    if tok.startswith('P:'):
                        self.firmware_kp = float(tok[2:])
                    elif tok.startswith('I:'):
                        self.firmware_ki = float(tok[2:])
                    elif tok.startswith('D:'):
                        self.firmware_kd = float(tok[2:])
                    elif tok.startswith('Offset:'):
                        self.firmware_offset = float(tok[7:])
            except (ValueError, IndexError):
                pass
        elif 'Calibration complete' in line:
            try:
                self.firmware_offset = float(line.split(':')[-1].strip())
            except ValueError:
                pass


# ════════════════════════════════════════════════════════════════════
#  SAFETY MONITOR
# ════════════════════════════════════════════════════════════════════

class SafetyMonitor:
    """Watches pitch and triggers emergency stop if needed."""

    def __init__(self, serial_iface, data_buffer, reader):
        self.serial = serial_iface
        self.buffer = data_buffer
        self.reader = reader
        self.fell = False
        self.emergency_triggered = False

    def check(self):
        """Returns True if safe, False if emergency triggered."""
        pitch = abs(self.buffer.latest_pitch)

        if pitch > SAFE_PITCH_LIMIT:
            self._emergency_stop(f"PITCH {pitch:.1f}° > {SAFE_PITCH_LIMIT}° LIMIT")
            return False

        return True

    def _emergency_stop(self, reason):
        """Kill motors and zero PID immediately."""
        if self.emergency_triggered:
            return
        self.emergency_triggered = True
        self.fell = True

        print(f"\n   🚨 EMERGENCY STOP: {reason}")

        # Zero PID first (fastest way to stop motor drive)
        self.serial.send("P0")
        self.serial.send("I0")
        self.serial.send("D0")

        # Disable motors if they're on
        if self.reader.motors_on:
            self.serial.toggle_motors()
            time.sleep(0.1)

        print("   🛑 Motors OFF, PID zeroed")

    def reset(self):
        """Reset safety flags for next trial."""
        self.fell = False
        self.emergency_triggered = False

    def is_warning(self):
        return abs(self.buffer.latest_pitch) > WARN_PITCH_LIMIT


# ════════════════════════════════════════════════════════════════════
#  PERFORMANCE EVALUATOR — Cost function
# ════════════════════════════════════════════════════════════════════

class PerformanceEvaluator:
    """Computes a single cost scalar from a data window."""

    @staticmethod
    def evaluate(data, fell=False):
        """
        Compute cost from a data window dict.
        Lower cost = better balancing performance.

        Components:
          - MAE:       Mean absolute pitch error (want ≈ 0)
          - Variance:  Oscillation penalty
          - Max pitch: Peak deviation penalty
          - Motor:     RMS motor effort / 255 (energy penalty)
          - Jitter:    Mean |d(pitch)/dt| (smoothness)
        """
        if data is None or len(data['pitch']) < 5:
            return 999.0, {}

        pitch = np.array(data['pitch'])
        pid_out = np.array(data['pid_out'])

        mae = np.mean(np.abs(pitch))
        variance = np.var(pitch)
        max_pitch = np.max(np.abs(pitch))
        motor_rms = np.sqrt(np.mean(pid_out ** 2)) / 255.0

        # Pitch derivative (jitter / oscillation rate)
        if len(pitch) > 1:
            dt_arr = np.diff(data['time'])
            dt_arr = np.where(dt_arr == 0, 0.001, dt_arr)  # Avoid div/0
            dpitch = np.abs(np.diff(pitch) / dt_arr)
            jitter = np.mean(dpitch)
        else:
            jitter = 0.0

        cost = (
            W_MAE * mae +
            W_VARIANCE * variance +
            W_MAX_PITCH * max_pitch +
            W_MOTOR * motor_rms +
            W_DERIVATIVE * jitter
        )

        if fell:
            cost += FALL_PENALTY

        details = {
            'mae': mae,
            'variance': variance,
            'max_pitch': max_pitch,
            'motor_rms': motor_rms,
            'jitter': jitter,
            'fell': fell,
            'cost': cost,
        }

        return cost, details


# ════════════════════════════════════════════════════════════════════
#  AI TUNING ADVISOR — Heuristic "LLM-style" reasoning
# ════════════════════════════════════════════════════════════════════

class AITuningAdvisor:
    """
    Rule-based expert system that analyzes balancing patterns and
    provides intelligent adjustments to the tuning strategy.

    Acts like an embedded "LLM reasoning layer" — it observes trends,
    diagnoses issues, and modifies step sizes & search directions.
    """

    def __init__(self):
        self.history = []   # List of (params, cost, details) tuples
        self.advice_log = []

    def analyze(self, params, cost, details, dp):
        """
        Analyze latest trial results, return modified dp and optional
        parameter overrides.

        Returns:
            dp_new: modified step sizes [dp_kp, dp_ki, dp_kd]
            overrides: dict of {param_index: forced_value} or None
            advice: human-readable reasoning string
        """
        kp, ki, kd = params
        dp_new = list(dp)
        overrides = None
        reasons = []

        mae = details.get('mae', 99)
        variance = details.get('variance', 99)
        jitter = details.get('jitter', 99)
        max_pitch = details.get('max_pitch', 99)
        motor_rms = details.get('motor_rms', 0)
        fell = details.get('fell', False)

        # ── Pattern 1: FALL DETECTED ────────────────────────────
        if fell:
            reasons.append("🔴 FALL detected — cutting Kp by 30%, zeroing Ki")
            dp_new[0] = max(dp_new[0] * 0.5, 0.5)  # Smaller Kp steps
            overrides = {}
            if kp > 15:
                overrides[0] = kp * 0.7  # Reduce Kp
            overrides[1] = 0.0  # Zero Ki (integral windup likely)
            return dp_new, overrides, " | ".join(reasons)

        # ── Pattern 2: HIGH-FREQUENCY OSCILLATION ───────────────
        if jitter > 30 and variance > 10:
            reasons.append(f"🟡 Oscillation detected (jitter={jitter:.1f}, var={variance:.1f})")
            reasons.append("   → Increasing Kd step, decreasing Kp step")
            dp_new[2] *= 1.3   # Explore more Kd (damping)
            dp_new[0] *= 0.8   # Slow down Kp exploration

            if kd < 1.0 and kp > 20:
                reasons.append("   → Kd is very low relative to Kp — boosting Kd")
                overrides = {2: kd + 1.0}

        # ── Pattern 3: SLOW DRIFT (low jitter, nonzero MAE) ────
        elif jitter < 5 and mae > 5 and variance < 3:
            reasons.append(f"🟡 Steady-state drift (MAE={mae:.1f}°, low oscillation)")
            reasons.append("   → Need more Ki (integral action)")
            dp_new[1] *= 1.3

            if ki < 0.3:
                reasons.append("   → Ki is near-zero — injecting small Ki")
                overrides = {1: 0.5}

        # ── Pattern 4: GOOD BUT NOT GREAT ──────────────────────
        elif mae < 5 and variance < 5 and max_pitch < 15:
            reasons.append(f"🟢 Decent balance (MAE={mae:.1f}°) — fine-tuning")
            # Shrink all step sizes for precision
            dp_new = [d * 0.85 for d in dp_new]

        # ── Pattern 5: MOTOR SATURATION ────────────────────────
        if motor_rms > 0.8:
            reasons.append(f"🟡 Motor effort high (RMS={motor_rms:.2f}) — Kp may be too large")
            dp_new[0] *= 0.7

        # ── Pattern 6: VERY STABLE (near goal) ─────────────────
        if mae < 2.0 and variance < 1.0 and max_pitch < 5:
            reasons.append(f"🟢 EXCELLENT balance! MAE={mae:.1f}°")
            # Tiny precision steps only
            dp_new = [max(d * 0.5, 0.1) for d in dp_new]

        # ── Pattern 7: Kp/Kd RATIO CHECK ──────────────────────
        if kp > 0 and kd > 0:
            ratio = kp / kd
            if ratio > 40:
                reasons.append(f"🟡 Kp/Kd ratio={ratio:.0f} (too high) — robot will oscillate")
                reasons.append("   → Boosting Kd exploration")
                dp_new[2] *= 1.5

        # ── Clamp dp values ────────────────────────────────────
        dp_new[0] = np.clip(dp_new[0], 0.1, 15.0)   # Kp step
        dp_new[1] = np.clip(dp_new[1], 0.02, 2.0)    # Ki step
        dp_new[2] = np.clip(dp_new[2], 0.05, 5.0)    # Kd step

        # Store history
        self.history.append({
            'params': list(params),
            'cost': cost,
            'details': details,
        })

        advice = " | ".join(reasons) if reasons else "ℹ️ No special patterns — continuing Twiddle"
        self.advice_log.append(advice)

        return dp_new, overrides, advice


# ════════════════════════════════════════════════════════════════════
#  PID AUTOTUNER — Main orchestrator
# ════════════════════════════════════════════════════════════════════

class PIDAutoTuner:
    """
    Fully autonomous PID tuner using Twiddle (coordinate descent)
    enhanced by an AI advisor for adaptive step sizing.

    Phases:
      1. Connect & calibrate (motors OFF)
      2. Ramp up — gently test initial gains
      3. Twiddle loop — optimize cost function
      4. Verify — hold best gains for STABLE_DURATION
      5. Report — print & save results
    """

    def __init__(self, port=DEFAULT_PORT, enable_plot=False):
        self.serial = SerialInterface(port, BAUD)
        self.buffer = DataBuffer()
        self.reader = TelemetryReader(self.serial, self.buffer)
        self.safety = SafetyMonitor(self.serial, self.buffer, self.reader)
        self.evaluator = PerformanceEvaluator()
        self.advisor = AITuningAdvisor()
        self.enable_plot = enable_plot

        # State
        self.params = list(INITIAL_PID)    # [Kp, Ki, Kd]
        self.dp = list(INITIAL_DP)
        self.best_params = list(INITIAL_PID)
        self.best_cost = float('inf')
        self.motors_on = False
        self.session_log = []
        self.start_time = None

        # Plotting
        self.fig = None
        self.axes = None
        self.plot_data = {
            'iteration': [],
            'cost': [],
            'kp': [],
            'ki': [],
            'kd': [],
        }

    # ──────────────────────────────────────────────────────────
    #  PUBLIC API
    # ──────────────────────────────────────────────────────────

    def run(self):
        """Execute the full autotuning pipeline."""
        try:
            self._print_banner()
            self._phase_connect()
            self._phase_calibrate()

            if self.enable_plot:
                self._setup_plot()

            self._phase_ramp_up()
            self._phase_twiddle()
            self._phase_verify()
            self._phase_report()

        except KeyboardInterrupt:
            print("\n\n⚠️ Interrupted by user")
            self._safe_shutdown()
        except Exception as e:
            print(f"\n\n❌ Fatal error: {e}")
            import traceback
            traceback.print_exc()
            self._safe_shutdown()
            raise

    # ──────────────────────────────────────────────────────────
    #  PHASE 1: CONNECT
    # ──────────────────────────────────────────────────────────

    def _phase_connect(self):
        print("\n" + "═" * 60)
        print("  PHASE 1 — CONNECT")
        print("═" * 60)

        self.serial.connect()
        self.reader.start()
        time.sleep(0.5)

        # Ensure motors are OFF and PID is zeroed
        self.serial.set_pid(0, 0, 0)
        time.sleep(0.2)

        # Check if motors are on (firmware starts with them off)
        if self.reader.motors_on:
            self.serial.toggle_motors()
            time.sleep(0.3)

        print("   ✅ Motors confirmed OFF, PID gains zeroed")

    # ──────────────────────────────────────────────────────────
    #  PHASE 2: CALIBRATE
    # ──────────────────────────────────────────────────────────

    def _phase_calibrate(self):
        print("\n" + "═" * 60)
        print("  PHASE 2 — IMU CALIBRATION (motors OFF)")
        print("═" * 60)

        # Wait for some telemetry to arrive
        print("   Waiting for telemetry stream...", end="", flush=True)
        t0 = time.time()
        while self.buffer.count < 5:
            time.sleep(0.1)
            if time.time() - t0 > 10:
                print("\n   ❌ No telemetry received! Check wiring & COM port.")
                self._safe_shutdown()
                sys.exit(1)
        print(f" got {self.buffer.count} samples ✅")

        # Pre-cal pitch reading
        pre_cal_pitch = self.buffer.latest_pitch
        print(f"   Pre-calibration pitch: {pre_cal_pitch:+.2f}°")

        # Run calibration
        self.serial.calibrate()
        time.sleep(1.0)  # Let post-cal data arrive

        # Verify
        post_cal_pitch = self.buffer.latest_pitch
        print(f"   Post-calibration pitch: {post_cal_pitch:+.2f}°")
        print(f"   Firmware offset: {self.reader.firmware_offset:.2f}°")

        if abs(post_cal_pitch) < 5:
            print("   ✅ Calibration looks good")
        else:
            print(f"   ⚠️ Pitch is {post_cal_pitch:.1f}° — consider adjusting robot stance")

    # ──────────────────────────────────────────────────────────
    #  PHASE 3: RAMP UP — Gentle initial test
    # ──────────────────────────────────────────────────────────

    def _phase_ramp_up(self):
        print("\n" + "═" * 60)
        print("  PHASE 3 — RAMP UP (gentle motor test)")
        print("═" * 60)

        self.start_time = time.time()

        # Start with very low Kp to test motor direction
        test_kp = 3.0
        print(f"   Setting initial test gains: Kp={test_kp}, Ki=0, Kd=0.3")
        self.serial.set_pid(test_kp, 0, 0.3)
        time.sleep(0.2)

        # Enable motors
        print("   ⚡ Enabling motors...")
        self.serial.toggle_motors()
        self.motors_on = True
        time.sleep(0.1)
        self.safety.reset()
        self.buffer.clear()

        # Watch for 2 seconds
        print("   Observing response...", end="", flush=True)
        t0 = time.time()
        while time.time() - t0 < 2.0:
            if not self.safety.check():
                print(f"\n   ⚠️ Robot fell during ramp-up with Kp={test_kp}")
                print("   Trying negative Kp (reversed motor direction)...")
                self._recover_from_fall()
                test_kp = -test_kp
                self.serial.set_pid(test_kp, 0, 0.3)
                self._enable_motors()
                time.sleep(2.0)
                if not self.safety.check():
                    print("   ⚠️ Still falling — motor direction issue or Kp too low.")
                break
            time.sleep(0.05)

        # Evaluate initial performance
        data = self.buffer.get_window(1.5)
        cost, details = self.evaluator.evaluate(data, fell=self.safety.fell)

        print(f" done")
        print(f"   Initial cost: {cost:.2f} (MAE={details.get('mae', 99):.1f}°)")

        # Set initial params from ramp-up
        self.params = [abs(test_kp) * (1 if test_kp > 0 else -1),
                       INITIAL_PID[1], INITIAL_PID[2]]

        # If Kp needed to be negative, start twiddle around negative Kp
        if test_kp < 0:
            self.params[0] = -INITIAL_PID[0]
            print(f"   📝 Motor direction reversed — using negative Kp")

        # Set actual initial PID for twiddle
        self.params = [self.params[0] if test_kp < 0 else INITIAL_PID[0],
                       INITIAL_PID[1], INITIAL_PID[2]]
        self.serial.set_pid(*self.params)
        print(f"   Starting Twiddle from: Kp={self.params[0]:.1f}, "
              f"Ki={self.params[1]:.1f}, Kd={self.params[2]:.1f}")

    # ──────────────────────────────────────────────────────────
    #  PHASE 4: TWIDDLE — Main optimization loop
    # ──────────────────────────────────────────────────────────

    def _phase_twiddle(self):
        print("\n" + "═" * 60)
        print("  PHASE 4 — TWIDDLE OPTIMIZATION")
        print("═" * 60)

        # Get baseline cost with current params
        self.best_cost = self._run_trial(self.params)
        self.best_params = list(self.params)

        print(f"\n   Baseline cost: {self.best_cost:.2f}")
        print(f"   Starting optimization (max {MAX_ITERATIONS} iterations)...\n")

        iteration = 0
        consecutive_stable = 0
        param_names = ['Kp', 'Ki', 'Kd']
        param_ranges = [KP_RANGE, KI_RANGE, KD_RANGE]

        while iteration < MAX_ITERATIONS:
            # Check timeout
            elapsed = time.time() - self.start_time
            if elapsed > MAX_TUNING_TIME:
                print(f"\n   ⏰ Timeout ({MAX_TUNING_TIME:.0f}s) reached")
                break

            # Check convergence
            if sum(self.dp) < TWIDDLE_TOLERANCE:
                print(f"\n   ✅ Converged! sum(dp) = {sum(self.dp):.4f}")
                break

            # Early success: stable for a while
            if consecutive_stable >= 3:
                print(f"\n   ✅ Stable for {consecutive_stable} consecutive iterations!")
                break

            for i in range(3):  # For each PID parameter
                iteration += 1
                if iteration > MAX_ITERATIONS:
                    break

                # ── Try INCREASE ──
                self.params[i] += self.dp[i]
                self.params[i] = np.clip(self.params[i],
                                         param_ranges[i][0],
                                         param_ranges[i][1])

                cost = self._run_trial(self.params)

                self._log_iteration(iteration, cost)

                if cost < self.best_cost:
                    # Improvement!
                    self.best_cost = cost
                    self.best_params = list(self.params)
                    self.dp[i] *= DP_GROW
                    self._print_trial(iteration, param_names[i], "⬆ +dp",
                                      "✅ BETTER", cost)
                else:
                    # Try DECREASE (go back and go the other way)
                    self.params[i] -= 2 * self.dp[i]
                    self.params[i] = np.clip(self.params[i],
                                             param_ranges[i][0],
                                             param_ranges[i][1])

                    cost = self._run_trial(self.params)

                    if cost < self.best_cost:
                        # Improvement in opposite direction!
                        self.best_cost = cost
                        self.best_params = list(self.params)
                        self.dp[i] *= DP_GROW
                        self._print_trial(iteration, param_names[i], "⬇ -dp",
                                          "✅ BETTER", cost)
                    else:
                        # Neither direction helped — shrink step
                        self.params[i] += self.dp[i]  # Reset to original
                        self.dp[i] *= DP_DECAY
                        self._print_trial(iteration, param_names[i], "↩ reset",
                                          "➖ shrink dp", cost)

                # ── AI Advisor pass ──
                data = self.buffer.get_window(EVAL_WINDOW)
                _, details = self.evaluator.evaluate(data, fell=self.safety.fell)
                self.dp, overrides, advice = self.advisor.analyze(
                    self.params, cost, details, self.dp
                )

                if advice and "ℹ️" not in advice:
                    print(f"   🧠 AI: {advice}")

                if overrides:
                    for idx, val in overrides.items():
                        self.params[idx] = val
                    self.serial.set_pid(*self.params)
                    print(f"   🧠 AI Override → Kp={self.params[0]:.2f}, "
                          f"Ki={self.params[1]:.2f}, Kd={self.params[2]:.2f}")

                # Track stability
                if details.get('mae', 99) < STABLE_THRESHOLD:
                    consecutive_stable += 1
                else:
                    consecutive_stable = 0

        # Restore best params
        self.params = list(self.best_params)
        self.serial.set_pid(*self.params)
        print(f"\n   🏆 Best gains: Kp={self.params[0]:.3f}, "
              f"Ki={self.params[1]:.3f}, Kd={self.params[2]:.3f}")
        print(f"   🏆 Best cost:  {self.best_cost:.3f}")

    # ──────────────────────────────────────────────────────────
    #  PHASE 5: VERIFY — Hold optimal gains and confirm stability
    # ──────────────────────────────────────────────────────────

    def _phase_verify(self):
        print("\n" + "═" * 60)
        print("  PHASE 5 — VERIFICATION")
        print("═" * 60)

        self.serial.set_pid(*self.best_params)
        print(f"   Holding optimal gains for {STABLE_DURATION:.0f}s...")
        print(f"   Kp={self.best_params[0]:.3f}, "
              f"Ki={self.best_params[1]:.3f}, "
              f"Kd={self.best_params[2]:.3f}")

        self.buffer.clear()
        stable_start = time.time()
        stable_ok = True

        while time.time() - stable_start < STABLE_DURATION:
            if not self.safety.check():
                print("   ❌ Robot fell during verification!")
                stable_ok = False
                break

            data = self.buffer.get_window(1.0)
            if data and len(data['pitch']) > 10:
                mae = np.mean(np.abs(data['pitch']))
                sys.stdout.write(f"\r   ⏱️ {time.time()-stable_start:.0f}s "
                                 f"| Pitch MAE: {mae:.2f}° "
                                 f"| Current: {self.buffer.latest_pitch:+.1f}°   ")
                sys.stdout.flush()

            time.sleep(0.1)

        print()

        if stable_ok:
            # Final evaluation
            data = self.buffer.get_window(STABLE_DURATION * 0.8)
            cost, details = self.evaluator.evaluate(data)
            print(f"\n   ✅ VERIFICATION PASSED")
            print(f"   📊 Final MAE: {details['mae']:.2f}°")
            print(f"   📊 Final variance: {details['variance']:.2f}")
            print(f"   📊 Final max |pitch|: {details['max_pitch']:.1f}°")
            print(f"   📊 Final cost: {cost:.3f}")
        else:
            print(f"\n   ❌ Verification failed — robot not stable at these gains")

    # ──────────────────────────────────────────────────────────
    #  PHASE 6: REPORT — Save results
    # ──────────────────────────────────────────────────────────

    def _phase_report(self):
        print("\n" + "═" * 60)
        print("  RESULTS")
        print("═" * 60)

        elapsed = time.time() - self.start_time
        print(f"""
   ╔══════════════════════════════════════╗
   ║  OPTIMAL PID GAINS                  ║
   ╠══════════════════════════════════════╣
   ║  Kp = {self.best_params[0]:>8.3f}                    ║
   ║  Ki = {self.best_params[1]:>8.3f}                    ║
   ║  Kd = {self.best_params[2]:>8.3f}                    ║
   ╠══════════════════════════════════════╣
   ║  Cost = {self.best_cost:>7.3f}                     ║
   ║  Time = {elapsed:>5.0f}s                        ║
   ╚══════════════════════════════════════╝
""")

        # For firmware hardcoding
        print(f"   // Copy to firmware:")
        print(f"   float Kp = {self.best_params[0]:.4f};")
        print(f"   float Ki = {self.best_params[1]:.4f};")
        print(f"   float Kd = {self.best_params[2]:.4f};")

        # Save to JSON
        self._save_session()

        # Shutdown
        self._safe_shutdown()

    # ──────────────────────────────────────────────────────────
    #  INTERNAL HELPERS
    # ──────────────────────────────────────────────────────────

    def _run_trial(self, params):
        """
        Execute one trial: set PID gains, wait, evaluate performance.
        Returns the cost value.
        """
        # Ensure motors are on
        if not self.motors_on:
            self._enable_motors()

        # Clamp params
        params[0] = np.clip(params[0], KP_RANGE[0], KP_RANGE[1])
        params[1] = np.clip(params[1], KI_RANGE[0], KI_RANGE[1])
        params[2] = np.clip(params[2], KD_RANGE[0], KD_RANGE[1])

        # Send gains to firmware
        self.serial.set_pid(*params)
        self.safety.reset()

        # Wait for transient to settle
        self.buffer.clear()
        settle_end = time.time() + SETTLE_TIME
        while time.time() < settle_end:
            if not self.safety.check():
                return FALL_PENALTY
            time.sleep(0.02)

        # Clear buffer and collect evaluation data
        self.buffer.clear()
        eval_end = time.time() + EVAL_WINDOW
        while time.time() < eval_end:
            if not self.safety.check():
                break
            time.sleep(0.02)

        # Evaluate
        data = self.buffer.get_window(EVAL_WINDOW)
        cost, _ = self.evaluator.evaluate(data, fell=self.safety.fell)

        # If the robot fell, we need to recover
        if self.safety.fell:
            self._recover_from_fall()

        return cost

    def _enable_motors(self):
        """Enable motors safely."""
        if not self.reader.motors_on:
            self.serial.toggle_motors()
            time.sleep(0.2)
        self.motors_on = True
        self.safety.reset()

    def _recover_from_fall(self):
        """
        Handle robot fall: disable motors, wait, prompt user if needed.
        """
        # Make sure motors are off
        if self.reader.motors_on:
            self.serial.toggle_motors()
            time.sleep(0.2)
        self.motors_on = False

        print("\n   ⚠️ Robot fell — waiting 3s for you to stand it back up...")
        print("   (Hold it upright, then let go when motors enable)")
        time.sleep(3.0)

        # Re-enable motors
        self._enable_motors()

    def _safe_shutdown(self):
        """Disable everything cleanly."""
        print("\n   🔌 Shutting down...")
        try:
            self.serial.send("P0")
            self.serial.send("I0")
            self.serial.send("D0")
            if self.reader.motors_on:
                self.serial.toggle_motors()
            self.reader.stop()
            self.serial.close()
        except Exception:
            pass
        print("   ✅ Serial closed, motors OFF")

    def _save_session(self):
        """Save tuning session to JSON."""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'autotune_session_{timestamp}.json'
        filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)

        session = {
            'timestamp': timestamp,
            'best_params': {
                'Kp': self.best_params[0],
                'Ki': self.best_params[1],
                'Kd': self.best_params[2],
            },
            'best_cost': self.best_cost,
            'total_time_s': time.time() - self.start_time,
            'iterations': self.plot_data,
            'advisor_log': self.advisor.advice_log,
        }

        with open(filepath, 'w') as f:
            json.dump(session, f, indent=2, default=str)

        print(f"\n   💾 Session saved to {filename}")

    def _print_trial(self, iteration, param, direction, result, cost):
        """Print a single trial result line."""
        elapsed = time.time() - self.start_time
        p = self.params
        print(f"   [{iteration:>3d}] {elapsed:>5.0f}s | "
              f"{param} {direction:>8s} → {result} | "
              f"Cost={cost:>7.2f} | "
              f"Kp={p[0]:>6.2f} Ki={p[1]:>5.2f} Kd={p[2]:>5.2f} | "
              f"dp=[{self.dp[0]:.2f},{self.dp[1]:.2f},{self.dp[2]:.2f}]")

    def _log_iteration(self, iteration, cost):
        """Store data for plotting."""
        self.plot_data['iteration'].append(iteration)
        self.plot_data['cost'].append(cost)
        self.plot_data['kp'].append(self.params[0])
        self.plot_data['ki'].append(self.params[1])
        self.plot_data['kd'].append(self.params[2])

    def _setup_plot(self):
        """Set up matplotlib live plot (optional)."""
        try:
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation

            self.fig, self.axes = plt.subplots(2, 1, figsize=(10, 6))
            self.fig.suptitle('Autonomous PID Autotuner', fontweight='bold')
            self.fig.patch.set_facecolor('#1a1a2e')

            for ax in self.axes:
                ax.set_facecolor('#16213e')
                ax.tick_params(colors='white')

            self.axes[0].set_ylabel('Cost', color='white')
            self.axes[0].set_title('Optimization Progress', color='white')
            self.axes[1].set_ylabel('PID Gains', color='white')
            self.axes[1].set_xlabel('Iteration', color='white')

            plt.ion()
            plt.show()
        except ImportError:
            print("   ⚠️ matplotlib not available — skipping plot")
            self.enable_plot = False

    def _print_banner(self):
        print("""
╔══════════════════════════════════════════════════════════════╗
║                                                              ║
║   🤖  AUTONOMOUS PID AUTOTUNER                              ║
║   Self-Balancing Robot — Twiddle + AI Advisor                ║
║                                                              ║
║   Protocol: Serial @ 115200 baud                             ║
║   Algorithm: Twiddle (coordinate descent)                    ║
║   Intelligence: Heuristic pattern matching                   ║
║   Safety: Auto-stop at ±30° pitch                            ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝""")


# ════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description='Autonomous PID Autotuner')
    parser.add_argument('--port', type=str, default=DEFAULT_PORT,
                        help=f'Serial port (default: {DEFAULT_PORT})')
    parser.add_argument('--plot', action='store_true',
                        help='Enable live matplotlib plot')
    parser.add_argument('--kp', type=float, default=None,
                        help='Override initial Kp')
    parser.add_argument('--ki', type=float, default=None,
                        help='Override initial Ki')
    parser.add_argument('--kd', type=float, default=None,
                        help='Override initial Kd')
    parser.add_argument('--resume', type=str, default=None,
                        help='Resume from a saved session JSON file')

    args = parser.parse_args()

    tuner = PIDAutoTuner(port=args.port, enable_plot=args.plot)

    # Override initial params if specified
    if args.kp is not None:
        tuner.params[0] = args.kp
        INITIAL_PID[0] = args.kp
    if args.ki is not None:
        tuner.params[1] = args.ki
        INITIAL_PID[1] = args.ki
    if args.kd is not None:
        tuner.params[2] = args.kd
        INITIAL_PID[2] = args.kd

    # Resume from previous session
    if args.resume:
        try:
            with open(args.resume, 'r') as f:
                session = json.load(f)
            bp = session['best_params']
            tuner.params = [bp['Kp'], bp['Ki'], bp['Kd']]
            tuner.dp = [d * 0.5 for d in INITIAL_DP]  # Smaller steps on resume
            print(f"📂 Resuming from {args.resume}")
            print(f"   Kp={bp['Kp']:.3f}, Ki={bp['Ki']:.3f}, Kd={bp['Kd']:.3f}")
        except Exception as e:
            print(f"⚠️ Could not load session: {e}")

    tuner.run()


if __name__ == '__main__':
    main()
