"""
autotune.py — Balance_Rework 5-parameter PID autotuner (safety-aware).

Tunes  Kp, Ki, Kd, Kd_vel, alpha  together against the reworked STM32 firmware.

What makes this different from the old tuner (Python_Controller_Digital_Twin/python
tuner/autotune.py):

  * Cost function (cost.py) scores *wobbliness the way you see it* — oscillation
    amplitude AND frequency, spectral concentration, and time-to-settle after a
    repeatable commanded disturbance — instead of variance + MAE, which cannot tell a
    gentle sway from a buzzing near-instability.
  * Safety-aware: it watches for the firmware's "SAFETY CUTOFF TRIGGERED" line, scores
    that trial as a hard failure, and stops to let you physically reset the robot before
    continuing. It also halts for confirmation after too many cutoffs in a row.
  * Validation: after the search converges it re-runs the winning gains several times and
    reports the spread, so a lucky single trial can't masquerade as a real result.
  * Every per-trial score is logged with all its components to a timestamped JSON.

Usage:
    python autotune.py --port COM7
    python autotune.py                     (prompts for the port)
    python autotune.py --port COM7 --iterations 40 --window 7

Requires: pyserial, numpy.
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import numpy as np

from serial_link import SerialLink
import cost as costmod


# ════════════════════════════════════════════════════════════════════════════
#  SEARCH / TRIAL CONFIGURATION
# ════════════════════════════════════════════════════════════════════════════

PARAM_NAMES = ["Kp", "Ki", "Kd", "Kd_vel", "alpha"]

# Initial point = the current firmware defaults (a known-bad but stable-ish start).
INITIAL_PARAMS = [11.21, 0.0, 0.715, 0.0, 0.96]

# Coordinate-descent (Twiddle) step sizes, per parameter.
INITIAL_DP = [3.0, 0.20, 0.30, 0.40, 0.015]

# Hard clamps. alpha must stay a valid complementary blend; Kd_vel non-negative.
PARAM_RANGES = [
    (0.0, 60.0),     # Kp
    (0.0, 8.0),      # Ki
    (0.0, 12.0),     # Kd
    (0.0, 10.0),     # Kd_vel
    (0.80, 0.999),   # alpha
]

DP_GROW = 1.2
DP_SHRINK = 0.6

# Trial timing (seconds)
SETUP_WAIT = 0.4         # after pushing new gains, before disturbing
DISTURB_ANGLE = 5.0      # commanded target step (deg) — a repeatable "push"
DISTURB_TIME = 0.4       # how long the step is held before releasing to 0
EVAL_WINDOW = 7.0        # measurement window after release
POLL = 0.02

# Convergence / limits
DEFAULT_ITERATIONS = 40  # total single-parameter probes
CONVERGE_TOL = 0.10      # stop when sum(dp / initial_dp) < this

# Safety
MAX_CONSEC_FAILURES = 3  # cutoffs in a row before we pause for a safety check
SAFE_TILT_LIMIT = 35.0   # pushed to firmware "T" at startup

# Validation
VALIDATION_RUNS = 3
DISAGREE_FACTOR = 1.5    # mean validation cost > best*this  => flag
DISAGREE_CV = 0.40       # coeff. of variation across runs > this => flag


# ════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ════════════════════════════════════════════════════════════════════════════

def clamp_params(p):
    return [float(np.clip(p[i], PARAM_RANGES[i][0], PARAM_RANGES[i][1]))
            for i in range(len(p))]


def fmt_params(p):
    return (f"Kp={p[0]:.3f} Ki={p[1]:.3f} Kd={p[2]:.3f} "
            f"Kd_vel={p[3]:.3f} alpha={p[4]:.4f}")


def ask(prompt):
    """Blocking terminal confirmation."""
    try:
        return input(prompt)
    except EOFError:
        return ""


# ════════════════════════════════════════════════════════════════════════════
#  TUNER
# ════════════════════════════════════════════════════════════════════════════

class Tuner:
    def __init__(self, link, iterations, window):
        self.link = link
        self.iterations = iterations
        self.window = window
        self.cfg = costmod.CostConfig()

        self.params = clamp_params(list(INITIAL_PARAMS))
        self.dp = list(INITIAL_DP)
        self.best_params = list(self.params)
        self.best_cost = float("inf")
        self.best_components = {}

        self.consec_failures = 0
        self.trial_index = 0
        self.trials = []          # full per-trial log
        self.start_time = time.time()
        self.aborted = False

    # ---------------------------------------------------------------- #
    #  Setup phases
    # ---------------------------------------------------------------- #
    def phase_prep(self):
        print("\n" + "=" * 66)
        print("  PHASE 0 - PREP & SAFETY CHECK")
        print("=" * 66)
        print("  Before continuing, confirm ALL of the following:")
        print("   - The robot is held in its support stand / harness.")
        print("   - Wheels can spin freely without hitting the ground hard.")
        print("   - Nothing (and no hand) is in the path of a sudden motor kick.")
        print("   - The USB-TTL is on Serial1 (PA9/PA10) and this is the right port.")
        resp = ask("\n  Type 'ready' to begin: ").strip().lower()
        if resp != "ready":
            print("  Aborted at safety check.")
            self.aborted = True
            return False

        # Push the firmware safety tilt limit and make sure motors start OFF.
        self.link.set_tilt(SAFE_TILT_LIMIT)
        time.sleep(0.1)
        self.link.ensure_motors(False)
        self.link.set_gains(0, 0, 0, 0, INITIAL_PARAMS[4])
        time.sleep(0.2)
        return True

    def phase_wait_telemetry(self):
        print("\n  Waiting for telemetry stream...", end="", flush=True)
        t0 = time.time()
        while self.link.sample_count < 5:
            time.sleep(0.1)
            if time.time() - t0 > 10:
                print("\n  ERROR: no telemetry. Check wiring / COM port / baud.")
                return False
        print(f" got {self.link.sample_count} samples.")
        return True

    def phase_calibrate(self):
        print("\n" + "=" * 66)
        print("  PHASE 1 - IMU CALIBRATION (motors OFF)")
        print("=" * 66)
        ask("  Hold the robot at its balance-upright angle, then press ENTER...")
        print(f"  Pre-cal pitch:  {self.link.latest_pitch:+.2f} deg")
        self.link.calibrate()
        time.sleep(0.5)
        print(f"  Post-cal pitch: {self.link.latest_pitch:+.2f} deg")
        if abs(self.link.latest_pitch) > 4.0:
            print("  WARNING: post-cal pitch is large; re-seat the robot and recalibrate")
            if ask("  Recalibrate? [y/N]: ").strip().lower() == "y":
                self.link.calibrate()
                time.sleep(0.5)
                print(f"  Post-cal pitch: {self.link.latest_pitch:+.2f} deg")
        return True

    # ---------------------------------------------------------------- #
    #  Core trial
    # ---------------------------------------------------------------- #
    def run_trial(self, params, phase="search"):
        """Set gains, apply a repeatable disturbance, measure, score. Returns dict."""
        params = clamp_params(params)
        self.trial_index += 1
        idx = self.trial_index

        # Make sure motors are live (recover from any latched cutoff first).
        if not self.link.motors_on:
            self._recover(reason="motors were off at trial start")
            if self.aborted:
                return None

        self.link.set_gains(*params)
        time.sleep(SETUP_WAIT)

        # Repeatable commanded disturbance: lean to +DISTURB_ANGLE, then release to 0.
        self.link.arm_cutoff_watch()
        self.link.set_target(DISTURB_ANGLE)
        if self._wait_watching(DISTURB_TIME):
            return self._finish_trial(idx, phase, params, cutoff=True)

        self.link.set_target(0.0)
        self.link.clear()  # window starts at the release instant → clean settle metric

        if self._wait_watching(self.window):
            return self._finish_trial(idx, phase, params, cutoff=True)

        return self._finish_trial(idx, phase, params, cutoff=False)

    def _wait_watching(self, duration):
        """Sleep `duration`, returning True early if a safety cutoff fires."""
        end = time.time() + duration
        while time.time() < end:
            if self.link.cutoff_since():
                return True
            time.sleep(POLL)
        return self.link.cutoff_since()

    def _finish_trial(self, idx, phase, params, cutoff):
        snap = self.link.snapshot()
        result = costmod.evaluate(
            snap["t"], snap["pitch"], snap["pid_out"], cutoff=cutoff, cfg=self.cfg
        )
        record = {
            "index": idx,
            "phase": phase,
            "params": {PARAM_NAMES[i]: params[i] for i in range(len(params))},
            "cost": result["cost"],
            "cutoff": bool(cutoff),
            "components": result["components"],
            "n_samples": len(snap["pitch"]),
            "elapsed_s": round(time.time() - self.start_time, 1),
        }
        self.trials.append(record)
        self._print_trial(record)

        if cutoff:
            self.consec_failures += 1
            self._recover(reason="SAFETY CUTOFF during trial")
        else:
            self.consec_failures = 0

        return record

    def _print_trial(self, r):
        c = r["components"]
        tag = "CUTOFF" if r["cutoff"] else "ok"
        line = (f"  [{r['index']:>3}] {r['phase']:<10} cost={r['cost']:>8.2f} [{tag}]  "
                f"{fmt_params([r['params'][n] for n in PARAM_NAMES])}")
        print(line)
        if not r["cutoff"]:
            print(f"        amp={c.get('amplitude_deg',0):.2f}deg "
                  f"freq={c.get('dominant_freq_hz',0):.2f}Hz "
                  f"conc={c.get('spectral_concentration',0):.2f} "
                  f"settle={c.get('settle_time_s',0):.2f}s "
                  f"| osc={c.get('contrib_osc',0):.1f} "
                  f"rms={c.get('contrib_rms',0):.1f} "
                  f"set={c.get('contrib_settle',0):.1f} "
                  f"sat={c.get('contrib_sat',0):.1f}")

    # ---------------------------------------------------------------- #
    #  Fall recovery
    # ---------------------------------------------------------------- #
    def _recover(self, reason):
        # Firmware has already latched motors off on a real cutoff; make sure.
        self.link.set_gains(0, 0, 0, 0, self.params[4])
        if self.link.motors_on:
            self.link.ensure_motors(False)

        print(f"\n  !! {reason}. Motors are OFF.")

        if self.consec_failures >= MAX_CONSEC_FAILURES:
            print(f"  !! {self.consec_failures} cutoffs in a row.")
            resp = ask("  Is the physical setup still SAFE to continue? type 'yes': ")
            if resp.strip().lower() != "yes":
                print("  Operator ended the session for safety.")
                self.aborted = True
                return
            self.consec_failures = 0

        ask("  Stand the robot upright in the harness, then press ENTER to re-enable...")
        self.link.set_target(0.0)
        # Restore the current search gains and re-enable motors.
        self.link.set_gains(*clamp_params(self.params))
        self.link.arm_cutoff_watch()
        ok = self.link.ensure_motors(True)
        if not ok:
            print("  WARNING: could not confirm motors re-enabled; retrying toggle.")
            self.link.toggle_motors()
        time.sleep(0.3)

    # ---------------------------------------------------------------- #
    #  Twiddle search
    # ---------------------------------------------------------------- #
    def search(self):
        print("\n" + "=" * 66)
        print("  PHASE 2 - SEARCH (5-parameter coordinate descent)")
        print("=" * 66)

        # First motor enable is an explicit, deliberate gate.
        print("  Motors are about to be ENABLED for the first time.")
        ask("  Confirm the harness is holding the robot, then press ENTER to enable...")
        self.link.set_gains(*clamp_params(self.params))
        self.link.arm_cutoff_watch()
        self.link.ensure_motors(True)
        time.sleep(0.3)

        base = self.run_trial(self.params, phase="baseline")
        if self.aborted:
            return
        self.best_cost = base["cost"]
        self.best_params = list(self.params)
        self.best_components = base["components"]
        print(f"\n  Baseline cost = {self.best_cost:.2f}\n")

        dp0 = list(INITIAL_DP)
        probes = 0
        i = 0
        n = len(self.params)

        while probes < self.iterations:
            if self.aborted:
                return
            norm = sum(self.dp[k] / dp0[k] for k in range(n))
            if norm < CONVERGE_TOL:
                print(f"\n  Converged: sum(dp/dp0) = {norm:.3f} < {CONVERGE_TOL}")
                break

            # Probe parameter i upward.
            self.params[i] += self.dp[i]
            self.params = clamp_params(self.params)
            probes += 1
            r = self.run_trial(self.params, phase=f"probe:{PARAM_NAMES[i]}+")
            if self.aborted:
                return

            if r["cost"] < self.best_cost:
                self.best_cost = r["cost"]
                self.best_params = list(self.params)
                self.best_components = r["components"]
                self.dp[i] *= DP_GROW
            else:
                # Probe downward.
                self.params[i] -= 2 * self.dp[i]
                self.params = clamp_params(self.params)
                probes += 1
                r = self.run_trial(self.params, phase=f"probe:{PARAM_NAMES[i]}-")
                if self.aborted:
                    return
                if r["cost"] < self.best_cost:
                    self.best_cost = r["cost"]
                    self.best_params = list(self.params)
                    self.best_components = r["components"]
                    self.dp[i] *= DP_GROW
                else:
                    # Neither helped: revert and shrink this parameter's step.
                    self.params[i] += self.dp[i]
                    self.params = clamp_params(self.params)
                    self.dp[i] *= DP_SHRINK

            i = (i + 1) % n

        # Leave the firmware holding the best gains.
        self.params = list(self.best_params)
        self.link.set_gains(*clamp_params(self.params))
        print(f"\n  Search winner: cost={self.best_cost:.2f}  {fmt_params(self.best_params)}")

    # ---------------------------------------------------------------- #
    #  Validation
    # ---------------------------------------------------------------- #
    def validate(self):
        print("\n" + "=" * 66)
        print(f"  PHASE 3 - VALIDATION ({VALIDATION_RUNS} confirmation runs at winner)")
        print("=" * 66)

        runs = []
        for k in range(VALIDATION_RUNS):
            if self.aborted:
                break
            r = self.run_trial(self.best_params, phase=f"validate:{k+1}")
            if self.aborted:
                break
            runs.append(r)

        costs = [r["cost"] for r in runs]
        summary = {
            "runs": runs,
            "search_best_cost": self.best_cost,
            "mean_cost": None,
            "std_cost": None,
            "cv": None,
            "flagged": False,
            "reasons": [],
        }
        if costs:
            mean = float(np.mean(costs))
            std = float(np.std(costs))
            cv = std / mean if mean > 0 else 0.0
            summary.update(mean_cost=mean, std_cost=std, cv=cv)

            n_cutoffs = sum(1 for r in runs if r["cutoff"])
            if n_cutoffs:
                summary["flagged"] = True
                summary["reasons"].append(
                    f"{n_cutoffs}/{len(runs)} validation runs hit the safety cutoff")
            if mean > self.best_cost * DISAGREE_FACTOR:
                summary["flagged"] = True
                summary["reasons"].append(
                    f"mean validation cost {mean:.1f} >> search cost "
                    f"{self.best_cost:.1f} (x{mean/max(self.best_cost,1e-6):.2f}) "
                    "- the search score was optimistic / noisy")
            if cv > DISAGREE_CV:
                summary["flagged"] = True
                summary["reasons"].append(
                    f"high run-to-run spread (CV={cv:.2f}) - result is not repeatable")

            print(f"\n  validation costs: {[round(c,1) for c in costs]}")
            print(f"  mean={mean:.2f}  std={std:.2f}  cv={cv:.2f}")
            if summary["flagged"]:
                print("  >> RESULT FLAGGED:")
                for rsn in summary["reasons"]:
                    print(f"     - {rsn}")
            else:
                print("  >> Validation consistent with the search result. Looks real.")
        else:
            print("  No validation runs completed.")

        self.validation = summary
        return summary


# ════════════════════════════════════════════════════════════════════════════
#  REPORTING
# ════════════════════════════════════════════════════════════════════════════

def save_session(tuner, port, path_dir):
    os.makedirs(path_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = os.path.join(path_dir, f"session_{stamp}.json")
    session = {
        "timestamp": stamp,
        "port": port,
        "aborted": tuner.aborted,
        "config": {
            "initial_params": {PARAM_NAMES[i]: INITIAL_PARAMS[i] for i in range(5)},
            "param_ranges": {PARAM_NAMES[i]: PARAM_RANGES[i] for i in range(5)},
            "iterations": tuner.iterations,
            "eval_window_s": tuner.window,
            "disturb_angle_deg": DISTURB_ANGLE,
            "disturb_time_s": DISTURB_TIME,
            "cost_weights": costmod.config_dict(tuner.cfg),
            "safe_tilt_limit": SAFE_TILT_LIMIT,
        },
        "best": {
            "params": {PARAM_NAMES[i]: tuner.best_params[i] for i in range(5)},
            "cost": tuner.best_cost,
            "components": tuner.best_components,
        },
        "validation": getattr(tuner, "validation", None),
        "trials": tuner.trials,
    }
    with open(fname, "w", encoding="utf-8") as f:
        json.dump(session, f, indent=2, default=str)
    return fname


def print_summary(tuner, session_file):
    bp = tuner.best_params
    val = getattr(tuner, "validation", None)

    print("\n" + "=" * 66)
    print("  RESULT")
    print("=" * 66)
    print(f"\n  Best gains (search cost {tuner.best_cost:.2f}):")
    print(f"    {fmt_params(bp)}")

    if val and val.get("mean_cost") is not None:
        print(f"\n  Validation: mean cost {val['mean_cost']:.2f} "
              f"(std {val['std_cost']:.2f}, cv {val['cv']:.2f})")
        if val["flagged"]:
            print("  *** THIS RESULT IS FLAGGED — do not trust it blindly: ***")
            for rsn in val["reasons"]:
                print(f"      - {rsn}")
        else:
            print("  Validation consistent — result looks trustworthy.")

    print("\n  ---- Paste these into Balance_Rework/firmware/src/main.cpp globals ----")
    print(f"  float Kp     = {bp[0]:.4f};")
    print(f"  float Ki     = {bp[1]:.4f};")
    print(f"  float Kd     = {bp[2]:.4f};")
    print(f"  float Kd_vel = {bp[3]:.4f};")
    print(f"  float alpha  = {bp[4]:.4f};")
    print("  ----------------------------------------------------------------------")
    print("  (or live-tune with:  "
          f"P{bp[0]:.4f}  I{bp[1]:.4f}  D{bp[2]:.4f}  V{bp[3]:.4f}  A{bp[4]:.4f} )")
    print(f"\n  Full per-trial log saved to:\n    {session_file}\n")


# ════════════════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="Balance_Rework 5-param safety-aware autotuner")
    ap.add_argument("--port", type=str, default=None, help="serial port (e.g. COM7). Prompts if omitted.")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--iterations", type=int, default=DEFAULT_ITERATIONS,
                    help="max single-parameter probes in the search")
    ap.add_argument("--window", type=float, default=EVAL_WINDOW,
                    help="measurement window per trial (seconds)")
    ap.add_argument("--kp", type=float, default=None)
    ap.add_argument("--ki", type=float, default=None)
    ap.add_argument("--kd", type=float, default=None)
    ap.add_argument("--kd_vel", type=float, default=None)
    ap.add_argument("--alpha", type=float, default=None)
    args = ap.parse_args()

    # Apply initial-gain overrides.
    for i, v in enumerate([args.kp, args.ki, args.kd, args.kd_vel, args.alpha]):
        if v is not None:
            INITIAL_PARAMS[i] = v

    port = args.port or ask("Serial port (e.g. COM7): ").strip()
    if not port:
        print("No port given. Exiting.")
        sys.exit(1)

    link = SerialLink(port, args.baud)
    try:
        link.connect()
    except Exception as e:
        print(f"Could not open {port}: {e}")
        sys.exit(1)

    tuner = Tuner(link, iterations=args.iterations, window=args.window)
    session_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sessions")

    try:
        if not tuner.phase_prep():
            raise SystemExit
        if not tuner.phase_wait_telemetry():
            raise SystemExit
        tuner.phase_calibrate()
        tuner.search()
        if not tuner.aborted:
            tuner.validate()
    except (KeyboardInterrupt, SystemExit):
        print("\n\n  Interrupted — shutting down safely.")
        tuner.aborted = True
    except Exception as e:
        import traceback
        traceback.print_exc()
        tuner.aborted = True
        print(f"\n  Fatal error: {e}")
    finally:
        # Always leave the robot safe: zero gains, motors off.
        try:
            link.set_gains(0, 0, 0, 0, INITIAL_PARAMS[4])
            link.ensure_motors(False)
        except Exception:
            pass
        session_file = save_session(tuner, port, session_dir)
        if tuner.trials:
            print_summary(tuner, session_file)
        else:
            print(f"\n  Session (no trials) saved to {session_file}")
        link.close()


if __name__ == "__main__":
    main()
