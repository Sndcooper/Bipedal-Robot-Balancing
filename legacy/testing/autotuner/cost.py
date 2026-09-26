"""
cost.py — Trial scoring for the Balance_Rework autotuner.

WHY THIS EXISTS
---------------
The original tuner scored a trial with:  MAE + variance + max_pitch + motor + jitter.
That is why its "winners" wobble on real hardware:

  * Variance can't tell a slow 8-degree rock apart from a fast 1-degree jitter — both
    can have similar variance, but one is a gentle sway you could live with and the
    other is the buzzing near-instability that wrecks gears and looks awful.
  * It never measured whether the robot RECOVERS from a disturbance (settling time),
    only how big the error was on average.
  * "jitter" = mean |d(pitch)/dt| conflates a fast small tremor with a big slow swing
    scaled by loop rate — it does not isolate a *sustained oscillation*.

This module instead characterises the pitch signal the way a human watching the robot
would: how big is the wobble, how fast is it, is it a clean sustained oscillation (bad)
or broadband noise, and how long does it take to settle after a disturbance.

Every component is returned separately so a trial's score can be *explained*, not just
trusted.

USAGE
-----
    from cost import evaluate, CostConfig
    result = evaluate(t, pitch, pid_out, cutoff=False, disturbance_release_t=t0)
    result["cost"]          # scalar, lower is better
    result["components"]    # dict of every raw sub-metric + its weighted contribution
"""

from dataclasses import dataclass, asdict, field
import numpy as np


@dataclass
class CostConfig:
    # Target pitch the controller is trying to hold.
    target: float = 0.0

    # Settling band: |pitch - target| must stay inside this (deg) to count as "settled".
    settle_band: float = 2.0

    # Ignore FFT content below this frequency as slow drift, above Nyquist is impossible.
    min_osc_hz: float = 0.25

    # --- Weights (tune these, not the firmware, to change what "good" means) ---
    w_rms: float = 1.0          # overall wobble energy (RMS of detrended pitch, deg)
    w_osc: float = 2.5          # sustained-oscillation penalty (the big one)
    w_settle: float = 1.5       # seconds-to-settle after the commanded disturbance
    w_mae: float = 0.4          # mean absolute error vs target (steady-state offset)
    w_motor: float = 0.3        # RMS motor effort (0..1), discourages thrashing
    w_sat: float = 4.0          # fraction of samples with saturated PWM (0..1)

    # Frequency weighting inside the oscillation term: a wobble of a given amplitude is
    # scored worse the faster it is (fast jitter ≈ near-instability, hardware-destroying).
    freq_weight: float = 0.6

    # Penalty added when the firmware safety cutoff fired during the trial. Must dominate
    # any achievable "good" cost so a fall is never mistaken for a decent result.
    cutoff_penalty: float = 1000.0

    # Penalty when there simply isn't enough usable data (dropout / disconnect).
    nodata_penalty: float = 900.0

    # Minimum samples to attempt scoring.
    min_samples: int = 20


def _resample_uniform(t, y):
    """Interpolate an unevenly-sampled signal onto a uniform grid at its median rate."""
    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)
    t = t - t[0]
    dts = np.diff(t)
    dts = dts[dts > 1e-6]
    if len(dts) == 0:
        return None, None, None
    dt = float(np.median(dts))
    n = int(np.floor(t[-1] / dt)) + 1
    if n < 4:
        return None, None, None
    grid = np.arange(n) * dt
    yu = np.interp(grid, t, y)
    return grid, yu, dt


def _spectral(y_detrended, dt):
    """Return (dominant_freq_hz, spectral_concentration) of a detrended signal.

    spectral_concentration = power in the dominant bin / total AC power, i.e. how much
    of the wobble is one clean tone (a sustained oscillation) vs. spread-out noise.
    """
    n = len(y_detrended)
    if n < 4:
        return 0.0, 0.0
    win = np.hanning(n)
    spec = np.abs(np.fft.rfft(y_detrended * win))
    freqs = np.fft.rfftfreq(n, d=dt)
    power = spec ** 2
    # Drop DC bin.
    if len(power) <= 1:
        return 0.0, 0.0
    ac_power = power[1:]
    ac_freqs = freqs[1:]
    total = float(np.sum(ac_power))
    if total <= 0:
        return 0.0, 0.0
    k = int(np.argmax(ac_power))
    dom_freq = float(ac_freqs[k])
    concentration = float(ac_power[k] / total)
    return dom_freq, concentration


def _zero_cross_rate(y_detrended, dt):
    """Zero crossings per second — a cheap, FFT-independent frequency sanity check."""
    n = len(y_detrended)
    if n < 2:
        return 0.0
    signs = np.sign(y_detrended)
    signs[signs == 0] = 1
    crossings = int(np.sum(signs[1:] != signs[:-1]))
    duration = dt * (n - 1)
    if duration <= 0:
        return 0.0
    return crossings / duration / 2.0  # 2 crossings per oscillation cycle


def _settle_time(t, pitch, cfg):
    """Time (s) from window start until |pitch-target| stays within settle_band forever.

    The trial commands a repeatable disturbance and then releases to target; the window
    passed in should begin at that release. Returns the full window length if it never
    settles (i.e. it never stops wobbling)."""
    t = np.asarray(t, dtype=float)
    pitch = np.asarray(pitch, dtype=float)
    t = t - t[0]
    err = np.abs(pitch - cfg.target)
    within = err <= cfg.settle_band
    # Find the last index that is OUT of band; settle time is the time just after it.
    out_idx = np.where(~within)[0]
    if len(out_idx) == 0:
        return 0.0  # already settled the whole time
    last_out = out_idx[-1]
    if last_out >= len(t) - 1:
        return float(t[-1])  # still out of band at the end → never settled
    return float(t[last_out + 1])


def evaluate(t, pitch, pid_out, cutoff=False, cfg=None):
    """Score one trial window. Lower cost is better.

    Args:
        t:        list/array of sample timestamps (seconds, monotonic).
        pitch:    list/array of pitch angles (deg).
        pid_out:  list/array of PID output values sent to motors (approx -255..255).
        cutoff:   True if the firmware safety cutoff fired during the trial.
        cfg:      optional CostConfig.

    Returns dict with keys: cost, components, ok.
    """
    cfg = cfg or CostConfig()

    if cutoff:
        # A fall dominates everything. Still report whatever partial stats we can.
        comp = {"cutoff": 1.0, "note": "SAFETY CUTOFF - trial counted as failure"}
        return {"cost": cfg.cutoff_penalty, "components": comp, "ok": False}

    pitch = np.asarray(pitch, dtype=float)
    pid_out = np.asarray(pid_out, dtype=float) if len(pid_out) else np.zeros_like(pitch)

    if len(pitch) < cfg.min_samples:
        return {
            "cost": cfg.nodata_penalty,
            "components": {"note": f"insufficient data ({len(pitch)} samples)"},
            "ok": False,
        }

    grid, yu, dt = _resample_uniform(t, pitch)
    if yu is None:
        return {
            "cost": cfg.nodata_penalty,
            "components": {"note": "could not resample (bad timestamps)"},
            "ok": False,
        }

    detrended = yu - np.mean(yu)

    # --- Raw metrics ---------------------------------------------------------
    mae = float(np.mean(np.abs(pitch - cfg.target)))
    rms = float(np.sqrt(np.mean(detrended ** 2)))
    p2p = float(np.max(yu) - np.min(yu))
    amplitude = p2p / 2.0

    dom_freq, concentration = _spectral(detrended, dt)
    zcr = _zero_cross_rate(detrended, dt)
    # Only treat it as oscillation if it's above the slow-drift floor.
    osc_freq = dom_freq if dom_freq >= cfg.min_osc_hz else 0.0

    settle = _settle_time(t, pitch, cfg)

    motor_rms = float(np.sqrt(np.mean((pid_out / 255.0) ** 2)))
    sat_frac = float(np.mean(np.abs(pid_out) >= 254.0))

    # --- Oscillation penalty -------------------------------------------------
    # Amplitude of the wobble, amplified by (a) how fast it is and (b) how much of a
    # single clean tone it is. This is what separates a slow gentle sway (low freq, low
    # concentration → modest) from a buzzing near-instability (high freq, high
    # concentration → heavily penalised) even when their variance is identical.
    osc_penalty = amplitude * (1.0 + cfg.freq_weight * osc_freq) * (0.5 + concentration)

    # --- Weighted contributions ---------------------------------------------
    c_rms = cfg.w_rms * rms
    c_osc = cfg.w_osc * osc_penalty
    c_settle = cfg.w_settle * settle
    c_mae = cfg.w_mae * mae
    c_motor = cfg.w_motor * motor_rms
    c_sat = cfg.w_sat * sat_frac

    cost = c_rms + c_osc + c_settle + c_mae + c_motor + c_sat

    components = {
        # raw, human-readable
        "mae_deg": round(mae, 3),
        "rms_deg": round(rms, 3),
        "peak_to_peak_deg": round(p2p, 3),
        "amplitude_deg": round(amplitude, 3),
        "dominant_freq_hz": round(dom_freq, 3),
        "osc_freq_used_hz": round(osc_freq, 3),
        "spectral_concentration": round(concentration, 3),
        "zero_cross_hz": round(zcr, 3),
        "settle_time_s": round(settle, 3),
        "motor_rms": round(motor_rms, 3),
        "saturation_frac": round(sat_frac, 3),
        "cutoff": 0.0,
        # weighted contributions to the final cost (so you can see WHY)
        "contrib_rms": round(c_rms, 3),
        "contrib_osc": round(c_osc, 3),
        "contrib_settle": round(c_settle, 3),
        "contrib_mae": round(c_mae, 3),
        "contrib_motor": round(c_motor, 3),
        "contrib_sat": round(c_sat, 3),
    }

    return {"cost": float(cost), "components": components, "ok": True}


def config_dict(cfg):
    """Serialisable view of a CostConfig for the session log."""
    return asdict(cfg)


# --------------------------------------------------------------------------- #
#  Self-test: synthetic signals prove the cost function ranks wobbles sanely.
#  Run:  python cost.py
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    fs = 20.0
    dur = 6.0
    t = np.arange(0, dur, 1.0 / fs)
    tl = t.tolist()

    def show(name, pitch, pid=None):
        pid = pid if pid is not None else np.clip(pitch * 5, -255, 255)
        r = evaluate(tl, pitch.tolist(), np.asarray(pid).tolist())
        c = r["components"]
        print(f"\n{name}")
        print(f"  cost = {r['cost']:.2f}")
        print(f"  amp={c['amplitude_deg']:.2f}deg  freq={c['dominant_freq_hz']:.2f}Hz  "
              f"conc={c['spectral_concentration']:.2f}  settle={c['settle_time_s']:.2f}s")
        print(f"  contrib: osc={c['contrib_osc']:.2f} rms={c['contrib_rms']:.2f} "
              f"settle={c['contrib_settle']:.2f}")

    # The classic trap: two signals with IDENTICAL variance (same amplitude) but very
    # different frequency. A variance-only cost scores them equal; ours does not.
    slow_rock = 4.0 * np.sin(2 * np.pi * 0.4 * t)     # amp 4 deg @ 0.4 Hz (gentle sway)
    fast_shake = 4.0 * np.sin(2 * np.pi * 5.0 * t)    # amp 4 deg @ 5 Hz  (buzzing)
    print(f"variance  slow_rock={np.var(slow_rock):.3f}  fast_shake={np.var(fast_shake):.3f}"
          "   <- IDENTICAL, which is exactly why variance-only tuning fails")
    show("SLOW ROCK  (0.4 Hz, +/-4 deg)", slow_rock)
    show("FAST SHAKE (5.0 Hz, +/-4 deg)", fast_shake)

    # A well-behaved run that settles quickly.
    settled = np.concatenate([
        4.0 * np.exp(-t[t < 2] * 2.5) * np.cos(2 * np.pi * 1.0 * t[t < 2]),
        0.3 * np.random.RandomState(0).randn(len(t) - len(t[t < 2])),
    ])
    show("SETTLES FAST then quiet", settled)

    # A safety cutoff.
    r = evaluate(tl, slow_rock.tolist(), (slow_rock * 5).tolist(), cutoff=True)
    print(f"\nSAFETY CUTOFF -> cost = {r['cost']:.2f}  ({r['components']['note']})")
