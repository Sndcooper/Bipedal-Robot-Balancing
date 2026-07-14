# Balance_Rework Autotuner

Safety-aware, 5-parameter (`Kp, Ki, Kd, Kd_vel, alpha`) PID tuner for the reworked
firmware. See `../README.md` for the full setup and first-run procedure — this file just
documents the code.

## Files

| File | Role |
|---|---|
| `autotune.py` | Orchestrator: connect → calibrate → coordinate-descent search → validation → report. Handles safety cutoffs and JSON logging. Run this. |
| `serial_link.py` | Thread-safe serial transport + background telemetry parser. Watches for `SAFETY CUTOFF TRIGGERED`. |
| `cost.py` | The trial-scoring logic (importable & self-testing). This is where "what counts as a good balance" is defined. Run `python cost.py` to see it rank synthetic wobbles. |
| `sessions/` | Timestamped per-session JSON logs (every trial, not just the winner). |

## Run

```
pip install pyserial numpy
python autotune.py --port COM7
```

Useful flags: `--iterations N` (search budget), `--window S` (measurement seconds per
trial), `--kp/--ki/--kd/--kd_vel/--alpha` (override the starting point).

## GUI tuner

If you want a live desktop control panel instead of the CLI autotuner, run:

```
python balance_tuner_gui.py --port COM3
```

The GUI reuses the same firmware protocol and shows the firmware's serial output, live telemetry, and per-parameter coarse/fine sliders for `Kp`, `Ki`, `Kd`, `Kd_vel`, `alpha`, `targetAngle`, `pitchOffset`, and `maxSafeTilt`.

## Tuning what "good" means

If the tuner's ranking doesn't match your eye, edit the weights in
`cost.py :: CostConfig` (not the firmware):

- `w_osc` — sustained-oscillation penalty (the dominant term).
- `freq_weight` — how much faster wobbles are punished vs. slow ones.
- `w_settle` — how much recovery time after the disturbance matters.
- `settle_band` — the `±deg` window that counts as "settled".
- `w_sat` / `w_motor` — discourage PWM saturation / thrashing.

Every trial's JSON records the exact weights used, so results stay reproducible.

## What the tuner does to the robot

Each trial: pushes the 5 gains, commands a small repeatable disturbance
(`target → +5°` for 0.4 s, then release to 0), and measures the ~7 s recovery window.
A firmware safety cutoff during any trial = maximum cost + a stop to reset the robot.
On exit/crash/Ctrl-C it always zeroes gains and disables motors.
