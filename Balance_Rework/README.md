# Balance_Rework

A self-contained rework of the balance control for the self-balancing bipedal robot
(STM32 Bluepill `bluepill_f103c8`, MPU6050 IMU, 2 DC wheel motors + quadrature encoders).

It exists as a **parallel workspace** so the originals stay untouched as a fallback:

- `PlatformIO_Firmware/` and `Python_Controller_Digital_Twin/` — **original, unmodified.**
- `Balance_Rework/firmware/` — a full PlatformIO project (copied from the original,
  same `env:bluepill_f103c8`, same pinout) with the control/safety changes applied.
- `Balance_Rework/autotuner/` — a new, safety-aware, 5-parameter PID tuning tool.
- `Balance_Rework/mpu_inspector/mpu_inspector_gui.py` — Python Desktop GUI Diagnostic tool with animated physical bipedal robot view, real-time Matplotlib charts, signal noise diagnostics (`σ`), and built-in simulation mode (`--mock`).
- `Balance_Rework/mpu_inspector/mpu_inspector_web.html` — Standalone HTML5 Web Serial GUI Dashboard (open in Chrome/Edge, zero install needed).
- `Balance_Rework/tuner_legcontrol/` — The ultimate unified GUI and firmware combining the PID balancer with live Inverse Kinematics (IK) for the AX-12 legs, velocity smoothing (EMA filter), and servo health monitoring.

Nothing here changes wiring: the pin map is identical to the root `Hardware_Connections.md`.

---

## 1. What's different from the original

### Firmware (`firmware/src/main.cpp`)

| Area | Original | Rework |
|---|---|---|
| **Safety** | none — a fall keeps driving the wheels | **Automatic tilt cutoff.** If `abs(pitch) > MAX_SAFE_TILT` (default **35°**), motors are forced to 0 and **latched OFF**, printing `SAFETY CUTOFF TRIGGERED`. Recovery requires an explicit `M` re-enable — no auto-recovery. |
| **IMU** | accelerometer-only pitch (`atan2`), noisy | **Complementary filter** fusing accelerometer with the gyro **Y-axis** rate (registers `0x45/0x46`, 131 LSB/°/s). `pitch = alpha*(pitch + gyroRate*dt) + (1-alpha)*accelPitch`. |
| **Control** | pitch PID only; encoders counted but never used | **Encoder wheel-velocity damping**: subtracts `Kd_vel * wheelVelocity` from the PID output. `Kd_vel` defaults to `0.0` (feature off until you sweep it). |
| **Tuning** | `P`/`I`/`D`/`O`/`S`/`C`/`M` | adds **`V`** (Kd_vel), **`A`** (alpha), **`T`** (MAX_SAFE_TILT), live over serial. |
| **Telemetry** | `PITCH:.. PID_OUT:.. ENC_L:.. <enc_r>` | same fields **in the same order**, plus appended `ENC_R:`, `VEL:`, `KDVEL:`, `ALPHA:`, `TILT:`. |

Full telemetry line now:
```
PITCH:<v>, PID_OUT:<v>, ENC_L:<v>, ENC_R:<v>, VEL:<v>, KDVEL:<v>, ALPHA:<v>, TILT:<v>
```

Two knobs worth knowing about:
- **`GYRO_PITCH_SIGN`** (`#define`, default `+1.0`) — the gyro contribution's sign depends
  on how your MPU is physically mounted. See the first-run checklist below for how to
  tell if it's wrong and how to fix it.
- **`MAX_SAFE_TILT_DEFAULT`** (`#define`, default `35.0`) seeds the live-tunable
  `maxSafeTilt`. Lower it while you're first testing, raise it once you trust the loop.

### Autotuner (`autotuner/`)

The old tuner (`Python_Controller_Digital_Twin/python tuner/autotune.py`) converged to
gains that *scored* well but *wobble* in reality. The root cause was the objective, not
the search: it scored trials with **MAE + variance + jitter**, which cannot distinguish a
slow gentle sway from a fast buzzing near-instability (they can have identical variance),
and it never measured whether the robot actually recovers from a disturbance.

The new tool fixes that:

1. **Better cost function** (`cost.py`) — scores each trial by oscillation **amplitude
   *and* frequency**, spectral concentration (is it one clean tone = sustained
   oscillation, or broadband noise?), and **time-to-settle** after a *repeatable
   commanded disturbance*. Every sub-component is logged so you can see *why* a trial
   scored badly. (Run `python cost.py` to see it rank synthetic wobbles.)
2. **Safety-aware** — detects `SAFETY CUTOFF TRIGGERED`, scores that trial as a hard
   failure, and **stops to let you physically reset the robot** before continuing. After
   `MAX_CONSEC_FAILURES` (3) cutoffs in a row it halts and asks whether the setup is
   still safe.
3. **Validation** — after the search, it re-runs the winning gains 3× and reports the
   spread. If the confirmation runs disagree with the search score (too high, too noisy,
   or any cutoff), it **flags the result** instead of trusting the optimizer's number.
4. **Tunes 5 parameters together** — `Kp, Ki, Kd, Kd_vel, alpha` (the old one did 3).
5. **Full logging** — every trial (not just the winner) is saved to a timestamped JSON in
   `autotuner/sessions/`, plus a paste-ready summary of the final gains.
6. **No hardcoded port** — pass `--port COMx` or you'll be prompted.

---

## 2. Physical setup you MUST do before running anything

The autotuner **will command the motors** and deliberately disturb the robot. Do not run
it with the robot free-standing on the first pass.

1. **Support stand / harness.** Suspend or clamp the robot so its body can tilt a little
   around the balance axis but **cannot actually fall over or drop**. A cord from an
   overhead point to the robot's top, or a bench clamp on the frame, both work. The wheels
   should be able to spin (or lightly touch the ground) without the whole robot toppling.
2. **Clear the swing path.** Keep hands, cables, and objects out of the arc the wheels and
   body will move through — a bad gain set *will* kick hard before the cutoff fires.
3. **Power.** Motor supply (≈12 V) on, all grounds common (see `Hardware_Connections.md`).
4. **Serial.** USB-TTL adapter on **Serial1 = PA9 (TX→adapter RX) / PA10 (RX→adapter TX)**,
   common GND. Note the COM port number.
5. **Start with a conservative cutoff.** For the very first runs, consider lowering the
   tilt limit, e.g. send `T25`, so the net catches earlier.

---

## 3. Exact order of operations (first-time run)

### Step A — flash the reworked firmware
```
cd Balance_Rework/firmware
pio run                 # compile (sanity check)
pio run -t upload       # flash the Bluepill (put it in upload/boot mode as usual)
```

### Step B — verify the IMU fusion sign (2 minutes, motors OFF)
Open a serial monitor at **115200** (`pio device monitor` in another terminal, or any
terminal program) and watch the `PITCH:` field:
- Slowly tilt the robot forward by hand and hold. `PITCH` should move smoothly and
  **settle at the true tilt**, then return to ~0 when you level it.
- If instead `PITCH` **runs away**, lags badly, or moves *opposite* to the real tilt, the
  gyro sign is wrong: open `firmware/src/main.cpp`, change
  `#define GYRO_PITCH_SIGN 1.0f` to `-1.0f`, re-flash, and re-check.
- Close the serial monitor before running the autotuner (only one program can own the
  port at a time).

### Step C — install Python deps
```
cd Balance_Rework/autotuner
pip install pyserial numpy
python cost.py          # optional: prove the cost function ranks wobbles sanely
```

### Step D — run the autotuner
```
python autotune.py --port COM7        # use your actual port
```
Then follow the prompts, **with your hand near the power switch**:
1. It prints a safety checklist — type `ready` only when the harness is set.
2. It asks you to hold the robot at its upright/balance angle and press ENTER to
   **calibrate the IMU** (motors stay off for this).
3. **Search phase** — it enables motors, applies a small repeatable disturbance each
   trial, and scores the response. Watch the live per-trial lines (`amp / freq / settle`).
4. On any `SAFETY CUTOFF`, it stops, tells you to stand the robot back upright, and waits
   for ENTER before re-enabling. After 3 cutoffs in a row it asks if it's safe to go on.
5. **Validation phase** — it re-runs the best gains 3× and tells you whether the result
   is trustworthy or **flagged**.
6. It prints a paste-ready block and saves the full session JSON.

### Step E — apply the result
Paste the reported globals into `firmware/src/main.cpp` and re-flash, **or** live-tune
them first with the printed `P.. I.. D.. V.. A..` commands to confirm on hardware before
committing to a flash.

> **Safety reminders:** motors only run while `M` is enabled; the firmware latches OFF on
> a fall and needs an explicit `M` (the autotuner handles this for you); and the tool
> always zeroes gains and disables motors on exit, Ctrl-C, or crash.

---

## 4. Unified Tuner & Leg Control (`tuner_legcontrol/`)

This folder contains the latest evolution of the project: integrating the balancing loop with the AX-12 leg inverse kinematics.

### Key Firmware Features (`firmware/src/main.cpp`)
- **Velocity EMA Low-Pass Filter**: A highly-optimized Exponential Moving Average filter (`velFilterAlpha = 0.15`) smooths the raw encoder velocity readings at 100Hz. This eliminates high-frequency quantization spikes ("Tk Tk Tk" chatter) when applying `Kd_vel` damping.
- **Strict Anti-Windup**: The PID integral term is robustly locked to `0.0` whenever the motors are disabled (or during a safety cutoff). Furthermore, the applied `Ki` instantly drops to `0.0` when motors are off, preventing any latent wind-up jerks upon re-enabling.
- **Non-Blocking Serial Protocol**: Supports high-frequency legacy PID tuning (`P`, `I`, `D`, `V`, etc.) alongside high-bandwidth multi-byte leg commands (`POS,id,val`, `TRQ,id,limit`, `CMP,id,margin,slope`) without choking the 100Hz balance loop.
- **Half-Duplex Echo Discarding**: Features a precise 10k-resistor hack workaround for the AX-12 UART that mathematically predicts and clears exactly 8 bytes of physical loopback echo before reading servo health diagnostics.

### Unified GUI (`gui/main_gui.py`)
- **Tab 1: Balance Tuner**: Live PID and telemetry charts. Includes a **"Reset Integral"** button, and parameter sliders specifically scaled for aggressive tuning (e.g., `Ki` up to 1000).
- **Tab 2: Kinematics & Health**: A 2D Matplotlib Digital Twin that solves bipedal Inverse Kinematics (IK) in real-time. When connected, dragging the foot `X/Y` sliders directly moves the physical AX-12 servos.
- **Live Health Diagnostics**: Background thread seamlessly polls the AX-12 servos for Load and Temperature, flashing red if a leg actuator crosses the 65°C danger threshold.
- **Profile Saving**: A **"Save Params"** button writes the complete current state of both the PID gains and leg IK geometry into a timestamped JSON file under `gui/profiles/`.
