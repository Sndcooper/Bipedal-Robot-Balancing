# Bipedal Robot — Status Recap & Roadmap (re-orientation after ~3 months away)

## Context

The user built this repo in a single intensive session on 2026-04-12 (all 5 commits landed that day: firmware bring-up, digital-twin IK tooling, PID auto-tuning, and full README documentation), then stepped away for roughly 3 months. They no longer remember the current state and need to be re-oriented before deciding what to do next. The end goal: a bipedal robot (two wheels at the tips of legs, each leg driven by a pair of AX-12 servos via 5-bar linkage) that stands stably without oscillation, using servo compliance as a balance-assist mechanism, and can eventually be remote-controlled to move forward/back/left/right like an RC vehicle. This plan is purely informational/advisory — a status report plus a phased roadmap — no code changes are proposed to be made yet in this turn.

## Current Status & Rework Architecture (`Balance_Rework`)

Since the initial historical assessment below, Phase A has been **implemented** in a dedicated, safety-first parallel workspace: [`Balance_Rework`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework). The original folders ([`PlatformIO_Firmware`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/PlatformIO_Firmware) and [`Python_Controller_Digital_Twin`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Python_Controller_Digital_Twin)) remain untouched as a historical fallback.

### 1. Active Firmware & Autotuner Features (`Balance_Rework`)
- **Safety Cutoff Latch**: Automatic motor cutoff and latch-off if tilt exceeds `MAX_SAFE_TILT` (default 35°), preventing wheel spinaway on falls. Requires explicit `M` serial re-enable.
- **IMU Complementary Filter**: Fuses MPU6050 accelerometer pitch with gyro Y-axis rate (`alpha = 0.98` default, live-tunable via `A`), eliminating tilt lag and noise.
- **Encoder Velocity Damping**: Subtracts `Kd_vel * wheelVelocity` from the PID output (`V` command), damping drift-driven oscillation.
- **5-Parameter Serial Tuning & Telemetry**: Live tuning for `P`, `I`, `D`, `V` (`Kd_vel`), `A` (`alpha`), and `T` (`MAX_SAFE_TILT`), streaming telemetry: `PITCH:<v>, PID_OUT:<v>, ENC_L:<v>, ENC_R:<v>, VEL:<v>, KDVEL:<v>, ALPHA:<v>, TILT:<v>`.
- **Real-Time Diagnostic GUIs**: Includes Python Desktop GUI ([`Balance_Rework/mpu_inspector/mpu_inspector_gui.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/mpu_inspector/mpu_inspector_gui.py)) and Web Serial Dashboard ([`Balance_Rework/mpu_inspector/mpu_inspector_web.html`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/mpu_inspector/mpu_inspector_web.html)) displaying an animated physical bipedal robot view, live Matplotlib/Chart.js telemetry graphs, RMS noise (`σ`), update rate (Hz), and interactive serial commands.
- **Safety-Aware Autotuner (`Balance_Rework/autotuner/`)**: Replaces old MAE/variance objective with [`cost.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/autotuner/cost.py) (amplitude, frequency, spectral concentration, time-to-settle), pauses on safety cutoff for physical reset, and performs 3× validation runs on winning gains.

### 2. Hardware Specifications & Reference Summary
Physical dimensions, linkage geometry, and pin mappings are documented in [`Robot_Specification.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Robot_Specification.md) and [`Hardware_Connections.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Hardware_Connections.md):

| Parameter / Subsystem | Measured / Configured Value | Primary Reference Source |
|---|---|---|
| **Body / Chassis Dimensions** | 148 × 118 × 89 mm servo box; 147 mm lower joint span | [`Robot_Specification.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Robot_Specification.md) |
| **Out-to-Out Wheel Width** | 270 mm (across wheels/shaft/motor assembly) | [`Robot_Specification.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Robot_Specification.md) |
| **Mass Breakdown (Known Subtotal)** | Total known ≈ 1074 g (Body 300g, LiPo 260g, 4× AX-12 214g, 2× Motors 300g) | [`Robot_Specification.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Robot_Specification.md) |
| **5-Bar Leg Linkage Geometry** | Femur 55 mm, Tibia 100 mm, Leg separation 180 mm | [`Robot_Specification.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Robot_Specification.md) |
| **AX-12 Servos (4x)** | Leg 1: IDs 6 (L), 14 (R); Leg 2: IDs 0 (L), 1 (R) | [`Robot_Specification.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Robot_Specification.md) |
| **Wheel Motors & Encoders** | 65 mm diameter wheels; L Motor (PA1/PB14/PB15), R Motor (PA0/PB12/PB13) | [`Hardware_Connections.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Hardware_Connections.md) |
| **IMU (MPU6050/9250)** | I2C1 (PB6=SCL, PB7=SDA); Gyro Y-axis rate fusion | [`Hardware_Connections.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Hardware_Connections.md) |

### 3. Immediate Next Milestone
- **Phase A Physical Hardware Tuning & Validation**: Flash [`Balance_Rework/firmware`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/firmware), verify IMU gyro sign via serial monitor, suspend robot in safety harness, and execute [`Balance_Rework/autotuner/autotune.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/autotuner/autotune.py) to converge on real-world stable gains (`Kp, Ki, Kd, Kd_vel, alpha`).

---

## Part 1 — Historical Baseline Analysis (Original Codebase State)

**Hardware**: STM32 "Bluepill" (STM32F103C8) is the brain, talking to an MPU6050 IMU over I2C, two DC wheel motors with quadrature encoders via H-bridge, and 4 AX-12+ Dynamixel servos (2 per leg, 5-bar linkage) over a half-duplex USART2 hack, all powered from a 12V LiPo. Full pinout is in `Hardware_Connections.md`.

**Firmware** (`PlatformIO_Firmware/src/main.cpp` — the *only* production file, 219 lines, never edited since its first commit):
- Tilt sensing is **accelerometer-only** (`readIMU()`, line 83–94): `atan2` on raw ax/ay/az, no gyro fusion despite the README implying one exists. This is noisy/laggy and is a prime suspect for the wobble.
- Balance is a **single-loop PID** (line 187–200): `Kp=20.0, Ki=0.5, Kd=1.0` hardcoded (line 32), output split evenly and inverted to both wheels. A comment at line 185 admits: *"Read Encoders to factor into PID (e.g. cascaded position/speed PID)... For now, simple standard balancing PID"* — i.e. you already knew this was incomplete.
- **Encoders are wired and counted but never used** in the control loop (dead code, telemetry only) — no velocity/position feedback exists, which on a wheeled inverted pendulum typically causes drift-driven oscillation.
- **Legs are locked rigid at boot** (`initAX12Legs()`, line 55–73): 4 servo IDs driven once to fixed goal positions with a tight Compliance Margin=1 / Slope=16. There is no dynamic leg control, no IK running on the STM32, and no compliance-based balancing algorithm actually implemented on hardware — compliance today is just a static register setting.
- Serial command surface is only single-letter tuning commands (`P/I/D/O/S/C/M` via `handleSerialTuning()`) — no directional/remote-control input exists anywhere yet.
- **Important discovery**: a richer command protocol (`POS/SPD/TRQ/TEN/LED/CMG/CSL/MOT`, raw Dynamixel passthrough) already exists, fully written, in `PlatformIO_Firmware/test/digitaltwin_current.cpp` (431 lines) — but it was **never merged into `main.cpp`**. This is unflashed, unused work already done.

**Python "Digital Twin"** (`Python_Controller_Digital_Twin/`):
- Despite the name, there's **no physics simulation** — it's pure geometric IK/FK for the 5-bar leg linkage with a Matplotlib GUI (`digital_tests/digital twin.py`, `digital twin 2_legs.py`), including sliders for foot position, servo torque, and compliance margin/slope, plus an "Adapt" checkbox that lowers torque limit to simulate compliance.
- `python tuner/autotune.py` (1120 lines) is a genuine, working Twiddle-style PID autotuner that talks to `main.cpp`'s real telemetry (`PITCH:`, `PID_OUT:`, `ENC_L/R:`) and already ran once successfully.
- **One completed tuning run exists** (`autotune_session_20260412_001035.json`): converged to `Kp=11.21, Ki=0.0, Kd=0.715` (cost 1.309) — **these were never copied back into `main.cpp`**, which still has the old untuned defaults. This is a zero-effort win sitting on the table.
- `tune.py` (manual jog tool) expects a telemetry format (`Roll: <v> | Kp: ...`) that `main.cpp` doesn't actually emit — it's stale/broken and not worth fixing right now.

**The core disconnect**: firmware and the "digital twin" tooling were built somewhat independently. The Python side's servo/IK protocol layer (`ax12_protocol.py`) assumes the fuller command set that only exists in the parked test file, not in the flashed firmware — so today, moving sliders in the digital-twin notebook would not actually move real servos. The IK visualization and the balance PID have never been wired together end-to-end.

**Bottom line**: the robot currently runs a bare single-loop, accel-only, encoder-blind tilt PID with rigid pinned legs — no wonder it wobbles. There are, however, several already-built pieces (tuned gains, a fuller servo protocol) sitting unused that can close a lot of the gap with modest effort before any new control theory is needed.

## Part 2 — Roadmap: from "wobbly stand" to "stable + compliance-assisted + RC-controllable"

Principle: smallest change that reduces wobble first, validated on hardware, before adding complexity. Don't touch the servo/IK merge until standing balance is solid — you won't be able to tell which change caused which effect otherwise.

### Phase A — Kill the wobble (**STATUS: CODE IMPLEMENTED IN `Balance_Rework` — NEXT STEP: HARDWARE TUNING**)
1. **[DONE] Safety Cutoff**: Implemented automatic tilt cutoff (`MAX_SAFE_TILT`) and latch-off in [`Balance_Rework/firmware/src/main.cpp`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/firmware/src/main.cpp).
2. **[DONE] Add gyro fusion**: Implemented complementary filter (`alpha`) fusing accelerometer pitch with gyro Y-axis rate in [`Balance_Rework/firmware/src/main.cpp`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/firmware/src/main.cpp).
3. **[DONE] Add encoder-derived velocity damping**: Implemented wheel velocity computation and damping term (`Kd_vel * wheelVelocity`) subtracted from PID output.
4. **[IMMEDIATE NEXT STEP] Physical Hardware Autotuning**: Suspend robot on harness and run [`Balance_Rework/autotuner/autotune.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/autotuner/autotune.py) to converge on and flash optimal 5-parameter gains.

### Phase B — Unify the firmware split
Merge `test/digitaltwin_current.cpp`'s command-parsing layer (`POS/SPD/TRQ/TEN/LED/CMG/CSL/MOT`) into `main.cpp`'s `handleSerialTuning()` dispatch (they operate on separate subsystems — servos vs. balance PID — so they coexist fine). Retire the duplicate packet-writer in `main.cpp` (lines 39–53) in favor of the test file's version. Make `initAX12Legs()` set an initial safe pose but leave compliance margin/slope runtime-adjustable via the merged `CMG`/`CSL` commands instead of hardcoded constants — this is the hook Phase C needs. Align `ax12_protocol.py` to whichever command framing survives. Once validated, archive `test/digitaltwin_current.cpp` so there's no ambiguity about which file is "the" firmware.

### Phase C — Compliance-angle balancing as a secondary behavior
Architecturally, wheels stay the primary, high-bandwidth balance actuator; leg-servo compliance is a slower, secondary shock-absorbing layer — not a replacement control loop (AX-12 compliance registers are a coarse mechanical-spring setting, not a fast digital position command). Add a low-rate (5–10Hz) watcher on tilt-error/PID-output variance in `main.cpp`; when disturbance magnitude crosses a threshold, loosen compliance slope (via the Phase B `CMG`/`CSL` commands) to let the legs absorb it, then tighten back toward rigid once calm — implemented as a small explicit state machine (`RIGID`/`COMPLIANT`), not a continuous PID. Validate by physically nudging the standing robot and comparing oscillation decay with/without dynamic compliance.

### Phase D — Remote directional control
Only after A–C produce stable standing. No new control theory needed:
- **Forward/back**: expose `targetAngle` (already exists, line 33) as a live RC-settable lean bias instead of only a tuning constant — commanding a forward lean makes the existing PID drive the wheels forward to catch it (Segway-style).
- **Left/right**: add a differential term to `setMotors(-output, -output)` (line 205), e.g. `setMotors(-output - turnBias, -output + turnBias)`.
- Reuse the Phase B text-command parser for a new `MOVE,<pitchBias>,<turnBias>` command, with a watchdog timeout (biases decay to 0 if no refresh within ~500ms) — critical fall-safety given this is an inverted pendulum.
- On the Python side, a small new keyboard-driven jog script (built against the *actual* current protocol, not `tune.py`'s stale format) is enough — no need for the full digital-twin visualization stack.

### Summary table

| Phase | Status | File(s) | Change |
|---|---|---|---|
| **A1–A3** | **[DONE] Implemented** | [`Balance_Rework/firmware/src/main.cpp`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/firmware/src/main.cpp) | Added automatic safety tilt cutoff, IMU gyro complementary filter (`alpha`), and encoder velocity damping (`Kd_vel`) |
| **A4** | **[IMMEDIATE NEXT STEP]** | [`Balance_Rework/autotuner/autotune.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/autotuner/autotune.py) | Run 5-parameter safety-aware autotuner on suspended physical robot & apply validated gains |
| **B** | Planned | `firmware/src/main.cpp` + legacy test files | Merge fuller AX-12 servo command parser (`POS/SPD/TRQ/CMG/CSL`) into main firmware loop |
| **B** | Planned | `digital_tests/ax12_protocol.py` | Align Python framing to merged command protocol |
| **C** | Planned | `firmware/src/main.cpp` | Low-rate compliance state machine (`CMG`/`CSL`) driven by disturbance variance |
| **D** | Planned | `firmware/src/main.cpp` | Live `targetAngle` + turn bias RC input with watchdog; differential `setMotors` |
| **D** | Planned | new directional jog script | Minimal directional jog sender against merged protocol |

## Verification approach
Each phase is a single flash-and-test cycle on the physical robot (supported off the ground first per the firmware README's PWM-surge warning, then hand-held/tethered standing tests). Use the existing serial telemetry (`PITCH:`, `PID_OUT:`, `ENC_L/R:`) and `autotune.py`'s cost function as the objective measure of "is wobble actually decreasing" rather than relying on visual judgment alone. No unit-testable code exists in this embedded context — validation is physical, incremental, and reversible (git commit after each phase so you can roll back if a change makes things worse).
