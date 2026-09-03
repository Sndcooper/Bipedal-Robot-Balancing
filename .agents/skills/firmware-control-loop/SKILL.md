---
name: firmware-control-loop
description: How the STM32 100 Hz balance firmware works — the 3-layer cascaded PID (position→velocity→balance), IMU complementary filter, safety cutoff, encoder velocity, and the strict loop-timing budget. Read before changing balancing, safety, motor, or IMU code in any tuner_legcontrol variant.
---

# Firmware Control Loop (`firmware-control-loop`)

Applies to the MCU firmwares in `Balance_Rework/tuner_legcontrol/*/firmware/src/main.cpp`.
**Pin yourself to one variant** (see `robot-overview`) — this skill describes the
shared architecture, but exact defaults/format live in that variant's `main.cpp`.

## The loop is a hard 100 Hz gate

```cpp
unsigned long now = micros();
if (now - lastTime < 10000) return;   // enforce 100 Hz (10,000 µs period)
```

Everything below must fit in 10 ms. The AX-12 bus and radio are the expensive
parts, so the firmware deliberately **time-slices** them:
- A `READ/WRITE` toggle runs servo health polling on one tick and telemetry RX on
  the next → each subsystem effectively runs at **50 Hz**.
- Telemetry TX is gated to **20 Hz** (`now - lastPrintTime >= 50000`).
- RX draining is **non-blocking** and budget-capped (40 µs hard cap, or dynamic
  `clamp(avail/5,1,20)` bytes/tick depending on variant) so a burst of commands
  can never blow the 10 ms budget. See `serial-protocol` for details.

> When you add work to `loop()`, account for its worst-case µs. Anything that can
> block (a `Serial2.flush()`, a busy-wait, a `delay()`) inside the 100 Hz path is
> a bug. `initAX12Legs()` and `calibrateIMU()` block on purpose and only run
> outside the balance path (setup / explicit command).

## The 3-layer cascade (why it is not a single PID)

A balancing robot **cannot be commanded to a pitch directly** — leaning *is* how
it accelerates. So a drive command sets a *velocity* target, and the middle layer
converts velocity error into the small lean needed.

| Layer | Loop | Input → Output | Gains (command) |
|---|---|---|---|
| 3 outer | Position hold (P) | encoder-pos error → velocity target | `Kp_pos` (`PP`) |
| 2 middle | Velocity (PI) | velocity error → **tilt bias (deg)** | `Kp_vel` (`VP`), `Ki_vel` (`VI`) |
| 1 inner | Balance (PID) | pitch error → motor PWM | `Kp`,`Ki`,`Kd` |

```
targetAngle = gui_base_angle + tilt_bias (+ rc_gimbal_offset on RC variant)
error       = targetAngle - pitch
output      = Kp*error + Ki*integral + Kd*(-gyroRate)
left/right  = -output ± (spin/turn term)
```

Key design decisions already baked in (keep them):
- **Derivative on measurement** (`-gyroRate`), not on error — a slider/tilt-bias
  step must not inject a one-tick derivative spike.
- **Integrator STATE is clamped**, not just the output: `MAX_TILT_BIAS` on the
  velocity integrator; `MAX_INTEGRAL_PWM/Ki` on the balance integrator. Holding
  the robot upright or stalling a wheel must not let integral wind up and slam
  full PWM on release. When `Ki < 1e-6`, integral is forced to 0.
- The old `Kp_straight` term (fed on `encL-encR`) was **removed** — mirror-mounted
  encoders make that difference track *forward distance*, not heading, so it drove
  the robot in circles. Straight-line comes from the cascade; turning is explicit.

### Cascade state machine

```
        drive cmd                 |cmd| < 0.05
HOLDING ─────────► DRIVING ─────────────────────► RAMPDOWN
   ▲                  ▲                               │
   │                  └──────── drive cmd ────────────┘
   └──────────────── |vel| < 20 counts/s ─────────────┘
```

`RAMPDOWN` exists so the hold point is latched **only after the robot has
physically stopped** — latching on stick-release would pin it to a spot it is
still sliding past.

## IMU — complementary filter

```cpp
gyroRate = GYRO_PITCH_SIGN * gy / 131.0f;              // 131 LSB/°/s
pitch    = alpha*(pitch + gyroRate*dt) + (1-alpha)*accelPitch;
```
- `alpha` is live-tunable (`A` command). Accel gives the low-freq truth; gyro
  gives fast, drift-prone rate.
- `GYRO_PITCH_SIGN` (`#define`) depends on physical MPU mounting. **First-run
  check:** tilt the robot by hand, motors off, watch `P`/`PITCH` — it must move
  *with* the real tilt and settle, not run away or invert. If wrong, flip the
  define to `-1.0f` and re-flash. This is the single most common bring-up gotcha.
- `calibrateIMU()` averages 100 accel samples into `pitchOffset` (blocking, ~1 s);
  triggered by `C` command or RC Ch5 rising edge.

## Safety (do not weaken without asking)

- **Tilt cutoff latch:** if `|pitch| > maxSafeTilt` while armed, motors are forced
  to 0, `safetyLatched = true`, integrators zeroed, and the loop prints a cutoff
  message. **No auto-recovery** — re-arm requires an explicit `M` (or RC arm).
- `maxSafeTilt` is live-tunable (`T`). Start low (~25°) during bring-up.
- On disarm the firmware zeroes all integrators/encoders and latches the current
  encoder position as the hold target, so re-arming never sprints.
- AX-12 torque is toggled with the motor arm state (edge-detected) so legs go limp
  when disarmed.

## Encoder velocity

```cpp
deltaL =  (encL - prevL);
deltaR = -(encR - prevR);            // mirror-mount normalisation
vel_raw = ((deltaL+deltaR)*0.5)/dt;  // counts/sec, integer-quantised → noisy
vel_current = vel_alpha*vel_current + (1-vel_alpha)*vel_raw;   // EMA, `VA` cmd
```
Without the EMA, a 2-count/tick signal alternates 0/200/0/200 c/s at 100 Hz and
that ±100 c/s noise is amplified straight into `tilt_bias`. `vel_alpha≈0.85` ≈
15 Hz cutoff.

## Tuning workflow (physical, incremental, reversible)

1. Flash the variant, **verify `GYRO_PITCH_SIGN`** with motors off.
2. Suspend the robot in a harness (see `Balance_Rework/README.md` §2). Clear the swing path.
3. Live-tune over serial/GUI — start with inner loop (`P`,`I`,`D`), then velocity
   (`VP`,`VI`,`VA`), then position (`PP`). Watch `TB` (tilt bias) and `ST` (cascade state).
4. Commit good gains into `main.cpp` defaults and re-flash. `git commit` after each
   change so a worse gain set can be rolled back — there is no unit test here;
   validation is physical.

The `Balance_Rework/autotuner/` tool automates step 3 but targets the **older
`Balance_Rework/firmware` colon-telemetry protocol** (`PITCH:`,`PID_OUT:`), not
the tuner_legcontrol frames — confirm protocol compatibility before pointing it
at a tuner_legcontrol variant.
