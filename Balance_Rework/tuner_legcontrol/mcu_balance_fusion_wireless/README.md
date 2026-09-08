# Sensor-Fusion Balance Tuner (`mcu_balance_fusion_wireless`)

This directory starts as a **copy of the validated single-loop baseline**
([`mcu_ik_engine_pretest_wireless`](../mcu_ik_engine_pretest_wireless), tag
`pretest-wireless-single-loop-v1`) and is where Stage 2 work happens: better
pitch/velocity estimation via sensor fusion (encoder + IMU), on top of the
same proven AX-12 leg bus, SYNC_WRITE, and telemetry guarding.

> Nothing below has diverged from the baseline yet -- this is the starting
> point, not a description of what has been built. Update this README as
> fusion work lands.

> **History:** this folder previously held a pure 3DR radio latency benchmark.
> That role is preserved — the firmware still answers `PING:<token>` with
> `PONG:<token>`, so [`latency_test.py`](latency_test.py) continues to work — but
> the primary purpose is now single-loop balancing. The original benchmark
> firmware is recoverable from git history.

---

## STATUS: finished, working single-loop balancer (frozen baseline)

This variant balances the robot untethered on 3DR radio and is the validated
reference before Stage 2 (sensor fusion / cascade). Board: **genericSTM32F103CB**
(the C8T6 die is 128 KB; the C8 board file under-declares it — see below).

**Known-good gains** (from real bench sessions, auto-trim ON):

| Kp | Ki | Kd | alpha | ATE | result |
| --: | --: | --: | --: | :--: | --- |
| 95 | 697 | 3.10 | 0.96 | **1** | pitch sd 0.77 deg, cleanest run logged |
| 70 | 729.5 | 3.10 | 0.96 | **1** | pitch sd 1.27 deg |
| 85 | 666.5 | 3.34 | 0.96 | 0 | sd 2.31 deg, saturated 4.2% -- usable but rougher |

**Run with auto-trim (`TE1`) on.** Every `ATE:0` session in the tuning logs
ended in `SAFETY:CUTOFF`; every `ATE:1` session survived. Let `trim_bias`
settle (~10-15 s) before committing with `TC` -- committing early bakes in a
half-converged bias permanently.

**Fixed this round** (see `firmware/src/main.cpp` diff at commit `5847bdd`):
- Crouch (`CR<mm>`) now routes through the interpolated trajectory engine --
  previously it cached goals but never updated `cur_x/cur_y` or sync-wrote,
  so telemetry lied and the next pose move would slam the legs.
- AX-12 status **ERROR byte** (overheat/overload) is now captured and exposed
  as `SRV:id,temp,load,err,fail` -- previously discarded, so a servo latched
  in Alarm Shutdown looked identical to a healthy one (temp just froze).
- `initAX12Legs()` now applies the live compliance/torque globals instead of
  frozen struct defaults, so an `SR` reset no longer silently discards slider
  settings.
- Flash headroom: was at 99.4% of the declared 64 KB; `genericSTM32F103CB`
  unlocks the real 128 KB (now 49.7%).

**Known issues to carry into Stage 2, not re-discover:**
- `MAX_INTEGRAL_PWM = 1200` is 4.7x the actual PWM range (+/-255) -- anti-windup
  in name only. One logged session pinned it at the clamp.
- `Kp` around 85-95 is only linear to +/-2.7-3.0 deg pitch; ~15% of logged
  samples exceeded that and saturated the motors. A velocity/tilt-bias cascade
  (Stage 2) should let `Kp` drop and widen the linear range instead of relying
  on auto-trim to save it.
- **No persistence.** A board reset (brownout suspected -- motor + servo
  stall current on a shared rail) silently reverts `Kp/Ki/Kd/pitchOffset` to
  compiled defaults while the GUI keeps displaying stale values. Confirm via
  the firmware's own `Updated ->` line, not the GUI slider, after any
  unexplained behavior change.

---

## ⚡ Hardware Connections

```
                    ┌───────────────────────────┐
                    │   STM32F103C8T6 BLUEPILL  │
                    ├───────────────────────────┤
  L298N Motor ENA   │ PA1                   PA6 │ Left Encoder A (Interrupt)
  L298N Motor ENB   │ PA0                   PA7 │ Left Encoder B
  L298N Motor IN1   │ PB14                  PB0 │ Right Encoder A (Interrupt)
  L298N Motor IN2   │ PB15                  PB1 │ Right Encoder B
  L298N Motor IN3   │ PB12                  PB6 │ MPU6050 SCL (I2C1)
  L298N Motor IN4   │ PB13                  PB7 │ MPU6050 SDA (I2C1)
                    │                           │
 AX-12 Bus TX (1M)  │ PA2                  PB10 │ 3DR Radio TX (Serial3 @ 115k)
 AX-12 Bus RX (1M)  │ PA3                  PB11 │ 3DR Radio RX (Serial3 @ 115k)
                    └───────────────────────────┘
```

Encoders feed **velocity telemetry for display only** — velocity is never used
in the control law (that would make it more than one loop).

---

## 🎛️ Control — strictly one PID loop

```
error  = targetAngle - pitch
output = Kp*error + Ki*integral + Kd*(-gyroRate)      # derivative on measurement
setMotors(-output, -output)                            # no steering, pure balance
```

- Integrator state is anti-windup clamped to `MAX_INTEGRAL_PWM/Ki`; with `Ki=0`
  the integral is held at zero so no latent kick can bank up.
- Safety cutoff: if `|pitch| > maxSafeTilt` while armed, motors latch OFF until an
  explicit re-arm (`M`).
- `alpha` (complementary filter), `targetAngle` (setpoint), `maxSafeTilt` (safety)
  and the IMU offset are tunable but are **not** additional control loops.

---

## 📡 Protocol (comma/colon, newline-terminated)

**Telemetry @ 20 Hz** (parsed by `gui/serial_link.py`):
```text
PITCH:<p>,PID_OUT:<o>,INT:<i>,EL:<encL>,ER:<encR>,VEL:<v>,MOT:<0|1>,TILT:<t>,LATCH:<0|1>
SRV:<id>,<temp>,<load%>            # one servo per frame, round-robin
```
**Command ack:** `Updated -> P:.. I:.. D:.. Offset:.. Target:.. Alpha:.. Tilt:..`
**Calibration:** `CAL:START` … `CAL:DONE,OFFSET:..`

| Command | Meaning |
| :--- | :--- |
| `P<f>` `I<f>` `D<f>` | Balance PID gains (the one loop) |
| `S<f>` | Balance setpoint / operator trim |
| `A<f>` | Complementary-filter `alpha` |
| `T<f>` | Max safe tilt |
| `O<f>` | Manual pitch offset |
| `C` / `M` / `R` | Calibrate IMU / toggle motors / reset integral |
| `S` (bare) | Re-init legs to standing pose |
| `PING:<tok>` | Latency probe → `PONG:<tok>` |

---

## 📂 File Map

* **[`firmware/src/main.cpp`](firmware/src/main.cpp)**: single-loop balancer — MPU6050 complementary filter, one PID, L298N motors, safety cutoff, AX-12 leg hold + health poll, PING/PONG.
* **[`gui/main_gui.py`](gui/main_gui.py)**: single-screen balance + calibration GUI (telemetry plot, Kp/Ki/Kd/Target/alpha/Tilt, servo health, serial monitor).
* **[`gui/serial_link.py`](gui/serial_link.py)**: 3DR telemetry worker (single-loop command/telemetry contract).
* **[`latency_test.py`](latency_test.py)**: standalone PING/PONG latency + throughput benchmark (still supported).

---

## 🚀 Running

```bash
cd firmware && pio run -t upload      # BOOT0=1, reset, then BOOT0=0 to run
python gui/main_gui.py                # connect to the 3DR COM port @ 115200
```

**First-run:** with motors OFF, hand-tilt the robot and confirm `Angle` moves the
right way and settles; if it inverts/runs away, flip `GYRO_PITCH_SIGN` in
`main.cpp`. Tune on a harness before free-standing.

---

## Tick profiler (Serial1 → COM3)

**Instrumentation only.** Not one line of control logic was altered to add this:
no stage reordered, no timing changed, nothing optimised. Verified by stripping
every profiler line from the file and diffing the remainder against the previous
version — zero residual code differences. The numbers therefore describe the
firmware you already trust, which is the whole point of measuring it rather than
rewriting it.

Output goes to `Serial1` (**PA9 = TX**, 115200) — a plain wired UART, *not* the
3DR radio. Sending profiling data over the link whose airtime starvation you are
characterising would perturb the very thing being measured. Wire PA9 to a
USB-TTL RX and share ground; PA10 is unused, so the port is output-only and a
stray terminal keystroke can never reach the balancer.

### Stream 1 — one raw row per 100 Hz tick

```
P10001 B1603 F8397 I1340 K0 Y1 E14 C38 R210 V0   T0  X44
P10001 B2023 F7977 I1345 K0 Y1 E14 C38 R205 V420 T0  X44
P11740 B3106 F6894 I1341 K0 Y1 E14 C38 R1712 V0  T0  X44 !OVR
```

| Col | Stage |
| --- | --- |
| `P` | tick period — the 100 Hz check, target 10000 |
| `B` | loop body total — **budget consumed** |
| `F` | free left of the 10000 |
| `I` | `readIMU` (I2C) |
| `K` | `calibrationTask` |
| `Y` | safety cutoff block |
| `E` | encoder → velocity |
| `C` | balance PID + `setMotors` |
| `R` | `handleTelemetryRX` + `parseCommand` |
| `V` | `pollLegServosTask` (AX-12 bus) |
| `T` | telemetry block |
| `X` | profiler's own cost, previous tick |

`B` is sampled *before* the row is composed, so it excludes the instrumentation;
the tax is reported separately as `X`.

### Stream 2 — the 1 Hz budget report

```
=== BUDGET 1s: 100 ticks x 10000us ===
  mean B1698us 16.9% free 8302us
  best B1603us free 8397us I1340 K0 Y1 E14 C38 R210 V0 T0
  mode B1600-1699us (71 of 100 ticks)
  w1 B3105us 31.0% I1341 K0 Y1 E14 C38 R1712 V0 T0
  w2 B1818us 18.1% I1345 K0 Y1 E14 C38 R205 V420 T0
  w3 B1495us 14.9% I1338 K0 Y1 E14 C39 R208 V0 T95
  w4 B1493us 14.9% ...
  w5 B1491us 14.9% ...
  WORST-CASE FREE 6895us (68.9%) <- RC budget
  ovr 0  servo_timeout 0  rows_dropped 0
```

Worst 5 ticks (with full stage breakdown, so you can see *what* made them worst),
best tick, mean, and modal 100 µs bucket.

**Use `WORST-CASE FREE`, not the mean, as the RC budget.** A stage that fits on
average but not on the worst tick is a stage that overruns the loop under load —
and that is exactly when a balancer must not stall.

### Why it cannot stall the loop it measures

The report is 11 lines but the UART TX ring is 256 B, so it can never be written
in one tick. It is emitted **one line per tick** across the following 11 ticks
(110 ms of every second), with the raw row suppressed while that runs — those
ticks are still counted in the statistics, only their printing is skipped. Every
write is a single guarded line. At ~56 B/row the raw stream is ~5.6 kB/s, 49% of
the port; the ring drains 115 B per tick and we add 56, so it stays near empty.

### Reading it

The firmware computes its own report, so a plain serial terminal on COM3 is
enough. For CSV logging or your own aggregation:

```
python prof_capture.py COM3                    # ranks stages by mean cost
python prof_capture.py COM3 --csv run.csv      # log every row
```

> Its column letters belong to **this variant only**. `ax12_control` emits a
> different set (`P B F R S U T L X`) for a different loop; pointing one
> variant's capture script at the other's port silently misreads the columns.

### What to look at first

`I` (`readIMU`) is expected to dominate: `Wire.setClock()` is never called, so
I2C runs at the STM32duino default of **100 kHz**, and the 15-byte transaction
costs roughly **1.3–1.5 ms — 13–15% of every single tick.** A single
`Wire.setClock(400000)` in `setupMPU()` would cut that ~4×, freeing ~1 ms per
tick. That change is *not* applied here — this pass is instrumentation only, so
measure it on your hardware first and decide with real numbers.

Spikes in `R` are the unguarded `Serial3.println()` acks in `parseCommand()`
blocking at ~87 µs/byte when the radio backs up (`ax12_control` guards these;
this firmware deliberately still does not).
