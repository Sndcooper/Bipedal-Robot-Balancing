# Single-Loop Wireless Balance Tuner (`mcu_ik_engine_pretest_wireless`)

This directory is a **minimal, single control-loop wireless balancer**. It runs
exactly **one** balance PID (`Kp`, `Ki`, `Kd`) on complementary-filtered pitch —
no cascade (velocity/position), no inverse kinematics, no RC. The legs are held
in the calibrated standing pose and polled for health only. The GUI is a single
screen for balancing + IMU calibration.

> **History:** this folder previously held a pure 3DR radio latency benchmark.
> That role is preserved — the firmware still answers `PING:<token>` with
> `PONG:<token>`, so [`latency_test.py`](latency_test.py) continues to work — but
> the primary purpose is now single-loop balancing. The original benchmark
> firmware is recoverable from git history.

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
