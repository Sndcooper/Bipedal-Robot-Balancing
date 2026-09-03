# MCU IK Engine Wireless (`mcu_ik_engine_wireless`)

This module provides an **Untethered MCU-Based Inverse Kinematics Architecture** operating over a **3DR 433MHz/915MHz Telemetry Radio** on `Serial3`. It allows untethered real-time PID balance tuning and dynamic leg Cartesian positioning ($X,Y$, distance, lean) without requiring an RC receiver module.

---

## ⚡ Hardware Connections & Pinout

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
 AX-12 Bus TX (1M)  │ PA2                  PB10 │ 3DR Radio TX (Serial3 TX @ 115k)
 AX-12 Bus RX (1M)  │ PA3                  PB11 │ 3DR Radio RX (Serial3 RX @ 115k)
                    └───────────────────────────┘
```

### Complete Pin Assignment Matrix

| Module | Signal / Component | STM32 Pin | Interface / Baud | Notes |
| :--- | :--- | :---: | :--- | :--- |
| **DC Motor Driver** | ENA / ENB | `PA1` / `PA0` | PWM Output | Speed control |
| | IN1..IN4 | `PB14`, `PB15`, `PB12`, `PB13` | GPIO Digital Out | H-Bridge Direction control |
| **Encoders** | Left A / B, Right A / B | `PA6`/`PA7`, `PB0`/`PB1` | Interrupt / GPIO | Quadrature speed measurement |
| **IMU** | MPU6050 SCL / SDA | `PB6` / `PB7` | I2C1 (400 kHz) | Complementary pitch estimation |
| **AX-12A Servos** | TX / RX | `PA2` / `PA3` | USART2 (1 Mbaud) | IDs: 6 (L Hip), 0 (L Knee), 14 (R Hip), 1 (R Knee) |
| **Wireless Telemetry** | 3DR Radio TX / RX | `PB10` / `PB11` | USART3 (115,200 Baud) | Non-blocking dynamic buffer-draining UART link |

---

## 📡 Wireless Protocol & Dynamic Drain

* **Terminator:** newline (`\n`, via `Serial3.println`). Commands are also
  newline-terminated. (This variant uses `key:value` comma style — **not** the
  space/pipe style of the `RC_mcu_IK_wireless` flagship.)
* **Buffer Drain Protection:** RX is drained non-blocking with a 40 µs hard budget
  per 100 Hz loop tick so a burst of commands can never blow the loop timing.
* **Outbound Telemetry Frame** (fields: seq, loop µs, pitch, PID out, integral,
  raw encoders L/R, velocity, tilt bias, cascade state, alpha, max tilt, motors,
  latched):
  ```text
  S:42,DT:10008,P:1.05,PO:-3.20,I:0.0100,EL:85,ER:-90,V:0.32,TB:0.02,ST:2,A:0.96,T:25.0,M:1,L:0
  ```

---

## 🎮 Drive Control & Cascaded Balance

`Kp_straight` has been **removed**. It fed on `(encL - encR)`, but the encoders
are mirror-mounted, so that difference tracks *forward distance*, not heading
error -- the correction grew the further the robot drove and steered it into a
circle. Straight-line travel now comes from the cascaded loops below, and
turning is an explicit operator command.

### Three-layer control stack

| Layer | Loop | Input | Output | Gains |
| :--- | :--- | :--- | :--- | :--- |
| 3 (outer) | Position hold | encoder position error | velocity target | `Kp_pos` (`PP`) |
| 2 (middle) | Velocity | velocity error | **tilt bias** (deg) | `Kp_vel` (`VP`), `Ki_vel` (`VI`) |
| 1 (inner) | Balance PID | pitch error | motor PWM | `Kp`, `Ki`, `Kd` |

A balancing robot cannot be commanded to a pitch directly -- leaning *is* how it
accelerates. Drive commands therefore set a **velocity** target, and layer 2
converts velocity error into the small lean required. The previous firmware fed
the operator setpoint straight into the balance loop, so the robot tried to
*stand up at* the commanded angle instead of using it to move, then ran away.

`Kd` now differentiates the **measurement** (`-gyroRate`) rather than the error,
so moving a slider no longer injects a one-tick derivative spike.

### Cascade state machine

```
        drive cmd                  |cmd| < 0.05
HOLDING ──────────► DRIVING ──────────────────► RAMPDOWN
   ▲                   ▲                            │
   │                   └──────── drive cmd ─────────┘
   └───────────────── |vel| < 20 counts/s ──────────┘
```

`RAMPDOWN` exists so the hold point is latched only once the robot has actually
stopped; latching on release would pin it to a spot it is still sliding past.

### Spin

Spin is a pure differential added after the balance term, so it cancels out of
the common-mode balance command and cannot tip the robot:

```
left  = -output + spin_pwm
right = -output - spin_pwm
```

* **Negative** `SPN` = left = **clockwise** (viewed from above)
* **Positive** `SPN` = right = **anticlockwise**
* Magnitude scales speed, up to `MAX_SPIN_PWM` (160 PWM units)

### GUI drive pad (Balance Tuner tab)

Both axes are **spring-loaded** -- releasing the mouse or key snaps them to zero
and sends a stop, so a command never outlives the operator's grip. Arrow keys
drive (Up/Down) and spin (Left/Right); a **STOP** button zeroes both. The pad is
also force-zeroed on safety cutoff, motor toggle, and disconnect.

---

## 📡 Command Reference

Commands are newline-terminated ASCII. Longer prefixes are matched **before**
single-letter commands, so `SPN`/`PP`/`VP` never collide with `S`/`P`.

| Command | Meaning |
| :--- | :--- |
| `FWD<-1..1>` | Forward/back velocity demand (no ack, high rate) |
| `SPN<-1..1>` | Spin demand; negative = clockwise (no ack, high rate) |
| `VP<f>` / `VI<f>` | Velocity loop `Kp_vel` / `Ki_vel` |
| `VA<f>` | Velocity EMA filter coefficient (0..0.99) |
| `PP<f>` | Position hold `Kp_pos` |
| `P` `I` `D` `<f>` | Balance PID gains |
| `A<f>` / `T<f>` | Complementary `alpha` / max safe tilt |
| `S<f>` | Operator pitch trim (base angle) |
| `O<f>` | Manual pitch offset **(newly implemented)** |
| `S` / `C` / `R` / `M` | Servo reset / calibrate / reset integrals / toggle motors |

Telemetry adds `V:` (filtered velocity), `TB:` (tilt bias) and `ST:` (cascade
state: 0=DRIVING 1=RAMPDOWN 2=HOLDING).

---

## 📂 File Map

* **[`firmware/src/main.cpp`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wireless/firmware/src/main.cpp)**: STM32 C++ firmware with MCU IK math, 3DR radio dynamic parser, and 100Hz balance controller.
* **[`firmware/platformio.ini`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wireless/firmware/platformio.ini)**: PlatformIO build configuration.
* **[`gui/main_gui.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wireless/gui/main_gui.py)**: Python desktop GUI.
* **[`gui/serial_link.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wireless/gui/serial_link.py)**: Wireless serial worker thread.

---

## 🚀 Quickstart

```bash
# Upload Firmware
cd firmware && pio run -t upload

# Run GUI over 3DR Radio COM port
python gui/main_gui.py
```
