# Leg Control & Tuning Ecosystem (`tuner_legcontrol`)

This directory contains the complete modular suite of firmware and graphical tuning applications for the self-balancing bipedal robot. The system supports multiple control architectures: Inverse Kinematics (IK) calculated on-board the **STM32 Microcontroller (MCU)** vs calculated on the **Host PC**, transported over **High-Speed Wired Serial (FTDI/USB)** or **Wireless 3DR Telemetry Radio**, with optional **FlySky iBUS RC Transmitter** remote control.

---

## 📁 Ecosystem Subdirectory Matrix

| Subdirectory Folder | IK Execution | Transport Medium | Baud Rate | FlySky iBUS RC | Primary Purpose |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **[`RC_mcu_IK_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/RC_mcu_IK_wireless)** | **MCU (STM32)** | Wireless (3DR) | 115,200 | **Enabled** (FS-iA10B) | **Full Autonomous & RC Untethered Balancing.** Features full remote control, live telemetry, dynamic lean/gimbal, and non-blocking iBUS input. |
| **[`mcu_ik_engine_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wireless)** | **MCU (STM32)** | Wireless (3DR) | 115,200 | Disabled | **Wireless Telemetry & GUI Tuning.** Untethered balance tuning with real-time GUI telemetry plotting over 3DR radio. |
| **[`mcu_ik_engine_wired`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wired)** | **MCU (STM32)** | Wired (USB/FTDI) | 500,000 | Disabled | **Low-Latency Wired Balance Tuning.** Zero-drop high-bandwidth tuning directly connected to Python GUI. |
| **[`mcu_ik_engine`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine)** | **MCU (STM32)** | Wired (Serial1) | 500,000 | Disabled | **Core MCU IK Reference Engine.** Baseline on-chip IK foot coordinate ($X,Y$), leg distance, and lean solver. |
| **[`mcu_ik_engine_pretest_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_pretest_wireless)** | None (Diagnostic) | Wireless (3DR) | 115,200 | Disabled | **Radio Latency & Diagnostic Benchmarking.** Minimal benchmark firmware to test throughput, ping/pong latency, and packet loss. |
| **[`pc_ik_engine`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/pc_ik_engine)** | **PC (Python)** | Wired (Serial1) | 115,200 | Disabled | **Kinematic GUI Prototyping.** PC calculates inverse kinematics in Python (`twin_kinematics.py`) and streams raw angle target writes to MCU. |

---

## ⚡ Master Hardware Pinouts & Wire Connections

All firmware targets the **STM32F103C8T6 (Bluepill)**. The pin assignments across all variants are strictly standardized:

```
                          ┌───────────────────────────┐
                          │   STM32F103C8T6 BLUEPILL  │
                          ├───────────────────────────┤
    L298N Motor ENA (PWM) │ PA1                   PA6 │ Left Encoder A (Interrupt)
    L298N Motor ENB (PWM) │ PA0                   PA7 │ Left Encoder B
    L298N Motor IN1       │ PB14                  PB0 │ Right Encoder A (Interrupt)
    L298N Motor IN2       │ PB15                  PB1 │ Right Encoder B
    L298N Motor IN3       │ PB12                  PB6 │ MPU6050 SCL (I2C1)
    L298N Motor IN4       │ PB13                  PB7 │ MPU6050 SDA (I2C1)
                          │                           │
   AX-12 Bus TX (USART2)  │ PA2                  PA10 │ FS-iA10B iBUS RX (USART1 RX)
   AX-12 Bus RX (USART2)  │ PA3                   PA9 │ FS-iA10B iBUS TX (USART1 TX)
                          │                           │
  3DR Radio TX (USART3)   │ PB10                 GND  │ Common Ground
  3DR Radio RX (USART3)   │ PB11                 3.3V │ 3.3V Logic Supply
                          └───────────────────────────┘
```

### 1. Complete Connections Table

| System Submodule | Component / Signal | STM32 Pin | Interface / Notes |
| :--- | :--- | :---: | :--- |
| **Drive Motors (L298N)** | Left Motor Speed (ENA) | `PA1` | Timer 2 PWM Output |
| | Left Motor Dir 1 (IN1) | `PB14` | GPIO Digital Output |
| | Left Motor Dir 2 (IN2) | `PB15` | GPIO Digital Output |
| | Right Motor Speed (ENB) | `PA0` | Timer 2 PWM Output |
| | Right Motor Dir 1 (IN3) | `PB12` | GPIO Digital Output |
| | Right Motor Dir 2 (IN4) | `PB13` | GPIO Digital Output |
| **Quadrature Encoders** | Left Encoder Ch A | `PA6` | External Interrupt `countLeft()` |
| | Left Encoder Ch B | `PA7` | GPIO Input (Pullup) |
| | Right Encoder Ch A | `PB0` | External Interrupt `countRight()` |
| | Right Encoder Ch B | `PB1` | GPIO Input (Pullup) |
| **IMU Orientation** | MPU6050 I2C Clock (SCL) | `PB6` | I2C1 Bus (400 kHz Fast-Mode) |
| | MPU6050 I2C Data (SDA) | `PB7` | I2C1 Bus |
| **Smart Servo Bus** | Dynamixel AX-12 TX | `PA2` | USART2 (1,000,000 Baud Half-Duplex) |
| | Dynamixel AX-12 RX | `PA3` | USART2 |
| | Servo IDs & Locations | `6` (Left Hip), `0` (Left Knee), `14` (Right Hip), `1` (Right Knee) |
| **Wireless Telemetry** | 3DR Radio Module TX | `PB10` | USART3 TX (115,200 Baud) |
| | 3DR Radio Module RX | `PB11` | USART3 RX (115,200 Baud) |
| **RC Receiver** | FlySky FS-iA10B iBUS | `PA10` | USART1 RX (115,200 Baud iBUS Frame Parser) |

---

## 🎮 FlySky FS-iA10B iBUS Remote Control Channel Mapping

In `RC_mcu_IK_wireless`, manual wireless control is handled via an iBUS connection from the FS-iA10B receiver to USART1 (`PA10`). The channel assignments are defined below:

| RC Channel | Control Function | Input Range | MCU Behavior & Mapping |
| :---: | :--- | :---: | :--- |
| **Ch 3** | Pitch / Forward-Backward | `1000 - 2000 µs` | Adjusts setpoint pitch modifier from $-2.0^\circ$ to $+2.0^\circ$ |
| **Ch 4** | Yaw / Steering Turn Bias | `1000 - 2000 µs` | Adjusts left/right motor speed differential for turning |
| **Ch 5** | IMU Calibration Trigger | Switch (High > 1500) | Rising-edge triggers auto IMU zero-pitch calibration |
| **Ch 7** | Motor Hardware Arming | Switch (High > 1500) | **Arming Switch:** High enables motors & clears safety latch; Low disables motors |
| **Ch 8** | Gimbal / Fine Pitch Offset | Knob (`1000 - 2000 µs`) | Live fine trim balance adjustment ($\pm 0.3^\circ$) |
| **Ch 10**| Integral Windup Kill | Switch (High > 1500) | **Active-HIGH:** Kills PID integral accumulation to prevent windup during tuning |

---

## 📡 Wireless Telemetry & Dynamic Buffer Drain Protocol

To maintain a strict **100 Hz real-time loop budget (10,000 µs)** without UART blocking:

1. **Pipe Terminator (`|`) Protocol:** Outgoing and incoming commands use `|` as the canonical delimiter (e.g. `PITCH:1.23|ACC:-0.45|ENC:120,-115|V:0.4|MOT:1|`). Standard `\n` is also accepted for backward compatibility with serial terminals.
2. **Non-Blocking Dynamic Drain:** RX buffers on `Serial3` are read dynamically every loop tick:
   $$\text{Chunk Size} = \operatorname{clamp}\left(\frac{\text{Serial3.available()}}{5},\, 1,\, 20\right) \text{ bytes/tick}$$
   This protects against buffer overruns during high data rate spikes without blocking CPU execution.
3. **Telemetry Packet Format:**
   ```
   PITCH:<val>|ACC:<val>|ENC:<l>,<r>|V:<vel>|SRV:<id>,<temp>,<load%>|
   ```

---

## 🚀 Quickstart & Usage

1. Open any subfolder in **PlatformIO** (VS Code).
2. Connect your STM32 Bluepill via ST-Link or USB.
3. Build and upload using PlatformIO task **Upload** or terminal command:
   ```bash
   pio run -t upload
   ```
4. Launch the python GUI corresponding to the selected subfolder:
   ```bash
   python gui/main_gui.py
   ```
time experiments, walking, leaning, and low-latency closed-loop balancing behaviors where the PC should not bottleneck the servo loop.
