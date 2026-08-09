# MCU IK Engine (`mcu_ik_engine`)

This module provides the baseline **On-Microcontroller Inverse Kinematics (MCU IK)** implementation. The geometrical solver (femur 55mm, tibia 100mm) runs directly in C++ on the **STM32 Bluepill (F103C8)**, allowing the host GUI to transmit high-level spatial coordinate goals rather than raw joint servo angles.

---

## 🔑 Key Architectural Features

1. **On-Chip Kinematic Resolution:** Computes joint angles ($\theta_1, \theta_2$) in C++ firmware at high speed.
2. **High-Level Command Protocol:** Accepts 2D Cartesian foot targets (`IK1,x,y`, `IK2,x,y`), leg separation distance (`IKD,dist`), and dynamic lean angle (`IKL,lean`).
3. **50Hz Read/Write Arbitration Toggle:** Alternates loop iterations between reading servo status and writing sync position targets to eliminate half-duplex UART bus contention.
4. **Wired High-Bandwidth Link:** Operates over USB FTDI (`Serial1`) @ **500,000 baud** for low-latency tuning.

---

## ⚡ Hardware Connections & Pinout

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
   AX-12 Bus TX (1M)      │ PA2                   PA9 │ USB/FTDI TX (Serial1 TX @ 500k)
   AX-12 Bus RX (1M)      │ PA3                  PA10 │ USB/FTDI RX (Serial1 RX @ 500k)
                          └───────────────────────────┘
```

### Pin Assignment Matrix

| Signal / Component | STM32 Pin | Function / Mode | Notes |
| :--- | :---: | :--- | :--- |
| **L298N ENA / ENB** | `PA1` / `PA0` | PWM Output | Left & Right motor speed control |
| **L298N IN1..IN4** | `PB14`, `PB15`, `PB12`, `PB13` | Digital Outputs | Direction bridge logic |
| **Left Encoder A / B** | `PA6` / `PA7` | Interrupt / GPIO | Speed & position feedback |
| **Right Encoder A / B** | `PB0` / `PB1` | Interrupt / GPIO | Speed & position feedback |
| **MPU6050 SCL / SDA** | `PB6` / `PB7` | I2C1 (400kHz) | Pitch angle estimation |
| **AX-12A Servo Bus** | `PA2` (TX) / `PA3` (RX) | USART2 (1 Mbaud) | Servo IDs: 6 (L Hip), 0 (L Knee), 14 (R Hip), 1 (R Knee) |
| **USB Serial Link** | `PA9` (TX1) / `PA10` (RX1) | USART1 (500,000 Baud) | High-speed GUI tuning interface |

---

## 🛠️ Command Protocol Reference

The firmware parses line-delimited ASCII strings received on `Serial1`:

| Command Format | Target Function | Example |
| :--- | :--- | :--- |
| `IK1,x,y` | Sets Left Leg foot Cartesian coordinates ($X, Y$) | `IK1,0,-120` |
| `IK2,x,y` | Sets Right Leg foot Cartesian coordinates ($X, Y$) | `IK2,0,-120` |
| `IKD,dist` | Sets inter-leg spacing distance | `IKD,60` |
| `IKL,lean` | Adjusts robot lateral lean angle | `IKL,5` |
| `P<val>`, `I<val>`, `D<val>` | Adjusts PID controller gains | `P95.0`, `I670.0`, `D1.9` |
| `POS,id,val` | Direct raw servo position write | `POS,6,512` |
| `TRQ,id,limit` | Sets maximum torque limit on servo | `TRQ,6,511` |
| `CMP,id,margin,slope` | Adjusts compliance margin and slope | `CMP,6,4,32` |
| `M` | Toggles motor power ON / OFF | `M` |

---

## 📂 File Map

* **[`firmware/src/main.cpp`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine/firmware/src/main.cpp)**: Main C++ balance firmware with trigonometric IK solver and 500k serial command parser.
* **[`firmware/platformio.ini`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine/firmware/platformio.ini)**: PlatformIO build parameters for `bluepill_f103c8`.
* **[`gui/main_gui.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine/gui/main_gui.py)**: Python desktop GUI for coordinate control, balance tuning, and telemetry visualization.
* **[`gui/serial_link.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine/gui/serial_link.py)**: Background serial thread managing communication at 500,000 baud.
