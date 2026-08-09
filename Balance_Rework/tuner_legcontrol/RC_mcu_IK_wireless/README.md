# RC MCU IK Wireless (`RC_mcu_IK_wireless`)

This module provides an **Untethered Remote-Controlled Self-Balancing Architecture** featuring:
* On-board 2D Inverse Kinematics (IK) calculation on the **STM32 Bluepill (F103C8)**.
* **FlySky FS-iA10B RC Receiver** input parsing over **iBUS** on `Serial1`.
* Wireless telemetry streaming over a **3DR 433MHz/915MHz Radio** on `Serial3` @ 115,200 baud.
* High-speed 1 Mbaud half-duplex **Dynamixel AX-12A** servo control on `Serial2`.
* Non-blocking dynamic serial buffer-draining algorithm using the pipe (`|`) frame terminator to preserve the 100 Hz real-time control loop.

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
 AX-12 Bus TX (1M)  │ PA2                  PA10 │ FS-iA10B iBUS RX (Serial1 RX)
 AX-12 Bus RX (1M)  │ PA3                   PA9 │ FS-iA10B iBUS TX (Serial1 TX)
                    │                           │
  3DR Radio TX      │ PB10                 GND  │ Common Ground
  3DR Radio RX      │ PB11                 3.3V │ 3.3V Logic Power
                    └───────────────────────────┘
```

### Complete Wiring Matrix

| Hardware Component | STM32 Pin | Function / Protocol | Description |
| :--- | :---: | :--- | :--- |
| **L298N ENA** | `PA1` | PWM Output | Left DC motor speed control |
| **L298N ENB** | `PA0` | PWM Output | Right DC motor speed control |
| **L298N IN1 / IN2** | `PB14` / `PB15` | Digital GPIO | Left DC motor direction pins |
| **L298N IN3 / IN4** | `PB12` / `PB13` | Digital GPIO | Right DC motor direction pins |
| **Left Encoder A / B** | `PA6` / `PA7` | External Interrupt / GPIO | Quadrature speed sensing (`countLeft()`) |
| **Right Encoder A / B** | `PB0` / `PB1` | External Interrupt / GPIO | Quadrature speed sensing (`countRight()`) |
| **MPU6050 SCL / SDA** | `PB6` / `PB7` | I2C1 (400kHz) | Pitch pitch angle sensing via complementary filter |
| **AX-12A Servos** | `PA2` (TX) / `PA3` (RX) | USART2 (1 Mbaud Half-Duplex) | IDs: 6 (L Hip), 0 (L Knee), 14 (R Hip), 1 (R Knee) |
| **FlySky iBUS Receiver** | `PA10` (RX1) | USART1 (115,200 iBUS) | FS-iA10B receiver iBUS port |
| **3DR Radio Telemetry** | `PB10` (TX3) / `PB11` (RX3) | USART3 (115,200 Baud) | Wireless GUI telemetry & tuning bridge |

---

## 🎮 FlySky FS-iA10B RC Channel Mappings

Wireless RC control commands are read in real-time by `readRC()` via `IBusBM`:

| Channel | Function | Signal Range | Action on Robot |
| :---: | :--- | :---: | :--- |
| **Ch 3** | Pitch / Throttle | `1000 - 2000 µs` | Adjusts pitch modifier setpoint from $-2.0^\circ$ to $+2.0^\circ$ |
| **Ch 4** | Yaw / Steering | `1000 - 2000 µs` | Adjusts turn bias (differential motor speed) |
| **Ch 5** | IMU Zero Calibrate | Switch (`> 1500`) | Rising-edge triggers zero-pitch IMU calibration |
| **Ch 7** | Motor Hardware Arming | Switch (`> 1500`) | High arms motors & resets safety latch; Low disarms motors |
| **Ch 8** | Fine Pitch Trim | Knob (`1000 - 2000 µs`) | Live fine trim balance adjustment ($\pm 0.3^\circ$) |
| **Ch 10**| Integral Windup Kill | Switch (`> 1500`) | Active-HIGH disables PID integral accumulation to prevent windup |

---

## 📡 Wireless Telemetry & Protocol Specifications

* **Frame Terminator:** Pipe (`|`) symbol.
* **Non-Blocking Dynamic Drain:** `Serial3` RX buffer reads up to $\operatorname{clamp}(\text{avail}/5, 1, 20)$ bytes per 100 Hz loop tick.
* **Telemetry Output Frame:**
  ```text
  PITCH:0.12|ACC:-0.05|ENC:120,-118|V:0.45|MOT:1|SRV:6,42,15.2|
  ```
* **Command Acknowledgement:**
  ```text
  Updated P95.000 I670.000 D1.900 Offset0.0000 Target0.000 Alpha0.9600 STR0.0000 Tilt25.00|
  ```

---

## 📂 File Map

* **[`firmware/src/main.cpp`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/RC_mcu_IK_wireless/firmware/src/main.cpp)**: STM32 C++ firmware containing 100Hz balance loop, iBUS parser, dynamic 3DR telemetry drain, complementary filter, and AX-12 leg driver.
* **[`firmware/platformio.ini`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/RC_mcu_IK_wireless/firmware/platformio.ini)**: PlatformIO build configuration specifying `-DENABLE_HWSERIAL2` and `-DENABLE_HWSERIAL3`.
* **[`gui/main_gui.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/RC_mcu_IK_wireless/gui/main_gui.py)**: Python desktop tuning GUI with real-time Matplotlib charts and balance/leg geometry sliders.
* **[`gui/serial_link.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/RC_mcu_IK_wireless/gui/serial_link.py)**: Background thread handling byte accumulation, pipe parsing, and state synchronization.

---

## 🚀 Running the Module

1. Upload firmware using PlatformIO:
   ```bash
   cd firmware
   pio run -t upload
   ```
2. Turn on FlySky iBUS transmitter and arm using Channel 7 switch.
3. Launch Python GUI:
   ```bash
   python gui/main_gui.py
   ```
