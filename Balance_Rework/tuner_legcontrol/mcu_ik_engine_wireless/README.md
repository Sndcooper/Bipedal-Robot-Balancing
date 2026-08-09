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

* **Terminator:** Pipe (`|`) character.
* **Buffer Drain Protection:** RX bytes are read dynamically every 100 Hz loop tick:
  $$\text{Chunk Size} = \operatorname{clamp}\left(\frac{\text{Serial3.available()}}{5},\, 1,\, 20\right)$$
* **Outbound Telemetry Frame:**
  ```text
  PITCH:1.05|ACC:-0.12|ENC:85,-90|V:0.32|MOT:1|
  ```

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
