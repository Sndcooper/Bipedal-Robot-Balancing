# MCU IK Engine Wired (`mcu_ik_engine_wired`)

This directory contains the **High-Speed Wired (USB/FTDI)** variant of the Microcontroller Inverse Kinematics Engine. It is optimized for zero-packet-drop balance tuning, dynamic leg geometry experiments, and high-frequency parameter identification over a direct physical cable connection.

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
   AX-12 Bus TX (1M)      │ PA2                   PA9 │ FTDI / USB Serial TX (Serial1 TX @ 500k)
   AX-12 Bus RX (1M)      │ PA3                  PA10 │ FTDI / USB Serial RX (Serial1 RX @ 500k)
                          └───────────────────────────┘
```

### Hardware Pinout Table

| Module | Signal / Component | STM32 Pin | Details |
| :--- | :--- | :---: | :--- |
| **DC Motors** | Left ENA / Right ENB | `PA1` / `PA0` | PWM speed channels |
| | Direction Pins | `PB14`, `PB15`, `PB12`, `PB13` | Motor direction control |
| **Encoders** | Left A / B, Right A / B | `PA6`/`PA7`, `PB0`/`PB1` | Hardware interrupts for wheel velocity calculation |
| **IMU** | MPU6050 SCL / SDA | `PB6` / `PB7` | I2C pitch measurement |
| **Servos** | AX-12 TX / RX | `PA2` / `PA3` | USART2 @ 1,000,000 Baud (Half-Duplex) |
| **Wired Link** | FTDI TX / RX | `PA9` / `PA10` | USART1 @ 500,000 Baud |

---

## 🛠️ Operational Characteristics

* **IK Computation:** Handled fully on-chip in C++.
* **Baud Rate:** `500,000` baud on `Serial1`.
* **Bus Arbitration:** 50 Hz Read/Write toggle mode to prevent AX-12 bus collisions.

---

## 📂 File Map

* **[`firmware/src/main.cpp`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wired/firmware/src/main.cpp)**: STM32 C++ firmware configured for 500k wired FTDI communication.
* **[`firmware/platformio.ini`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wired/firmware/platformio.ini)**: PlatformIO compilation settings.
* **[`gui/main_gui.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wired/gui/main_gui.py)**: Python tuning GUI interface.
* **[`gui/serial_link.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wired/gui/serial_link.py)**: Serial communication module.

---

## 🚀 Quickstart

```bash
# Upload Firmware
cd firmware && pio run -t upload

# Launch GUI
python gui/main_gui.py
```
