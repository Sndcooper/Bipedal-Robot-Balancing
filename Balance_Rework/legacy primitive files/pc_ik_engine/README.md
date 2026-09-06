# PC IK Engine (`pc_ik_engine`)

This module provides the **PC-Calculated Inverse Kinematics Architecture**. The geometrical solver ($X,Y \to \theta_1, \theta_2$) runs inside the Python application (`twin_kinematics.py`). The PC streams explicit servo goal positions down to the **STM32 Bluepill (F103C8)** over USB serial.

---

## 🔑 Advantages & Use Cases

* **Rapid Kinematic Prototyping:** Ideal for debugging geometric leg link adjustments (femur/tibia lengths, offset shifts) in Python code without re-compiling or re-flashing the STM32 microcontroller.
* **Direct Servo Control:** The GUI sends explicit position commands (`POS,<servo_id>,<position_value>`) directly to individual AX-12 servos.

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
   AX-12 Bus TX (1M)      │ PA2                   PA9 │ USB / FTDI TX (Serial1 TX @ 115k)
   AX-12 Bus RX (1M)      │ PA3                  PA10 │ USB / FTDI RX (Serial1 RX @ 115k)
                          └───────────────────────────┘
```

### Complete Pin Assignment Matrix

| Module | Signal / Component | STM32 Pin | Interface / Baud | Notes |
| :--- | :--- | :---: | :--- | :--- |
| **DC Motor Driver** | ENA / ENB | `PA1` / `PA0` | PWM Output | Left & Right speed control |
| | IN1..IN4 | `PB14`, `PB15`, `PB12`, `PB13` | GPIO Out | H-Bridge direction control |
| **Encoders** | Left A / B, Right A / B | `PA6`/`PA7`, `PB0`/`PB1` | Interrupt / GPIO | Quadrature wheel velocity feedback |
| **IMU** | MPU6050 SCL / SDA | `PB6` / `PB7` | I2C1 (400 kHz) | Pitch angle estimation |
| **AX-12A Servos** | TX / RX | `PA2` / `PA3` | USART2 (1 Mbaud) | Servo IDs: 6 (L Hip), 0 (L Knee), 14 (R Hip), 1 (R Knee) |
| **Serial Control** | USB FTDI TX / RX | `PA9` / `PA10` | USART1 (115,200 Baud) | Receives `POS,<id>,<val>` commands from PC GUI |

---

## 🛠️ Serial Command Interface

| Command | Action | Example |
| :--- | :--- | :--- |
| `POS,id,val` | Write raw position to AX-12 servo | `POS,6,512` |
| `TRQ,id,val` | Set torque limit | `TRQ,6,511` |
| `CMP,id,m,s` | Set compliance margin `m` and slope `s` | `CMP,6,4,32` |
| `P<val>`, `I<val>`, `D<val>` | Balance PID tuning gains | `P78.0`, `I0.0`, `D0.0` |

---

## 📂 File Map

* **[`firmware/src/main.cpp`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/pc_ik_engine/firmware/src/main.cpp)**: STM32 C++ firmware running balance loop and accepting raw servo target commands.
* **[`firmware/platformio.ini`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/pc_ik_engine/firmware/platformio.ini)**: PlatformIO build parameters.
* **[`gui/main_gui.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/pc_ik_engine/gui/main_gui.py)**: Python tuning GUI application.
* **[`gui/twin_kinematics.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/pc_ik_engine/gui/twin_kinematics.py)**: Python inverse kinematics calculation library.
* **[`gui/serial_link.py`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/pc_ik_engine/gui/serial_link.py)**: Background serial thread.

---

## 🚀 Quickstart

```bash
# Upload Firmware
cd firmware && pio run -t upload

# Launch GUI
python gui/main_gui.py
```
