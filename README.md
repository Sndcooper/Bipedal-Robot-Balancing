# Self-Balancing Bipedal Robot Firmware & Control Suite

This repository contains the complete firmware, hardware documentation, digital twin utilities, and graphical tuning suites for a **Self-Balancing Bipedal Robot**. The system combines an STM32-based 100 Hz embedded PID control loop with articulated Dynamixel AX-12+ legs, complementary IMU posture sensing, 3DR wireless telemetry radio, FlySky FS-iA10B RC remote control, and a Python GUI / Jupyter Digital Twin interface.

---

## 🤖 About the Robot

The Self-Balancing Bipedal Robot is a dynamic robotics platform balancing on two wheels attached to articulated, servo-driven legs:
* **Dynamic Posture Adjustment:** Alter height, stride separation, and lateral lean dynamically while preserving upright balance via inverted pendulum control.
* **100 Hz Embedded Control Loop:** An onboard MPU6050 IMU calculates body pitch inclination, fused via a complementary filter ($\alpha = 0.96$). The STM32 drives L298N H-Bridge PWM outputs to wheel motors with quadrature encoder velocity feedback.
* **FlySky iBUS RC & 3DR Wireless Telemetry:** Real-time remote operation via FlySky FS-iA10B receiver over iBUS (`Serial1`), alongside non-blocking telemetry streaming over 3DR Radio (`Serial3` @ 115,200 baud) using a pipe (`|`) terminator protocol.

---

## ⚡ Electronics & Hardware Architecture

* **Microcontroller:** STM32F103C8T6 (Bluepill) operating as the real-time core.
* **Leg Actuators:** 4× Dynamixel AX-12+ Smart Servos (IDs: 6, 0, 14, 1) communicating over a 1 Mbaud half-duplex UART bus (`Serial2`).
* **Wheel Actuators:** 2× 12V DC Motors equipped with Hall-effect Quadrature Encoders for wheel odometry.
* **Motor Drivers:** L298N H-Bridge PWM motor driver module.
* **Sensors & Peripherals:** MPU6050 I2C IMU (400 kHz), 3DR 433MHz/915MHz Telemetry Radio (`Serial3`), and FlySky FS-iA10B Receiver (`Serial1` iBUS).
* **Power Supply:** 11.1V – 12V 3S LiPo battery for motors and servos, step-down BEC for 5V/3.3V logic.

*(For detailed pin assignments, wiring schematics, and USART assignments, see [**`Hardware_Connections.md`**](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Hardware_Connections.md)).*

---

## 📁 Repository Organization & Core Submodules

### 1) [`Balance_Rework/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework)
The active development area containing the updated firmware, safety cutoffs, autotuner, and unified leg control environments.
* **[`tuner_legcontrol/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol)**: Unified leg control and balance tuning suite featuring 6 project subfolders:
  * **[`RC_mcu_IK_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/RC_mcu_IK_wireless)**: Flagship MCU IK + 3DR Wireless Telemetry + FlySky FS-iA10B RC Receiver module.
  * **[`mcu_ik_engine_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wireless)**: Wireless 3DR GUI tuning module.
  * **[`mcu_ik_engine_wired`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_wired)**: Zero-drop high-speed (500k baud) USB serial tuning.
  * **[`mcu_ik_engine`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine)**: Baseline on-chip IK reference solver.
  * **[`mcu_ik_engine_pretest_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine_pretest_wireless)**: Radio throughput and latency diagnostic tool (`latency_test.py`).
  * **[`pc_ik_engine`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/pc_ik_engine)**: Kinematic GUI prototyping engine running Python IK.
* **`autotuner/`**: Safety-aware PID tuning tool with automated disturbance testing and validation scoring.
* **`mpu_inspector/`**: Diagnostic GUI (`mpu_inspector_gui.py`) and Web Serial interface (`mpu_inspector_web.html`).

### 2) [`PlatformIO_Firmware/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/PlatformIO_Firmware)
Original baseline C++ firmware platform containing motor control snippets, hardware tests, and early PID implementations.

### 3) [`Python_Controller_Digital_Twin/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Python_Controller_Digital_Twin)
Python Jupyter notebooks (`bipedal_digital_twin_controller.ipynb`), legacy AX-12 controllers (`ax12_controller_legacy.ipynb`), and initial communication scripts.

---

## 🛠️ Key Firmware Specifications

* **Default Balance Gains:** $K_p = 95.0$, $K_i = 670.0$, $K_d = 1.9$.
* **Loop Rate:** Enforced 100 Hz (10,000 µs period).
* **Non-Blocking Telemetry RX:** Dynamic chunking ($\text{avail}/5$, max 20 bytes per tick) over `Serial3` using pipe (`|`) delimiter.
* **Encoder Smoothing:** Exponential Moving Average (EMA) velocity filter ($\alpha = 0.15$).
* **Servo Half-Duplex Hack:** 10 kΩ resistor between `PA2` and `PA3` with automatic 8-byte echo rejection.