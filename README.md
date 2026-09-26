# Self-Balancing Bipedal Robot Firmware & Control Suite

This repository contains the complete firmware, hardware documentation, digital twin utilities, and graphical tuning suites for a **Self-Balancing Bipedal Robot**. The system combines an STM32-based 100 Hz embedded PID control loop with articulated Dynamixel AX-12+ legs, complementary IMU posture sensing, 3DR wireless telemetry radio, FlySky FS-iA10B RC remote control, and a Python GUI / Jupyter Digital Twin interface.

---

## 📸 Gallery & Demo

Meet **PAW** — the physical build of this project, 3D-printed and assembled on the bench:

<p align="center">
  <img src="media/paw_front_view.jpg" alt="PAW bipedal robot - front view" width="45%" />
  <img src="media/paw_top_view.jpg" alt="PAW bipedal robot - top view with gripper" width="45%" />
</p>

*Left: front view showing the two articulated Dynamixel-driven legs, wheel motors, and the 3D-printed chassis with carry handle. Right: top-down view of the chassis, showing the parallel-link leg mechanism and the orange gripper arms mounted at the front.*

**🎥 Demo video:** a short clip of PAW balancing and moving under its own control loop.
[**▶️ Watch on Google Drive**](https://drive.google.com/file/d/1A6eR97t5H_khKykf_Axxi-ui6EwEtNq_/view?usp=sharing) *(streams inline — recommended)*, or [download the raw file](media/paw_demo.mp4) straight from the repo.

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

*(For detailed pin assignments, wiring schematics, and USART assignments, see [**`docs/Hardware_Connections.md`**](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/docs/Hardware_Connections.md)).*

---

## 📁 Repository Organization & Core Submodules

The robot's **working code lives at the repo root, in `robot_control_suite/`.**
Everything else — earlier rework iterations, the original baseline firmware,
and one-off tuning/diagnostic tools that have been superseded — is archived
under `legacy/`, kept for reference but not actively maintained.

### 1) [`robot_control_suite/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite) — **main, active codebase**
Unified leg control and balance tuning suite. Each subfolder is a **self-contained
firmware + GUI pair with its own serial protocol** — see
[`robot_control_suite/README.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/README.md)
before assuming anything crosses over between variants. RC-capable variants
(`rc_mcu_ik_wireless`, `rc_balance_fusion_wireless`) sit directly here; wireless
variants with no RC transmitter are grouped under
[`wireless_no_rc/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/wireless_no_rc)
— see its own `README.md` for that list. Also present: `blue_pill_dev/`,
`black_pill_dev/` (staged bring-up sub-projects) and `ax12_control/`
(standalone servo tool) — see each folder's own `README.md`.

### 2) [`legacy/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/legacy) — archived, not actively maintained
Split into
[`old_firmware/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/legacy/old_firmware)
(superseded full firmware/controller generations) and
[`testing/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/legacy/testing)
(one-off tuning/diagnostic tools) by what the code *is*, not just when it was
written — see [`legacy/README.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/legacy/README.md)
for the full breakdown.

### 3) [`docs/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/docs) — reference documentation
Physical spec, hardware pinout, AX-12 control table, and project history/roadmap
— see [`docs/README.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/docs/README.md)
for the index.

---

## 🛠️ Key Firmware Specifications

* **Default Balance Gains:** $K_p = 95.0$, $K_i = 670.0$, $K_d = 1.9$.
* **Loop Rate:** Enforced 100 Hz (10,000 µs period).
* **Non-Blocking Telemetry RX:** Dynamic chunking ($\text{avail}/5$, max 20 bytes per tick) over `Serial3` using pipe (`|`) delimiter.
* **Encoder Smoothing:** Exponential Moving Average (EMA) velocity filter ($\alpha = 0.15$).
* **Servo Half-Duplex Hack:** 10 kΩ resistor between `PA2` and `PA3` with automatic 8-byte echo rejection.