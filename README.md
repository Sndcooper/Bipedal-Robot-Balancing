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
firmware + GUI pair with its own serial protocol** — see `robot_control_suite/README.md`
before assuming anything crosses over between variants.
RC-capable variants (drive the robot with a FlySky transmitter) sit directly here:
* **[`rc_mcu_ik_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/rc_mcu_ik_wireless)**: Flagship — MCU IK + 3DR wireless telemetry + FlySky FS-iA10B RC receiver.
* **[`rc_balance_fusion_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/rc_balance_fusion_wireless)**: `mcu_balance_fusion_wireless` with a FlySky transmitter added.

**[`wireless_no_rc/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/wireless_no_rc)** — wireless variants with no RC transmitter, GUI/bench-tuned only:
* **[`mcu_ik_engine_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/wireless_no_rc/mcu_ik_engine_wireless)**: Wireless 3DR GUI tuning module.
* **[`mcu_ik_engine_pretest_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/wireless_no_rc/mcu_ik_engine_pretest_wireless)**: Minimal single-loop wireless balancer; retains a PING/PONG latency probe (`latency_test.py`).
* **[`mcu_balance_fusion_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/wireless_no_rc/mcu_balance_fusion_wireless)**: Sensor-fusion balancer (Stage 2, in progress). Grouped here on its own name/README; its `main.cpp` does contain an iBUS decoder (it's the parent `rc_balance_fusion_wireless` was forked from).
* **[`mcu_pos_wireless`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/wireless_no_rc/mcu_pos_wireless)**: Position/encoder-target tuner.

Also present: **[`blue_pill_dev/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/blue_pill_dev)**, **[`black_pill_dev/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/black_pill_dev)** (staged bring-up sub-projects), **[`ax12_control/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/robot_control_suite/ax12_control)** (standalone servo tool) — see each folder's own `README.md`.

### 2) [`legacy/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/legacy) — archived, not actively maintained
Split into two subfolders by what the code *is*, not just when it was written:

* **[`legacy/old_firmware/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/legacy/old_firmware)** — superseded full firmware/controller generations:
  * **`mcu_ik_engine`, `mcu_ik_engine_wired`, `pc_ik_engine`**: earlier `robot_control_suite` variants (on-host/wired IK), superseded by the wireless variants above.
  * **`Balance_Rework_firmware/`** (+ `Balance_Rework_README.md`): earlier single-cascade rework firmware (colon-delimited telemetry), predates `robot_control_suite`.
  * **`PlatformIO_Firmware/`**: original baseline C++ firmware — motor control snippets, hardware tests, early PID implementations.
  * **`Python_Controller_Digital_Twin/`**: original Python Jupyter notebooks (`bipedal_digital_twin_controller.ipynb`), legacy AX-12 controllers (`ax12_controller_legacy.ipynb`), and initial communication scripts.
* **[`legacy/testing/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/legacy/testing)** — one-off tuning/diagnostic tools and test rigs, not full robot programs:
  * **`autotuner/`**: safety-aware PID tuning tool with automated disturbance testing and validation scoring.
  * **`mpu_inspector/`**: diagnostic GUI (`mpu_inspector_gui.py`) and Web Serial interface (`mpu_inspector_web.html`).
  * **`servo_home/`**: servo-homing/calibration test rig (`leg_control.py`, `digital_twin_legs.py`).
  * **`temp_stm32_uploads/`**: scratch STM32 upload experiments.
  * **`latency_log.csv`**: recorded output from a past latency test run.

### 3) [`docs/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/docs) — reference documentation
* **[`Robot_Specification.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/docs/Robot_Specification.md)**: physical dimensions, mass, leg geometry, servo IDs, calibration.
* **[`Hardware_Connections.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/docs/Hardware_Connections.md)**: full pinout, power, USART map, RC channels.
* **[`AX12_Control_Table_Mapping.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/docs/AX12_Control_Table_Mapping.md)**: AX-12+ control table and buffer offsets.
* **[`Project_History_and_Roadmap.md`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/docs/Project_History_and_Roadmap.md)**: project history and phased roadmap (renamed from `go-throuugh-all-the-golden-sketch.md`).

---

## 🛠️ Key Firmware Specifications

* **Default Balance Gains:** $K_p = 95.0$, $K_i = 670.0$, $K_d = 1.9$.
* **Loop Rate:** Enforced 100 Hz (10,000 µs period).
* **Non-Blocking Telemetry RX:** Dynamic chunking ($\text{avail}/5$, max 20 bytes per tick) over `Serial3` using pipe (`|`) delimiter.
* **Encoder Smoothing:** Exponential Moving Average (EMA) velocity filter ($\alpha = 0.15$).
* **Servo Half-Duplex Hack:** 10 kΩ resistor between `PA2` and `PA3` with automatic 8-byte echo rejection.