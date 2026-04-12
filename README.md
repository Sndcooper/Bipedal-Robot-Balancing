# Self Balancing Bipedal Robot

This repository contains the firmware and high-level control scripts for a Self-Balancing Bipedal Robot. It features an STM32-based embedded PID control loop for dynamic 2-wheel balancing, integrated with a Python "Digital Twin" interface for real-time Inverse Kinematics and 3D posture visualization.

## About the Robot
The Self-Balancing Bipedal Robot is an advanced robotics platform that balances dynamically on two wheels attached to articulated, servo-driven legs. 
* **What it does:** The robot is capable of altering its height, posture, and leg configuration dynamically while simultaneously maintaining upright balance using inverted pendulum control logic. 
* **How it works:** An onboard Inertial Measurement Unit (IMU) constantly calculates the robot's pitch and tilt. A high-speed embedded PID loop reads this inclination and drives the wheel motors to keep the robot balanced. Concurrently, a "Digital Twin" interface on a PC runs Inverse Kinematics (IK), visualizes the robot's state in 3D, and sends real-time pose targets to the micro-controller over a Serial link.

## Hardware & Electronics Components
The robot integrates power electronics, sensors, and microcontrollers to achieve real-time balancing:
*   **Microcontroller:** STM32 Bluepill (or similar STM32F103C8T6 based board) serving as the main embedded brain for real-time sensor polling and motor control.
*   **Leg Actuators:** Dynamixel AX-12+ Smart Servos, providing high-torque articulation for the bipedal joints. They communicate via a half-duplex UART bus.
*   **Wheel Actuators:** 12V DC Motors equipped with Hall-effect Quadrature Encoders for precise wheel odometry and speed control.
*   **Motor Driver:** H-Bridge module (e.g., L298N or TB6612FNG) handling the high-current demands of the drive wheels.
*   **Sensors:** MPU6050 or MPU9250 IMU communicating via I2C, providing 6 or 9-axis motion tracking data for the balancing loop.
*   **Power System:** An 11.1V - 12V LiPo battery directly powers the motors and servos, while a step-down converter (BEC) supplies clean 5V/3.3V power to the STM32 logic and sensors.

*(For detailed wiring, pinouts, and electrical schematics, please see the [**Hardware_Connections.md**](Hardware_Connections.md) file included in this repository).*

## Project Structure & Code Details

This project uses a split architecture separating real-time embedded logic from higher-level PC visualizations and analytics. Each folder contains its own `README.md` clarifying its internal files.

### 1) `PlatformIO_Firmware/`
Contains the low-level embedded C++ firmware (PlatformIO project) running on the STM32 Bluepill micro-controller or similar.
* **`src/` and `include/`**: Production H-Bridge PID motor control, encoder processing, MPU6050 angle calculations, and AX-12 position locking.
* **`test/` (Archive & Staging)**: Legacy tests, old iterations (`old_main.cpp`), component isolation snippets (`dc_motor_test.cpp`), and tuning experiments that were written in C++ over UART.

### 2) `Python_Controller_Digital_Twin/`
Contains all high-level control scripts, inverse kinematics (IK), system tuning, and digital twin monitoring interfaces written in Python.
* **`bipedal_digital_twin_controller.ipynb`**: The master Jupyter Notebook connecting Inverse Kinematics to the STM32 serial interface.
* **`tuner_opus/`**: Tuning routines (`autotune.py`, `tune.py`, `balance_tuner.ipynb`) for calculating PID terms by sending step-response telemetry and saving session data.
* **`digital_tests/`**: Individual python experiment branches exploring the 2-leg digital twin interface, 3D visualization mapping, and raw `ax12_protocol.py`.
* **`root_tests/`**: Basic communication testers (e.g., `test_communication.py`) used to verify PC-to-Firmware Serial linkages before launching the main Notebook.

---
Enjoy building and tuning the robot!