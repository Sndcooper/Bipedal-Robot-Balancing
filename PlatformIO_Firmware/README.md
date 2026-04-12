# PlatformIO Firmware

This folder contains the main embedded C++ firmware for the bipedal robot.

## Files Description

* **`platformio.ini`**: The PlatformIO configuration file. Sets the environment for the STM32 (`bluepill_f103c8`) board, framework (Arduino), upload protocol, and monitors variables.
* **`src/main.cpp`**: The primary C++ source file containing:
  - MPU6050 IMU configuration and pitch angle tracking.
  - The PID control logic for the self-balancing algorithm.
  - Encoder interrupt handling to track DC motor positions.
  - Serial2 interface helpers for talking to the AX-12 smart servos to lock their positions.
* **`lib/`**: Directory for custom or third-party libraries.
* **`include/`**: Directory for external C++ header files.
