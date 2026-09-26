# Testing (legacy)

One-off tuning/diagnostic tools and test rigs — not full robot programs, just
things that were used to poke at the hardware during development. See
[`../README.md`](../README.md) for how this fits into `legacy/` as a whole.

| Folder / file | Purpose |
| :--- | :--- |
| [`autotuner/`](autotuner) | Safety-aware PID tuning tool with automated disturbance testing and validation scoring against `Balance_Rework_firmware`'s colon-telemetry protocol. |
| [`mpu_inspector/`](mpu_inspector) | Diagnostic GUI (`mpu_inspector_gui.py`) and Web Serial dashboard (`mpu_inspector_web.html`) for the MPU6050 IMU. |
| [`servo_home/`](servo_home) | Servo-homing/calibration test rig (`leg_control.py`, `digital_twin_legs.py`). |
| [`temp_stm32_uploads/`](temp_stm32_uploads) | Scratch STM32 upload experiments. |
| [`latency_log.csv`](latency_log.csv) | Recorded output from a past 3DR radio latency test run. |
