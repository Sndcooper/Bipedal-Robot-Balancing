# Legacy

Archived code, kept for reference. **Not actively maintained** — nothing here
is guaranteed to build or match the current hardware. The robot's active
codebase is [`robot_control_suite/`](../robot_control_suite) at the repo root.

Split into two subfolders by what the code *is*, not just when it was written:

## [`old_firmware/`](old_firmware)
Superseded full firmware/controller generations — earlier attempts at "the
whole robot program," each replaced by a later one:
* `mcu_ik_engine`, `mcu_ik_engine_wired`, `pc_ik_engine` — earlier
  `robot_control_suite` variants (on-host/wired IK), superseded by the
  wireless variants there now.
* `Balance_Rework_firmware/` (+ `Balance_Rework_README.md`) — earlier
  single-cascade rework firmware (colon-delimited telemetry), predates
  `robot_control_suite` entirely.
* `PlatformIO_Firmware/` — original baseline C++ firmware: motor control
  snippets, hardware tests, early PID implementations.
* `Python_Controller_Digital_Twin/` — original Python Jupyter notebooks and
  initial communication scripts.

## [`testing/`](testing)
One-off tuning/diagnostic tools and test rigs — never a full robot program,
just something used to poke at the hardware:
* `autotuner/` — safety-aware PID tuning tool with automated disturbance
  testing and validation scoring.
* `mpu_inspector/` — diagnostic GUI and Web Serial IMU dashboard.
* `servo_home/` — servo-homing/calibration test rig.
* `temp_stm32_uploads/` — scratch STM32 upload experiments.
* `latency_log.csv` — recorded output from a past latency test run.
