# Old Firmware (legacy)

Superseded full firmware/controller generations. Each subfolder has its own
`README.md` with details — this is just the index. See [`../README.md`](../README.md)
for how this fits into `legacy/` as a whole.

| Folder | What it was | Superseded by |
| :--- | :--- | :--- |
| [`Balance_Rework_firmware/`](Balance_Rework_firmware) (+ [`Balance_Rework_README.md`](Balance_Rework_README.md)) | Earlier single-cascade rework firmware, colon-delimited telemetry (`PITCH:`) | `robot_control_suite/` (all variants) |
| [`mcu_ik_engine/`](mcu_ik_engine) | Baseline on-chip IK reference, wired USB @ 500k | `robot_control_suite/rc_mcu_ik_wireless` and siblings |
| [`mcu_ik_engine_wired/`](mcu_ik_engine_wired) | Zero-drop wired bench tuning | `robot_control_suite/rc_mcu_ik_wireless` and siblings |
| [`pc_ik_engine/`](pc_ik_engine) | IK computed on the host PC (`twin_kinematics.py`), streamed to MCU | on-MCU IK in all current `robot_control_suite` variants |
| [`PlatformIO_Firmware/`](PlatformIO_Firmware) | Original baseline C++ firmware — motor snippets, hardware tests, early PID | everything above |
| [`Python_Controller_Digital_Twin/`](Python_Controller_Digital_Twin) | Original Python Jupyter notebooks and communication scripts | everything above |
