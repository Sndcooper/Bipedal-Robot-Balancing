# Firmware Test & Scratchpad Files

This directory contains various legacy snippets, test sketches, and prototype C++ files that were used during the early development of the bipedal robot. They were originally located in the `test` directory.

## What these files do:

* **Hardware Component Tests (`motortest.cpp`, `dc_motor_test.cpp`, `motor control.cpp`)**: Standalone snippets to test raw DC motor functionality and H-Bridge controls.
* **IMU & Balance Testing (`balance_wheels.cpp`, `balancing tuning.cpp`, `reading9axisdata.cpp`)**: Scratchpad files used for isolating the MPU6050 reading, understanding Euler angles, and tuning basic balance before merging into the main program.
* **AX-12 Smart Servo Setup (`setting ax12.cpp`)**: Initial calibration code for ID configuration and moving specific joints to set positions.
* **Digital Twin Connection Tests (`digitaltwin.cpp`, `digitaltwin_current.cpp`, `Digitaltwin2legs.cpp`)**: Early codes handling custom byte-protocols over Serial for synchronizing the real robot's leg positions with the python digital twin simulation.
* **Archived Main (`old_main.cpp`, `python autoTune.cpp`)**: Previous main iterations with auto-tuning PID components which proved unstable or required refactoring.

> **Note**: These files are not meant to be compiled together. They are saved here as reference material. Only `src/main.cpp` should be compiled for the firmware.