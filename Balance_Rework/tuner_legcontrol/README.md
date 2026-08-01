# Leg Control Tuner (tuner_legcontrol)

This directory contains two alternative implementations of the tuning and leg control system for the self-balancing bipedal robot:
1. **`pc_ik_engine`**: The Inverse Kinematics (IK) logic is computed on the PC (Python side).
2. **`mcu_ik_engine`**: The Inverse Kinematics (IK) logic is computed directly on the Microcontroller Unit (STM32 side).

Both environments allow real-time telemetry plotting, PID balance tuning, and interactive leg coordinate manipulation via a GUI.

---

## Directory Structure

```
tuner_legcontrol/
├── mcu_ik_engine/          # Microcontroller-computed IK Implementation
│   ├── firmware/           # PlatformIO project for the STM32 Bluepill
│   │   ├── src/main.cpp
│   │   └── platformio.ini
│   └── gui/                # Python-based control GUI
│       ├── main_gui.py
│       ├── serial_link.py
│       ├── twin_kinematics.py
│       └── profiles/       # Saved parameter sets (.json)
│
└── pc_ik_engine/           # PC-computed IK Implementation
    ├── firmware/           # PlatformIO project for the STM32 Bluepill
    │   ├── src/main.cpp
    │   └── platformio.ini
    └── gui/                # Python-based control GUI
        ├── main_gui.py
        ├── serial_link.py
        ├── twin_kinematics.py
        └── profiles/       # Saved parameter sets (.json)
```

---

## File Functionality Reference

Within both engine folders, the files are structured identically but have internal logical differences:

### 1. Firmware Files (`firmware/`)
*   **[src/main.cpp](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine/firmware/src/main.cpp)**: The STM32 firmware logic. It reads encoder data, retrieves raw orientation values from the MPU6050 IMU, computes a complementary filter for the pitch angle, runs the PID loop, generates PWM output for the DC drive motors, and manages AX-12 bus communication (half-duplex servo commands, torque limit adjustments, compliance margin adjustments, and state healing). It also contains safety-cutoff logic for robot tilts exceeding the safe threshold.
*   **[platformio.ini](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine/firmware/platformio.ini)**: The configuration file for compiling and uploading the STM32 code using PlatformIO. It targets the `bluepill_f103c8` board, specifies USART configurations, and defines the serial monitor baud rate.

### 2. GUI Files (`gui/`)
*   **[main_gui.py](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine/gui/main_gui.py)**: The Python desktop user interface built using Tkinter and Matplotlib. It renders real-time telemetry plots (pitch, PID output, velocities, encoder counts) and offers two control panels:
    1.  *PID / Balance Tuning*: Adjust PID coefficients, alpha values, target offsets, and maximum safety angles.
    2.  *Leg Geometry & Dynamics*: Sliders to adjust target foot coordinates ($X, Y$), compliance margin, slope, and torque limits.
*   **[serial_link.py](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine/gui/serial_link.py)**: The background worker thread that handles the serial connection with the STM32 over USB. It deserializes incoming telemetry data (`PITCH:`, `SRV:`, status responses) into memory structures for GUI graphing and parses GUI requests to format and send outgoing commands down the line.
*   **[twin_kinematics.py](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol/mcu_ik_engine/gui/twin_kinematics.py)**: The geometrical library that maps coordinates to AX-12 servo units. It holds parameters for the femur (55mm) and tibia (100mm) link lengths, servo joint locations, and AX-12 calibration constants (e.g., matching the straight-down orientation to AX-12 angles). It also contains the math for forward and inverse kinematics calculations.
*   **`profiles/`**: A subdirectory storing parameter sets (JSON files) that save the GUI's slider configuration (PID gains, leg heights, limits, margins).

---

## Architectural & Technical Differences

The key differences between `pc_ik_engine` and `mcu_ik_engine` are summarized below:

| Feature / Metric | `pc_ik_engine` | `mcu_ik_engine` |
| :--- | :--- | :--- |
| **IK Execution Location** | **PC (Python GUI)** <br> Resolves coordinates to servo angles on the PC and sends specific targets to each servo. | **MCU (STM32)** <br> The C++ firmware calculates the intersection math. The PC just sends the goal end-effector coordinates. |
| **Serial Command Protocol** | GUI sends raw servo commands: <br> `POS,<servo_id>,<position_value>` | GUI sends high-level spatial instructions: <br> `IK1,x,y` / `IK2,x,y` / `IKD,dist` / `IKL,lean` |
| **Baud Rate** | `115200` baud | `500000` baud (High-speed, required for high-frequency coordinate command streams) |
| **Lean Control (`IKL`)** | Not supported | Supported (Exposes a **Lean** slider to shift and tilt both legs dynamically) |
| **Servo Polling Frequency** | Slow (`2500ms` intervals between single servo checks; 10s for a full loop) | Fast (`20ms` intervals between single servo checks; 80ms for a full loop) |
| **Bus Arbitration Control** | Telemetry reading and tuning writes run concurrently in the firmware main loop. | Uses a strict **50Hz Read/Write toggle cycle** (alternating loop iterations) to prevent bus collisions between polling commands and high-speed IK writes. |

### Summary Recommendation
*   Use **`pc_ik_engine`** for simple debugging and testing kinematic parameters since any adjustment in `twin_kinematics.py` is immediately active without needing to re-flash the STM32 board.
*   Use **`mcu_ik_engine`** for dynamic real-time experiments, walking, leaning, and low-latency closed-loop balancing behaviors where the PC should not bottleneck the servo loop.
