# Python Controller & Digital Twin

Welcome to the **Python Controller & Digital Twin** module of the Self-Balancing Bipedal Robot project. This directory is the central hub for all high-level control systems, complex mathematics, and graphical visualization tools necessary to orchestrate the robot's movements. 

While the embedded STM32 microprocessor handles the real-time ultra-fast reactions necessary to just keep the robot upright (the inverted pendulum loop), it does not have the processing power to run advanced Inverse Kinematics (IK), 3D rendering, or session-based machine learning tuning. That is where this Python-based Digital Twin comes in.

## System Architecture & Purpose

The **Digital Twin** concept is a core pillar of this robotics project. By creating an exact mathematical and visual replica of the robot's physical dimensions (leg lengths, wheel radius, joint limits) in a Python environment, we can:
1. **Simulate Before Actuating:** Test complex movements, gaits, and postures in software before sending potentially dangerous commands to the physical motors.
2. **Calculate Inverse Kinematics (IK):** The Python equations calculate exactly what angle each of the AX-12+ servos must be at to place the "hip" of the robot at a specific X, Y, Z coordinate gracefully.
3. **Telemetry & Tuning:** The Python controller listens to high-speed telemetry (current pitch, PID errors, motor PWM values) coming from the STM32 over serial. It generates live graphs, allowing you to visually see how the robot reacts to being pushed or thrown off balance.

## Essential Files & Sub-Directories

### 1) The Master Controller
* **`bipedal_digital_twin_controller.ipynb`**
  * **Role**: Primary Jupyter Notebook interface connecting Inverse Kinematics to the STM32 serial interface.
  * **Features**: Connects to the COM port, parses incoming telemetry streams, and renders UI sliders to command leg coordinates ($X, Y, Z$) and body posture.

### 2) `digital_tests/`
* **Role**: Experimental playground for digital twin mathematics, 3D visualization, and raw `ax12_protocol.py` verification.

### 3) `root_tests/`
* **Role**: Serial communication diagnostic scripts (`test_communication.py`, `test_serial.py`) used to verify baud rates and USB-TTL handshakes.

### 4) `tuner_opus/`
* **Role**: Legacy automated PID tuning routines (`tune.py`, `autotune.py`, `balance_tuner.ipynb`).

---

## 🖥️ Modern Graphical Tuning Interfaces

> [!TIP]
> **Active GUI Applications:** For the latest graphical tuning apps featuring real-time Matplotlib plots, live AX-12 servo health diagnostics (temperature/load), 3DR radio link support, and profile saving, use **[`Balance_Rework/tuner_legcontrol/`](file:///c:/Users/vilas/Documents/PlatformIO/Projects/self%20balancing%20Bipedal%20robot/Balancing_Bipedal_Firmware_and_Scripts/Balance_Rework/tuner_legcontrol)**.

---

## Setup & Dependencies

To run the Python applications, set up a Python 3.8+ environment with the following dependencies:
* `pyserial` — Serial UART communication
* `numpy`, `scipy` — Inverse Kinematics & mathematical operations
* `matplotlib` — Live plotting and 2D/3D visualization
* `jupyterlab` or `notebook` — Notebook interface

### Running the Controller
1. Connect the STM32 via USB-TTL or 3DR Telemetry Radio.
2. Identify the active COM port (e.g. `COM3` on Windows or `/dev/ttyUSB0` on Linux).
3. Open `bipedal_digital_twin_controller.ipynb` or launch a desktop GUI in `Balance_Rework/tuner_legcontrol/*/gui/main_gui.py`.
