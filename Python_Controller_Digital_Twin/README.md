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
* **ipedal_digital_twin_controller.ipynb** (Previously known as x12_controller.ipynb)
  * **Role**: This Jupyter Notebook is the grandmaster interface. When you are using the robot, this is the dashboard you look at. 
  * **Features**: It connects to the COM port established with the robot. It parses the incoming packet stream containing IMU pitch data and motor outputs. It also contains the UI sliders/widgets to command the robot to "crouch", "stand tall", or "lean forward". Under the hood, it takes those UI inputs, runs the 3-DOF Inverse Kinematics for both legs, packages the target servo angles into a byte array, and transmits it down to the STM32.

### 2) digital_tests/
* **Role**: Experimental playground involving the digital twin math.
* **Contents**: Several iterations of digital twin.py and x12_protocol.py that were used purely to establish the initial 3D visualization using libraries like Matplotlib. These scripts were used to debug the math behind the leg joints without needing the physical robot turned on.

### 3) oot_tests/
* **Role**: The lowest-level PC communication scripts.
* **Contents**: Scripts such as 	est_communication.py and 	est_serial.py. If the robot refuses to connect to the Digital Twin dashboard, you run these scripts to verify whether the COM port is active, whether baud rates match, and if the STM32 is successfully returning basic ping/pong handshakes.

### 4) 	uner_opus/
* **Role**: The Automated PID Tuning Suite.
* **Contents**: This folder contains 	une.py, utotune.py, and alance_tuner.ipynb. Manual PID tuning is extremely tedious. These scripts inject specialized "step-functions" into the robot, record the oscillation graph back from the IMU, and utilize analytical models to suggest optimal Kp, Ki, and Kd values for the balancing loop.

## Setup & Dependencies

To optimally run the scripts in this folder, you will need a Python 3.8+ environment (preferably managed via Conda or venv) with the following typical dependencies installed:
* pyserial - Crucial for the 1Mbps UART telemetry stream.
* 
umpy, scipy - For heavy matrices handling and Ik computations.
* matplotlib - Used for plotting the live PID tuning graphs and the 3D wireframe robot model.
* jupyterlab or 
otebook - To launch the main controller interface.

### Running the Digital Twin
1. Ensure the Robot's STM32 is powered on and plugged via USB-TTL.
2. Identify the active COM port (e.g. COM4 on Windows or /dev/ttyUSB0 on Linux).
3. Open ipedal_digital_twin_controller.ipynb.
4. Update the serial port cell to match your connected path.
5. Execute the Notebook cells sequentially to initiate the handshake, launch the 3D twin, and begin commanding leg positions.
