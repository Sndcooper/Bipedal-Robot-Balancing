# Serial Checkers & Root Communication Tests

Welcome to the oot_tests/ directory within the Python Digital Twin. Unlike the full Master Controller Notebook (ipedal_digital_twin_controller.ipynb), this folder is strictly comprised of the simplest, barebones PC-to-Robot handshakes possible.

When working with split architectures where a high-level PC interfaces with an embedded STM32 microcontroller via high-speed Serial ports (1 Mbps+), tracking down a connection error is extremely difficult without isolation. If the robot stumbles, or the Digital Twin visualization is completely unresponsive, the scripts in this folder act as the lowest-level diagnostics—verifying the hardware bridges.

## Files Description & Diagnostic Workflows

### 1) Ping & Handshake Protocols
* **	est_communication.py**
  * **Role:** The primary diagnostic utility tool. This script relies purely on standard pyserial without integrating complex GUIs or Inverse Kinematics libraries. 
  * **Procedure:** It opens the active COM port, listens for the specific sequence of bytes coming from the physical STM32 UART transmitter (such as the IMU 0xFF arrays), parses a small snapshot, and confirms via CLI terminal whether the baudrate and hardware wiring are successfully synchronized. If this script throws a SerialException or returns garbled garbage bytes, do not launch the main Jupyter Notebook until it is resolved.

### 2) Automated Tuning Sandbox
* **utotune.py**
  * **Role:** A legacy testing branch for the 	uner_opus/ auto-tuning system. 
  * **Usage:** Before combining the UI with the step-response mathematical generators, this terminal-only script verified that Python could deliberately inject arbitrary Proportional, Integral, or Derivative byte commands into the STM32 memory over USB.

### 3) Raw Servo Serialization Layer
* **x12_protocol.py**
  * **Role:** While identical implementations exist in the digital_tests/ folder, this root-level version was used to test immediate joint override behaviors via the COM port connection line. By bypassing the primary Jupyter IK controller, one can construct an AX-12 instruction packet manually via the CLI to check exactly how a given joint on the hardware body responds sequentially.

