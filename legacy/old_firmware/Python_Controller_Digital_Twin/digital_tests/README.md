# Digital Twin 3D Emulation Tests

Welcome to the digital_tests/ folder beneath the Python Controller suite. Unlike the automated tuning processes or low-level firmware integration files, this directory serves fundamentally as a 3D visualization workbench. The bipedal robot isn't a rigid body—it bends its "knees" and angles forward. Creating accurate software equations to represent this mechanically was the primary goal here.

## Focus: Inverse Kinematics & Mechanics

To keep the robot upright, the physical firmware strictly handles spinning the wheels to stay beneath the Center of Mass (CoM). However, when the user commands the robot to "crouch," the Python Controller must change the Dynamixel servo angles while simultaneously maintaining a balanced center of mass.

### Core Mathematical Scripts

* **digital twin.py**, **digital twin 2_legs.py**
  * **Role:** These Python scripts constitute the structural mathematical mapping of the robot using Inverse Kinematics (IK). They render the robot inside a standard Python Matplotlib 3D environment by charting vectors from the base (the wheels) up to the knee joints and finally to the hip/torso. 
  * **Mechanics:** They are specifically designed to ingest (X, Y, Z) constraints and iteratively calculate the precise rotational degrees the physical AX-12+ servos must adopt. This is a critical debugging safety harness; before a single physical test, you input vectors here to visually guarantee an invalid angle command is impossible and won't physically damage or break the real robot structure.

### Joint Emulation 

* **x12_protocol.py**, **set_limits.py**
  * **Role:** Pure serialization models for the Dynamixel. Since the physical servos are chained in a complex master-slave bus using half-duplex UART configurations, the math mapped by the IK sequence must be converted into physical byte arrays correctly.
  * **Usage:** Utilizing pyserial, these tests generate 0xFF header packets containing instruction bytes, IDs, and CRC Checksums perfectly formatted for the specific Dynamixel controller limits. Instead of a direct GUI to the user, they represent an abstraction layer ensuring calculations don't command invalid multi-turn joint positions.

### Sandbox Elements

* **pythons/test/test_move.py**, **	est_serial.py**, **	est_communication.py**
  * **Role:** A minor repository branch containing scratchpads and sandbox programs that verify serial baud rates and PC-to-Firmware communication consistency isolated from the intense processor load of the Matplotlib 3D rendering.
