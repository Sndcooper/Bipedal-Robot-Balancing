# Digital Twin Interface Tests

This folder contains Python scripts and experiments for the digital twin visualization and Inverse Kinematics (IK).

## Files Overview:

* **`digital twin.py`**: Initial rigid body simulation linking pure python calculations to visualization interfaces. Maps out single leg joints.
* **`digital twin 2_legs.py`**: The expanded bipedal digital twin calculation connecting all joints to represent both legs accurately based on structural measurements.
* **`ax12_protocol.py`**: Serial communication protocol library explicitly written for Python to talk to the Dynamixel AX-12s via the STM32 passthrough or USB2Dynamixel dongle.
* **`pythons/`**: Miscellaneous Python dependencies and older iteration logic.