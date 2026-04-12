# Scripts and Basic Communication Tests

This folder houses top-level Python scripts focused on establishing ground communication.

## Files Overview:

* **`test_communication.py`**: A low-level PySerial test to verify that the PC can correctly open ports and send/receive handshakes to the STM32 firmware before running complex Digital Twin logic.
* **`ax12_protocol.py`**: Raw packet constructor for building instructions to configure the Dynamixel AX-12 motors directly from python.
* **`autotune.py`**: An early prototype procedural tuner file, superseded by the scripts in `tuner_opus`.