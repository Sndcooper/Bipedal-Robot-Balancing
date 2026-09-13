# Black Pill staged bring-up

Run these in order. Upload only one stage at a time. Keep the wheels lifted for
stages 2 and 5; motors are disabled at boot in every sketch.

| Stage | Purpose | Firmware UART |
| --- | --- | --- |
| `01_servo_gui` | AX-12 torque, position, and bus checks with a small GUI | FTDI USART1: PA9/PA10 |
| `02_motors_encoders` | Verify motor direction and encoder sign | FTDI USART1: PA9/PA10 |
| `03_imu` | Verify MPU6050 connection, orientation, and pitch sign | FTDI USART1: PA9/PA10 |
| `04_telemetry` | Verify 3DR and FTDI mirrored telemetry | 3DR USART6 PA11/PA12; FTDI USART1 PA9/PA10 |
| `05_full_100hz_profile` | Build the real balancing firmware with the timing profiler enabled | Same as stage 4 |

## Shared power and wiring

* Battery negative, Black Pill GND, motor driver GND, AX-12 GND, IMU GND, 3DR
  GND, encoder GND, and FTDI GND must be common.
* AX-12 VDD comes from the 3S servo rail (9--12 V), never the Black Pill or
  L298N 5 V output.
* Black Pill uses a separate regulated 5 V buck on its `5V` pin.
* AX-12: PA2 through a series resistor to DATA; PA3 connected to that DATA node.
* FTDI: TX to PA10, RX to PA9, GND to GND. Use 3.3 V logic.
* 3DR: Black Pill PA11 TX to radio RX; PA12 RX from radio TX; common GND.

Use ST-Link for these staged tests: `pio run -t upload` from each `firmware`
directory. The final profile stage is observational: leave motor enable OFF until
the direction and IMU stages pass.
