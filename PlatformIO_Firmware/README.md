# PlatformIO Firmware

Welcome to the **PlatformIO Firmware** component for the Self-Balancing Bipedal Robot. This directory contains the entire C++ codebase responsible for the incredibly fast, real-time reactions required to keep an inherently unstable robot completely upright.

The entire project is structured to run on an STM32 Bluepill microcontroller (STM32F103C8T6) via the Arduino Framework, allowing for high-performance timer counters, hardware interrupts, multi-channel hardware PWM, and I2C buses. We rely on the PlatformIO environment instead of the basic Arduino IDE because of its powerful dependency tracking, intelligent autocomplete, and reliable build tools for ARM Cortex-M3 embedded targets.

## System Architecture

The robot essentially functions as a two-wheeled inverted pendulum—if it leans forward, it must drive forward to catch itself; if it leans backwards, it must drive backwards. However, because it has articulated servo-driven "legs," its center of gravity, physical height, and momentum are constantly shifting. 

To manage this complex interaction, the firmware has **three primary loops**:
1. **The Sensor Loop (I2C):** At an extremely high refresh rate, the firmware pulls accelerometer and gyroscope data from an onboard IMU (typically an MPU6050 or MPU9250). That noisy 6-axis raw data is filtered heavily—often via a Complementary Filter or Kalman Filter—into a clean, absolute pitch angle. This tells the robot exactly how many degrees off from straight up (0 degrees) it is.
2. **The Control Loop (PID & Motors):** The calculated pitch error is fed into a Proportional-Integral-Derivative (PID) algorithm. The output of that algorithm determines exactly how much PWM speed and in which direction to drive the base wheels. Using standard H-Bridges (L298N/TB6612), it corrects the lean before the robot falls over. Encoders on the wheels track odometry.
3. **The Sub-Processor Loop (UART):** Simultaneously, the STM32 is communicating continuously with the Python Digital Twin on a host PC. The PC calculates all the Inverse Kinematics (IK) for the joint angles and commands the STM32 via USB-TTL. The STM32 parses these massive byte frames and relays exact target positions to the Dynamixel AX-12+ smart servos running down the legs via a half-duplex UART bus.

## Directory Structure

### 1) Configuration
* **platformio.ini**
  * The configuration root for PlatformIO. This file defines the luepill_f103c8 environment, sets upload protocol methodologies (stlink or serial), defines strict build flags for optimization, hardcodes baud rates for serial monitors, and maps out third-party library dependencies (like specific MPU libraries or fast mathematical routines).
* **Hardware_Connections.md** (Found in the parent repository)
  * Since embedded C++ relies entirely on pins, do not flash this firmware without ensuring your GPIOs (PWM outputs, I2C lines, Timer Interrupts) precisely match the schematic.

### 2) Core Source Code
* **src/main.cpp**
  * The single entry point for compilation. It orchestrates setup()—which initializes the IMU, arms the TIM2/TIM3 encoder counters, configures fast PWM, and initiates Serial1 (PC) and Serial2 (Servos) buffers. It then runs the loop() where the master execution of balance checks and UART commands happen sequentially.

### 3) Dependencies & Header Inclusions
* **include/**
  * This folder contains .h or .hpp headers. Due to the high complexity, splitting specific motor controls, serial decoding functions, or PID mathematical objects into dedicated header structures improves readability across the project.
* **lib/**
  * Designed to house custom, local libraries specifically packaged for this project that aren't fetched via PlatformIO's global package registry. For instance, modified versions of the AX-12+ protocol packets or stripped-down Wire (I2C) libraries that avoid software blocking.

### 4) The Test Bench
* **	est/**
  * This folder holds the entire history of module isolation code. You cannot successfully build a complex balancing biped on day 1. The files in here (such as dc_motor_test.cpp, digitaltwin.cpp, motortest.cpp, alance_wheels.cpp) are strictly for component verification. You would temporarily disable src/main.cpp, move one of the test scripts over, and verify that your H-Bridge turns the wheel, or verify that your IMU actually reads 90 degrees when you mount it sideways, before attempting the full balancing challenge.

## Building and Uploading

### Prerequisites
1. Ensure you have **VS Code** with the **PlatformIO IDE** extension installed.
2. Connect your STM32 using either an ST-Link V2 programmer or a 3.3V USB-TTL Serial dongle via the BOOT0 jumper.
3. Open this PlatformIO_Firmware folder specifically in VS Code (do not open the parent directory).

### Commands
* Click the **PlatformIO: Build** (✓) checkmark at the bottom of the window to verify the C++ syntax and link logic.
* Click the **PlatformIO: Upload** (→) arrow. Make sure the robot is physically supported and off the ground when flashing, as unexpected PWM signals during boot sequences can occasionally cause the wheels to surge aggressively.
