# Firmware Component Backups & Test Suites

Welcome to the 	est/ directory of the PlatformIO_Firmware. This directory stands as an organized history folder composed of legacy codebase iterations and hardware-specific component isolation snippets. These snippets are exactly how the main robot logic was successfully assembled and verified piece by piece on the STM32 board.

Building a two-wheeled balancing biped is physically and algorithmically intensive. The overarching code (src/main.cpp) merges at least three disparate libraries and timing modules which are notoriously difficult to coordinate:
1. Fast-read non-blocking I2C queries for the IMU logic.
2. Dual-channel Quadrature Encoders attached to complex internal STM32 Timer Interrupts (EXTI).
3. Specialized half-duplex UART buses firing strings to standard Dynamixel smart-servos.

## Testing Procedures
If a single component breaks mechanically or software-wise upon merging, it is highly recommended to strip back the firmware layer and run one of these independent test modules to verify the individual piece of hardware hasn’t failed.

**To run a test script:**
Do not compile this complete folder simultaneously. Rename, exclude, or comment out the src/main.cpp script, then copy the file you intend to verify (such as dc_motor_test.cpp) directly into your src/ folder. Build and Flash via PlatformIO to the STM32.

## Catalog Details

### Pure Actuator Isolation
* **motortest.cpp**, **dc_motor_test.cpp**, **motor control.cpp**: These files focus exclusively on activating the dual variable-speed DC motor wheels. Use these scripts to map the EnA, EnB, IN1, IN2, IN3, IN4 H-Bridge lines, verifying forward versus backwards polarity relative to PWM signals.
* **setting ax12.cpp**: Employs an alternate software serial logic loop to interface with the AX-12+ ID configs. Used to forcibly center the structural servos to position 512 during mechanical assembly.

### Instrumentation Isolation
* **eading9axisdata.cpp**: Raw test to confirm I2C IMU connections (MPU6050 / MPU9250) pulling Gyroscopic vectors. Validates Euler angles converting internal register gravity values to measurable physical degrees.
* **alance_wheels.cpp**, **alancing tuning.cpp**: Basic static iteration loops of the single rigid-body inverted pendulum algorithm focusing strictly on PID wheel rotation variables.

### Telemetry Simulation
* **digitaltwin.cpp**, **digitaltwin_current.cpp**, **Digitaltwin2legs.cpp**: Archival C++ firmware testing scripts that established the custom byte-packet header (0xFF arrays) used to constantly relay IMU and positional data upward via Serial1 to Python without stalling the essential 10ms PID physical cycle time required so the robot wouldn't faceplant mid-transmission.

### Development History
* **old_main.cpp**, **python autoTune.cpp**: Fully compiled obsolete master scripts showcasing early tuning models, varying hardware architectures, or mathematical approaches that were eventually replaced by the highly-optimized control loop found currently in src/main.cpp.
