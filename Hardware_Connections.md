# Bipedal Robot - Hardware Connections Guide

This document outlines the hardware wiring and connections for the STM32-based Bipedal Robot, including the AX-12+ Servos, DC Motors with Encoders, and the MPU (Gyro/Accelerometer).

## 1. Power Distribution
**WARNING:** Ensure all grounds (GND) are connected together across all boards and power supplies!
*   **AX-12+ Servos:** 11.1V - 12V DC (Use a dedicated LiPo battery or high-current power supply).
*   **DC Motor Driver (e.g., L298N / TB6612FNG):** 12V to VMOT/VCC.
*   **STM32:** 5V (via 5V pin) or 3.3V, depending on your BEC/Step-down module.
*   **MPU (6050 / 9250):** 3.3V or 5V (Check your specific module's VCC pin requirement).
*   **Encoders:** Typically 5V or 3.3V for the Hall sensors.

---

## 2. PC Communication (Serial1)
Used to communicate with the Python Digital Twin.
*   **PA9 (TX):** Connect to USB-TTL Adapter RX
*   **PA10 (RX):** Connect to USB-TTL Adapter TX
*   **GND:** Connect to USB-TTL Adapter GND

---

## 3. AX-12+ Servos (USART2)
Dynamixel AX-12+ uses a single-wire half-duplex UART communication. The STM32 communicates with the servos via `Serial2` at 1 Mbps.
*   **PA2 (TX) & PA3 (RX):** These are the USART2 pins on the STM32.
*   **Data Line Circuit (The "Half-Duplex Hack"):**
    If you do not have a dedicated half-duplex buffer IC (like the 74LS241), you can use a simple resistor setup:
    1. Connect a **10kΩ resistor** between STM32 **PA2 (TX)** and **PA3 (RX)**.
    2. Connect **PA3 (RX)** directly to the **DATA pin** of the AX-12+ servos.
    3. Ensure the STM32 and AX-12+ share a common **GND**.
*   **Servo Power:** Connect VDD on the AX-12 to your 11.1V - 12V supply. Do not power the servos from the STM32!

---

## 4. MPU Accelerometer & Gyroscope (I2C1)
Since you are replacing the Nano with a direct I2C MPU module, use the primary I2C1 peripheral on the STM32:
*   **PB6 (SCL):** Connect to MPU SCL pin
*   **PB7 (SDA):** Connect to MPU SDA pin
*   *(Note: Ensure pull-up resistors are present on the I2C lines. Most MPU breakout boards already include them).*

---

## 5. DC Motor Connections
Based on the defined configurations in `main.cpp`:

Note: Left/right motor channels are intentionally swapped to match your physical mounting.

### Left DC Motor (Motor 1)
*   **PA1:** Motor Speed / PWM (`ENA`)
*   **PB14:** Motor Direction 1 (`IN1`)
*   **PB15:** Motor Direction 2 (`IN2`)

### Right DC Motor (Motor 2)
*   **PA0:** Motor Speed / PWM (`ENB`)
*   **PB12:** Motor Direction 1 (`IN3`)
*   **PB13:** Motor Direction 2 (`IN4`)

---

## 6. Motor Encoders (Recommended Pins)
To properly read encoder ticks, you need hardware timer pins capable of Encoder Mode. Since PA9/10 (Timer 1) are taken by Serial1, here are standard, non-conflicting Timer 2 and Timer 3 pins:

### Left Motor Encoder (C1, C2)
Suggest using **Timer 3** (Channels 1 & 2):
*   **PA6:** Left Encoder Phase A (C1)
*   **PA7:** Left Encoder Phase B (C2)

### Right Motor Encoder (C1, C2)
Suggest using **Timer 3** (Channels 3 & 4) or **Timer 4**:
*   **PB0:** Right Encoder Phase A (C1)
*   **PB1:** Right Encoder Phase B (C2)

---

## Summary Pin Map

| STM32 Pin | Function | Component |
| :--- | :--- | :--- |
| **PA0** | PWM Output | Right Motor (ENB) |
| **PA1** | PWM Output | Left Motor (ENA) |
| **PA2** | USART2 TX | AX-12+ (Buffer TX) |
| **PA3** | USART2 RX | AX-12+ (Buffer RX) |
| **PA6** | TIM3_CH1 | Left Encoder C1 |
| **PA7** | TIM3_CH2 | Left Encoder C2 |
| **PA9** | USART1 TX | USB-TTL RX (PC Comm) |
| **PA10** | USART1 RX | USB-TTL TX (PC Comm) |
| **PB0** | TIM3_CH3 | Right Encoder C1 |
| **PB1** | TIM3_CH4 | Right Encoder C2 |
| **PB6** | I2C1 SCL | MPU SCL |
| **PB7** | I2C1 SDA | MPU SDA |
| **PB12** | Digital Out | Right Motor (IN3) |
| **PB13** | Digital Out | Right Motor (IN4) |
| **PB14** | Digital Out | Left Motor (IN1) |
| **PB15** | Digital Out | Left Motor (IN2) |
