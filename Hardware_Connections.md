# Bipedal Robot - Master Hardware Connections & Wiring Guide

This document outlines the complete hardware wiring and electrical pinouts for the STM32-based Self-Balancing Bipedal Robot, including the Dynamixel AX-12+ Servos, L298N DC Motor Drivers with Quadrature Encoders, MPU6050 IMU, 3DR Wireless Telemetry Radio, and FlySky FS-iA10B iBUS RC Receiver.

---

## 1. Power Distribution

> [!WARNING]
> **COMMON GROUND MANDATE:** Ensure all ground (GND) lines are tied together across all power supplies, STM32 board, motor drivers, radio modules, and RC receivers!

* **AX-12+ Smart Servos:** 11.1V – 12.0V DC (Dedicated high-current LiPo battery).
* **DC Motor Driver (L298N / TB6612FNG):** 12.0V DC to `VCC`/`VMOT`.
* **STM32 Bluepill:** 5.0V (via 5V pin from BEC step-down) or 3.3V logic.
* **MPU6050 IMU:** 3.3V (VCC pin connected to 3.3V logic supply).
* **3DR Telemetry Radio:** 3.3V / 5V VCC to radio module.
* **FlySky FS-iA10B Receiver:** 5V supply from BEC / motor driver 5V rail.

---

## 2. Serial Communication Architecture (USART 1, 2 & 3)

The STM32 Bluepill utilizes all three hardware serial peripherals for dedicated real-time control streams:

```
  ┌─────────────────────────────────────────────────────────────────────────┐
  │                            STM32F103C8T6                                │
  │                                                                         │
  │  [USART1]  PA9 (TX1) / PA10 (RX1) ───►  FlySky FS-iA10B iBUS (115.2k)    │
  │                                         OR USB-FTDI Serial (500k wired) │
  │                                                                         │
  │  [USART2]  PA2 (TX2) / PA3 (RX2)  ───►  AX-12A Half-Duplex Bus (1 Mbaud)│
  │                                                                         │
  │  [USART3]  PB10 (TX3) / PB11 (RX3)───►  3DR Telemetry Radio (115.2k)    │
  └─────────────────────────────────────────────────────────────────────────┘
```

### A. USART1 (`PA9` / `PA10`) — RC iBUS or Wired USB
* **RC Mode (`RC_mcu_IK_wireless`):** Connect `PA10` (RX1) to the **iBUS Servo/Sensor port** on the **FlySky FS-iA10B receiver** @ **115,200 baud**.
* **Wired Tuning Mode (`mcu_ik_engine_wired`):** Connect `PA9` (TX1) to FTDI RX and `PA10` (RX1) to FTDI TX @ **500,000 baud**.

### B. USART2 (`PA2` / `PA3`) — AX-12+ Servo Bus (1 Mbaud)
Dynamixel AX-12+ uses a half-duplex UART bus at **1,000,000 baud (1 Mbaud)**:
* **PA2 (TX2) & PA3 (RX2):** USART2 communication pins.
* **Resistor Circuit (10k Half-Duplex Hack):**
  1. Connect a **10 kΩ resistor** between STM32 **PA2** and **PA3**.
  2. Connect **PA3 (RX2)** directly to the **DATA line** of all AX-12+ servos.
  3. Share common **GND** between STM32 and AX-12 power supply.

### C. USART3 (`PB10` / `PB11`) — 3DR Wireless Telemetry Radio
* **PB10 (TX3):** Connect to 3DR Telemetry Radio **RX** @ **115,200 baud**.
* **PB11 (RX3):** Connect to 3DR Telemetry Radio **TX** @ **115,200 baud**.
* Uses pipe (`|`) frame terminator and dynamic non-blocking buffer draining ($\text{avail}/5$, capped at 20 bytes/tick).

---

## 3. MPU6050 Accelerometer & Gyroscope (I2C1)

The IMU communicates via the I2C1 hardware peripheral:
* **PB6 (SCL):** Connect to MPU6050 **SCL** (400 kHz Fast-Mode).
* **PB7 (SDA):** Connect to MPU6050 **SDA**.
* Complementary filter calculation: $\text{pitch} = \alpha \cdot (\text{pitch} + \text{gyro}_y \cdot dt) + (1-\alpha) \cdot \text{accel}_{\text{pitch}}$.

---

## 4. DC Drive Motors & Encoders

### Motor Speed & Direction Pins (L298N)
* **Left Motor (Motor 1):** `PA1` (ENA PWM), `PB14` (IN1), `PB15` (IN2).
* **Right Motor (Motor 2):** `PA0` (ENB PWM), `PB12` (IN3), `PB13` (IN4).

### Quadrature Wheel Encoders
* **Left Encoder:** `PA6` (Phase A - Ext Interrupt `countLeft()`), `PA7` (Phase B - Input Pullup).
* **Right Encoder:** `PB0` (Phase A - Ext Interrupt `countRight()`), `PB1` (Phase B - Input Pullup).
* Encoder velocity is smoothed using an Exponential Moving Average (EMA) low-pass filter ($\alpha = 0.15$).

---

## 5. FlySky FS-iA10B RC Receiver Channel Allocation

| Channel | Function | Input Range | Operational Action |
| :---: | :--- | :---: | :--- |
| **Ch 3** | Pitch / Speed Modifier | `1000 - 2000 µs` | Adjusts forward/backward pitch setpoint ($\pm 2.0^\circ$) |
| **Ch 4** | Yaw / Steering Bias | `1000 - 2000 µs` | Adjusts differential motor speed for turning |
| **Ch 5** | IMU Zero Calibrate | Switch (`> 1500`) | Rising-edge triggers zero-pitch IMU calibration |
| **Ch 7** | Motor Hardware Arming | Switch (`> 1500`) | **Arming Switch:** High enables motors; Low disables motors |
| **Ch 8** | Fine Pitch Trim | Knob (`1000 - 2000 µs`) | Live fine trim balance adjustment ($\pm 0.3^\circ$) |
| **Ch 10**| Integral Windup Kill | Switch (`> 1500`) | Active-HIGH disables PID integral accumulation |

---

## 📋 Master STM32 Pin Assignment Table

| STM32 Pin | Function | Associated Component | Protocol / Specs |
| :--- | :--- | :--- | :--- |
| **PA0** | TIM2_CH1 PWM | Right Motor Speed (ENB) | PWM 0–255 Speed Control |
| **PA1** | TIM2_CH2 PWM | Left Motor Speed (ENA) | PWM 0–255 Speed Control |
| **PA2** | USART2 TX | AX-12 Servo Bus (TX) | 1,000,000 Baud (10k resistor to PA3) |
| **PA3** | USART2 RX | AX-12 Servo Bus (Data) | 1,000,000 Baud Half-Duplex Data Line |
| **PA6** | EXTI6 Interrupt | Left Encoder Phase A | RISING Edge Interrupt (`countLeft()`) |
| **PA7** | GPIO Input | Left Encoder Phase B | Pullup Input |
| **PA9** | USART1 TX | USB-FTDI RX / Debug Serial | 115,200 or 500,000 Baud |
| **PA10** | USART1 RX | FlySky iBUS / USB-FTDI TX | 115,200 Baud iBUS Frame Input |
| **PB0** | EXTI0 Interrupt | Right Encoder Phase A | RISING Edge Interrupt (`countRight()`) |
| **PB1** | GPIO Input | Right Encoder Phase B | Pullup Input |
| **PB6** | I2C1 SCL | MPU6050 SCL | I2C1 400 kHz Fast-Mode |
| **PB7** | I2C1 SDA | MPU6050 SDA | I2C1 400 kHz Fast-Mode |
| **PB10** | USART3 TX | 3DR Radio Module RX | 115,200 Baud Telemetry Output |
| **PB11** | USART3 RX | 3DR Radio Module TX | 115,200 Baud Telemetry Input |
| **PB12** | GPIO Output | Right Motor Direction 1 (IN3)| H-Bridge Direction Line |
| **PB13** | GPIO Output | Right Motor Direction 2 (IN4)| H-Bridge Direction Line |
| **PB14** | GPIO Output | Left Motor Direction 1 (IN1) | H-Bridge Direction Line |
| **PB15** | GPIO Output | Left Motor Direction 2 (IN2) | H-Bridge Direction Line |

