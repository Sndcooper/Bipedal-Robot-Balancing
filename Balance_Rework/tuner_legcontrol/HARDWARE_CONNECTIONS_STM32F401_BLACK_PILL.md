# STM32F401 Black Pill hardware connections

This is the authoritative wiring and programming reference for every firmware
variant under Balance_Rework/tuner_legcontrol.

## Board and programming target

- MCU: STM32F401xD, reported by CubeProgrammer as device ID 0x433 with 384 KB flash.
- PlatformIO board: genericSTM32F401CD.
- Upload/debugger: ST-Link over SWD. BOOT0 stays LOW; no FTDI boot sequence is required.

| ST-Link | Black Pill |
| --- | --- |
| SWDIO | PA13 |
| SWCLK | PA14 |
| GND | GND |
| NRST, recommended | NRST |
| 3V3, bare-board test only | 3V3 |

Never connect the ST-Link 3V3 supply while the robot external 5 V buck is
already powering the Black Pill.

## Standard motor, encoder, IMU, and AX-12 pins

| Component | Signal | Black Pill pin |
| --- | --- | --- |
| L298N left motor | ENA PWM | PA1 |
|  | IN1 / IN2 | PB14 / PB15 |
| L298N right motor | ENB PWM | PA0 |
|  | IN3 / IN4 | PB12 / PB13 |
| Left wheel encoder | A / B | PA6 / PA7 |
| Right wheel encoder | A / B | PB0 / PB1 |
| MPU6050 | SCL / SDA | PB6 / PB7 |
| MPU6050 | VCC / GND | 3V3 / GND |
| AX-12 half-duplex bus | TX / data-RX | PA2 / PA3 |

All grounds join: battery negative, L298N GND, AX-12 GND, buck GND, Black Pill
GND, IMU GND, radio GND, and encoder GND.

## AX-12 10 kOhm half-duplex circuit

Retain the existing circuit because the firmware deliberately parses its UART
echo:

    PA2 (USART2 TX) -- 10 kOhm --+-- PA3 (USART2 RX)
                                  +-- AX-12 DATA, daisy-chained to all servos

PA2 must not connect directly to AX-12 DATA. A direction-controlled transceiver
is a future improvement, not a drop-in replacement: its lack of echo would
require a matching parser change.

## 3DR and RC serial wiring

STM32F401 has no USART3. Do not connect a radio to old F103 pins PB10/PB11.

| Variant | 3DR radio UART | Black Pill TX to radio RX | Black Pill RX from radio TX | Notes |
| --- | --- | --- | --- | --- |
| mcu_balance_fusion_wireless | USART1 | PA9 | PA10 | Profiler moves to PA11. |
| mcu_ik_engine_pretest_wireless | USART1 | PA9 | PA10 | Profiler moves to PA11. |
| mcu_ik_engine_wireless | USART1 | PA9 | PA10 | |
| mcu_pos_wireless | USART1 | PA9 | PA10 | |
| ax12_control | USART1 | PA9 | PA10 | Profiler moves to PA11. |
| RC_mcu_IK_wireless | USART6 | PA11 | PA12 | USART1 remains iBUS RX on PA10. USB is unavailable while radio is fitted. |

The radio must share GND with the Black Pill. If radio TX is 5 V TTL instead
of 3.3 V TTL, level-shift it before PA10 or PA12.

## Power

    3S LiPo -> L298N motor VIN
             -> AX-12 servo supply
             -> separate regulated 5 V buck -> Black Pill 5V pin

    Black Pill 3V3 -> MPU6050 VCC

Never put the LiPo or 12 V rail into 5V or 3V3 on the Black Pill. Do not use
the L298N module 5 V regulator as the Black Pill supply; use a separate buck.
Do not power AX-12 servos from the Black Pill or its logic buck.

## Bring-up sequence

1. ST-Link only: flash a minimal build with motors and servos disconnected.
2. Verify MPU6050 and encoder readings with 3.3 V logic levels.
3. Verify L298N direction and PWM with wheels lifted.
4. Add the AX-12 bus with the 10 kOhm circuit and test one servo at a time.
5. Add the 3DR radio last and verify radio TX/RX before balancing.
