# STM32F103 Blue Pill hardware connections

Authoritative wiring and programming reference for every firmware variant under
Balance_Rework/tuner_legcontrol. Supersedes
`HARDWARE_CONNECTIONS_STM32F401_BLACK_PILL.md`, which is kept only as a record of
the Black Pill attempt.

## Board and programming target

- MCU: STM32F103C8, 72 MHz Cortex-M3, no FPU. 20 KB RAM.
- PlatformIO board: `bluepill_f103c8_128k` (clone C8 dies almost always carry the
  full 128 KB flash; switch to `bluepill_f103c8` if your chip really is 64 KB).
- Upload/debugger: **ST-Link over SWD only.** There is no FTDI / ROM-bootloader
  upload environment any more — BOOT0 stays LOW permanently.

| ST-Link | Blue Pill |
| --- | --- |
| SWDIO | PA13 |
| SWCLK | PA14 |
| GND | GND |
| NRST, recommended | NRST |
| 3V3, bare-board test only | 3V3 |

Never connect the ST-Link 3V3 supply while the robot's external 5 V buck is
already powering the Blue Pill.

If ST-Link reports "no target" on a fresh clone, hold BOOT0 HIGH, reset, connect
under reset once to erase, then return BOOT0 LOW.

## UART map — the whole reason for the revert

The Blue Pill has three USARTs and the firmware uses all three:

| Port | Pins | Role |
| --- | --- | --- |
| USART1 | PA9 TX / PA10 RX | wired FTDI console, tick profiler, or FlySky iBUS in |
| USART2 | PA2 TX / PA3 RX | AX-12 half-duplex bus @ 1 Mbaud |
| USART3 | PB10 TX / PB11 RX | 3DR / SiK telemetry radio @ 115200 |

Per firmware:

| Variant | 3DR radio | USART1 used for |
| --- | --- | --- |
| mcu_balance_fusion_wireless | USART3 (PB10/PB11) | wired FTDI console, telemetry mirrored to it |
| mcu_ik_engine_pretest_wireless | USART3 (PB10/PB11) | tick profiler out (PA9 TX only) |
| mcu_ik_engine_wireless | USART3 (PB10/PB11) | free |
| mcu_pos_wireless | USART3 (PB10/PB11) | free |
| ax12_control | USART3 (PB10/PB11) | tick profiler out (PA9 TX only) |
| RC_mcu_IK_wireless | USART3 (PB10/PB11) | FlySky iBUS receiver in (PA10 RX) |

The radio must share GND with the Blue Pill. If the radio TX is 5 V TTL rather
than 3.3 V TTL, level-shift it before PB11.

USB (PA11/PA12) is not used by any firmware. It is free for the bootloader-less
board, but nothing in this tree depends on it.

## Motor, encoder, IMU, AX-12 pins

Unchanged from the Black Pill build — these were never F4-specific.

| Component | Signal | Blue Pill pin |
| --- | --- | --- |
| L298N left motor | ENA PWM | PA1 (TIM2_CH2) |
|  | IN1 / IN2 | PB14 / PB15 |
| L298N right motor | ENB PWM | PA0 (TIM2_CH1) |
|  | IN3 / IN4 | PB12 / PB13 |
| Left wheel encoder | A / B | PA6 / PA7 |
| Right wheel encoder | A / B | PB0 / PB1 |
| MPU6050 | SCL / SDA | PB6 / PB7 (I2C1) |
| MPU6050 | VCC / GND | 3V3 / GND |
| AX-12 half-duplex bus | TX / data-RX | PA2 / PA3 |

The four encoder pins land on EXTI0, EXTI1, EXTI6 and EXTI7 — four distinct EXTI
lines, so all four interrupts coexist. Do not move an encoder onto a pin whose
number collides with another EXTI in use.

All grounds join: battery negative, L298N GND, AX-12 GND, buck GND, Blue Pill
GND, IMU GND, radio GND, encoder GND.

## AX-12 10 kOhm half-duplex circuit

Retain the existing circuit — the firmware deliberately parses its UART echo:

    PA2 (USART2 TX) -- 10 kOhm --+-- PA3 (USART2 RX)
                                  +-- AX-12 DATA, daisy-chained to all servos

PA2 must not connect directly to AX-12 DATA. A direction-controlled transceiver
is a future improvement, not a drop-in replacement: its lack of echo would
require a matching parser change.

## Power

    3S LiPo -> L298N motor VIN
             -> AX-12 servo supply
             -> separate regulated 5 V buck -> Blue Pill 5V pin

    Blue Pill 3V3 -> MPU6050 VCC

Never put the LiPo or 12 V rail into 5V or 3V3. Do not use the L298N module's
5 V regulator as the Blue Pill supply; use a separate buck. Do not power AX-12
servos from the Blue Pill or its logic buck.

## What to watch after the revert

- **No FPU.** Every `float` operation is now software-emulated at 72 MHz instead
  of single-cycle at 84 MHz. The 100 Hz tick budget is 10 000 us; re-run the tick
  profiler on `mcu_ik_engine_pretest_wireless` or `ax12_control` before trusting
  the loop, and watch for `!OVR` rows.
- **20 KB RAM.** The UART ring buffers are now 256–384 B on three ports. If the
  linker reports a RAM overflow, drop `SERIAL_TX_BUFFER_SIZE` to 256 on
  `mcu_balance_fusion_wireless` first.
- **Flash.** If `bluepill_f103c8_128k` overflows, remove `-Wl,-u,_printf_float`
  and replace `%f` telemetry fields with fixed-point integers.
- **CMSIS pin.** Every env pins `framework-cmsis @ <2.60000.0`; CMSIS 6.x breaks
  on GCC 12 for Cortex-M3.

## Bring-up sequence

1. ST-Link only: flash a minimal build with motors and servos disconnected.
2. Verify MPU6050 and encoder readings with 3.3 V logic levels.
3. Verify L298N direction and PWM with wheels lifted.
4. Add the AX-12 bus with the 10 kOhm circuit and test one servo at a time.
5. Add the 3DR radio on PB10/PB11 last and verify TX/RX before balancing.
