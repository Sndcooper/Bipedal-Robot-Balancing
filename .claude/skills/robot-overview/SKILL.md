---
name: robot-overview
description: Orientation map for the self-balancing bipedal robot repo — what the robot is, the hardware truth-table (pins, servos, IK geometry), where the ACTIVE firmware lives, and the 7-variant robot_control_suite matrix. Read this FIRST before touching any firmware or GUI.
---

# Robot Overview — read this first (`robot-overview`)

A self-balancing **bipedal** robot: two wheels at the base driven by DC motors,
two articulated legs driven by 4× Dynamixel AX-12+ servos, balanced by an
**STM32F103C8T6 (Bluepill)** running a **100 Hz** control loop. It is a wheeled
inverted pendulum (Segway-style) whose leg geometry is set by on-board or
PC-side inverse kinematics.

## The one rule that prevents most mistakes

`robot_control_suite/` is a **fan-out of near-identical variants**, NOT one program.
Each variant is a self-contained `firmware/` + `gui/` pair. **They have drifted
apart at the wire-protocol layer.** A fact true in one folder is often false in
the next (e.g. `mcu_ik_engine_wireless` uses `IK1,fx,fy` comma/newline; the
flagship `rc_mcu_ik_wireless` uses `IK1 fx fy` space/pipe).

> Before editing, **pin yourself to ONE variant folder** and read *its own*
> `firmware/src/main.cpp` and `gui/serial_link.py` together. Never copy a command
> string, telemetry field, or gain default from one folder into another without
> re-checking. See the `serial-protocol` skill for the per-variant contract.

## Authoritative sources (do not restate from memory — go read these)

| Topic | File |
|---|---|
| Physical dims, mass, leg geometry, servo IDs, calibration | `docs/Robot_Specification.md` |
| Full pinout, power, USART map, RC channels | `docs/Hardware_Connections.md` |
| AX-12+ control table + buffer offsets | `docs/AX12_Control_Table_Mapping.md` |
| Project history, roadmap (Phases A–D), why things exist | `docs/Project_History_and_Roadmap.md` |
| Variant matrix + protocol notes | `robot_control_suite/README.md` |

> ⚠️ The prose docs have drifted from the code in places (default gains,
> `MAX_SAFE_TILT`, telemetry format). When a doc and a `main.cpp` disagree,
> **the code you are about to flash wins** — verify the number in that variant's
> `main.cpp`, and flag the doc conflict to the user rather than trusting the doc.

## Hardware truth-table (verify against `docs/Hardware_Connections.md`)

- **MCU:** STM32F103C8T6 Bluepill, 72 MHz, Arduino framework via PlatformIO
  (`env:bluepill_f103c8`).
- **USART1** `PA9`/`PA10` → FlySky FS-iA10B iBUS RC (115200) **or** USB-FTDI wired tuning.
- **USART2** `PA2`/`PA3` → AX-12+ half-duplex bus @ **1,000,000 baud** (10 kΩ resistor
  between PA2↔PA3; echo bytes must be discarded — see `leg-ik-and-servos`).
- **USART3** `PB10`/`PB11` → 3DR telemetry radio (115200).
- **I2C1** `PB6`(SCL)/`PB7`(SDA) → MPU6050 IMU (400 kHz).
- **Motors (L298N):** L = `PA1`(ENA)/`PB14`(IN1)/`PB15`(IN2); R = `PA0`(ENB)/`PB12`(IN3)/`PB13`(IN4).
- **Encoders:** L = `PA6`(A,IRQ)/`PA7`(B); R = `PB0`(A,IRQ)/`PB1`(B). Mirror-mounted:
  forward travel makes `encL` count up and `encR` count down.
- **Servos:** IDs `6`=Leg1-Left-hip, `14`=Leg1-Right-hip, `0`=Leg2-Left, `1`=Leg2-Right.
- **Power:** 3S LiPo (~11.1–12 V) for motors+servos; BEC 5V/3.3V for logic. **Common ground is mandatory.**

## Leg / IK constants (verify against `twin_kinematics.py` + `docs/Robot_Specification.md`)

- Servo mounts in leg frame: L=`(-30,0)`, R=`(+30,0)` mm. Femur 55 mm, tibia 100 mm.
- Leg separation 180 mm. Leg-2 is a **mirrored mount** (`LEG2_INVERTED_MOUNT`).
- AX-12 straight-down calibration: **818** for left servos, **441** for right
  (≈3.413 units/degree). This is a physical calibration, not a generic center.
- `gui/twin_kinematics.py` is the shared source of truth for the solver; the MCU
  firmware reimplements the same math in C++ (`solve_ik`, `map_angle_to_ax12`).

## Variant map (`robot_control_suite/`)

> ⚠️ **This table is stale** (predates `blue_pill_dev/`, `black_pill_dev/`,
> `rc_balance_fusion_wireless/`, `ax12_control/`, and the `legacy/` split).
> Run `ls robot_control_suite/` and read that variant's own `README.md` /
> `main.cpp` rather than trusting row-for-row details here.

RC-capable variants sit directly under `robot_control_suite/`; variants with
**no** RC receiver wired in are grouped under `robot_control_suite/wireless_no_rc/`.
That split is based on what each variant's `main.cpp` actually does, not just
its name — `mcu_balance_fusion_wireless`'s own name/README say "no RC", but its
`main.cpp` does decode iBUS (drive/steer/arm/crouch all wired up). It's kept
in `wireless_no_rc/` on the user's explicit call (name/docs win over code for
grouping purposes here) — don't silently "fix" that placement.

| Folder | IK on | Transport | Baud | RC | Use |
|---|---|---|---|---|---|
| `rc_mcu_ik_wireless` | MCU | 3DR (Serial3) | 115200 | **yes** | **Flagship** untethered RC + balancing |
| `rc_balance_fusion_wireless` | MCU | 3DR | 115200 | **yes** | RC + sensor-fusion balancer — check its own README |
| `wireless_no_rc/mcu_ik_engine_wireless` | MCU | 3DR (Serial3) | 115200 | no | Wireless GUI tuning |
| `wireless_no_rc/mcu_ik_engine_pretest_wireless` | none | 3DR | 115200 | no | Radio latency benchmark (`latency_test.py`) |
| `wireless_no_rc/mcu_balance_fusion_wireless` | MCU | 3DR | 115200 | no (per docs; code has iBUS decode — see note above) | Sensor-fusion balancer (Stage 2, forked from pretest) |
| `wireless_no_rc/mcu_pos_wireless` | — | 3DR | 115200 | no | Position/encoder-target tuner |
| `blue_pill_dev/*`, `black_pill_dev/*` | — | — | — | ? | Staged bring-up sub-projects (servo/motor/IMU/telemetry) — check each `README.md` |
| `ax12_control` | — | — | — | no | Standalone AX-12 servo control tool |

`mcu_ik_engine_wired`, `mcu_ik_engine`, and `pc_ik_engine` are **not** under
`robot_control_suite/` anymore — they moved to `legacy/` (below).

Older, still-present code (do not edit unless asked):
- `legacy/old_firmware/Balance_Rework_firmware/` — earlier single-cascade rework, colon telemetry (`PITCH:`).
- `legacy/old_firmware/mcu_ik_engine/`, `legacy/old_firmware/mcu_ik_engine_wired/`, `legacy/old_firmware/pc_ik_engine/` — superseded
  `robot_control_suite` variants (wired/on-host IK), moved out once the wireless variants took over.
- `legacy/testing/autotuner/`, `legacy/testing/mpu_inspector/`, `legacy/testing/servo_home/`, `legacy/testing/temp_stm32_uploads/` —
  earlier tuning/diagnostic tools, superseded by the tools inside each active variant's `gui/`.
- `legacy/old_firmware/PlatformIO_Firmware/`, `legacy/old_firmware/Python_Controller_Digital_Twin/` — original historical baseline.

## Where to go next

- Editing the control loop / balance / safety → `firmware-control-loop` skill.
- Adding/parsing a serial command or telemetry field → `serial-protocol` skill.
- Legs, IK, AX-12 registers, compliance → `leg-ik-and-servos` skill.
- Building/uploading firmware → `flash-firmware` skill.
