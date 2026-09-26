---
name: flash-firmware
description: Compiles and uploads the STM32 balancing bipedal robot firmware with PlatformIO — picking the correct robot_control_suite variant, its required build_flags (ENABLE_HWSERIALx / IBusBM / TX buffer), BOOT0 serial-bootloader setup, and COM-port handling.
---

# Flash & Upload STM32 Firmware (`flash-firmware`)

When the user asks to **build / upload / flash / deploy** the robot firmware,
first identify **which variant** they mean (the repo has many — see
`robot-overview`), then build/upload that folder's PlatformIO project
(`env:bluepill_f103c8`, board `STM32F103C8`).

## 1. Pick the target directory

Ask/confirm the variant if ambiguous. Current active firmwares live under
`robot_control_suite/<variant>/firmware/`. RC-capable (remote-controlled)
variants sit directly under `robot_control_suite/`; variants with **no** RC
receiver are grouped under `robot_control_suite/wireless_no_rc/`:

- `rc_mcu_ik_wireless/firmware` — flagship: RC + wireless + IK.
- `rc_balance_fusion_wireless/firmware` — RC + sensor-fusion balancer.
- `wireless_no_rc/mcu_ik_engine_wireless/firmware` — wireless GUI tuning, no RC.
- `wireless_no_rc/mcu_ik_engine_pretest_wireless/firmware` — radio latency benchmark, no RC.
- `wireless_no_rc/mcu_balance_fusion_wireless/firmware` — sensor-fusion balancer (Stage 2), no RC per its own README (its `main.cpp` does decode iBUS — flag that discrepancy if it matters for the task).
- `wireless_no_rc/mcu_pos_wireless/firmware` — position tuner, no RC.
- `blue_pill_dev/*/firmware`, `black_pill_dev/*/firmware`, `ax12_control/firmware`
  — newer variants added after this skill was last updated; `ls robot_control_suite/`
  and that variant's own `README.md` are the source of truth, not this list.

`mcu_ik_engine_wired`, `mcu_ik_engine`, and `pc_ik_engine` moved to `legacy/`
(see below) — they are no longer under `robot_control_suite/`.

Older/legacy (only if explicitly requested): `legacy/old_firmware/Balance_Rework_firmware`,
`legacy/old_firmware/mcu_ik_engine_wired`, `legacy/old_firmware/mcu_ik_engine`, `legacy/old_firmware/pc_ik_engine`,
`legacy/old_firmware/PlatformIO_Firmware`.

> There is no single "the firmware" anymore. If the user just says "flash the
> firmware," confirm the variant rather than defaulting silently.

## 2. Verify that variant's `platformio.ini`

The base is the same, but **build_flags differ per variant** and matter:

```ini
[env:bluepill_f103c8]
platform = ststm32
board = bluepill_f103c8
framework = arduino
upload_protocol = serial
monitor_speed = 115200
board_build.f_cpu = 72000000L
build_flags =
    -DHAL_UART_MODULE_ENABLED
    -DENABLE_HWSERIAL2        ; AX-12 bus on USART2 (all variants)
    -DENABLE_HWSERIAL3        ; 3DR radio on USART3 (wireless variants)
```

Extra requirements by variant:
- **Wireless (`Serial3`) variants** need `-DENABLE_HWSERIAL3`.
- **RC flagship (`rc_mcu_ik_wireless`)** additionally needs:
  `-DENABLE_HWSERIAL1` (iBUS), `-DSERIAL_TX_BUFFER_SIZE=256` (extended telemetry
  frame > default 64 B), `lib_deps = IBusBM`, and a CMSIS pin
  `platform_packages = framework-cmsis @ <2.60000.0` (CMSIS 6.x breaks on
  GCC 12 / Cortex-M3). Do not drop these — the build fails or telemetry truncates.
- **Wired variants** talk to the host on `Serial1` @ **500000** baud (not 115200).

## 3. Serial-bootloader hardware prep (Bluepill has no onboard debugger)

`upload_protocol = serial` uploads over a USB-TTL adapter on `PA9`/`PA10`:
1. Set **BOOT0 = 1 (HIGH)**, **BOOT1 = 0 (LOW)**, press **Reset** → enters bootloader.
2. Wire adapter: `TX→PA10`, `RX→PA9`, `GND→GND`.
3. Close any serial monitor / GUI holding the COM port.

(If an ST-Link is available, `upload_protocol = stlink` avoids the BOOT0 dance —
only if the user's `platformio.ini` is set up for it.)

## 4. Build & upload (run from workspace root; adjust the `-d` path)

```powershell
pio device list                                         # find COM ports
pio run -d "robot_control_suite/rc_mcu_ik_wireless/firmware"            # compile only
pio run -d "robot_control_suite/rc_mcu_ik_wireless/firmware" -t upload # flash
# target a specific port: append  --upload-port COM3
```

Note: on Windows the working dir is under a path with spaces; keep the `-d` value
quoted.

## 5. Post-upload

1. Move **BOOT0 back to 0 (LOW)**, press **Reset** (or power-cycle) to run the new firmware.
2. There is a **2-second startup delay** (`delay(2000)` in `setup()`) so AX-12
   servos stabilise before UART traffic — expect no output for ~2 s, then a
   `BOOT:OK` / `BOOT OK|` line and the legs snapping to the standing pose.
3. Motors stay **disarmed** until an `M` command (or RC arm switch). Verify
   `GYRO_PITCH_SIGN` with motors off before arming (see `firmware-control-loop`).

## Common failures
- Nothing uploads → BOOT0 not HIGH, or a monitor/GUI owns the COM port.
- Build error about CMSIS/`__has_include` on the RC variant → missing the CMSIS
  version pin.
- Telemetry line cut off mid-frame on RC variant → missing `-DSERIAL_TX_BUFFER_SIZE=256`.
- `IBusBM.h: No such file` → missing `lib_deps = IBusBM`.
