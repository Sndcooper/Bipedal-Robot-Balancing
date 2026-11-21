# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

PAW is a wheel-legged self-balancing biped. It has two wheels driven by encoder gearmotors, and each of its two legs is a 5-bar linkage driven by two AX-12+ servos (4 servos total). The robot currently runs **`robot_control_suite/rcbfw/`** (STM32F103 "Blue Pill", 100 Hz). Everything else under `robot_control_suite/` is an earlier stage or a bench tool, and `legacy/` is archived. Read the `.claude/skills/robot-overview` skill first.

## Owner's standing rules (these carry as much weight as the code)

1. **Teach, don't just do.** The owner is learning their own robot. For any code you write, explain what each block does and *why* that design was chosen over the alternatives. Comments in new firmware should explain intent and physics, not restate syntax. Never make a silent architectural decision: say what you picked and what you rejected.
2. **Every physical value is reconfirmed by the owner.** Masses, dimensions, COM, inertia, gear ratios, encoder CPR, pin wiring and voltage levels count as UNCONFIRMED until the owner confirms them, even if a doc in this repo states them. Track them in `model/robot_params.yaml`. Every entry there has a `status:` field. Never promote a value to `confirmed` on your own.
3. **Report first, then questions.** For big design questions, write a report that restates each question and gives the recommendation, the reasoning, the rejected alternatives and what would change the answer. Put your questions to the owner *after* the report.
4. **Roadmap order:** (a) port to ESP32-S3 and tune the PID on the ESP32 until it is excellent; (b) build and validate a laptop simulation model; (c) LQR from that model; (d) RL on top. Don't skip ahead.

## Active migration (planned, not yet built)

The plan is to move from the STM32F103 to an **ESP32-S3-WROOM**, swap the MPU6050 for a **BNO08x**, run from a **3S 3300 mAh** LiPo, and replace the **L298N**. The decisions and reasoning are in `docs/ESP32_S3_Migration_Report.md`. The pin map in that report is a **DRAFT**: don't write firmware against it until the owner has confirmed the board/module variant, the IMU breakout, the motor driver and the measured logic levels. The new firmware goes in a **new variant folder**. Leave `rcbfw` untouched as the known-good fallback.

## Commands

```bash
pio run -d robot_control_suite/rcbfw/firmware               # build (what CI runs)
pio run -d robot_control_suite/rcbfw/firmware -t upload     # flash via ST-Link (SWD)
python -m py_compile robot_control_suite/rcbfw/gui/*.py     # GUI syntax check (what CI runs)
pip install -r robot_control_suite/rcbfw/gui/requirements.txt && python robot_control_suite/rcbfw/gui/main_gui.py
```

There are no unit tests. Validation is physical: test with the wheels off the ground first, then compare the session logs in `gui/logs/`. CI (`.github/workflows/build.yml`) only builds `rcbfw` and runs `py_compile` on its GUI. Other variants each have their own `firmware/platformio.ini` (`pio run -d <variant>/firmware`).

## Architecture: what `rcbfw/firmware/src/main.cpp` actually does

Verified from the code. Several docs and skills describe older variants, so trust the code over them.
- `loop()` is gated at 10 000 µs (`if (now - lastTime < 10000) return;`). Each tick runs in this order: `readRC()` (decimated to 10 ms, 400 µs budget) → `readIMU()` (complementary filter, α 0.96, I²C at the **default 100 kHz**) → tilt-cutoff latch → encoder velocity and yaw (EMA, `vel_alpha` 0.85) → control → `handleTelemetryRX()` (300 µs budget) → AX-12 bus arbiter → telemetry.
- **Control is two layers, not three.** The outer loop maps velocity error to lean (`Kp_vel`, plus a `trim_bias` integrator that only learns while standing still and settled). The inner PID maps pitch error to PWM, with derivative on measurement (`-gyroRate`) and the integrator *state* clamped to `MAX_INTEGRAL_PWM/Ki`. Steering is open-loop differential PWM added after the PID. The 3-layer position cascade in the `firmware-control-loop` skill belongs to other variants.
- Gains are in **PWM counts (±255) and encoder counts/s**. They are not SI units and won't carry over to a new driver, PWM resolution, encoder decode mode or loop rate.
- Encoders use **1× decoding** (`RISING` interrupt on channel A only).
- AX-12 bus: one Serial2 transaction per tick (move SYNC_WRITE / settings push / hold + health poll). The 10 kΩ half-duplex hack means every write **echoes**, and the parsers count on that echo (READ expects 8 echo + 10 reply bytes). A real direction-controlled transceiver removes the echo, so the parser must change with it.
- Telemetry is a `key:value,...\n` frame at **10 Hz** on Serial3 (3DR, 115200), cut down from 20 Hz because the 3DR air link saturated (`latency_test.py` saw 97% PING loss). The firmware and `gui/serial_link.py` are a matched pair: change both together.
- IK constants (mounts ±30 mm, femur 55, tibia 100, leg spacing 180, AX-12 calibration 818/441, leg-2 swap and negate) are duplicated in `main.cpp` and `gui/twin_kinematics.py`. Keep them identical.

## Known doc conflicts (flag these, don't trust them)

- The RC channel maps differ between `robot_control_suite/README.md`, `docs/Hardware_Connections.md` and `rcbfw/README.md`. For rcbfw, the `rcbfw` README and its `main.cpp` header are correct.
- `HARDWARE_CONNECTIONS_STM32F401_BLACK_PILL.md` calls itself authoritative for "every variant", but `rcbfw` targets the F103 Blue Pill.
- `docs/Robot_Specification.md` lists a partial mass budget (~1074 g subtotal) with no COM or inertia. All of it is unconfirmed for modelling.

## Rules from CONTRIBUTING.md

Match multi-char command prefixes before single letters in `parseCommand()`. Nothing blocking in the control path, and every UART write is guarded by `availableForWrite()`. Don't weaken the tilt cutoff, RC failsafe or arm interlock without asking. Keep folder names short (Windows path limit).

## Assets

`BipedalMeshes.zip` holds the Fusion 360 assembly (`.f3d`, `.step`) and per-part STEP/STL (base_link, ax-12, top/bottom femur, tibia_1/2, motor, battery) to use for the simulation model.
