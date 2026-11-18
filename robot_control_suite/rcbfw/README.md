# 🐾 `rcbfw`: Balance Fusion + FlySky RC (main firmware & GUI)

**This is the firmware and GUI the robot runs today.** It is a 100 Hz cascaded
balancer for PAW with FlySky iBUS remote control, on-board 5-bar leg IK, AX-12
servo management and a Tkinter tuning GUI over the 3DR radio.

> Target: **STM32F103C8 Blue Pill** (128 KB die), flashed with **ST-Link over SWD**.
> Forked from [`../rc_balance_fusion_wireless`](../rc_balance_fusion_wireless) at commit `16ef5ba`.
> The two have diverged since (see [Known issues](#-known-issues)).

---

## 🧠 Control architecture

```
 Ch3 drive ─► expo ─► slew limit (1200 c/s²) ─► target_velocity
                                                    │
 encoders ─► EMA ─► vel_current ──────────► OUTER LOOP  (velocity → lean, degrees)
                                             lean = Kp_vel·e + trim_bias   (±12°)
                                             trim_bias learns only when standing
                                                    │
 MPU6050 ─► complementary filter (α 0.96) ─► INNER PID @100 Hz
                                             error = (targetAngle + lean) − pitch
                                             out = Kp·e + Ki·∫e + Kd·(−gyro)
                                                    │
 Ch4 steer ─► cubic expo ─────────────────► left = −out + steer
                                            right = −out − steer ─► L298N
```

| Layer | What it does | Key defaults |
| --- | --- | --- |
| Inner PID | Holds pitch. Derivative on measurement; integrator state clamped to 64 PWM | `Kp 78`, `Ki 650`, `Kd 3.52`, `α 0.96` |
| Outer loop | Converts velocity error into a lean angle. The trim integrator freezes while driving or braking | `Kp_vel 0.003`, `Ki_trim 0.0015`, learn band 250 c/s |
| Steering | Differential PWM added **after** the PID, so it never fights balance | max 80 PWM, expo 0.6 |
| Legs | 2-circle 5-bar IK → SYNC_WRITE all 4 servos; smoothstep pose moves; RC crouch is rate-controlled | crouch 0–40 mm @ 30 mm/s |
| Safety | Tilt cutoff latch, 500 ms RC failsafe, arm interlock, Ch7 always kills | `maxSafeTilt 25°` |

**Real-time budget:** the loop is gated at 10 000 µs. The iBUS drain is cooperative
(`IBUSBM_NOTIMER`, 400 µs cap, polled every 10 ms). Radio RX has a 300 µs cap per
tick. Each tick carries **one** Serial2 (AX-12) transaction: move, settings push,
or hold/health-poll.

---

## 🔌 Hardware

| Port | Pins | Role |
| --- | --- | --- |
| USART1 | **PA10 RX** | FlySky FS-iA10B iBUS in (PA9 unused) |
| USART2 | PA2 / PA3 | AX-12 half-duplex bus @ 1 Mbaud (10 kΩ PA2↔PA3, DATA on PA3) |
| USART3 | PB10 / PB11 | 3DR radio @ 115200 |
| I²C1 | PB6 / PB7 | MPU6050 |
| Motors | PA1/PB14/PB15 (L), PA0/PB12/PB13 (R) | L298N |
| Encoders | PA6/PA7 (L), PB0/PB1 (R) | quadrature, A on interrupt |

Full wiring and power: [`../HARDWARE_CONNECTIONS_STM32F103_BLUE_PILL.md`](../HARDWARE_CONNECTIONS_STM32F103_BLUE_PILL.md).

---

## 🎮 Transmitter channel map

This map is unique to this variant, so don't reuse a model memory from another one.

| Ch | Control | Type | Behaviour |
| :-: | --- | --- | --- |
| 3 | **Drive** | stick | Forward/back → velocity target (±800 c/s) |
| 4 | **Steer** | stick | Differential PWM, cubic expo |
| 5 | Calibrate | button | Rising edge → IMU zero (only while disarmed) |
| 6 | Target = 0 | button | Zeroes target angle, velocity target, trim and encoders |
| 7 | **ARM** | 2-pos switch | HIGH = armed. Must be seen LOW once after boot or link loss |
| 8 | Target trim ± | switch | Held = `targetAngle` slews 0.05°/s (clamp ±8°) |
| 9 | Leg select | 3-pos switch | LOW = both, MID = left, HIGH = right |
| 10 | Crouch ± | switch | Held = selected leg(s) crouch/stretch at 30 mm/s |

> **Power up with sticks centred and switches in their resting positions.** The
> firmware captures the Ch3/Ch4 centres and the Ch8/Ch10 idle positions from the
> first ~20 frames (reported as `RCCEN:`).

---

## 🚀 How to use it

### 1 · Flash the firmware
```powershell
# from the repo root (quote the path, it contains spaces)
pio run -d "robot_control_suite/rcbfw/firmware"              # build
pio run -d "robot_control_suite/rcbfw/firmware" -t upload    # flash via ST-Link
```
Wire the ST-Link as SWDIO→PA13, SWCLK→PA14, GND, and optionally NRST. BOOT0 stays LOW.

### 2 · Launch the GUI
```powershell
pip install -r robot_control_suite/rcbfw/gui/requirements.txt
cd robot_control_suite/rcbfw/gui
python main_gui.py        # enter the 3DR COM port (default COM13) → Connect
```
The GUI has three tabs:
1. **Balance Tuner**: live plot, PID, velocity-loop and RC sliders (coarse/fine zoom), servo temperatures, serial monitor, auto log-on-arm.
2. **AX-12 Tuner**: 5-bar twin, foot IK targets, compliance/torque/speed, limp and re-grip.
3. **Kinematics & Health**: per-servo raw position bypass and health.

Logs land in `gui/logs/` (gitignored). Tuning profiles save to `gui/profiles/`.

### 3 · Bring-up checklist (wheels off the ground)
1. Power on and look for `BOOT:OK RC` in the serial monitor.
2. Move each stick and switch and check that the matching field on the `RC:` line changes.
3. Turn the TX off: `RCL` should drop to 0 and the motors disarm.
4. Hand-tilt the robot with motors off. `PITCH` must follow the tilt; if it inverts, flip `GYRO_PITCH_SIGN`.
5. Arm on Ch7 with the robot supported, and check the steering direction.
6. Put it on the ground. Start with a low drive limit (`RV300`) and raise it from there.

---

## 📡 Protocol (`key:value`, comma-separated, `\n`-terminated)

**Telemetry**
| Line | Rate | Fields |
| --- | --- | --- |
| `PITCH:` | 10 Hz | `PITCH PID_OUT INT EL ER VEL MOT TILT TRIM ATE LATCH TORQ CRL CRR FX1 FY1 FX2 FY2 IK1 IK2 MOVE RCL RCE RCA RCD RCS TVEL RCT RC8 YW YT` |
| `AX12:` | 2 Hz | `MODE TQ TL CM CS MS MT` |
| `RC:` | 2 Hz | Ch1…Ch10 raw µs, `LINK` |
| `SRV:` | 1 Hz (round-robin) | `id,temp,load%,errByte,failCount` |

**Commands.** Multi-character prefixes are matched before single letters.

| Group | Commands |
| --- | --- |
| Balance | `P` `I` `D` `A` (alpha) `T` (max tilt) `S<f>` (target) `O` (offset) `C` (calibrate) `R` (reset integrals) `M` (arm toggle) |
| Outer loop | `VP` `TG` (trim gain) `TE0/1` (loop on/off) `TC` (commit trim) `TLB` (learn band) |
| RC | `RE0/1` (authority) `RV` `RVS` (slew) `RS` `RSE` `RDE` `RCM` `RCR` `RTR` |
| Legs | `CR` `CRL` `CRR` (crouch mm) `FT` `FA` (foot targets) `MD0/1` (crouch/IK mode) `MT` `HM` `SR`/`S` (servo reset) |
| Servos | `TQ0/1` (limp / re-grip) `TL` `CM` `CS` `MS` `PS<id> <pos>` `RB` (state resync) |
| Misc | `PING<tok>` → `PONG<tok>` |

Tuning commands reply with a single `Updated -> P:.. I:.. D:.. ...` line, which the GUI uses to sync its sliders.

---

## ⚠️ Known issues

- **Divergence from `rc_balance_fusion_wireless`.** That variant has the D-term
  low-pass (`DF`), the `SAT` telemetry field and the `crouchDirty` fix, which
  moves the RC-crouch bus write into the arbiter. This variant still writes the
  bus from `readRC()`, a second Serial2 transaction per tick, which rbfw
  documented as desyncing the AX-12 bus. Merging the two is the top priority.
- The logs show the pitch integrator pinned at its ±64 PWM clamp in about half
  of armed frames, which points to Ki being too high for the inner loop.
- `Wire.setClock()` and the MPU6050 DLPF are never configured (I²C runs at 100 kHz).
- No parameter persistence: a reset restores the compiled defaults.

The full analysis and fix list is in [`../../docs/PAW_Future_Roadmap.html`](../../docs/PAW_Future_Roadmap.html).

## 📂 Files
- `firmware/src/main.cpp`: all firmware; RC decoding is in `readRC()`, the control law in `loop()`
- `firmware/platformio.ini`: `env:bluepill_f103c8_128k`, `IBusBM`, 384/256 B UART buffers, CMSIS 5 pin
- `gui/main_gui.py`, `gui/serial_link.py`, `gui/twin_kinematics.py`
