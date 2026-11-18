# 🐾 PAW: Research Notes & Improvement Catalogue

Condensed findings from a review of `robot_control_suite/rcbfw` (firmware + GUI) and
21 session logs (`rcbfw/gui/logs`, 2026-09-19, ~16 min armed). The visual version is
[`PAW_Future_Roadmap.html`](PAW_Future_Roadmap.html).

---

## 1. System summary

| Block | Current implementation |
| --- | --- |
| MCU | STM32F103C8 (72 MHz, no FPU), 100 Hz loop, ST-Link |
| Balance | Inner PID on pitch (Kp 78, Ki 650, Kd 3.52), derivative on gyro, integral clamped at 64 PWM |
| Cascade | Outer velocity loop sets a lean angle (Kp_vel 0.003 + gated trim integrator, lean clamp ±12°) |
| Steering | Open-loop differential PWM after the PID, cubic expo (max 80) |
| Legs | 5-bar IK on the MCU (55/100 mm), calibration 818/441, SYNC_WRITE to 4× AX-12 |
| RC | FlySky iBUS, cooperative drain (no ISR), boot centre capture, debounce, failsafe, arm interlock |
| Link | 3DR radio at 115200, `key:value\n` protocol, 10 Hz telemetry |
| GUI | Tkinter, 3 tabs: tuner, AX-12 plus twin, health. Auto-logs whenever the robot is armed |

## 2. Findings from the logs

| # | Finding | Evidence | Fix |
| - | --- | --- | --- |
| 1 | Inner Ki behaves like a relay | integral at ±clamp about 50% of frames, 0.59 rail flips/s, 4–6 s velocity wobble | Ki 150–250 (or 0, with the outer trim owning the offset) plus a leak |
| 2 | Firmware fork | rcbfw lacks rbfw's D-filter, SAT and `crouchDirty`; rbfw lacks the slew, drive expo, learn band and poll flush | Merge into one firmware |
| 3 | Whole-bus AX-12 dropouts | fail count reached 255 on all 4 servos within about 4 s in 6 sessions | Single bus transaction per tick, servo power isolation, log servo voltage |
| 4 | I²C at 100 kHz | `Wire.setClock` is never called, costing ~1.4 ms per tick | `Wire.setClock(400000)` |
| 5 | No IMU low-pass filter or gyro bias | register 0x1A unset, calibration zeroes the accelerometer only | DLPF about 44 Hz, average the gyro bias at calibration |
| 6 | Coarse velocity | 1× encoder decoding; EMA corner about 2.6 Hz (the docs say 15 Hz, which is wrong) | 4× decoding or timer encoder mode |
| 7 | Poor actuation near zero | L298N, ~1 kHz PWM, no deadband compensation, median \|out\| 72 | TB6612/DRV8871, 15 kHz PWM, friction feed-forward |
| 8 | 8 tilt cutoffs | output peaked at 1.2k–3.2k before a fall | Covered by fixes 1 and 3, plus better state estimation |
| 9 | No persistence | a reset silently restores the compiled defaults | Store parameters in flash, report the reset reason |

Doc errors to correct: the docs claim 400 kHz I²C, the vel_alpha "15 Hz" figure is wrong, the old rcbfw README had the wrong channel map, and the photo caption confused the leg cranks with gripper arms.

## 3. What's novel

- **Platform:** a variable-height wheel-legged inverted pendulum built on cheap serial-bus servos and an MCU without an FPU. Comparable robots (Ascento, Upkie, Diablo) cost 10–50× more.
- **Learn-band-gated trim:** the integrator learns the centre-of-mass offset only near steady state (conditional integration).
- **Bus discipline:** one Serial2 transaction per tick on a resistor-hacked half-duplex bus.
- **RC robustness:** boot-captured centre and idle positions, debounce, and a kill switch that works under any authority mode.
- **Log-driven tuning:** every constant is tied to logged evidence, which makes the logs a potential dataset.

## 4. Sensor-side additions

| Sensor / change | What it enables | Priority |
| --- | --- | --- |
| MPU6050 DLPF, 400 kHz I²C, gyro bias | Cleaner D-term, ~1 ms freed per tick | ⭐ now |
| 4× encoders / timer encoder mode + edge-timing velocity | 4× velocity resolution, lighter filtering | ⭐ now |
| Better IMU (ICM-42688 / BMI088 via SPI) | Lower noise, higher rate, less vibration pickup | high |
| Battery voltage sense (ADC divider) | Scale PWM with voltage, low-battery warning | high |
| Motor current sense (INA219/INA226 or shunt) | Torque estimate, stall detection, energy metrics | high |
| AX-12 present position/load/voltage in telemetry | Leg state estimate, contact and sag detection | high |
| Magnetometer / 9-DoF (heading) | Heading hold, odometry | medium |
| ToF (VL53L1X) front and down | Obstacle, cliff and curb detection for climbing | medium |
| Foot/wheel contact or load (FSR / servo load) | Lift-off and jump detection | medium |
| Optical flow (PMW3901) | Slip-free odometry | medium |
| Camera (ESP32-S3 / Pi Zero 2 W / OAK-D Lite) | Line or person following, VLM control | later |
| 2D LiDAR (RPLIDAR C1) | SLAM and navigation | later |
| Mocap or AprilTag ground truth | Evaluation data for papers | for research |

Estimation upgrades: an EKF over pitch, rate, velocity and gyro bias; compensate the accelerometer for base acceleration; slip detection from encoder-IMU disagreement; a small learned estimator (TinyML).

## 5. Actuation, servo & jerkiness

- **Trajectories:** minimum-jerk quintic instead of smoothstep; stream at 50 Hz while moving.
- **Synchronised Moving Speed:** SYNC_WRITE goal and speed (addr 30–33), with speed ∝ travel.
- **Crouch setpoint filter:** second-order, ωn about 8 rad/s, before IK.
- **Compliance scheduling:** stiff for posing, soft (slope 32–64) while balancing so the legs act as suspension.
- **Centre-of-mass feed-forward:** crouch height sets a target-angle offset plus a gain lookup.
- **Bus:** native half-duplex USART (HDSEL) or a tri-state buffer; drop the 10 kΩ hack (the echo parser must change to match).
- **Power:** a separate servo rail with ≥1000 µF bulk capacitance.
- **Upgrades:** XL430/XC330 (current mode, 4 Mbps), then QDD BLDC actuators; replace the L298N with a MOSFET driver.

## 6. Control & software features

| Feature | Benefit |
| --- | --- |
| LQR per leg height (after system identification) | One principled controller in place of three hand-tuned loops |
| Heading hold (encoders + gyro Z) | Straight driving and waypoints |
| Roll balance / leaning into turns | Slopes and faster turns |
| Curb / step climbing | Capability demo |
| Jump (with faster actuators) | Showpiece |
| Binary 50 Hz telemetry + black-box ring buffer | Visible jitter, post-fall diagnosis |
| Flash-persisted parameters | Robust to resets |
| MuJoCo twin | Sim-to-real, regression tests, RL |
| Safe Bayesian optimisation auto-tuner | Automated tuning |
| CI build (`pio run`) | No broken firmware reaches the robot |
| Companion computer (ESP32-S3 / Pi) | Runs policies and vision while the MCU stays the safety layer |

## 7. Applications & robot directions

- **Education / research kit:** a sub-$300 open wheel-legged platform (CAD, firmware, twin, dataset).
- **Indoor delivery / telepresence:** height adjustment for tables and shelves, camera on top.
- **Inspection:** crouch under obstacles, climb small steps, carry sensors (thermal, gas).
- **Follow-me / carry assistant:** person following with vision.
- **Multi-robot swarm:** several PAWs cooperating to carry objects.
- **Human-robot interaction:** expressive posture (lean, bob, crouch) as body language.
- **Variants:** 2-DoF legs with knee motors; jumping version on BLDC; quadruped-wheel version (four PAW legs); a small wheeled humanoid with arms.

## 8. Paper topics

| ID | Topic | Venues | Readiness |
| -- | --- | --- | --- |
| A | Actuator-aware sim-to-real RL on low-cost serial servos | CoRL, ICRA, RA-L | needs twin |
| B | Residual RL over a cascaded PID for safe on-robot learning | ICRA, IROS | needs companion computer |
| C | Height-scheduled / LPV control of a variable-height WIP with 5-bar legs | IROS, Humanoids, RA-L | ⭐ doable now |
| D | Learned variable impedance on position-controlled servos | ICRA, IROS | medium |
| E | Safe Bayesian optimisation of balance gains | ICRA, CASE | ⭐ doable now |
| F | Learned proprioceptive estimation on an MCU (TinyML) | IROS, RA-L | medium |
| G | PAW open-source low-cost platform | HardwareX, ICRA, RAM | after cleanup |
| H | Language-to-skill with an MCU safety monitor | CoRL | later |

What gets accepted: a clear claim, strong baselines (LQR, tuned PID, prior work), many hardware trials with statistics, and open code and data. No topic guarantees acceptance.

## 9. Roadmap

1. **Consolidate (week 1):** merge rcbfw and rbfw, apply the IMU/Ki/encoder/PWM fixes, persist parameters, add CI.
2. **Smooth & observable (weeks 2–3):** jerk fixes, compliance scheduling, binary telemetry, servo power. Target: no bus dropouts.
3. **Model-based (weeks 4–6):** system identification, MuJoCo twin, LQR per height, EKF, heading hold. Target: pitch σ below 0.8°.
4. **Legs as actuators (weeks 7–9):** roll balance, leaning into turns, climbing, XL430 upgrade.
5. **Learning & papers (weeks 10+):** papers E and C, then B and A, then open release (G).

## 10. References

- Upkie: https://github.com/upkie/upkie
- Ascento (ETH Zürich): https://scholar.google.com/scholar?q=Ascento+two-wheeled+jumping+robot
- Actuator networks (Hwangbo et al. 2019): https://scholar.google.com/scholar?q=Learning+agile+and+dynamic+motor+skills+for+legged+robots
- SafeOpt: https://scholar.google.com/scholar?q=SafeOpt+safe+Bayesian+optimization
- legged_gym: https://github.com/leggedrobotics/legged_gym · MuJoCo: https://github.com/google-deepmind/mujoco · Isaac Lab: https://github.com/isaac-sim/IsaacLab
- AX-12A manual: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/ · XL430: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/
- STM32duino HardwareSerial (half-duplex): https://github.com/stm32duino/Arduino_Core_STM32/wiki/HardwareSerial
- IBusBM: https://github.com/bmellink/IBusBM
