<h1 align="center">🐾 PAW: a Self-Balancing Wheel-Legged Biped</h1>

<p align="center"><b>An open-source, low-cost Ascento-style robot: two wheels, two height-adjustable 5-bar legs, and a $3 microcontroller that keeps it all upright.</b></p>

<p align="center">
  <b>STM32F103 · 100 Hz cascaded balance control · 5-bar legs on 4× Dynamixel AX-12+ · FlySky RC · 3DR telemetry · Python tuning GUI</b><br>
  <a href="https://github.com/Sndcooper/Bipedal-Robot-Balancing/actions/workflows/build.yml"><img alt="build" src="https://github.com/Sndcooper/Bipedal-Robot-Balancing/actions/workflows/build.yml/badge.svg"></a>
  <img alt="MCU" src="https://img.shields.io/badge/MCU-STM32F103C8-03234B">
  <img alt="Framework" src="https://img.shields.io/badge/PlatformIO-Arduino-orange">
  <img alt="Loop" src="https://img.shields.io/badge/control%20loop-100%20Hz-success">
  <img alt="License" src="https://img.shields.io/badge/license-MIT-blue">
</p>

<p align="center">
  <img src="media/paw_front_view.jpg" alt="PAW front view" width="45%" />
  <img src="media/paw_top_view.jpg" alt="PAW top view" width="45%" />
</p>
<p align="center"><i>Left: front view with the two wheel-leg modules and JGA12V encoder motors. Right: top view; the orange parts are the AX-12-driven cranks of the 5-bar leg linkage.</i></p>

<p align="center">🎥 <a href="https://drive.google.com/file/d/1A6eR97t5H_khKykf_Axxi-ui6EwEtNq_/view?usp=sharing"><b>Watch PAW balance (Google Drive)</b></a> · <a href="media/paw_demo.mp4">raw mp4</a></p>

---

## ✨ What PAW does

- **Balances on two wheels** with a two-layer cascade. A fast pitch PID (100 Hz) holds a lean angle, and a slow velocity loop decides what that lean should be. Driving forward means asking the robot to lean.
- **Changes height while balancing.** Each leg is a 5-bar linkage driven by two AX-12+ servos. Inverse kinematics runs on the MCU and all four servos are updated in one SYNC_WRITE packet.
- **Drives by RC.** FlySky iBUS provides drive, steer, arm, trim, per-leg crouch and calibration, with a link-loss failsafe and arm interlock.
- **Tunes over the air.** The 3DR radio streams telemetry to a GUI with live plots, sliders for every gain, a 5-bar digital twin, servo health, and automatic session logs.

## 💡 Why PAW?

Wheel-legged robots such as Ascento, Diablo and Tron1 are among the most capable small robots around, but they use actuators that cost thousands of dollars. PAW asks how far you can get with **hobby parts and one Cortex-M3**. Its constraints (slow servos, a half-duplex bus, no FPU) make it a good testbed for control, estimation and **sim-to-real learning** on cheap hardware. See the [research notes](docs/PAW_Research_Notes.md) for open problems and paper ideas.

## 🚦 Start here

| You want to… | Go to |
| --- | --- |
| **Run the robot as it works today** | [`robot_control_suite/rcbfw/`](robot_control_suite/rcbfw/README.md): main firmware and GUI |
| See the analysis, improvement plan and research ideas | [`docs/PAW_Future_Roadmap.html`](docs/PAW_Future_Roadmap.html) (open in a browser) |
| Wire the robot | [`docs/Hardware_Connections.md`](docs/Hardware_Connections.md) · [`robot_control_suite/HARDWARE_CONNECTIONS_STM32F103_BLUE_PILL.md`](robot_control_suite/HARDWARE_CONNECTIONS_STM32F103_BLUE_PILL.md) |
| Check dimensions, masses and leg geometry | [`docs/Robot_Specification.md`](docs/Robot_Specification.md) |
| Browse other firmware variants | [`robot_control_suite/README.md`](robot_control_suite/README.md) |

## ⚡ Quickstart

```powershell
# 1. Flash (ST-Link on SWDIO/SWCLK/GND)
pio run -d "robot_control_suite/rcbfw/firmware" -t upload

# 2. Tune / monitor over the 3DR radio
pip install -r robot_control_suite/rcbfw/gui/requirements.txt
python robot_control_suite/rcbfw/gui/main_gui.py
```

Then work through the bring-up checklist in the [rcbfw README](robot_control_suite/rcbfw/README.md#3--bring-up-checklist-wheels-off-the-ground): wheels off the ground, verify the RC map and failsafe, check the gyro sign, and only then balance.

## 🧩 Hardware at a glance

| Subsystem | Part | Interface |
| --- | --- | --- |
| MCU | STM32F103C8T6 Blue Pill (72 MHz, no FPU) | ST-Link SWD |
| Legs | 4× Dynamixel AX-12+ (IDs 6, 14 = leg 1; 0, 1 = leg 2), 5-bar linkage (55 mm crank, 100 mm rod) | USART2, 1 Mbaud half-duplex (10 kΩ hack) |
| Wheels | 2× JGA12V 300 rpm + Hall quadrature encoders, 65 mm wheels, L298N driver | TIM2 PWM + EXTI |
| IMU | MPU6050 | I²C1 (PB6/PB7) |
| Telemetry | 3DR/SiK radio | USART3 @ 115200 |
| RC | FlySky FS-iA10B | USART1 RX, iBUS |
| Power | 3S LiPo, separate 5 V buck for logic | common ground |

## 🗂️ Repository layout

```
robot_control_suite/
  rcbfw/                       ← ⭐ main firmware + GUI (what the robot runs)
  rc_balance_fusion_wireless/  ← parent variant (has fixes rcbfw still needs, see roadmap)
  rc_mcu_ik_wireless/          ← older flagship with a different (space/pipe) protocol
  wireless_no_rc/  blue_pill_dev/  black_pill_dev/  ax12_control/   ← bring-up & tools
docs/                          ← spec, wiring, AX-12 table, history, roadmap (HTML)
legacy/                        ← archived firmware generations and one-off tools
media/                         ← photos + demo video
```

Each variant is a self-contained **firmware + GUI pair with its own serial protocol**, so always pair a firmware with its own `gui/serial_link.py`.

## 🔭 What's next

The [roadmap](docs/PAW_Future_Roadmap.html) covers four things:
- **Log-backed fixes:** integrator saturation, the rcbfw/rbfw divergence, and AX-12 bus dropouts.
- **Servo and jerk fixes:** minimum-jerk trajectories, synchronised Moving Speed, and compliance scheduling.
- **New features:** LQR per leg height, roll balance, curb climbing, and a MuJoCo twin.
- **Research directions:** actuator-aware sim-to-real RL, residual RL, height-scheduled control, and safe Bayesian tuning.

## 🤝 Contributing

Fork it, build one, and break it. Read **[CONTRIBUTING.md](CONTRIBUTING.md)** first: it lists the rules that keep the firmware and GUI in sync. Good first issues are in the *Findings* table of the [research notes](docs/PAW_Research_Notes.md#2-findings-from-the-logs). Bug reports with a session log attached get fixed fastest.

## 📑 Citing

If PAW helps your research, please cite the repository:

```bibtex
@misc{paw2026,
  author = {Vilas B S},
  title  = {PAW: A Low-Cost Self-Balancing Wheel-Legged Biped},
  year   = {2026},
  url    = {https://github.com/Sndcooper/Bipedal-Robot-Balancing}
}
```

## 📜 License

MIT © 2026 Vilas B S. See [LICENSE](LICENSE).
