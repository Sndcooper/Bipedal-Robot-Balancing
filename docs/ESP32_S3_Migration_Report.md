# PAW → ESP32-S3: Migration Report

*Status: recommendations and drafts. No new firmware has been written yet. Anything marked **DRAFT** or **UNCONFIRMED** waits for your answers in §6.*

This report answers your three questions in order. For each one it gives **your question → my recommendation → the reasoning → what I rejected and why → what would change my mind**. Section 0 comes first. It walks through how your current robot actually works, because every later decision builds on it.

---

## 0. What your robot does today (read from `rcbfw/firmware/src/main.cpp`, not from the docs)

You need this picture before porting anything, because the port is mostly a *re-housing* of these same pieces.

### 0.1 The tick

```
loop() runs only when 10 000 µs have passed  → 100 Hz
 ├─ readRC()             iBUS from FlySky. Runs every 10 ms, max 400 µs, no interrupts
 ├─ readIMU(dt)          MPU6050 over I²C → complementary filter → pitch, gyroRate
 ├─ safety               |pitch| > 25° while armed → motors off, latched until re-armed
 ├─ encoder velocity     Δcounts/dt → EMA filter (0.85) → vel_current (and yaw)
 ├─ CONTROL              outer: velocity error → lean angle
 │                       inner: (target + lean − pitch) → PID → PWM ±255
 │                       steer: ± differential PWM added after the PID
 ├─ handleTelemetryRX()  3DR commands, max 300 µs per tick
 ├─ AX-12 arbiter        exactly ONE servo-bus transaction per tick
 └─ telemetry            one text line at 10 Hz over the 3DR radio
```

**Why it's built like this:** the Blue Pill has one core, so *everything* shares the same 10 ms. Each slow peripheral (radio, servo bus, RC) gets a microsecond budget so it can never delay the PID. That discipline is good. On the ESP32 we keep the idea but enforce it with hardware (two cores and task priorities) instead of by hand.

### 0.2 Things the code reveals that the docs don't say

| Finding | Where | Why it matters for the port |
|---|---|---|
| Control is **2 layers** (velocity→lean, lean→PWM), not the 3-layer cascade the old skill describes | `loop()` ~L1934–2011 | We port what is really there |
| Encoders count **only rising edges of channel A** (1× decoding) | `setup()` `attachInterrupt(...RISING)` | The ESP32's hardware PCNT gives 4× decoding for free, which means 4× finer velocity |
| I²C runs at the **default 100 kHz** (no `Wire.setClock`) | `setupMPU()` | Costs ~1.4 ms per tick (your own research notes measured this) |
| Motor PWM is `analogWrite` at the core's default frequency, 8-bit | `setMotors()` | The new driver will use 20 kHz (silent) and 10–12 bit resolution, so **gains in "PWM counts" won't transfer** |
| Telemetry was **cut to 10 Hz** because the 3DR link saturated, and `latency_test.py` saw **97% PING loss** | telemetry comment ~L2103 | Hard evidence for §3 |
| AX-12 parsing **relies on the echo** from the 10 kΩ hack (READ = 8 echo + 10 reply bytes) | `pollLegServosTask()` | If the bus circuit changes, this parser must change too |

---

## 1. Question 1: Hardware for the ESP32-S3, BNO08x, 3S battery, L298N replacement and power distribution

### 1.1 Replace the L298N? **Yes.**

**Your question:** the L298N is a bottleneck and runs hot. Should it be replaced with a "DRV88xx", or should you keep it?

**Why the L298N hurts a balancer specifically**
- It is a **bipolar (Darlington) bridge**, so it loses roughly **2–4 V** across itself, and that drop grows with current. From an 11.1 V pack the motor sees only ~7–9 V, which means less torque and less top speed exactly when the robot needs a hard catch.
- The lost voltage becomes heat: P ≈ V_drop × I. At 2 A that is ~4–6 W in a small heatsink. That is your heating.
- The drop is non-linear at small PWM, which creates a **deadband**. Small corrections, the ones that matter most when balancing, simply don't move the wheel. The PID compensates by getting jumpy.
- Its **onboard 5 V regulator is linear** (78M05-type). Feeding the STM32, sensors and receiver from it means dropping 12.6 → 5 V as heat: (12.6 − 5) × 0.4 A ≈ 3 W in a tiny part. That is the *second* heat source, and a brownout risk when the motors pull the battery down.

**Candidates checked against your 3S pack (12.6 V fully charged, plus regen spikes):**

| Driver | Motor voltage limit | Current | Verdict |
|---|---|---|---|
| **DRV8833** | 10.8 V recommended, 11.8 V abs max ([TI](https://ti.com/document-viewer/DRV8833/datasheet)) | 1.5 A | ❌ **Not usable on 3S.** A charged pack exceeds its absolute maximum |
| TB6612FNG | 15 V max | 1.2 A continuous | ⚠️ Voltage margin is too thin and the current is too low for stall |
| **DRV8871** (one per motor) | 6.5–45 V ([TI datasheet](https://ti.com/lit/gpn/DRV8871)) | 3.6 A peak, **adjustable current limit** via the ILIM resistor (64 kV / R_ILIM, e.g. 32 kΩ ≈ 2 A) | ✅ **Recommended** if motor stall current ≤ ~2.5 A |
| **Cytron MDD3A** (dual) | 4–16 V ([datasheet](https://cdn.shopify.com/s/files/1/0176/3274/files/MDD3A_Datasheet.pdf)) | 3 A continuous / 5 A peak (<5 s) per channel, up to 20 kHz PWM, 3.3 V logic | ✅ Alternative if stall current is higher. Easy to buy in India. Only 3.4 V of margin above 12.6 V |
| BTS7960 | 5.5–27 V | 43 A | Overkill: big and heavy for 12 V gearmotors |

**Recommendation: 2× DRV8871 breakouts**, wired as IN1/IN2 PWM (no separate enable pin).
- **Why:** it has a huge voltage margin (45 V vs 12.6 V), so regen spikes are harmless. Its MOSFET bridge drops well under 1 V at 1 A. Its **hardware current limit** protects the gearboxes, the battery and the 5 V rail from stall surges, and the L298N has nothing equivalent. Its IN1/IN2 interface supports drive, coast *and* brake modes.
- **What would change this:** if your measured **stall current is above ~2.5 A** (§6, question Q-M3), choose the MDD3A. I can't decide this without the motor's real number, so it's question #3 in §6.

### 1.2 Power distribution

```
3S LiPo 3300 mAh (9.9 V empty … 12.6 V full)
  │
  ├─ main switch ─ main fuse (≈10 A blade) ─┬───────────────────────────────────────────┐
  │                                         │                                           │
  │                     ┌───────────────────┼──────────────────┐                        │
  │                     ▼                   ▼                  ▼                        ▼
  │          Motor drivers VM      AX-12 bus V+ (direct,   5 V buck, 3 A          Battery-sense divider
  │          + 470–1000 µF low-ESR  as today) + 470 µF     (e.g. MP1584/          100 kΩ / 22 kΩ → GPIO1
  │          cap AT the driver                             Pololu D36V28F5)       (12.6 V → 2.27 V)
  │                                                           │
  │                                         ┌─────────────────┼──────────────┬──────────────┐
  │                                         ▼                 ▼              ▼              ▼
  │                                ESP32-S3 devkit 5V     3DR radio     FlySky RX      AX-12 bus
  │                                (its LDO → 3.3 V)        (5 V)         (5 V)      buffer chip (5 V)
  │                                     │
  │                                     └─ 3.3 V → BNO08x, encoders (if 3.3 V works, see §1.4)
  │
  └─ ONE common ground: battery −, drivers, buck, ESP32, servos, radio, receiver, IMU
```

**5 V current budget (estimates; I'll confirm against your exact parts)**

| Load | Typical | Peak |
|---|---|---|
| ESP32-S3 with Wi-Fi transmitting | ~100 mA | ~350–500 mA |
| 3DR radio (if kept) | ~50 mA | ~150+ mA at full TX power |
| FlySky FS-iA10B | ~60 mA | ~80 mA |
| BNO08x + encoders + buffer | ~40 mA | ~60 mA |
| **Total** | **~0.25 A** | **~0.8 A** |

A **3 A buck** gives about 3× headroom. This is why I recommend a *separate switching buck* and not the L298N's linear regulator: a buck converts the extra voltage efficiently instead of burning it, and it stays in regulation when a motor stall drags the pack down. **A brownout here reboots the ESP32 mid-balance, which means a fall.**

**Why the servos stay on raw battery:** AX-12s draw up to ~1 A or more each when stalled. Putting four of them on a 5 V buck makes no sense, because they are rated for 9–12 V (11.1 V recommended). ⚠️ A fully charged 3S pack is 12.6 V, slightly above the 12 V rating. You already run them this way, but it's worth knowing: avoid parking the robot at full charge under heavy servo load.

**Battery sense:** a divider into an ADC1 pin lets the firmware disarm at ~9.9 V under load (3.3 V/cell) and warn at ~10.5 V. It must be **ADC1**, because ADC2 cannot be used while Wi-Fi is on.

### 1.3 ESP32-S3: what changes and why it's a real upgrade

| Feature | Blue Pill (F103) | ESP32-S3 | What we use it for |
|---|---|---|---|
| CPU | 72 MHz, 1 core, **no FPU** | 240 MHz, **2 cores, single-precision FPU** | LQR matrix math and RL inference become cheap |
| Encoders | interrupt per edge | **PCNT hardware counters**, 4× quadrature, glitch filter | zero CPU, 4× resolution |
| PWM | `analogWrite` | **LEDC**, 20 kHz at 10–12 bit | silent motors, finer control |
| UARTs | 3 | 3, any pin via GPIO matrix, **RS-485 half-duplex mode** that drives a DIR pin automatically | AX-12 bus without the echo hack |
| Radio | none | Wi-Fi + BLE | telemetry, tuning, **OTA flashing** |
| ⚠️ Pins | some 5 V tolerant | **NONE are 5 V tolerant** (3.6 V absolute max) | must measure every external signal level (§1.4) |

**Software architecture (planned):** FreeRTOS tasks instead of one `loop()`.
- **Core 1 → `controlTask`** (highest priority). A hardware timer wakes it every 1/f seconds. It reads the IMU and encoders, runs safety and control, and writes the PWM. Nothing else runs on this core, so Wi-Fi can never steal time from it.
- **Core 0 → `commsTask`** (Wi-Fi/UDP telemetry and commands), **`rcTask`** (iBUS), and **`legTask`** (AX-12 bus, IK).
- **Shared state** passes between tasks through a small struct copied under a lock (a snapshot), so the control task never waits on the network.

This is the ESP32 version of the budget discipline you already have. The difference is that it's enforced by the scheduler, not by manual µs counters.

### 1.4 Logic levels: things YOU must measure before wiring (multimeter, 2 minutes each)

Because no ESP32-S3 pin tolerates 5 V, each of these decides a circuit:

| Signal | How to measure | If ≤ 3.3 V | If ~5 V |
|---|---|---|---|
| **AX-12 DATA line**, idle, servos powered, MCU disconnected | DC volts, DATA → GND | the simple path is possible | needs the buffer circuit below |
| **Encoder A/B high level** with encoder VCC = 5 V, then retry with VCC = 3.3 V | turn the wheel slowly and read the max | power encoders from 3.3 V | divider or level shifter |
| **iBUS signal** (FS-iA10B servo/iBUS port) | DC volts with the TX on | direct | divider (10k/20k) |
| **3DR radio TX pin** | DC volts, idle | direct | divider |

**AX-12 bus circuit (DRAFT, depends on the DATA measurement):**
- **Option A (recommended):** use a direction-controlled buffer. ESP32 TX goes through a 74HCT1G126 powered at 5 V onto DATA; its "T" type input accepts 3.3 V logic as HIGH. DATA comes back through a 74LVC1G125 powered at 3.3 V to ESP32 RX; LVC inputs tolerate 5 V. ESP32 UART1 runs in RS-485 half-duplex mode and drives the DIR pin, so the hardware flips direction exactly when the last bit leaves, with no `flush()` busy-wait.
  - **Consequence:** there is no more echo. The READ parser changes from "skip 8 echo bytes, read 10" to "read 10". I'll walk you through that line by line when we write it.
- **Option B:** keep the 10 kΩ hack, but only if the DATA line measures ≤ 3.3 V. It is simpler, but it's out of spec for the S3 if the servo drives 5 V.

### 1.5 BNO08x (replacing the MPU6050)

- **Use SPI, not I²C.** BNO08x chips use non-standard I²C clock stretching, which commonly causes timeouts and resets on ESP32. SPI avoids it ([discussion](https://github.com/bradcar/bno08x_i2c_spi_MicroPython)). SPI needs CS, SCK, MOSI, MISO, INT and RST, and PS0/PS1 strapped for SPI (on most breakouts that is a solder jumper).
- **Reports to use:** *Game Rotation Vector* (gyro + accel fused, no magnetometer, because motors and magnets near the wheels corrupt magnetometer heading) for pitch, plus *Calibrated Gyroscope* for pitch rate (the D-term). Rotation vectors run at up to ~400 Hz ([Luxonis BNO08x table](https://docs.luxonis.com/hardware/platform/sensors/imu/bno08x)). I'll confirm the exact maximum rates on your specific chip and breakout.
- **What this replaces:** your complementary filter (α 0.96) and the `GYRO_PITCH_SIGN` hack. The BNO does the fusion on-chip. We still must **verify the axis mapping** (which BNO axis is pitch, and its sign) with the motors off, using the same "tilt it by hand" test.

### 1.6 Draft pin map (DRAFT: do not wire until §6 Q-B1 is answered)

Assumes an ESP32-S3-DevKitC-1 with a WROOM-1 **N16R8/N8R8** module (octal PSRAM). It avoids:
- strapping pins 0/3/45/46,
- USB 19/20,
- flash/PSRAM 26–37,
- UART0 bridge pins 43/44,
- GPIO48, the RGB LED on many devkits.

| Function | GPIO | Peripheral | Why this pin |
|---|---|---|---|
| Battery sense | 1 | ADC1_CH0 | ADC1 works with Wi-Fi on |
| Left motor IN1 / IN2 | 4 / 5 | LEDC 20 kHz | not strapping, so motors can't twitch at boot |
| Right motor IN1 / IN2 | 6 / 7 | LEDC 20 kHz | same |
| Left encoder A / B | 15 / 16 | PCNT unit 0 | free (no 32 kHz crystal used) |
| Right encoder A / B | 17 / 18 | PCNT unit 1 | |
| BNO08x CS / MOSI / SCK / MISO | 10 / 11 / 12 / 13 | SPI2 (FSPI) | these are the **native IOMUX** SPI pins, the fastest and cleanest routing |
| BNO08x INT / RST | 9 / 14 | GPIO | |
| BNO08x PS0/WAKE (optional) | 21 | GPIO | |
| iBUS in (FlySky) | 38 | UART0 RX remapped | console moves to native USB-CDC |
| 3DR TX→radio RX / radio TX→RX | 39 / 40 | UART2 | only if you keep the 3DR |
| AX-12 TX / RX / DIR | 41 / 42 / 47 | UART1, RS-485 mode | |
| Spare | 2, 8 | | buzzer / status LED / e-stop |

**Boot safety:** at power-up and during flashing, ESP32 pins float for a moment. Put **10 kΩ pull-downs on all four motor IN pins** so the bridges stay off (coast) until the firmware takes control. The DRV8871 has weak internal pull-downs, and external ones make this certain.

---

## 2. Question 2: A faithful model of the robot on your laptop

**Your question:** build a good, faithful model of the real biped on the laptop. Every dimension, mass and COM must be reconfirmed by you, and I should tell you exactly how to measure each one.

### 2.1 Which simulator: **MuJoCo**

- **Deciding reason:** your leg is a **closed kinematic chain** (5-bar: two servo cranks joined at the knee and foot). **URDF can only describe trees**, so Gazebo, Isaac or PyBullet would need workarounds. MuJoCo's MJCF closes loops natively with equality constraints (`<equality><connect>`).
- **LQR:** MuJoCo can linearize the model numerically (`mjd_transitionFD`). That gives the A and B matrices for LQR directly, at any leg height.
- **RL:** MuJoCo is the standard for legged and balance RL, and it runs fast on CPU. MJX/Playground use the GPU if you have one. Isaac Lab needs an NVIDIA RTX GPU (§6 Q-S1 asks about your laptop).
- **Rejected:** Gazebo (heavier, URDF tree limit), PyBullet (no longer maintained), Simulink/Simscape (licence cost, weaker RL tooling).

**Two models, on purpose:**
1. A **reduced analytic model**: a wheeled inverted pendulum with leg height as a parameter, in Python. You'll understand every equation, and it's what LQR is designed on.
2. A **full MuJoCo model** built from your CAD meshes (`BipedalMeshes.zip`) with measured masses. It's used to check the reduced model and to train RL.

If the two disagree, a measurement is wrong. That cross-check is the point of having both.

### 2.2 What to measure, and exactly how

Every value goes into [`model/robot_params.yaml`](../model/robot_params.yaml) with `status: UNCONFIRMED` until you sign it off. Measure in **two phases**, because the electronics are changing:

**Phase M1: now (doesn't depend on the new electronics)**

| # | Quantity | Tool | Procedure |
|---|---|---|---|
| M1.1 | Each part's mass | kitchen scale, ±1 g | Weigh every part listed in the YAML separately: body, each femur/tibia, each servo with horn, each motor with bracket, each wheel, battery. Weigh three times and record the average. |
| M1.2 | Link lengths (hole centre to hole centre) | calipers | For each femur and tibia, measure edge-to-edge distance between the two holes, then add one hole diameter. Compare with 55 / 100 mm. |
| M1.3 | Servo mount spacing | calipers | Horn axis to horn axis on each leg (the IK assumes 60 mm). Also the spacing between the two legs (assumed 180 mm). |
| M1.4 | Wheel **effective** rolling radius | tape | With the robot's weight on it, roll 10 full turns in a straight line. r = distance / (20π). Tyres squash, so this is not the diameter divided by 2. |
| M1.5 | Encoder counts per wheel turn | firmware | Mark the wheel, turn it exactly 10 turns by hand, and read the counts (4× decode on the new board). Gives CPR × gear ratio directly. |
| M1.6 | Motor resistance | multimeter Ω | Across the motor leads, at 4–5 shaft positions; take the lowest. |
| M1.7 | Stall current | bench supply or multimeter in series | At 11.1 V, hold the wheel firmly for **under 2 s**. This is the value that picks the motor driver (§1.1). |
| M1.8 | Motor model and gear ratio | label or seller page | Photo of the label or the purchase link. |

**Phase M2: after the new electronics are mounted (they change mass and COM)**

| # | Quantity | Method |
|---|---|---|
| M2.1 | Total mass | scale, everything installed, at the standing leg pose |
| M2.2 | **COM height above the wheel axle** | (a) **Balance-edge method:** lay the robot on its side on a thin rod parallel to the wheel axle and slide it until it balances; the COM is directly above the rod, so measure rod-to-axle distance. (b) **Plumb-line:** hang it from two different points, draw the vertical through each, and the intersection is the COM. **Do it at 2–3 leg heights**, since crouch moves the COM. |
| M2.3 | COM fore/aft offset | the same methods along the other axis. This is what your `trim_bias` integrator has been learning. |
| M2.4 | **Pitch inertia** about the COM | **Bifilar pendulum:** hang the robot on two parallel strings (length L, spacing D) so the pitch axis is vertical. Twist it slightly, time 20 oscillations, T = total/20. Then I = m·g·D²·T² / (16·π²·L). |
| M2.5 | Wheel and rotor inertia | Bifilar on one wheel; the motor rotor reflected through the gearbox comes from the motor sysID (M2.7) |
| M2.6 | IMU position and axes | ruler: x/z from the axle, plus a photo showing the chip's axis markings |
| M2.7 | Motor dynamics (system ID) | wheels in the air, PWM steps at 20/40/60/80%, log wheel speed at the full loop rate over Wi-Fi → gain K and time constant τ per motor. Also the **deadband**: the smallest PWM that starts the wheel turning, in each direction. |

**Cross-check and validation** (how you'll *know* the model is faithful):
- Assign your measured masses to each body in Fusion (override the density). Fusion then reports COM and inertia, which should agree with M2.2/M2.4 to within roughly 10%.
- **Falling test:** motors off, start at ~3°, let it fall, and log pitch at 200 Hz. Run the same initial condition in the sim. The time to fall to 25° should match closely. This one test validates mass, COM height and inertia together.

---

## 3. Question 3: micro-ROS? Wi-Fi vs 3DR telemetry? What latency? Is 200–400 Hz needed?

### 3.1 micro-ROS: **not now** (keep the option open)

**Deciding fact:** the balance controller runs **on the ESP32**, now and later (PID → LQR → a small RL policy, which all fit on the S3). The laptop link is only for **tuning, logging and supervision**. Link latency therefore never sits inside the balance loop, and it only affects how quickly you see data and how quickly a slider change lands.

| | Custom UDP (recommended) | micro-ROS |
|---|---|---|
| What runs where | ESP32 → Python GUI | ESP32 → micro-ROS **agent** on the laptop → ROS 2 DDS → your nodes |
| Extra pieces | none | ROS 2 install, agent, message definitions, XRCE-DDS memory on the MCU |
| What you gain | full understanding of every byte; same GUI style you know | rosbag, rviz, plotjuggler, ros2_control, the ecosystem |
| Risk for you now | low | a second big new thing alongside the hardware migration |

**Recommendation:** a custom **binary protocol over Wi-Fi UDP**. A packed C struct goes out with a sequence number and CRC on every control tick (or every second tick), and commands come back the same way. I'll write the ESP32 side as a separate comms task, so switching the transport to micro-ROS later only replaces that one module. Revisit micro-ROS when you want ROS tools (rosbag of RL rollouts, navigation, a ROS-based sim pipeline).

### 3.2 Wi-Fi vs the 3DR radio

| | 3DR SiK @ 115200 | ESP32 Wi-Fi (UDP) |
|---|---|---|
| Measured on *your* robot | downlink saturated at ~2.7 kB/s; 97% PING loss before telemetry was halved to 10 Hz | not yet measured |
| What 200 Hz full-state logging needs | 200 Hz × ~64 B ≈ **12.8 kB/s → impossible** | trivial (Wi-Fi carries Mbit/s) |
| Latency | tens of ms or more (TDM half-duplex air protocol, buffering) | typically a few ms on a good local network **with Wi-Fi power-save off**, with occasional spikes. We will **measure it** with a PING/PONG test like your existing `latency_test.py` rather than trust quoted numbers. |
| Range | long (hundreds of m) | room/house scale, or the ESP32 runs its own access point |
| Flashing | — | **OTA upload over Wi-Fi** (`upload_protocol = espota`), only allowed while disarmed |

**Recommendation:** Wi-Fi becomes the main telemetry and tuning link. Keep the 3DR only if you run outdoors beyond Wi-Fi range (§6 Q-C1). **The FlySky RC stays as the independent arm/kill link.** Wi-Fi can drop out, and a robot that loses Wi-Fi must keep balancing, while the RC switch must always be able to kill the motors. That separation is a safety rule, not a preference.

### 3.3 Is 200–400 Hz needed?

**Physics first.** A wheeled inverted pendulum falls away from upright at roughly ω ≈ √(g/l), where l is the COM height above the axle. With l ≈ 0.10–0.15 m (unconfirmed, §2 M2.2), ω ≈ 8–10 rad/s. The robot doubles its tilt roughly every 70–90 ms. A common rule of thumb is to sample 20–50× faster than the closed-loop bandwidth you need (a few Hz), which gives about **100–250 Hz**. So 100 Hz *works* (it already balances), but it is close to the lower edge.

**What faster buys you:** less delay in the loop. That allows a stronger D-term without oscillation and smoother catches.

**What it costs (porting traps):**
- **Velocity from encoders gets noisier per tick.** At 400 Hz each tick sees a quarter of the counts. The 4× PCNT decoding exactly offsets this, which is one more reason to use it.
- **Filter constants depend on the rate.** EMA α = 0.85 at 100 Hz is *not* the same filter at 400 Hz. We'll define filters by **cutoff frequency in Hz** and compute α from the rate.
- **Gains are in PWM counts and counts/s.** On the ESP32 we move to **SI units**: radians, rad/s, m/s, and motor command as a fraction of supply voltage. Gains then survive a new driver, PWM resolution or rate change. **LQR requires SI units anyway**, so this is a prerequisite for your roadmap, not extra work.
- **AX-12 legs don't need it.** The bus and servo internals limit them to ~50–100 Hz, so the leg task keeps its own slower rate.

**Recommendation:** run the **balance loop at 200 Hz** with the IMU at 400 Hz, and make the rate a single compile-time constant. Then **measure** 100 vs 200 vs 400 Hz on the real robot with identical SI-unit gains, and let the data decide. Beyond ~250 Hz I expect diminishing returns, because the gearmotor's own time constant (tens of ms) dominates. The motor sysID in M2.7 will show exactly that.

---

## 4. The roadmap you described, and how each step feeds the next

1. **Port the PID to the ESP32**, with tuning over Wi-Fi (I'm reading your "esptune" as "tune on the ESP32"; §6 Q-T1 confirms this). The same 2-layer structure, in SI units, with full-rate logging.
2. **Measure → model** (§2): Phase M1 now, M2 after the port.
3. **Validate the model** against logged real data (the falling test, plus closed-loop PID runs replayed in sim with the same gains).
4. **LQR**, designed on the validated model, with gains scheduled on leg height. Compare it against the tuned PID on the same disturbance tests.
5. **RL**: train in MuJoCo with domain randomization around *your measured* values and uncertainties. Start with **residual RL** (the policy adds a correction on top of LQR), which is the safer first step. Deploy a small network on the ESP32.

Each step only works if the step before it is honest. That's why the measurements carry so much weight.

---

## 5. What I deliberately did NOT do yet

- **No ESP32 firmware and no final pin map.** Both depend on the answers below.
- **No changes to `rcbfw`.** It stays your working fallback. The ESP32 firmware goes in a new folder.

---

## 6. Questions for you

**Board and parts**
- **Q-B1.** Exact ESP32-S3 board: brand, model, and the module marking on the metal can (e.g. "ESP32-S3-WROOM-1 **N16R8**"). A photo of both sides is ideal.
- **Q-B2.** Which BNO08x breakout (Adafruit 4754, SparkFun BNO086, generic purple "GY-BNO08X", ...)? Does it have PS0/PS1 jumpers?
- **Q-B3.** The motor: exact model (e.g. "JGA25-370, 12 V, 300 rpm, 1:xx"), with a photo of the label or the purchase link. Plus your stall-current measurement (M1.7).
- **Q-B4.** Battery C-rating and connector type.

**Measurements for §1.4** (multimeter): AX-12 DATA idle voltage; encoder output high level at 5 V and at 3.3 V supply; iBUS signal level; 3DR TX level.

**Decisions**
- **Q-D1.** Motor driver: 2× DRV8871 (recommended) or Cytron MDD3A? Which can you buy?
- **Q-C1.** Keep the 3DR radio (outdoor range) or go Wi-Fi only plus FlySky?
- **Q-C2.** Wi-Fi mode: the robot joins your home router, or the robot runs its own access point?
- **Q-T1.** What exactly do you mean by "esptune"? A tool, or just tuning on the ESP32?
- **Q-S1.** Laptop: OS, CPU, RAM, and does it have an NVIDIA GPU? This decides MuJoCo CPU vs MJX/Isaac for RL.
- **Q-F1.** Name for the new firmware folder (suggestion: `robot_control_suite/esp32s3_bal/`).

---

*Sources checked for this report:* [TI DRV8871 datasheet](https://ti.com/lit/gpn/DRV8871) · [TI DRV8833 datasheet](https://ti.com/document-viewer/DRV8833/datasheet) · [Cytron MDD3A datasheet](https://cdn.shopify.com/s/files/1/0176/3274/files/MDD3A_Datasheet.pdf) · [Espressif ESP32-S3 datasheet](https://documentation.espressif.com/esp32-s3_datasheet_en.html) · [Espressif boot-mode/strapping](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html) · [BNO08x report rates (Luxonis)](https://docs.luxonis.com/hardware/platform/sensors/imu/bno08x) · [BNO08x I²C clock-stretching notes](https://github.com/bradcar/bno08x_i2c_spi_MicroPython)
