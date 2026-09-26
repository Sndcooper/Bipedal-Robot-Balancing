# rc_balance_fusion_wireless

Balancing firmware + FlySky RC remote control. This is
`mcu_balance_fusion_wireless` with a transmitter added: the balance PID, the
velocity→lean outer loop, the crouch IK, the AX-12 bus arbitration and the GUI
protocol are carried over unchanged, so **a gain set tuned on that variant is
still valid here**.

Target: STM32F103C8 Blue Pill, ST-Link over SWD. 100 Hz control loop.

## Hardware — one pin differs from the parent variant

| Port | Pins | Role |
| --- | --- | --- |
| USART1 | **PA10 RX** | **FlySky FS-iA10B iBUS in** (PA9 TX unused) |
| USART2 | PA2 / PA3 | AX-12 half-duplex bus @ 1 Mbaud |
| USART3 | PB10 / PB11 | 3DR telemetry radio @ 115200 |

Everything else (motors, encoders, IMU, power) is per
`HARDWARE_CONNECTIONS_STM32F103_BLUE_PILL.md` and unchanged.

Wire the receiver's iBUS/SENS servo-rail signal pin to **PA10**, and share
ground. The receiver is powered from the 5 V rail, **not** from PA10.

> **The tick profiler is gone from this variant.** In the parent, USART1 was the
> profiler's output (PA9 TX). The receiver needs PA10 as iBUS RX and the two
> cannot share the port, so the profiler is deleted here rather than left to
> fight the receiver. Profile on `mcu_balance_fusion_wireless`, then flash this:
> the control code is otherwise identical, so those numbers still apply.

## Channel map

`readChannel()` is 0-indexed, so Ch*n* is `readChannel(n-1)`.

| Ch | Control | Behaviour |
| --- | --- | --- |
| 1 | Steer | Differential PWM trim, ±`RC_MAX_STEER` (default 40) |
| 2 | Drive | Velocity target, ±`RC_MAX_VEL` counts/s (default 400) |
| 5 | **ARM** | HIGH = motors armed. LOW = disarm. |
| 6 | Crouch | Knob → 0..`RC_MAX_CROUCH` mm (default 40), both legs |
| 7 | Calibrate | Momentary; rising edge starts IMU cal (only while disarmed) |
| 8 | Integral kill | HIGH = hold the balance integrator at zero (debug) |

This is **not** the same map as `RC_mcu_IK_wireless` (which uses
Ch3/4/5/7/8/10). Set your transmitter to match *this* table; a model memory from
that variant will not work here.

Verify the map before trusting it: the firmware emits a raw-channel line at
2 Hz, `RC:<ch1>,...,<ch8>,LINK:<0|1>`, so you can watch the actual widths move.

## Loop timing — how RC stays out of the PID's way

The tick budget is 10 000 µs and the RC path is designed around it:

- **`IBUSBM_NOTIMER`.** `IBusBM::begin(serial)` defaults to `timerid=0`, which on
  STM32 seizes **TIM1** and runs `IBusBM::loop()` from a **1 ms ISR** — and that
  ISR's sensor branch calls `delayMicroseconds(100)` plus blocking writes. An
  interrupt that can burn 100+ µs at a time of its own choosing, 1000×/second,
  is unbounded jitter injected into a 100 Hz PID. With `NOTIMER` there is no
  timer and no ISR; frames are drained cooperatively from `loop()`.
- **Decimated to `RC_POLL_INTERVAL_MS` (10 ms).** An iBUS frame arrives every
  ~7 ms, so polling faster mostly finds an empty buffer. Stick values are held
  between polls, so the control loop always has a value and never waits.
- **Capped at `RC_BUDGET_US` (400 µs = 4% of the tick).** The drain reads bytes
  already sitting in the UART ring buffer, so it runs at memory speed — a whole
  frame measures well under 100 µs. The 2.8 ms figure people quote for a 32-byte
  frame at 115200 is *arrival* latency, not loop cost. The cap is a backstop
  against a pathological burst, not a normal operating limit.
- Worst-case stick latency is therefore ~17 ms (7 ms frame interval + 10 ms
  poll) — far below human reaction time. Raise `RC_POLL_INTERVAL_MS` to 20 to
  halve the RC cost if the budget ever gets tight.

## Where RC enters the control law

```
Ch2 drive ──► target_velocity ──► OUTER LOOP (velocity → lean, deg)
                                      │
                              targetAngle + lean_cmd
                                      │
                                      ▼
                             INNER LOOP (balance PID) ──► output
                                                            │
Ch1 steer ──────────────────────────────────────────────────┼──► left  = -output + steer
                                                            └──► right = -output - steer
```

Two deliberate choices:

- **Drive sets a velocity, not a pitch and not a PWM.** Leaning *is* how this
  robot accelerates, so a drive stick written onto `targetAngle` would fight the
  balance loop for the setpoint. Feeding the loop that already converts velocity
  error into lean keeps one authority over it.
- **Steer is applied after the PID.** The balance PID owns the *common* component
  of the two motor outputs; steering is the *differential* component, which pitch
  dynamics are blind to. So steering turns the robot without eroding the balance
  loop's authority.

With the sticks centred, `target_velocity` is 0 and this firmware behaves
exactly like `mcu_balance_fusion_wireless`.

**`TE0` makes the drive stick inert by construction** — the velocity loop is the
only thing that can turn a drive command into motion, so disabling it reverts to
pure single-loop balance. Steering still works (it is a post-PID PWM trim).

## Safety

- **Failsafe:** no valid iBUS frame for `RC_TIMEOUT_MS` (500 ms, ~70 missed
  frames) → commands zeroed, motors disarmed, `SAFETY:RC_LINK_LOST` sent.
- **Arm interlock:** the Ch5 switch must be seen LOW at least once since boot
  (and since any link loss) before it can arm. Powering up with the switch
  already HIGH will *not* arm the motors, and a link that recovers with the
  switch still HIGH will not instantly re-energise them.
- **Ch5 is always a kill switch.** Flipping it LOW disarms even under `RE0`,
  and even if the disarm originally came from the GUI. An operator reaching for
  it does not know or care which side holds authority.
- **Tilt cutoff** (`|pitch| > maxSafeTilt`) is unchanged and still latches. An RC
  arm clears the latch — that is the intended re-arm path.
- Stick over-travel is clamped: transmitters with endpoint/travel adjust above
  100% emit past 2000 µs, and un-clamped that would command ~141% of the
  configured limits.
- Ch7 calibration is refused while armed — it holds the motors off for ~1 s.

## New serial commands

Protocol is this family's `key:value` comma style with a `\n` terminator (**not**
the flagship's space/pipe style — see the `serial-protocol` skill).

| Command | Effect |
| --- | --- |
| `RE0` / `RE1` | RC authority off / on. `RE0` parks the sticks so GUI sliders can be used with a live TX nearby. It does **not** disarm on its own and does **not** stop the failsafe. The Ch5 switch keeps working as a **kill switch** under `RE0` (it can still STOP, just not ARM). |
| `RV<f>` | `RC_MAX_VEL`, counts/s at full drive stick (0–2000) |
| `RS<f>` | `RC_MAX_STEER`, PWM counts at full steer stick (0–120) |
| `RCM<f>` | `RC_MAX_CROUCH`, mm at full Ch6 knob (0–80) |

`RV`/`RS`/`RCM` are echoed in the `Updated ->` ack. New telemetry fields on the
`PITCH:` line: `RCL` link, `RCE` rc-enabled, `RCA` armed, `RCD` drive, `RCS`
steer, `TVEL` target velocity.

## Bring-up order

1. **Wheels off the ground.** Flash, connect the GUI, and confirm `BOOT:OK RC`.
2. Power the transmitter. Watch the `RC:` line and move each stick/switch in turn
   to confirm the channel map against the table above. Fix the TX, not the
   firmware, if a channel is wrong.
3. Confirm the failsafe: turn the TX off and check that `RCL` goes to 0.
4. Verify `GYRO_PITCH_SIGN` by hand-tilting with motors disarmed (see
   `firmware-control-loop`).
5. Arm via Ch5 with the robot still supported. Check steering direction at low
   `RS` before trusting it.
6. Only then balance on the ground, starting with a low `RV` (~150).

## Files

- `firmware/src/main.cpp` — firmware (RC block is at the top, `readRC()` above `setup()`)
- `firmware/platformio.ini` — `env:bluepill_f103c8_128k`, `lib_deps = IBusBM`
- `gui/serial_link.py` — protocol engine, extended with the RC state mirror
- `gui/main_gui.py`, `gui/twin_kinematics.py` — carried over from the parent variant

Build: `pio run` in `firmware/`. Flash: `pio run -t upload` (ST-Link).
