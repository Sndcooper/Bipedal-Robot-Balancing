# bfrc — balance fusion + RC receiver (blue_pill_dev)

Balancing firmware with the FlySky receiver integrated, a re-mapped channel
layout, a loop-body timer, and a GUI tab for calibrating the transmitter.

Derived from `tuner_legcontrol/rc_balance_fusion_wireless`. The balance PID, the
velocity→lean outer loop, the crouch IK and the AX-12 bus arbitration are
carried over unchanged, **so a gain set tuned on that variant is still valid
here**. What changed is the RC layer and the telemetry.

Target: STM32F103C8 Blue Pill, ST-Link over SWD. 100 Hz control loop.
Builds at 53.6% flash / 22.9% RAM.

> **The folder is named `bfrc`, not `balance_fusion_rc`, and that is load-bearing.**
> PlatformIO's build path here is 262 characters — two over Windows' 260-char
> `MAX_PATH`. `ar.exe` then fails to read `startup_stm32yyxx.S.o` with "No such
> file or directory" even though the file is present, because it uses the short
> path API. Renaming the folder to something longer will break the build again
> with an error that looks nothing like a path-length problem.

## Hardware

Unchanged from the parent variant — see
`HARDWARE_CONNECTIONS_STM32F103_BLUE_PILL.md`.

| Port | Pins | Role |
| --- | --- | --- |
| USART1 | **PA10 RX** | FlySky FS-iA10B iBUS in (PA9 TX unused) |
| USART2 | PA2 / PA3 | AX-12 half-duplex bus @ 1 Mbaud |
| USART3 | PB10 / PB11 | 3DR telemetry radio @ 115200 |

Wire the receiver's iBUS signal pin to **PA10** and share ground. Power the
receiver from the 5 V rail, not from PA10.

## Channel map

`readChannel()` is 0-indexed, so Ch*n* is `readChannel(n-1)`.

| Ch | Control | Type | Behaviour |
| --- | --- | --- | --- |
| 3 | Drive | pot, self-centring | Forward/back → velocity target. Idle → 0, robot just balances. |
| 4 | Steer | pot, self-centring | Left/right → differential PWM trim. Idle → 0. |
| 5 | Calibrate | button | Rising edge starts IMU calibration (only while disarmed) |
| 6 | Target = 0 | button | Rising edge zeroes target, trim and encoder origin |
| 7 | Motors | switch, 2-pos | HIGH = armed, LOW = disarmed |
| 8 | Target ± | pot, self-centring | **Rate control**: 10–100 counts/sec, idle = hold |
| 9 | Leg select | switch, 3-pos | LOW = mirrored, MID = left only, HIGH = right only |
| 10 | Crouch / stretch | pot, self-centring | **Rate control**: mm/sec, gated by Ch9, idle = hold |

Ch1/Ch2 are deliberately unused so a mode-2 right-hand stick can be left alone.

**This map matches no other variant.** `rc_balance_fusion_wireless` uses
Ch1/2/5/6/7/8 and `RC_mcu_IK_wireless` uses Ch3/4/5/7/8/10, both with different
meanings. A model memory from either will not work here — check the live bars in
the GUI's RC tab before trusting anything.

### Why Ch8 and Ch10 are rate controls, not direct setpoints

A pot that maps straight to a value has to be *held* at the value you want, and
its full travel has to cover the whole range — so fine adjustment near the
middle is impossible. As a rate control, centre means "hold what you have" and
deflection means "move at this speed", which is what you actually want for a
target you nudge and a crouch you ease into.

The axis→rate curve is **squared**, so the slow end of the pot is expanded:

| Pot position | Rate |
| --- | --- |
| centred (inside deadband) | 0 — nothing moves |
| just off centre | 10 counts/sec |
| half travel | ~35 counts/sec |
| full deflection | 100 counts/sec |

A linear map would give 55 counts/sec at half travel and feel jumpy at the fine
end. `RTN` / `RTX` change the two endpoints of that band.

Ch10 uses the same curve with `RC_CROUCH_RATE` (default 25 mm/s) as its maximum
and 20% of it as the slow end. It re-issues the move through `startPoseMove()`
only when the accumulated value has changed by more than 0.5 mm — calling that
function on every poll from a jittering pot would continuously restart the
interpolation and the legs would never arrive.

## Loop-body timer

The requirement was to measure the time the loop body *actually* takes — from
the top of the body to after its last task — and report it on the next tick.

```
loop() {
  ...100 Hz gate...
  loop_t0 = micros();        // window opens
  ...all tasks, including the telemetry write...
  body = micros() - loop_t0; // window closes: LAST statement in loop()
  loop_us_last = body;       // stashed, NOT printed
}                            // -> reported by the NEXT tick's telemetry
```

The one-tick deferral is the point, not a workaround. The telemetry write is
itself one of the loop's tasks and the most expensive one (a ~200-byte
`snprintf` plus a UART write). Printing the figure inside the same tick would
mean either printing before the body ends (measuring an incomplete tick) or
after (putting the print outside the window it reports on). Both make the number
stop describing the work it claims to. Deferring keeps the measured window
closed and complete; the cost is that the value is 10 ms old, which does not
matter for a budget figure.

> Anything you add to `loop()` must go **above** the closing line or it escapes
> the measurement.

Four numbers reach the GUI:

| Field | Meaning |
| --- | --- |
| `LUS` | Previous tick's body time, µs |
| `LPER` | Previous tick's start-to-start interval, µs (~10000 when healthy) |
| `LPK` | Worst body time since the last `LTR` reset |
| `LOVR` | Count of ticks whose body exceeded 10 000 µs |

`LPER` and `LUS` together separate a tick that ran **late** (long period, normal
body) from one that ran **slow** (long body). Body creeping toward 10 000 is the
overrun warning; `LOVR` climbing means it already happened.

The parent variant's tick profiler is still absent — USART1 is the iBUS input
and cannot also be the profiler console. Profile stage-by-stage on
`mcu_balance_fusion_wireless`; this timer gives the whole-body figure with the
receiver actually running.

## GUI — tab 4, "RC Calibration & Tuning"

```
python gui/main_gui.py
```

**Calibration workflow — one channel at a time**, which is what the tab is built
around:

1. Connect, and confirm `RC link: OK`.
2. Move one physical control. Watch which row's bar moves — that is the mapping,
   observed rather than assumed.
3. Sweep that control through its full travel. The tab records the extremes as
   they go past.
4. Press **Sweep** to commit those extremes into the Min/Max boxes.
5. Release the control to its resting position, press **Centre**.
6. Press **Apply** to send it to the robot (`RCC<ch>,<min>,<cen>,<max>`).
7. Repeat for the next channel, then **Save to file**.

The firmware validates every calibration line (`500 ≤ min < centre < max ≤ 2500`)
and NAKs a bad one, keeping the previous values — a calibration with
`min ≥ centre` would otherwise make the axis maths divide by a negative span and
produce inverted or enormous commands on a channel you believe you just fixed.

Each axis is scaled **independently per side** against its own endpoint, because
a real pot is rarely symmetric: with centre at 1480 and travel to 1000/2000, one
shared span would make full-left read 0.92 while full-right read 1.00.

**Read from robot** (`RCD`) dumps the live table so the GUI shows what is
actually flashed rather than a local file that may have drifted.

The tab also carries the rate/travel tuning boxes and the live loop-timing
readout — worth watching while moving the sticks, since RC polling is the thing
most likely to disturb the tick budget.

## Serial commands added

| Command | Effect |
| --- | --- |
| `RCC<ch>,<min>,<cen>,<max>` | Set one channel's calibration (ch is **1-based**) |
| `RCD` | Dump the calibration table as `RCCAL:` lines |
| `RCZ` | Reset all channels to 1000/1500/2000 |
| `RCR<f>` | Ch10 crouch rate, mm/sec at full pot |
| `RTN<f>` / `RTX<f>` | Ch8 rate band, slow / fast end |
| `RTL<f>` | Clamp on the accumulated Ch8 target |
| `LTR` | Reset the loop timer's peak and overrun counter |

Inherited from the parent: `RE0`/`RE1`, `RV<f>`, `RS<f>`, `RCM<f>`.

> New `R*` commands must be added **before** the bare-`R` (reset integral) case
> in the command parser, or they parse as a reset and are silently discarded.
> The parser has a comment at that spot; heed it.

## Safety

Unchanged from the parent and all still active:

- **Failsafe.** No valid iBUS frame for 500 ms → commands zeroed, motors
  disarmed, `SAFETY:RC_LINK_LOST`.
- **Arm interlock.** The Ch7 switch must be seen LOW once since boot (or since a
  link loss) before it can arm, so powering up with the switch already HIGH does
  not energise the motors.
- **Disarm always works.** Under `RE0` (GUI has control) the Ch7 switch loses
  its ability to arm but keeps its ability to stop.
- **Tilt cutoff.** Unchanged, and an RC arm clears a previous latch.

## Not yet done

The firmware compiles and the GUI runs, but **none of this has been tested on
the robot** — no receiver was connected during development. Before trusting it:

1. Verify every channel against the live bars (step 2 above) before arming.
2. Confirm Ch7 disarms, with the wheels off the ground.
3. Confirm the failsafe by switching the transmitter off, wheels still off the
   ground.
4. Check `LUS` sits well under 10 000 µs with the sticks moving.

`rc_target` (the Ch8 accumulator) is decoded, clamped, reported and zeroed by
Ch6, but **no control law consumes it yet** — the outer loop still takes its
setpoint from the Ch3 drive pot. Wiring it to a position hold is the obvious
next step and needs a decision about how it should interact with the drive pot.
