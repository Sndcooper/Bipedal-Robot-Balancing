# ax12_control — AX-12+ Leg Subsystem Bench Rig (wireless)

A dedicated servo rig: firmware + GUI for tuning and exercising the four AX-12+
leg servos on their own, with **no balancing anywhere in the loop**.

This is the 5th `tuner_legcontrol` variant, a peer of the other four:

```
tuner_legcontrol/
├── mcu_ik_engine_pretest_wireless/   single-loop balancer (the reference)
├── mcu_ik_engine_wireless/
├── mcu_pos_wireless/
├── RC_mcu_IK_wireless/
└── ax12_control/                     <-- this one: legs only
    ├── firmware/                     STM32 Bluepill, PlatformIO
    └── gui/                          Python + tkinter + matplotlib
```

---

## Why it exists separately

`mcu_ik_engine_pretest_wireless` is **not modified by this variant and must not
be.** Its loop timing, RX budget and downlink discipline are the fix for the
bottleneck seen during sequential running, and they are load-bearing. This rig
copies those techniques into its own firmware rather than adding servo knobs to a
balancer that is already airtime-constrained. The one place it deliberately
departs is how goal positions reach the bus — see *Coordinated motion* below.

Practically: you can hang the robot on the bench, run this, and fiddle with
compliance and torque all day without a single line of balance code executing.

---

## Hardware / wiring

Identical to the balancing variants — flash either firmware onto the same board.

| Function       | Port            | Pins        | Baud      |
| -------------- | --------------- | ----------- | --------- |
| 3DR telemetry  | `Serial3`       | PB10 / PB11 | 115200    |
| AX-12+ bus     | `Serial2`       | PA2 / PA3   | 1 000 000 |
| Drive motors   | *forced LOW*    | —           | —         |

The L298N pins (`PA0 PA1 PB12-PB15`) are set OUTPUT and driven LOW in `setup()`
and never touched again. A floating enable on a powered L298N can latch a wheel
on while your hands are in the leg linkage; this makes that impossible.

Servo IDs and the calibrated straight-down positions (do not "tidy" these — they
are measured, see the `leg-ik-and-servos` skill):

| ID | Leg  | Side  | Standing |
| -- | ---- | ----- | -------- |
| 6  | Leg1 | Left  | 818      |
| 14 | Leg1 | Right | 441      |
| 0  | Leg2 | Left  | 818      |
| 1  | Leg2 | Right | 441      |

---

## Wire protocol

> **This protocol belongs to `ax12_control` alone.** Every variant in
> `tuner_legcontrol` has its own, and they are not interchangeable. The trap
> worth naming: `TE1` means *auto-trim enable* in `mcu_ik_engine_pretest_wireless`.
> Here the torque kill is deliberately `TQ1`, never `TE`, so opening a GUI
> against the wrong firmware cannot silently do the wrong thing.

### Uplink (GUI → MCU), `\n`-terminated, case-insensitive

Every command is a **two-character prefix**. The pretest parser had to order its
multi-char cases before its single-char ones (or `TE1` parsed as a tilt of
`atof("E1")` = 0); with no single-char commands at all, that hazard cannot exist
here.

| Command          | Meaning                                       | Range      |
| ---------------- | --------------------------------------------- | ---------- |
| `TL<n>`          | Torque limit, all four (addr 34)              | 0–1023     |
| `CM<n>`          | Compliance margin, CW+CCW (addr 26/27)        | 0–254      |
| `CS<n>`          | Compliance slope, CW+CCW (addr 28/29)         | 0–254      |
| `MS<n>`          | Moving speed, all four (addr 32)              | 0–1023     |
| `MT<n>`          | Interpolated pose-move duration, ms           | 100–3000   |
| `TQ<0\|1>`       | Master torque kill / re-grip (addr 24)        | —          |
| `MD<0\|1>`       | Pose mode: 0 = CROUCH, 1 = IK                 | —          |
| `CR<f>`          | Crouch offset, mm (CROUCH mode only)          | 0–80       |
| `FT<leg> <x> <y>`| Foot target, one leg, mm (IK mode only)       | leg ∈ {1,2}|
| `FA <x1> <y1> <x2> <y2>` | Foot target, BOTH legs atomically     | mm         |
| `PS<id> <pos>`   | Raw goal position, bypasses IK                | 0–1023     |
| `HM`             | Home — back to the calibrated standing pose   | —          |
| `SR`             | Re-init the servo bus                         | —          |
| `RB`             | Request a `STATE:` line (resync)              | —          |
| `PING<token>`    | → `PONG<token>`                               | —          |

### Downlink (MCU → GUI)

```
SRV:<id>,POS:<n>,GOAL:<n>,SPD:<n>,LOAD:<f>,TEMP:<n>,VOLT:<f>
STATE:MODE:<0|1>,TQ:<0|1>,TL:<n>,CM:<n>,CS:<n>,MS:<n>,MT:<n>,CROUCH:<f>,
      FX1:<f>,FY1:<f>,FX2:<f>,FY2:<f>,IK1:<0|1>,IK2:<0|1>
ACK:TORQUE_ON | ACK:TORQUE_LIMP | ACK:SERVOS_RESET
PS:OK ... | PS:ERR ... | FT:ERR ...
BOOT:OK AX12_CONTROL
```

`STATE:` is emitted on every settings/pose command (it doubles as the ack the
GUI sliders sync against) and once per second as a keepalive.

**Airtime budget.** One `SRV:` line per 100 ms, round-robining one servo per
frame, so each servo refreshes at 2.5 Hz. That is ~650 B/s total — at or under
the balancing firmware's downlink. The 3DR/SiK link is half-duplex TDM: each end
only gets about half the airtime, so a fatter downlink leaves no window for the
ground unit to transmit and your slider commands sit in the radio buffer for
seconds.

---

## The GUI

```
python gui/main_gui.py        # needs: pyserial matplotlib numpy
```

Single window, everything visible at once.

**Left — inverse kinematics.** Both 5-bar legs drawn live. In IK mode you drag a
foot circle with the mouse and watch femur/tibia/knee re-solve and the AX-12
counts update under the plot. **Nothing reaches the robot until you click
"Send Pose to Robot".** Unreachable targets are *refused* rather than clamped —
the 5-bar's reach boundary is an annulus, not a box, so clamping would snap the
foot to a corner the linkage cannot occupy; instead the drag simply stalls at
the true edge of the workspace and the title reads `[UNREACHABLE]`.

**Pose modes are mutually exclusive**, and switching between them is a
guaranteed **zero-motion** event:

* `CROUCH → IK` — the crouch pose already *is* a foot target, so it is copied
  across exactly and the solve is bit-identical.
* `IK → CROUCH` — crouch has one DOF and cannot represent an arbitrary dragged
  pose, so it takes the **mean vertical displacement**: body height is preserved
  and only the horizontal offset is given up.

Both the firmware and the GUI implement this seeding, and they must stay in
sync — otherwise the plot would draw one pose while the servos hold another.

**Right — the knobs.**

| Control            | Register | What it actually does                                       |
| ------------------ | -------- | ----------------------------------------------------------- |
| Torque Limit       | 34       | Holding force. 1023 = full — also doubles jam force.        |
| Compliance Margin  | 26 / 27  | Deadband. Higher = ignores small errors, feels loose.        |
| Compliance Slope   | 28 / 29  | Proportional band. Higher = wider = slow ease-in, weak hold. |
| Moving Speed       | 32       | Slew cap toward goal. 0 = uncapped.                          |

All four are **RAM** registers (addr ≥ 24), so they can be hammered live with
zero EEPROM wear — which is what makes a live-tuning GUI safe here. Only
`Status Return Level` (16) and `Return Delay Time` (5) are EEPROM, and those are
written once at init.

The classic sluggish combo is **slope 32 / margin 4**: an 8× wider proportional
band plus a wide deadband, so the joint eases in slowly and holds weakly and
visibly creeps under the robot's own weight. That reads as "slow servos" and has
nothing to do with serial latency. The defaults here (**slope 4 / margin 1 /
torque 1023**) are the responsive baseline.

**Live health** shows `pos`, `goal`, `err`, `load`, temperature and voltage per
servo. `err = goal − pos` **is the compliance droop** — how far short of target
the joint settles under load, which is exactly what the margin/slope sliders are
chasing. Rows go amber ≥ 55 °C, red ≥ 65 °C, and **yellow below 10 V**: AX-12
torque scales with supply voltage, so a sagging pack presents identically to bad
compliance tuning. That flag is how you tell them apart instead of chasing slope
values that were never the problem.

**Servo Control** drives one joint at raw counts, bypassing IK entirely. Travel
is guarded to the standing pose ±200 counts (the firmware still hard-clamps
0–1023) so a stray drag cannot fold the linkage into itself. Each row has its own
explicit Send — nothing moves without a deliberate click. "Load Current Pose into
Sliders" copies the reported present positions in, so you nudge from where the
joint actually *is* rather than from a stale value that would make it jump.

**TORQUE kill** releases all four servos (addr 24 = 0) and stops the firmware
re-asserting goal position, so the legs go completely limp for hand-positioning.
It asks for confirmation first, because **the robot will collapse if unsupported.**

> On **re-grip**, the firmware seeds every `goalPos` from that servo's last-read
> *present* position. Without that, going limp, repositioning by hand and
> re-enabling would slam all four servos back to the stale pose at full torque —
> with your hands still in the linkage. After re-gripping, command a pose
> deliberately.

**Profiles** save and load the full knob set plus the pose to
`gui/profiles/ax12_<timestamp>.json`. Loading updates every widget locally first
and *then* pushes in one burst, so the pushes do not interleave with the sliders'
echo guards.

---

## Coordinated motion: SYNC_WRITE + interpolation

**The bug this fixes.** Goal positions used to be written inside
`pollLegServosTask()`, one servo per slot. A slot is 40 ms (write + read request
on one call, reply consumed on the next 20 ms call, next servo eligible 20 ms
later), so a full round-robin is **160 ms**. The two servos of *one leg* sit two
slots apart in `legServos[]` — ID 6 at index 0, ID 14 at index 2 — so they
received their new goals **80 ms apart**. At the AX-12's uncapped ~1208 counts/s
that is ~97 counts: **37% of a full crouch-to-stand travel completed by one joint
before the other was even told to move.** That is the "one servo goes first, then
the other one of the same leg" symptom, and it is pure write scheduling — not
compliance, not torque, not the radio.

Worth naming what was *not* the cause: the travel is symmetric. A pure vertical
foot move sends ID 6 `569→831` (+262) and ID 14 `697→435` (−262) — same
magnitude, so at equal speed they would finish together. The stagger was the
whole story.

**Two fixes, layered.**

1. **SYNC_WRITE** (instruction `0x83`, broadcast ID `0xFE`) hands all four goal
   positions out in a single 20-byte packet — ~200 µs at 1 Mbaud, 2% of the tick.
   Every joint latches on the same byte, so stagger is not reduced, it is
   *structurally impossible*. `pollLegServosTask()` no longer writes goals at
   all; it only re-asserts torque enable and torque limit.

2. **Trajectory interpolation.** Sync-write alone makes the joints *start*
   together, but handed only an endpoint each servo still races there at its own
   uncapped speed, so the foot bulges rather than tracking a line. So the
   firmware ramps the **foot target** from its current position to the commanded
   one at the full 100 Hz, re-solving the IK and sync-writing every tick. The
   ramp is **smoothstep** (`3f²−2f³`), which additionally starts and stops at
   zero velocity so the legs neither snap into motion nor slam to a halt. For a
   strictly linear ramp, use `f` directly in `updateMoveTask()` — one line.

Duration is set by **Move Time** (`MT<n>`, 100–3000 ms) and is *fixed time, not
fixed speed*: every move takes that long regardless of distance.

**Bus arbitration.** Exactly one transaction owns `Serial2` per tick. This
matters more than it looks: the health poll spans two ticks (request, then
reply), and a SYNC_WRITE landing in between would push its own half-duplex echo
into the 22-byte reply the poll is counting, desynchronising the read
permanently. So while a move is in flight it takes the bus outright and health
readback is suspended for the (sub-3 s) duration; when idle, the hold pose is
re-asserted by one sync-write at 10 Hz, gated on `POLL_IDLE`.

**Why `FA` exists.** At the 300 µs RX budget (~3.4 bytes/tick) an 18-byte `FT`
command takes ~6 ticks to arrive, so two back-to-back would start leg 2 moving
~60 ms after leg 1 *and* restart the move timer mid-flight. `FA` carries both
legs in one command so they start on the same tick, exactly as `CR` already does.
The GUI's Send Pose uses `FA`.

**Limp → re-grip.** Interpolation makes the torque kill more delicate, not less:
`cur_*` still holds the pre-limp foot, so the first sync-write of the next move
would slam the legs back to it. So re-grip now runs **forward kinematics** on the
reported present positions and re-seeds `cur_*` from where the legs actually are.

---

## Techniques inherited from `mcu_ik_engine_pretest_wireless`

Kept unchanged on purpose. All are protocol-independent. The one deliberate
departure is goal-position writing, covered in the section above.

* **256 B UART TX+RX buffers** (`platformio.ini`). The 64 B default broke both
  directions: TX, because a >64 B line could never pass the
  `availableForWrite() >= n` guard and was silently never sent; RX, because a
  radio burst overflows the FIFO (STM32 ORE) and drops mid-command bytes or the
  `\n`. For *this* firmware a truncated command is worse than a lost one — a
  mangled `TL900` landing as torque limit `0` makes every joint go slack with no
  error anywhere.
* **300 µs RX drain budget, every tick.** A byte at 115200 is 87 µs, so the old
  40 µs budget drained at most one byte per call.
* **Telemetry into one buffer, one guarded `write()`.** Per-field `print()`
  blocks once the radio backs up and stretches the loop's `dt`.
* **Non-blocking one-servo-per-20 ms poll** state machine.
* **Goal writes removed from the poll** — the one departure; see the
  coordinated-motion section above. The poll now re-asserts torque only.
* **Settings writes spread one servo per tick.** Pushing all registers to all
  four at once is ~28 packets (~2.5 ms of blocking `flush()`, since half-duplex
  must drain before the bus flips direction) — a quarter of the 10 ms tick. A
  dirty bitmask drains one servo per tick instead, ~630 µs.

---

## Notes

* Readback is `READ addr 36, len 8` → Pos, Speed, Load, **Volt**, Temp. The
  control table is contiguous, so voltage at addr 42 cannot be skipped on the way
  to temp at 43 — it is free. Reply is 14 B and the half-duplex bus echoes our
  own 8 B instruction, hence the `>= 22` wait.
* `PING`/`PONG` is implemented, but the balancer's `latency_test.py` also parses
  `DT`/`BD`/sequence fields that this firmware does not emit (they would cost
  airtime this rig has no use for). Round-trip latency testing works; the loop
  and drop statistics do not.
* `gui/twin_kinematics.py` is byte-identical to the pretest copy above the
  `REVERSE MAPPING` banner, and must stay that way — the firmware carries the
  same math in C, so any divergence silently makes the GUI draw a pose the robot
  is not holding. The block below that banner is additive: it maps a reported
  present position back into a foot coordinate for the "Actual foot" readout.
