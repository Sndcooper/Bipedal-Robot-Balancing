---
name: serial-protocol
description: The firmware↔GUI wire contract for the tuner_legcontrol variants — and the critical fact that the protocol DIFFERS per variant (comma/colon/newline vs space/pipe). Read before adding/changing any serial command, telemetry field, or serial_link.py parser.
---

# Serial Protocol & Firmware↔GUI Contract (`serial-protocol`)

Every tuner_legcontrol variant is a **matched pair**: the firmware's
`parseCommand()` + telemetry TX in `firmware/src/main.cpp` must agree byte-for-byte
with the GUI's `gui/serial_link.py` (`_process_line`, `_parse_telemetry`,
`_parse_fw_update`, and the `_send(...)` methods). **Change one side, change the
other, in the same edit.** A mismatch fails silently — the GUI just shows zeros.

## ⚠️ The protocols are NOT the same across folders

Two families exist. Confirm which one you are in by reading that folder's files —
do not assume.

| | `mcu_ik_engine_wireless` (and siblings) | `RC_mcu_IK_wireless` (flagship) |
|---|---|---|
| Frame terminator | `\n` (`Serial3.println`) | `|` pipe (also accepts `\n`) |
| Telemetry style | `key:value`, comma-separated | `key<value>`, space-separated |
| Telemetry example | `S:0,DT:10000,P:1.23,PO:0.5,I:0.01,EL:..,ER:..,V:..,TB:..,ST:..,A:..,T:..,M:..,L:..` | `S0 DT10000 P1.23 O0.5 I0.01 V.. TB.. EP.. EV.. VR.. ST.. A.. T.. M.. L..\|` |
| Command style | comma: `IK1,fx,fy` `POS,id,pos` `CMP,id,m,s` | space: `IK1 fx fy` `POS id pos` `CMP id m s` |
| FW ack | `Updated -> P:.. I:.. D:.. ...` | `Updated P.. I.. D.. ...\|` |
| Cal reply | `CAL:DONE,OFFSET:..` | `CAL DONE OFFSET..\|` |

GUI-side terminator handling reflects this: the wireless-engine `serial_link.py`
splits on `b"\n"`; the flagship splits on `for terminator in (b"|", b"\n")`.

## Command surface (single-letter and prefix)

Longer prefixes are matched **before** single letters so `SPN`/`PP`/`VP`/`TRQE`
never collide with `S`/`P`/`T`. Order in `parseCommand()` matters — keep prefixes first.

| Command | Meaning | Ack? |
|---|---|---|
| `P<f>` `I<f>` `D<f>` | Balance PID gains | yes |
| `VP<f>` `VI<f>` | Velocity loop `Kp_vel`/`Ki_vel` | yes |
| `VA<f>` | Velocity EMA coefficient (0..0.99) | yes |
| `PP<f>` | Position-hold `Kp_pos` | yes |
| `A<f>` `T<f>` | Complementary `alpha` / max safe tilt | yes |
| `S<f>` | Operator pitch trim (`gui_base_angle`) | yes |
| `O<f>` | Manual pitch offset | yes* |
| `FWD<-1..1>` `SPN<-1..1>` | Drive / spin demand | no (high rate) |
| `IK1 x y` `IK2 x y` `IKD d` `IKL lean` | Leg IK foot targets / dist / lean | no |
| `POS id pos` `TRQ id v` `CMP id m s` `TRQE 0\|1` | Direct AX-12 leg control | no |
| `S` (bare) | Re-init legs to standing pose | `ACK:SERVOS_RESET` |
| `C` | Calibrate IMU zero | `CAL:...` |
| `R` | Reset integrators | `ACK:INT_RESET` |
| `M` | Toggle motors (arm/disarm) | `Motors ENABLED/DISABLED` |

> \* **Known gap:** the `O` (offset) handler exists in `mcu_ik_engine_wireless`
> (added, "newly implemented") but is **absent from `RC_mcu_IK_wireless`'s
> `parseCommand()`** — so the flagship GUI's "Set Offset" is a silent no-op. If
> you touch offset behaviour, reconcile both. (Flag to user before "fixing" the
> flagship — see the findings the maintainer confirmed.)

## Telemetry field glossary

`S`=sequence, `DT`=loop period µs, `P`=pitch°, `PO`/`O`=PID output, `I`=integral,
`EL`/`ER`=raw encoder counts, `V`=filtered velocity (c/s), `TB`=tilt bias°,
`EP`/`EV`=position/velocity error, `VR`=vel EMA alpha, `ST`=cascade state
(0=DRIVING 1=RAMPDOWN 2=HOLDING), `A`=alpha, `T`=maxSafeTilt, `M`=motors,
`L`=safety latched.

## Adding a field or command — checklist

1. **Firmware TX:** add the token to the telemetry `print` block. Keep the frame
   under the UART TX buffer (64 B default; the flagship raises it with
   `-DSERIAL_TX_BUFFER_SIZE=256` and guards with `availableForWrite()`). Do not
   grow a wireless frame past what fits without that guard.
2. **Firmware RX:** add a branch in `parseCommand()`; put multi-char prefixes
   **above** single-letter cases. Keep it zero-heap (no `String`), non-blocking.
3. **Firmware ack:** if it is a tuning value, extend the `Updated ...` ack string
   (and its buffer size) so the GUI can confirm the value.
4. **GUI parse:** mirror the token key in `_parse_telemetry` / `_parse_fw_update`
   in *that folder's* `serial_link.py`.
5. **GUI send:** add a `set_x()` / `send_x()` that emits in the folder's exact
   delimiter style (comma vs space) and terminator.
6. Verify the round-trip on hardware or with a loopback before committing.

## Transport gotchas

- The 3DR radio delivers bytes in chunks unaligned to line boundaries. The GUI
  **must** accumulate bytes and only dispatch on the terminator — never
  `readline()` with a short timeout (returns partial `PITCH:,PID_OUT:,` lines).
  `serial_link.py` uses `in_waiting` + a byte accumulator; keep it that way.
- Only one process can own a COM port. Close the serial monitor before launching
  the GUI or flashing.
- Wired variants run at **500000** baud on Serial1; wireless at **115200** on
  Serial3. Match `SerialLink(baud=...)` to the variant.
