---
name: leg-ik-and-servos
description: The AX-12+ leg subsystem — half-duplex 1 Mbaud bus, control-table writes/reads, the 2D 5-bar inverse kinematics solver, servo-angle→AX-12 position mapping (818/441 calibration), leg-2 mirroring, and compliance. Read before touching leg geometry, IK, or servo register code.
---

# Legs, IK & AX-12 Servos (`leg-ik-and-servos`)

The legs are a **separate subsystem** from the wheels/balance loop. Four AX-12+
servos on a shared half-duplex bus (`Serial2`, 1,000,000 baud). Geometry constants
are shared between the Python twin (`gui/twin_kinematics.py`) and the C++ firmware
(`solve_ik`, `map_angle_to_ax12` in `main.cpp`) — **keep them in sync**.

Authoritative refs: `Robot_Specification.md` (geometry, calibration),
`AX12_Control_Table_Mapping.md` (register/buffer offsets).

## The half-duplex bus (the "hack")

- Single data line shared TX/RX. A 10 kΩ resistor bridges `PA2`(TX2)↔`PA3`(RX2);
  the servo DATA line connects to `PA3`. Common ground with the servo supply.
- **Every write echoes back** on RX. The firmware discards echoes explicitly:
  after write bursts it does `while (Serial2.available()) Serial2.read();`, and a
  READ transaction expects `8-byte echo + 10-byte reply = 18 bytes`.
- `Serial2.flush()` after each write is mandatory — the bus cannot switch
  direction until the outgoing bytes have physically drained. (This blocks, so it
  runs only in the 50 Hz servo slice / setup, never in a tight inner path.)

## Packet formats (Dynamixel 1.0)

```cpp
// WRITE byte:  FF FF ID 04 03 ADDR VAL   CHK
// WRITE word:  FF FF ID 05 03 ADDR LO HI CHK      CHK = ~(sum of ID..last)&0xFF
// SYNC_WRITE positions (all 4 at once, addr 30, 2 bytes each):
//   FF FF FE 10 83 1E 02  06 lo hi  0E lo hi  00 lo hi  01 lo hi  CHK
// READ:        FF FF ID 04 02 ADDR LEN CHK
```
Health poll reads **addr 40 len 4** → Present Load (2B) + Voltage (1B) + Temp (1B).
`loadPct = (loadRaw & 0x3FF)/1023*100`. See `AX12_Control_Table_Mapping.md` for the
full table and the `buf[5+addr]` offset rule.

## Servo init & polling

- `initAX12Legs()` (setup + on `S` command): sets Status Return Level, Return Delay
  0, Torque Enable, Torque Limit, CW/CCW compliance margin+slope, and goal position.
  EEPROM regs (16, 5) are written **once** here — not in the poll loop, to avoid
  EEPROM wear.
- `pollLegServosTask()` runs one servo per 20 ms (all four every 80 ms) as a
  non-blocking `IDLE→WAITING` state machine with a 20 µs-ish timeout so a dead
  servo cannot stall the loop. It re-asserts torque/limit/goal (RAM only) each pass.

## The IK solver (2-circle intersection, 5-bar leg)

`solve_ik(tx, ty, leg_offset_x)`:
1. Two fixed servo mounts at `(-30,0)` and `(+30,0)` (shifted by `leg_offset_x`).
2. Each drives a **femur** (55 mm crank); the **tibia** (100 mm rod) reaches the
   foot target `(tx,ty)`.
3. Knee = intersection of circle(mount, femur) and circle(foot, tibia).
4. **Knee selection:** left knee takes the smaller-X intersection, right knee the
   larger-X → outward-bending geometry. Angle = `atan2(knee - mount)`.

`updateIK()` applies `ik_lean` as a rotation of both foot targets, solves leg 1 at
offset 0 and leg 2 at offset `ik_dist` (180 mm), then SYNC_WRITEs all four.

## Angle → AX-12 position mapping (calibrated — do not "simplify")

```cpp
base_angle = is_leg2 ? +90 : -90;              // straight-down reference
diff_deg   = wrap180(ik_angle - base_angle);
base_pos   = is_left ? 818 : 441;              // CALIBRATED standing pose
ax_pos     = clamp(base_pos + diff_deg*3.413, 0, 1023);   // ~3.413 units/deg
```
818 (left) / 441 (right) are **physical calibration constants**, not a generic
150° center. The C++ and Python versions must produce identical positions.

## Leg-2 mirroring

Leg 2 is a physically mirrored mount (`LEG2_INVERTED_MOUNT = true`). The solver
**swaps and negates** before mapping:
```
ik_L2 = -Angle_R;   ik_R2 = -Angle_L;
```
If you add a third leg pose or change mounting, this is the line to revisit.

## Compliance (Phase C hook — mostly unused today)

AX-12 compliance margin/slope (`CMP id margin slope`, regs 26–29) is a **coarse
mechanical-spring** setting, not a fast position command. The roadmap
(`go-throuugh-all-the-golden-sketch.md`, Phase C) intends a low-rate `RIGID`/
`COMPLIANT` state machine that loosens slope on disturbance — not yet implemented.
Today compliance is just the static value set at init. Treat legs as the slow,
secondary shock-absorbing layer; wheels remain the primary balance actuator.

## PC-side IK variant

`pc_ik_engine` computes IK in Python (`twin_kinematics.py` → `leg1_positions` /
`leg2_positions`) and streams raw `POS id pos` writes. Everything above about the
mapping/mirroring applies identically — just executed on the host instead of the MCU.
