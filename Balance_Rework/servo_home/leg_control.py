#!/usr/bin/env python3
"""
leg_control.py — interactive control of the 4 AX-12+ leg servos.

Use this to physically dial in a STANDING pose, then read off the exact servo
positions to hardcode (into initAX12Legs() in the balance firmware).

Talks to the servo_home firmware (Balance_Rework/servo_home) over its TEXT protocol:
    id,pos    move one servo to pos (0..1023)
    H         re-home all to straight-down
    F<id>     torque OFF (free a servo so you can hand-pose it)
    L<id>     torque ON  (lock)

Straight-down reference (calibrated standing pose):  LEFT = 818, RIGHT = 441.

  Servo map:   6 = Leg1-L    14 = Leg1-R
               0 = Leg2-L     1 = Leg2-R

Usage:
    python leg_control.py --port COM3
    python leg_control.py                 (prompts for the port)

Interactive commands (type 'help' to reprint):
    <id> <pos>     set one servo absolute      e.g.  6 810
    <id> +<n> / -<n>  nudge one servo          e.g.  6 +15   or   14 -10
    l +<n> / -<n>  nudge BOTH left servos (6,0)
    r +<n> / -<n>  nudge BOTH right servos (14,1)
    a <id> <deg>   set one servo by IK angle (uses YOUR tuned twin mapping)
    foot1 <x> <y>  pose Leg 1 by foot X/Y (runs YOUR tuned solve_ik -> ids 6,14)
    foot2 <x> <y>  pose Leg 2 by foot X/Y (YOUR tuned IK + leg2 inversion -> ids 0,1)
    dist <n>       set leg distance used by foot2 (default = twin's LEG_DISTANCE)
    home           re-home all to straight-down
    free / lock    torque OFF / ON for all 4 (free = pose by hand)
    show           print current positions
    dump           print a ready-to-paste hardcode block
    q / quit       exit

All angle/foot math is imported from twin_kinematics.py, which is your digital twin's
tuned logic extracted verbatim (one source of truth).
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("pyserial is required:  pip install pyserial")
    sys.exit(1)

# YOUR tuned kinematics, extracted from the digital twin (see twin_kinematics.py).
import twin_kinematics as tk

# ── Servo layout ──
LEFT_IDS = [6, 0]      # Leg1-L, Leg2-L
RIGHT_IDS = [14, 1]    # Leg1-R, Leg2-R
ALL_IDS = [6, 14, 0, 1]
SIDE = {6: "L", 0: "L", 14: "R", 1: "R"}
LABEL = {6: "Leg1-L", 14: "Leg1-R", 0: "Leg2-L", 1: "Leg2-R"}
IS_LEG2 = {6: False, 14: False, 0: True, 1: True}

POS_LEFT_DOWN = 818
POS_RIGHT_DOWN = 441


def straight_down(sid):
    return POS_LEFT_DOWN if SIDE[sid] == "L" else POS_RIGHT_DOWN


class LegController:
    def __init__(self, port, baud=115200):
        print(f"[serial] opening {port} @ {baud} ...")
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(2.5)  # STM32 reboots on connect; servo_home re-homes on boot
        self.ser.reset_input_buffer()
        # We track commanded positions; the firmware re-homed on the reboot above.
        self.pos = {sid: straight_down(sid) for sid in ALL_IDS}
        self.dist = tk.LEG_DISTANCE  # leg spacing used by foot2's IK offset
        self._drain(print_it=True)

    # ---- serial helpers ----
    def _send(self, line):
        self.ser.write(f"{line}\n".encode())
        self.ser.flush()
        time.sleep(0.04)
        self._drain()

    def _drain(self, print_it=False):
        t0 = time.time()
        while time.time() - t0 < 0.15:
            line = self.ser.readline().decode(errors="ignore").strip()
            if line and print_it:
                print(f"   fw> {line}")

    # ---- actions ----
    def move(self, sid, pos):
        if sid not in ALL_IDS:
            print(f"   ! unknown servo id {sid} (valid: {ALL_IDS})")
            return
        pos = int(max(0, min(1023, pos)))
        self.pos[sid] = pos
        self._send(f"{sid},{pos}")
        print(f"   {LABEL[sid]} (id {sid}) -> {pos}")

    def nudge(self, sid, delta):
        self.move(sid, self.pos[sid] + delta)

    def nudge_side(self, ids, delta):
        for sid in ids:
            self.nudge(sid, delta)

    def foot(self, leg, x, y):
        """Pose one leg by foot (x,y) using YOUR tuned twin kinematics."""
        if leg == 1:
            positions = tk.leg1_positions(x, y)
        else:
            positions = tk.leg2_positions(x, y, self.dist)
        if positions is None:
            print(f"   ! Leg{leg} foot ({x}, {y}) is OUT OF REACH (IK invalid)")
            return
        print(f"   Leg{leg} foot ({x:.1f}, {y:.1f}) -> {positions}")
        for sid, pos in positions.items():
            self.move(sid, pos)

    def home(self):
        self._send("H")
        for sid in ALL_IDS:
            self.pos[sid] = straight_down(sid)
        print("   homed all to straight-down (L=818, R=441)")

    def set_torque_all(self, on):
        for sid in ALL_IDS:
            self._send(("L" if on else "F") + str(sid))
        print("   all servos " + ("LOCKED (torque ON)" if on else "FREED (torque OFF)"))

    def show(self):
        print("   current positions:")
        for sid in ALL_IDS:
            print(f"     {LABEL[sid]:8s} (id {sid:2d})  {self.pos[sid]:4d}")

    def dump(self):
        p = self.pos
        print("\n   ---- paste into initAX12Legs() in Balance_Rework/firmware/src/main.cpp ----")
        print(f"   uint8_t  left_servos[]    = {{6, 0}};")
        print(f"   uint8_t  right_servos[]   = {{14, 1}};")
        print(f"   uint16_t left_positions[] = {{{p[6]}, {p[0]}}};   // id6, id0")
        print(f"   uint16_t right_positions[]= {{{p[14]}, {p[1]}}};   // id14, id1")
        print("   ----------------------------------------------------------------------")
        print("   (or tell me these 4 numbers: "
              f"id6={p[6]}, id0={p[0]}, id14={p[14]}, id1={p[1]})\n")

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


HELP = __doc__.split("Interactive commands")[1]


def parse_and_run(ctrl, raw):
    parts = raw.split()
    if not parts:
        return True
    cmd = parts[0].lower()

    if cmd in ("q", "quit", "exit"):
        return False
    if cmd == "help":
        print("Interactive commands" + HELP)
        return True
    if cmd == "home":
        ctrl.home(); return True
    if cmd == "free":
        ctrl.set_torque_all(False); return True
    if cmd == "lock":
        ctrl.set_torque_all(True); return True
    if cmd == "show":
        ctrl.show(); return True
    if cmd == "dump":
        ctrl.dump(); return True

    # leg distance:  dist <n>
    if cmd == "dist" and len(parts) == 2:
        try:
            ctrl.dist = float(parts[1])
            print(f"   leg distance = {ctrl.dist} (used by foot2)")
        except ValueError:
            print("   ! usage: dist <n>")
        return True

    # foot mode:  foot1 <x> <y>  /  foot2 <x> <y>
    if cmd in ("foot1", "foot2") and len(parts) == 3:
        try:
            x = float(parts[1]); y = float(parts[2])
        except ValueError:
            print("   ! usage: foot1 <x> <y>"); return True
        ctrl.foot(1 if cmd == "foot1" else 2, x, y)
        return True

    # angle mode:  a <id> <deg>
    if cmd == "a" and len(parts) == 3:
        try:
            sid = int(parts[1]); deg = float(parts[2])
        except ValueError:
            print("   ! usage: a <id> <deg>"); return True
        if sid not in ALL_IDS:
            print(f"   ! unknown id {sid}"); return True
        pos = tk.map_angle_to_ax12(deg, is_left=(SIDE[sid] == "L"), is_leg2=IS_LEG2[sid])
        print(f"   angle {deg:+.1f} deg -> pos {pos}")
        ctrl.move(sid, pos); return True

    # side nudge:  l +10 / r -10
    if cmd in ("l", "r") and len(parts) == 2:
        try:
            delta = int(parts[1])
        except ValueError:
            print("   ! usage: l +<n> | r -<n>"); return True
        ctrl.nudge_side(LEFT_IDS if cmd == "l" else RIGHT_IDS, delta)
        return True

    # per-servo:  <id> <pos>  or  <id> +<n>/-<n>
    if len(parts) == 2:
        try:
            sid = int(parts[0])
        except ValueError:
            print("   ! unrecognized. type 'help'"); return True
        val = parts[1]
        if val[0] in "+-":
            try:
                ctrl.nudge(sid, int(val))
            except ValueError:
                print("   ! bad nudge amount")
        else:
            try:
                ctrl.move(sid, int(val))
            except ValueError:
                print("   ! bad position")
        return True

    print("   ! unrecognized. type 'help'")
    return True


def main():
    ap = argparse.ArgumentParser(description="Interactive AX-12 leg controller")
    ap.add_argument("--port", default=None, help="serial port, e.g. COM3")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    port = args.port or input("Serial port (e.g. COM3): ").strip()
    if not port:
        print("No port given."); sys.exit(1)

    try:
        ctrl = LegController(port, args.baud)
    except Exception as e:
        print(f"Could not open {port}: {e}")
        sys.exit(1)

    print("\nReady. Servos are at straight-down. Type 'help' for commands, 'q' to quit.")
    print("Tip: 'free' to pose the legs by hand, then 'lock', then 'dump' to read them off.\n")
    ctrl.show()
    print()

    try:
        while True:
            try:
                raw = input("legs> ").strip()
            except EOFError:
                break
            if not parse_and_run(ctrl, raw):
                break
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        ctrl.close()
        print("serial closed. (servos keep holding their last commanded pose)")


if __name__ == "__main__":
    main()
