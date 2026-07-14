"""
digital_twin_legs.py — Balance_Rework copy of your 2-leg digital twin.

Same visual + kinematics as your original twin, but:
  * uses twin_kinematics.py (your tuned logic, one source of truth),
  * SHOWS ALL angles/values (on-screen panel + console + "Print Values" button), and
  * LIVE SEND that actually drives the real servos through the servo_home firmware's
    text protocol on COM3 ("id,pos" lines) — NOT the raw Dynamixel protocol the
    original twin used, so no firmware change is needed.

Your original (Python_Controller_Digital_Twin/.../digital twin 2_legs.py) is untouched.

Run:  python digital_twin_legs.py --port COM3
"""

import argparse
import sys
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons

import twin_kinematics as tk

try:
    import serial
except ImportError:
    serial = None

# ── CLI ──
_ap = argparse.ArgumentParser()
_ap.add_argument("--port", default="COM3", help="serial port of the servo_home board")
_ap.add_argument("--baud", type=int, default=115200)
_args, _ = _ap.parse_known_args()

# ── Tuned constants/IDs from your kinematics module ──
SERVO_L, SERVO_R = tk.SERVO_L, tk.SERVO_R
ID6, ID14 = tk.LEG1_SERVO_L_ID, tk.LEG1_SERVO_R_ID   # Leg1 L, R
ID0, ID1 = tk.LEG2_SERVO_L_ID, tk.LEG2_SERVO_R_ID    # Leg2 L, R
ALL_IDS = [ID6, ID14, ID0, ID1]


# ══════════════════════════════════════════════════════════════════
#  SERIAL LINK to servo_home (TEXT protocol: "id,pos", "H", "F<id>", "L<id>")
# ══════════════════════════════════════════════════════════════════
class LegLink:
    def __init__(self):
        self.ser = None
        self.last = {}          # id -> last sent pos (send only on change)

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def connect(self, port, baud):
        if serial is None:
            print("[link] pyserial not installed: pip install pyserial")
            return False
        try:
            self.close()
            print(f"[link] opening {port} @ {baud} (board reboots + re-homes) ...")
            self.ser = serial.Serial(port, baud, timeout=0.2)
            time.sleep(2.5)     # STM32 reboots on connect; servo_home re-homes first
            self.ser.reset_input_buffer()
            self.last = {}
            print("[link] connected")
            return True
        except Exception as e:
            print(f"[link] could NOT open {port}: {e}")
            self.ser = None
            return False

    def send_positions(self, positions):
        """positions: {id: pos}. Sends only changed servos."""
        if not self.connected:
            return
        try:
            sent = {}
            for sid, pos in positions.items():
                if self.last.get(sid) != pos:
                    self.ser.write(f"{sid},{pos}\n".encode())
                    self.last[sid] = pos
                    sent[sid] = pos
            if sent:
                self.ser.flush()
                print("[send] " + "  ".join(f"id{k}={v}" for k, v in sent.items()))
        except Exception as e:
            print(f"[link] write failed: {e}")

    def cmd(self, text):
        if not self.connected:
            print("[link] not connected")
            return
        try:
            self.ser.write((text + "\n").encode())
            self.ser.flush()
        except Exception as e:
            print(f"[link] cmd failed: {e}")

    def torque_all(self, on):
        for sid in ALL_IDS:
            self.cmd(("L" if on else "F") + str(sid))
        self.last = {}  # positions unknown after a torque change

    def close(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None


link = LegLink()


# ══════════════════════════════════════════════════════════════════
#  KINEMATICS PIPELINE (your twin's logic, via twin_kinematics)
# ══════════════════════════════════════════════════════════════════
def compute(x1, y1, x2, y2, dist):
    sol1 = tk.solve_ik(x1, y1, 0.0)
    sol2 = tk.solve_ik(x2 + dist, y2, dist)
    out = {"sol1": sol1, "sol2": sol2, "dist": dist,
           "foot1": (x1, y1), "foot2": (x2, y2)}
    if sol1:
        out["p6"] = tk.map_angle_to_ax12(sol1["Angle_L"], is_left=True, is_leg2=False)
        out["p14"] = tk.map_angle_to_ax12(sol1["Angle_R"], is_left=False, is_leg2=False)
    if sol2:
        if tk.LEG2_INVERTED_MOUNT:
            ikL2, ikR2 = -sol2["Angle_R"], -sol2["Angle_L"]
        else:
            ikL2, ikR2 = sol2["Angle_L"], sol2["Angle_R"]
        out["ikL2"], out["ikR2"] = ikL2, ikR2
        out["p0"] = tk.map_angle_to_ax12(ikL2, is_left=True, is_leg2=True)
        out["p1"] = tk.map_angle_to_ax12(ikR2, is_left=False, is_leg2=True)
    return out


def positions_from(d):
    """Extract {id: pos} for whatever legs have valid IK."""
    p = {}
    if d["sol1"]:
        p[ID6], p[ID14] = d["p6"], d["p14"]
    if d["sol2"]:
        p[ID0], p[ID1] = d["p0"], d["p1"]
    return p


def format_values(d):
    lines = []
    s1 = d["sol1"]
    lines.append(f"LEG 1  (ids {ID6}=L, {ID14}=R)")
    lines.append(f"  foot (x,y): ({d['foot1'][0]:+.1f}, {d['foot1'][1]:+.1f})")
    if s1:
        lines.append(f"  Angle_L: {s1['Angle_L']:+7.2f} deg   Angle_R: {s1['Angle_R']:+7.2f} deg")
        lines.append(f"  Knee_L: ({s1['Knee_L'][0]:+.1f},{s1['Knee_L'][1]:+.1f})"
                     f"   Knee_R: ({s1['Knee_R'][0]:+.1f},{s1['Knee_R'][1]:+.1f})")
        lines.append(f"  AX-12:  id{ID6} = {d['p6']:4d}    id{ID14} = {d['p14']:4d}")
    else:
        lines.append("  IK INVALID (out of reach)")
    lines.append("")
    s2 = d["sol2"]
    inv = "  [inverted mount]" if tk.LEG2_INVERTED_MOUNT else ""
    lines.append(f"LEG 2  (ids {ID0}=L, {ID1}=R){inv}   dist={d['dist']:.0f}")
    lines.append(f"  foot (x,y): ({d['foot2'][0]:+.1f}, {d['foot2'][1]:+.1f})")
    if s2:
        lines.append(f"  Angle_L: {s2['Angle_L']:+7.2f} deg   Angle_R: {s2['Angle_R']:+7.2f} deg")
        lines.append(f"  sent ik_L: {d['ikL2']:+7.2f} deg   ik_R: {d['ikR2']:+7.2f} deg")
        lines.append(f"  AX-12:  id{ID0} = {d['p0']:4d}    id{ID1} = {d['p1']:4d}")
    else:
        lines.append("  IK INVALID (out of reach)")
    if s1 and s2:
        lines.append("")
        lines.append(f"  hardcode: id6={d['p6']} id0={d['p0']} id14={d['p14']} id1={d['p1']}")
    return "\n".join(lines)


# ══════════════════════════════════════════════════════════════════
#  GUI
# ══════════════════════════════════════════════════════════════════
BG, FG = "#1a1a2e", "#e0e0e0"
plt.rcParams.update({
    "figure.facecolor": BG, "axes.facecolor": "#111122",
    "axes.edgecolor": "#333355", "axes.labelcolor": FG,
    "xtick.color": FG, "ytick.color": FG, "text.color": FG,
})

fig = plt.figure(figsize=(15, 10))
fig.canvas.manager.set_window_title("Balance_Rework — 2-Leg Twin (live send)")

ax = fig.add_axes([0.05, 0.40, 0.52, 0.55])
ax.set_aspect("equal")
ax.set_xlim(-150, 300)
ax.set_ylim(-200, 50)
ax.grid(True, linestyle=":", alpha=0.3, color="#333355")


def make_leg_artists(color_femur, color_tibia):
    lf, = ax.plot([], [], "o-", color=color_femur, lw=6, solid_capstyle="round", ms=7, zorder=5)
    rf, = ax.plot([], [], "o-", color=color_femur, lw=6, solid_capstyle="round", ms=7, zorder=5)
    lt, = ax.plot([], [], "o-", color=color_tibia, lw=5, solid_capstyle="round", ms=6, zorder=4)
    rt, = ax.plot([], [], "o-", color=color_tibia, lw=5, solid_capstyle="round", ms=6, zorder=4)
    fd, = ax.plot([], [], "o", color="#51cf66", ms=14, zorder=6)
    return lf, rf, lt, rt, fd


leg1_arts = make_leg_artists("#ff6b6b", "#e0e0e0")
leg2_arts = make_leg_artists("#da77f2", "#a5d8ff")

pnl = fig.add_axes([0.60, 0.42, 0.38, 0.54])
pnl.set_xlim(0, 1); pnl.set_ylim(0, 1); pnl.axis("off")
txt_vals = pnl.text(0.02, 0.98, "", fontsize=12, color=FG, family="monospace",
                    va="top", weight="bold")
txt_status = pnl.text(0.02, 0.02, "", fontsize=11, color="#ffd43b", family="monospace")

skw = dict(color="#7950f2", initcolor="none")
s_fx1 = Slider(fig.add_axes([0.08, 0.30, 0.45, 0.02]), "Leg1 X", -100, 100, valinit=0, valstep=0.5, **skw)
s_fy1 = Slider(fig.add_axes([0.08, 0.26, 0.45, 0.02]), "Leg1 Y", -160, -20, valinit=-130, valstep=0.5, **skw)
s_fx2 = Slider(fig.add_axes([0.08, 0.20, 0.45, 0.02]), "Leg2 X", -100, 100, valinit=0, valstep=0.5, **skw)
s_fy2 = Slider(fig.add_axes([0.08, 0.16, 0.45, 0.02]), "Leg2 Y", -160, -20, valinit=-130, valstep=0.5, **skw)
s_dist = Slider(fig.add_axes([0.08, 0.10, 0.45, 0.02]), "Leg Dist", 100, 250, valinit=int(tk.LEG_DISTANCE), valstep=1, **skw)

chk_ax = fig.add_axes([0.62, 0.30, 0.18, 0.09], facecolor=BG)
chk = CheckButtons(chk_ax, ["Mirror Leg2=Leg1", "Live Send"], [True, True])
for lb in chk.labels:
    lb.set_color(FG)


def _mk_btn(pos, label, hover="#7950f2"):
    b = Button(fig.add_axes(pos), label, color="#2a2a4a", hovercolor=hover)
    b.label.set_color(FG)
    return b


btn_conn = _mk_btn([0.62, 0.23, 0.16, 0.05], "Connect", "#51cf66")
btn_print = _mk_btn([0.80, 0.23, 0.16, 0.05], "Print Values")
btn_free = _mk_btn([0.62, 0.16, 0.16, 0.05], "Torque OFF", "#ff6b6b")
btn_lock = _mk_btn([0.80, 0.16, 0.16, 0.05], "Torque ON", "#51cf66")

state = {"mirror": True, "live": True, "last_send": 0.0}
SEND_INTERVAL = 0.05  # s — throttle live sends to ~20 Hz
_updating = False


def refresh_status():
    conn = "CONNECTED " + _args.port if link.connected else "not connected"
    live = "LIVE" if state["live"] else "live off"
    txt_status.set_text(f"[{conn}]   [{live}]")


def update(_=None):
    global _updating
    if _updating:
        return
    _updating = True

    if state["mirror"]:
        s_fx2.set_val(-s_fx1.val)
        s_fy2.set_val(s_fy1.val)

    dist = s_dist.val
    d = compute(s_fx1.val, s_fy1.val, s_fx2.val, s_fy2.val, dist)

    s1 = d["sol1"]
    if s1:
        leg1_arts[0].set_data([SERVO_L[0], s1["Knee_L"][0]], [SERVO_L[1], s1["Knee_L"][1]])
        leg1_arts[1].set_data([SERVO_R[0], s1["Knee_R"][0]], [SERVO_R[1], s1["Knee_R"][1]])
        leg1_arts[2].set_data([s1["Knee_L"][0], s_fx1.val], [s1["Knee_L"][1], s_fy1.val])
        leg1_arts[3].set_data([s1["Knee_R"][0], s_fx1.val], [s1["Knee_R"][1], s_fy1.val])
        leg1_arts[4].set_data([s_fx1.val], [s_fy1.val])
    else:
        for a in leg1_arts:
            a.set_data([], [])

    s2 = d["sol2"]
    if s2:
        lx, rx = SERVO_L[0] + dist, SERVO_R[0] + dist
        leg2_arts[0].set_data([lx, s2["Knee_L"][0]], [SERVO_L[1], s2["Knee_L"][1]])
        leg2_arts[1].set_data([rx, s2["Knee_R"][0]], [SERVO_R[1], s2["Knee_R"][1]])
        leg2_arts[2].set_data([s2["Knee_L"][0], s_fx2.val + dist], [s2["Knee_L"][1], s_fy2.val])
        leg2_arts[3].set_data([s2["Knee_R"][0], s_fx2.val + dist], [s2["Knee_R"][1], s_fy2.val])
        leg2_arts[4].set_data([s_fx2.val + dist], [s_fy2.val])
    else:
        for a in leg2_arts:
            a.set_data([], [])

    txt_vals.set_text(format_values(d))
    refresh_status()
    fig.canvas.draw_idle()
    _updating = False

    # ── LIVE SEND (throttled) ──
    if state["live"] and link.connected:
        now = time.time()
        if now - state["last_send"] >= SEND_INTERVAL:
            link.send_positions(positions_from(d))
            state["last_send"] = now
    return d


def send_now():
    d = compute(s_fx1.val, s_fy1.val, s_fx2.val, s_fy2.val, s_dist.val)
    link.send_positions(positions_from(d))


def print_values(_=None):
    d = compute(s_fx1.val, s_fy1.val, s_fx2.val, s_fy2.val, s_dist.val)
    print("\n" + "=" * 50)
    print(format_values(d))
    print("=" * 50)


def on_connect(_=None):
    if link.connect(_args.port, _args.baud):
        if state["live"]:
            send_now()
    refresh_status()
    fig.canvas.draw_idle()


def on_chk(label):
    if "Mirror" in label:
        state["mirror"] = not state["mirror"]
    elif "Live" in label:
        state["live"] = not state["live"]
        if state["live"]:
            if not link.connected:
                on_connect()          # auto-connect when Live is switched on
            if link.connected:
                send_now()            # push the current pose immediately
    refresh_status()
    update()


def on_free(_=None):
    link.torque_all(False)
    print("[link] all servos FREED (torque OFF)")


def on_lock(_=None):
    link.torque_all(True)
    print("[link] all servos LOCKED (torque ON)")
    send_now()


for s in (s_fx1, s_fy1, s_fx2, s_fy2, s_dist):
    s.on_changed(update)
chk.on_clicked(on_chk)
btn_conn.on_clicked(on_connect)
btn_print.on_clicked(print_values)
btn_free.on_clicked(on_free)
btn_lock.on_clicked(on_lock)

update()
print("Digital twin (Balance_Rework) ready.")
print_values()

# Auto-connect and prime live send so sliders drive the servos immediately.
print(f"\nAuto-connecting to {_args.port} for live send ...")
on_connect()
if link.connected:
    send_now()
    print("Live send is ON. Drag the sliders - servos should follow (watch [send] lines).")
else:
    print("Not connected. Fix the port (--port) or click 'Connect'. Live send is idle.")

try:
    plt.show()
finally:
    link.close()
