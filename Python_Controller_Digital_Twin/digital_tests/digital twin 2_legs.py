"""
BIPEDAL ROBOT — Digital Twin + AX-12+ Servo Control (2 Legs)
=====================================================
5-bar parallel linkage IK/FK simulation with live AX-12+ control for TWO legs.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons
import time
import sys
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ax12_protocol import AX12Controller, deg_to_ax12, ax12_to_deg, ik_angle_to_ax12

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


# ==========================================
#  CONFIG — CHANGE THESE FOR YOUR ROBOT
# ==========================================
# Geometry
SERVO_L = np.array([-30.0, 0.0])    # Left servo mount position (mm)
SERVO_R = np.array([30.0, 0.0])     # Right servo mount position (mm)
FEMUR_LEN = 55.0                    # Upper arm / crank (mm)
TIBIA_LEN = 100.0                   # Lower arm / rod (mm)

# Distance between legs in the 2D view (X offset)
LEG_DISTANCE = 180.0

# AX-12+ Servo IDs
# LEG 1 (Left Leg of Biped)
LEG1_SERVO_L_ID = 6
LEG1_SERVO_R_ID = 14

# LEG 2 (Right Leg of Biped)
LEG2_SERVO_L_ID = 0
LEG2_SERVO_R_ID = 1

# Serial port
SERIAL_PORT = 'COM10'

# Leg 2 is physically mirrored, meaning the left servo behaves like the right,
# and the right behaves like the left inversely.
LEG2_INVERTED_MOUNT = True

# ==========================================
#  CUSTOM SERVO MAPPING
# ==========================================
def map_angle_to_ax12(ik_angle, is_left=True, is_leg2=False):
    """
    Maps an IK angle to AX-12 position where exactly straight down (90 or -90) 
    maps to a servo position of 800 for left, and 430 for right.
    AX-12 Resolution: 0 to 1023 across 300 degrees. (1 degree = ~3.413 units)
    """
    # Base angle relies on which leg it is (Leg 1 points down: -90, Leg 2 is inverted: +90)
    base_angle = 90.0 if is_leg2 else -90.0
    
    # Calculate clever difference with wrap-around to prevent 360-degree snapping at horizontal lines
    diff_deg = (ik_angle - base_angle + 180.0) % 360.0 - 180.0
    
    # Base position depends on whether it's a left or right servo
    base_pos = 800 if is_left else 430
    
    # If your servos move 'backwards' from the IK, change this to -1.0
    SERVO_DIR = 1.0  
    
    ax_pos = base_pos + (diff_deg * 3.413 * SERVO_DIR)
    return int(max(0, min(1023, ax_pos)))

# ==========================================
#  KINEMATICS ENGINE
# ==========================================
def circle_intersections(p0, r0, p1, r1):
    d = np.linalg.norm(p1 - p0)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return None, None
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = np.sqrt(max(r0**2 - a**2, 0))
    p2 = p0 + a * (p1 - p0) / d
    rx = -h * (p1[1] - p0[1]) / d
    ry =  h * (p1[0] - p0[0]) / d
    return np.array([p2[0]+rx, p2[1]+ry]), np.array([p2[0]-rx, p2[1]-ry])

def solve_ik(tx, ty, leg_offset_x=0.0):
    """Inverse Kinematics: foot (x,y) → servo angles. offset_x shifts the leg mounts."""
    foot = np.array([tx, ty])
    sl = SERVO_L + np.array([leg_offset_x, 0])
    sr = SERVO_R + np.array([leg_offset_x, 0])
    
    li1, li2 = circle_intersections(sl, FEMUR_LEN, foot, TIBIA_LEN)
    ri1, ri2 = circle_intersections(sr, FEMUR_LEN, foot, TIBIA_LEN)
    if li1 is None or ri1 is None:
        return None
    kL = li1 if li1[0] < li2[0] else li2
    kR = ri1 if ri1[0] > ri2[0] else ri2
    aL = np.degrees(np.arctan2(kL[1]-sl[1], kL[0]-sl[0]))
    aR = np.degrees(np.arctan2(kR[1]-sr[1], kR[0]-sr[0]))
    return {'Knee_L': kL, 'Knee_R': kR, 'Angle_L': aL, 'Angle_R': aR}

def solve_fk(aL_deg, aR_deg, leg_offset_x=0.0):
    sl = SERVO_L + np.array([leg_offset_x, 0])
    sr = SERVO_R + np.array([leg_offset_x, 0])
    kL = sl + FEMUR_LEN * np.array([np.cos(np.radians(aL_deg)), np.sin(np.radians(aL_deg))])
    kR = sr + FEMUR_LEN * np.array([np.cos(np.radians(aR_deg)), np.sin(np.radians(aR_deg))])
    fi1, fi2 = circle_intersections(kL, TIBIA_LEN, kR, TIBIA_LEN)
    if fi1 is None:
        return None
    return fi1 if fi1[1] < fi2[1] else fi2


# ==========================================
#  SERIAL STATE
# ==========================================
controller = None

def connect_serial():
    global controller
    try:
        controller = AX12Controller(SERIAL_PORT, baudrate=115200, timeout=0.1, debug=False)
        return True
    except Exception as e:
        print(f"[WARN] Cannot connect to {SERIAL_PORT}: {e}")
        controller = None
        return False

def send_positions(ik_sol1, ik_sol2):
    if controller is None:
        return
    pos1_L = map_angle_to_ax12(ik_sol1['Angle_L'], is_left=True, is_leg2=False) if ik_sol1 else None
    pos1_R = map_angle_to_ax12(ik_sol1['Angle_R'], is_left=False, is_leg2=False) if ik_sol1 else None
    pos2_L = map_angle_to_ax12(ik_sol2['Angle_L'], is_left=True, is_leg2=True) if ik_sol2 else None
    pos2_R = map_angle_to_ax12(ik_sol2['Angle_R'], is_left=False, is_leg2=True) if ik_sol2 else None
    
    # Send to all 4 servos if we have valid IK
    if ik_sol1 and ik_sol2:
        controller.sync_positions(LEG1_SERVO_L_ID, pos1_L, LEG1_SERVO_R_ID, pos1_R,
                                 LEG2_SERVO_L_ID, pos2_L, LEG2_SERVO_R_ID, pos2_R)
    elif ik_sol1:
        controller.sync_positions(LEG1_SERVO_L_ID, pos1_L, LEG1_SERVO_R_ID, pos1_R)
    elif ik_sol2:
        controller.sync_positions(LEG2_SERVO_L_ID, pos2_L, LEG2_SERVO_R_ID, pos2_R)

# ==========================================
#  GUI
# ==========================================
BG = '#1a1a2e'
FG = '#e0e0e0'

plt.rcParams.update({
    'figure.facecolor': BG, 'axes.facecolor': '#111122',
    'axes.edgecolor': '#333355', 'axes.labelcolor': FG,
    'xtick.color': FG, 'ytick.color': FG, 'text.color': FG,
})

fig = plt.figure(figsize=(15, 10))
fig.canvas.manager.set_window_title('Bipedal Robot — 2 Legs Control')

ax = fig.add_axes([0.05, 0.40, 0.52, 0.55])
ax.set_aspect('equal')
ax.set_xlim(-150, 300)
ax.set_ylim(-200, 50)
ax.grid(True, linestyle=':', alpha=0.3, color='#333355')

def make_leg_artists(color_femur, color_tibia, label_suffix):
    lf, = ax.plot([], [], 'o-', color=color_femur, lw=6, solid_capstyle='round', ms=7, zorder=5)
    rf, = ax.plot([], [], 'o-', color=color_femur, lw=6, solid_capstyle='round', ms=7, zorder=5)
    lt, = ax.plot([], [], 'o-', color=color_tibia, lw=5, solid_capstyle='round', ms=6, zorder=4)
    rt, = ax.plot([], [], 'o-', color=color_tibia, lw=5, solid_capstyle='round', ms=6, zorder=4)
    fd, = ax.plot([], [], 'o', color='#51cf66', ms=14, zorder=6)
    fr, = ax.plot([], [], 'o', mec='#51cf66', mfc='none', ms=22, mew=2, zorder=6)
    return lf, rf, lt, rt, fd, fr

leg1_arts = make_leg_artists('#ff6b6b', '#e0e0e0', 'Leg1')
leg2_arts = make_leg_artists('#da77f2', '#a5d8ff', 'Leg2')

# ── Right panel ──
pnl = fig.add_axes([0.60, 0.40, 0.38, 0.55])
pnl.set_xlim(0, 1); pnl.set_ylim(0, 1); pnl.axis('off')

txt_warn = pnl.text(0.05, 0.9, '', fontsize=12, color='#ff6b6b', weight='bold')
txt_leg1_angles = pnl.text(0.05, 0.8, '', fontsize=14, color=FG, family='monospace', weight='bold')
txt_leg2_angles = pnl.text(0.05, 0.6, '', fontsize=14, color=FG, family='monospace', weight='bold')

# ── Sliders ──
tc = '#2a2a4a'
skw = dict(color='#7950f2', initcolor='none')

s_dist = Slider(fig.add_axes([0.08, 0.32, 0.45, 0.02]), 'Leg Dist', 100, 250, valinit=180, valstep=1, **skw)
s_fx1  = Slider(fig.add_axes([0.08, 0.28, 0.45, 0.02]), 'Leg1 X', -100, 100, valinit=0, valstep=0.5, **skw)
s_fy1  = Slider(fig.add_axes([0.08, 0.24, 0.45, 0.02]), 'Leg1 Y', -160, -20, valinit=-130, valstep=0.5, **skw)

s_fx2  = Slider(fig.add_axes([0.08, 0.18, 0.45, 0.02]), 'Leg2 X', -100, 100, valinit=0, valstep=0.5, **skw)
s_fy2  = Slider(fig.add_axes([0.08, 0.14, 0.45, 0.02]), 'Leg2 Y', -160, -20, valinit=-130, valstep=0.5, **skw)

s_spd  = Slider(fig.add_axes([0.08, 0.08, 0.45, 0.02]), 'Speed', 0, 1023, valinit=300, valstep=1, **skw)
s_trq  = Slider(fig.add_axes([0.08, 0.04, 0.45, 0.02]), 'Torque', 0, 1023, valinit=512, valstep=1, **skw)

# ── Buttons & Toggles ──
def make_btn(pos, label, hover_clr):
    b = Button(fig.add_axes(pos), label, color='#2a2a4a', hovercolor=hover_clr)
    b.label.set_color(FG)
    return b

chk_ax = fig.add_axes([0.80, 0.25, 0.15, 0.1], facecolor=BG)
chk = CheckButtons(chk_ax, ['Mirror Leg2 to Leg1', 'Live Send'], [True, False])
for lb in chk.labels: lb.set_color(FG)

btn_conn  = make_btn([0.62, 0.18, 0.14, 0.04], 'Connect', '#7950f2')
btn_ton   = make_btn([0.78, 0.18, 0.14, 0.04], 'Torque ON', '#51cf66')
btn_toff  = make_btn([0.62, 0.12, 0.14, 0.04], 'Torque OFF', '#ff6b6b')
btn_apply = make_btn([0.78, 0.12, 0.14, 0.04], 'Apply Params', '#51cf66')

state = {'mirror': True, 'live': False}

updating = False
def update_gui(val):
    global updating
    if updating: return
    updating = True
    
    if state['mirror']:
        s_fx2.set_val(-s_fx1.val)
        s_fy2.set_val(s_fy1.val)

    dist = s_dist.val
    # Leg 1 is at X=0, Leg 2 is at X=dist
    sol1 = solve_ik(s_fx1.val, s_fy1.val, 0)
    sol2 = solve_ik(s_fx2.val + dist, s_fy2.val, dist)
    
    # Draw Leg 1
    if sol1:
        leg1_arts[0].set_data([SERVO_L[0], sol1['Knee_L'][0]], [SERVO_L[1], sol1['Knee_L'][1]])
        leg1_arts[1].set_data([SERVO_R[0], sol1['Knee_R'][0]], [SERVO_R[1], sol1['Knee_R'][1]])
        leg1_arts[2].set_data([sol1['Knee_L'][0], s_fx1.val], [sol1['Knee_L'][1], s_fy1.val])
        leg1_arts[3].set_data([sol1['Knee_R'][0], s_fx1.val], [sol1['Knee_R'][1], s_fy1.val])
        leg1_arts[4].set_data([s_fx1.val], [s_fy1.val])
        leg1_arts[5].set_data([s_fx1.val], [s_fy1.val])
        
        p1L = map_angle_to_ax12(sol1['Angle_L'], is_left=True, is_leg2=False)
        p1R = map_angle_to_ax12(sol1['Angle_R'], is_left=False, is_leg2=False)
        txt_leg1_angles.set_text(
            f"Leg1 (IDs {LEG1_SERVO_L_ID}, {LEG1_SERVO_R_ID}):\n"
            f"  L: {sol1['Angle_L']:+6.1f}° -> [{p1L:4d}]\n"
            f"  R: {sol1['Angle_R']:+6.1f}° -> [{p1R:4d}]"
        )
    else:
        txt_leg1_angles.set_text("Leg1: IK INVALID (Out of reach)")
    
    # Draw Leg 2
    if sol2:
        l2_sx_l = SERVO_L[0] + dist
        l2_sx_r = SERVO_R[0] + dist
        leg2_arts[0].set_data([l2_sx_l, sol2['Knee_L'][0]], [SERVO_L[1], sol2['Knee_L'][1]])
        leg2_arts[1].set_data([l2_sx_r, sol2['Knee_R'][0]], [SERVO_R[1], sol2['Knee_R'][1]])
        leg2_arts[2].set_data([sol2['Knee_L'][0], s_fx2.val + dist], [sol2['Knee_L'][1], s_fy2.val])
        leg2_arts[3].set_data([sol2['Knee_R'][0], s_fx2.val + dist], [sol2['Knee_R'][1], s_fy2.val])
        leg2_arts[4].set_data([s_fx2.val + dist], [s_fy2.val])
        leg2_arts[5].set_data([s_fx2.val + dist], [s_fy2.val])

        if LEG2_INVERTED_MOUNT:
            # If Leg2 is physically mounted mirrored (facing the other way), invert the angles and swap L/R mapping.
            ik_L = -sol2['Angle_R']
            ik_R = -sol2['Angle_L']
        else:
            ik_L = sol2['Angle_L']
            ik_R = sol2['Angle_R']
            
        p2L = map_angle_to_ax12(ik_L, is_left=True, is_leg2=True)
        p2R = map_angle_to_ax12(ik_R, is_left=False, is_leg2=True)
        
        # Override the angles we send in sol2 dictionary for physical sync writing
        sol2['Angle_L'] = ik_L
        sol2['Angle_R'] = ik_R

        txt_leg2_angles.set_text(
            f"Leg2 (IDs {LEG2_SERVO_L_ID}, {LEG2_SERVO_R_ID}):\n"
            f"  L: {ik_L:+6.1f}° -> [{p2L:4d}]\n"
            f"  R: {ik_R:+6.1f}° -> [{p2R:4d}]"
        )
    else:
        txt_leg2_angles.set_text("Leg2: IK INVALID (Out of reach)")

    if state['live'] and controller:
        send_positions(sol1, sol2)
        
    fig.canvas.draw_idle()
    updating = False

for s in [s_fx1, s_fy1, s_fx2, s_fy2, s_dist, s_spd, s_trq]:
    s.on_changed(update_gui)

def on_chk(label):
    if 'Mirror' in label: state['mirror'] = not state['mirror']
    if 'Live' in label: state['live'] = not state['live']
    update_gui(None)
chk.on_clicked(on_chk)

def on_conn(e): connect_serial()
def on_ton(e):
    if controller:
        for i in [LEG1_SERVO_L_ID, LEG1_SERVO_R_ID, LEG2_SERVO_L_ID, LEG2_SERVO_R_ID]:
            controller.set_torque_enable(i, True)
def on_toff(e):
    if controller:
        for i in [LEG1_SERVO_L_ID, LEG1_SERVO_R_ID, LEG2_SERVO_L_ID, LEG2_SERVO_R_ID]:
            controller.set_torque_enable(i, False)
def on_apply(e):
    if controller:
        for i in [LEG1_SERVO_L_ID, LEG1_SERVO_R_ID, LEG2_SERVO_L_ID, LEG2_SERVO_R_ID]:
            controller.set_speed(i, int(s_spd.val))
            controller.set_torque_limit(i, int(s_trq.val))

btn_conn.on_clicked(on_conn)
btn_ton.on_clicked(on_ton)
btn_toff.on_clicked(on_toff)
btn_apply.on_clicked(on_apply)

update_gui(None)
plt.show()


