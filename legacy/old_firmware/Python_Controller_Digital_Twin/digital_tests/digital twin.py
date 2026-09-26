"""
BIPEDAL ROBOT — Digital Twin + AX-12+ Servo Control
=====================================================
5-bar parallel linkage IK/FK simulation with live AX-12+ control.

PC sends text commands → STM32 builds Dynamixel packets → AX-12+
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

# AX-12+ Servo IDs  ← CHANGE TO YOUR IDs
SERVO_L_ID = 6 #0 6
SERVO_R_ID = 14 # 1 14

# Serial port  ← CHANGE TO YOUR PORT
SERIAL_PORT = 'COM10'

# Angle mapping: how IK angles map to AX-12 positions
# When IK angle = 0 (pointing right), what AX-12 degree should it be?
# (We set it so that -90 deg / straight down = 150 deg physical center)
SERVO_L_OFFSET = 240.0
SERVO_R_OFFSET = 240.0

# Mirror: set True if the servo rotates opposite to simulation
SERVO_L_INVERT = False
SERVO_R_INVERT = False    # Changed to False since user mentioned right servo is reversed from expected


# ==========================================
#  KINEMATICS ENGINE
# ==========================================
def circle_intersections(p0, r0, p1, r1):
    """Find intersection points of two circles."""
    d = np.linalg.norm(p1 - p0)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return None, None
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = np.sqrt(max(r0**2 - a**2, 0))
    p2 = p0 + a * (p1 - p0) / d
    rx = -h * (p1[1] - p0[1]) / d
    ry =  h * (p1[0] - p0[0]) / d
    return np.array([p2[0]+rx, p2[1]+ry]), np.array([p2[0]-rx, p2[1]-ry])

def solve_ik(tx, ty):
    """Inverse Kinematics: foot (x,y) → servo angles."""
    foot = np.array([tx, ty])
    li1, li2 = circle_intersections(SERVO_L, FEMUR_LEN, foot, TIBIA_LEN)
    ri1, ri2 = circle_intersections(SERVO_R, FEMUR_LEN, foot, TIBIA_LEN)
    if li1 is None or ri1 is None:
        return None
    # Left knee bends outward (min X), right knee outward (max X)
    kL = li1 if li1[0] < li2[0] else li2
    kR = ri1 if ri1[0] > ri2[0] else ri2
    aL = np.degrees(np.arctan2(kL[1]-SERVO_L[1], kL[0]-SERVO_L[0]))
    aR = np.degrees(np.arctan2(kR[1]-SERVO_R[1], kR[0]-SERVO_R[0]))
    return {'Knee_L': kL, 'Knee_R': kR, 'Angle_L': aL, 'Angle_R': aR}

def solve_fk(aL_deg, aR_deg):
    """Forward Kinematics: servo angles → foot position."""
    kL = SERVO_L + FEMUR_LEN * np.array([np.cos(np.radians(aL_deg)), np.sin(np.radians(aL_deg))])
    kR = SERVO_R + FEMUR_LEN * np.array([np.cos(np.radians(aR_deg)), np.sin(np.radians(aR_deg))])
    fi1, fi2 = circle_intersections(kL, TIBIA_LEN, kR, TIBIA_LEN)
    if fi1 is None:
        return None
    return fi1 if fi1[1] < fi2[1] else fi2  # lowest Y = foot


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
        print("       Running in SIMULATION-ONLY mode")
        controller = None
        return False

def send_positions(ik_sol):
    """Send IK solution to real servos via sync write."""
    if controller is None:
        return
    pos_L = ik_angle_to_ax12(ik_sol['Angle_L'], SERVO_L_OFFSET, SERVO_L_INVERT)
    pos_R = ik_angle_to_ax12(ik_sol['Angle_R'], SERVO_R_OFFSET, SERVO_R_INVERT)
    controller.sync_positions(SERVO_L_ID, pos_L, SERVO_R_ID, pos_R)


# ==========================================
#  GUI
# ==========================================
BG = '#1a1a2e'
FG = '#e0e0e0'

plt.rcParams.update({
    'figure.facecolor': BG, 'axes.facecolor': '#111122',
    'axes.edgecolor': '#333355', 'axes.labelcolor': FG,
    'xtick.color': FG, 'ytick.color': FG, 'text.color': FG,
    'font.size': 10,
})

fig = plt.figure(figsize=(14, 9))
fig.canvas.manager.set_window_title('Bipedal Robot — Digital Twin + AX-12+ Control')

# ── Main plot ──
ax = fig.add_axes([0.05, 0.35, 0.52, 0.60])
ax.set_aspect('equal')
ax.set_xlim(-150, 150)
ax.set_ylim(-200, 50)
ax.grid(True, linestyle=':', alpha=0.3, color='#333355')
ax.set_title('5-BAR LINKAGE  |  DIGITAL TWIN', fontsize=13, weight='bold', color='#7950f2')

# Chassis bar
ax.plot([SERVO_L[0], SERVO_R[0]], [SERVO_L[1], SERVO_R[1]],
        's-', color='#555577', lw=8, markersize=12, zorder=2)
ax.text(0, 8, 'CHASSIS', ha='center', fontsize=7, color='#777799')

# Linkage artists
line_fL, = ax.plot([], [], 'o-', color='#ff6b6b', lw=6, solid_capstyle='round', ms=7, zorder=5, label='Femurs')
line_fR, = ax.plot([], [], 'o-', color='#ff6b6b', lw=6, solid_capstyle='round', ms=7, zorder=5)
line_tL, = ax.plot([], [], 'o-', color='#e0e0e0', lw=5, solid_capstyle='round', ms=6, zorder=4, label='Tibias')
line_tR, = ax.plot([], [], 'o-', color='#e0e0e0', lw=5, solid_capstyle='round', ms=6, zorder=4)
foot_dot,  = ax.plot([], [], 'o', color='#51cf66', ms=14, zorder=6, label='Foot')
foot_ring, = ax.plot([], [], 'o', mec='#51cf66', mfc='none', ms=22, mew=2, zorder=6)

ax.legend(loc='upper right', fontsize=7, facecolor=BG, edgecolor='#333355', labelcolor=FG)

# Info text on plot
txt_ik   = ax.text(-145, 40, '', fontsize=9, color='#4dabf7', weight='bold', family='monospace')
txt_fk   = ax.text(-145, 30, '', fontsize=9, color='#51cf66', family='monospace')
txt_ax   = ax.text(-145, 20, '', fontsize=8, color='#ffa94d', family='monospace')
txt_warn = ax.text(0, -100, '', fontsize=14, color='#ff6b6b', weight='bold', ha='center')

# ── Right panel ──
pnl = fig.add_axes([0.60, 0.35, 0.38, 0.60])
pnl.set_xlim(0, 1); pnl.set_ylim(0, 1); pnl.axis('off')
pnl.text(0.5, 0.97, 'SERVO TELEMETRY', ha='center', va='top',
         fontsize=13, weight='bold', color='#7950f2')

T = {}  # text objects
info = [
    ('status',  0.94, ''),
    ('sep0',    0.91, '-'*36),
    ('h_ids',   0.88, 'SERVO IDs'),
    ('ids',     0.84, ''),
    ('fb_l',    0.80, ''),
    ('fb_r',    0.76, ''),
    ('sep1',    0.73, '-'*36),
    ('h_ik',    0.70, 'IK ANGLES'),
    ('ik_l',    0.66, ''),
    ('ik_r',    0.62, ''),
    ('sep2',    0.59, '-'*36),
    ('h_ax',    0.56, 'AX-12+ COMMANDS'),
    ('ax_l',    0.52, ''),
    ('ax_r',    0.48, ''),
    ('sep3',    0.45, '-'*36),
    ('h_pr',    0.42, 'PARAMETERS'),
    ('p_spd',   0.38, ''),
    ('p_trq',   0.34, ''),
    ('p_cmg',   0.30, ''),
    ('p_csl',   0.26, ''),
    ('sep4',    0.23, '-'*36),
    ('foot',    0.19, ''),
    ('h_cmd',   0.13, 'LAST SERIAL CMD'),
    ('cmd',     0.09, ''),
]
for k, y, default in info:
    c = '#7950f2' if k.startswith('h_') or k.startswith('sep') else FG
    w = 'bold' if k.startswith('h_') else 'normal'
    T[k] = pnl.text(0.05, y, default, fontsize=9, family='monospace', color=c, weight=w)

T['ids'].set_text(f'  L: ID {SERVO_L_ID} | R: ID {SERVO_R_ID}')
T['fb_l'].set_text('  L: Pos --- | Load: --%')
T['fb_r'].set_text('  R: Pos --- | Load: --%')

# ── Sliders ──
skw = dict(color='#7950f2', initcolor='none')
tc = '#2a2a4a'

s_fx   = Slider(fig.add_axes([0.08, 0.25, 0.48, 0.022], facecolor=tc), 'Foot X',      -100, 100,  valinit=0,    valstep=0.5, **skw)
s_fy   = Slider(fig.add_axes([0.08, 0.21, 0.48, 0.022], facecolor=tc), 'Foot Y',      -160, -20,  valinit=-130, valstep=0.5, **skw)
s_spd  = Slider(fig.add_axes([0.08, 0.15, 0.48, 0.022], facecolor=tc), 'Speed',        0,   1023, valinit=300,  valstep=1,   **skw)
s_trq  = Slider(fig.add_axes([0.08, 0.11, 0.48, 0.022], facecolor=tc), 'Torque Lim',   0,   1023, valinit=512,  valstep=1,   **skw)
s_cmg  = Slider(fig.add_axes([0.08, 0.07, 0.48, 0.022], facecolor=tc), 'Comp Margin',  0,   255,  valinit=1,    valstep=1,   **skw)
s_csl  = Slider(fig.add_axes([0.08, 0.03, 0.48, 0.022], facecolor=tc), 'Comp Slope',   1,   128,  valinit=32,   valstep=1,   **skw)

for s in [s_fx, s_fy, s_spd, s_trq, s_cmg, s_csl]:
    s.label.set_color(FG); s.label.set_fontsize(8)
    s.valtext.set_color(FG); s.valtext.set_fontsize(8)

# ── Buttons ──
def make_btn(pos, label, hover_clr):
    ax_b = fig.add_axes(pos)
    b = Button(ax_b, label, color='#2a2a4a', hovercolor=hover_clr)
    b.label.set_color(FG); b.label.set_fontsize(9); b.label.set_weight('bold')
    return b

btn_conn   = make_btn([0.62, 0.18, 0.14, 0.045], 'Connect',     '#7950f2')
btn_test   = make_btn([0.78, 0.24, 0.14, 0.045], 'Test Servos', '#4dabf7')
btn_jump   = make_btn([0.62, 0.24, 0.14, 0.045], 'Jump',        '#cca831')
btn_apply  = make_btn([0.78, 0.18, 0.14, 0.045], 'Apply Params', '#51cf66')
btn_toff   = make_btn([0.62, 0.12, 0.14, 0.045], 'Torque OFF',   '#ff6b6b')
btn_ton    = make_btn([0.78, 0.12, 0.14, 0.045], 'Torque ON',    '#51cf66')
btn_send1  = make_btn([0.62, 0.06, 0.14, 0.045], 'Send Once',    '#ffa94d')

# Live send & Adaptive checkboxes
live = {'on': False}
adapt = {'on': False}
ax_chk = fig.add_axes([0.78, 0.06, 0.18, 0.045], facecolor=BG)
chk = CheckButtons(ax_chk, ['Live', 'Adapt'], [False, False])
for lb in chk.labels:
    lb.set_color(FG); lb.set_fontsize(9)


# ==========================================
#  CALLBACKS
# ==========================================
last_cmd = ['']
last_send_t = [0.0]

def on_connect(event):
    ok = connect_serial()
    if ok:
        T['status'].set_text(f'  CONNECTED ({SERIAL_PORT})')
        T['status'].set_color('#51cf66')
        btn_conn.label.set_text('Reconnect')
    else:
        T['status'].set_text('  DISCONNECTED')
        T['status'].set_color('#ff6b6b')
    fig.canvas.draw_idle()

def on_test(event):
    """Test servo connectivity with PING."""
    if not controller:
        print("[WARN] Not connected"); return
    
    print("\n" + "="*50)
    print("TESTING SERVO CONNECTIVITY")
    print("="*50)
    
    servo_l_ok = controller.ping(SERVO_L_ID)
    servo_r_ok = controller.ping(SERVO_R_ID)
    
    if servo_l_ok and servo_r_ok:
        msg = f'  BOTH SERVOS ONLINE ({SERVO_L_ID}, {SERVO_R_ID})'
        T['status'].set_text(msg)
        T['status'].set_color('#51cf66')
        print(f"\n✓ SUCCESS: Both servos responding")
    elif servo_l_ok:
        msg = f'  ID {SERVO_L_ID} OK | ID {SERVO_R_ID} OFFLINE'
        T['status'].set_text(msg)
        T['status'].set_color('#ffa94d')
        print(f"\n⚠ WARNING: Only servo {SERVO_L_ID} responding")
    elif servo_r_ok:
        msg = f'  ID {SERVO_L_ID} OFFLINE | ID {SERVO_R_ID} OK'
        T['status'].set_text(msg)
        T['status'].set_color('#ffa94d')
        print(f"\n⚠ WARNING: Only servo {SERVO_R_ID} responding")
    else:
        msg = f'  NO SERVOS RESPONDING'
        T['status'].set_text(msg)
        T['status'].set_color('#ff6b6b')
        print(f"\n✗ ERROR: No servos responding")
        print("\nTroubleshooting:")
        print("  1. Check servo power supply")
        print("  2. Verify servo IDs are correct")
        print("  3. Check serial wiring (TX/RX/GND)")
        print("  4. Ensure servos are on same bus")
    
    print("="*50 + "\n")
    last_cmd[0] = 'Ping test complete'
    fig.canvas.draw_idle()

def on_apply(event):
    if not controller:
        print("[WARN] Not connected"); return
    spd = int(s_spd.val)
    trq = int(s_trq.val)
    cmg = int(s_cmg.val)
    csl = int(s_csl.val)
    for sid in [SERVO_L_ID, SERVO_R_ID]:
        controller.set_speed(sid, spd)
        controller.set_torque_limit(sid, trq)
        controller.set_compliance_margin(sid, cmg, cmg)
        controller.set_compliance_slope(sid, csl, csl)
    last_cmd[0] = f'Applied spd={spd} trq={trq}'
    print(f"[OK] Applied params to both servos")

def on_toff(event):
    if controller:
        controller.set_torque_enable(SERVO_L_ID, False)
        controller.set_torque_enable(SERVO_R_ID, False)
        last_cmd[0] = 'Torque OFF'
        print("[OK] Torque OFF")

def on_ton(event):
    if controller:
        controller.set_torque_enable(SERVO_L_ID, True)
        controller.set_torque_enable(SERVO_R_ID, True)
        last_cmd[0] = 'Torque ON'
        print("[OK] Torque ON")

def on_send_once(event):
    """Send current IK solution to servos once."""
    if not controller:
        print("[WARN] Not connected"); return
    sol = solve_ik(s_fx.val, s_fy.val)
    if sol:
        send_positions(sol)
        pos_L = ik_angle_to_ax12(sol['Angle_L'], SERVO_L_OFFSET, SERVO_L_INVERT)
        pos_R = ik_angle_to_ax12(sol['Angle_R'], SERVO_R_OFFSET, SERVO_R_INVERT)
        last_cmd[0] = f'W,{SERVO_L_ID},{pos_L},{SERVO_R_ID},{pos_R}'
        print(f"[SENT] L→{pos_L}  R→{pos_R}")

def on_jump(event):
    """Perform a jumping motion."""
    if not controller:
        print("[WARN] Not connected"); return
    
    print("[JUMP] Initiating jump sequence...")
    # Store previous values so we can return
    orig_y = s_fy.val
    
    # 1. Crouch
    s_fy.set_val(-150)
    if live['on']: plt.pause(0.2)
    else: on_send_once(None); plt.pause(0.2)
    
    # 2. Jump (explode upwards)
    s_fy.set_val(-50)
    if live['on']: plt.pause(0.2)
    else: on_send_once(None); plt.pause(0.2)
    
    # 3. Land / Recover
    s_fy.set_val(orig_y)
    if not live['on']: on_send_once(None)
    
def on_chk_toggle(label):
    if label == 'Live':
        live['on'] = not live['on']
        print(f"[LIVE] {'ON' if live['on'] else 'OFF'}")
    elif label == 'Adapt':
        adapt['on'] = not adapt['on']
        print(f"[ADAPT] {'ON' if adapt['on'] else 'OFF'}")
        if controller:
            if adapt['on']:
                print("[ADAPT] Lowering torque limit to 150 for compliance")
                controller.set_torque_limit(SERVO_L_ID, 150)
                controller.set_torque_limit(SERVO_R_ID, 150)
                # Note: This will naturally reduce resistance (adapt to external force)
            else:
                print(f"[ADAPT] Restoring torque limit to {int(s_trq.val)}")
                controller.set_torque_limit(SERVO_L_ID, int(s_trq.val))
                controller.set_torque_limit(SERVO_R_ID, int(s_trq.val))

btn_conn.on_clicked(on_connect)
btn_test.on_clicked(on_test)
btn_jump.on_clicked(on_jump)
btn_apply.on_clicked(on_apply)
btn_toff.on_clicked(on_toff)
btn_ton.on_clicked(on_ton)
btn_send1.on_clicked(on_send_once)
chk.on_clicked(on_chk_toggle)


# ==========================================
#  MAIN UPDATE
# ==========================================
def update(val):
    tx, ty = s_fx.val, s_fy.val
    sol = solve_ik(tx, ty)

    if sol is None:
        txt_warn.set_text('TARGET OUT OF REACH!')
        txt_ik.set_text('IK: INVALID')
        txt_fk.set_text(''); txt_ax.set_text('')
        for art in [line_fL, line_fR, line_tL, line_tR]:
            art.set_data([], [])
        foot_dot.set_data([tx], [ty])
        foot_ring.set_data([tx], [ty])
        T['ik_l'].set_text(''); T['ik_r'].set_text('')
        T['ax_l'].set_text(''); T['ax_r'].set_text('')
        T['foot'].set_text(f'  ({tx:.1f}, {ty:.1f}) UNREACHABLE')
        T['foot'].set_color('#ff6b6b')
        fig.canvas.draw_idle()
        return

    txt_warn.set_text('')
    kL, kR = sol['Knee_L'], sol['Knee_R']
    aL, aR = sol['Angle_L'], sol['Angle_R']

    # Draw linkage
    line_fL.set_data([SERVO_L[0], kL[0]], [SERVO_L[1], kL[1]])
    line_fR.set_data([SERVO_R[0], kR[0]], [SERVO_R[1], kR[1]])
    line_tL.set_data([kL[0], tx], [kL[1], ty])
    line_tR.set_data([kR[0], tx], [kR[1], ty])
    foot_dot.set_data([tx], [ty])
    foot_ring.set_data([tx], [ty])

    # FK check
    fk = solve_fk(aL, aR)
    fk_txt = f'FK: ({fk[0]:.1f}, {fk[1]:.1f})' if fk is not None else ''

    # AX-12 positions
    pL = ik_angle_to_ax12(aL, SERVO_L_OFFSET, SERVO_L_INVERT)
    pR = ik_angle_to_ax12(aR, SERVO_R_OFFSET, SERVO_R_INVERT)

    # Update text
    txt_ik.set_text(f'IK: L={aL:+6.1f}  R={aR:+6.1f} deg')
    txt_fk.set_text(fk_txt)
    txt_ax.set_text(f'AX12: L={pL}  R={pR}')

    T['ik_l'].set_text(f'  Left:  {aL:+7.1f} deg'); T['ik_l'].set_color('#ff6b6b')
    T['ik_r'].set_text(f'  Right: {aR:+7.1f} deg'); T['ik_r'].set_color('#ff6b6b')
    T['ax_l'].set_text(f'  ID {SERVO_L_ID}: pos {pL:4d} ({ax12_to_deg(pL):5.1f} AX deg)')
    T['ax_l'].set_color('#ffa94d')
    T['ax_r'].set_text(f'  ID {SERVO_R_ID}: pos {pR:4d} ({ax12_to_deg(pR):5.1f} AX deg)')
    T['ax_r'].set_color('#ffa94d')

    spd = int(s_spd.val); trq = int(s_trq.val)
    cmg = int(s_cmg.val); csl = int(s_csl.val)
    T['p_spd'].set_text(f'  Speed:  {spd} ({spd*0.111:.1f} rpm)')
    T['p_trq'].set_text(f'  Torque: {trq} ({trq*100/1023:.0f}%)')
    T['p_cmg'].set_text(f'  Margin: {cmg}')
    T['p_csl'].set_text(f'  Slope:  {csl}')

    T['foot'].set_text(f'  Foot: ({tx:.1f}, {ty:.1f})'); T['foot'].set_color('#51cf66')
    T['cmd'].set_text(f'  {last_cmd[0]}'); T['cmd'].set_color('#ffa94d')

    # Live send
    if live['on'] and controller:
        now = time.time()
        if now - last_send_t[0] > 0.05:  # 50ms throttle
            send_positions(sol)
            last_cmd[0] = f'W,{SERVO_L_ID},{pL},{SERVO_R_ID},{pR}'
            last_send_t[0] = now

    fig.canvas.draw_idle()


# Connect callbacks
for s in [s_fx, s_fy, s_spd, s_trq, s_cmg, s_csl]:
    s.on_changed(update)

# Initial state
T['status'].set_text('  DISCONNECTED (sim only)')
T['status'].set_color('#ffa94d')
update(None)

def poll_telemetry():
    if controller:
        load_l = controller.get_load(SERVO_L_ID)
        load_r = controller.get_load(SERVO_R_ID)
        pos_l = controller.get_position(SERVO_L_ID)
        pos_r = controller.get_position(SERVO_R_ID)
        
        if load_l is not None and pos_l is not None:
            T['fb_l'].set_text(f'  L: Pos {pos_l:4d} ({ax12_to_deg(pos_l):5.1f}°) | Load: {abs(load_l):5.1f}% {"CCW" if load_l < 0 else "CW "}')
            T['fb_l'].set_color('#ff6b6b' if abs(load_l) > 50 else '#51cf66')
        
        if load_r is not None and pos_r is not None:
            T['fb_r'].set_text(f'  R: Pos {pos_r:4d} ({ax12_to_deg(pos_r):5.1f}°) | Load: {abs(load_r):5.1f}% {"CCW" if load_r < 0 else "CW "}')
            T['fb_r'].set_color('#ff6b6b' if abs(load_r) > 50 else '#51cf66')

        fig.canvas.draw_idle()

timer = fig.canvas.new_timer(interval=200) # Poll every 200ms
timer.add_callback(poll_telemetry)
timer.start()

plt.show()


