"""
twin_kinematics.py — YOUR tuned kinematics, extracted verbatim so tools can share them.
"""

import numpy as np

# ==========================================
#  CONFIG — copied from your twin  # <-- twin
# ==========================================
SERVO_L = np.array([-30.0, 0.0])    # Left servo mount position (mm)
SERVO_R = np.array([30.0, 0.0])     # Right servo mount position (mm)
FEMUR_LEN = 55.0                    # Upper arm / crank (mm)
TIBIA_LEN = 100.0                   # Lower arm / rod (mm)

LEG_DISTANCE = 180.0               # Distance between legs (X offset)

# AX-12+ Servo IDs
LEG1_SERVO_L_ID = 6
LEG1_SERVO_R_ID = 14
LEG2_SERVO_L_ID = 0
LEG2_SERVO_R_ID = 1

# Leg 2 is physically mirrored (left servo behaves like right, inversely).
LEG2_INVERTED_MOUNT = True


# ==========================================
#  CUSTOM SERVO MAPPING  (verbatim)  # <-- twin
# ==========================================
def map_angle_to_ax12(ik_angle, is_left=True, is_leg2=False):
    """
    Maps an IK angle to AX-12 position where exactly straight down (90 or -90)
    maps to the CALIBRATED standing pose: 818 for left, 441 for right.
    AX-12 Resolution: 0 to 1023 across 300 degrees. (1 degree = ~3.413 units)
    """
    base_angle = 90.0 if is_leg2 else -90.0
    diff_deg = (ik_angle - base_angle + 180.0) % 360.0 - 180.0
    base_pos = 818 if is_left else 441
    SERVO_DIR = 1.0
    ax_pos = base_pos + (diff_deg * 3.413 * SERVO_DIR)
    return int(max(0, min(1023, ax_pos)))


# ==========================================
#  KINEMATICS ENGINE  (verbatim)  # <-- twin
# ==========================================
def circle_intersections(p0, r0, p1, r1):
    d = np.linalg.norm(p1 - p0)
    if d > r0 + r1 or d < abs(r0 - r1) or d == 0:
        return None, None
    a = (r0**2 - r1**2 + d**2) / (2 * d)
    h = np.sqrt(max(r0**2 - a**2, 0))
    p2 = p0 + a * (p1 - p0) / d
    rx = -h * (p1[1] - p0[1]) / d
    ry = h * (p1[0] - p0[0]) / d
    return np.array([p2[0]+rx, p2[1]+ry]), np.array([p2[0]-rx, p2[1]-ry])


def solve_ik(tx, ty, leg_offset_x=0.0):
    """Inverse Kinematics: foot (x,y) -> servo angles. offset_x shifts the leg mounts."""
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


def leg1_positions(x, y):
    """Leg 1 foot (x,y) -> {6: posL, 14: posR}."""
    sol = solve_ik(x, y, 0.0)
    if sol is None:
        return None
    return {
        LEG1_SERVO_L_ID: map_angle_to_ax12(sol['Angle_L'], is_left=True, is_leg2=False),
        LEG1_SERVO_R_ID: map_angle_to_ax12(sol['Angle_R'], is_left=False, is_leg2=False),
    }


def leg2_positions(x, y, dist=LEG_DISTANCE):
    """Leg 2 foot (x,y) -> {0: posL, 1: posR}. Applies the leg2 mount inversion."""
    sol = solve_ik(x + dist, y, dist)
    if sol is None:
        return None
    if LEG2_INVERTED_MOUNT:
        ik_L = -sol['Angle_R']
        ik_R = -sol['Angle_L']
    else:
        ik_L = sol['Angle_L']
        ik_R = sol['Angle_R']
    return {
        LEG2_SERVO_L_ID: map_angle_to_ax12(ik_L, is_left=True, is_leg2=True),
        LEG2_SERVO_R_ID: map_angle_to_ax12(ik_R, is_left=False, is_leg2=True),
    }
