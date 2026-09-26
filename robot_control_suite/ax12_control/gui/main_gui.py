"""
main_gui.py — ax12_control
AX-12+ Leg Subsystem Bench Rig, 3DR telemetry edition.

Single window, no tabs — everything visible at once:

    header : port / connect / TORQUE kill / home / reset / profiles / status
    left   : the 5-bar linkage plot for BOTH legs, mode switch, crouch knob
    right  : global compliance+torque+speed, per-servo raw position, live health
    bottom : serial monitor

Pairs with the servo-only firmware in ../firmware. There is no balance PID, no
IMU and no wheel drive anywhere in this tool — the firmware pins the L298N low
at boot, so the only thing that can move is a leg.
"""

import json
import os
import time
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import numpy as np

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import twin_kinematics as tk_ik
from serial_link import SerialLink


# Calibrated standing pose — the single source of truth, mirrored from the
# firmware's BASE_FX/BASE_FY constants. Do not "tidy" these numbers.
BASE_FOOT = {1: (1.0, -151.1), 2: (-6.0, -149.6)}
LEG_DIST  = tk_ik.LEG_DISTANCE

# Per-servo raw travel guard rail. The firmware always hard-clamps 0-1023; this
# narrower window is a GUI-side rail so a stray drag cannot fold the 5-bar
# linkage into itself or drive a joint onto a hard stop.
SERVO_SPAN = 200
SERVO_ROWS = [
    (6,  "Leg1 Left  (ID 6)",  818),
    (14, "Leg1 Right (ID 14)", 441),
    (0,  "Leg2 Left  (ID 0)",  818),
    (1,  "Leg2 Right (ID 1)",  441),
]

# A STATE line older than this is not evidence of anything. The firmware
# sends one every second as a keepalive, so 3 s is three consecutive misses.
STATE_STALE_S = 3.0

MODE_CROUCH, MODE_IK = 0, 1


# ---------------------------------------------------------------------------
# Integer register slider with the pretest GUI's echo-guard discipline
# ---------------------------------------------------------------------------
class SettingSlider(ttk.Frame):
    """One AX-12 RAM register. Debounced on drag, fires immediately on release.

    Carries over two hard-won behaviours from the pretest tuner:
      * 300 ms debounce, because every surviving drag step costs an uplink
        transmission on a half-duplex radio and a burst of them queues for
        seconds. Release still sends immediately, so the final value is never
        delayed by the debounce.
      * a SELF-HEALING echo deadline. While waiting for the firmware's STATE
        ack the slider ignores incoming values, so it does not fight the user
        mid-drag. Acks DO get dropped on this radio, so without the timeout the
        flag would latch forever and the slider would stop tracking firmware for
        the rest of the session.
    """

    def __init__(self, master, label, lo, hi, initial, on_change, hint="",
                 on_resync=None):
        super().__init__(master)
        self.label_text = label
        self.on_change  = on_change
        self._resync    = on_resync
        self._retried   = False
        self._debounce  = None
        self._dragging  = False
        self._last_sent = int(initial)
        self._awaiting  = False
        self._deadline  = 0.0

        self.var = tk.IntVar(value=int(initial))

        head = ttk.Frame(self)
        head.pack(fill=tk.X)
        ttk.Label(head, text=label, font=("Helvetica", 9, "bold")).pack(side=tk.LEFT)
        self._val_lbl = ttk.Label(head, text=str(int(initial)), font=("Consolas", 9))
        self._val_lbl.pack(side=tk.RIGHT)

        self._scale = tk.Scale(self, orient=tk.HORIZONTAL, from_=lo, to=hi,
                               showvalue=False, variable=self.var,
                               command=self._on_move, length=230)
        self._scale.pack(fill=tk.X)
        if hint:
            ttk.Label(self, text=hint, font=("Helvetica", 7),
                      foreground="#666666").pack(anchor="w")
        self._scale.bind("<ButtonPress-1>",   lambda e: setattr(self, "_dragging", True))
        self._scale.bind("<ButtonRelease-1>", self._on_release)

    def _on_move(self, _raw):
        self._val_lbl.config(text=str(self.var.get()))
        if self._debounce is not None:
            self.after_cancel(self._debounce)
        if self._dragging:
            self._debounce = self.after(300, self._send_now)

    def _on_release(self, _e):
        self._dragging = False
        self._send_now()

    def _send_now(self):
        if self._debounce is not None:
            try:
                self.after_cancel(self._debounce)
            except Exception:
                pass
            self._debounce = None
        v = int(self.var.get())
        if v == self._last_sent:
            return
        self._last_sent = v
        self._awaiting  = True
        self._retried   = False
        self._deadline  = time.time() + 2.0
        self.on_change(v)

    def sync_from_firmware(self, value):
        """Adopt a firmware value. The caller MUST have checked it is fresh.

        On a missed ack this does NOT simply surrender to whatever the mirror
        holds. From here a lost command and a lost ack look identical, and
        guessing wrong means silently discarding what the operator set. So the
        first timeout re-asks the firmware what it actually holds and waits one
        more window. If the reply says the old value the command really was
        lost, and reverting is both correct and informative; if it says the new
        one then only the ack was lost, and nothing moves.
        """
        value = int(value)
        if self._awaiting and value == self._last_sent:
            self._awaiting = False          # ack arrived
            self._retried  = False
        elif self._awaiting and time.time() > self._deadline:
            if not self._retried and self._resync is not None:
                self._retried  = True
                self._deadline = time.time() + 2.0
                self._resync()              # ask the firmware directly
                return
            self._awaiting = False          # asked twice — believe the answer
            self._retried  = False
        if self._dragging or self._awaiting:
            return
        if value != self.var.get():
            self.var.set(value)
            self._val_lbl.config(text=str(value))

    def set_local(self, value):
        """Set without sending — used when loading a profile before pushing."""
        self.var.set(int(value))
        self._val_lbl.config(text=str(int(value)))
        self._last_sent = int(value)


# ---------------------------------------------------------------------------
# The 5-bar linkage view — both legs, drag the foot, explicit send
# ---------------------------------------------------------------------------
class LinkageView(ttk.Frame):
    """Draws both legs from the CURRENT GUI foot targets and lets you drag them.

    Nothing here touches the robot. Dragging only moves the local target and
    re-solves the IK for the drawing; the pose reaches the servos when the
    operator clicks Send Pose. That separation is the point: you can explore an
    unreachable or silly pose safely and see it rejected before committing it.
    """

    PICK_RADIUS_MM = 18.0

    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        # Foot targets in each leg's LOCAL frame, matching the FT<leg> command.
        self.foot = {1: list(BASE_FOOT[1]), 2: list(BASE_FOOT[2])}
        self._drag_leg = None

        self.fig = Figure(figsize=(6.2, 5.0), dpi=100)
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True, alpha=0.25)
        self.ax.set_xlim(-95, 275)
        self.ax.set_ylim(-195, 45)
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")

        self.artists = {}
        for leg, colour in ((1, "#1f77b4"), (2, "#d62728")):
            femur_l, = self.ax.plot([], [], color=colour, lw=3, solid_capstyle="round")
            femur_r, = self.ax.plot([], [], color=colour, lw=3, solid_capstyle="round")
            tibia_l, = self.ax.plot([], [], color=colour, lw=2, alpha=0.65)
            tibia_r, = self.ax.plot([], [], color=colour, lw=2, alpha=0.65)
            mounts,  = self.ax.plot([], [], "s", color="#333333", ms=6)
            knees,   = self.ax.plot([], [], "o", color=colour, ms=5)
            foot,    = self.ax.plot([], [], "o", color=colour, ms=11, mfc="none", mew=2.5)
            self.artists[leg] = dict(femur_l=femur_l, femur_r=femur_r,
                                     tibia_l=tibia_l, tibia_r=tibia_r,
                                     mounts=mounts, knees=knees, foot=foot)

        self.ax.set_title("Leg 1 (blue)   |   Leg 2 (red)   —  drag a foot circle")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.mpl_connect("button_press_event",   self._on_press)
        self.canvas.mpl_connect("motion_notify_event",  self._on_motion)
        self.canvas.mpl_connect("button_release_event", self._on_release)

        self.redraw()

    # -- coordinate helpers: leg 2 is drawn shifted right by LEG_DIST --------
    def _plot_x(self, leg, local_x):
        return local_x + (LEG_DIST if leg == 2 else 0.0)

    def _local_x(self, leg, plot_x):
        return plot_x - (LEG_DIST if leg == 2 else 0.0)

    # -- mouse ---------------------------------------------------------------
    def _on_press(self, event):
        if event.inaxes is not self.ax or self.app.mode_var.get() != MODE_IK:
            return
        for leg in (1, 2):
            fx = self._plot_x(leg, self.foot[leg][0])
            fy = self.foot[leg][1]
            if np.hypot(event.xdata - fx, event.ydata - fy) < self.PICK_RADIUS_MM:
                self._drag_leg = leg
                return

    def _on_motion(self, event):
        if self._drag_leg is None or event.inaxes is not self.ax:
            return
        leg = self._drag_leg
        cand_x = self._local_x(leg, float(event.xdata))
        cand_y = float(event.ydata)
        # Reject unreachable targets rather than clamping them: the 5-bar's
        # reach boundary is an annulus, not a box, so a naive clamp would snap
        # the foot to a corner the linkage cannot occupy. Refusing the move
        # leaves the last good pose on screen and the drag simply stalls at the
        # true edge of the workspace.
        if self._solve(leg, cand_x, cand_y) is None:
            return
        self.foot[leg] = [cand_x, cand_y]
        self.redraw()
        self.app.refresh_target_readout()

    def _on_release(self, _event):
        self._drag_leg = None

    # -- kinematics ----------------------------------------------------------
    def _solve(self, leg, x, y):
        """IK for one leg in its local frame -> the shifted-frame solution."""
        if leg == 1:
            return tk_ik.solve_ik(x, y, 0.0)
        return tk_ik.solve_ik(x + LEG_DIST, y, LEG_DIST)

    def counts(self):
        """Current GUI pose -> {servo_id: ax12 counts}, or None per leg."""
        out = {}
        p1 = tk_ik.leg1_positions(*self.foot[1])
        p2 = tk_ik.leg2_positions(self.foot[2][0], self.foot[2][1], LEG_DIST)
        if p1:
            out.update(p1)
        if p2:
            out.update(p2)
        return out

    def set_foot(self, leg, x, y):
        self.foot[leg] = [float(x), float(y)]

    def set_from_crouch(self, crouch_mm):
        """CROUCH mode: x pinned to the calibrated default, y slid vertically."""
        for leg in (1, 2):
            bx, by = BASE_FOOT[leg]
            self.foot[leg] = [bx, by + float(crouch_mm)]

    def redraw(self):
        any_invalid = False
        for leg in (1, 2):
            a = self.artists[leg]
            x, y = self.foot[leg]
            sol = self._solve(leg, x, y)
            if sol is None:
                any_invalid = True
                for k in ("femur_l", "femur_r", "tibia_l", "tibia_r", "mounts", "knees"):
                    a[k].set_data([], [])
                a["foot"].set_data([self._plot_x(leg, x)], [y])
                a["foot"].set_color("#999999")
                continue

            offset = LEG_DIST if leg == 2 else 0.0
            pts = tk_ik.linkage_points(sol["Angle_L"], sol["Angle_R"],
                                       (self._plot_x(leg, x), y), offset)
            sl, sr = pts["servo_l"], pts["servo_r"]
            kL, kR, ft = pts["knee_l"], pts["knee_r"], pts["foot"]

            a["femur_l"].set_data([sl[0], kL[0]], [sl[1], kL[1]])
            a["femur_r"].set_data([sr[0], kR[0]], [sr[1], kR[1]])
            a["tibia_l"].set_data([kL[0], ft[0]], [kL[1], ft[1]])
            a["tibia_r"].set_data([kR[0], ft[0]], [kR[1], ft[1]])
            a["mounts"].set_data([sl[0], sr[0]], [sl[1], sr[1]])
            a["knees"].set_data([kL[0], kR[0]], [kL[1], kR[1]])
            a["foot"].set_data([ft[0]], [ft[1]])
            a["foot"].set_color("#1f77b4" if leg == 1 else "#d62728")

        mode = "IK — drag a foot circle" if self.app.mode_var.get() == MODE_IK \
               else "CROUCH — use the crouch slider (dragging disabled)"
        suffix = "   [UNREACHABLE]" if any_invalid else ""
        self.ax.set_title(f"Leg 1 (blue) | Leg 2 (red)   —   {mode}{suffix}")
        self.canvas.draw_idle()


# ---------------------------------------------------------------------------
# Main application
# ---------------------------------------------------------------------------
class AX12App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AX-12 Leg Control — Wireless Bench Rig")
        self.geometry("1480x940")
        self.minsize(1280, 820)
        self.link = None
        self._poll_id = None

        self.port_var    = tk.StringVar(value="COM13")
        self.status_var  = tk.StringVar(value="Disconnected")
        self.torque_var  = tk.BooleanVar(value=True)
        self.mode_var    = tk.IntVar(value=MODE_CROUCH)
        self.target_var  = tk.StringVar(value="")
        self.link_var    = tk.StringVar(value="link: --")
        self.crouch_var  = tk.DoubleVar(value=0.0)

        self._crouch_debounce = None

        self._build_header()
        self._build_body()
        self._build_monitor()
        self.refresh_target_readout()

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self._poll_id = self.after(100, self._poll)

    # ── header ───────────────────────────────────────────────────────────────
    def _build_header(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.port_var, width=9).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Connect",    command=self.connect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT, padx=2)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)

        # The torque kill is a raw tk.Button so it can be coloured — it is the
        # one control in this window that can drop the robot.
        self.torque_btn = tk.Button(top, text="TORQUE: ON", width=14,
                                    font=("Helvetica", 10, "bold"),
                                    bg="#2e7d32", fg="white",
                                    activebackground="#2e7d32",
                                    command=self.toggle_torque)
        self.torque_btn.pack(side=tk.LEFT, padx=4)

        ttk.Button(top, text="Home Pose",   command=self.home).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Servo Reset", command=self.servo_reset).pack(side=tk.LEFT, padx=2)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=10)

        ttk.Button(top, text="Save Profile", command=self.save_profile).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Load Profile", command=self.load_profile).pack(side=tk.LEFT, padx=2)

        ttk.Label(top, textvariable=self.link_var, width=15,
                  font=("Consolas", 9, "bold")).pack(side=tk.RIGHT, padx=8)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.RIGHT, padx=8)

    # ── body: plot on the left, all controls on the right ────────────────────
    def _build_body(self):
        self.settings = {}
        body = ttk.Frame(self, padding=(8, 0))
        body.pack(fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=3)
        body.columnconfigure(1, weight=0)
        body.rowconfigure(0, weight=1)

        # -- left: linkage + mode + crouch --
        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.rowconfigure(0, weight=1)
        left.columnconfigure(0, weight=1)

        plot_box = ttk.LabelFrame(left, text="Inverse Kinematics — 5-bar linkage, both legs")
        plot_box.grid(row=0, column=0, sticky="nsew")
        self.view = LinkageView(plot_box, self)
        self.view.pack(fill=tk.BOTH, expand=True)

        pose_box = ttk.LabelFrame(left, text="Pose Mode  (the two modes are mutually exclusive)")
        pose_box.grid(row=1, column=0, sticky="ew", pady=(8, 8))

        mode_row = ttk.Frame(pose_box)
        mode_row.pack(fill=tk.X, padx=6, pady=(6, 2))
        ttk.Radiobutton(mode_row, text="CROUCH  (one vertical knob)",
                        variable=self.mode_var, value=MODE_CROUCH,
                        command=self.on_mode_change).pack(side=tk.LEFT, padx=(0, 18))
        ttk.Radiobutton(mode_row, text="IK  (drag the foot targets)",
                        variable=self.mode_var, value=MODE_IK,
                        command=self.on_mode_change).pack(side=tk.LEFT)

        crouch_row = ttk.Frame(pose_box)
        crouch_row.pack(fill=tk.X, padx=6, pady=2)
        ttk.Label(crouch_row, text="Crouch (mm)", width=12).pack(side=tk.LEFT)
        self.crouch_scale = tk.Scale(crouch_row, orient=tk.HORIZONTAL, from_=0.0, to=80.0,
                                     resolution=0.5, variable=self.crouch_var,
                                     command=self._on_crouch_move, length=330)
        self.crouch_scale.pack(side=tk.LEFT, padx=6)
        self.crouch_scale.bind("<ButtonRelease-1>", lambda e: self._send_crouch())

        # Move Time belongs here rather than with the AX-12 registers: it is
        # not a servo register at all, it is how long the firmware takes to
        # interpolate the FOOT from its current position to the commanded one.
        mt = SettingSlider(pose_box, "Move Time  (interpolated pose move)",
                           100, 3000, 800,
                           lambda v: self._on_setting("moveTime", v),
                           "ms for a pose move, any distance. "
                           "smoothstep ramp, all 4 servos sync-written per tick",
                           on_resync=self._request_state)
        mt.pack(fill=tk.X, padx=6, pady=(2, 4))
        self.settings["moveTime"] = mt

        send_row = ttk.Frame(pose_box)
        send_row.pack(fill=tk.X, padx=6, pady=(2, 8))
        self.send_btn = ttk.Button(send_row, text="Send Pose to Robot",
                                   command=self.send_pose)
        self.send_btn.pack(side=tk.LEFT)
        ttk.Label(send_row, textvariable=self.target_var,
                  font=("Consolas", 9)).pack(side=tk.LEFT, padx=12)

        # -- right: settings, per-servo, health --
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")

        set_box = ttk.LabelFrame(right, text="Global Servo Settings  (all four, identical)")
        set_box.pack(fill=tk.X, pady=(0, 8))
        for key, label, lo, hi, init, hint in [
            ("torqueLimit", "Torque Limit  (addr 34)", 0, 1023, 1023,
             "holding force. 1023 = full; also doubles jam force"),
            ("compMargin",  "Compliance Margin  (26/27)", 0, 254, 1,
             "deadband. higher = joint ignores small errors, feels loose"),
            ("compSlope",   "Compliance Slope  (28/29)", 0, 254, 4,
             "proportional band. higher = wider = slow ease-in, weak hold"),
            ("movingSpeed", "Moving Speed  (addr 32)", 0, 1023, 0,
             "slew cap toward goal. 0 = uncapped"),
        ]:
            s = SettingSlider(set_box, label, lo, hi, init,
                              lambda v, k=key: self._on_setting(k, v), hint,
                              on_resync=self._request_state)
            s.pack(fill=tk.X, padx=6, pady=3)
            self.settings[key] = s

        servo_box = ttk.LabelFrame(
            right,
            text=f"Servo Control — raw counts, bypasses IK (travel guarded to standing +/-{SERVO_SPAN})")
        servo_box.pack(fill=tk.X, pady=(0, 8))
        self.servo_vars = {}
        self.unlock_var = tk.BooleanVar(value=False)
        self.servo_scales = {}
        for sid, name, home_pos in SERVO_ROWS:
            row = ttk.Frame(servo_box)
            row.pack(fill=tk.X, padx=6, pady=2)
            ttk.Label(row, text=name, width=18).pack(side=tk.LEFT)
            var = tk.IntVar(value=home_pos)
            self.servo_vars[sid] = var
            sc = tk.Scale(row, orient=tk.HORIZONTAL,
                          from_=max(0, home_pos - SERVO_SPAN),
                          to=min(1023, home_pos + SERVO_SPAN),
                          variable=var, length=210, showvalue=True)
            sc.pack(side=tk.LEFT, padx=4)
            self.servo_scales[sid] = sc
            # Explicit per-row Send, never live-drag-send: nothing moves without
            # a deliberate click.
            ttk.Button(row, text="Send", width=6,
                       command=lambda s=sid, v=var: self.send_servo(s, v.get())
                       ).pack(side=tk.LEFT, padx=3)
        ttk.Button(servo_box, text="Load Current Pose into Sliders",
                   command=self.pull_pose_into_sliders).pack(anchor="w", padx=6, pady=(4, 6))

        health_box = ttk.LabelFrame(right, text="Live Servo Health  (2.5 Hz per servo)")
        health_box.pack(fill=tk.X)
        self.health_labels = {}
        for sid, name, _ in SERVO_ROWS:
            lbl = tk.Label(health_box, text=f"ID {sid} ({name}): waiting...",
                           font=("Consolas", 9), bg="#eeeeee", fg="black",
                           anchor="w", justify=tk.LEFT, pady=4, padx=6)
            lbl.pack(fill=tk.X, padx=6, pady=2)
            self.health_labels[sid] = lbl
        self.actual_var = tk.StringVar(value="Actual foot (from present position): --")
        ttk.Label(health_box, textvariable=self.actual_var,
                  font=("Consolas", 8), foreground="#444444").pack(anchor="w", padx=6, pady=(2, 6))

    def _build_monitor(self):
        box = ttk.LabelFrame(self, text="Serial Monitor")
        box.pack(fill=tk.BOTH, expand=False, padx=8, pady=(0, 8))
        self.log_text = tk.Text(box, height=8, state=tk.DISABLED, font=("Consolas", 8))
        self.log_text.pack(fill=tk.BOTH, expand=True)

    # ── connection ───────────────────────────────────────────────────────────
    def connect(self):
        if self.link:
            return
        port = self.port_var.get()
        try:
            self.link = SerialLink(port, baud=115200)
            self.link.connect()
            self.status_var.set(f"Connected to {port} @ 115200")
        except Exception as exc:
            self.link = None
            messagebox.showerror("Connection Error", str(exc))

    def disconnect(self):
        if self.link:
            self.link.close()
            self.link = None
        self.status_var.set("Disconnected")

    # ── actions ──────────────────────────────────────────────────────────────
    def toggle_torque(self):
        if not self.link:
            return
        going_limp = self.torque_var.get()      # currently ON -> about to kill
        if going_limp:
            if not messagebox.askokcancel(
                    "Release all servo torque?",
                    "All four AX-12 servos will go COMPLETELY LIMP.\n\n"
                    "The legs will collapse under the robot's own weight unless "
                    "it is supported.\n\nSupport the robot before continuing."):
                return
        self.torque_var.set(not going_limp)
        self.link.set_torque(self.torque_var.get())
        self.status_var.set("Torque released (LIMP)" if going_limp else "Torque re-engaged")

    def home(self):
        self.crouch_var.set(0.0)
        self.view.set_foot(1, *BASE_FOOT[1])
        self.view.set_foot(2, *BASE_FOOT[2])
        self.view.redraw()
        self.refresh_target_readout()
        if self.link:
            self.link.home()

    def servo_reset(self):
        if self.link:
            self.link.servo_reset()
            self.status_var.set("Servo bus re-initialised")

    def on_mode_change(self):
        """Mirror the firmware's zero-motion mode seeding exactly.

        The firmware seeds the incoming mode from the pose the outgoing one is
        holding, so a mode switch never moves the robot. This must mirror that
        arithmetic or the plot would show one pose while the servos hold another.
          CROUCH -> IK : the crouch pose is already a foot target, copy it.
          IK -> CROUCH : crouch has one DOF, so take the mean vertical
                         displacement and give up the horizontal offset.
        """
        mode = self.mode_var.get()
        if mode == MODE_CROUCH:
            crouch = 0.5 * ((self.view.foot[1][1] - BASE_FOOT[1][1]) +
                            (self.view.foot[2][1] - BASE_FOOT[2][1]))
            self.crouch_var.set(round(crouch, 1))
            self.view.set_from_crouch(self.crouch_var.get())
        # CROUCH -> IK needs no work: view.foot already holds the crouch pose,
        # which is precisely what the firmware copies into ft_x/ft_y.
        self.crouch_scale.config(state=tk.NORMAL if mode == MODE_CROUCH else tk.DISABLED)
        self.view.redraw()
        self.refresh_target_readout()
        if self.link:
            self.link.set_mode(mode)

    def _on_crouch_move(self, _raw):
        # Live LOCAL preview only — the wire send waits for button release, so a
        # drag across the slider does not fire 40 uplink packets.
        if self.mode_var.get() != MODE_CROUCH:
            return
        self.view.set_from_crouch(self.crouch_var.get())
        self.view.redraw()
        self.refresh_target_readout()

    def _send_crouch(self):
        if self.link and self.mode_var.get() == MODE_CROUCH:
            self.link.set_crouch(self.crouch_var.get())
            self.status_var.set(f"Crouch -> {self.crouch_var.get():.1f} mm")

    def send_pose(self):
        """Explicit commit of the dragged IK pose. Nothing moves before this."""
        if not self.link:
            self.status_var.set("Not connected")
            return
        if self.mode_var.get() != MODE_IK:
            self.status_var.set("Send Pose only applies in IK mode")
            return
        counts = self.view.counts()
        if len(counts) < 4:
            messagebox.showwarning("Unreachable pose",
                                   "At least one leg has no IK solution at this "
                                   "foot target. Nothing was sent.")
            return
        # One atomic command so both legs start their move on the same tick.
        self.link.set_foot_all(self.view.foot[1][0], self.view.foot[1][1],
                               self.view.foot[2][0], self.view.foot[2][1])
        self.status_var.set("Pose sent: " + "  ".join(
            f"ID{k}={v}" for k, v in sorted(counts.items())))

    def send_servo(self, sid, pos):
        if self.link:
            self.link.set_servo_position(sid, pos)
            self.status_var.set(f"Sent PS{sid} {pos}")

    def pull_pose_into_sliders(self):
        """Copy the servos' reported present positions into the raw sliders.

        Makes the raw panel a safe starting point: you nudge from where the
        joint actually IS rather than from a stale slider value that would make
        the servo jump the moment you hit Send.
        """
        if not self.link:
            return
        for sid, data in self.link.servo_snapshot().items():
            if data["pos"] is None or sid not in self.servo_vars:
                continue
            sc = self.servo_scales[sid]
            lo, hi = int(sc.cget("from")), int(sc.cget("to"))
            self.servo_vars[sid].set(max(lo, min(hi, int(data["pos"]))))
        self.status_var.set("Sliders loaded from present positions")

    def _request_state(self):
        """Ask the firmware to restate everything (the RB command)."""
        if self.link:
            self.link.request_state()

    def _on_setting(self, key, value):
        if not self.link:
            return
        {
            "torqueLimit": self.link.set_torque_limit,
            "compMargin":  self.link.set_comp_margin,
            "compSlope":   self.link.set_comp_slope,
            "movingSpeed": self.link.set_moving_speed,
            "moveTime":    self.link.set_move_time,
        }[key](value)
        self.status_var.set(f"{key} -> {value}")

    def refresh_target_readout(self):
        counts = self.view.counts()
        f1, f2 = self.view.foot[1], self.view.foot[2]
        if len(counts) < 4:
            self.target_var.set("target UNREACHABLE — Send disabled")
            return
        self.target_var.set(
            f"L1({f1[0]:+6.1f},{f1[1]:+7.1f}) -> 6:{counts.get(6,0):4d} 14:{counts.get(14,0):4d}   "
            f"L2({f2[0]:+6.1f},{f2[1]:+7.1f}) -> 0:{counts.get(0,0):4d} 1:{counts.get(1,0):4d}")

    # ── profiles ─────────────────────────────────────────────────────────────
    def _profiles_dir(self):
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")
        os.makedirs(d, exist_ok=True)
        return d

    def save_profile(self):
        data = {k: s.var.get() for k, s in self.settings.items()}
        data.update({
            "mode":   self.mode_var.get(),
            "crouch": round(float(self.crouch_var.get()), 2),
            "fx1": round(self.view.foot[1][0], 2), "fy1": round(self.view.foot[1][1], 2),
            "fx2": round(self.view.foot[2][0], 2), "fy2": round(self.view.foot[2][1], 2),
        })
        path = os.path.join(self._profiles_dir(),
                            f"ax12_{time.strftime('%Y%m%d_%H%M%S')}.json")
        try:
            with open(path, "w") as f:
                json.dump(data, f, indent=4)
            self.status_var.set(f"Saved {os.path.basename(path)}")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    def load_profile(self):
        path = filedialog.askopenfilename(initialdir=self._profiles_dir(),
                                          filetypes=[("Profile", "*.json")])
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
        except Exception as e:
            messagebox.showerror("Load Error", str(e))
            return

        # Update every widget locally FIRST, then push in one deliberate burst.
        # Pushing per-widget as they update would interleave with the sliders'
        # echo guards and leave some of them ignoring the firmware for 2 s.
        for k, s in self.settings.items():
            if k in data:
                s.set_local(data[k])
        self.mode_var.set(int(data.get("mode", MODE_CROUCH)))
        self.crouch_var.set(float(data.get("crouch", 0.0)))
        self.view.set_foot(1, data.get("fx1", BASE_FOOT[1][0]), data.get("fy1", BASE_FOOT[1][1]))
        self.view.set_foot(2, data.get("fx2", BASE_FOOT[2][0]), data.get("fy2", BASE_FOOT[2][1]))
        self.crouch_scale.config(
            state=tk.NORMAL if self.mode_var.get() == MODE_CROUCH else tk.DISABLED)
        self.view.redraw()
        self.refresh_target_readout()

        if self.link:
            for k, s in self.settings.items():
                self._on_setting(k, s.var.get())
            self.link.set_mode(self.mode_var.get())
            if self.mode_var.get() == MODE_CROUCH:
                self.link.set_crouch(self.crouch_var.get())
            else:
                self.link.set_foot_all(self.view.foot[1][0], self.view.foot[1][1],
                                       self.view.foot[2][0], self.view.foot[2][1])
        self.status_var.set(f"Loaded {os.path.basename(path)}")

    # ── periodic refresh ─────────────────────────────────────────────────────
    def _poll(self):
        if self.link:
            st  = self.link.state()
            age = self.link.state_age()

            # Sync sliders ONLY from a fresh, real STATE line. Two ways this
            # goes wrong otherwise, and both look like "the slider moved on its
            # own": age is None means no STATE has EVER arrived, so the mirror
            # still holds the GUI's seeded guesses; a large age means the link
            # died and the mirror is frozen at whatever it last heard. Neither
            # is grounds for overruling what the operator just set.
            if age is None:
                self.link_var.set("link: no reply")
            elif age > STATE_STALE_S:
                self.link_var.set("link: STALE %.0fs" % age)
            else:
                self.link_var.set("link: ok")
                for key in self.settings:
                    if key in st:
                        self.settings[key].sync_from_firmware(st[key])

            on = bool(st.get("torqueOn", 1))
            if on != self.torque_var.get():
                self.torque_var.set(on)
            self.torque_btn.config(
                text="TORQUE: ON" if on else "TORQUE: LIMP",
                bg="#2e7d32" if on else "#c62828",
                activebackground="#2e7d32" if on else "#c62828")

            self._refresh_health()

        self._poll_id = self.after(150, self._poll)

    def _refresh_health(self):
        snap = self.link.servo_snapshot()
        names = {sid: name for sid, name, _ in SERVO_ROWS}
        for sid, d in snap.items():
            if sid not in self.health_labels:
                continue
            lbl = self.health_labels[sid]
            if d["pos"] is None:
                lbl.config(text=f"ID {sid} ({names[sid]}): waiting...", bg="#eeeeee", fg="black")
                continue
            # goal - present IS the compliance droop: the joint settles short of
            # its target by however much load the margin/slope band lets through.
            droop = (d["goal"] - d["pos"]) if d["goal"] is not None else 0
            lbl.config(text=(f"ID {sid} ({names[sid]})  pos {d['pos']:4d}  goal "
                             f"{d['goal'] if d['goal'] is not None else '----'}  "
                             f"err {droop:+4d}  load {d['load']:5.1f}%  "
                             f"{d['temp']:3d}C  {d['volt']:4.1f}V"))
            if d["temp"] >= 65:
                lbl.config(bg="#ff3333", fg="white")
            elif d["temp"] >= 55:
                lbl.config(bg="#ffaa00", fg="black")
            elif d["volt"] and d["volt"] < 10.0:
                lbl.config(bg="#ffd54f", fg="black")   # pack sagging, not compliance
            else:
                lbl.config(bg="#eeeeee", fg="black")

        # Where the legs ACTUALLY are, run back through the same calibration.
        # "no data" and "unreachable" are kept distinct on purpose: the first
        # just means that servo's 2.5 Hz slot has not come round yet, while the
        # second means the two reported positions do not describe a pose the
        # 5-bar can physically be in — i.e. a servo is reporting garbage or has
        # been forced past its linkage. Collapsing them would hide a real fault.
        def actual(ids, solver):
            if any(snap[i]["pos"] is None for i in ids):
                return "   no data   "
            try:
                p = solver(*[snap[i]["pos"] for i in ids])
            except Exception:
                p = None
            return f"({p[0]:+6.1f},{p[1]:+7.1f})" if p else " unreachable "

        self.actual_var.set(
            "Actual foot (from present position):  "
            f"L1 {actual((6, 14), tk_ik.leg1_foot_from_positions)}   "
            f"L2 {actual((0, 1), tk_ik.leg2_foot_from_positions)}")

        lines = self.link.recent_lines(60)
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert(tk.END, "\n".join(lines))
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def on_close(self):
        # Cancel the queued tick first. Without this the pending after()
        # fires into an already-destroyed widget on exit.
        if self._poll_id is not None:
            try:
                self.after_cancel(self._poll_id)
            except Exception:
                pass
            self._poll_id = None
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    AX12App().mainloop()
