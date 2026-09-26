"""
main_gui.py
Single-Position Balancing Tuner & Position Controller GUI for mcu_pos_wireless.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import time
import math
import json
import os

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from serial_link import SerialLink
import twin_kinematics as tkin


# ---------------------------------------------------------------------------
# Parameter specification
# ---------------------------------------------------------------------------
class ParamSpec:
    def __init__(self, key, label, coarse_min, coarse_max, coarse_step,
                 fine_span, fine_step, digits, command_type, start_val):
        self.key          = key
        self.label        = label
        self.coarse_min   = coarse_min
        self.coarse_max   = coarse_max
        self.coarse_step  = coarse_step
        self.fine_span    = fine_span
        self.fine_step    = fine_step
        self.digits       = digits
        self.command_type = command_type
        self.start_val    = start_val


PARAM_SPECS = [
    ParamSpec("Kp",          "Kp (Balance)",  0.0,  200.0,  0.1,   5.0,  0.01,   3, "gain",  95.0),
    ParamSpec("Ki",          "Ki (Balance)",  0.0, 1000.0,  0.5,   1.0, 0.001,   4, "gain", 670.0),
    ParamSpec("Kd",          "Kd (Balance)",  0.0,   50.0,  0.1,  10.0,  0.01,   3, "gain",   1.9),
    ParamSpec("alpha",       "Alpha (CF)",    0.80, 0.999, 0.001, 0.02, 0.0001,  4, "gain",  0.96),
    ParamSpec("targetAngle", "Base Angle",   -20.0, 20.0,  0.1,   5.0,  0.01,   3, "target", 0.0),
    ParamSpec("maxSafeTilt", "Safe Tilt",     5.0, 50.0,  0.1,   5.0,  0.01,   2, "tilt",  25.0),
    # ── Cascaded loops ─────────────────────────────────────────────────────────────────
    ParamSpec("Kp_vel",      "Kp Vel→Tilt",   0.0,   0.5, 0.001, 0.05, 0.0001,  5, "gain",  0.02),
    ParamSpec("Ki_vel",      "Ki Vel→Tilt",   0.0,   0.1, 0.001, 0.01, 0.0001,  5, "gain", 0.001),
    ParamSpec("vel_alpha",   "Vel Filter α",  0.0,  0.99, 0.01,  0.1,  0.005,   3, "gain",  0.85),
    ParamSpec("Kp_pos",      "Kp Pos Hold",   0.0,   5.0,  0.01,  0.5,  0.001,  4, "gain",   0.5),
]

IK_PARAM_SPECS = [
    ParamSpec("fx1",  "Leg1 X",  -100.0, 100.0,  1.0, 10.0, 0.1, 1, "ik",   1.0),
    ParamSpec("fy1",  "Leg1 Y",  -160.0, -20.0,  1.0, 10.0, 0.1, 1, "ik",-151.1),
    ParamSpec("fx2",  "Leg2 X",  -100.0, 100.0,  1.0, 10.0, 0.1, 1, "ik",  -6.0),
    ParamSpec("fy2",  "Leg2 Y",  -160.0, -20.0,  1.0, 10.0, 0.1, 1, "ik",-149.6),
    ParamSpec("dist", "Leg Dist", 100.0, 250.0,  1.0, 20.0, 0.1, 1, "ik", 180.0),
    ParamSpec("lean", "Body Lean",-45.0,  45.0,  1.0, 10.0, 0.1, 1, "ik",   0.0),
]

CMD_PARAM_SPECS = [
    ParamSpec("trq_limit",  "Torque Limit", 0.0, 1023.0, 1.0, 100.0, 1.0, 0, "cmd", 511.0),
    ParamSpec("cmp_margin", "Comp Margin",  0.0,  254.0,  1.0,  20.0, 1.0, 0, "cmd",   4.0),
    ParamSpec("cmp_slope",  "Comp Slope",   0.0,  254.0,  1.0,  20.0, 1.0, 0, "cmd",  32.0),
]


# ---------------------------------------------------------------------------
# Coarse/Fine precision zoom slider
# ---------------------------------------------------------------------------
class CoarseFineSlider(ttk.Frame):
    def __init__(self, master, spec: ParamSpec, initial_value: float, on_change_callback):
        super().__init__(master)
        self.spec               = spec
        self.on_change_callback = on_change_callback
        self._debounce_id       = None
        self._zoom              = tk.BooleanVar(value=False)
        self._value             = tk.DoubleVar(value=initial_value)
        self._entry_value       = tk.StringVar(value=self._fmt(initial_value))
        self._last_sent         = initial_value
        self._user_dragging     = False
        self._entry_focused     = False
        self._awaiting_echo     = False

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=0)
        self.columnconfigure(2, weight=0)

        ttk.Label(self, text=spec.label, font=("Helvetica", 9, "bold")).grid(
            row=0, column=0, sticky="w", pady=(2, 0))
        self._value_label = ttk.Label(self, text=self._fmt(initial_value))
        self._value_label.grid(row=0, column=1, sticky="e", pady=(2, 0))

        self._entry = ttk.Entry(self, textvariable=self._entry_value, width=8)
        self._entry.grid(row=0, column=2, sticky="e", padx=(5, 0), pady=(2, 0))
        self._entry.bind("<FocusIn>",  lambda e: setattr(self, "_entry_focused", True))
        self._entry.bind("<FocusOut>", lambda e: setattr(self, "_entry_focused", False))
        self._entry.bind("<Return>",   self._on_entry_commit)

        self._scale = tk.Scale(
            self, orient=tk.HORIZONTAL, showvalue=False,
            resolution=spec.coarse_step, from_=spec.coarse_min, to=spec.coarse_max,
            variable=self._value, command=self._on_change, length=180,
            bg="#0A192F", fg="#E6E6E6", troughcolor="#050B14", 
            highlightthickness=0, activebackground="#FF8C00"
        )
        self._scale.grid(row=1, column=0, columnspan=2, sticky="ew", padx=(0, 5), pady=(0, 2))
        self._scale.bind("<ButtonPress-1>",   lambda e: setattr(self, "_user_dragging", True))
        self._scale.bind("<ButtonRelease-1>", self._on_release)

        ttk.Checkbutton(self, text="Zoom", variable=self._zoom,
                        command=self._apply_zoom_mode).grid(
            row=1, column=2, sticky="w", padx=(5, 0), pady=(0, 2))

        self._apply_zoom_mode()

    def _fmt(self, v): return f"{v:.{self.spec.digits}f}"

    def _quantize(self, v):
        step = float(self._scale.cget("resolution"))
        return round(v / step) * step if step > 0 else v

    def _clamp(self, v):
        lo, hi = float(self._scale.cget("from")), float(self._scale.cget("to"))
        return max(min(v, max(lo, hi)), min(lo, hi))

    def _on_release(self, _e):
        self._user_dragging = False
        self._send_now()

    def _on_entry_commit(self, _e):
        self.apply_entry_value()
        return "break"

    def _on_change(self, _raw):
        v = self._clamp(self._quantize(self._value.get()))
        if abs(v - self._value.get()) > 1e-12:
            self._value.set(v)
        self._value_label.config(text=self._fmt(v))
        if not self._entry_focused:
            self._entry_value.set(self._fmt(v))
        if self._debounce_id is not None:
            self.after_cancel(self._debounce_id)
        if self._user_dragging:
            self._debounce_id = self.after(150, self._send_now)

    def _apply_zoom_mode(self):
        cur = float(self._value.get())
        if self._zoom.get():
            lo = max(cur - self.spec.fine_span, self.spec.coarse_min)
            hi = min(cur + self.spec.fine_span, self.spec.coarse_max)
            if hi - lo < self.spec.fine_step:
                hi = min(self.spec.coarse_max, lo + self.spec.fine_step)
            self._scale.configure(from_=lo, to=hi, resolution=self.spec.fine_step)
        else:
            self._scale.configure(from_=self.spec.coarse_min, to=self.spec.coarse_max,
                                  resolution=self.spec.coarse_step)
        self._scale.set(self._clamp(cur))
        v = float(self._value.get())
        self._value_label.config(text=self._fmt(v))
        if not self._entry_focused:
            self._entry_value.set(self._fmt(v))

    def set_value(self, value: float, send: bool = False):
        value = self._clamp(self._quantize(value))
        self._value.set(value)
        self._scale.set(value)
        self._value_label.config(text=self._fmt(value))
        if not self._entry_focused:
            self._entry_value.set(self._fmt(value))
        if send:
            self._send_now()

    def get_value(self) -> float:
        return float(self._value.get())

    def _send_now(self):
        if self._debounce_id is not None:
            try:
                self.after_cancel(self._debounce_id)
            except Exception:
                pass
            self._debounce_id = None
        v = self._clamp(self._quantize(self._value.get()))
        self._value.set(v)
        self._scale.set(v)
        self._value_label.config(text=self._fmt(v))
        self._entry_value.set(self._fmt(v))
        if abs(v - self._last_sent) < 1e-9:
            return
        self._last_sent = v
        self._awaiting_echo = True
        self.on_change_callback(self.spec.key, v)

    def apply_entry_value(self):
        raw = self._entry_value.get().strip()
        if not raw:
            self._entry_value.set(self._fmt(self._value.get()))
            return
        try:
            v = float(raw)
        except ValueError:
            self._entry_value.set(self._fmt(self._value.get()))
            return
        self._value.set(v)
        self._scale.set(self._clamp(self._quantize(v)))
        self._send_now()

    def sync_from_external(self, value: float):
        if self._user_dragging or self._awaiting_echo or self._entry_focused:
            return
        value = self._clamp(self._quantize(value))
        if abs(value - self._value.get()) < 1e-9:
            return
        self._value.set(value)
        self._scale.set(value)
        self._value_label.config(text=self._fmt(value))
        self._entry_value.set(self._fmt(value))

    def mark_echo_received(self, value: float):
        if abs(self._clamp(self._quantize(value)) - self._last_sent) < 1e-9:
            self._awaiting_echo = False


# ---------------------------------------------------------------------------
# TAB 1: Position & Balance Tuner
# ---------------------------------------------------------------------------
class BalanceTunerTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app       = app
        self._plot_max = 250
        self._build_ui()

    def _build_ui(self):
        self.columnconfigure(0, weight=2)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.LabelFrame(self, text="Live Telemetry")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.fig = Figure(figsize=(6, 4), dpi=100)
        self.fig.patch.set_facecolor("#0A192F")
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_facecolor("#050B14")
        self.ax2 = self.ax.twinx()
        
        self.ax.tick_params(colors="#E6E6E6")
        for spine in self.ax.spines.values():
            spine.set_color("#E6E6E6")
            
        self.ax2.tick_params(colors="#E6E6E6")
        for spine in self.ax2.spines.values():
            spine.set_color("#E6E6E6")
            
        self.ax.grid(True, alpha=0.15, color="#E6E6E6")

        self.pitch_line,    = self.ax.plot([], [], color="#4ea8de", label="pitch (°)")
        self.target_line,   = self.ax.plot([], [], color="#38b000", ls="--", label="target (°)")
        self.tiltbias_line, = self.ax.plot([], [], color="#f72585", ls=":",  label="tilt_bias (°)", lw=1.5)
        self.pid_line,      = self.ax2.plot([], [], color="#FF8C00", alpha=0.9, label="pid_out")
        self.vel_line,      = self.ax2.plot([], [], color="#9d4edd", alpha=0.7, label="vel (c/s)")
        self.pos_err_line,  = self.ax2.plot([], [], color="#ff4d6d", ls="-.", alpha=0.7, label="pos_err")
        self.ax.legend(loc="upper left", fontsize=8, facecolor="#112240", edgecolor="#FF8C00", labelcolor="#E6E6E6")

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        right    = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="nsew")

        # ── Position & Motion Target Bar (±3000 Ticks) ─────────────────────
        pos_box = ttk.LabelFrame(right, text="Position & Motion Target Control")
        pos_box.pack(fill=tk.X, pady=(0, 10))

        # Position Bar: ±3000 ticks
        pos_spec = ParamSpec("pos_offset", "Position Target Offset", -3000.0, 3000.0, 10.0, 500.0, 1.0, 0, "pos", 0.0)
        self.pos_slider = CoarseFineSlider(pos_box, pos_spec, 0.0, self._on_pos_offset_change)
        self.pos_slider.pack(fill=tk.X, pady=4, padx=5)

        self.rot_label = ttk.Label(pos_box, text="Rotations: 0.00 rev (313 ticks/rev)", font=("Consolas", 9, "bold"))
        self.rot_label.pack(fill=tk.X, padx=5, pady=(0, 4))

        # Steering / Turn Bias Slider: ±100
        turn_spec = ParamSpec("turn_bias", "Turn / Steering Bias", -100.0, 100.0, 1.0, 20.0, 0.5, 1, "turn", 0.0)
        self.turn_slider = CoarseFineSlider(pos_box, turn_spec, 0.0, self._on_turn_bias_change)
        self.turn_slider.pack(fill=tk.X, pady=4, padx=5)

        btn_box = ttk.Frame(pos_box)
        btn_box.pack(fill=tk.X, padx=5, pady=4)
        ttk.Button(btn_box, text="Zero Pos Offset", command=self._reset_pos_offset).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_box, text="Zero Turn Bias", command=self._reset_turn_bias).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_box, text="Latch Home Here", command=self._latch_home).pack(side=tk.LEFT, padx=2)

        # ── PID & Cascade Parameter Sliders ──────────────────────────────────
        controls = ttk.LabelFrame(right, text="PID & Cascade Loop Parameters")
        controls.pack(fill=tk.X, pady=(0, 10))

        self.sliders = {}
        for spec in PARAM_SPECS:
            ctrl = CoarseFineSlider(controls, spec, spec.start_val, self._on_slider)
            ctrl.pack(fill=tk.X, pady=3, padx=5)
            self.sliders[spec.key] = ctrl

        log_frame = ttk.LabelFrame(right, text="Serial Monitor")
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=8, state=tk.DISABLED, 
                                bg="#050B14", fg="#E6E6E6", insertbackground="#FF8C00", 
                                selectbackground="#FF8C00", selectforeground="#050B14", 
                                highlightthickness=0, bd=0)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _on_pos_offset_change(self, key, val):
        revs = val / 313.0
        self.rot_label.config(text=f"Rotations: {revs:+.2f} rev (313 ticks/rev)")
        if self.app.link and self.app.link.ser:
            self.app.link.set_pos_offset(val)

    def _on_turn_bias_change(self, key, val):
        if self.app.link and self.app.link.ser:
            self.app.link.set_turn_bias(val)

    def _reset_pos_offset(self):
        self.pos_slider.set_value(0.0, send=True)

    def _reset_turn_bias(self):
        self.turn_slider.set_value(0.0, send=True)

    def _latch_home(self):
        if self.app.link and self.app.link.ser:
            self.app.link.latch_home()
            self.pos_slider.set_value(0.0, send=False)

    def _on_slider(self, key, val):
        if not (self.app.link and self.app.link.ser):
            return
        lk = self.app.link
        if   key == "Kp":          lk.set_kp(val)
        elif key == "Ki":          lk.set_ki(val)
        elif key == "Kd":          lk.set_kd(val)
        elif key == "Kp_vel":      lk.set_kp_vel(val)
        elif key == "Ki_vel":      lk.set_ki_vel(val)
        elif key == "vel_alpha":   lk.set_vel_alpha(val)
        elif key == "Kp_pos":      lk.set_kp_pos(val)
        elif key == "alpha":       lk.set_alpha(val)
        elif key == "targetAngle": lk.set_target(val)
        elif key == "maxSafeTilt": lk.set_tilt(val)

    def update_tab(self):
        if not self.app.link:
            return

        for spec in PARAM_SPECS:
            val = self.app.link.fw.get(spec.key)
            if val is not None and spec.key in self.sliders:
                self.sliders[spec.key].sync_from_external(float(val))

        po_val = self.app.link.fw.get("pos_offset")
        if po_val is not None:
            self.pos_slider.sync_from_external(float(po_val))
            revs = float(po_val) / 313.0
            self.rot_label.config(text=f"Rotations: {revs:+.2f} rev (313 ticks/rev)")

        tr_val = self.app.link.fw.get("turn_bias")
        if tr_val is not None:
            self.turn_slider.sync_from_external(float(tr_val))

        snap = self.app.link.snapshot()
        if snap["t"]:
            n         = min(len(snap["pitch"]), self._plot_max)
            x         = list(range(n))
            pitch     = snap["pitch"][-n:]
            pid_out   = snap["pid_out"][-n:]
            vel       = snap["vel"][-n:]
            tilt_bias = snap["tilt_bias"][-n:]
            pos_err   = snap["pos_err"][-n:]
            target    = self.app.link.fw.get("targetAngle", 0.0)

            self.pitch_line.set_data(x, pitch)
            self.target_line.set_data(x, [float(target)] * n)
            self.tiltbias_line.set_data(x, tilt_bias)
            self.pid_line.set_data(x, pid_out)
            self.vel_line.set_data(x, vel)
            self.pos_err_line.set_data(x, pos_err)

            self.ax.set_xlim(0, max(1, n - 1))
            self.ax.set_ylim(-15, 15)
            self.ax2.set_ylim(-500, 500)
            self.canvas.draw_idle()

        lines = self.app.link.recent_lines(50)
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert(tk.END, "\n".join(lines))
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)


# ---------------------------------------------------------------------------
# TAB 2: Leg Twin & Health
# ---------------------------------------------------------------------------
class LegTwinTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app            = app
        self.last_send_time = 0
        self.SEND_INTERVAL  = 0.05
        self.torque_state   = True
        self._build_ui()

    def _build_ui(self):
        self.columnconfigure(0, weight=2)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.fig.patch.set_facecolor("#0A192F")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-150, 300)
        self.ax.set_ylim(-200, 50)
        self.ax.set_facecolor("#050B14")
        
        self.ax.tick_params(colors="#E6E6E6")
        for spine in self.ax.spines.values():
            spine.set_color("#E6E6E6")

        self.leg1_arts = self._make_leg_artists("#ff6b6b", "#e0e0e0")
        self.leg2_arts = self._make_leg_artists("#da77f2", "#a5d8ff")

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="nsew")

        pos_frame = ttk.LabelFrame(right, text="Inverse Kinematics")
        pos_frame.pack(fill=tk.X, pady=5)

        self.sliders = {}
        for spec in IK_PARAM_SPECS:
            ctrl = CoarseFineSlider(pos_frame, spec, spec.start_val, self._on_ik_change)
            ctrl.pack(fill=tk.X, pady=4, padx=5)
            self.sliders[spec.key] = ctrl

        options_frame = ttk.Frame(pos_frame)
        options_frame.pack(fill=tk.X, pady=5)

        self.mirror_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(options_frame, text="Mirror Leg2 to Leg1", variable=self.mirror_var,
                        command=lambda: self._on_ik_change("mirror", 0.0)).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(options_frame, text="Torque ON/OFF",
                   command=self.toggle_torque).pack(side=tk.LEFT, padx=5)

        ttk.Button(pos_frame, text="Send Pose to Servos (Motors OFF)",
                   command=self.send_manual_pose).pack(fill=tk.X, padx=5, pady=5)

        trq_frame = ttk.LabelFrame(right, text="Dynamic Compliance & Torque")
        trq_frame.pack(fill=tk.X, pady=5)

        self.cmd_sliders = {}
        for spec in CMD_PARAM_SPECS:
            ctrl = CoarseFineSlider(trq_frame, spec, spec.start_val, self._on_cmd_change)
            ctrl.pack(fill=tk.X, pady=4, padx=5)
            self.cmd_sliders[spec.key] = ctrl

        health_frame = ttk.LabelFrame(right, text="Live Servo Health")
        health_frame.pack(fill=tk.BOTH, expand=True, pady=5)

        self.health_labels = {}
        for sid, name in [(6, "Leg1 L"), (14, "Leg1 R"), (0, "Leg2 L"), (1, "Leg2 R")]:
            lbl = tk.Label(health_frame,
                           text=f"ID {sid} ({name}): --°C  |  Load: --%",
                           font=("Consolas", 11, "bold"), bg="#eeeeee", fg="black", pady=8)
            lbl.pack(fill=tk.X, pady=2, padx=5)
            self.health_labels[sid] = lbl

        self._on_ik_change("init", 0.0)

    def _make_leg_artists(self, c_femur, c_tibia):
        lf, = self.ax.plot([], [], "o-", color=c_femur, lw=6)
        rf, = self.ax.plot([], [], "o-", color=c_femur, lw=6)
        lt, = self.ax.plot([], [], "o-", color=c_tibia, lw=5)
        rt, = self.ax.plot([], [], "o-", color=c_tibia, lw=5)
        return lf, rf, lt, rt

    def send_manual_pose(self):
        if not (self.app.link and self.app.link.ser):
            messagebox.showwarning("Not Connected", "Please connect to the serial port first.")
            return
        positions, _, _ = self._calculate_current_positions()
        if not positions:
            messagebox.showerror("IK Error", "Current leg positions are out of reach / invalid!")
            return
        for sid, pos in positions.items():
            self.app.link.send_leg_position(sid, pos)
        self.app.status_var.set("Manual pose sent to servos!")

    def toggle_torque(self):
        if not (self.app.link and self.app.link.ser):
            messagebox.showwarning("Not Connected", "Please connect to the serial port first.")
            return
        self.torque_state = not self.torque_state
        self.app.link.send_torque_enable(int(self.torque_state))
        state_str = "ON" if self.torque_state else "OFF"
        self.app.status_var.set(f"Sent Torque Enable: {state_str}")

    def _calculate_current_positions(self):
        x1   = self.sliders["fx1"].get_value()
        y1   = self.sliders["fy1"].get_value()
        x2   = self.sliders["fx2"].get_value()
        y2   = self.sliders["fy2"].get_value()
        dist = self.sliders["dist"].get_value()
        lean = self.sliders["lean"].get_value()

        rad = math.radians(-lean)
        c, s = math.cos(rad), math.sin(rad)
        rx1 = x1 * c - y1 * s;  ry1 = x1 * s + y1 * c
        rx2 = x2 * c - y2 * s;  ry2 = x2 * s + y2 * c

        sol1 = tkin.solve_ik(rx1, ry1, 0.0)
        sol2 = tkin.solve_ik(rx2 + dist, ry2, dist)
        positions = {}

        if sol1:
            positions[tkin.LEG1_SERVO_L_ID] = tkin.map_angle_to_ax12(sol1["Angle_L"], is_left=True,  is_leg2=False)
            positions[tkin.LEG1_SERVO_R_ID] = tkin.map_angle_to_ax12(sol1["Angle_R"], is_left=False, is_leg2=False)
        if sol2:
            ikL2, ikR2 = sol2["Angle_L"], sol2["Angle_R"]
            if tkin.LEG2_INVERTED_MOUNT:
                ikL2, ikR2 = -ikR2, -ikL2
            positions[tkin.LEG2_SERVO_L_ID] = tkin.map_angle_to_ax12(ikL2, is_left=True,  is_leg2=True)
            positions[tkin.LEG2_SERVO_R_ID] = tkin.map_angle_to_ax12(ikR2, is_left=False, is_leg2=True)

        return positions, sol1, sol2

    def _on_ik_change(self, key, value):
        if self.mirror_var.get() and key != "init":
            if key == "fx1": self.sliders["fx2"].set_value(value, send=False)
            if key == "fy1": self.sliders["fy2"].set_value(value, send=False)

        x1   = self.sliders["fx1"].get_value()
        y1   = self.sliders["fy1"].get_value()
        x2   = self.sliders["fx2"].get_value()
        y2   = self.sliders["fy2"].get_value()
        dist = self.sliders["dist"].get_value()
        lean = self.sliders["lean"].get_value()

        rad = math.radians(-lean)
        c, s = math.cos(rad), math.sin(rad)
        rx1 = x1 * c - y1 * s;  ry1 = x1 * s + y1 * c
        rx2 = x2 * c - y2 * s;  ry2 = x2 * s + y2 * c

        sol1 = tkin.solve_ik(rx1, ry1, 0.0)
        sol2 = tkin.solve_ik(rx2 + dist, ry2, dist)

        if sol1:
            self.leg1_arts[0].set_data([tkin.SERVO_L[0], sol1["Knee_L"][0]], [tkin.SERVO_L[1], sol1["Knee_L"][1]])
            self.leg1_arts[1].set_data([tkin.SERVO_R[0], sol1["Knee_R"][0]], [tkin.SERVO_R[1], sol1["Knee_R"][1]])
            self.leg1_arts[2].set_data([sol1["Knee_L"][0], rx1], [sol1["Knee_L"][1], ry1])
            self.leg1_arts[3].set_data([sol1["Knee_R"][0], rx1], [sol1["Knee_R"][1], ry1])

        if sol2:
            lx = tkin.SERVO_L[0] + dist
            rx = tkin.SERVO_R[0] + dist
            self.leg2_arts[0].set_data([lx, sol2["Knee_L"][0]], [tkin.SERVO_L[1], sol2["Knee_L"][1]])
            self.leg2_arts[1].set_data([rx, sol2["Knee_R"][0]], [tkin.SERVO_R[1], sol2["Knee_R"][1]])
            self.leg2_arts[2].set_data([sol2["Knee_L"][0], rx2 + dist], [sol2["Knee_L"][1], ry2])
            self.leg2_arts[3].set_data([sol2["Knee_R"][0], rx2 + dist], [sol2["Knee_R"][1], ry2])

        self.canvas.draw_idle()

        now = time.time()
        if self.app.link and self.app.link.ser and (now - self.last_send_time > self.SEND_INTERVAL):
            if key in ("fx1", "fy1", "init"):
                self.app.link.send_ik1(x1, y1)
                if self.mirror_var.get():
                    self.app.link.send_ik2(x2, y2)
            if key in ("fx2", "fy2", "init"):
                self.app.link.send_ik2(x2, y2)
            if key in ("dist", "init"):
                self.app.link.send_ikd(dist)
            if key in ("lean", "init"):
                self.app.link.send_ikl(lean)
            if key == "mirror":
                self.app.link.send_ik1(x1, y1)
                self.app.link.send_ik2(x2, y2)
                self.app.link.send_ikd(dist)
                self.app.link.send_ikl(lean)
            self.last_send_time = now

    def _on_cmd_change(self, key, value):
        if not self.app.link:
            return
        if key == "trq_limit":
            for sid in [6, 14, 0, 1]:
                self.app.link.send_torque_limit(sid, value)
        elif key in ("cmp_margin", "cmp_slope"):
            margin = self.cmd_sliders["cmp_margin"].get_value()
            slope  = self.cmd_sliders["cmp_slope"].get_value()
            for sid in [6, 14, 0, 1]:
                self.app.link.send_compliance(sid, margin, slope)

    def update_tab(self):
        if not self.app.link:
            return
        health_data = self.app.link.get_servo_health()
        names = {6: "Leg1 L", 14: "Leg1 R", 0: "Leg2 L", 1: "Leg2 R"}

        for sid, data in health_data.items():
            if sid not in self.health_labels:
                continue
            temp = data["temp"]
            load = data["load"]
            lbl  = self.health_labels[sid]
            lbl.config(text=f"ID {sid} ({names[sid]}): {temp}°C  |  Load: {load:.1f}%")
            if temp >= 65:
                lbl.config(bg="#ff3333", fg="white")
            elif temp >= 55:
                lbl.config(bg="#ffaa00", fg="black")
            else:
                lbl.config(bg="#eeeeee", fg="black")


# ---------------------------------------------------------------------------
# Main Application
# ---------------------------------------------------------------------------
class BipedPosTunerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Single-Position Balancing & Target Tuner — mcu_pos_wireless")
        self.geometry("1280x900")
        self.link = None

        self._apply_theme()

        self.port_var         = tk.StringVar(value="COM13")
        self.status_var       = tk.StringVar(value="Disconnected")
        self.motor_var        = tk.StringVar(value="Motors: OFF")
        self.cutoff_var       = tk.StringVar(value="Safety: clear")
        self.pitch_var        = tk.StringVar(value="Angle: --°")
        self.offset_var       = tk.StringVar(value="Offset: --")
        self.offset_entry_var = tk.StringVar(value="0.0")

        self._build_header()

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.tab_tuner = BalanceTunerTab(self.notebook, self)
        self.tab_legs  = LegTwinTab(self.notebook, self)

        self.notebook.add(self.tab_tuner, text="1. Position & Balance Tuner")
        self.notebook.add(self.tab_legs,  text="2. Kinematics & Health")

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._poll)

    def _build_header(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.port_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="Connect",       command=self.connect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Disconnect",    command=self.disconnect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Motors On/Off", command=self.toggle_motors).pack(side=tk.LEFT, padx=15)
        ttk.Button(top, text="Safety Reset",  command=self.safety_reset).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Calibrate IMU", command=self.calibrate).pack(side=tk.LEFT, padx=2)

        calib_frame = ttk.Frame(top)
        calib_frame.pack(side=tk.LEFT, padx=10)
        ttk.Label(calib_frame, textvariable=self.offset_var, width=12).pack(side=tk.LEFT)
        ttk.Entry(calib_frame, textvariable=self.offset_entry_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Button(calib_frame, text="Set Offset", command=self.set_manual_offset).pack(side=tk.LEFT)

        ttk.Button(top, text="Reset Integral", command=self.reset_integral).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Save Params",    command=self.save_params).pack(side=tk.LEFT, padx=10)

        ttk.Label(top, textvariable=self.cutoff_var,
                  font=("Consolas", 10, "bold"), foreground="#ff4d6d").pack(side=tk.RIGHT, padx=10)
        ttk.Label(top, textvariable=self.motor_var,
                  font=("Consolas", 10, "bold"), foreground="#FF8C00").pack(side=tk.RIGHT, padx=10)
        ttk.Label(top, textvariable=self.status_var, foreground="#E6E6E6").pack(side=tk.RIGHT, padx=10)
        ttk.Label(top, textvariable=self.pitch_var,
                  font=("Consolas", 12, "bold"), foreground="#4ea8de").pack(side=tk.RIGHT, padx=15)

    def _apply_theme(self):
        bg_color = "#0A192F"
        fg_color = "#E6E6E6"
        btn_bg = "#112240"
        orange_accent = "#FF8C00"
        black_color = "#050B14"

        self.configure(bg=bg_color)
        
        style = ttk.Style()
        style.theme_use('clam')
        
        style.configure('.',
            background=bg_color,
            foreground=fg_color,
            fieldbackground=black_color,
            troughcolor=black_color,
            selectbackground=orange_accent,
            selectforeground=black_color
        )
        
        style.configure('TFrame', background=bg_color)
        style.configure('TLabelframe', background=bg_color, foreground=orange_accent, bordercolor=orange_accent)
        style.configure('TLabelframe.Label', background=bg_color, foreground=orange_accent, font=('Helvetica', 10, 'bold'))
        style.configure('TLabel', background=bg_color, foreground=fg_color)
        style.configure('TButton', background=btn_bg, foreground=fg_color, bordercolor=black_color, lightcolor=btn_bg, darkcolor=btn_bg)
        style.map('TButton',
                  background=[('active', orange_accent), ('pressed', '#E67300')],
                  foreground=[('active', black_color)])
        
        style.configure('TEntry', fieldbackground=black_color, foreground=orange_accent, bordercolor=black_color, insertcolor=orange_accent)
        style.configure('TCheckbutton', background=bg_color, foreground=fg_color)
        style.map('TCheckbutton', background=[('active', bg_color)], foreground=[('active', orange_accent)])
        style.configure('TNotebook', background=bg_color, tabmargins=[2, 5, 2, 0])
        style.configure('TNotebook.Tab', background=btn_bg, foreground=fg_color, padding=[10, 2])
        style.map('TNotebook.Tab',
                  background=[('selected', orange_accent)],
                  foreground=[('selected', black_color)])

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
        self.pitch_var.set("Angle: --°")

    def toggle_motors(self):
        if self.link: self.link.toggle_motors()

    def safety_reset(self):
        if self.link: self.link.arm_cutoff_watch()

    def calibrate(self):
        if self.link: self.link.calibrate()

    def set_manual_offset(self):
        if self.link:
            try:
                val = float(self.offset_entry_var.get())
                self.link.set_offset(val)
                self.status_var.set(f"Sent manual offset: {val}")
            except ValueError:
                self.status_var.set("Invalid offset value")

    def reset_integral(self):
        if self.link: self.link.reset_integral()

    def save_params(self):
        profiles_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")
        os.makedirs(profiles_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename  = os.path.join(profiles_dir, f"params_{timestamp}.json")
        data = {}
        for key, slider in self.tab_tuner.sliders.items():
            data[key] = slider.get_value()
        for key, slider in self.tab_legs.sliders.items():
            data[key] = slider.get_value()
        for key, slider in self.tab_legs.cmd_sliders.items():
            data[key] = slider.get_value()
        data["pos_offset"] = self.tab_tuner.pos_slider.get_value()
        data["turn_bias"]  = self.tab_tuner.turn_slider.get_value()
        try:
            with open(filename, "w") as f:
                json.dump(data, f, indent=4)
            self.status_var.set(f"Saved to profiles/params_{timestamp}.json")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    def _poll(self):
        if self.link:
            self.motor_var.set(f"Motors: {'ON' if self.link.motors_on else 'OFF'}")
            self.cutoff_var.set("Safety: LATCHED!" if self.link.cutoff_since() else "Safety: clear")

            snap = self.link.snapshot()
            if snap and snap.get("pitch"):
                self.pitch_var.set(f"Angle: {snap['pitch'][-1]:+.2f}°")
            else:
                self.pitch_var.set("Angle: --°")

            offset_val = self.link.fw.get("pitchOffset")
            self.offset_var.set(f"Offset: {offset_val:.2f}" if offset_val is not None else "Offset: --")

            active_tab = self.notebook.index(self.notebook.select())
            if active_tab == 0:
                self.tab_tuner.update_tab()
            elif active_tab == 1:
                self.tab_legs.update_tab()

        self.after(100, self._poll)

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = BipedPosTunerApp()
    app.mainloop()
