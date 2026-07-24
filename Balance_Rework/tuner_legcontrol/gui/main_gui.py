"""
main_gui.py
The Unified Bipedal Tuner & Digital Twin GUI
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

# Your existing local modules
from serial_link import SerialLink
import twin_kinematics as tkin

# --- Parameter Specification Class ---
class ParamSpec:
    def __init__(self, key, label, coarse_min, coarse_max, coarse_step, fine_span, fine_step, digits, command_type, start_val):
        self.key = key
        self.label = label
        self.coarse_min = coarse_min
        self.coarse_max = coarse_max
        self.coarse_step = coarse_step
        self.fine_span = fine_span
        self.fine_step = fine_step
        self.digits = digits
        self.command_type = command_type
        self.start_val = start_val

# --- Balance Tuner Specs Verbatim from balance_tuner_gui.py ---
PARAM_SPECS = [
    ParamSpec("Kp", "Kp", 0.0, 200.0, 0.1, 5.0, 0.01, 3, "gain", 78.0),
    ParamSpec("Ki", "Ki", 0.0, 1000.0, 0.5, 1.0, 0.001, 4, "gain", 0.0),
    ParamSpec("Kd", "Kd", 0.0, 10.0, 0.1, 5.0, 0.01, 3, "gain", 0.0),
    ParamSpec("Kd_vel", "Kd_vel", 0.0, 10.0, 0.1, 1.0, 0.01, 3, "gain", 0.0),
    ParamSpec("alpha", "alpha", 0.80, 0.999, 0.001, 0.02, 0.0001, 4, "gain", 0.96),
    ParamSpec("targetAngle", "Target", -20.0, 20.0, 0.1, 5.0, 0.01, 3, "target", 0.0),
    ParamSpec("pitchOffset", "Offset", -20.0, 20.0, 0.1, 5.0, 0.01, 3, "offset", 0.0),
    ParamSpec("maxSafeTilt", "Tilt", 5.0, 50.0, 0.1, 5.0, 0.01, 2, "tilt", 25.0),
]

# --- Kinematics & Compliance Specs ---
IK_PARAM_SPECS = [
    ParamSpec("fx1", "Leg1 X", -100.0, 100.0, 1.0, 10.0, 0.1, 1, "ik", 0.0),
    ParamSpec("fy1", "Leg1 Y", -160.0, -20.0, 1.0, 10.0, 0.1, 1, "ik", -157.0),
    ParamSpec("fx2", "Leg2 X", -100.0, 100.0, 1.0, 10.0, 0.1, 1, "ik", 0.0),
    ParamSpec("fy2", "Leg2 Y", -160.0, -20.0, 1.0, 10.0, 0.1, 1, "ik", -157.0),
    ParamSpec("dist", "Leg Dist", 100.0, 250.0, 1.0, 20.0, 0.1, 1, "ik", 180.0),
]

CMD_PARAM_SPECS = [
    ParamSpec("trq_limit", "Torque Limit", 0.0, 1023.0, 1.0, 100.0, 1.0, 0, "cmd", 511.0),
    ParamSpec("cmp_margin", "Comp Margin", 0.0, 254.0, 1.0, 20.0, 1.0, 0, "cmd", 4.0),
    ParamSpec("cmp_slope", "Comp Slope", 0.0, 254.0, 1.0, 20.0, 1.0, 0, "cmd", 32.0),
]

# --- Coarse / Fine Precision Zoom Slider Class ---
class CoarseFineSlider(ttk.Frame):
    def __init__(self, master, spec: ParamSpec, initial_value: float, on_change_callback):
        super().__init__(master)
        self.spec = spec
        self.on_change_callback = on_change_callback
        self._debounce_id = None
        self._zoom = tk.BooleanVar(value=False)
        self._value = tk.DoubleVar(value=initial_value)
        self._entry_value = tk.StringVar(value=self._fmt_value(initial_value))
        self._last_sent = initial_value
        self._user_dragging = False
        self._entry_focused = False
        self._awaiting_echo = False

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=0)
        self.columnconfigure(2, weight=0)

        # Row 0: Title, Value label, Entry box
        self._title = ttk.Label(self, text=spec.label, font=("Helvetica", 9, "bold"))
        self._title.grid(row=0, column=0, sticky="w", pady=(2, 0))

        self._value_label = ttk.Label(self, text=self._fmt_value(initial_value))
        self._value_label.grid(row=0, column=1, sticky="e", pady=(2, 0))

        self._entry = ttk.Entry(self, textvariable=self._entry_value, width=8)
        self._entry.grid(row=0, column=2, sticky="e", padx=(5, 0), pady=(2, 0))
        self._entry.bind("<FocusIn>", self._on_entry_focus_in)
        self._entry.bind("<FocusOut>", self._on_entry_focus_out)
        self._entry.bind("<Return>", self._on_entry_commit)

        # Row 1: Scale, Zoom checkbox
        self._scale = tk.Scale(
            self,
            orient=tk.HORIZONTAL,
            showvalue=False,
            resolution=spec.coarse_step,
            from_=spec.coarse_min,
            to=spec.coarse_max,
            variable=self._value,
            command=self._on_change,
            length=180,
        )
        self._scale.grid(row=1, column=0, columnspan=2, sticky="ew", padx=(0, 5), pady=(0, 2))
        self._scale.bind("<ButtonPress-1>", self._on_press)
        self._scale.bind("<ButtonRelease-1>", self._on_release)

        self._zoom_box = ttk.Checkbutton(
            self,
            text="Zoom",
            variable=self._zoom,
            command=self._apply_zoom_mode,
        )
        self._zoom_box.grid(row=1, column=2, sticky="w", padx=(5, 0), pady=(0, 2))

        self._apply_zoom_mode()

    def _fmt_value(self, value: float) -> str:
        return f"{value:.{self.spec.digits}f}"

    def _quantize(self, value: float) -> float:
        step = self._scale.cget("resolution")
        if isinstance(step, str):
            step = float(step)
        if step <= 0:
            return value
        return round(value / step) * step

    def _clamp(self, value: float) -> float:
        lo = float(self._scale.cget("from"))
        hi = float(self._scale.cget("to"))
        return max(min(value, max(lo, hi)), min(lo, hi))

    def _on_press(self, _event):
        self._user_dragging = True

    def _on_release(self, _event):
        self._user_dragging = False
        self._send_now()

    def _on_entry_focus_in(self, _event):
        self._entry_focused = True

    def _on_entry_focus_out(self, _event):
        self._entry_focused = False

    def _on_entry_commit(self, _event):
        self.apply_entry_value()
        return "break"

    def _on_change(self, _raw_value):
        value = self._clamp(self._quantize(self._value.get()))
        if abs(value - self._value.get()) > 1e-12:
            self._value.set(value)
        self._value_label.config(text=self._fmt_value(value))
        if not self._entry_focused:
            self._entry_value.set(self._fmt_value(value))

        if self._debounce_id is not None:
            self.after_cancel(self._debounce_id)
        if self._user_dragging:
            self._debounce_id = self.after(150, self._send_now)

    def _apply_zoom_mode(self):
        current = float(self._value.get())
        if self._zoom.get():
            lo = current - self.spec.fine_span
            hi = current + self.spec.fine_span
            lo = max(lo, self.spec.coarse_min)
            hi = min(hi, self.spec.coarse_max)
            if hi - lo < self.spec.fine_step:
                hi = min(self.spec.coarse_max, lo + self.spec.fine_step)
            self._scale.configure(from_=lo, to=hi, resolution=self.spec.fine_step)
        else:
            self._scale.configure(
                from_=self.spec.coarse_min,
                to=self.spec.coarse_max,
                resolution=self.spec.coarse_step,
            )
        self._scale.set(self._clamp(current))
        self._value_label.config(text=self._fmt_value(float(self._value.get())))
        if not self._entry_focused:
            self._entry_value.set(self._fmt_value(float(self._value.get())))

    def set_value(self, value: float, send: bool = False):
        value = self._clamp(self._quantize(value))
        self._value.set(value)
        self._scale.set(value)
        self._value_label.config(text=self._fmt_value(value))
        if not self._entry_focused:
            self._entry_value.set(self._fmt_value(value))
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
        value = self._clamp(self._quantize(self._value.get()))
        self._value.set(value)
        self._scale.set(value)
        self._value_label.config(text=self._fmt_value(value))
        self._entry_value.set(self._fmt_value(value))
        if abs(value - self._last_sent) < 1e-9:
            return
        self._last_sent = value
        self._awaiting_echo = True
        self.on_change_callback(self.spec.key, value)

    def apply_entry_value(self):
        raw = self._entry_value.get().strip()
        if not raw:
            self._entry_value.set(self._fmt_value(self._value.get()))
            return
        try:
            value = float(raw)
        except ValueError:
            self._entry_value.set(self._fmt_value(self._value.get()))
            return
        self._value.set(value)
        self._scale.set(self._clamp(self._quantize(value)))
        self._send_now()

    def sync_from_external(self, value: float):
        if self._user_dragging or self._awaiting_echo or self._entry_focused:
            return
        value = self._clamp(self._quantize(value))
        if abs(value - self._value.get()) < 1e-9:
            return
        self._value.set(value)
        self._scale.set(value)
        self._value_label.config(text=self._fmt_value(value))
        self._entry_value.set(self._fmt_value(value))

    def mark_echo_received(self, value: float):
        value = self._clamp(self._quantize(value))
        if abs(value - self._last_sent) < 1e-9:
            self._awaiting_echo = False


# --- TAB 1: BALANCE TUNER ---
class BalanceTunerTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self._plot_max = 250
        self._build_ui()

    def _build_ui(self):
        self.columnconfigure(0, weight=2)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        # Left: Live Plot
        left = ttk.LabelFrame(self, text="Live Telemetry")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        
        self.fig = Figure(figsize=(6, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax2 = self.ax.twinx()
        self.ax.grid(True, alpha=0.25)
        
        self.pitch_line, = self.ax.plot([], [], color="#1f77b4", label="pitch")
        self.target_line, = self.ax.plot([], [], color="#2ca02c", ls="--", label="target")
        self.pid_line, = self.ax2.plot([], [], color="#ff7f0e", alpha=0.9, label="pid_out")
        self.vel_line, = self.ax2.plot([], [], color="#9467bd", alpha=0.7, label="vel")
        self.ax.legend(loc="upper left")

        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Right: Sliders & Log
        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="nsew")

        controls = ttk.LabelFrame(right, text="Tunable Parameters")
        controls.pack(fill=tk.X, pady=(0, 10))
        
        self.sliders = {}
        for spec in PARAM_SPECS:
            ctrl = CoarseFineSlider(controls, spec, spec.start_val, self._on_slider)
            ctrl.pack(fill=tk.X, pady=4, padx=5)
            self.sliders[spec.key] = ctrl

        log_frame = ttk.LabelFrame(right, text="Serial Monitor")
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=10, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _on_slider(self, key, val):
        if self.app.link and self.app.link.ser:
            if key == "Kp": self.app.link.set_kp(val)
            elif key == "Ki": self.app.link.set_ki(val)
            elif key == "Kd": self.app.link.set_kd(val)
            elif key == "Kd_vel": self.app.link.set_kd_vel(val)
            elif key == "alpha": self.app.link.set_alpha(val)
            elif key == "targetAngle": self.app.link.set_target(val)
            elif key == "pitchOffset": self.app.link.set_offset(val)
            elif key == "maxSafeTilt": self.app.link.set_tilt(val)

    def update_tab(self):
        if not self.app.link: return
        
        # Update Slider Values if external echo is received
        for spec in PARAM_SPECS:
            val = self.app.link.fw.get(spec.key)
            if val is not None and spec.key in self.sliders:
                self.sliders[spec.key].sync_from_external(float(val))

        # Update Plot
        snap = self.app.link.snapshot()
        if snap["t"]:
            n = min(len(snap["pitch"]), self._plot_max)
            x = list(range(n))
            pitch = snap["pitch"][-n:]
            pid_out = snap["pid_out"][-n:]
            vel = snap["vel"][-n:]
            target = self.app.link.fw.get("targetAngle", 0.0)

            self.pitch_line.set_data(x, pitch)
            self.target_line.set_data(x, [float(target)]*n)
            self.pid_line.set_data(x, pid_out)
            self.vel_line.set_data(x, vel)

            self.ax.set_xlim(0, max(1, n-1))
            self.ax.set_ylim(-15, 15)
            self.ax2.set_ylim(-200, 200)
            self.canvas.draw_idle()

        # Update Log
        lines = self.app.link.recent_lines(50)
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert(tk.END, "\n".join(lines))
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)


# --- TAB 2: LEG TWIN & HEALTH ---
class LegTwinTab(ttk.Frame):
    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.last_send_time = 0
        self.SEND_INTERVAL = 0.05 # 20Hz Throttle
        self._build_ui()

    def _build_ui(self):
        self.columnconfigure(0, weight=2)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        # Left: Matplotlib Canvas (The Digital Twin)
        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        
        self.fig = Figure(figsize=(6, 5), dpi=100)
        self.fig.patch.set_facecolor("#1a1a2e")
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-150, 300)
        self.ax.set_ylim(-200, 50)
        self.ax.set_facecolor("#111122")
        
        self.leg1_arts = self._make_leg_artists("#ff6b6b", "#e0e0e0")
        self.leg2_arts = self._make_leg_artists("#da77f2", "#a5d8ff")
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=left)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Right: Controls & Health
        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="nsew")

        # --- Position Controls ---
        pos_frame = ttk.LabelFrame(right, text="Inverse Kinematics")
        pos_frame.pack(fill=tk.X, pady=5)
        
        self.sliders = {}
        for spec in IK_PARAM_SPECS:
            ctrl = CoarseFineSlider(pos_frame, spec, spec.start_val, self._on_ik_change)
            ctrl.pack(fill=tk.X, pady=4, padx=5)
            self.sliders[spec.key] = ctrl

        self.mirror_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(pos_frame, text="Mirror Leg2 to Leg1", variable=self.mirror_var, 
                        command=lambda: self._on_ik_change("mirror", 0.0)).pack(pady=5)

        # --- Manual Pose Push Button (For when motors are OFF) ---
        ttk.Button(pos_frame, text="Send Pose to Servos (Motors OFF)", command=self.send_manual_pose).pack(fill=tk.X, padx=5, pady=5)

        # --- Torque & Compliance Controls ---
        trq_frame = ttk.LabelFrame(right, text="Dynamic Compliance & Torque")
        trq_frame.pack(fill=tk.X, pady=5)
        
        self.cmd_sliders = {}
        for spec in CMD_PARAM_SPECS:
            ctrl = CoarseFineSlider(trq_frame, spec, spec.start_val, self._on_cmd_change)
            ctrl.pack(fill=tk.X, pady=4, padx=5)
            self.cmd_sliders[spec.key] = ctrl

        # --- Health Warning Dashboard ---
        health_frame = ttk.LabelFrame(right, text="Live Servo Health")
        health_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.health_labels = {}
        for sid, name in [(6, "Leg1 L"), (14, "Leg1 R"), (0, "Leg2 L"), (1, "Leg2 R")]:
            lbl = tk.Label(health_frame, text=f"ID {sid} ({name}): --°C  |  Load: --%", 
                           font=("Consolas", 11, "bold"), bg="#eeeeee", fg="black", pady=8)
            lbl.pack(fill=tk.X, pady=2, padx=5)
            self.health_labels[sid] = lbl

        self._on_ik_change("init", 0.0) # Init draw

    def _make_leg_artists(self, c_femur, c_tibia):
        lf, = self.ax.plot([], [], "o-", color=c_femur, lw=6)
        rf, = self.ax.plot([], [], "o-", color=c_femur, lw=6)
        lt, = self.ax.plot([], [], "o-", color=c_tibia, lw=5)
        rt, = self.ax.plot([], [], "o-", color=c_tibia, lw=5)
        return lf, rf, lt, rt

    def _calculate_current_positions(self):
        """Helper to resolve current slider coordinates into AX-12 servo targets."""
        x1 = self.sliders["fx1"].get_value()
        y1 = self.sliders["fy1"].get_value()
        x2 = self.sliders["fx2"].get_value()
        y2 = self.sliders["fy2"].get_value()
        dist = self.sliders["dist"].get_value()

        sol1 = tkin.solve_ik(x1, y1, 0.0)
        sol2 = tkin.solve_ik(x2 + dist, y2, dist)
        positions_to_send = {}

        if sol1:
            p6 = tkin.map_angle_to_ax12(sol1["Angle_L"], is_left=True, is_leg2=False)
            p14 = tkin.map_angle_to_ax12(sol1["Angle_R"], is_left=False, is_leg2=False)
            positions_to_send[tkin.LEG1_SERVO_L_ID] = p6
            positions_to_send[tkin.LEG1_SERVO_R_ID] = p14

        if sol2:
            ikL2, ikR2 = sol2["Angle_L"], sol2["Angle_R"]
            if tkin.LEG2_INVERTED_MOUNT:
                ikL2, ikR2 = -ikR2, -ikL2
            p0 = tkin.map_angle_to_ax12(ikL2, is_left=True, is_leg2=True)
            p1 = tkin.map_angle_to_ax12(ikR2, is_left=False, is_leg2=True)
            positions_to_send[tkin.LEG2_SERVO_L_ID] = p0
            positions_to_send[tkin.LEG2_SERVO_R_ID] = p1

        return positions_to_send, sol1, sol2

    def send_manual_pose(self):
        """Dispatches current IK slider positions explicitly via button click (works when motors OFF)."""
        if not self.app.link or not self.app.link.ser:
            messagebox.showwarning("Not Connected", "Please connect to the serial port first.")
            return
        
        positions, _, _ = self._calculate_current_positions()
        if not positions:
            messagebox.showerror("IK Error", "Current leg positions are out of reach / invalid!")
            return

        for sid, pos in positions.items():
            self.app.link.send_leg_position(sid, pos)
        
        # Brief visual feedback in log or status
        self.app.status_var.set("Manual pose sent to servos!")

    def _on_ik_change(self, key, value):
        if self.mirror_var.get() and key != "init":
            if key == "fx1":
                self.sliders["fx2"].set_value(-value, send=False)
            elif key == "fy1":
                self.sliders["fy2"].set_value(value, send=False)

        x1 = self.sliders["fx1"].get_value()
        y1 = self.sliders["fy1"].get_value()
        x2 = self.sliders["fx2"].get_value()
        y2 = self.sliders["fy2"].get_value()
        dist = self.sliders["dist"].get_value()

        # Solve IK using your existing twin_kinematics library
        sol1 = tkin.solve_ik(x1, y1, 0.0)
        sol2 = tkin.solve_ik(x2 + dist, y2, dist)

        positions_to_send = {}

        # Draw Leg 1
        if sol1:
            self.leg1_arts[0].set_data([tkin.SERVO_L[0], sol1["Knee_L"][0]], [tkin.SERVO_L[1], sol1["Knee_L"][1]])
            self.leg1_arts[1].set_data([tkin.SERVO_R[0], sol1["Knee_R"][0]], [tkin.SERVO_R[1], sol1["Knee_R"][1]])
            self.leg1_arts[2].set_data([sol1["Knee_L"][0], x1], [sol1["Knee_L"][1], y1])
            self.leg1_arts[3].set_data([sol1["Knee_R"][0], x1], [sol1["Knee_R"][1], y1])
            
            p6 = tkin.map_angle_to_ax12(sol1["Angle_L"], is_left=True, is_leg2=False)
            p14 = tkin.map_angle_to_ax12(sol1["Angle_R"], is_left=False, is_leg2=False)
            positions_to_send[tkin.LEG1_SERVO_L_ID] = p6
            positions_to_send[tkin.LEG1_SERVO_R_ID] = p14

        # Draw Leg 2
        if sol2:
            lx, rx = tkin.SERVO_L[0] + dist, tkin.SERVO_R[0] + dist
            self.leg2_arts[0].set_data([lx, sol2["Knee_L"][0]], [tkin.SERVO_L[1], sol2["Knee_L"][1]])
            self.leg2_arts[1].set_data([rx, sol2["Knee_R"][0]], [tkin.SERVO_R[1], sol2["Knee_R"][1]])
            self.leg2_arts[2].set_data([sol2["Knee_L"][0], x2 + dist], [sol2["Knee_L"][1], y2])
            self.leg2_arts[3].set_data([sol2["Knee_R"][0], x2 + dist], [sol2["Knee_R"][1], y2])
            
            ikL2, ikR2 = sol2["Angle_L"], sol2["Angle_R"]
            if tkin.LEG2_INVERTED_MOUNT:
                ikL2, ikR2 = -ikR2, -ikL2
                
            p0 = tkin.map_angle_to_ax12(ikL2, is_left=True, is_leg2=True)
            p1 = tkin.map_angle_to_ax12(ikR2, is_left=False, is_leg2=True)
            positions_to_send[tkin.LEG2_SERVO_L_ID] = p0
            positions_to_send[tkin.LEG2_SERVO_R_ID] = p1

        self.canvas.draw_idle()

        # Send IK to Firmware (Throttled automatically when connected)
        now = time.time()
        if self.app.link and self.app.link.ser and (now - self.last_send_time > self.SEND_INTERVAL):
            for sid, pos in positions_to_send.items():
                self.app.link.send_leg_position(sid, pos)
            self.last_send_time = now

    def _on_cmd_change(self, key, value):
        if not self.app.link: return
        if key == "trq_limit":
            for sid in [6, 14, 0, 1]:
                self.app.link.send_torque_limit(sid, value)
        elif key in ["cmp_margin", "cmp_slope"]:
            margin = self.cmd_sliders["cmp_margin"].get_value()
            slope = self.cmd_sliders["cmp_slope"].get_value()
            for sid in [6, 14, 0, 1]:
                self.app.link.send_compliance(sid, margin, slope)

    def update_tab(self):
        if not self.app.link: return
        
        # Update Health Warnings
        health_data = self.app.link.get_servo_health()
        names = {6: "Leg1 L", 14: "Leg1 R", 0: "Leg2 L", 1: "Leg2 R"}
        
        for sid, data in health_data.items():
            if sid not in self.health_labels: continue
            
            temp = data["temp"]
            load = data["load"]
            lbl = self.health_labels[sid]
            
            lbl.config(text=f"ID {sid} ({names[sid]}): {temp}°C  |  Load: {load:.1f}%")
            
            # Flash RED if temperature crosses 65C hardware danger zone
            if temp >= 65:
                lbl.config(bg="#ff3333", fg="white")
            elif temp >= 55:
                lbl.config(bg="#ffaa00", fg="black") # Orange warning
            else:
                lbl.config(bg="#eeeeee", fg="black") # Normal


# --- MAIN APPLICATION ROOT ---
class BipedTunerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Unified Biped Tuner & Digital Twin")
        self.geometry("1200x850")
        self.link = None
        
        self.port_var = tk.StringVar(value="COM3")
        self.status_var = tk.StringVar(value="Disconnected")
        self.motor_var = tk.StringVar(value="Motors: OFF")
        self.cutoff_var = tk.StringVar(value="Safety: clear")
        
        # Global tracking variable for the falling angle
        self.pitch_var = tk.StringVar(value="Angle: --°")

        self._build_header()
        
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.tab_tuner = BalanceTunerTab(self.notebook, self)
        self.tab_legs = LegTwinTab(self.notebook, self)
        
        self.notebook.add(self.tab_tuner, text="1. Balance Tuner")
        self.notebook.add(self.tab_legs, text="2. Kinematics & Health")

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._poll)

    def _build_header(self):
        top = ttk.Frame(self, padding=10)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.port_var, width=10).pack(side=tk.LEFT, padx=5)
        ttk.Button(top, text="Connect", command=self.connect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Motors On/Off", command=self.toggle_motors).pack(side=tk.LEFT, padx=15)
        ttk.Button(top, text="Safety Reset", command=self.safety_reset).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Calibrate IMU", command=self.calibrate).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Reset Integral", command=self.reset_integral).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Save Params", command=self.save_params).pack(side=tk.LEFT, padx=10)
        
        # Right side indicators
        ttk.Label(top, textvariable=self.cutoff_var, font=("Consolas", 10, "bold"), 
                  foreground="red").pack(side=tk.RIGHT, padx=10)
        ttk.Label(top, textvariable=self.motor_var, font=("Consolas", 10, "bold")).pack(side=tk.RIGHT, padx=10)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.RIGHT, padx=10)
        
        # Live Angle Indicator pinned to the header
        ttk.Label(top, textvariable=self.pitch_var, font=("Consolas", 12, "bold"), 
                  foreground="#1f77b4").pack(side=tk.RIGHT, padx=15)

    def connect(self):
        if self.link: return
        port = self.port_var.get()
        try:
            self.link = SerialLink(port, baud=115200)
            self.link.connect()
            self.status_var.set(f"Connected to {port}")
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

    def reset_integral(self):
        if self.link: self.link.reset_integral()

    def save_params(self):
        profiles_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")
        os.makedirs(profiles_dir, exist_ok=True)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(profiles_dir, f"params_{timestamp}.json")
        
        data = {}
        for key, slider in self.tab_tuner.sliders.items():
            data[key] = slider.get_value()
            
        for key, slider in self.tab_legs.sliders.items():
            data[key] = slider.get_value()
            
        for key, slider in self.tab_legs.cmd_sliders.items():
            data[key] = slider.get_value()
            
        try:
            with open(filename, "w") as f:
                json.dump(data, f, indent=4)
            self.status_var.set(f"Saved to profiles/params_{timestamp}.json")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    def _poll(self):
        if self.link:
            # Update Header Variables
            self.motor_var.set(f"Motors: {'ON' if self.link.motors_on else 'OFF'}")
            self.cutoff_var.set("Safety: LATCHED!" if self.link.cutoff_since() else "Safety: clear")
            
            # Extract the latest falling angle (pitch) from the data snapshot
            snap = self.link.snapshot()
            if snap and snap.get("pitch"):
                current_pitch = snap["pitch"][-1]
                self.pitch_var.set(f"Angle: {current_pitch:+.2f}°")
            else:
                self.pitch_var.set("Angle: --°")
            
            # Delegate updates to the active tab to save CPU
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
    app = BipedTunerApp()
    app.mainloop()