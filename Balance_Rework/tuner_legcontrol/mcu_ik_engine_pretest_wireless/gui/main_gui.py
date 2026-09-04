"""
main_gui.py
Single-Loop Wireless Balance Tuner — 3DR telemetry edition.

Deliberately minimal control loop: ONE balance PID (Kp, Ki, Kd) plus setpoint /
filter / safety / calibration / auto-trim / crouch. No cascade, no full IK, no
RC. Laid out like the RC_mcu_IK_wireless GUI: a shared header (connection,
arm/disarm, calibration, save) above a two-tab Notebook —
  1. Balance Tuner        — live telemetry plot, tuning sliders, serial monitor
  2. Kinematics & Health   — live servo health, raw per-servo position control
Pairs with the single-loop balancing firmware in ../firmware.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import time
import json
import os

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from serial_link import SerialLink


# ---------------------------------------------------------------------------
# Parameter specification
# ---------------------------------------------------------------------------
class ParamSpec:
    def __init__(self, key, label, coarse_min, coarse_max, coarse_step,
                 fine_span, fine_step, digits, start_val):
        self.key         = key
        self.label       = label
        self.coarse_min  = coarse_min
        self.coarse_max  = coarse_max
        self.coarse_step = coarse_step
        self.fine_span   = fine_span
        self.fine_step   = fine_step
        self.digits      = digits
        self.start_val   = start_val


# The single control loop is Kp/Ki/Kd. Target is the setpoint the loop holds to;
# alpha is IMU fusion; Tilt is the safety cutoff; Trim Gain/Crouch are the two
# opt-in add-ons (auto-trim, crouch IK) — none of these are extra control loops.
PARAM_SPECS = [
    ParamSpec("Kp",          "Kp",     0.0,  200.0, 0.1,   5.0,  0.01,   3,  78.0),
    ParamSpec("Ki",          "Ki",     0.0, 1000.0, 0.5,   1.0,  0.001,  4,   0.0),
    ParamSpec("Kd",          "Kd",     0.0,   50.0, 0.1,  10.0,  0.01,   3,   0.0),
    ParamSpec("targetAngle", "Target",-20.0,  20.0, 0.1,   5.0,  0.01,   3,   0.0),
    ParamSpec("alpha",       "alpha",  0.80, 0.999, 0.001, 0.02, 0.0001, 4,  0.96),
    ParamSpec("maxSafeTilt", "Max Tilt",5.0,  50.0, 0.1,   5.0,  0.01,   2,  25.0),
    # Auto-trim: gain of the opt-in drift-cancelling bias (see firmware AUTO-TRIM
    # block). Range derived from the 100Hz loop + 15deg clamp: below ~0.0033 a
    # mild drift (~10 counts/s) takes over 30s to produce 1deg of correction;
    # above ~0.03 a single 10ms tick can move the bias >1% of the whole clamp,
    # i.e. it stops acting like a slow trim and starts acting like a step.
    ParamSpec("Ki_trim",     "Trim Gain",0.0, 0.03, 0.001, 0.003,0.0001, 5, 0.001),
    # Crouch bar: 0 = standing (original calibrated pose), + = crouched (mm the
    # foot target is pulled toward the hip). Works with motorsEnabled off — see
    # firmware CROUCH IK block. Capped well short of the 5-bar's reach-circle
    # singularity (see leg-ik-and-servos skill). Lives on the main tuner screen
    # alongside the balance controls, not on the Kinematics & Health tab.
    ParamSpec("crouchOffset","Crouch",   0.0, 80.0, 1.0,   10.0, 0.1,    1,   0.0),
]


# ---------------------------------------------------------------------------
# Coarse/Fine precision zoom slider (unchanged behaviour)
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
        self._echo_deadline     = 0.0

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
            # 300 ms, up from 150 ms. Each drag step that survives the debounce
            # costs an uplink transmission on a half-duplex radio, and a burst
            # of them queues for seconds. _on_release() still fires _send_now()
            # immediately, so the final value is never delayed by this.
            self._debounce_id = self.after(300, self._send_now)

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
        # Self-healing deadline: mark_echo_received() only clears this flag on a
        # matching ack, and acks DO get dropped on this radio. Without a timeout
        # the flag latches True forever and sync_from_external() goes dead, so the
        # slider stops tracking firmware for the rest of the session.
        self._echo_deadline = time.time() + 2.0
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
        if self._awaiting_echo and time.time() > self._echo_deadline:
            self._awaiting_echo = False        # ack lost — stop ignoring firmware
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
# TAB 1: Balance Tuner — plot, tuning sliders (incl. Crouch), serial monitor
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

        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=3, minsize=280)
        left.rowconfigure(1, weight=1)

        plot_frame = ttk.LabelFrame(left, text="Live Telemetry")
        plot_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 8))

        self.fig = Figure(figsize=(6, 4), dpi=100)
        self.ax  = self.fig.add_subplot(111)
        self.ax2 = self.ax.twinx()
        self.ax.grid(True, alpha=0.25)
        self.pitch_line,  = self.ax.plot([], [], color="#1f77b4", label="pitch")
        self.target_line, = self.ax.plot([], [], color="#2ca02c", ls="--", label="target")
        self.trim_line,   = self.ax.plot([], [], color="#d62728", ls=":", label="trim")
        self.pid_line,    = self.ax2.plot([], [], color="#ff7f0e", alpha=0.9, label="pid_out")
        self.vel_line,    = self.ax2.plot([], [], color="#9467bd", alpha=0.6, label="vel")
        self.ax.set_ylabel("angle (°)")
        self.ax2.set_ylabel("pid / vel")
        self.ax.legend(loc="upper left", fontsize=8)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        log_frame = ttk.LabelFrame(left, text="Serial Monitor")
        log_frame.grid(row=1, column=0, sticky="nsew")
        self.log_text = tk.Text(log_frame, height=8, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # -- tuning: balance PID + auto-trim gain + crouch, all on this screen --
        tuning_frame = ttk.LabelFrame(self, text="Tuning  (balance PID + auto-trim + crouch)")
        tuning_frame.grid(row=0, column=1, sticky="nsew")
        self.sliders = {}
        for spec in PARAM_SPECS:
            ctrl = CoarseFineSlider(tuning_frame, spec, spec.start_val, self._on_slider)
            ctrl.pack(fill=tk.X, pady=4, padx=5)
            self.sliders[spec.key] = ctrl

    def _on_slider(self, key, val):
        if not (self.app.link and self.app.link.ser):
            return
        lk = self.app.link
        if   key == "Kp":          lk.set_kp(val)
        elif key == "Ki":          lk.set_ki(val)
        elif key == "Kd":          lk.set_kd(val)
        elif key == "targetAngle": lk.set_target(val)
        elif key == "alpha":       lk.set_alpha(val)
        elif key == "maxSafeTilt": lk.set_tilt(val)
        elif key == "Ki_trim":     lk.set_trim_gain(val)
        elif key == "crouchOffset":lk.set_crouch(val)

    def update_tab(self):
        app = self.app
        if not app.link:
            return

        for spec in PARAM_SPECS:
            val = app.link.fw.get(spec.key)
            if val is not None and spec.key in self.sliders:
                # Clear the pending-echo flag first: the firmware's "Updated ->"
                # ack IS the confirmation, and until this is called the slider
                # ignores every firmware value it sees.
                self.sliders[spec.key].mark_echo_received(float(val))
                self.sliders[spec.key].sync_from_external(float(val))

        snap = app.link.snapshot()
        if snap and snap.get("pitch"):
            app.pitch_var.set(f"Angle: {snap['pitch'][-1]:+.2f}°")
            n      = min(len(snap["pitch"]), self._plot_max)
            x      = list(range(n))
            target = app.link.fw.get("targetAngle", 0.0)
            self.pitch_line.set_data(x, snap["pitch"][-n:])
            self.target_line.set_data(x, [float(target)] * n)
            self.pid_line.set_data(x, snap["pid_out"][-n:])
            self.vel_line.set_data(x, snap["vel"][-n:])
            self.trim_line.set_data(x, snap.get("trim", [])[-n:])
            self.ax.set_xlim(0, max(1, n - 1))
            self.ax.set_ylim(-15, 15)
            self.ax2.set_ylim(-260, 260)
            self.canvas.draw_idle()

            trim_hist = snap.get("trim", [])
            if trim_hist:
                state = "auto" if app.link.auto_trim_on else "idle"
                app.trim_var.set(f"Trim: {trim_hist[-1]:+.3f}° ({state})")
        else:
            app.pitch_var.set("Angle: --°")

        lines = app.link.recent_lines(50)
        self.log_text.config(state=tk.NORMAL)
        self.log_text.delete("1.0", tk.END)
        self.log_text.insert(tk.END, "\n".join(lines))
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)


# ---------------------------------------------------------------------------
# TAB 2: Kinematics & Health — servo health + raw per-servo position control
# ---------------------------------------------------------------------------
class KinematicsHealthTab(ttk.Frame):
    SERVO_NAMES = {6: "Leg1 L", 14: "Leg1 R", 0: "Leg2 L", 1: "Leg2 R"}

    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self._build_ui()

    def _build_ui(self):
        # -- live servo health --
        health_frame = ttk.LabelFrame(self, text="Live Servo Health")
        health_frame.pack(fill=tk.X, padx=5, pady=(5, 8))
        self.health_labels = {}
        for sid, name in [(6, "Leg1 L"), (14, "Leg1 R"), (0, "Leg2 L"), (1, "Leg2 R")]:
            lbl = tk.Label(health_frame,
                           text=f"ID {sid} ({name}): --°C  |  Load: --%",
                           font=("Consolas", 10, "bold"), bg="#eeeeee", fg="black", pady=6)
            lbl.pack(fill=tk.X, pady=2, padx=5)
            self.health_labels[sid] = lbl

        # -- raw per-servo position control (bypasses crouch IK entirely) --
        # Ported from the pre-variant-split GUI's "Send Pose to Servos" panel
        # (git 4e54920), adapted to raw AX-12 position rather than IK foot
        # targets — moves exactly one joint, useful for confirming which
        # physical leg an ID corresponds to and for freeing/testing a stuck
        # joint without going through the crouch solver. Explicit per-row
        # Send button (no live-drag-send) so nothing moves without a
        # deliberate click. Defaults are each servo's calibrated standing
        # position, not 0 — dragging to an extreme and hitting Send is on you.
        servo_ctrl_frame = ttk.LabelFrame(
            self, text="Servo Control (raw position, bypasses crouch IK — verify a joint moves freely by hand before sending)")
        servo_ctrl_frame.pack(fill=tk.X, padx=5, pady=(0, 8))
        self.servo_pos_vars = {}
        for sid, name, default_pos in [
            (6,  "Leg1 Left  (ID 6)",  818),
            (14, "Leg1 Right (ID 14)", 441),
            (0,  "Leg2 Left  (ID 0)",  818),
            (1,  "Leg2 Right (ID 1)",  441),
        ]:
            row = ttk.Frame(servo_ctrl_frame)
            row.pack(fill=tk.X, padx=5, pady=3)
            ttk.Label(row, text=name, width=18).pack(side=tk.LEFT)
            var = tk.IntVar(value=default_pos)
            self.servo_pos_vars[sid] = var
            tk.Scale(row, orient=tk.HORIZONTAL, from_=0, to=1023,
                     variable=var, length=300, showvalue=True).pack(side=tk.LEFT, padx=5)
            ttk.Button(row, text="Send", width=6,
                       command=lambda s=sid, v=var: self.send_servo_position(s, v.get())
                       ).pack(side=tk.LEFT, padx=5)

    def send_servo_position(self, servo_id, pos):
        if self.app.link:
            self.app.link.set_servo_position(servo_id, pos)
            self.app.status_var.set(f"Sent PS{servo_id} {pos}")

    def update_tab(self):
        if not self.app.link:
            return
        for sid, data in self.app.link.get_servo_health().items():
            if sid not in self.health_labels:
                continue
            temp, load = data["temp"], data["load"]
            lbl = self.health_labels[sid]
            lbl.config(text=f"ID {sid} ({self.SERVO_NAMES.get(sid, '?')}): {temp}°C  |  Load: {load:.1f}%")
            if temp >= 65:
                lbl.config(bg="#ff3333", fg="white")
            elif temp >= 55:
                lbl.config(bg="#ffaa00", fg="black")
            else:
                lbl.config(bg="#eeeeee", fg="black")


# ---------------------------------------------------------------------------
# Main Application — shared header + 2-tab Notebook (mirrors RC_mcu_IK_wireless)
# ---------------------------------------------------------------------------
class BalanceApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Single-Loop Wireless Balance Tuner")
        self.geometry("1200x850")
        self.minsize(1000, 700)
        self.link = None

        self.port_var         = tk.StringVar(value="COM13")
        self.status_var       = tk.StringVar(value="Disconnected")
        self.motor_var        = tk.StringVar(value="Motors: OFF")
        self.cutoff_var       = tk.StringVar(value="Safety: clear")
        self.pitch_var        = tk.StringVar(value="Angle: --°")
        self.offset_var       = tk.StringVar(value="Offset: --")
        self.offset_entry_var = tk.StringVar(value="0.0")
        self.trim_var         = tk.StringVar(value="Trim: --")
        self.auto_trim_var    = tk.BooleanVar(value=False)

        self._build_header()

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.tab_tuner = BalanceTunerTab(self.notebook, self)
        self.tab_legs  = KinematicsHealthTab(self.notebook, self)
        self.notebook.add(self.tab_tuner, text="1. Balance Tuner")
        self.notebook.add(self.tab_legs,  text="2. Kinematics & Health")

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._poll)

    # ── header: connection + actions + status (shared across both tabs) ─────
    def _build_header(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill=tk.X)

        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.port_var, width=9).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Connect",       command=self.connect).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Disconnect",    command=self.disconnect).pack(side=tk.LEFT, padx=2)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Button(top, text="Motors On/Off", command=self.toggle_motors).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Calibrate IMU", command=self.calibrate).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Reset Integral",command=self.reset_integral).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Safety Reset",  command=self.safety_reset).pack(side=tk.LEFT, padx=2)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Label(top, textvariable=self.offset_var, width=12).pack(side=tk.LEFT)
        ttk.Entry(top, textvariable=self.offset_entry_var, width=6).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Set Offset", command=self.set_manual_offset).pack(side=tk.LEFT)

        ttk.Separator(top, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)

        ttk.Checkbutton(top, text="Auto-Trim", variable=self.auto_trim_var,
                        command=self.toggle_auto_trim).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text="Commit Trim", command=self.commit_trim).pack(side=tk.LEFT, padx=2)
        ttk.Label(top, textvariable=self.trim_var, width=14).pack(side=tk.LEFT)

        ttk.Button(top, text="Save Params", command=self.save_params).pack(side=tk.LEFT, padx=8)

        # right-aligned live status
        ttk.Label(top, textvariable=self.cutoff_var,
                  font=("Consolas", 10, "bold"), foreground="red").pack(side=tk.RIGHT, padx=8)
        ttk.Label(top, textvariable=self.motor_var,
                  font=("Consolas", 10, "bold")).pack(side=tk.RIGHT, padx=8)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.RIGHT, padx=8)
        ttk.Label(top, textvariable=self.pitch_var,
                  font=("Consolas", 13, "bold"), foreground="#1f77b4").pack(side=tk.RIGHT, padx=12)

    # ── connection / actions ─────────────────────────────────────────────────
    def connect(self):
        if self.link:
            return
        port = self.port_var.get()
        try:
            self.link = SerialLink(port, baud=115200)   # 3DR radio baud
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

    def reset_integral(self):
        if self.link: self.link.reset_integral()

    def toggle_auto_trim(self):
        if self.link: self.link.set_auto_trim(self.auto_trim_var.get())

    def commit_trim(self):
        if self.link:
            self.link.commit_trim()
            self.status_var.set("Trim commit sent")

    def set_manual_offset(self):
        if self.link:
            try:
                val = float(self.offset_entry_var.get())
                self.link.set_offset(val)
                self.status_var.set(f"Sent manual offset: {val}")
            except ValueError:
                self.status_var.set("Invalid offset value")

    def save_params(self):
        profiles_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")
        os.makedirs(profiles_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename  = os.path.join(profiles_dir, f"params_{timestamp}.json")
        data = {key: slider.get_value() for key, slider in self.tab_tuner.sliders.items()}
        try:
            with open(filename, "w") as f:
                json.dump(data, f, indent=4)
            self.status_var.set(f"Saved to profiles/params_{timestamp}.json")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    # ── periodic refresh — only the active tab does plotting/redraw work ─────
    def _poll(self):
        if self.link:
            self.motor_var.set(f"Motors: {'ON' if self.link.motors_on else 'OFF'}")
            self.cutoff_var.set("Safety: LATCHED!" if self.link.cutoff_since() else "Safety: clear")
            self.auto_trim_var.set(self.link.auto_trim_on)

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
    app = BalanceApp()
    app.mainloop()
