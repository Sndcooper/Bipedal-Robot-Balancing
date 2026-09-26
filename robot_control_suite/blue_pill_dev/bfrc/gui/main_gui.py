"""
main_gui.py
Unified Wireless Balance + AX-12 Tuner — 3DR telemetry edition.

Three-tab Notebook:
  1. Balance Tuner        — live telemetry plot, PID/filter/trim sliders, serial monitor
  2. AX-12 Tuner          — SYNC_WRITE servo control (0-350 raw sliders), foot IK targets,
                            compliance/speed settings, live kinematics twin, torque on/off
  3. Kinematics & Health  — live servo temp/load, raw per-servo position (0-1023 bypass)

Pairs with the unified firmware in ../firmware (mcu_balance_fusion_wireless, forked from pretest_wireless v1 + SYNC_WRITE).
"""

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
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


# Balance loop parameters
PARAM_SPECS = [
    ParamSpec("Kp",          "Kp",       0.0,  200.0, 0.1,   5.0,  0.01,   3,  78.0),
    ParamSpec("Ki",          "Ki",       0.0, 1000.0, 0.5,   1.0,  0.001,  4, 650.0),
    ParamSpec("Kd",          "Kd",       0.0,   50.0, 0.1,  10.0,  0.01,   3,   3.52),
    ParamSpec("targetAngle", "Target",  -20.0,  20.0, 0.1,   5.0,  0.01,   3,   0.0),
    ParamSpec("alpha",       "alpha",    0.80, 0.999, 0.001, 0.02, 0.0001, 4,  0.96),
    ParamSpec("maxSafeTilt", "Max Tilt", 5.0,  50.0, 0.1,   5.0,  0.01,   2,  25.0),
    # Outer velocity loop. Kp_vel reacts to a push NOW; Ki_trim learns the
    # standing CoM offset over ~10 s. Both output DEGREES OF LEAN, not PWM.
    ParamSpec("Kp_vel",      "Vel P (lean)",0.0, 0.02, 0.0005, 0.002, 0.0001, 5, 0.0030),
    ParamSpec("Ki_trim",     "Vel I (trim)",0.0, 0.03, 0.001,  0.003, 0.0001, 5, 0.0015),
    ParamSpec("crouchOffsetL","Crouch L",0.0,  80.0, 1.0,  10.0,  0.1,    1,   0.0),
    ParamSpec("crouchOffsetR","Crouch R",0.0,  80.0, 1.0,  10.0,  0.1,    1,   0.0),
]

# IK foot-target parameters (mm)
IK_PARAM_SPECS = [
    ParamSpec("fx1",  "Leg1 X",  -100.0, 100.0,  1.0, 10.0, 0.1, 1,   1.0),
    ParamSpec("fy1",  "Leg1 Y",  -160.0, -20.0,  1.0, 10.0, 0.1, 1,-151.1),
    ParamSpec("fx2",  "Leg2 X",  -100.0, 100.0,  1.0, 10.0, 0.1, 1,  -6.0),
    ParamSpec("fy2",  "Leg2 Y",  -160.0, -20.0,  1.0, 10.0, 0.1, 1,-149.6),
]

# Compliance/speed parameters
CMD_PARAM_SPECS = [
    ParamSpec("torque_limit", "Torque Limit", 0.0, 1023.0, 1.0, 100.0, 1.0, 0, 1023.0),
    ParamSpec("comp_margin",  "Comp Margin",  0.0,  254.0, 1.0,  20.0, 1.0, 0,    1.0),
    ParamSpec("comp_slope",   "Comp Slope",   0.0,  254.0, 1.0,  20.0, 1.0, 0,    4.0),
    ParamSpec("moving_speed", "Moving Speed", 0.0, 1023.0, 1.0, 100.0, 1.0, 0,    0.0),
    ParamSpec("move_time_ms", "Move Time ms",100.0,3000.0,10.0, 200.0,10.0, 0,  800.0),
]


# ---------------------------------------------------------------------------
# Coarse/Fine precision zoom slider (shared by all three tabs)
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
            self._awaiting_echo = False
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
# TAB 1: Balance Tuner — plot, PID/filter/trim sliders, serial monitor
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
        left.rowconfigure(0, weight=3, minsize=260)
        left.rowconfigure(1, weight=0)
        left.rowconfigure(2, weight=1)

        plot_frame = ttk.LabelFrame(left, text="Live Telemetry")
        plot_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 6))

        self.fig = Figure(figsize=(6, 4), dpi=100)
        self.ax  = self.fig.add_subplot(111)
        self.ax2 = self.ax.twinx()
        self.ax.grid(True, alpha=0.25)
        self.pitch_line,  = self.ax.plot([], [], color="#1f77b4", label="pitch")
        self.target_line, = self.ax.plot([], [], color="#2ca02c", ls="--", label="target")
        self.trim_line,   = self.ax.plot([], [], color="#d62728", ls=":",  label="trim")
        self.pid_line,    = self.ax2.plot([], [], color="#ff7f0e", alpha=0.9, label="pid_out")
        self.vel_line,    = self.ax2.plot([], [], color="#9467bd", alpha=0.6, label="vel")
        self.ax.set_ylabel("angle (°)")
        self.ax2.set_ylabel("pid / vel")
        self.ax.legend(loc="upper left", fontsize=8)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ── Live Servo Health & Temperatures ──────────────────────────────────
        servo_frame = ttk.LabelFrame(left, text="Servo Temperatures & Health")
        servo_frame.grid(row=1, column=0, sticky="ew", pady=(0, 6))
        servo_frame.columnconfigure((0, 1, 2, 3), weight=1)

        self.servo_labels = {}
        for col, (sid, name) in enumerate([(6, "Leg1 L"), (14, "Leg1 R"), (0, "Leg2 L"), (1, "Leg2 R")]):
            box = tk.Frame(servo_frame, relief=tk.GROOVE, bd=1, padx=4, pady=3, bg="#f8f9fa")
            box.grid(row=0, column=col, sticky="ew", padx=3, pady=3)

            lbl_title = tk.Label(box, text=f"ID {sid} · {name}", font=("Helvetica", 8, "bold"), bg="#f8f9fa", fg="#555555")
            lbl_title.pack(anchor="center")

            lbl_val = tk.Label(box, text="--°C  |  --%", font=("Consolas", 10, "bold"), bg="#f8f9fa", fg="#111111")
            lbl_val.pack(anchor="center")

            self.servo_labels[sid] = (box, lbl_title, lbl_val)

        log_frame = ttk.LabelFrame(left, text="Serial Monitor & Backup")
        log_frame.grid(row=2, column=0, sticky="nsew")

        # Backup control toolbar
        bar = ttk.Frame(log_frame)
        bar.pack(fill=tk.X, padx=4, pady=(2, 4))

        self.auto_backup_var = tk.BooleanVar(value=True)
        self.cb_auto = ttk.Checkbutton(
            bar, text="Auto-Backup on Motor ON", variable=self.auto_backup_var,
            command=self._toggle_auto_backup
        )
        self.cb_auto.pack(side=tk.LEFT, padx=(2, 6))

        self.btn_rec = ttk.Button(bar, text="Manual REC", width=11, command=self._toggle_manual_rec)
        self.btn_rec.pack(side=tk.LEFT, padx=2)

        ttk.Button(bar, text="📁 Open Logs", command=self.open_logs_folder).pack(side=tk.LEFT, padx=4)

        self.backup_lbl = ttk.Label(
            bar, text="Backup: Armed", font=("Consolas", 9, "bold"), foreground="#2ca02c"
        )
        self.backup_lbl.pack(side=tk.RIGHT, padx=4)

        self.log_text = tk.Text(log_frame, height=8, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=2, pady=(0, 2))

        tuning_frame = ttk.LabelFrame(self, text="Tuning  (balance PID + auto-trim + crouch)")
        tuning_frame.grid(row=0, column=1, sticky="nsew")
        self.sliders = {}
        for spec in PARAM_SPECS:
            ctrl = CoarseFineSlider(tuning_frame, spec, spec.start_val, self._on_slider)
            ctrl.pack(fill=tk.X, pady=4, padx=5)
            self.sliders[spec.key] = ctrl
            if spec.key == "crouchOffsetR":
                # Mirror sits right under the L/R crouch pair it links.
                self.crouch_mirror_var = tk.BooleanVar(value=True)
                ttk.Checkbutton(
                    tuning_frame, text="Mirror L↔R Crouch",
                    variable=self.crouch_mirror_var
                ).pack(anchor="w", padx=5, pady=(0, 4))

        # Save tuned values with comment button
        btn_save = ttk.Button(
            tuning_frame, text="💾 Save Tuned Params (with Comment)",
            command=self.app.save_params_with_comment
        )
        btn_save.pack(fill=tk.X, padx=5, pady=(8, 4))

    def _toggle_auto_backup(self):
        if self.app.link:
            self.app.link.set_auto_backup(self.auto_backup_var.get())

    def _toggle_manual_rec(self):
        if not self.app.link:
            messagebox.showinfo("Not Connected", "Please connect to serial port first.")
            return
        b_stat = self.app.link.get_backup_status()
        if b_stat["active"]:
            self.app.link.stop_backup_session(reason="Manual Stop by User")
        else:
            self.app.link.start_backup_session(reason="Manual Start by User")

    def open_logs_folder(self):
        gui_dir = os.path.dirname(os.path.abspath(__file__))
        logs_dir = os.path.join(gui_dir, "logs")
        os.makedirs(logs_dir, exist_ok=True)
        try:
            os.startfile(logs_dir)
        except Exception:
            messagebox.showinfo("Logs Directory", f"Logs folder:\n{logs_dir}")

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
        elif key == "Kp_vel":      lk.set_vel_p(val)
        elif key == "crouchOffsetL":
            if self.crouch_mirror_var.get():
                self.sliders["crouchOffsetR"].set_value(val)
                lk.set_crouch(val)
            else:
                lk.set_crouch_l(val)
        elif key == "crouchOffsetR":
            if self.crouch_mirror_var.get():
                self.sliders["crouchOffsetL"].set_value(val)
                lk.set_crouch(val)
            else:
                lk.set_crouch_r(val)

    def update_tab(self):
        app = self.app
        if not app.link:
            return

        for spec in PARAM_SPECS:
            val = app.link.fw.get(spec.key)
            if val is not None and spec.key in self.sliders:
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

        # Update backup status indicator
        b_stat = app.link.get_backup_status()
        if b_stat["active"]:
            self.backup_lbl.config(
                text=f"● REC: {b_stat['filename']} ({b_stat['lines']} lines)",
                foreground="#d62728"
            )
            self.btn_rec.config(text="Stop REC")
        else:
            last = os.path.basename(b_stat["last_saved"]) if b_stat["last_saved"] else ""
            if last:
                self.backup_lbl.config(
                    text=f"Saved: {last} ({b_stat['lines']} lines)",
                    foreground="#444444"
                )
            else:
                self.backup_lbl.config(
                    text="Backup: Armed (Auto on Motor ON)" if b_stat["auto_enabled"] else "Backup: Disabled",
                    foreground="#2ca02c" if b_stat["auto_enabled"] else "#888888"
                )
            self.btn_rec.config(text="Manual REC")

        # Update servo temperature and health displays
        srv_health = app.link.get_servo_health()
        for sid, (box, lbl_title, lbl_val) in self.servo_labels.items():
            if sid in srv_health:
                temp = srv_health[sid].get("temp", 0)
                load = srv_health[sid].get("load", 0.0)
                err  = srv_health[sid].get("err", 0)
                fail = srv_health[sid].get("fail", 0)

                # An AX-12 that latched Alarm Shutdown goes limp and IGNORES
                # every goal write. Surfacing it is the whole point: previously
                # the only clue was a temperature that stopped changing, which
                # looks identical to a healthy servo.
                flags = []
                if err & 0x04: flags.append("OVERHEAT")
                if err & 0x20: flags.append("OVERLOAD")
                if err & 0x01: flags.append("VOLTAGE")
                if err & 0x02: flags.append("ANGLE")

                if fail >= 3:
                    lbl_val.config(text=f"NO REPLY x{fail}")
                    bg_col, fg_col = "#ffdddd", "#cc0000"
                elif flags:
                    lbl_val.config(text=f"{temp}°C  {'+'.join(flags)}")
                    bg_col, fg_col = "#ffdddd", "#cc0000"
                else:
                    lbl_val.config(text=f"{temp}°C  |  {load:.1f}%")
                    if temp >= 65:
                        bg_col, fg_col = "#ffdddd", "#cc0000"
                    elif temp >= 55:
                        bg_col, fg_col = "#fff3cd", "#856404"
                    elif temp > 0:
                        bg_col, fg_col = "#e8f5e9", "#1b5e20"
                    else:
                        bg_col, fg_col = "#f8f9fa", "#555555"
                box.config(bg=bg_col)
                lbl_title.config(bg=bg_col)
                lbl_val.config(bg=bg_col, fg=fg_col)


# ---------------------------------------------------------------------------
# TAB 2: AX-12 Tuner — SYNC_WRITE servo control, IK foot targets,
#         compliance/speed settings, live 5-bar kinematics twin
# ---------------------------------------------------------------------------
class AX12TunerTab(ttk.Frame):
    """
    0-350 raw position sliders: the user specified 0-350 as the operating
    sub-range for the direct servo position sliders. This caps the slider
    travel to prevent over-travel, but the firmware still accepts 0-1023;
    the foot IK sliders work in mm and have no such cap.
    """
    SERVO_RAW_MAX = 350   # safety cap for raw position sliders

    def __init__(self, master, app):
        super().__init__(master)
        self.app            = app
        self.last_send_time = 0
        self.SEND_INTERVAL  = 0.05   # 20 Hz throttle for IK sliders
        self._build_ui()

    # ── build ─────────────────────────────────────────────────────────────────
    def _build_ui(self):
        self.columnconfigure(0, weight=2)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)

        # ── Left column: twin diagram + raw servo sliders ────────────────────
        left = ttk.Frame(self)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=2)
        left.rowconfigure(1, weight=1)

        # Live kinematics twin
        twin_frame = ttk.LabelFrame(left, text="Live Kinematics Twin (5-bar IK)")
        twin_frame.grid(row=0, column=0, sticky="nsew", pady=(0, 6))

        self.fig = Figure(figsize=(5, 4), dpi=100)
        self.fig.patch.set_facecolor("#1a1a2e")
        self.ax  = self.fig.add_subplot(111)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-150, 340)
        self.ax.set_ylim(-200, 50)
        self.ax.set_facecolor("#111122")
        self.ax.tick_params(colors="#888888")
        for spine in self.ax.spines.values():
            spine.set_edgecolor("#333355")

        self.leg1_arts = self._make_leg_artists("#ff6b6b", "#ffd6d6")
        self.leg2_arts = self._make_leg_artists("#da77f2", "#f3d6ff")

        # Status dots (IK valid indicator)
        self.ik_dots = []
        for c in ("#00ff88", "#00ff88"):
            dot, = self.ax.plot([], [], "o", color=c, ms=8, zorder=10)
            self.ik_dots.append(dot)

        self.canvas = FigureCanvasTkAgg(self.fig, master=twin_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Raw servo position sliders (0-350 cap, no live-send — explicit Send btn)
        raw_frame = ttk.LabelFrame(
            left,
            text=f"Raw Servo Position (0–{self.SERVO_RAW_MAX} cap, bypasses IK — verify joint free before Send)")
        raw_frame.grid(row=1, column=0, sticky="nsew")
        self.servo_pos_vars = {}
        _servo_defaults = [(6, "Leg1 Left  (ID 6)", 218), (14, "Leg1 Right (ID 14)", 221),
                           (0,  "Leg2 Left  (ID 0)", 218), (1,  "Leg2 Right (ID 1)", 221)]
        for sid, name, default_pos in _servo_defaults:
            row = ttk.Frame(raw_frame)
            row.pack(fill=tk.X, padx=5, pady=3)
            ttk.Label(row, text=name, width=20).pack(side=tk.LEFT)
            var = tk.IntVar(value=default_pos)
            self.servo_pos_vars[sid] = var
            tk.Scale(row, orient=tk.HORIZONTAL,
                     from_=0, to=self.SERVO_RAW_MAX,
                     variable=var, length=260, showvalue=True).pack(side=tk.LEFT, padx=4)
            ttk.Button(row, text="Send", width=6,
                       command=lambda s=sid, v=var: self._send_raw_pos(s, v.get())
                       ).pack(side=tk.LEFT, padx=4)
        # Sync-write all four
        ttk.Button(raw_frame, text="⚡ SYNC WRITE ALL  (all 4 servos simultaneously)",
                   command=self._sync_write_all
                   ).pack(fill=tk.X, padx=5, pady=(2, 6))

        # ── Right column: IK foot targets + compliance + mode ─────────────────
        right = ttk.Frame(self)
        right.grid(row=0, column=1, sticky="nsew")

        # Foot IK targets
        ik_frame = ttk.LabelFrame(right, text="Foot IK Targets (mm) — triggers interpolated move")
        ik_frame.pack(fill=tk.X, pady=(0, 6))

        self.ik_sliders = {}
        for spec in IK_PARAM_SPECS:
            ctrl = CoarseFineSlider(ik_frame, spec, spec.start_val, self._on_ik_change)
            ctrl.pack(fill=tk.X, pady=3, padx=5)
            self.ik_sliders[spec.key] = ctrl

        self.mirror_var = tk.BooleanVar(value=True)
        mir_row = ttk.Frame(ik_frame)
        mir_row.pack(fill=tk.X, padx=5, pady=(0, 4))
        ttk.Checkbutton(mir_row, text="Mirror Leg2 to Leg1",
                        variable=self.mirror_var,
                        command=lambda: self._on_ik_change("mirror", 0.0)).pack(side=tk.LEFT)
        ttk.Button(mir_row, text="Home Pose", command=self._home_pose).pack(side=tk.LEFT, padx=5)

        # Move time slider
        mt_frame = ttk.Frame(ik_frame)
        mt_frame.pack(fill=tk.X, padx=5, pady=(0, 4))
        ttk.Label(mt_frame, text="Move Time ms:", width=16).pack(side=tk.LEFT)
        self.move_time_var = tk.IntVar(value=800)
        tk.Scale(mt_frame, orient=tk.HORIZONTAL, from_=100, to=3000,
                 variable=self.move_time_var, length=140, showvalue=True,
                 command=lambda _: self._on_move_time_change()).pack(side=tk.LEFT)

        # Pose mode
        mode_frame = ttk.LabelFrame(right, text="Pose Mode")
        mode_frame.pack(fill=tk.X, pady=(0, 6))
        self.ik_mode_var = tk.BooleanVar(value=False)
        ttk.Radiobutton(mode_frame, text="CROUCH (one vertical knob)",
                        variable=self.ik_mode_var, value=False,
                        command=self._on_mode_change).pack(anchor="w", padx=5, pady=2)
        ttk.Radiobutton(mode_frame, text="IK (free foot x,y targets)",
                        variable=self.ik_mode_var, value=True,
                        command=self._on_mode_change).pack(anchor="w", padx=5, pady=2)

        # Compliance & torque
        cmp_frame = ttk.LabelFrame(right, text="Global Compliance & Torque (all 4 servos)")
        cmp_frame.pack(fill=tk.X, pady=(0, 6))

        self.cmd_sliders = {}
        for spec in CMD_PARAM_SPECS:
            ctrl = CoarseFineSlider(cmp_frame, spec, spec.start_val, self._on_cmd_change)
            ctrl.pack(fill=tk.X, pady=3, padx=5)
            self.cmd_sliders[spec.key] = ctrl

        # Torque on/off
        torq_row = ttk.Frame(cmp_frame)
        torq_row.pack(fill=tk.X, padx=5, pady=(2, 6))
        self.torq_lbl = ttk.Label(torq_row, text="Torque: ON", font=("Consolas", 10, "bold"),
                                   foreground="green")
        self.torq_lbl.pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(torq_row, text="Torque ON",  width=10,
                   command=lambda: self._set_torque(True)).pack(side=tk.LEFT, padx=2)
        ttk.Button(torq_row, text="LIMP (off)", width=10,
                   command=lambda: self._set_torque(False)).pack(side=tk.LEFT, padx=2)

        # Move indicator
        self.move_lbl = ttk.Label(right, text="Move: idle",
                                   font=("Consolas", 9), foreground="#888888")
        self.move_lbl.pack(pady=2)

    # ── Twin diagram helpers ───────────────────────────────────────────────────
    def _make_leg_artists(self, femur_color, tibia_color):
        """Create (line_femurL, line_tibiaL, line_femurR, line_tibiaR, dot_foot)."""
        fL, = self.ax.plot([], [], lw=3, color=femur_color)
        tL, = self.ax.plot([], [], lw=2, color=tibia_color, alpha=0.8)
        fR, = self.ax.plot([], [], lw=3, color=femur_color)
        tR, = self.ax.plot([], [], lw=2, color=tibia_color, alpha=0.8)
        dot, = self.ax.plot([], [], "o", ms=8, color=femur_color)
        return (fL, tL, fR, tR, dot)

    def _draw_leg(self, arts, sol, offset_x):
        """Update matplotlib artists for one leg from an IK solution dict."""
        if sol is None:
            for a in arts:
                a.set_data([], [])
            return
        sl = tkin.SERVO_L + [offset_x, 0]
        sr = tkin.SERVO_R + [offset_x, 0]
        kL = sol["Knee_L"]
        kR = sol["Knee_R"]
        foot = sol["Knee_L"]  # placeholder — actual foot via FK
        # Femur segments
        arts[0].set_data([sl[0], kL[0]], [sl[1], kL[1]])
        arts[2].set_data([sr[0], kR[0]], [sr[1], kR[1]])
        # Tibia segments (knee to foot)
        # For the twin we approximate foot from the FK of the two knee angles
        fk = tkin.solve_fk(
            math.degrees(math.atan2(kL[1]-sl[1], kL[0]-sl[0])),
            math.degrees(math.atan2(kR[1]-sr[1], kR[0]-sr[0])),
            offset_x
        )
        if fk is not None:
            arts[1].set_data([kL[0], fk[0]], [kL[1], fk[1]])
            arts[3].set_data([kR[0], fk[0]], [kR[1], fk[1]])
            arts[4].set_data([fk[0]], [fk[1]])
        else:
            arts[1].set_data([], [])
            arts[3].set_data([], [])
            arts[4].set_data([], [])

    def _redraw_twin(self):
        """Called on every _on_ik_change to keep the twin live."""
        x1 = self.ik_sliders["fx1"].get_value()
        y1 = self.ik_sliders["fy1"].get_value()
        x2 = self.ik_sliders["fx2"].get_value()
        y2 = self.ik_sliders["fy2"].get_value()

        sol1 = tkin.solve_ik(x1, y1, 0.0)
        sol2 = tkin.solve_ik(x2 + tkin.LEG_DISTANCE, y2, tkin.LEG_DISTANCE)

        self._draw_leg(self.leg1_arts, sol1, 0.0)
        self._draw_leg(self.leg2_arts, sol2, tkin.LEG_DISTANCE)

        # IK validity dots
        dot1_x = x1 if sol1 is not None else 0
        dot2_x = x2 + tkin.LEG_DISTANCE if sol2 is not None else tkin.LEG_DISTANCE
        c1 = "#00ff88" if sol1 is not None else "#ff4444"
        c2 = "#00ff88" if sol2 is not None else "#ff4444"
        self.ik_dots[0].set_data([dot1_x], [y1])
        self.ik_dots[0].set_color(c1)
        self.ik_dots[1].set_data([dot2_x], [y2])
        self.ik_dots[1].set_color(c2)

        self.canvas.draw_idle()

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _on_ik_change(self, key, val):
        if self.mirror_var.get() and key in ("fx1", "fy1"):
            # Mirror leg1→leg2
            if key == "fx1":
                self.ik_sliders["fx2"].set_value(val)
            elif key == "fy1":
                self.ik_sliders["fy2"].set_value(val)

        self._redraw_twin()

        now = time.time()
        if now - self.last_send_time < self.SEND_INTERVAL:
            return
        self.last_send_time = now

        if not (self.app.link and self.app.link.ser):
            return
        lk = self.app.link
        x1 = self.ik_sliders["fx1"].get_value()
        y1 = self.ik_sliders["fy1"].get_value()
        x2 = self.ik_sliders["fx2"].get_value()
        y2 = self.ik_sliders["fy2"].get_value()
        if self.mirror_var.get():
            x2, y2 = x1, y1
        lk.set_both_feet(x1, y1, x2, y2)

    def _on_cmd_change(self, key, val):
        if not (self.app.link and self.app.link.ser):
            return
        lk = self.app.link
        if   key == "torque_limit": lk.set_torque_limit(int(val))
        elif key == "comp_margin":  lk.set_comp_margin(int(val))
        elif key == "comp_slope":   lk.set_comp_slope(int(val))
        elif key == "moving_speed": lk.set_moving_speed(int(val))
        elif key == "move_time_ms": lk.set_move_time(int(val))

    def _on_move_time_change(self):
        if self.app.link and self.app.link.ser:
            self.app.link.set_move_time(self.move_time_var.get())

    def _on_mode_change(self):
        if self.app.link and self.app.link.ser:
            self.app.link.set_pose_mode(self.ik_mode_var.get())

    def _set_torque(self, on: bool):
        if self.app.link and self.app.link.ser:
            self.app.link.set_torque(on)
            self.torq_lbl.config(
                text=f"Torque: {'ON' if on else 'LIMP'}",
                foreground="green" if on else "orange")
            self.app.status_var.set(f"Torque {'ON' if on else 'LIMP'} sent")

    def _send_raw_pos(self, servo_id, pos):
        if self.app.link:
            self.app.link.set_servo_position(servo_id, pos)
            self.app.status_var.set(f"Sent PS{servo_id} {pos}")

    def _sync_write_all(self):
        """Sends the current raw slider values for all 4 servos via PS commands.
        The firmware's SYNC_WRITE broadcasts them atomically on the next tick."""
        if not (self.app.link and self.app.link.ser):
            return
        for sid, var in self.servo_pos_vars.items():
            self.app.link.set_servo_position(sid, var.get())
        self.app.status_var.set("SYNC WRITE — all 4 servos sent")

    def _home_pose(self):
        if self.app.link and self.app.link.ser:
            self.app.link.home_pose()
            self.app.status_var.set("Home pose sent")

    # ── Periodic update from firmware telemetry ───────────────────────────────
    def update_tab(self):
        if not self.app.link:
            return
        ax12 = self.app.link.get_ax12_state()

        # Sync compliance sliders from firmware state
        for key, slider in self.cmd_sliders.items():
            fw_val = ax12.get(key)
            if fw_val is not None:
                slider.sync_from_external(float(fw_val))

        # Move time
        mt = ax12.get("move_time_ms")
        if mt is not None:
            self.move_time_var.set(int(mt))

        # Torque indicator
        ton = ax12.get("torque_on", True)
        self.torq_lbl.config(
            text=f"Torque: {'ON' if ton else 'LIMP'}",
            foreground="green" if ton else "orange")

        # Move indicator
        moving = ax12.get("move_active", False)
        self.move_lbl.config(
            text="Move: RUNNING ●" if moving else "Move: idle",
            foreground="#ff9900" if moving else "#888888")

        # Pose mode
        self.ik_mode_var.set(bool(ax12.get("mode", 0)))

        # Sync IK sliders to firmware's current foot targets
        for key in ("fx1", "fy1", "fx2", "fy2"):
            fw_val = ax12.get(key)
            if fw_val is not None and key in self.ik_sliders:
                self.ik_sliders[key].sync_from_external(float(fw_val))

        # Keep twin live from firmware foot targets
        self._redraw_twin()


# ---------------------------------------------------------------------------
# TAB 3: Kinematics & Health — servo health + raw per-servo position bypass
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

        # -- raw per-servo position control (full 0-1023 for diagnostic use) --
        servo_ctrl_frame = ttk.LabelFrame(
            self,
            text="Servo Position Bypass (full 0-1023 range, bypasses IK — verify joint free first)")
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
            scale = tk.Scale(row, orient=tk.HORIZONTAL, from_=0, to=1023,
                     variable=var, length=300, showvalue=True,
                     command=lambda val, s=sid: self.send_servo_position(s, val))
            scale.pack(side=tk.LEFT, padx=5)
            
            ttk.Button(row, text="Send", width=6,
                       command=lambda s=sid, v=var: self.send_servo_position(s, v.get())
                       ).pack(side=tk.LEFT, padx=5)

    def send_servo_position(self, servo_id, pos):
        if self.app.link:
            val = int(float(pos))
            self.app.link.set_servo_position(servo_id, val)
            self.app.status_var.set(f"Sent PS{servo_id} {val}")

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



# ============================================================================
# TAB 4 - RC CALIBRATION & TUNING
# ----------------------------------------------------------------------------
# Two jobs, deliberately in one tab because they are done together during
# bring-up:
#
#   1. CALIBRATE. The firmware ships nominal 1000/1500/2000 endpoints, but real
#      pots and switches do not sit there. The operator moves one channel at a
#      time and captures its min / centre / max; the numbers go to the firmware
#      with RCC and are used by rcAxisCal() from then on. Calibrating one
#      channel at a time is the whole workflow - a live bar per channel makes it
#      obvious which physical control moves which channel, which is the single
#      most common RC bring-up mistake.
#
#   2. TUNE the rate bands (Ch8 target, Ch10 crouch) and the travel limits.
#
# The loop-timing readout lives here too, because RC polling is the thing most
# likely to disturb the tick budget and you want to watch it while moving the
# sticks.
#
# CHANNEL MAP (must match main.cpp's readRC()):
#   Ch3 drive, Ch4 steer, Ch5 calibrate, Ch6 zero-target, Ch7 motors,
#   Ch8 target+/-, Ch9 leg select, Ch10 crouch.
# ============================================================================
class RCTuneTab(ttk.Frame):
    # Kept in step with main.cpp's header comment. Index 0 == Ch1.
    CH_INFO = [
        ("Ch1",  "(unused)",             "-"),
        ("Ch2",  "(unused)",             "-"),
        ("Ch3",  "DRIVE fwd/back",       "pot, self-centring"),
        ("Ch4",  "STEER left/right",     "pot, self-centring"),
        ("Ch5",  "CALIBRATE",            "button, momentary"),
        ("Ch6",  "TARGET = 0",           "button, momentary"),
        ("Ch7",  "MOTORS on/off",        "switch, 2-pos"),
        ("Ch8",  "TARGET +/-",           "pot, rate control"),
        ("Ch9",  "LEG SELECT",           "switch, 3-pos"),
        ("Ch10", "CROUCH / STRETCH",     "pot, rate control"),
    ]
    LEG_SEL_NAMES = {0: "MIRRORED (both)", 1: "LEFT leg only", 2: "RIGHT leg only"}

    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        # Live capture buffers, per channel: what the operator has swept so far.
        # Separate from the committed calibration so nothing reaches the robot
        # until Apply is pressed.
        self._seen_min = [None] * 10
        self._seen_max = [None] * 10
        self._cal_vars = []      # (min, cen, max) StringVars per channel
        self._bars     = []
        self._val_lbls = []
        self._build_ui()

    def _build_ui(self):
        # -- link status -------------------------------------------------------
        top = ttk.Frame(self)
        top.pack(fill=tk.X, padx=5, pady=(5, 0))
        self.link_var = tk.StringVar(value="RC link: --")
        self.arm_var  = tk.StringVar(value="Armed: --")
        ttk.Label(top, textvariable=self.link_var,
                  font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=(0, 15))
        ttk.Label(top, textvariable=self.arm_var,
                  font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT)

        # -- per-channel calibration grid -------------------------------------
        cal = ttk.LabelFrame(self, text="Channel calibration - sweep ONE channel at a time, then Apply")
        cal.pack(fill=tk.X, padx=5, pady=6)

        hdr = ("Ch", "Function", "Type", "Live us", "Bar", "Min", "Centre", "Max", "")
        for c, h in enumerate(hdr):
            ttk.Label(cal, text=h, font=("Segoe UI", 8, "bold")).grid(
                row=0, column=c, padx=3, pady=(3, 4), sticky="w")

        for i, (name, func, kind) in enumerate(self.CH_INFO):
            r = i + 1
            ttk.Label(cal, text=name, font=("Consolas", 9, "bold")).grid(
                row=r, column=0, padx=3, sticky="w")
            ttk.Label(cal, text=func, font=("Segoe UI", 8)).grid(
                row=r, column=1, padx=3, sticky="w")
            ttk.Label(cal, text=kind, font=("Segoe UI", 8),
                      foreground="#666").grid(row=r, column=2, padx=3, sticky="w")

            lbl = ttk.Label(cal, text="0", font=("Consolas", 9), width=6)
            lbl.grid(row=r, column=3, padx=3, sticky="w")
            self._val_lbls.append(lbl)

            bar = ttk.Progressbar(cal, orient=tk.HORIZONTAL, length=150,
                                  mode="determinate", maximum=1000)
            bar.grid(row=r, column=4, padx=3, sticky="w")
            self._bars.append(bar)

            mn = tk.StringVar(value="1000")
            cn = tk.StringVar(value="1500")
            mx = tk.StringVar(value="2000")
            self._cal_vars.append((mn, cn, mx))
            for c, var in ((5, mn), (6, cn), (7, mx)):
                ttk.Entry(cal, textvariable=var, width=6,
                          font=("Consolas", 9)).grid(row=r, column=c, padx=2)

            btns = ttk.Frame(cal)
            btns.grid(row=r, column=8, padx=3, sticky="w")
            # Sweep captures the extremes seen since the last reset; Centre
            # snapshots the CURRENT width, which is why they are separate
            # buttons - you release the stick, then press Centre.
            ttk.Button(btns, text="Sweep", width=6,
                       command=lambda n=i: self._capture_sweep(n)).pack(side=tk.LEFT, padx=1)
            ttk.Button(btns, text="Centre", width=7,
                       command=lambda n=i: self._capture_centre(n)).pack(side=tk.LEFT, padx=1)
            ttk.Button(btns, text="Apply", width=6,
                       command=lambda n=i: self._apply_channel(n)).pack(side=tk.LEFT, padx=1)

        act = ttk.Frame(cal)
        act.grid(row=len(self.CH_INFO) + 1, column=0, columnspan=9, pady=(6, 4), sticky="w")
        ttk.Button(act, text="Apply ALL", command=self._apply_all).pack(side=tk.LEFT, padx=3)
        ttk.Button(act, text="Read from robot", command=self._read_from_robot).pack(side=tk.LEFT, padx=3)
        ttk.Button(act, text="Reset sweep", command=self._reset_sweep).pack(side=tk.LEFT, padx=3)
        ttk.Button(act, text="Defaults on robot", command=self._reset_robot).pack(side=tk.LEFT, padx=3)
        ttk.Button(act, text="Save to file", command=self._save_cal).pack(side=tk.LEFT, padx=3)
        ttk.Button(act, text="Load from file", command=self._load_cal).pack(side=tk.LEFT, padx=3)

        # -- decoded state ----------------------------------------------------
        dec = ttk.LabelFrame(self, text="Decoded state")
        dec.pack(fill=tk.X, padx=5, pady=6)
        self.dec_vars = {}
        for c, (key, label) in enumerate([
                ("drive",  "Drive (Ch3)"), ("steer", "Steer (Ch4)"),
                ("target", "Target (Ch8)"), ("leg",   "Leg sel (Ch9)"),
                ("crouch", "Crouch (Ch10)")]):
            ttk.Label(dec, text=label, font=("Segoe UI", 8, "bold")).grid(
                row=0, column=c, padx=8, pady=(4, 0), sticky="w")
            v = tk.StringVar(value="--")
            self.dec_vars[key] = v
            ttk.Label(dec, textvariable=v, font=("Consolas", 10)).grid(
                row=1, column=c, padx=8, pady=(0, 5), sticky="w")

        # -- rate / travel tuning ---------------------------------------------
        tune = ttk.LabelFrame(self, text="Rate & travel tuning")
        tune.pack(fill=tk.X, padx=5, pady=6)
        self.tune_vars = {}
        rows = [
            ("rate_min",  "Ch8 rate at min pot (counts/s)", "10",  self._send_rate_min),
            ("rate_max",  "Ch8 rate at full pot (counts/s)", "100", self._send_rate_max),
            ("tgt_limit", "Ch8 target clamp (counts)",       "5000", self._send_tgt_limit),
            ("crouch_rate", "Ch10 crouch rate (mm/s)",       "25",  self._send_crouch_rate),
            ("max_crouch",  "Ch10 crouch travel limit (mm)", "40",  self._send_max_crouch),
            ("max_vel",   "Ch3 max velocity (counts/s)",     "400", self._send_max_vel),
            ("max_steer", "Ch4 max steer (PWM counts)",      "40",  self._send_max_steer),
        ]
        for r, (key, label, default, cb) in enumerate(rows):
            ttk.Label(tune, text=label, font=("Segoe UI", 9)).grid(
                row=r, column=0, padx=6, pady=2, sticky="w")
            v = tk.StringVar(value=default)
            self.tune_vars[key] = v
            ttk.Entry(tune, textvariable=v, width=9,
                      font=("Consolas", 9)).grid(row=r, column=1, padx=4, pady=2)
            ttk.Button(tune, text="Set", width=5, command=cb).grid(
                row=r, column=2, padx=4, pady=2)

        # -- loop timing ------------------------------------------------------
        lt = ttk.LabelFrame(self, text="Loop timing (body time, reported one tick late)")
        lt.pack(fill=tk.X, padx=5, pady=6)
        self.lt_vars = {}
        for c, (key, label) in enumerate([
                ("body",   "Body us"), ("period", "Period us"),
                ("peak",   "Peak us"), ("overruns", "Overruns"),
                ("budget", "% of 10 ms budget")]):
            ttk.Label(lt, text=label, font=("Segoe UI", 8, "bold")).grid(
                row=0, column=c, padx=10, pady=(4, 0), sticky="w")
            v = tk.StringVar(value="--")
            self.lt_vars[key] = v
            ttk.Label(lt, textvariable=v, font=("Consolas", 11)).grid(
                row=1, column=c, padx=10, pady=(0, 5), sticky="w")
        ttk.Button(lt, text="Reset peak", command=self._reset_loop_timer).grid(
            row=1, column=5, padx=10)

    # -- capture helpers -------------------------------------------------------
    def _live(self, idx):
        if not self.app.link:
            return 0
        ch = self.app.link.get_rc_state().get("channels", [])
        return ch[idx] if idx < len(ch) else 0

    def _capture_sweep(self, idx):
        """Commit the extremes seen since the last reset into the min/max boxes.

        Uses the swept extremes rather than the instantaneous value because the
        operator cannot hold a pot at its stop and click a button at the same
        time; the poll loop records the extremes as they move it.
        """
        mn, mx = self._seen_min[idx], self._seen_max[idx]
        if mn is None or mx is None or mn == mx:
            messagebox.showinfo(
                "Sweep",
                f"Move {self.CH_INFO[idx][0]} through its full travel first, "
                "then press Sweep.")
            return
        self._cal_vars[idx][0].set(str(mn))
        self._cal_vars[idx][2].set(str(mx))

    def _capture_centre(self, idx):
        v = self._live(idx)
        if v <= 0:
            messagebox.showinfo("Centre", f"{self.CH_INFO[idx][0]} is not reporting data.")
            return
        self._cal_vars[idx][1].set(str(v))

    def _channel_values(self, idx):
        try:
            mn = int(float(self._cal_vars[idx][0].get()))
            cn = int(float(self._cal_vars[idx][1].get()))
            mx = int(float(self._cal_vars[idx][2].get()))
        except ValueError:
            return None
        # Same validation the firmware applies, done here too so the operator
        # gets a useful message instead of a bare NAK.
        if not (500 <= mn < cn < mx <= 2500):
            return None
        return mn, cn, mx

    def _apply_channel(self, idx, quiet=False):
        if not self.app.link:
            return False
        vals = self._channel_values(idx)
        if vals is None:
            if not quiet:
                messagebox.showerror(
                    "Calibration",
                    f"{self.CH_INFO[idx][0]}: need 500 <= min < centre < max <= 2500.")
            return False
        self.app.link.set_rc_cal(idx + 1, *vals)
        return True

    def _apply_all(self):
        if not self.app.link:
            return
        bad = [self.CH_INFO[i][0] for i in range(10) if not self._apply_channel(i, quiet=True)]
        if bad:
            messagebox.showwarning(
                "Calibration",
                "Applied the valid channels. Skipped (min<centre<max not satisfied):\n"
                + ", ".join(bad))

    def _read_from_robot(self):
        if self.app.link:
            self.app.link.request_rc_cal()

    def _reset_sweep(self):
        self._seen_min = [None] * 10
        self._seen_max = [None] * 10

    def _reset_robot(self):
        if self.app.link and messagebox.askyesno(
                "Reset calibration",
                "Reset ALL channels on the robot to 1000/1500/2000?"):
            self.app.link.reset_rc_cal()
            self.app.link.request_rc_cal()

    def _save_cal(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rc_calibration.json")
        data = []
        for i in range(10):
            vals = self._channel_values(i)
            data.append({"ch": i + 1,
                         "min": vals[0] if vals else 1000,
                         "cen": vals[1] if vals else 1500,
                         "max": vals[2] if vals else 2000})
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            messagebox.showinfo("Saved", f"RC calibration written to:\n{path}")
        except OSError as e:
            messagebox.showerror("Save error", str(e))

    def _load_cal(self):
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rc_calibration.json")
        try:
            with open(path, encoding="utf-8") as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            messagebox.showerror("Load error", str(e))
            return
        for row in data:
            i = int(row.get("ch", 0)) - 1
            if 0 <= i < 10:
                self._cal_vars[i][0].set(str(row.get("min", 1000)))
                self._cal_vars[i][1].set(str(row.get("cen", 1500)))
                self._cal_vars[i][2].set(str(row.get("max", 2000)))

    # -- tuning senders --------------------------------------------------------
    def _num(self, key):
        try:
            return float(self.tune_vars[key].get())
        except ValueError:
            return None

    def _send(self, key, fn):
        v = self._num(key)
        if v is None:
            messagebox.showerror("Value", "Enter a number.")
            return
        if self.app.link:
            fn(v)

    def _send_rate_min(self):    self._send("rate_min",    lambda v: self.app.link.set_rc_rate_min(v))
    def _send_rate_max(self):    self._send("rate_max",    lambda v: self.app.link.set_rc_rate_max(v))
    def _send_tgt_limit(self):   self._send("tgt_limit",   lambda v: self.app.link.set_rc_target_limit(v))
    def _send_crouch_rate(self): self._send("crouch_rate", lambda v: self.app.link.set_rc_crouch_rate(v))
    def _send_max_crouch(self):  self._send("max_crouch",  lambda v: self.app.link.set_rc_max_crouch(v))
    def _send_max_vel(self):     self._send("max_vel",     lambda v: self.app.link.set_rc_max_vel(v))
    def _send_max_steer(self):   self._send("max_steer",   lambda v: self.app.link.set_rc_max_steer(v))

    def _reset_loop_timer(self):
        if self.app.link:
            self.app.link.reset_loop_timer()

    # -- periodic refresh ------------------------------------------------------
    def update_tab(self):
        if not self.app.link:
            return
        rc = self.app.link.get_rc_state()
        chans = rc.get("channels", [0] * 10)

        self.link_var.set("RC link: OK" if rc.get("link_ok") else "RC link: LOST")
        self.arm_var.set("Armed: YES" if rc.get("armed") else "Armed: no")

        for i in range(min(10, len(chans))):
            v = chans[i]
            self._val_lbls[i].config(text=str(v))
            # Bar spans 1000..2000 us. A dead channel reads 0 and must show an
            # empty bar, not a full-left one - "no data" is not "stick at min".
            self._bars[i]["value"] = 0 if v <= 0 else max(0, min(1000, v - 1000))
            if v > 0:
                if self._seen_min[i] is None or v < self._seen_min[i]:
                    self._seen_min[i] = v
                if self._seen_max[i] is None or v > self._seen_max[i]:
                    self._seen_max[i] = v

        self.dec_vars["drive"].set(f"{rc.get('drive', 0.0):+.2f}")
        self.dec_vars["steer"].set(f"{rc.get('steer', 0.0):+.2f}")
        self.dec_vars["target"].set(f"{rc.get('target', 0.0):+.0f}")
        self.dec_vars["leg"].set(self.LEG_SEL_NAMES.get(rc.get("leg_sel", 0), "?"))
        self.dec_vars["crouch"].set(f"{rc.get('crouch_axis', 0.0):+.2f}")

        lt = self.app.link.get_loop_timing()
        body = lt.get("body_us", 0)
        self.lt_vars["body"].set(str(body))
        self.lt_vars["period"].set(str(lt.get("period_us", 0)))
        self.lt_vars["peak"].set(str(lt.get("peak_us", 0)))
        self.lt_vars["overruns"].set(str(lt.get("overruns", 0)))
        self.lt_vars["budget"].set(f"{body / 100.0:.1f}%")




# ---------------------------------------------------------------------------
# Main Application — shared header + 3-tab Notebook
# ---------------------------------------------------------------------------
class BalanceApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Unified Balance + AX-12 Tuner  (SYNC_WRITE edition)")
        self.geometry("1280x900")
        self.minsize(1100, 750)
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
        self._last_known_offset = None

        self._build_header()

        self.bind("<c>", self._on_key_c)
        self.bind("<C>", self._on_key_c)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.tab_tuner = BalanceTunerTab(self.notebook, self)
        self.tab_ax12  = AX12TunerTab(self.notebook, self)
        self.tab_legs  = KinematicsHealthTab(self.notebook, self)
        self.notebook.add(self.tab_tuner, text="1. Balance Tuner")
        self.notebook.add(self.tab_ax12,  text="2. AX-12 Tuner (SYNC_WRITE)")
        self.notebook.add(self.tab_legs,  text="3. Kinematics & Health")
        self.tab_rc = RCTuneTab(self.notebook, self)
        self.notebook.add(self.tab_rc, text="4. RC Calibration & Tuning")

        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._poll)

    # ── header: connection + actions + status ─────────────────────────────────
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

        ttk.Button(top, text="💾 Save Params", command=self.save_params_with_comment).pack(side=tk.LEFT, padx=8)
        ttk.Button(top, text="Resync",      command=self.resync).pack(side=tk.LEFT, padx=2)

        # right-aligned live status
        ttk.Label(top, textvariable=self.cutoff_var,
                  font=("Consolas", 10, "bold"), foreground="red").pack(side=tk.RIGHT, padx=8)
        ttk.Label(top, textvariable=self.motor_var,
                  font=("Consolas", 10, "bold")).pack(side=tk.RIGHT, padx=8)
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.RIGHT, padx=8)
        ttk.Label(top, textvariable=self.pitch_var,
                  font=("Consolas", 13, "bold"), foreground="#1f77b4").pack(side=tk.RIGHT, padx=12)

    # ── connection / actions ──────────────────────────────────────────────────
    def connect(self):
        if self.link:
            return
        port = self.port_var.get()
        try:
            self.link = SerialLink(port, baud=115200)
            if hasattr(self, "tab_tuner") and hasattr(self.tab_tuner, "auto_backup_var"):
                self.link.set_auto_backup(self.tab_tuner.auto_backup_var.get())
            self.link.connect()     # also sends RB for immediate state resync
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

    def _on_key_c(self, event):
        # Don't trigger calibration if user is typing in an entry box
        if not isinstance(self.focus_get(), (tk.Entry, ttk.Entry)):
            self.calibrate()

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

    def resync(self):
        """Ask firmware to broadcast a full AX12 state line."""
        if self.link:
            self.link.request_state()
            self.status_var.set("State resync requested")

    def save_params_with_comment(self):
        comment = simpledialog.askstring(
            "Save Tuned Parameters",
            "Enter a comment or description for this tuning profile:\n(e.g., 'Stable on carpet, responsive balance')",
            parent=self
        )
        if comment is None:
            # User cancelled dialog
            return

        comment = comment.strip()
        profiles_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "profiles")
        os.makedirs(profiles_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename  = os.path.join(profiles_dir, f"params_{timestamp}.json")

        balance_params = {key: slider.get_value() for key, slider in self.tab_tuner.sliders.items()}

        # Also capture AX-12 compliance / speed / IK foot targets if available
        ax12_settings = {}
        if hasattr(self, "tab_ax12") and hasattr(self.tab_ax12, "cmd_sliders"):
            for k, s in self.tab_ax12.cmd_sliders.items():
                ax12_settings[k] = s.get_value()

        ik_targets = {}
        if hasattr(self, "tab_ax12") and hasattr(self.tab_ax12, "ik_sliders"):
            for k, s in self.tab_ax12.ik_sliders.items():
                ik_targets[k] = s.get_value()

        data = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "comment": comment,
            "params": balance_params,
            "ax12_settings": ax12_settings,
            "ik_targets": ik_targets,
        }
        # Flat keys for backward compatibility
        for k, v in balance_params.items():
            data[k] = v

        try:
            with open(filename, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=4)
            msg = f"Saved to profiles/params_{timestamp}.json"
            if comment:
                msg += f" ('{comment[:25]}...')" if len(comment) > 25 else f" ('{comment}')"
            self.status_var.set(msg)
            messagebox.showinfo(
                "Profile Saved",
                f"Tuned parameters saved to:\n\n{filename}\n\nComment: \"{comment if comment else '(none)'}\""
            )
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    def save_params(self):
        self.save_params_with_comment()

    # ── periodic refresh — only the active tab does heavy redraw work ─────────
    def _poll(self):
        if self.link:
            self.motor_var.set(f"Motors: {'ON' if self.link.motors_on else 'OFF'}")
            self.cutoff_var.set("Safety: LATCHED!" if self.link.cutoff_since() else "Safety: clear")
            self.auto_trim_var.set(self.link.auto_trim_on)

            offset_val = self.link.fw.get("pitchOffset")
            if offset_val is not None:
                self.offset_var.set(f"Offset: {offset_val:.4f}")
                if offset_val != self._last_known_offset:
                    self._last_known_offset = offset_val
                    # Don't overwrite if the user is currently typing a new offset
                    if not isinstance(self.focus_get(), (tk.Entry, ttk.Entry)):
                        self.offset_entry_var.set(f"{offset_val:.4f}")

            active_tab = self.notebook.index(self.notebook.select())
            if active_tab == 0:
                self.tab_tuner.update_tab()
            elif active_tab == 1:
                self.tab_ax12.update_tab()
            elif active_tab == 2:
                self.tab_legs.update_tab()
            elif active_tab == 3:
                self.tab_rc.update_tab()

        self.after(100, self._poll)

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = BalanceApp()
    app.mainloop()
