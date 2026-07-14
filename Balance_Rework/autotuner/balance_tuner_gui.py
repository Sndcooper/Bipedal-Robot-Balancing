"""Balance_Rework GUI tuner.

Desktop control panel for the reworked biped firmware.

Features:
    - Connects to COM3 @ 115200 by default.
    - Streams firmware telemetry and raw serial-monitor output in the app.
    - Provides coarse/fine sliders for Kp, Ki, Kd, Kd_vel, alpha, targetAngle,
      pitchOffset, and maxSafeTilt.
    - Uses the existing SerialLink backend so command parsing stays consistent
      with the autotuner.

Run:
    python balance_tuner_gui.py --port COM3
"""

from __future__ import annotations

import argparse
import math
import threading
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

import tkinter as tk
from tkinter import ttk, messagebox

from serial_link import SerialLink

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


BAUD_RATE = 115200
POLL_MS = 100
LOG_LIMIT = 500


@dataclass(frozen=True)
class ParamSpec:
    key: str
    label: str
    coarse_min: float
    coarse_max: float
    coarse_step: float
    fine_span: float
    fine_step: float
    digits: int
    command: str
    start_value: float


PARAM_SPECS: List[ParamSpec] = [
    ParamSpec("Kp", "Kp", 0.0, 100.0, 0.1, 5.0, 0.01, 3, "gain", 11.21),
    ParamSpec("Ki", "Ki", 0.0, 10.0, 0.1, 2.0, 0.01, 3, "gain", 0.0),
    ParamSpec("Kd", "Kd", 0.0, 40.0, 0.1, 5.0, 0.01, 3, "gain", 0.715),
    ParamSpec("Kd_vel", "Kd_vel", 0.0, 10.0, 0.1, 1.0, 0.01, 3, "gain", 0.0),
    ParamSpec("alpha", "alpha", 0.80, 0.999, 0.001, 0.02, 0.0001, 4, "gain", 0.96),
    ParamSpec("targetAngle", "Target", -20.0, 20.0, 0.1, 5.0, 0.01, 3, "target", 0.0),
    ParamSpec("pitchOffset", "Offset", -20.0, 20.0, 0.1, 5.0, 0.01, 3, "offset", 0.0),
    ParamSpec("maxSafeTilt", "Tilt", 5.0, 50.0, 0.1, 5.0, 0.01, 2, "tilt", 35.0),
]


class CoarseFineSlider(ttk.Frame):
    def __init__(
        self,
        master,
        spec: ParamSpec,
        initial_value: float,
        send_value: Callable[[str, float], None],
    ):
        super().__init__(master)
        self.spec = spec
        self.send_value = send_value
        self._debounce_id: Optional[str] = None
        self._zoom = tk.BooleanVar(value=False)
        self._value = tk.DoubleVar(value=initial_value)
        self._entry_value = tk.StringVar(value=self._fmt_value(initial_value))
        self._last_sent = initial_value
        self._user_dragging = False
        self._awaiting_echo = False
        self._entry_focused = False

        self._title = ttk.Label(self, text=spec.label)
        self._title.grid(row=0, column=0, sticky="w")

        self._value_label = ttk.Label(self, text=self._fmt_value(initial_value))
        self._value_label.grid(row=0, column=1, sticky="e")

        self._entry = ttk.Entry(self, textvariable=self._entry_value, width=10)
        self._entry.grid(row=0, column=2, sticky="e", padx=(8, 0))
        self._entry.bind("<FocusIn>", self._on_entry_focus_in)
        self._entry.bind("<FocusOut>", self._on_entry_focus_out)
        self._entry.bind("<Return>", self._on_entry_commit)

        self._scale = tk.Scale(
            self,
            orient=tk.HORIZONTAL,
            showvalue=False,
            resolution=spec.coarse_step,
            from_=spec.coarse_min,
            to=spec.coarse_max,
            variable=self._value,
            command=self._on_change,
            length=280,
        )
        self._scale.grid(row=1, column=0, columnspan=2, sticky="ew", padx=(0, 8))
        self._scale.bind("<ButtonPress-1>", self._on_press)
        self._scale.bind("<ButtonRelease-1>", self._on_release)

        self._zoom_box = ttk.Checkbutton(
            self,
            text="Zoom",
            variable=self._zoom,
            command=self._apply_zoom_mode,
        )
        self._zoom_box.grid(row=2, column=0, sticky="w")

        self._send_now_btn = ttk.Button(self, text="Set", command=self.apply_entry_value)
        self._send_now_btn.grid(row=2, column=1, sticky="e")

        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=0)
        self.columnconfigure(2, weight=0)
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
        self.send_value(self.spec.key, value)

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

    def has_pending_update(self) -> bool:
        return self._user_dragging or self._awaiting_echo or self._entry_focused

    def mark_echo_received(self, value: float):
        value = self._clamp(self._quantize(value))
        if abs(value - self._last_sent) < 1e-9:
            self._awaiting_echo = False


class SerialLogPanel(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.text = tk.Text(self, height=18, wrap=tk.NONE, state=tk.DISABLED)
        yscroll = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.text.yview)
        xscroll = ttk.Scrollbar(self, orient=tk.HORIZONTAL, command=self.text.xview)
        self.text.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)

        self.text.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self._seen = 0

    def append_lines(self, lines: List[str]):
        if not lines:
            return
        self.text.configure(state=tk.NORMAL)
        for line in lines:
            self.text.insert(tk.END, line + "\n")
        self.text.see(tk.END)
        self.text.configure(state=tk.DISABLED)


class BipedTunerApp(tk.Tk):
    def __init__(self, port: str):
        super().__init__()
        self.title("Balance_Rework Tuner")
        self.geometry("1440x900")
        self.minsize(1200, 800)

        self.port_var = tk.StringVar(value=port)
        self.status_var = tk.StringVar(value="Disconnected")
        self.connection_var = tk.StringVar(value="COM3 @ 115200")
        self.fw_var = tk.StringVar(value="P:— I:— D:— Kd_vel:— alpha:— Tilt:— Target:— Offset:—")
        self.telemetry_var = tk.StringVar(value="PITCH:— PID_OUT:— VEL:— ENC_L:— ENC_R:—")
        self.motor_var = tk.StringVar(value="Motors: OFF")
        self.cutoff_var = tk.StringVar(value="Safety: clear")

        self.link: Optional[SerialLink] = None
        self._polling = False
        self._log_cursor = 0
        self._plot_max = 250
        self._plot_history: Dict[str, List[float]] = {
            "t": [],
            "pitch": [],
            "pid_out": [],
            "vel": [],
        }

        self._build_styles()
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._poll)

    def _build_styles(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except Exception:
            pass
        style.configure("Header.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("Status.TLabel", font=("Consolas", 10))
        style.configure("Control.TLabelframe", padding=8)
        style.configure("Control.TLabelframe.Label", font=("Segoe UI", 10, "bold"))

    def _build_ui(self):
        root = ttk.Frame(self, padding=10)
        root.pack(fill=tk.BOTH, expand=True)

        top = ttk.Frame(root)
        top.pack(fill=tk.X, pady=(0, 8))

        ttk.Label(top, text="Balance Rework Tuner", style="Header.TLabel").pack(side=tk.LEFT)
        ttk.Label(top, textvariable=self.connection_var, style="Status.TLabel").pack(side=tk.LEFT, padx=18)
        ttk.Label(top, textvariable=self.status_var, style="Status.TLabel").pack(side=tk.LEFT, padx=18)

        conn = ttk.Frame(root)
        conn.pack(fill=tk.X, pady=(0, 10))

        ttk.Label(conn, text="Port").pack(side=tk.LEFT)
        ttk.Entry(conn, textvariable=self.port_var, width=12).pack(side=tk.LEFT, padx=(6, 8))
        ttk.Button(conn, text="Connect", command=self.connect).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Disconnect", command=self.disconnect).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Calibrate IMU", command=self.calibrate).pack(side=tk.LEFT, padx=12)
        ttk.Button(conn, text="Motors On/Off", command=self.toggle_motors).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Zero Gains", command=self.zero_gains).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Clear Log", command=self.clear_log).pack(side=tk.LEFT, padx=4)
        ttk.Button(conn, text="Safety Reset", command=self.safety_reset).pack(side=tk.LEFT, padx=4)

        summary = ttk.Frame(root)
        summary.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(summary, textvariable=self.fw_var, style="Status.TLabel").pack(anchor=tk.W)
        ttk.Label(summary, textvariable=self.telemetry_var, style="Status.TLabel").pack(anchor=tk.W)
        ttk.Label(summary, textvariable=self.motor_var, style="Status.TLabel").pack(anchor=tk.W)
        ttk.Label(summary, textvariable=self.cutoff_var, style="Status.TLabel").pack(anchor=tk.W)

        body = ttk.Frame(root)
        body.pack(fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=2)
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")

        self._build_plot(left)

        controls = ttk.LabelFrame(right, text="Tunable Parameters", style="Control.TLabelframe")
        controls.pack(fill=tk.X, pady=(0, 10))

        self.controls: Dict[str, CoarseFineSlider] = {}
        initial_map = {spec.key: spec.start_value for spec in PARAM_SPECS}
        for spec in PARAM_SPECS:
            ctrl = CoarseFineSlider(controls, spec, initial_map[spec.key], self._send_param)
            ctrl.pack(fill=tk.X, pady=6)
            self.controls[spec.key] = ctrl

        log_frame = ttk.LabelFrame(right, text="Serial Monitor", style="Control.TLabelframe")
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_panel = SerialLogPanel(log_frame)
        self.log_panel.pack(fill=tk.BOTH, expand=True)

    def _build_plot(self, parent):
        frame = ttk.LabelFrame(parent, text="Live Telemetry", style="Control.TLabelframe")
        frame.pack(fill=tk.BOTH, expand=True)

        fig = Figure(figsize=(8, 5), dpi=100)
        fig.patch.set_facecolor("white")
        self.ax = fig.add_subplot(111)
        self.ax2 = self.ax.twinx()
        self.ax.set_title("Pitch, target, and output response")
        self.ax.set_xlabel("sample")
        self.ax.set_ylabel("deg")
        self.ax2.set_ylabel("PID / velocity")
        self.ax.grid(True, alpha=0.25)

        (self.pitch_line,) = self.ax.plot([], [], color="#1f77b4", lw=1.8, label="pitch")
        (self.target_line,) = self.ax.plot([], [], color="#2ca02c", lw=1.2, ls="--", label="target")
        (self.pid_line,) = self.ax2.plot([], [], color="#ff7f0e", lw=1.1, alpha=0.9, label="pid_out")
        (self.vel_line,) = self.ax2.plot([], [], color="#9467bd", lw=1.0, alpha=0.7, label="vel")

        lines = [self.pitch_line, self.target_line, self.pid_line, self.vel_line]
        self.ax.legend(lines, [line.get_label() for line in lines], loc="upper left")

        self.canvas = FigureCanvasTkAgg(fig, master=frame)
        self.canvas.draw()
        widget = self.canvas.get_tk_widget()
        widget.pack(fill=tk.BOTH, expand=True)

    def _format_fw_summary(self, fw: Dict[str, Optional[float]]) -> str:
        kp = fw.get("Kp")
        ki = fw.get("Ki")
        kd = fw.get("Kd")
        kd_vel = fw.get("Kd_vel")
        alpha = fw.get("alpha")
        tilt = fw.get("tilt")
        target = fw.get("target")
        offset = fw.get("offset")
        return (
            f"P:{self._fmt(kp, 2)} I:{self._fmt(ki, 2)} D:{self._fmt(kd, 2)} "
            f"Kd_vel:{self._fmt(kd_vel, 3)} alpha:{self._fmt(alpha, 4)} "
            f"Tilt:{self._fmt(tilt, 1)} Target:{self._fmt(target, 2)} Offset:{self._fmt(offset, 2)}"
        )

    def _fmt(self, value, digits: int) -> str:
        if value is None:
            return "—"
        try:
            return f"{float(value):.{digits}f}"
        except Exception:
            return "—"

    def _ensure_link(self) -> Optional[SerialLink]:
        if self.link is not None:
            return self.link
        return None

    def connect(self):
        if self.link is not None:
            return
        port = self.port_var.get().strip() or "COM3"
        try:
            self.link = SerialLink(port, baud=BAUD_RATE)
            self.link.connect()
            self.status_var.set(f"Connected to {port}")
            self.connection_var.set(f"{port} @ {BAUD_RATE}")
            self._log_cursor = 0
        except Exception as exc:
            self.link = None
            messagebox.showerror("Connection failed", str(exc))
            self.status_var.set("Connection failed")

    def disconnect(self):
        if self.link is None:
            return
        try:
            self.link.close()
        finally:
            self.link = None
            self.status_var.set("Disconnected")
            self.connection_var.set("Disconnected")

    def _send_gain_bundle(self):
        if self.link is None:
            return
        self.link.set_gains(
            self.controls["Kp"].get_value(),
            self.controls["Ki"].get_value(),
            self.controls["Kd"].get_value(),
            self.controls["Kd_vel"].get_value(),
            self.controls["alpha"].get_value(),
        )

    def _send_param(self, key: str, value: float):
        if self.link is None:
            return
        if key == "Kp":
            self.link.set_kp(value)
        elif key == "Ki":
            self.link.set_ki(value)
        elif key == "Kd":
            self.link.set_kd(value)
        elif key == "Kd_vel":
            self.link.set_kd_vel(value)
        elif key == "alpha":
            self.link.set_alpha(value)
        else:
            if key == "targetAngle":
                self.link.set_target(value)
            elif key == "pitchOffset":
                self.link.set_offset(value)
            elif key == "maxSafeTilt":
                self.link.set_tilt(value)
            return

    def calibrate(self):
        if self.link is None:
            self.connect()
        if self.link is None:
            return
        self.link.calibrate()
        self.status_var.set("Calibration command sent")

    def toggle_motors(self):
        if self.link is None:
            self.connect()
        if self.link is None:
            return
        self.link.toggle_motors()
        self.status_var.set("Motor toggle sent")

    def safety_reset(self):
        if self.link is None:
            return
        self.link.arm_cutoff_watch()
        self.status_var.set("Safety latch cleared")

    def zero_gains(self):
        for key in ("Kp", "Ki", "Kd", "Kd_vel"):
            self.controls[key].set_value(0.0)
        self.controls["alpha"].set_value(0.96)
        self.controls["targetAngle"].set_value(0.0)
        self.controls["pitchOffset"].set_value(0.0)
        self._send_gain_bundle()
        if self.link is not None:
            self.link.set_target(0.0)
            self.link.set_offset(0.0)
        self.status_var.set("Zeroed gains")

    def clear_log(self):
        self.log_panel.text.configure(state=tk.NORMAL)
        self.log_panel.text.delete("1.0", tk.END)
        self.log_panel.text.configure(state=tk.DISABLED)
        if self.link is not None:
            self.link.clear_log()
        self._log_cursor = 0

    def _append_new_log_lines(self):
        if self.link is None:
            return
        lines = self.link.recent_lines(limit=LOG_LIMIT)
        if self._log_cursor > len(lines):
            self._log_cursor = 0
        new_lines = lines[self._log_cursor:]
        self._log_cursor = len(lines)
        self.log_panel.append_lines(new_lines)

    def _update_controls_from_fw(self):
        if self.link is None:
            return
        fw = self.link.fw
        mapping = {
            "Kp": "Kp",
            "Ki": "Ki",
            "Kd": "Kd",
            "Kd_vel": "Kd_vel",
            "alpha": "alpha",
            "tilt": "maxSafeTilt",
            "target": "targetAngle",
            "offset": "pitchOffset",
        }
        for fw_key, ctrl_key in mapping.items():
            value = fw.get(fw_key)
            if value is None:
                continue
            ctrl = self.controls.get(ctrl_key)
            if ctrl is None:
                continue
            ctrl.mark_echo_received(float(value))
            if ctrl.has_pending_update():
                continue
            ctrl.sync_from_external(float(value))

    def _update_plot(self):
        if self.link is None:
            return
        snap = self.link.snapshot()
        if not snap["t"]:
            return

        self._plot_history["t"] = snap["t"][-self._plot_max:]
        self._plot_history["pitch"] = snap["pitch"][-self._plot_max:]
        self._plot_history["pid_out"] = snap["pid_out"][-self._plot_max:]
        self._plot_history["vel"] = snap["vel"][-self._plot_max:]

        n = len(self._plot_history["pitch"])
        x = list(range(n))
        pitch = self._plot_history["pitch"]
        pid_out = self._plot_history["pid_out"]
        vel = self._plot_history["vel"]
        target = self.link.fw.get("target")
        if target is None:
            target = 0.0

        self.pitch_line.set_data(x, pitch)
        self.target_line.set_data(x, [float(target)] * n)
        self.pid_line.set_data(x, pid_out)
        self.vel_line.set_data(x, vel)

        left_values = pitch + [float(target)]
        left_min = min(left_values) if left_values else -5.0
        left_max = max(left_values) if left_values else 5.0
        if math.isclose(left_min, left_max):
            left_min -= 1.0
            left_max += 1.0
        left_pad = max(2.0, (left_max - left_min) * 0.2)
        self.ax.set_xlim(0, max(1, n - 1))
        self.ax.set_ylim(left_min - left_pad, left_max + left_pad)

        right_values = pid_out + vel
        if right_values:
            right_min = min(right_values)
            right_max = max(right_values)
            if math.isclose(right_min, right_max):
                right_min -= 1.0
                right_max += 1.0
            right_pad = max(5.0, (right_max - right_min) * 0.2)
            self.ax2.set_ylim(right_min - right_pad, right_max + right_pad)

        self.canvas.draw_idle()

    def _poll(self):
        if self.link is not None:
            self._append_new_log_lines()
            self._update_controls_from_fw()
            self._update_plot()
            self.telemetry_var.set(self._format_telemetry())
            self.fw_var.set(self._format_fw_summary(self.link.fw))
            self.motor_var.set(f"Motors: {'ON' if self.link.motors_on else 'OFF'}")
            self.cutoff_var.set(
                f"Safety: {'latched' if self.link.cutoff_since() else 'clear'}"
            )
        else:
            self.telemetry_var.set("PITCH:— PID_OUT:— VEL:— ENC_L:— ENC_R:—")
            self.fw_var.set("P:— I:— D:— Kd_vel:— alpha:— Tilt:— Target:— Offset:—")
            self.motor_var.set("Motors: OFF")
            self.cutoff_var.set("Safety: clear")

        self.after(POLL_MS, self._poll)

    def _format_telemetry(self) -> str:
        if self.link is None:
            return "PITCH:— PID_OUT:— VEL:— ENC_L:— ENC_R:—"
        snap = self.link.snapshot()
        if not snap["pitch"]:
            return "PITCH:— PID_OUT:— VEL:— ENC_L:— ENC_R:—"
        pitch = snap["pitch"][-1]
        pid_out = snap["pid_out"][-1]
        vel = snap["vel"][-1]
        enc_l = snap["enc_l"][-1]
        enc_r = snap["enc_r"][-1]
        return (
            f"PITCH:{pitch:+.2f} PID_OUT:{pid_out:+.2f} VEL:{vel:+.2f} "
            f"ENC_L:{enc_l} ENC_R:{enc_r}"
        )

    def on_close(self):
        try:
            self.disconnect()
        finally:
            self.destroy()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Balance_Rework COM3 tuner GUI")
    parser.add_argument("--port", default="COM3", help="Serial port, default: COM3")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    app = BipedTunerApp(args.port)
    app.mainloop()


if __name__ == "__main__":
    main()