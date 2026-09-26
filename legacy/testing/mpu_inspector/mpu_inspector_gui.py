#!/usr/bin/env python3
"""
Balance_Rework MPU6050 & Motor Telemetry Inspector GUI
-------------------------------------------------------
Real-time diagnostic GUI for checking MPU6050 signal health, complementary filter
performance, motor effort, and physical orientation from Balance_Rework firmware.

Features:
  - Live physical robot horizon & wheel visualizer canvas
  - 3-channel real-time Matplotlib plots (Pitch angle, Motor effort/Velocity, Encoders)
  - Real-time diagnostic statistics: Loop frequency (Hz), RMS Noise (sigma), Status badge
  - Interactive serial controls: Calibrate IMU ('C'), Enable/Disable Motors ('M'), PID/Alpha Tuning
  - Built-in Simulation Mode (--mock or UI toggle) for testing without hardware plugged in
"""

import sys
import time
import math
import argparse
import threading
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox

# Matplotlib embedding
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


class MPUInspectorGUI:
    def __init__(self, root, default_port="", baudrate=115200, use_mock=False):
        self.root = root
        self.root.title("Bipedal Robot — MPU6050 & Motor Inspector (Balance_Rework)")
        self.root.geometry("1380x880")
        self.root.configure(bg="#1e1e24")

        # Configure style
        self.style = ttk.Style()
        self.style.theme_use("clam")
        self.style.configure("TFrame", background="#1e1e24")
        self.style.configure("TLabel", background="#1e1e24", foreground="#e0e0e0", font=("Segoe UI", 10))
        self.style.configure("Title.TLabel", font=("Segoe UI", 13, "bold"), foreground="#00e5ff")
        self.style.configure("StatValue.TLabel", font=("Segoe UI", 16, "bold"), foreground="#4caf50")
        self.style.configure("TButton", font=("Segoe UI", 10, "bold"), padding=6)

        # Connection / Threading
        self.serial_conn = None
        self.running = False
        self.read_thread = None
        self.is_mock = use_mock

        # Data buffers (last 200 samples ~ 10 seconds at 20 Hz)
        self.max_len = 200
        self.timestamps = deque(maxlen=self.max_len)
        self.pitch_data = deque(maxlen=self.max_len)
        self.pid_out_data = deque(maxlen=self.max_len)
        self.vel_data = deque(maxlen=self.max_len)
        self.enc_l_data = deque(maxlen=self.max_len)
        self.enc_r_data = deque(maxlen=self.max_len)

        # Live state
        self.current_pitch = 0.0
        self.current_pid_out = 0.0
        self.current_vel = 0.0
        self.current_enc_l = 0
        self.current_enc_r = 0
        self.current_alpha = 0.98
        self.current_max_tilt = 35.0
        self.safety_cutoff_triggered = False
        self.start_time = time.time()

        # Stats tracking
        self.packet_count = 0
        self.last_stat_time = time.time()
        self.measured_hz = 0.0
        self.noise_rms = 0.0

        self.setup_ui()

        if default_port and not use_mock:
            self.connect_serial(default_port, baudrate)
        elif use_mock:
            self.start_mock_stream()

        # UI refresh loop
        self.root.after(50, self.refresh_dashboard)

    def setup_ui(self):
        # Top Header Bar
        top_bar = ttk.Frame(self.root, padding="10 8 10 8")
        top_bar.pack(side=tk.TOP, fill=tk.X)

        title_lbl = ttk.Label(top_bar, text="🤖 MPU6050 & MOTOR REAL-TIME INSPECTOR", style="Title.TLabel")
        title_lbl.pack(side=tk.LEFT, padx=5)

        # Port Selection / Controls
        port_frame = ttk.Frame(top_bar)
        port_frame.pack(side=tk.RIGHT)

        ttk.Label(port_frame, text="COM Port:").pack(side=tk.LEFT, padx=4)
        self.port_combo = ttk.Combobox(port_frame, width=12, state="readonly")
        self.refresh_ports()
        self.port_combo.pack(side=tk.LEFT, padx=4)

        refresh_btn = ttk.Button(port_frame, text="🔄 Ports", command=self.refresh_ports)
        refresh_btn.pack(side=tk.LEFT, padx=2)

        self.connect_btn = ttk.Button(port_frame, text="Connect Hardware COM", command=self.toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=6)

        self.mock_btn = ttk.Button(port_frame, text="Simulate Demo Data (5° Sine)", command=self.toggle_mock)
        self.mock_btn.pack(side=tk.LEFT, padx=4)

        # Main Layout split: Left (Canvas + Stats + Command Panel), Right (Graphs)
        content_frame = ttk.Frame(self.root, padding=10)
        content_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left_col = ttk.Frame(content_frame, width=380)
        left_col.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))

        right_col = ttk.Frame(content_frame)
        right_col.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- LEFT COLUMN: Physical Visualizer + Diagnostics + Controls ---
        vis_card = ttk.LabelFrame(left_col, text=" Actual Physical Robot View ", padding=8)
        vis_card.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))

        self.canvas = tk.Canvas(vis_card, width=360, height=230, bg="#121217", highlightthickness=1, highlightbackground="#333")
        self.canvas.pack()

        # Diagnostics & Health Card
        diag_card = ttk.LabelFrame(left_col, text=" MPU6050 Signal Diagnostics & Health ", padding=10)
        diag_card.pack(side=tk.TOP, fill=tk.X, pady=(0, 10))

        grid_f = ttk.Frame(diag_card)
        grid_f.pack(fill=tk.X)

        ttk.Label(grid_f, text="Update Rate:").grid(row=0, column=0, sticky=tk.W, pady=3)
        self.hz_label = ttk.Label(grid_f, text="0.0 Hz", style="StatValue.TLabel")
        self.hz_label.grid(row=0, column=1, sticky=tk.E, pady=3)

        ttk.Label(grid_f, text="Signal Noise (RMS σ):").grid(row=1, column=0, sticky=tk.W, pady=3)
        self.noise_label = ttk.Label(grid_f, text="0.00°", style="StatValue.TLabel")
        self.noise_label.grid(row=1, column=1, sticky=tk.E, pady=3)

        ttk.Label(grid_f, text="Noise Rating:").grid(row=2, column=0, sticky=tk.W, pady=3)
        self.quality_label = ttk.Label(grid_f, text="Waiting...", font=("Segoe UI", 11, "bold"), foreground="#ffb300")
        self.quality_label.grid(row=2, column=1, sticky=tk.E, pady=3)

        ttk.Label(grid_f, text="Safety Cutoff:").grid(row=3, column=0, sticky=tk.W, pady=3)
        self.safety_badge = ttk.Label(grid_f, text="SAFE", font=("Segoe UI", 12, "bold"), foreground="#4caf50")
        self.safety_badge.grid(row=3, column=1, sticky=tk.E, pady=3)

        # Interactive Command Panel
        cmd_card = ttk.LabelFrame(left_col, text=" Firmware Controls & Calibration ", padding=10)
        cmd_card.pack(side=tk.TOP, fill=tk.X)

        btn_grid = ttk.Frame(cmd_card)
        btn_grid.pack(fill=tk.X, pady=4)

        ttk.Button(btn_grid, text="⚖️ Calibrate MPU ('C')", command=self.send_calibrate).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(btn_grid, text="⚡ Toggle Motors ('M')", command=self.send_toggle_motors).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        tune_f = ttk.Frame(cmd_card)
        tune_f.pack(fill=tk.X, pady=(10, 0))

        ttk.Label(tune_f, text="Live Tuning Command:").pack(anchor=tk.W)
        row_send = ttk.Frame(tune_f)
        row_send.pack(fill=tk.X, pady=4)

        self.cmd_entry = ttk.Entry(row_send)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
        self.cmd_entry.insert(0, "A0.98")
        ttk.Button(row_send, text="Send", command=self.send_custom_cmd).pack(side=tk.RIGHT)

        # --- RIGHT COLUMN: Real-Time Matplotlib Charts ---
        self.fig = Figure(figsize=(8, 7), facecolor="#1e1e24")
        self.fig.subplots_adjust(left=0.08, right=0.96, top=0.94, bottom=0.07, hspace=0.35)

        # Subplot 1: Pitch Angle vs Target & Safety Limits
        self.ax_pitch = self.fig.add_subplot(311, facecolor="#121217")
        self.ax_pitch.set_title("MPU6050 Fused Pitch Angle (deg) vs Safety Cutoff Envelope", color="#e0e0e0", fontsize=10)
        self.line_pitch, = self.ax_pitch.plot([], [], color="#00e5ff", lw=2, label="Pitch (°)")
        self.ax_pitch.set_ylim(-45, 45)
        self.ax_pitch.tick_params(colors="#aaaaaa")
        self.ax_pitch.grid(True, color="#2a2a35", linestyle="--")

        # Subplot 2: Motor Control Effort & Velocity
        self.ax_motor = self.fig.add_subplot(312, facecolor="#121217")
        self.ax_motor.set_title("PID Control Effort (PID_OUT) & Encoder Velocity Damping (VEL)", color="#e0e0e0", fontsize=10)
        self.line_pid, = self.ax_motor.plot([], [], color="#ff9800", lw=1.8, label="PID_OUT (-255 to 255)")
        self.line_vel, = self.ax_motor.plot([], [], color="#8bc34a", lw=1.5, label="Wheel VEL (ticks/s)")
        self.ax_motor.set_ylim(-300, 300)
        self.ax_motor.tick_params(colors="#aaaaaa")
        self.ax_motor.grid(True, color="#2a2a35", linestyle="--")
        self.ax_motor.legend(loc="upper right", facecolor="#1e1e24", edgecolor="#444", labelcolor="#e0e0e0", fontsize=8)

        # Subplot 3: Wheel Encoders Odometry
        self.ax_enc = self.fig.add_subplot(313, facecolor="#121217")
        self.ax_enc.set_title("Quadrature Encoder Ticks (Left vs Right Odometry)", color="#e0e0e0", fontsize=10)
        self.line_encl, = self.ax_enc.plot([], [], color="#e91e63", lw=1.5, label="ENC_L")
        self.line_encr, = self.ax_enc.plot([], [], color="#03a9f4", lw=1.5, label="ENC_R")
        self.ax_enc.tick_params(colors="#aaaaaa")
        self.ax_enc.grid(True, color="#2a2a35", linestyle="--")
        self.ax_enc.legend(loc="upper left", facecolor="#1e1e24", edgecolor="#444", labelcolor="#e0e0e0", fontsize=8)

        self.canvas_plot = FigureCanvasTkAgg(self.fig, master=right_col)
        self.canvas_plot.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def refresh_ports(self):
        if not SERIAL_AVAILABLE:
            self.port_combo["values"] = ["No PySerial"]
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_combo.get():
            self.port_combo.current(0)

    def toggle_connect(self):
        if self.running and self.serial_conn:
            self.disconnect()
        else:
            port = self.port_combo.get()
            if not port:
                messagebox.showerror("Error", "Please select a valid COM port.")
                return
            self.connect_serial(port, 115200)

    def connect_serial(self, port, baudrate=115200):
        try:
            self.serial_conn = serial.Serial(port, baudrate, timeout=1.0)
            self.running = True
            self.is_mock = False
            self.connect_btn.config(text="Disconnect")
            self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
            self.read_thread.start()
        except Exception as e:
            messagebox.showerror("Serial Error", f"Failed to connect to {port}:\n{e}")

    def disconnect(self):
        self.running = False
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except Exception:
                pass
            self.serial_conn = None
        self.connect_btn.config(text="Connect")

    def toggle_mock(self):
        if self.is_mock:
            self.running = False
            self.is_mock = False
            self.mock_btn.config(text="Simulate Data")
        else:
            self.disconnect()
            self.start_mock_stream()
            self.mock_btn.config(text="Stop Simulation")

    def start_mock_stream(self):
        self.is_mock = True
        self.running = True
        self.read_thread = threading.Thread(target=self.mock_data_loop, daemon=True)
        self.read_thread.start()

    def serial_read_loop(self):
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_telemetry_line(line)
            except Exception:
                break

    def mock_data_loop(self):
        """Simulates realistic balancing biped telemetry for UI testing."""
        t0 = time.time()
        enc_l, enc_r = 0, 0
        while self.running and self.is_mock:
            t = time.time() - t0
            # Realistic sway + minor noise
            pitch = 4.0 * math.sin(t * 1.8) + 0.5 * math.sin(t * 12.0)
            pid_out = -11.2 * pitch - 5.0 * math.cos(t * 1.8)
            vel = 12.0 * math.sin(t * 1.8)
            enc_l += int(vel * 0.05)
            enc_r += int(vel * 0.05)

            line = f"PITCH:{pitch:.2f}, PID_OUT:{pid_out:.2f}, ENC_L:{enc_l}, ENC_R:{enc_r}, VEL:{vel:.2f}, KDVEL:0.0000, ALPHA:0.9800, TILT:35.00"
            self.parse_telemetry_line(line)
            time.sleep(0.05) # 20 Hz simulation

    def parse_telemetry_line(self, line):
        if "SAFETY CUTOFF TRIGGERED" in line:
            self.safety_cutoff_triggered = True
            return

        # Expected format:
        # PITCH:<v>, PID_OUT:<v>, ENC_L:<v>, ENC_R:<v>, VEL:<v>, KDVEL:<v>, ALPHA:<v>, TILT:<v>
        try:
            parts = [p.strip() for p in line.split(",") if ":" in p]
            if len(parts) < 4:
                return
            data = {}
            for item in parts:
                k, v = item.split(":", 1)
                data[k.strip()] = float(v.strip())

            now = time.time() - self.start_time
            pitch = data.get("PITCH", 0.0)
            pid_out = data.get("PID_OUT", 0.0)
            enc_l = int(data.get("ENC_L", 0))
            enc_r = int(data.get("ENC_R", 0))
            vel = data.get("VEL", 0.0)

            self.timestamps.append(now)
            self.pitch_data.append(pitch)
            self.pid_out_data.append(pid_out)
            self.vel_data.append(vel)
            self.enc_l_data.append(enc_l)
            self.enc_r_data.append(enc_r)

            self.current_pitch = pitch
            self.current_pid_out = pid_out
            self.current_vel = vel
            self.current_enc_l = enc_l
            self.current_enc_r = enc_r
            self.current_alpha = data.get("ALPHA", 0.98)
            self.current_max_tilt = data.get("TILT", 35.0)

            self.packet_count += 1
            if abs(pitch) < self.current_max_tilt:
                self.safety_cutoff_triggered = False
        except Exception:
            pass

    def send_command(self, cmd):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write((cmd + "\n").encode('utf-8'))
            except Exception as e:
                print(f"Send error: {e}")

    def send_calibrate(self):
        self.send_command("C")

    def send_toggle_motors(self):
        self.send_command("M")
        self.safety_cutoff_triggered = False

    def send_custom_cmd(self):
        cmd = self.cmd_entry.get().strip()
        if cmd:
            self.send_command(cmd)

    def draw_physical_visualizer(self):
        """Draws an animated 2D view of the bipedal robot tilting with MPU pitch angle."""
        self.canvas.delete("all")

        cx, cy = 180, 140
        pitch_rad = math.radians(self.current_pitch)

        # Draw Safety Envelope Fan (Green/Orange zone)
        max_rad = math.radians(self.current_max_tilt)
        r_env = 110
        self.canvas.create_arc(cx - r_env, cy - r_env, cx + r_env, cy + r_env,
                               start=90 - self.current_max_tilt, extent=self.current_max_tilt * 2,
                               fill="#1c2b23", outline="#2e7d32", width=1)

        # Ground level
        self.canvas.create_line(20, cy + 50, 340, cy + 50, fill="#444", width=2)

        # Robot Wheels (at ground contact)
        wx = cx
        wy = cy + 40
        self.canvas.create_oval(wx - 24, wy - 24, wx + 24, wy + 24, fill="#2a2a33", outline="#00e5ff", width=2)

        # Rotate robot body around wheel axis
        body_length = 85
        hx = wx + body_length * math.sin(pitch_rad)
        hy = wy - body_length * math.cos(pitch_rad)

        # Leg / Body Linkage
        line_color = "#f44336" if abs(self.current_pitch) > self.current_max_tilt else "#00e5ff"
        self.canvas.create_line(wx, wy, hx, hy, fill=line_color, width=6, capstyle=tk.ROUND)

        # Head / Top Chassis Box
        box_w, box_h = 36, 20
        # Draw top box centered at (hx, hy)
        self.canvas.create_rectangle(hx - box_w/2, hy - box_h/2, hx + box_w/2, hy + box_h/2,
                                     fill="#1e1e24", outline=line_color, width=2)

        # Angle Text Banner
        self.canvas.create_text(cx, 22, text=f"Pitch Angle: {self.current_pitch:+.2f}°",
                                fill=line_color, font=("Segoe UI", 13, "bold"))
        self.canvas.create_text(cx, 210, text=f"Motors Effort: {self.current_pid_out:+.1f}  |  Alpha: {self.current_alpha:.2f}",
                                fill="#aaaaaa", font=("Segoe UI", 9))

    def update_statistics(self):
        now = time.time()
        elapsed = now - self.last_stat_time
        if elapsed >= 1.0:
            self.measured_hz = self.packet_count / elapsed
            self.packet_count = 0
            self.last_stat_time = now

            # Calculate RMS noise of last 40 samples
            if len(self.pitch_data) > 10:
                recent = list(self.pitch_data)[-40:]
                mean = sum(recent) / len(recent)
                var = sum((x - mean)**2 for x in recent) / len(recent)
                self.noise_rms = math.sqrt(var)

        self.hz_label.config(text=f"{self.measured_hz:.1f} Hz")
        self.noise_label.config(text=f"{self.noise_rms:.2f}° σ")

        if self.noise_rms < 0.12:
            self.quality_label.config(text="EXCELLENT (Clean)", foreground="#4caf50")
        elif self.noise_rms < 0.40:
            self.quality_label.config(text="GOOD (Normal Sway)", foreground="#8bc34a")
        else:
            self.quality_label.config(text="HIGH NOISE / VIBRATION", foreground="#ff5722")

        if self.safety_cutoff_triggered or abs(self.current_pitch) > self.current_max_tilt:
            self.safety_badge.config(text="CUTOFF TRIGGERED (Latched OFF)", foreground="#f44336")
        elif abs(self.current_pitch) > self.current_max_tilt * 0.75:
            self.safety_badge.config(text="WARNING (Near Cutoff)", foreground="#ff9800")
        else:
            self.safety_badge.config(text="SAFE (Active)", foreground="#4caf50")

    def refresh_dashboard(self):
        self.draw_physical_visualizer()
        self.update_statistics()

        if len(self.timestamps) > 2:
            t_list = list(self.timestamps)
            self.line_pitch.set_data(t_list, list(self.pitch_data))
            self.ax_pitch.set_xlim(min(t_list), max(t_list) + 0.1)

            self.line_pid.set_data(t_list, list(self.pid_out_data))
            self.line_vel.set_data(t_list, list(self.vel_data))
            self.ax_motor.set_xlim(min(t_list), max(t_list) + 0.1)

            self.line_encl.set_data(t_list, list(self.enc_l_data))
            self.line_encr.set_data(t_list, list(self.enc_r_data))
            self.ax_enc.set_xlim(min(t_list), max(t_list) + 0.1)
            self.ax_enc.relim()
            self.ax_enc.autoscale_view()

            self.canvas_plot.draw_idle()

        self.root.after(80, self.refresh_dashboard)


def main():
    parser = argparse.ArgumentParser(description="Balance_Rework MPU6050 & Motor Inspector")
    parser.add_argument("--port", help="Default serial COM port (e.g. COM7)", default="")
    parser.add_argument("--mock", help="Run with simulated telemetry data", action="store_true")
    args = parser.parse_args()

    root = tk.Tk()
    app = MPUInspectorGUI(root, default_port=args.port, use_mock=args.mock)
    root.mainloop()


if __name__ == "__main__":
    main()
