"""
╔══════════════════════════════════════════════════════════╗
║   BIPEDAL ROBOT — PID AUTO-TUNER  (Relay / Z-N Method)  ║
║   Author: Claude / Antigravity                           ║
╚══════════════════════════════════════════════════════════╝

Nano IMU Physical Orientation (YOUR ROBOT):
  ROLL  (gx / atan2(ay,az))  = FORWARD / BACKWARD tilt  → PID balancing axis
  PITCH (gy / atan2(-ax,…))  = SIDEWAYS lean             → left leg higher than right

HOW IT WORKS
────────────
1. First runs a CALIBRATION pass — finds your robot's true upright angle.
2. Then runs a RELAY AUTO-TUNE:
     • Applies bang-bang (relay) output to the motors
     • Measures forward/backward oscillation (roll axis)
     • Uses Ziegler-Nichols formulas to compute Kp, Ki, Kd
3. Sends the computed gains back to the STM32.
4. Shows a live roll / pitch chart the whole time.

REQUIREMENTS  pip install pyserial matplotlib
"""

import serial
import serial.tools.list_ports
import threading
import time
import sys
import re
import collections
import math

import matplotlib
matplotlib.use("TkAgg")          # works on Windows without extra config
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches

# ─────────────────────────────── CONFIG ──────────────────────────────────────
SERIAL_PORT  = "COM10"          # ← CHANGE THIS to your STM32 port
BAUD_RATE    = 115200
PLOT_WINDOW  = 300             # number of roll samples shown on the live chart
RELAY_AMP    = 60              # relay output amplitude sent to STM32 (0-120)
# ─────────────────────────────────────────────────────────────────────────────

# ── Shared state (serial thread → main / plot) ─────────────────────────────
lock        = threading.Lock()
roll_buf    = collections.deque([0.0] * PLOT_WINDOW, maxlen=PLOT_WINDOW)
pitch_buf   = collections.deque([0.0] * PLOT_WINDOW, maxlen=PLOT_WINDOW)
time_buf    = collections.deque([0.0] * PLOT_WINDOW, maxlen=PLOT_WINDOW)

state = {
    "roll"      : 0.0,
    "pitch"     : 0.0,
    "Kp"        : 0.0,
    "Ki"        : 0.0,
    "Kd"        : 0.0,
    "setpoint"  : 0.0,
    "relay_on"  : False,
    "status"    : "Connecting…",
    "phase"     : "IDLE",      # IDLE | CALIB | RELAY | DONE | ERROR
    "result"    : None,        # dict of Tu, Au, Ku, Kp, Ki, Kd after tuning
    "log"       : [],          # list of log strings for side panel
    "connected" : False,
}

ser = None   # serial.Serial instance


# ═══════════════════════════════════════════════════════════════════════════
#  SERIAL THREAD — reads STM32 lines and updates state{}
# ═══════════════════════════════════════════════════════════════════════════
TELEM_RE = re.compile(
    r"TELEM:([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+),(0|1)"
)
TUNE_RE  = re.compile(r"TUNE:(\w+)(?:=([\d.]+))?")

t0 = time.time()

def serial_thread():
    global ser
    while True:
        line = ""
        try:
            raw = ser.readline()
            line = raw.decode("utf-8", errors="ignore").strip()
        except Exception:
            with lock:
                state["status"]    = "Serial error — reconnecting…"
                state["connected"] = False
            time.sleep(1)
            try:
                ser.close()
            except Exception:
                pass
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
                with lock:
                    state["status"]    = "Reconnected."
                    state["connected"] = True
            except Exception:
                pass
            continue

        if not line:
            continue

        # ── Telemetry line ──────────────────────────────────────────────
        m = TELEM_RE.match(line)
        if m:
            r, p, kp, ki, kd, sp, relay = [float(x) for x in m.groups()]
            t = time.time() - t0
            with lock:
                state["roll"]     = r
                state["pitch"]    = p
                state["Kp"]       = kp
                state["Ki"]       = ki
                state["Kd"]       = kd
                state["setpoint"] = sp
                state["relay_on"] = bool(relay)
                roll_buf.append(r)
                pitch_buf.append(p)
                time_buf.append(t)
            continue

        # ── Tune messages ───────────────────────────────────────────────
        m = TUNE_RE.match(line)
        if m:
            key, val = m.group(1), m.group(2)
            with lock:
                if key == "START":
                    state["phase"]   = "RELAY"
                    state["status"]  = f"Relay tuning… amp={RELAY_AMP} — hold robot upright!"
                    state["log"].append(f"[{_ts()}] Relay ON (amp={RELAY_AMP})")
                elif key == "DONE":
                    state["phase"]  = "DONE"
                    state["status"] = "Auto-tune COMPLETE ✓"
                    state["log"].append(f"[{_ts()}] Tuning complete!")
                elif key in ("Tu","Au","Ku","Kp","Ki","Kd"):
                    if state["result"] is None:
                        state["result"] = {}
                    state["result"][key] = float(val)
                    state["log"].append(f"  {key} = {float(val):.4f}")
                elif key == "ABORTED":
                    state["phase"]  = "IDLE"
                    state["status"] = "Tune aborted."
            continue

        # ── Regular text — just log ────────────────────────────────────
        with lock:
            state["status"] = line[:80]
            if any(k in line for k in ("CALIB", "Center", "Kp=", "Ki=", "Kd=", "Setpoint")):
                state["log"].append(f"[{_ts()}] {line}")


def _ts():
    return f"{time.time()-t0:7.2f}s"


# ═══════════════════════════════════════════════════════════════════════════
#  CONTROLLER — sends commands in sequence
# ═══════════════════════════════════════════════════════════════════════════
def send(cmd: str):
    """Thread-safe send with delay."""
    if ser and ser.is_open:
        ser.write((cmd + "\n").encode())
        ser.flush()
    with lock:
        state["log"].append(f"[{_ts()}] → {cmd}")
    time.sleep(0.05)


def run_autotune_sequence():
    """
    Full automated sequence:
      1. Wait for connection
      2. COM calibration (hold still ~2-3 s)
      3. Relay auto-tune
      4. Apply computed gains
    """
    # ── Wait for connection ──────────────────────────────────────────────
    with lock:
        state["status"] = "Waiting for STM32…"
    for _ in range(40):
        with lock:
            ok = state["connected"]
        if ok:
            break
        time.sleep(0.25)
    else:
        with lock:
            state["status"] = "ERROR: Cannot connect to STM32!"
            state["phase"]  = "ERROR"
        return

    time.sleep(1.0)   # let firmware settle

    # ── Step 1: Reset gains to safe defaults ────────────────────────────
    with lock:
        state["phase"]  = "CALIB"
        state["status"] = "Resetting gains…"
    send("I0.0")      # zero integral first
    send("D0.0")
    send("P5.0")      # low Kp so robot doesn't fight during calib

    # ── Step 2: COM Calibration ─────────────────────────────────────────
    with lock:
        state["status"] = "COM Calibration — hold robot perfectly still for 2 s…"
        state["log"].append(f"[{_ts()}] === CALIBRATING UPRIGHT ANGLE ===")
    send("C")

    # Wait for calibration to complete (STM32 takes ~100 IMU cycles ≈ 2-3 s)
    for _ in range(60):
        with lock:
            log = state["log"]
        if any("CALIB" in l or "Center" in l for l in log[-5:]):
            break
        time.sleep(0.1)
    time.sleep(0.5)

    # ── Step 3: Relay auto-tune ─────────────────────────────────────────
    with lock:
        state["status"] = "Starting relay auto-tune…"
        state["log"].append(f"[{_ts()}] === RELAY AUTO-TUNE START ===")
    send(f"R{RELAY_AMP}")

    # Wait for DONE (max 60 s)
    for _ in range(600):
        with lock:
            phase = state["phase"]
        if phase == "DONE":
            break
        time.sleep(0.1)
    else:
        with lock:
            state["status"] = "TIMEOUT — try increasing RELAY_AMP or holding more still."
            state["phase"]  = "ERROR"
        return

    # ── Step 4: Apply computed gains ────────────────────────────────────
    with lock:
        result = state["result"]

    if result and "Kp" in result:
        send(f"P{result['Kp']:.3f}")
        send(f"I{result['Ki']:.3f}")
        send(f"D{result['Kd']:.3f}")
        with lock:
            state["log"].append(f"[{_ts()}] Gains applied to STM32!")
            state["log"].append(f"  Kp={result['Kp']:.4f}  Ki={result['Ki']:.4f}  Kd={result['Kd']:.4f}")

    with lock:
        state["status"] = "✓ Done! Robot should now self-balance."


# ═══════════════════════════════════════════════════════════════════════════
#  MATPLOTLIB LIVE DASHBOARD
# ═══════════════════════════════════════════════════════════════════════════
DARK_BG  = "#0d1117"
PANEL_BG = "#161b22"
GREEN    = "#39d353"
CYAN     = "#58a6ff"
ORANGE   = "#f0883e"
RED      = "#f85149"
WHITE    = "#e6edf3"
GREY     = "#8b949e"


def build_figure():
    fig = plt.figure(figsize=(14, 8), facecolor=DARK_BG)
    fig.canvas.manager.set_window_title("Bipedal Robot — PID Auto-Tuner")

    gs = gridspec.GridSpec(3, 3, figure=fig,
                           left=0.06, right=0.98,
                           top=0.93, bottom=0.08,
                           hspace=0.45, wspace=0.35)

    # ── Roll chart (top-left, wide) ──────────────────────────────────────
    ax_roll = fig.add_subplot(gs[0:2, 0:2])
    ax_roll.set_facecolor(PANEL_BG)
    ax_roll.tick_params(colors=GREY)
    ax_roll.spines[:].set_color("#30363d")
    ax_roll.set_title("Roll = Fwd/Back tilt  |  Pitch = Sideways lean",
                      color=WHITE, fontsize=10, pad=6)
    ax_roll.set_ylabel("degrees", color=GREY, fontsize=9)
    ax_roll.set_ylim(-30, 30)
    ax_roll.axhline(0, color=GREY, lw=0.8, ls="--")

    xdata = list(range(PLOT_WINDOW))
    line_roll,  = ax_roll.plot(xdata, list(roll_buf),  color=CYAN,   lw=1.8, label="Roll — fwd/back (PID axis)")
    line_sp,    = ax_roll.plot(xdata, [0]*PLOT_WINDOW,  color=GREEN,  lw=1.0, ls="--", label="Setpoint")
    line_pitch, = ax_roll.plot(xdata, list(pitch_buf), color=ORANGE, lw=1.0, alpha=0.7, label="Pitch — sideways")
    ax_roll.legend(loc="upper right", fontsize=8, facecolor=PANEL_BG,
                   labelcolor=WHITE, edgecolor="#30363d")

    # ── Gauge: current roll (bottom-left) ───────────────────────────────
    ax_gauge = fig.add_subplot(gs[2, 0])
    ax_gauge.set_facecolor(PANEL_BG)
    ax_gauge.set_xlim(-1, 1); ax_gauge.set_ylim(-1, 1)
    ax_gauge.axis("off")
    ax_gauge.set_title("Roll  (fwd / back)", color=CYAN, fontsize=9, weight="bold")
    gauge_bar = ax_gauge.barh(0, 0, height=0.3, color=CYAN, align="center")
    ax_gauge.axvline(0, color=WHITE, lw=1)
    ax_gauge.set_xlim(-20, 20)
    gauge_txt = ax_gauge.text(0, 0.6, "0.0°", ha="center", color=WHITE, fontsize=16, weight="bold")
    ax_gauge.text(-18, -0.65, "← lean back", color=GREY, fontsize=6.5)
    ax_gauge.text(  9, -0.65, "lean fwd →",  color=GREY, fontsize=6.5)

    # ── PID values panel (bottom-mid) ───────────────────────────────────
    ax_pid = fig.add_subplot(gs[2, 1])
    ax_pid.set_facecolor(PANEL_BG); ax_pid.axis("off")
    ax_pid.set_title("Current Gains", color=WHITE, fontsize=10)
    pid_texts = {
        "Kp": ax_pid.text(0.1, 0.7, "Kp = —", color=CYAN,   fontsize=13, weight="bold", transform=ax_pid.transAxes),
        "Ki": ax_pid.text(0.1, 0.45,"Ki = —", color=GREEN,  fontsize=13, weight="bold", transform=ax_pid.transAxes),
        "Kd": ax_pid.text(0.1, 0.2, "Kd = —", color=ORANGE, fontsize=13, weight="bold", transform=ax_pid.transAxes),
    }

    # ── Status / Log panel (right column) ───────────────────────────────
    ax_log = fig.add_subplot(gs[0:3, 2])
    ax_log.set_facecolor(PANEL_BG); ax_log.axis("off")
    ax_log.set_title("Status Log", color=WHITE, fontsize=10)
    log_text = ax_log.text(0.02, 0.97, "", color=GREEN, fontsize=7.5,
                           va="top", ha="left", transform=ax_log.transAxes,
                           fontfamily="monospace", wrap=False)

    # ── Phase badge (top center) ─────────────────────────────────────────
    phase_txt = fig.text(0.5, 0.965, "● IDLE",
                         ha="center", va="top", fontsize=13, weight="bold", color=GREY)

    # ── Big status bar (bottom) ──────────────────────────────────────────
    status_txt = fig.text(0.06, 0.012, "Connecting…",
                          ha="left", fontsize=9, color=WHITE)

    return (fig, ax_roll, line_roll, line_sp, line_pitch,
            ax_gauge, gauge_bar, gauge_txt,
            pid_texts, log_text, phase_txt, status_txt)


PHASE_COLORS = {
    "IDLE"  : GREY,
    "CALIB" : ORANGE,
    "RELAY" : CYAN,
    "DONE"  : GREEN,
    "ERROR" : RED,
}
PHASE_LABELS = {
    "IDLE"  : "● IDLE",
    "CALIB" : "⟳ CALIBRATING",
    "RELAY" : "⟳ RELAY TUNING",
    "DONE"  : "✓ DONE",
    "ERROR" : "✗ ERROR",
}


def make_update(fig, ax_roll, line_roll, line_sp, line_pitch,
                ax_gauge, gauge_bar, gauge_txt,
                pid_texts, log_text, phase_txt, status_txt):

    def update(_frame):
        with lock:
            rolls   = list(roll_buf)
            pitches = list(pitch_buf)
            sp      = state["setpoint"]
            kp      = state["Kp"]
            ki      = state["Ki"]
            kd      = state["Kd"]
            roll_v  = state["roll"]
            status  = state["status"]
            phase   = state["phase"]
            log     = state["log"][-22:]   # last 22 lines

        # Roll plot
        line_roll.set_ydata(rolls)
        line_pitch.set_ydata(pitches)
        line_sp.set_ydata([sp] * PLOT_WINDOW)

        mn, mx = min(rolls + pitches), max(rolls + pitches)
        pad = max(5.0, (mx - mn) * 0.3)
        ax_roll.set_ylim(mn - pad, mx + pad)

        # Gauge
        clr = RED if abs(roll_v) > 15 else (ORANGE if abs(roll_v) > 5 else CYAN)
        gauge_bar[0].set_width(roll_v)
        gauge_bar[0].set_color(clr)
        gauge_txt.set_text(f"{roll_v:+.1f}°")
        gauge_txt.set_color(clr)

        # PID text
        pid_texts["Kp"].set_text(f"Kp = {kp:.3f}")
        pid_texts["Ki"].set_text(f"Ki = {ki:.3f}")
        pid_texts["Kd"].set_text(f"Kd = {kd:.3f}")

        # Log panel
        log_text.set_text("\n".join(log))

        # Phase badge
        phase_txt.set_text(PHASE_LABELS.get(phase, phase))
        phase_txt.set_color(PHASE_COLORS.get(phase, WHITE))

        # Status bar
        status_txt.set_text(status[:120])

        return (line_roll, line_sp, line_pitch,
                gauge_bar[0], gauge_txt,
                log_text, phase_txt, status_txt)

    return update


# ═══════════════════════════════════════════════════════════════════════════
#  KEYBOARD COMMANDS (optional — type in terminal while GUI open)
# ═══════════════════════════════════════════════════════════════════════════
def keyboard_thread():
    """Read commands from the terminal so you can override gains or drive."""
    print("\n  Manual overrides (type + Enter):")
    print("  P<val>  I<val>  D<val>  O<val>  C  R<amp>  Q  W A S D X\n")
    import sys
    for line in sys.stdin:
        cmd = line.strip()
        if cmd:
            send(cmd)


# ═══════════════════════════════════════════════════════════════════════════
#  FIND PORT AUTOMATICALLY (optional helper)
# ═══════════════════════════════════════════════════════════════════════════
def find_stm32_port():
    """Try to find STM32 COM port automatically."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if any(k in desc for k in ("stm32", "stlink", "usb serial")):
            return p.device
    return None


# ═══════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════
def main():
    global ser

    port = SERIAL_PORT
    auto = find_stm32_port()
    if auto and auto != port:
        print(f"[INFO] Found STM32 on {auto} (overriding config)")
        port = auto

    print(f"[INFO] Connecting to {port} @ {BAUD_RATE} baud…")
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.5)
        with lock:
            state["connected"] = True
            state["status"]    = f"Connected on {port}"
        print(f"[INFO] Connected!")
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open {port}: {e}")
        print("  → Set SERIAL_PORT at the top of this file to your STM32 port.")
        sys.exit(1)

    # Start serial reader
    t_ser = threading.Thread(target=serial_thread, daemon=True)
    t_ser.start()

    # Start auto-tune sequence
    t_tune = threading.Thread(target=run_autotune_sequence, daemon=True)
    t_tune.start()

    # Start keyboard override thread
    t_kbd = threading.Thread(target=keyboard_thread, daemon=True)
    t_kbd.start()

    # Build GUI
    widgets = build_figure()
    fig = widgets[0]
    update_fn = make_update(*widgets)

    ani = FuncAnimation(fig, update_fn, interval=100, blit=False, cache_frame_data=False)

    plt.show()

    # Cleanup
    try:
        ser.close()
    except Exception:
        pass
    print("\n[INFO] Auto-tuner closed.")
    with lock:
        result = state.get("result")
    if result:
        print("\n════════════════════════════════")
        print("  FINAL TUNED GAINS")
        print("════════════════════════════════")
        for k, v in result.items():
            print(f"  {k:4s} = {v:.4f}")
        print()
        print("  Copy these into main.cpp:")
        print(f"  float Kp = {result.get('Kp',0):.3f};")
        print(f"  float Ki = {result.get('Ki',0):.3f};")
        print(f"  float Kd = {result.get('Kd',0):.3f};")
        print("════════════════════════════════\n")


if __name__ == "__main__":
    main()
