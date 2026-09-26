"""
main_gui.py
Motor + encoder wiring-check GUI for blue_pill_dev/motor_testing.

Press Forward/Back/Left/Right and watch EL (left encoder) / ER (right
encoder) live below. Confirms after rewiring that:
  - Forward makes EL count UP and ER count DOWN (mirror-mounted encoders).
  - Back reverses both.
  - Left/Right spin the wheels in opposite directions from each other.
If a direction moves the wrong way physically, or an encoder counts the
wrong sign, the motor leads or encoder A/B pair for that side are swapped.
"""

import tkinter as tk
from tkinter import ttk
import sys

from serial_link import SerialLink

PORT = "COM13"  # change to your Blue Pill's port


class MotorTestGUI:
    def __init__(self, root):
        self.root = root
        root.title("Blue Pill Motor + Encoder Wiring Check")

        self.link = SerialLink(port=PORT)
        try:
            self.link.connect()
            status = f"Connected on {PORT}"
        except Exception as e:
            status = f"Connection failed: {e}"

        self.status_var = tk.StringVar(value=status)
        ttk.Label(root, textvariable=self.status_var).grid(row=0, column=0, columnspan=3, pady=4)

        # Direction pad
        pad = ttk.Frame(root)
        pad.grid(row=1, column=0, columnspan=3, pady=8)

        self._make_btn(pad, "Forward", self.link.forward, 0, 1)
        self._make_btn(pad, "Left", self.link.turn_left, 1, 0)
        self._make_btn(pad, "Stop", self.link.stop, 1, 1)
        self._make_btn(pad, "Right", self.link.turn_right, 1, 2)
        self._make_btn(pad, "Back", self.link.backward, 2, 1)

        # PWM slider
        ttk.Label(root, text="Test PWM").grid(row=2, column=0)
        self.pwm_var = tk.IntVar(value=150)
        pwm_slider = ttk.Scale(root, from_=0, to=255, variable=self.pwm_var,
                               orient="horizontal",
                               command=lambda v: self.link.set_pwm(float(v)))
        pwm_slider.grid(row=2, column=1, columnspan=2, sticky="ew", padx=8)

        # Encoder readout
        enc_frame = ttk.LabelFrame(root, text="Live Encoder Counts")
        enc_frame.grid(row=3, column=0, columnspan=3, pady=10, sticky="ew")

        self.el_var = tk.StringVar(value="EL (Left): 0")
        self.er_var = tk.StringVar(value="ER (Right): 0")
        ttk.Label(enc_frame, textvariable=self.el_var, font=("Consolas", 14)).pack(pady=4)
        ttk.Label(enc_frame, textvariable=self.er_var, font=("Consolas", 14)).pack(pady=4)

        ttk.Label(root, text="Expected: Forward -> EL up, ER down. Back -> reverses.\n"
                              "Left -> wheels spin opposite ways; Right -> opposite of Left.",
                  justify="center").grid(row=4, column=0, columnspan=3, pady=6)

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll()

    def _make_btn(self, parent, text, command, row, col):
        btn = ttk.Button(parent, text=text, command=command, width=10)
        btn.grid(row=row, column=col, padx=4, pady=4)
        return btn

    def _poll(self):
        el, er = self.link.snapshot()
        self.el_var.set(f"EL (Left): {el}")
        self.er_var.set(f"ER (Right): {er}")
        self.root.after(100, self._poll)

    def _on_close(self):
        self.link.stop()
        self.link.close()
        self.root.destroy()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        PORT = sys.argv[1]
    root = tk.Tk()
    app = MotorTestGUI(root)
    root.mainloop()
