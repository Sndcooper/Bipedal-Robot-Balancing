import serial
import time
import sys
import threading
from tkinter import Tk, Label, StringVar, Frame

PORT = 'COM10'
BAUD = 250000

print(f"Connecting to {PORT}...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
except Exception as e:
    print(f"Failed to connect: {e}")
    sys.exit(1)

# Current state
state = {
    'Kp': 15.0,
    'Ki': 0.0,
    'Kd': 2.0,
    'Offset': 0.0,
    'Roll': 0.0
}

def send_cmd(cmd):
    ser.write(f"{cmd}\n".encode())

def toggle_polarity(*args):
    state['Kp'] = -state['Kp']
    send_cmd(f"P{state['Kp']}")
    update_labels()

def kp_up(*args):
    state['Kp'] += 5.0 if state['Kp'] >= 0 else -5.0
    send_cmd(f"P{state['Kp']}")
    update_labels()

def kp_down(*args):
    state['Kp'] -= 5.0 if state['Kp'] >= 0 else -5.0
    send_cmd(f"P{state['Kp']}")
    update_labels()

def kd_up(*args):
    state['Kd'] += 0.5
    send_cmd(f"D{state['Kd']}")
    update_labels()

def kd_down(*args):
    state['Kd'] = max(0.0, state['Kd'] - 0.5)
    send_cmd(f"D{state['Kd']}")
    update_labels()

def set_zero(*args):
    # Set current roll as the perfect 0 upright position
    state['Offset'] = state['Roll']
    send_cmd(f"O{state['Offset']}")
    print(f"Zeroed at {state['Offset']}")
    update_labels()

def stop_motors(*args):
    state['Kp'] = 0.0
    state['Kd'] = 0.0
    send_cmd("P0\nD0\n")
    update_labels()

root = Tk()
root.title("Balancing Robot Tuner")
root.geometry("400x300")

lbl_roll = StringVar()
lbl_kp = StringVar()
lbl_kd = StringVar()
lbl_offset = StringVar()

def update_labels():
    lbl_roll.set(f"Live Roll: {state['Roll']:.2f}°")
    lbl_kp.set(f"Kp (Power): {state['Kp']:.2f}")
    lbl_kd.set(f"Kd (Damping): {state['Kd']:.2f}")
    lbl_offset.set(f"Zero Angle: {state['Offset']:.2f}°")

def read_serial():
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith("Roll:"):
                # Parse: "Roll: 12.34 | Kp: ..."
                parts = line.split("|")
                r_val = float(parts[0].replace("Roll:", "").strip())
                state['Roll'] = r_val
                lbl_roll.set(f"Live Roll: {state['Roll']:.2f}°")
        except:
            pass

Label(root, text="Robot Live Tuner", font=("Arial", 16, "bold")).pack(pady=5)
Label(root, textvariable=lbl_roll, font=("Arial", 14), fg="blue").pack()
Label(root, textvariable=lbl_offset, font=("Arial", 12)).pack()
Label(root, textvariable=lbl_kp, font=("Arial", 12)).pack()
Label(root, textvariable=lbl_kd, font=("Arial", 12)).pack()

frame = Frame(root)
frame.pack(pady=10)
Label(frame, text="Controls:\n'Z' = Set Perfect Upright (ZERO)\n'Up/Down' = Increase/Decrease Power (Kp)\n'Right/Left' = Increase/Decrease Damping (Kd)\n'SPACE' = EMERGENCY STOP\n'R' = Reverse Motor Direction (+/- Kp)", justify="left").pack()

root.bind('<z>', set_zero)
root.bind('<Up>', kp_up)
root.bind('<Down>', kp_down)
root.bind('<Right>', kd_up)
root.bind('<Left>', kd_down)
root.bind('<space>', stop_motors)
root.bind('<r>', toggle_polarity)

update_labels()
t = threading.Thread(target=read_serial, daemon=True)
t.start()

root.mainloop()
ser.close()
