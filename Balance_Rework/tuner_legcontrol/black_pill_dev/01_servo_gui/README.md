# 01: AX-12 servo test

Connect one servo first. The GUI sends `TQ`, `P`, and `ALL` commands through the
FTDI COM port. Start at a safe centre position (512), then move only small steps.
The sketch never moves a servo until a command arrives.

Run: `cd firmware; pio run -t upload`, then `python gui\servo_gui.py COMx`.
