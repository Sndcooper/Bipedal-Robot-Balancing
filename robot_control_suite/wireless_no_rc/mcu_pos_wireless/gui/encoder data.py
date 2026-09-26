import serial
import time
import sys
import re

PORT = "COM13"  # Change to your serial port
BAUD = 115200

print(f"Connecting to {PORT} @ {BAUD}...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print("Connected! Listening to Encoder Telemetry...\n")
    print(f"{'SEQ':<8} | {'PITCH(°)':<10} | {'EL (Left)':<12} | {'ER (Right)':<12} | {'VEL (c/s)':<12} | {'POS_ERR (c)':<12} | {'VEL_ERR (c/s)':<14} | {'STATUS'}")
    print("-" * 105)

    buffer = ""
    while True:
        chunk = ser.read(ser.in_waiting or 1).decode("utf-8", errors="ignore")
        if not chunk:
            continue
        buffer += chunk

        while "|" in buffer or "\n" in buffer:
            delimiter = "|" if "|" in buffer else "\n"
            frame, buffer = buffer.split(delimiter, 1)
            frame = frame.strip()

            if frame.startswith("S") and "DT" in frame:
                # Parse key-value telemetry tokens
                data = {}
                tokens = frame.split()
                seq = tokens[0]

                for tok in tokens[1:]:
                    m = re.match(r"([A-Za-z]+)([-+]?\d*\.?\d+)", tok)
                    if m:
                        key, val = m.groups()
                        data[key] = float(val)

                pitch = data.get("P", 0.0)
                el = int(data.get("EL", 0))
                er = int(data.get("ER", 0))
                vel = data.get("V", 0.0)
                pos_err = data.get("EP", 0.0)
                vel_err = data.get("EV", 0.0)
                motors = int(data.get("M", 0))

                status = "MOTORS ON" if motors else "STANDBY"

                sys.stdout.write(f"\r{seq:<8} | {pitch:<10.2f} | {el:<12d} | {er:<12d} | {vel:<12.1f} | {pos_err:<12.1f} | {vel_err:<14.1f} | {status}")
                sys.stdout.flush()

except KeyboardInterrupt:
    print("\nClosed monitor.")
except Exception as e:
    print(f"Error: {e}")
