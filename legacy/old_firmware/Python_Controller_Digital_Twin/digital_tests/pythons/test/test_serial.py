import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ax12_protocol import AX12Controller

import serial
import time

PORT = 'COM10'
BAUD = 115200

print(f"Connecting to {PORT}...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
except Exception as e:
    print(f"Failed: {e}")
    exit(1)

# toggle DTR/RTS to try to reset the board (depends on wiring)
ser.dtr = False
ser.rts = False
time.sleep(0.1)
ser.dtr = True
ser.rts = True
time.sleep(1)

while ser.in_waiting:
    print(ser.read(ser.in_waiting).decode(errors='ignore'), end='')
print("\n--- Sending RAW mode string test ---")
ser.write(b"TEXT\n")
ser.flush()

time.sleep(0.5)
resp = b""
while ser.in_waiting:
    resp += ser.read(ser.in_waiting)

print(f"Response: {resp.decode(errors='ignore')}")

print("Sending RAW to ID 6...")
ser.write(b"RAW\n")
ser.flush()
time.sleep(0.5)
resp = b""
while ser.in_waiting:
    resp += ser.read(ser.in_waiting)
print(f"Response to RAW switch: {resp.decode(errors='ignore')}")

print("Sending PING via RAW passthrough...")
# 0xFF 0xFF 0x06 0x02 0x01 0xF6
ser.write(bytes([0xFF, 0xFF, 0x06, 0x02, 0x01, 0xF6]))
ser.flush()
time.sleep(0.5)

resp = b""
while ser.in_waiting:
    resp += ser.read(ser.in_waiting)
print(f"RAW Response Hex: {' '.join(f'{b:02X}' for b in resp)}")


ser.close()


