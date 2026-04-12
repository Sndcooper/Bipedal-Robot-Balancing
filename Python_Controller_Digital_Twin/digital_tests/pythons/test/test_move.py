import time
import sys
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ax12_protocol import AX12Controller

# Ensure we can import the protocol
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PORT = 'COM10'
SERVO_IDS = [6, 14] # Put your actual servo IDs here

def test_movement():
    print(f"Connecting to {PORT}...")
    try:
        ctrl = AX12Controller(PORT, baudrate=115200, debug=True)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    print("\n--- Pinging Servos ---")
    for sid in SERVO_IDS:
        if ctrl.ping(sid):
            print(f"Servo {sid} OK")
        else:
            print(f"Servo {sid} NOT RESPONDING")

    print("\n--- Enabling Torque & Setting Speed ---")
    for sid in SERVO_IDS:
        ctrl.set_torque_enable(sid, True)
        ctrl.set_speed(sid, 150) # Slow speed for testing

    time.sleep(1)

    print("\n--- Moving to Center (512) ---")
    for sid in SERVO_IDS:
        ctrl.set_position(sid, 512)
    time.sleep(2)

    print("\n--- Moving to 400 ---")
    for sid in SERVO_IDS:
        ctrl.set_position(sid, 400)
    time.sleep(2)

    print("\n--- Moving to 600 ---")
    for sid in SERVO_IDS:
        ctrl.set_position(sid, 600)
    time.sleep(2)

    print("\n--- Moving back to Center (512) ---")
    for sid in SERVO_IDS:
        ctrl.set_position(sid, 512)
    time.sleep(2)

    print("\n--- Disabling Torque ---")
    for sid in SERVO_IDS:
        ctrl.set_torque_enable(sid, False)

    ctrl.close()
    print("Test complete.")

if __name__ == "__main__":
    test_movement()


