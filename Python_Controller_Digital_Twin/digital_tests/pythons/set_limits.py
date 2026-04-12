import time
import sys
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ax12_protocol import AX12Controller

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

PORT = 'COM10'
SERVO_ID = 0

def change_limits():
    print(f"Connecting to {PORT}...")
    try:
        ctrl = AX12Controller(PORT, baudrate=115200, timeout=0.1, debug=True)
    except Exception as e:
        print(f"Failed to connect: {e}")
        return

    # Check connection
    if not ctrl.ping(SERVO_ID):
        print(f"Servo {SERVO_ID} NOT RESPONDING")
        return

    # Read current limits
    cw_data = ctrl.read_register(SERVO_ID, 6, 2)
    ccw_data = ctrl.read_register(SERVO_ID, 8, 2)

    if cw_data and ccw_data:
        cw_limit = cw_data[0] | (cw_data[1] << 8)
        ccw_limit = ccw_data[0] | (ccw_data[1] << 8)
        print(f"Current limits for ID {SERVO_ID}:")
        print(f"  CW (Min):  {cw_limit}")
        print(f"  CCW (Max): {ccw_limit}")
    else:
        print("Failed to read current limits.")
        return

    print("\nDo you want to reset them to factory defaults (0 to 1023)?")
    response = input("Enter 'y' for YES, or custom limits like '200 800': ").strip()

    if response.lower() == 'y':
        new_cw = 0
        new_ccw = 1023
    else:
        try:
            parts = response.split()
            new_cw = int(parts[0])
            new_ccw = int(parts[1])
        except:
            print("Invalid input. Cancelling.")
            return

    print(f"\nSetting limits for ID {SERVO_ID} to: CW={new_cw}, CCW={new_ccw}")
    # Disable torque before setting EEPROM
    ctrl.set_torque_enable(SERVO_ID, False)
    time.sleep(0.1)

    ctrl.set_angle_limits(SERVO_ID, new_cw, new_ccw)
    print("Done! You can verify by running this script again.")
    ctrl.close()

if __name__ == "__main__":
    change_limits()


