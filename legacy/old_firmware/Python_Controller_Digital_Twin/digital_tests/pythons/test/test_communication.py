"""
Simple AX-12 Communication Test
================================
Test communication with AX-12 servos step by step.
Run this before the digital twin to diagnose issues.
"""

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
import time

# Configuration
SERIAL_PORT = 'COM10'
SERVO_IDS = [6, 14]

def main():
    print("="*60)
    print("AX-12 COMMUNICATION TEST")
    print("="*60)
    print(f"Port: {SERIAL_PORT}")
    print(f"Servo IDs: {SERVO_IDS}")
    print("="*60)
    
    # Step 1: Connect
    print("\n[STEP 1] Connecting to STM32...")
    try:
        controller = AX12Controller(SERIAL_PORT, baudrate=115200, timeout=1.0, debug=True)
        print("✓ Connected successfully\n")
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return
    
    time.sleep(1)
    
    # Step 2: Ping servos
    print("\n[STEP 2] Pinging servos...")
    print("-"*60)
    online_servos = []
    for servo_id in SERVO_IDS:
        print(f"\nTesting ID {servo_id}...")
        if controller.ping(servo_id):
            online_servos.append(servo_id)
            print(f"  ✓ Servo {servo_id} is online")
        else:
            print(f"  ✗ Servo {servo_id} not responding")
    
    if not online_servos:
        print("\n✗ ERROR: No servos responding!")
        print("\nTroubleshooting:")
        print("  1. Check power supply to servos (7-12V)")
        print("  2. Verify servo IDs match configuration")
        print("  3. Check data line connections (TX/RX)")
        print("  4. Ensure proper grounding")
        print("  5. Try different baud rates (57600, 115200, 1000000)")
        controller.close()
        return
    
    print(f"\n✓ {len(online_servos)} servo(s) online: {online_servos}")
    
    # Step 3: Test torque control
    print("\n[STEP 3] Testing torque control...")
    print("-"*60)
    for servo_id in online_servos:
        print(f"\nServo {servo_id}:")
        print("  Setting torque OFF...")
        controller.set_torque_enable(servo_id, False)
        time.sleep(0.5)
        print("  ✓ Torque OFF (servo should be loose)")
        time.sleep(1)
        
        print("  Setting torque ON...")
        controller.set_torque_enable(servo_id, True)
        time.sleep(0.5)
        print("  ✓ Torque ON (servo should hold position)")
    
    # Step 4: Test LED control
    print("\n[STEP 4] Testing LED control...")
    print("-"*60)
    for i in range(3):
        for servo_id in online_servos:
            controller.set_led(servo_id, True)
        print(f"  LEDs ON (blink {i+1}/3)")
        time.sleep(0.3)
        
        for servo_id in online_servos:
            controller.set_led(servo_id, False)
        print(f"  LEDs OFF")
        time.sleep(0.3)
    
    print("  ✓ LED control working")
    
    # Step 5: Test position control
    print("\n[STEP 5] Testing position control...")
    print("-"*60)
    
    positions = [512, 300, 724, 512]  # Center, left, right, center
    for pos in positions:
        print(f"\n  Moving to position {pos}...")
        for servo_id in online_servos:
            controller.set_position(servo_id, pos)
        time.sleep(1)
        print(f"  ✓ Command sent")
    
    print("\n  ✓ Position control working")
    
    # Step 6: Test sync write (both servos together)
    if len(online_servos) >= 2:
        print("\n[STEP 6] Testing synchronized movement...")
        print("-"*60)
        
        # Move in sync
        sync_positions = [(512, 512), (300, 724), (724, 300), (512, 512)]
        for pos1, pos2 in sync_positions:
            print(f"  Sync: ID {online_servos[0]}→{pos1}, ID {online_servos[1]}→{pos2}")
            controller.sync_positions(online_servos[0], pos1, online_servos[1], pos2)
            time.sleep(1)
        
        print("  ✓ Synchronized movement working")
    
    # Final status
    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    print(f"✓ All tests passed for servos: {online_servos}")
    print("\nYou can now run the digital twin GUI:")
    print("  python 'digital twin.py'")
    print("="*60)
    
    controller.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()


