import serial
import time

# Replace 'COM3' with your actual FTDI port (e.g., '/dev/ttyUSB0' on Ubuntu/Linux)
# The baudrate MUST match the radio's default (57600)
PORT = 'COM3'  
BAUD_RATE = 57600

try:
    # Open the serial port
    radio_link = serial.Serial(PORT, BAUD_RATE, timeout=2)
    print(f"Connected to {PORT} at {BAUD_RATE} baud.")
    
    # Wait a moment for the connection to stabilize
    time.sleep(2)
    
    # 1. Send a command to the robot
    command_to_send = "P"
    print(f"Sending command: {command_to_send}")
    radio_link.write(command_to_send.encode('utf-8'))
    
    # 2. Wait for and read the response
    print("Waiting for response...")
    response = radio_link.readline().decode('utf-8').strip()
    
    if response:
        print(f"Robot replied: {response}")
    else:
        print("No response received. Check wiring and baud rates.")

except serial.SerialException as e:
    print(f"Serial error: {e}")
finally:
    if 'radio_link' in locals() and radio_link.is_open:
        radio_link.close()
        print("Connection closed.")