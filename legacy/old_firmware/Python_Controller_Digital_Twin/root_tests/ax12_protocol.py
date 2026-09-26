"""
AX-12+ Dynamixel Protocol Implementation
==========================================
Handles low-level packet construction and serial communication.
"""

import serial
import time


# ==========================================
#  AX-12+ CONTROL TABLE ADDRESSES
# ==========================================
class AX12_ADDR:
    TORQUE_ENABLE = 24
    LED = 25
    CW_COMPLIANCE_MARGIN = 26
    CCW_COMPLIANCE_MARGIN = 27
    CW_COMPLIANCE_SLOPE = 28
    CCW_COMPLIANCE_SLOPE = 29
    GOAL_POSITION = 30
    MOVING_SPEED = 32
    TORQUE_LIMIT = 34
    PRESENT_POSITION = 36
    PRESENT_SPEED = 38
    PRESENT_LOAD = 40


# ==========================================
#  UTILITY FUNCTIONS
# ==========================================
def deg_to_ax12(deg):
    """Convert AX-12 degrees (0-300°) to position value (0-1023)."""
    return int(deg * 1023 / 300)


def ax12_to_deg(pos):
    """Convert AX-12 position value (0-1023) to degrees (0-300°)."""
    return pos * 300 / 1023


def ik_angle_to_ax12(ik_deg, offset=150.0, invert=False):
    """
    Convert IK angle to AX-12 position.
    
    ik_deg: angle from kinematics (typically -180 to +180)
    offset: what AX-12 degree corresponds to ik_deg=0 (default 150°, position 512)
    invert: flip direction if servo is mirrored
    """
    if invert:
        ax_deg = offset - ik_deg
    else:
        ax_deg = offset + ik_deg
    
    # Clamp to AX-12 range (0-300°)
    ax_deg = max(0, min(300, ax_deg))
    return deg_to_ax12(ax_deg)


# ==========================================
#  DYNAMIXEL PACKET BUILDER
# ==========================================
def build_packet(servo_id, instruction, params):
    """Build a Dynamixel protocol packet."""
    length = len(params) + 2  # instruction + params + checksum
    packet = [0xFF, 0xFF, servo_id, length, instruction] + params
    
    # Checksum: ~(ID + Length + Instruction + Params)
    checksum = (~sum(packet[2:])) & 0xFF
    packet.append(checksum)
    
    return bytes(packet)


def build_write_packet(servo_id, address, value):
    """Build WRITE packet for 1 or 2 byte value."""
    if isinstance(value, int):
        if value <= 0xFF:
            # Single byte
            return build_packet(servo_id, 0x03, [address, value])
        else:
            # Two bytes (little-endian)
            lo = value & 0xFF
            hi = (value >> 8) & 0xFF
            return build_packet(servo_id, 0x03, [address, lo, hi])
    else:
        raise ValueError("Value must be int")


def build_sync_write_packet(address, data_len, id_value_pairs):
    """
    Build SYNC_WRITE packet.
    
    address: register address
    data_len: number of bytes per servo (1 or 2)
    id_value_pairs: list of (id, value) tuples
    """
    params = [address, data_len]
    for servo_id, value in id_value_pairs:
        params.append(servo_id)
        if data_len == 1:
            params.append(value & 0xFF)
        elif data_len == 2:
            params.append(value & 0xFF)
            params.append((value >> 8) & 0xFF)
    
    # SYNC_WRITE uses broadcast ID (0xFE)
    return build_packet(0xFE, 0x83, params)


# ==========================================
#  AX-12 CONTROLLER CLASS
# ==========================================
class AX12Controller:
    def __init__(self, port, baudrate=115200, timeout=0.5, debug=False):
        """
        Initialize AX-12 controller.
        
        port: serial port (e.g., 'COM10' or '/dev/ttyUSB0')
        baudrate: baud rate for STM32 USB connection (default 115200)
        timeout: read timeout in seconds
        debug: print packet debug info
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.debug = debug
        self.ser = None
        
        self._connect()
    
    def _connect(self):
        """Open serial connection."""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            time.sleep(2)  # Wait for STM32 to boot
            
            # Flush startup messages
            while self.ser.in_waiting:
                line = self.ser.readline()
                if self.debug:
                    print(f"[STARTUP] {line.decode('utf-8', errors='ignore').strip()}")
            
            if self.debug:
                print(f"[AX12] Connected to {self.port} @ {self.baudrate} baud")
        
        except Exception as e:
            raise RuntimeError(f"Failed to open {self.port}: {e}")
    
    def _send_packet(self, packet):
        """Send raw packet and optionally read response."""
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open")
        
        if self.debug:
            hex_str = ' '.join(f'{b:02X}' for b in packet)
            print(f"[TX] {hex_str}")
        
        # Clear input buffer before sending
        self.ser.reset_input_buffer()
        
        self.ser.write(packet)
        self.ser.flush()
        
        # For broadcast packets (SYNC_WRITE), no response expected
        if packet[2] == 0xFE:
            if self.debug:
                print("[INFO] Broadcast packet - no response expected")
            time.sleep(0.02)  # Allow time for execution
            return None
        
        # Read response with better timeout handling
        time.sleep(0.05)  # Increased delay for STM32 to process and forward response
        response = b''
        timeout = time.time() + self.timeout
        
        # Wait for response header (0xFF 0xFF)
        header_found = False
        while time.time() < timeout and not header_found:
            if self.ser.in_waiting:
                byte = self.ser.read(1)
                response += byte
                # Look for 0xFF 0xFF pattern
                if len(response) >= 2 and response[-2:] == b'\xFF\xFF':
                    header_found = True
            else:
                time.sleep(0.005)
        
        if not header_found:
            if self.debug:
                print("[WARN] No response header received")
            return None
        
        # Read rest of packet: ID, Length, then Length bytes (error + params + checksum)
        while time.time() < timeout and len(response) < 4:
            if self.ser.in_waiting:
                response += self.ser.read(1)
            else:
                time.sleep(0.005)
        
        if len(response) >= 4:
            expected_len = response[3] + 4  # header(2) + ID(1) + len(1) + payload
            while time.time() < timeout and len(response) < expected_len:
                if self.ser.in_waiting:
                    remaining = expected_len - len(response)
                    response += self.ser.read(remaining)
                else:
                    time.sleep(0.005)
        
        if self.debug:
            if response:
                hex_str = ' '.join(f'{b:02X}' for b in response)
                print(f"[RX] {hex_str}")
                if len(response) >= 5:
                    error = response[4]
                    if error == 0:
                        print(f"[OK] Servo responded successfully")
                    else:
                        print(f"[ERROR] Servo error byte: 0x{error:02X} (0b{error:08b})")
                        self._decode_error(error)
            else:
                print("[WARN] No response received")
        
        return response
    
    def _decode_error(self, error_byte):
        """Decode Dynamixel error byte."""
        errors = []
        if error_byte & 0x01: errors.append("Input Voltage")
        if error_byte & 0x02: errors.append("Angle Limit")
        if error_byte & 0x04: errors.append("Overheating")
        if error_byte & 0x08: errors.append("Range Error")
        if error_byte & 0x10: errors.append("Checksum Error")
        if error_byte & 0x20: errors.append("Overload")
        if error_byte & 0x40: errors.append("Instruction Error")
        if errors:
            print(f"       Errors: {', '.join(errors)}")
    
    # ── Single Servo Commands ──
    
    def set_position(self, servo_id, position):
        """Set goal position (0-1023)."""
        position = max(0, min(1023, int(position)))
        packet = build_write_packet(servo_id, AX12_ADDR.GOAL_POSITION, position)
        self._send_packet(packet)
    
    def set_speed(self, servo_id, speed):
        """Set moving speed (0-1023). 0 = max speed."""
        speed = max(0, min(1023, int(speed)))
        packet = build_write_packet(servo_id, AX12_ADDR.MOVING_SPEED, speed)
        self._send_packet(packet)
    
    def set_torque_limit(self, servo_id, torque):
        """Set torque limit (0-1023)."""
        torque = max(0, min(1023, int(torque)))
        packet = build_write_packet(servo_id, AX12_ADDR.TORQUE_LIMIT, torque)
        self._send_packet(packet)
    
    def set_torque_enable(self, servo_id, enable):
        """Enable/disable torque."""
        value = 1 if enable else 0
        packet = build_write_packet(servo_id, AX12_ADDR.TORQUE_ENABLE, value)
        self._send_packet(packet)
    
    def set_led(self, servo_id, on):
        """Turn LED on/off."""
        value = 1 if on else 0
        packet = build_write_packet(servo_id, AX12_ADDR.LED, value)
        self._send_packet(packet)
    
    def set_compliance_margin(self, servo_id, cw, ccw):
        """Set compliance margin (0-255) for both directions."""
        cw = max(0, min(255, int(cw)))
        ccw = max(0, min(255, int(ccw)))
        packet_cw = build_write_packet(servo_id, AX12_ADDR.CW_COMPLIANCE_MARGIN, cw)
        packet_ccw = build_write_packet(servo_id, AX12_ADDR.CCW_COMPLIANCE_MARGIN, ccw)
        self._send_packet(packet_cw)
        time.sleep(0.01)
        self._send_packet(packet_ccw)
    
    def set_compliance_slope(self, servo_id, cw, ccw):
        """Set compliance slope (1-254) for both directions."""
        cw = max(1, min(254, int(cw)))
        ccw = max(1, min(254, int(ccw)))
        packet_cw = build_write_packet(servo_id, AX12_ADDR.CW_COMPLIANCE_SLOPE, cw)
        packet_ccw = build_write_packet(servo_id, AX12_ADDR.CCW_COMPLIANCE_SLOPE, ccw)
        self._send_packet(packet_cw)
        time.sleep(0.01)
        self._send_packet(packet_ccw)
    
    # ── Sync Write (Multiple Servos) ──
    
    def sync_positions(self, *args):
        """
        Send positions to multiple servos simultaneously using SYNC_WRITE.
        
        Usage: sync_positions(id1, pos1, id2, pos2, ...)
        Example: sync_positions(6, 512, 14, 300)
        """
        if len(args) % 2 != 0:
            raise ValueError("Arguments must be pairs of (id, position)")
        
        pairs = [(args[i], args[i+1]) for i in range(0, len(args), 2)]
        packet = build_sync_write_packet(AX12_ADDR.GOAL_POSITION, 2, pairs)
        self._send_packet(packet)
    
    def close(self):
        """Close serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            if self.debug:
                print("[AX12] Connection closed")
