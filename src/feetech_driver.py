"""
Unified Feetech STS/SCS series driver for Protocol 0 (SCS).
This module provides a high-level interface for communicating with Feetech servos
using the SCS (Protocol 0) serial communication standard, optimized for the SO-ARM100.
"""

import serial

class FeetechDriver:
    """
    A driver class to handle serial communication with Feetech STS/SCS servos.
    
    Attributes:
        ser (serial.Serial): The underlying serial connection.
    """
    
    # Instructions
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04
    INST_ACTION = 0x05
    INST_SYNC_READ = 0x82
    INST_SYNC_WRITE = 0x83

    # Control Table Addresses (STS3215)
    # EPROM (Persistent settings)
    ADDR_ID = 5
    ADDR_BAUD_RATE = 6
    ADDR_MIN_POS_LIMIT = 9  # 2 bytes
    ADDR_MAX_POS_LIMIT = 11 # 2 bytes
    ADDR_MAX_TORQUE = 16    # 2 bytes
    ADDR_P_COEFF = 21
    ADDR_D_COEFF = 22
    ADDR_I_COEFF = 23
    ADDR_PROTECTION_CURRENT = 28 # 2 bytes
    ADDR_HOMING_OFFSET = 31 # 2 bytes
    
    # SRAM (Volatile / Runtime settings)
    ADDR_TORQUE_ENABLE = 40
    ADDR_ACCELERATION = 41
    ADDR_GOAL_POSITION = 42 # 2 bytes
    ADDR_GOAL_TIME = 44     # 2 bytes
    ADDR_GOAL_SPEED = 46    # 2 bytes
    ADDR_LOCK = 55
    ADDR_PRESENT_POSITION = 56 # 2 bytes
    ADDR_PRESENT_SPEED = 58    # 2 bytes
    ADDR_PRESENT_LOAD = 60     # 2 bytes
    ADDR_PRESENT_VOLTAGE = 62
    ADDR_PRESENT_TEMP = 63
    ADDR_MOVING = 66
    ADDR_OVERLOAD_TORQUE = 36

    def __init__(self, port, baudrate=1000000):
        """
        Initializes the serial connection to the servo bus.
        
        Args:
            port (str): The serial port name (e.g., 'COM3' or '/dev/ttyUSB0').
            baudrate (int): Communication speed. Defaults to 1,000,000.
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.01)
        except Exception as e:
            print(f"Error opening port {port}: {e}")
            raise e
        
    def _create_packet(self, id, instruction, params):
        """
        Constructs a Feetech Protocol 0 packet.
        
        Args:
            id (int): The target servo ID.
            instruction (int): The instruction type (Read, Write, etc.).
            params (list): List of parameter bytes.
            
        Returns:
            bytearray: The complete formatted packet with checksum.
        """
        length = len(params) + 2
        packet = [0xFF, 0xFF, id, length, instruction] + params
        checksum = ~(sum(packet[2:]) & 0xFF) & 0xFF
        packet.append(checksum)
        return bytearray(packet)

    def write_byte(self, id, address, value):
        """
        Writes a single byte (8-bit) to a servo register.
        
        Args:
            id (int): Target servo ID.
            address (int): Register address.
            value (int): Byte value to write (0-255).
        """
        packet = self._create_packet(id, self.INST_WRITE, [address, value & 0xFF])
        self.ser.write(packet)

    def write_word(self, id, address, value):
        """
        Writes a 2-byte word (16-bit) to a servo register using little-endian order.
        
        Args:
            id (int): Target servo ID.
            address (int): Starting register address.
            value (int): Word value to write (0-65535).
        """
        params = [address, value & 0xFF, (value >> 8) & 0xFF]
        packet = self._create_packet(id, self.INST_WRITE, params)
        self.ser.write(packet)

    def read_word(self, id, address):
        """
        Reads a 2-byte word (16-bit) from a servo register.
        
        Args:
            id (int): Target servo ID.
            address (int): Starting register address.
            
        Returns:
            int or None: The 16-bit value read, or None if the read failed.
        """
        packet = self._create_packet(id, self.INST_READ, [address, 2])
        self.ser.write(packet)
        
        # Expected response: 0xFF, 0xFF, ID, LENGTH, ERROR, DATA_L, DATA_H, CHECKSUM
        header = self.ser.read(5)
        if len(header) == 5 and header[0] == 0xFF and header[1] == 0xFF:
            length = header[3]
            data = self.ser.read(length - 1)
            if len(data) >= 3:
                return data[1] | (data[2] << 8)
        return None

    def set_torque(self, id, enable):
        """
        Enables or disables torque for a specific servo.
        
        Args:
            id (int): Target servo ID.
            enable (bool): True to enable torque, False to disable.
        """
        self.write_byte(id, self.ADDR_TORQUE_ENABLE, 1 if enable else 0)

    def configure_pid(self, id, p=16, i=0, d=32):
        """
        Sets the internal PID coefficients for a specific servo.
        
        Args:
            id (int): Target servo ID.
            p (int): Proportional coefficient. Defaults to 16.
            i (int): Integral coefficient. Defaults to 0.
            d (int): Derivative coefficient. Defaults to 32.
        """
        self.write_byte(id, self.ADDR_P_COEFF, p)
        self.write_byte(id, self.ADDR_I_COEFF, i)
        self.write_byte(id, self.ADDR_D_COEFF, d)

    def set_gripper_safety(self, id):
        """
        Applies safety limits specifically for the gripper motor to prevent burnout.
        
        Sets max torque to 50%, reduces protection current, and lowers overload torque.
        
        Args:
            id (int): Target servo ID (usually 6 for the SO-ARM100).
        """
        self.write_word(id, self.ADDR_MAX_TORQUE, 500)
        self.write_word(id, self.ADDR_PROTECTION_CURRENT, 250)
        self.write_byte(id, self.ADDR_OVERLOAD_TORQUE, 25)

    def close(self):
        """
        Closes the serial port connection.
        """
        if self.ser.is_open:
            self.ser.close()
