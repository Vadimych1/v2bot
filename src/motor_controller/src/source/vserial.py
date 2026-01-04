import serial
import struct
import time


class ArduinoSerial:
    """
    Serial communication module for Arduino
    """

    def __init__(self, port, baudrate=115200):
        self.serial = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)

    def _sound_startup(self):
        """Play the startup sound (two beeps)"""
        self.send_floats(0.4, 0.4)
        time.sleep(0.1)
        self.send_floats(0.0, 0.0)
        time.sleep(0.1)
        self.send_floats(0.4, 0.4)
        time.sleep(0.1)
        self.send_floats(0.0, 0.0)

    def send_ping(self) -> bool:
        """Send PING signal"""
        # PING packet format: 'P' (1 byte)
        packet = b"P"
        self.serial.write(packet)

        # Wait for response
        response = self.serial.readline().decode().strip()
        if response:
            return True
        
        return False

    def send_floats(self, left: float, right: float) -> None:
        """Send l/r speed values"""
        
        # 'D' (1 byte) + left (float / 4 bytes) + float2 (float / 4 bytes)
        header = b"D"
        data = struct.pack("<ff", left, right)

        self.serial.write(header + data)

    def close(self):
        """Close serial connection"""
        self.serial.close()
