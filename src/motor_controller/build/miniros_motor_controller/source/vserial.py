import serial
import struct
import time


class ArduinoSerial:
    def __init__(self, port, baudrate=115200):
        """Initialize serial connection"""
        self.serial = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset
    
    def _sound_startup(self):
        self.send_floats(0.4, 0.4)
        time.sleep(0.01)
        self.send_floats(0.0, 0.0)
        time.sleep(0.01)
        self.send_floats(0.4, 0.4)
        time.sleep(0.01)
        self.send_floats(0.0, 0.0)
        time.sleep(0.01)
    
    def send_ping(self):
        """Send PING signal"""
        # PING packet format: 'P' (1 byte)
        packet = b'P'
        self.serial.write(packet)
        print("PING sent")
        
        # Wait for response
        response = self.serial.readline().decode().strip()
        if response:
            print(f"Arduino response: {response}")
            return True
        return False
    
    def send_floats(self, float1, float2):
        """Send two float values"""
        # 'D' (1 byte) + left (float / 4 bytes) + float2 (float / 4 bytes)
        header = b'D'
        data = struct.pack('<ff', float1, float2)
        
        self.serial.write(header + data)
            
    def close(self):
        """Close serial connection"""
        self.serial.close()