import threading
import serial
import struct
import time
from collections import deque


class ArduinoSerial:
    def __init__(self, port, baudrate=115200):
        """Initialize serial connection"""
        self.serial = serial.Serial(port, baudrate, timeout=1)

        self.fetch_thread = None
        self.running = threading.Lock()

        self.odometry_queue = deque()

        time.sleep(2)  # Wait for Arduino to reset

    def _sound_startup(self):
        self.send_floats(0.4, 0.4)
        time.sleep(0.05)
        self.send_floats(0.0, 0.0)
        time.sleep(0.05)
        self.send_floats(0.4, 0.4)
        time.sleep(0.05)
        self.send_floats(0.0, 0.0)
        time.sleep(0.05)

    # def send_ping(self):
    #     """Send PING signal"""
    #     # PING packet format: 'P' (1 byte)
    #     packet = b'P'
    #     self.serial.write(packet)
    #     print("PING sent")

    #     # Wait for response
    #     if response := self.serial.readline().decode().strip():
    #         print(f"Arduino response: {response}")
    #         return True

    #     return False

    def send_floats(self, float1, float2):
        """Send two float values"""
        # 'D' (1 byte) + left (float / 4 bytes) + float2 (float / 4 bytes)
        header = b"D"
        data = struct.pack("<ff", float1, float2)

        self.serial.write(header + data)

    def _deltas_fetch(self):
        while self.running.locked():
            try:
                l = self.serial.readline().decode().strip().split(",")
                l = list(map(float, l))
                self.odometry_queue.append(l)
                
            except Exception:
                self.odometry_queue.append([0, 0, 0.5])

    def run_deltas_fetch(self):
        self.fetch_thread = threading.Thread(target=self._deltas_fetch, daemon=True)
        print("Started thread" if self.running.acquire(False) else "Failed to start")
        self.fetch_thread.start()

    def close(self):
        """Close serial connection"""
        self.serial.close()
        self.running.release()
        self.fetch_thread.join()
