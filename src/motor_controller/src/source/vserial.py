from asyncio import Queue, Event
import threading
import wireio
import time


class ArduinoSerial:
    def __init__(self, port, baudrate=115200):
        """Initialize serial connection"""
        self.serial = wireio.AsyncSerial(port, baudrate, timeout=1)
        self.odometry_speeds_queue = Queue(100)
        
        self.running = Event()

        time.sleep(2)  # Wait for Arduino to connect

    # def _sound_startup(self):
    #     self.send_floats(0.4, 0.4)
    #     time.sleep(0.05)
    #     self.send_floats(0.0, 0.0)
    #     time.sleep(0.05)
    #     self.send_floats(0.4, 0.4)
    #     time.sleep(0.05)
    #     self.send_floats(0.0, 0.0)
    #     time.sleep(0.05)

    async def set_speeds(self, left: float, right: float):
        data = f"S {left:.4f} {right:.4f}"
        await self.serial.write(data.encode())

    async def reset_position(self, newX: float, newY: float, newTheta: float):
        data = f"R {newX:.4f} {newY:.4f} {newTheta:.4f}"
        await self.serial.write(data.encode())

    async def fetch_one(self):
        data = await self.serial.read_until(b"\n")

        l = data.decode().strip().split(" ")
        l = list(map(float, l))

        if self.odometry_speeds_queue.full():
            for _ in range(int(self.odometry_speeds_queue.maxsize / 2)):
                await self.odometry_speeds_queue.get()

        await self.odometry_speeds_queue.put(l)

    async def fetch_task(self):
        self.running.set()
        while self.running.is_set():
            await self.fetch_one()

    async def close(self):
        """Close serial connection"""
        await self.serial.close()
