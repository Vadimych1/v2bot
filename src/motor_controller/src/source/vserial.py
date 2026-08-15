from asyncio import Queue, Event, Lock
import wireio
import time


class ArduinoSerial:
    def __init__(self, port, baudrate=115200):
        """Initialize serial connection"""
        self.serial = wireio.AsyncSerial(port, baudrate, timeout=1)
        self.odometry_speeds_queue = Queue(5)
        
        self.running = Event()
        self._lock = Lock()

        time.sleep(2)  # Wait for Arduino to connect

    async def set_speeds(self, left: float, right: float):
        data = f"S {left:.4f} {right:.4f}\n"
        
        async with self._lock:
            await self.serial.write(data.encode())
            await self.serial.flush()

    async def reset_position(self, newX: float, newY: float, newTheta: float):
        data = f"R {newX:.4f} {newY:.4f} {newTheta:.4f}\n"
        
        async with self._lock:
            await self.serial.write(data.encode())
            await self.serial.flush()

    async def fetch_one(self):
        try:
            async with self._lock:
                data = await self.serial.read_until(b"\n")

            l = data.decode().strip().split(" ")
            l = list(map(float, l))

            if self.odometry_speeds_queue.full():
                for _ in range(int(self.odometry_speeds_queue.maxsize / 2)):
                    await self.odometry_speeds_queue.get()
            
            if len(l) == 5:
                await self.odometry_speeds_queue.put(l)

        except ValueError:
            pass

    async def fetch_task(self):
        self.running.set()
        while self.running.is_set():
            await self.fetch_one()

    async def close(self):
        """Close serial connection"""
        await self.serial.close()