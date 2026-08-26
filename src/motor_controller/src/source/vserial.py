from asyncio import Event, Lock
import wireio
import time


class ArduinoSerial:
    def __init__(self, port, baudrate=115200):
        """Initialize serial connection"""
        self.serial = wireio.AsyncSerial(port, baudrate, timeout=1)
        self.last_odometry = None
        
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
            data = await self.serial.read_until(b"\n", 32)

            l = data.decode().strip().split(" ")
            l = list(map(float, l))

            if len(l) == 5:
                self.last_odometry = l

        except ValueError:
            pass

    async def fetch_task(self):
                
        nq = 0
        nq_start_time = time.time()

        self.running.set()
        while self.running.is_set():
            nq += 1
            await self.fetch_one()
            
            if nq % 30 == 0:
                print(f"Running at {nq / (time.time() - nq_start_time)}Hz")
    

    async def close(self):
        """Close serial connection"""
        await self.serial.close()