from asyncio import Event, Lock
import wireio
import time


class ArduinoSerial:
    def __init__(self, port, baudrate=115200):
        """Initialize serial connection"""
        self.serial = wireio.AsyncSerial(port, baudrate, timeout=1)
        
        self.last_odometry = None
        self.odometry_event = Event()
        
        self.running = Event()
        self._lock = Lock()

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
            # TODO: check if that fixed bad formats and value errors
            data = await self.serial.read_until(b"\n", 128)
            line = data.decode(errors="ignore").strip()
            
            if not line or line[0] not in "-0123456789":
                return

            parts = line.split(" ")
            if len(parts) != 5:
                return
            
            l = list(map(float, parts))
            self.last_odometry = l
            self.odometry_event.set()

        except UnicodeDecodeError:
            pass

        except ValueError as e:
            print(f"VALUE ERROR: {e}")
            pass

    async def fetch_task(self):
        nq = 0
        nq_start_time = time.time()

        self.running.set()
        while self.running.is_set():
            nq += 1
            await self.fetch_one()
            
            if (nq + 1) % 30 == 0:
                print(f"Running at {nq / (time.time() - nq_start_time)}hz")
    
            if (nq + 1) % 100 == 0:
                nq = 0
                nq_start_time = time.time()

    async def close(self):
        """Close serial connection"""
        self.running.clear()
        await self.serial.close()