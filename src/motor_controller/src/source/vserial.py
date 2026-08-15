from asyncio import Queue, Event
import serial_asyncio as serial


class ArduinoSerial:
    def __init__(self, port, baudrate=115200):
        """Initialize serial connection"""
        self.reader, self.writer = None, None
        self.odometry_speeds_queue = Queue(100)
        self.running = Event()

        self.port = port
        self.baudrate = baudrate
        
    async def connect(self):
        self.reader, self.writer = await serial.open_serial_connection(url=self.port, baudrate=self.baudrate)

    async def set_speeds(self, left: float, right: float):
        data = f"S {left:.4f} {right:.4f}"
        self.writer.write(data.encode())
        await self.writer.drain()

    async def reset_position(self, newX: float, newY: float, newTheta: float):
        data = f"R {newX:.4f} {newY:.4f} {newTheta:.4f}"
        self.writer.write(data.encode())
        await self.writer.drain()

    async def fetch_one(self):
        data = await self.reader.readuntil(b"\n")
        
        try:
            l = data.decode().strip().split(" ")
            l = list(map(float, l))
        except (ValueError, UnicodeDecodeError) as e:
            print(e)
            return

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
        self.writer.close()
        await self.writer.wait_closed()