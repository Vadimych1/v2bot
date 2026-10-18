import pyrplidarsdk
import math
import time

import platform
import asyncio
import signal
import multiprocessing as mp
from queue import Empty

from miniros import AsyncROSClient, datatypes, aparsedata
from miniros_configurator import get_config


def scan_worker(port: str, baudrate: int, running, queue: mp.Queue):
    lidar = pyrplidarsdk.RplidarDriver(
        port=port,
        baudrate=baudrate,
    )
    
    if not lidar.connect():
        print("[] failed to connect", flush=True)
        exit(1)
    
    lidar.start_scan()

    while running.is_set():
        dat = lidar.get_scan_data()
        
        if dat is not None:
            angles, distances, quality = dat
            timestamp = time.time()
            
            try:
                if queue.full():
                    try:
                        queue.get_nowait()
                    except Empty:
                        pass
                
                queue.put_nowait((angles, distances, timestamp))
            
            except Exception:
                pass
        
        else:
            print("[] null scan", flush=True)
    
    lidar.stop_scan()
    lidar.disconnect()

class FixedSizeList(list):
    def __init__(self, maxlen, iterable=()):
        self.maxlen = maxlen
        super().__init__(iterable)
        # Если переданный итерируемый объект длиннее maxlen, обрезаем начало
        if len(self) > maxlen:
            del self[: len(self) - maxlen]

    def append(self, item):
        super().append(item)
        if len(self) > self.maxlen:
            self.pop(0)  # удаляем самый старый элемент

    def insert(self, index, item):
        super().insert(index, item)
        if len(self) > self.maxlen:
            self.pop(0)

    def extend(self, iterable):
        super().extend(iterable)
        while len(self) > self.maxlen:
            self.pop(0)


class LidarClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("lidar", ip, port)

        win_port = get_config("lidar.win_port")
        ldr_port = get_config("lidar.port")
        self.port = win_port if platform.system() == "Windows" else ldr_port
        self.baudrate = get_config("lidar.baudrate")

        self.lidar_proc = mp.Process(target=scan_worker)

        self.last_ping_time = time.time()
        self.lidar_queue = mp.Queue(10)

        self._odometry_event = asyncio.Event()
        self._odometry = FixedSizeList(
            10,
            [
                datatypes.TimedMovement3DoF(
                    timestamp=time.time(),
                    movement=datatypes.Movement3DoF(
                        x=0,
                        y=0,
                        theta=0,
                    ),
                ),
                datatypes.TimedMovement3DoF(
                    timestamp=time.time() - 1,
                    movement=datatypes.Movement3DoF(
                        x=0,
                        y=0,
                        theta=0,
                    ),
                ),
                datatypes.TimedMovement3DoF(
                    timestamp=time.time() - 2,
                    movement=datatypes.Movement3DoF(
                        x=0,
                        y=0,
                        theta=0,
                    ),
                ),
            ],
        )

        self.running = mp.Event()

    def scans_job(self):
        if self.lidar_proc is not None and self.lidar_proc.is_alive():
            return
        
        self.running.set()
        self.lidar_proc = mp.Process(target=scan_worker, args=(self.port, self.baudrate, self.running, self.lidar_queue))
        self.lidar_proc.start()
        
    async def get_scan_async(self):
        loop = asyncio.get_running_loop()
        
        try:
            data = await loop.run_in_executor(None, self.lidar_queue.get, True, 3)
            return data
        except Empty:
            return None

    async def on_ping(self, _, node):
        self.last_ping_time = time.time()

    @aparsedata(datatypes.TimedMovement3DoF)
    async def on_motorcontroller_odometry(self, pos):
        self._odometry.append(pos)
        self._odometry_event.set()
        
    async def wait_for_odometry_ts(self, timestamp: float, max_tries: int = 2):
        """
        Waits for two odometry values that are `before` and `after` timestamp
        to interpolate them after
        """

        for i, odom in enumerate(self._odometry):
            if odom.timestamp >= timestamp:
                if i > 0:
                    prev_odom = self._odometry[i - 1]
                    return self._interpolate(odom, prev_odom, timestamp)

                else:
                    return self._odometry[0]

        self._odometry_event.clear()

        for _ in range(max_tries):
            await self._odometry_event.wait()
            self._odometry_event.clear()

            if self._odometry[-1].timestamp >= timestamp:
                return self._interpolate(
                    self._odometry[-1], self._odometry[-2], timestamp
                )

        return self._odometry[-1]

    def _normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi

        while angle < -math.pi:
            angle += 2 * math.pi

        return angle

    def _interpolate(self, odom, prev_odom, timestamp):
        mov = odom.movement
        prev_mov = prev_odom.movement

        alpha = (timestamp - prev_odom.timestamp) / (odom.timestamp - prev_odom.timestamp)
        x = prev_mov.x + alpha * (mov.x - prev_mov.x)
        y = prev_mov.y + alpha * (mov.y - prev_mov.y)
        theta = prev_mov.theta + alpha * (mov.theta - prev_mov.theta)

        return datatypes.TimedMovement3DoF(
            timestamp=timestamp,
            movement=datatypes.Movement3DoF(
                x=x,
                y=y,
                theta=self._normalize_angle(theta),
            ),
        )


async def main():
    nq_start_time = time.time()
    nq = 0

    client = LidarClient()
    client.scans_job()

    def shutdown(sig, frame):
        client.running.clear()

    async def run():
        await client.wait()

        ldr_topic = await client.topic("lidar", datatypes.Lidar2D)

        while client.running.is_set():
            nq += 1
            
            lidar = await client.get_scan_async()

            if lidar is None:
                continue

            angles, distances, ts = lidar
            pos = await client.wait_for_odometry_ts(ts, max_tries=3)

            await ldr_topic.post(
                datatypes.Lidar2D(
                    distances=distances,
                    angles=angles,
                    pos=pos.movement,
                    timestamp=ts,
                )
            )
            
            if nq % 30 == 0:
                print(f"Running at {nq / (time.time() - nq_start_time)}Hz")

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        await asyncio.gather(
            client.run(),
            run(),
        )

    except (KeyboardInterrupt, asyncio.QueueShutDown):
        pass


if __name__ == "__main__":
    asyncio.run(main())
