import pyrplidarsdk
import math
import time

import platform
import asyncio
import signal

from miniros import AsyncROSClient, datatypes, threaded, aparsedata
from miniros_configurator import get_config


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


class FixedSizeQueue:
    def __init__(self, maxsize=10) -> None:
        self._queue = asyncio.Queue(maxsize)

    async def get(self):
        return await self._queue.get()

    async def put(self, data):
        if self._queue.full():
            self._queue.get_nowait()

        await self.put(data)

    def put_nowait(self, data):
        if self._queue.full():
            self._queue.get_nowait()

        self._queue.put_nowait(data)


class LidarClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("lidar", ip, port)

        win_port = get_config("lidar.win_port")
        ldr_port = get_config("lidar.port")
        baudrate = get_config("lidar.baudrate")

        self.lidar = pyrplidarsdk.RplidarDriver(
            port=win_port if platform.system() == "Windows" else ldr_port,
            baudrate=baudrate,
        )

        if not self.lidar.connect():
            print("[] failed to connect")
            exit(1)

        self.last_ping_time = time.time()
        self.lidar_queue = FixedSizeQueue(10)

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

        self.running = asyncio.Event()

    async def on_ping(self, _, node):
        self.last_ping_time = time.time()

    def iter_scans(self):
        self.running.set()

        while self.running.is_set():
            dat = self.lidar.get_scan_data()

            if dat is not None:
                yield (dat, time.time())

            else:
                print("[] null")

    @threaded()
    def scans_job(self):
        self.lidar.start_scan()

        for (angles, distances, quality), t in self.iter_scans():
            self.lidar_queue.put_nowait((angles, distances, t))

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
                    print("OK, got from stack")
                    return self._interpolate(odom, prev_odom, timestamp)

                else:
                    print("too late, got last")
                    return self._odometry[0]

        self._odometry_event.clear()

        for _ in range(max_tries):
            await self._odometry_event.wait()
            self._odometry_event.clear()

            if self._odometry[-1].timestamp >= timestamp:
                print("OK, got from new stack")
                return self._interpolate(
                    self._odometry[-1], self._odometry[-2], timestamp
                )

        print("too early, got latest")
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

        alpha = (timestamp - prev_mov.timestamp) / (mov.timestamp - prev_mov.timestamp)
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
    client = LidarClient()
    t = client.scans_job()

    def shutdown(sig, frame):
        client.running.clear()
        client.lidar.stop_scan()
        client.lidar.disconnect()

        t.join()

    async def run():
        await client.wait()

        ldr_topic = await client.topic("lidar", datatypes.Lidar2D)

        while client.running.is_set():
            lidar = await client.lidar_queue.get()

            if lidar is None:
                continue

            angles, distances, ts = lidar
            pos = await client.wait_for_odometry_ts(ts, max_tries=2)

            await ldr_topic.post(
                datatypes.Lidar2D(
                    distances=distances,
                    angles=angles,
                    pos=pos.movement,
                    timestamp=ts,
                )
            )

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
