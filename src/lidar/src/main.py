import pyrplidarsdk
import time

import platform
import asyncio
import signal

from miniros import AsyncROSClient, datatypes, threaded, aparsedata
from miniros_configurator import get_config


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

        self.last_lidar = None
        self.new_lidar_event = asyncio.Event()

        self.running = asyncio.Event()

        self._loop = asyncio.get_event_loop()
        self.current_position = datatypes.Vector(0, 0, 0)

    async def on_ping(self, _, node):
        self.last_ping_time = time.time()

    def iter_scans(self):
        self.running.set()

        while self.running.is_set():
            dat = self.lidar.get_scan_data()

            if dat is not None:
                yield dat

            else:
                print("[] null")

    @threaded()
    def scans_job(self):
        self.lidar.start_scan()

        for angles, distances, quality in self.iter_scans():
            self.last_lidar = (angles, distances, self.current_position.copy())
            self.new_lidar_event.set()

    @aparsedata(datatypes.Vector)
    async def on_motorcontroller_odometry(self, pos: datatypes.Vector):
        self.current_position = pos


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

        ldr_topic = await client.topic("lidar", datatypes.LidarDatatype)

        while True:
            await client.new_lidar_event.wait()
            client.new_lidar_event.clear()

            if client.last_lidar is None:
                return

            angles, distances, pos = client.last_lidar

            await ldr_topic.post(
                datatypes.LidarDatatype(distances=distances, angles=angles, pos=pos)
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
