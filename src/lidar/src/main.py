from miniros import AsyncROSClient, datatypes, threaded
import pyrplidarsdk
import platform
import asyncio
import signal
import time


class LidarClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("lidar", ip, port)

        # TODO: configurable ports
        self.lidar = pyrplidarsdk.RplidarDriver(
            port="COM3" if platform.system() == "Windows" else "/dev/ttyUSB1",
            baudrate=115200,
        )

        if not self.lidar.connect():
            print("[] failed to connect")
            exit(1)

        self.last_ping_time = time.time()
        self.lidar_queue = asyncio.Queue(100)
        self.running = asyncio.Event()

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
            if self.lidar_queue.full():
                for _ in range(int(self.lidar_queue.maxsize / 2)):
                    self.lidar_queue.get_nowait()

            self.lidar_queue.put_nowait((angles, distances))


async def main():
    client = LidarClient()
    t = client.scans_job()

    def shutdown(sig, frame):
        client.running.clear()
        client.lidar_queue.shutdown(immediate=True)

        client.lidar.stop_scan()
        client.lidar.disconnect()

        t.join()

    async def run():
        await client.wait()

        ldr_topic = await client.topic("lidar", datatypes.LidarDatatype)

        while True:
            angles, distances = await client.lidar_queue.get()

            await ldr_topic.post(
                datatypes.LidarDatatype(
                    distances=distances,
                    angles=angles,
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
