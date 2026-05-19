from miniros import AsyncROSClient, datatypes
import asyncio
import pyrplidarsdk
import platform
import time


class LidarClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("lidar", ip, port)

        self.lidar = pyrplidarsdk.RplidarDriver(
            port="COM3" if platform.system() == "Windows" else "/dev/lidar",
            baudrate=115200,
        )

        self.lidar.stop_scan()

        if not self.lidar.connect():
            print("[] failed to connect")
            exit(1)

        print(self.lidar.get_health())
        print(self.lidar.get_device_info())

        self.last_ping_time = time.time()

    async def on_ping(self, _, node):
        self.last_ping_time = time.time()

    def iter_scans(self, *args, **kwargs):
        try:
            while True:
                dat = self.lidar.get_scan_data()

                if dat is not None:
                    yield dat

                else:
                    print("[] null")

        except Exception as _:
            print("[e] exception occurred")
            self.lidar.stop_scan()

    def __del__(self):
        self.lidar.stop_scan()
        self.lidar.disconnect()


async def main():
    client = LidarClient()

    async def run():
        await client.wait()

        ldr_topic = await client.topic("lidar", datatypes.LidarDatatype)
        
        client.lidar.start_scan()
        while True:
            if time.time() - client.last_ping_time > 7.0:
                await asyncio.sleep(1.0)

            else:
                # if True:
                for scan in client.iter_scans():
                    # radians // meters // %
                    angles, distances, _quality = scan

                    await ldr_topic.post(
                        datatypes.LidarDatatype(
                            distances,
                            angles,
                        )
                    )

                    if time.time() - client.last_ping_time > 7.0:
                        break

    await asyncio.gather(
        client.run(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
