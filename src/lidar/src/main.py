from miniros import AsyncROSClient, datatypes
import asyncio
import rplidar
import platform
import time


class LidarClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("lidar", ip, port)

        self.lidar = rplidar.RPLidar(
            port="COM4" if platform.system() == "Windows" else "/dev/ttyLidar",
            baudrate=115200,
        )

        self.lidar.stop_motor()

        self.last_ping_time = time.time()

    async def on_ping(self, _, node):
        self.last_ping_time = time.time()


async def main():
    client = LidarClient()

    async def run():
        await client.wait()

        ldr_topic = await client.topic("lidar", datatypes.LidarDatatype)

        while True:
            if time.time() - client.last_ping_time > 7.0:
                client.lidar.stop_motor()
                asyncio.sleep(1.0)

            else:
                try:
                    client.lidar.clean_input()
                    client.lidar.start_motor()

                    for scan in client.lidar.iter_scans(min_len=360, max_buf_meas=540):
                        _, angles, distances = zip(*scan)

                        await ldr_topic.post(
                            datatypes.LidarDatatype(
                                list(distances),
                                list(angles),
                            )
                        )

                        if time.time() - client.last_ping_time > 7.0:
                            break

                except rplidar.RPLidarException as e:
                    print(e)
                    print(
                        "[] Lidar exception (see full exception above). Reconnecting..."
                    )

                    client.lidar.stop()
                    client.lidar.disconnect()

                    await asyncio.sleep(0.4)

                    client.lidar.connect()

                except Exception as e:
                    print(f"[] Unexpected error occurred: {e}")

                    await asyncio.sleep(0.2)

    await asyncio.gather(
        client.run(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
