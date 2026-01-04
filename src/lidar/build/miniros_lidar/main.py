from miniros import AsyncROSClient, datatypes
# import adafruit_rplidar as pyrplidar
import rplidar
import asyncio


class LidarClient(AsyncROSClient):
    def __init__(self, ip = "localhost", port = 3000):
        super().__init__("lidar", ip, port)

        self.lidar = rplidar.RPLidar(
            port="/dev/ttyLidar",
            baudrate=115200
        )
        
        self.lidar.stop_motor()


async def main():
    client = LidarClient()

    async def run():
        await client.wait()

        ldr_topic = await client.topic("lidar", datatypes.LidarDatatype)

        client.lidar.start_motor()

        while True:
            try:               
                client.lidar.clean_input()
                                 
                # x = 0
                for scan in client.lidar.iter_scans(min_len=360):
                    x += 1
                    
                    # if x < 3:
                    #     continue
                    
                    # x = 0
                    
                    _, angles, distances = zip(*scan)
                
                    await ldr_topic.post(
                        datatypes.LidarDatatype(
                            list(distances),
                            list(angles),
                        )
                    )

            except rplidar.RPLidarException as e:
                print(f"Lidar exception: {e}. Reconnecting...")
                
                client.lidar.stop()
                client.lidar.disconnect()

                await asyncio.sleep(0.4)
                
                client.lidar.connect()
                
            except Exception as e:
                print(f"Unexpected error: {e}")

                await asyncio.sleep(0.2)
                
                
    await asyncio.gather(
        client.run(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
