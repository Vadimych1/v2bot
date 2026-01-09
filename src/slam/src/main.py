import asyncio
import numpy as np
from miniros.util.util import Ticker
from miniros.util.datatypes import Vector
from miniros_constants import main as cnst
import miniros_breezyslam.sensors as sensors
from miniros import AsyncROSClient, datatypes
import miniros_breezyslam.algorithms as algos
from miniros.util.decorators import aparsedata
from miniros_slam.source.datatypes import SLAMMap


class SLAMClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("slam", ip, port)

        self.slam = algos.RMHC_SLAM(
            sensors.RPLidarA1(),
            cnst.MAP_SIZE_PX,
            cnst.MAP_SIZE_MET,
            hole_width_mm=130,
            sigma_theta_degrees=5
        )

        self.map = bytearray(cnst.MAP_SIZE_PX**2)
        self.pos = (0, 0, 0)
        
        self.dxy = 0
        self.dtheta = 0
        self.dt = 0

    @aparsedata(datatypes.LidarDatatype)
    async def on_lidar_lidar(self, data: datatypes.LidarDatatype):
        dist, ang = data.distances, data.angles

        self.slam.update(scans_mm=dist, scan_angles_degrees=ang, pose_change=(self.dxy, self.dtheta, self.dt))
        self.dx = 0
        self.dtheta = 0
        self.dt = 0

        self.slam.getmap(self.map)
        self.pos = self.slam.getpos()
        
    @aparsedata(datatypes.Vector)
    async def on_motorcontroller_odometry(self, data: datatypes.Vector):
        self.dxy += data.x
        self.dtheta += data.y
        self.dt += data.z
        
        print("Got data!", data.x, data.y, data.z)


async def main():
    client = SLAMClient()

    async def run():
        await client.wait()

        map_topic = await client.topic("map", SLAMMap)
        pos_topic = await client.topic("pose", Vector)

        ticker = Ticker(0.5)

        while True:
            await ticker.tick_async()

            x, y, theta = client.pos
            await pos_topic.post(Vector(x / 1000.0, y / 1000.0, np.deg2rad(theta)))
            await map_topic.post(SLAMMap(client.map))

            await client.anon("lidar", "ping", b"hi")

    await asyncio.gather(
        client.run(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
