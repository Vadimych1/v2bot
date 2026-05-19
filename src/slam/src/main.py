import cv2
import asyncio
import numpy as np
from asyncio import Queue
from queue import Queue as SyncQueue
from miniros_constants import main as cnst
from miniros import AsyncROSClient, datatypes
from miniros.util.decorators import aparsedata, threaded
from miniros_slam.source.datatypes import SLAMMap
from miniros.util.datatypes import Movement, Vector

# import miniros_breezyslam.sensors as sensors # old slam
# import miniros_breezyslam.algorithms as algos # old slam

from yag_slam.graph_slam import GraphSlam, make_near_scan_visitor
from yag_slam.scan_matching import Scan2DMatcherCpp
from yag_slam.graph import do_breadth_first_traversal
from karto_scanmatcher import create_occupancy_grid, Pose2
from yag_slam.models import LocalizedRangeScan
from yag_slam.splicing import map_to_graph
from tiny_tf.tf import Transform
from tiny_tf.transformations import euler_from_quaternion, quaternion_from_euler

def movement2pose(msg: Movement) -> Pose2:
    t = msg.pos
    r = msg.ang
    return Pose2(t.x, t.y, r.z)

def movement2transform(msg: Movement) -> Transform:
    t = msg.pos
    r = msg.ang
    return Transform(t.x, t.y, t.z, *quaternion_from_euler(r.x, r.y, r.z))

def pose2movement(pose: Pose2) -> Movement:
    return Movement(
        Vector(pose.x, pose.y, 0),
        Vector(0, 0, pose.yaw)
    )

class SLAMClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("slam", ip, port)

        self.mapper = None
        self.last_pose = Movement(Vector(0, 10, 0), Vector(0, 0, 0))
        self.scans = []

        # self.slam = algos.RMHC_SLAM(
        #     sensors.RPLidarA1(),
        #     cnst.MAP_SIZE_PX,
        #     cnst.MAP_SIZE_MET,
        #     hole_width_mm=150,
        #     # sigma_theta_degrees=5,
        # )
        
        # self.slam = yag_slam.

        self.dxy = 0
        self.dtheta = 0
        self.dt = 0
        
        self.scan_queue: SyncQueue[tuple[datatypes.LidarDatatype, datatypes.Movement]] = SyncQueue(16)
        self.pos_queue: Queue[datatypes.Movement] = Queue(16)
        self.map_queue: Queue[bool] = Queue(16)
        
    def _setup_mapper(self):
        seq_scan_matcher_config = {
            "angle_variance_penalty": 0.0349, # 0.349
            "distance_variance_penalty": 0.03, # 0.3
            "coarse_search_angle_offset": 0.1, # 0.349
            "coarse_angle_resolution": 0.0349,
            "fine_search_angle_resolution": 0.00349,
            "use_response_expansion": True,
            "range_threshold": 20,
            "minimum_angle_penalty": 0.9,
            "search_size": 1.0,
            "resolution": 0.01,
            "smear_deviation": 0.09
        }
        
        loop_scan_matcher_config = seq_scan_matcher_config.copy()
        loop_scan_matcher_config.update({
            "search_size": 4.0,
            "resolution": 0.05,
            "smear_deviation": 0.03,
        })
        
        seq_matcher = Scan2DMatcherCpp(config_dict=seq_scan_matcher_config)
        loop_matcher = Scan2DMatcherCpp(loop_scan_matcher_config, loop=True)
        
        self.mapper = GraphSlam(
            seq_matcher,
            loop_matcher,
            scan_buffer_len=10,
            min_response_coarse=0.6,
            min_response_fine=0.7,
        )
    
    def _make_map(self):
        grid = self.mapper.make_occupancy_grid(0.05, 20)
        im = grid.image
        
        # static_only = 255 - im.copy()
        # static_only[static_only < 200] = 0
        # num_conn, mask, stats, position = cv2.connectedComponentsWithStats(static_only)
        
        # for ii, stat in enumerate(stats):
        #     if stat[-1] < 5:
        #         im[mask == ii] = 255
        
        # im = im.astype('int16')
        
        # im[im == 0] = 100
        # im[im == 200] = -1
        # im[im == 255] = 0
        
        return im

    @threaded()
    def process_scans(self):
        while True:
            _d = self.scan_queue.get()
            
            scan: datatypes.LidarDatatype = _d[0]
            pose: Movement = _d[1]
            
            ranges, angles = zip(*sorted(zip(scan.distances, scan.angles), key=lambda x: x[1]))
            step = (angles[-1] - angles[0]) / len(angles)
            start_ang = min(angles)
            end_ang = max(angles)
            
            data = LocalizedRangeScan(
                ranges,
                start_ang,
                end_ang,
                step,
                0,
                20,
                20,
                pose.pos.x,
                pose.pos.y,
                pose.ang.z,
            )
            
            data.odom_pose = Pose2()
            
            res, closed = self.mapper.process_scan(data)
            
            if res is None:
                continue
            
            else:
                self.last_pose = pose2movement(res.best_pose)
            
            try:
                self.pos_queue.put_nowait(self.last_pose)
            except Exception as e:
                print(e)
            
            try:
                self.map_queue.put_nowait(True)
            except Exception as e:
                print(e)
        
    @aparsedata(datatypes.LidarDatatype)
    async def on_lidar_lidar(self, data: datatypes.LidarDatatype):
        self.scan_queue.put((data, self.last_pose))
        
        # print("Got lidar data")
        
        ## old logic with breezyslam
        ## wont work now
         
        # if len(dist) < 721 or len(ang) < 721:
        #     return

        # self.slam.update(
        #     scans_mm=dist[:720:2],
        #     scan_angles_degrees=ang[:720:2],
        #     # pose_change=(self.dxy, self.dtheta, self.dt),
        # )
    
        # self.slam.update(
        #     scans_mm=dist[1:721:2],
        #     scan_angles_degrees=ang[1:721:2],
        #     # pose_change=(0, 0, 0),
        # )
    
        # self.dx = 0
        # self.dtheta = 0
        # self.dt = 0

        # self.slam.getmap(self.map)
        # self.pos = self.slam.getpos()
        
        await self.anon("lidar", "ping", b'hi')

    @aparsedata(datatypes.Vector)
    async def on_motorcontroller_odometry(self, data: datatypes.Vector):
        self.dxy += data.x
        self.dtheta += data.y
        self.dt += data.z

        print("[] got odometry data {dxy dtheta dt}:", data.x, data.y, data.z)


async def main():
    client = SLAMClient()
    client._setup_mapper()

    async def post_pos_job():
        await client.wait()
        
        pos_topic = await client.topic("pose", Movement)
        
        while True:
            pos = await client.pos_queue.get()
            
            # print("Got slam pos")
            
            print("POS:", pos.pos, pos.ang)
            
            await pos_topic.post(pos)

    async def post_map_job():
        await client.wait()
        
        map_topic = await client.topic("map", datatypes.NumpyArray)

        n = 0
        while True:
            _ = await client.map_queue.get()
            n += 1
            
            # print("Got map")

            if n % 3 == 0:
                _map = client._make_map()
                cv2.imwrite("map.png", _map)
                
                await map_topic.post(_map)
                
                # print("Posting map")

    async def run():
        await client.wait()
        await client.anon("lidar", "ping", b'hi')

    client.process_scans()

    await asyncio.gather(
        client.run(),
        post_map_job(),
        post_pos_job(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
