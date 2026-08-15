from asyncio import Queue
import cv2
import asyncio
from queue import Queue as SyncQueue, Empty
from miniros import AsyncROSClient, datatypes
from miniros.util.decorators import aparsedata, threaded
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.datatypes import Movement, Vector
from yag_slam.graph_slam import GraphSlam
from yag_slam.scan_matching import Scan2DMatcherCpp
from karto_scanmatcher import Pose2
from yag_slam.models import LocalizedRangeScan
from tiny_tf.tf import Transform
from tiny_tf.transformations import quaternion_from_euler
import time


def movement2transform(msg: Movement) -> Transform:
    t = msg.pos
    r = msg.ang
    return Transform(t.x, t.y, t.z, *quaternion_from_euler(r.x, r.y, r.z))


def pose2movement(pose: Pose2) -> Movement:
    return Movement(Vector(pose.x, pose.y, 0), Vector(0, 0, pose.yaw))


class SLAMClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("slam", ip, port)

        self.mapper = None

        self.dxy = 0
        self.dtheta = 0
        self.dt = 0

        self.scan_queue = SyncQueue(
            16
        )  # SyncQueue[tuple[datatypes.LidarDatatype, datatypes.Movement]]
        self.pos_queue: Queue[datatypes.Movement] = Queue(16)
        self.map_queue: Queue[tuple] = Queue(16)

        self._map_counter = 0

        self.running = asyncio.Event()
        self._loop = asyncio.get_event_loop()

    def _setup_mapper(self):
        seq_scan_matcher_config = {
            "angle_variance_penalty": 0.349,
            "distance_variance_penalty": 0.3,
            "coarse_search_angle_offset": 0.349,
            "coarse_angle_resolution": 0.0349,
            "fine_search_angle_resolution": 0.0174,
            "use_response_expansion": True,
            "range_threshold": 20,
            "minimum_angle_penalty": 0.9,
            "search_size": 1.0,
            "resolution": 0.05,
            "smear_deviation": 0.09,
        }

        loop_scan_matcher_config = seq_scan_matcher_config.copy()
        loop_scan_matcher_config.update(
            {
                "search_size": 2.0,
                "resolution": 0.05,
                "smear_deviation": 0.03,
            }
        )

        seq_matcher = Scan2DMatcherCpp(seq_scan_matcher_config)
        loop_matcher = Scan2DMatcherCpp(loop_scan_matcher_config, loop=True)

        self.mapper = GraphSlam(
            seq_matcher,
            loop_matcher,
            scan_buffer_len=10,
            min_response_coarse=0.3,
            min_response_fine=0.4,
        )

    def _make_map(self, resolution=0.05):
        """
        Creates grid map

        Returns tuple of:
        - binary grid (255/0) map, where 255 is empty and 0 is full
        - width
        - height
        - offset_x (px)
        - offset_y (px)
        - resolution
        """

        grid = self.mapper.make_occupancy_grid(resolution, 8)
        # image = grid.image

        # full = image < 100
        # empty = image >= 100

        # image[full] = 0
        # image[empty] = 255

        return (
            grid.image,
            grid.width,
            grid.height,
            grid.offset.x,
            grid.offset.y,
            resolution,
        )

    @threaded()
    def process_scans(self):
        self.running.set()

        while self.running.is_set():        
            try:
                scan = self.scan_queue.get(timeout=1)

            except Empty:
                continue

            pose: Vector = scan.pos
            ranges, angles = zip(
                *sorted(zip(scan.distances, scan.angles), key=lambda x: x[1])
            )
            step = (angles[-1] - angles[0]) / len(angles)
            start_ang = min(angles)
            end_ang = max(angles)

            data = LocalizedRangeScan(
                ranges,
                start_ang,
                end_ang,
                step,
                0.04,
                20,
                8,
                pose.x,
                pose.y,
                pose.z,
            )

            res, closed = self.mapper.process_scan(data)

            if res is None or res.best_pose is None:
                continue

            if self.pos_queue.full():
                for _ in range(int(self.pos_queue.maxsize)):
                    self.pos_queue.get_nowait()
                    
            if self.map_queue.full():
                for _ in range(int(self.map_queue.maxsize)):
                    self.map_queue.get_nowait()

            asyncio.run_coroutine_threadsafe(self.pos_queue.put(pose2movement(res.best_pose)), self._loop)
            
            if self._map_counter % 3 == 0:
                mmap = self._make_map()
                asyncio.run_coroutine_threadsafe(self.map_queue.put(mmap), self._loop)
                
                if self._map_counter % 30 == 0:
                    print(f"[map] {time.time()}")
                    cv2.imwrite("map.png", mmap[0])

            self._map_counter += 1

    @aparsedata(datatypes.LidarDatatype)
    async def on_lidar_lidar(self, data):  # datatypes.LidarDatatype
        if self.scan_queue.full():
            for _ in range(int(self.scan_queue.maxsize / 2)):
                self.scan_queue.get_nowait()
            self.scan_queue.get_nowait()

        self.scan_queue.put(data)

        await self.anon("lidar", "ping", b"hi")


# TODO: make graceful shutdown
async def main():
    client = SLAMClient()
    client._setup_mapper()

    async def post_pos_job():
        await client.wait()

        pos_topic = await client.topic("pose", Movement)

        while client.running.is_set():
            pos = await client.pos_queue.get()
            await pos_topic.post(pos)

    async def post_map_job():
        await client.wait()

        map_topic = await client.topic("map", SLAMOffsetMap)

        while client.running.is_set():
            grid, width, height, ofs_x, ofs_y, resolution = await client.map_queue.get()

            # "grid": NumpyArray,
            # "width": Int,
            # "height": Int,
            # "offset_x": Int,
            # "offset_y": Int,
            # "resolution": Float,
            _map = SLAMOffsetMap(
                grid=grid,
                width=width,
                height=height,
                offset_x=-int(ofs_x / resolution),  # offset is negative because
                offset_y=-int(ofs_y / resolution),  # it is given in world coordinates
                resolution=resolution,
            )

            await map_topic.post(_map)

    async def run():
        await client.wait()
        await client.anon("lidar", "ping", b"hi")

    client.process_scans()

    await asyncio.gather(
        client.run(),
        post_map_job(),
        post_pos_job(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
