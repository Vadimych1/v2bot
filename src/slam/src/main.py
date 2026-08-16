# import cv2
import asyncio
import multiprocessing as mp
from queue import Empty
from miniros import AsyncROSClient, datatypes
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.datatypes import Movement, Vector
from yag_slam.graph_slam import GraphSlam
from yag_slam.scan_matching import Scan2DMatcherCpp
from karto_scanmatcher import Pose2
from yag_slam.models import LocalizedRangeScan
from tiny_tf.tf import Transform
from tiny_tf.transformations import quaternion_from_euler

# import time


def movement2transform(msg: Movement) -> Transform:
    t = msg.pos
    r = msg.ang
    return Transform(t.x, t.y, t.z, *quaternion_from_euler(r.x, r.y, r.z))


def pose2movement(pose: Pose2) -> Movement:
    return Movement(Vector(pose.x, pose.y, 0), Vector(0, 0, pose.yaw))


def slam_worker(input_queue: mp.Queue, output_queue: mp.Queue):
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

    mapper = GraphSlam(
        seq_matcher,
        loop_matcher,
        scan_buffer_len=10,
        min_response_coarse=0.3,
        min_response_fine=0.4,
    )

    map_counter = 0
    resolution = 0.05

    while True:
        try:
            scan = input_queue.get(timeout=1)

            # shutdown signal
            if scan is None:
                break

            scan = datatypes.LidarDatatype.decode(scan)

        except Empty:
            continue

        pose: Vector = scan.pos

        # todo: check if this line necessary
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

        res, closed = mapper.process_scan(data)
        if res is None or res.best_pose is None:
            continue

        movement_msg = pose2movement(res.best_pose)
        mmap_data = None

        if map_counter % 3 == 0:
            grid = mapper.make_occupancy_grid(resolution, 8)
            mmap_data = SLAMOffsetMap(
                grid=grid.image,
                width=grid.width,
                height=grid.height,
                offset_x=-int(grid.offset.x / resolution),  # offset is negative because
                offset_y=-int(
                    grid.offset.y / resolution
                ),  # it is given in world coordinates
                resolution=resolution,
            )

            mmap_data = SLAMOffsetMap.encode(mmap_data)

        map_counter += 1
        output_queue.put((Movement.encode(movement_msg), mmap_data))


class SLAMClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("slam", ip, port)

        # self.mapper = None

        self.dxy = 0
        self.dtheta = 0
        self.dt = 0

        self.scan_queue = mp.Queue(10)
        self.result_queue = mp.Queue()

        self.last_pos: datatypes.Movement | None = None
        self.last_map: tuple | None = None

        self.running = asyncio.Event()
        self.slam_proc = None

        # self._map_counter = 0

    def start_slam_process(self):
        self.running.set()
        self.slam_proc = mp.Process(
            target=slam_worker, args=(self.scan_queue, self.result_queue), daemon=True
        )
        self.slam_proc.start()

    async def consume_results(self):
        loop = asyncio.get_running_loop()
        while self.running.is_set():
            try:
                res = await loop.run_in_executor(None, self.result_queue.get, True, 1.0)
                movement_data, mmap_data = res

                self.last_pos = movement_data
                if mmap_data is not None:
                    self.last_map = mmap_data

            except Empty:
                await asyncio.sleep(0.01)

    # we are passing raw bytes to slam process for efficiency
    async def on_lidar_lidar(self, data):
        if self.scan_queue.full():
            try:
                self.scan_queue.get_nowait()

            except Empty:
                pass

        try:
            self.scan_queue.put_nowait(data)

        except Exception:
            pass

        # await self.anon("lidar", "ping", b"hi")

    def stop(self):
        self.running.clear()
        self.scan_queue.put(None)

        if self.slam_proc:
            self.slam_proc.join()


async def main():
    client = SLAMClient()
    client.start_slam_process()

    # posts last pos at 10hz
    async def post_pos_job():
        await client.wait()

        # miniros allows you to send and receive data with any type you want
        # here we sending pre-encoded Movement bytes but on the other end we
        # will parse it as Movement using @aparsedata
        pos_topic = await client.topic("pose", datatypes.Bytes)

        while client.running.is_set():
            if client.last_pos is not None:
                await pos_topic.post(client.last_pos)

            await asyncio.sleep(1 / 10)

    # posts last map at 2hz
    async def post_map_job():
        await client.wait()

        map_topic = await client.topic("map", datatypes.Bytes)

        while client.running.is_set():
            if client.last_map is not None:
                await map_topic.post(client.last_map)

            await asyncio.sleep(1 / 2)

    async def run():
        await client.wait()
        await client.anon("lidar", "ping", b"hi")

    try:
        await asyncio.gather(
            client.run(),
            client.consume_results(),
            post_map_job(),
            post_pos_job(),
            run(),
        )

    finally:
        client.stop()


if __name__ == "__main__":
    asyncio.run(main())
