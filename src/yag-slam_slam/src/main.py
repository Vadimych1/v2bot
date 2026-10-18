import asyncio
from queue import Empty
import multiprocessing as mp

from karto_scanmatcher import Pose2
from yag_slam.graph_slam import GraphSlam
from yag_slam.models import LocalizedRangeScan
from yag_slam.scan_matching import Scan2DMatcherCpp
from tiny_tf.tf import Transform
from tiny_tf.transformations import quaternion_from_euler

import time
from miniros_configurator import get_config
from miniros import AsyncROSClient, datatypes
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.datatypes import Movement3DoF, TimedMovement3DoF


def movement2transform(msg) -> Transform:
    return Transform(msg.x, msg.y, 0, *quaternion_from_euler(0, 0, msg.theta))


def pose2movement(pose: Pose2, timestamp: float):
    return TimedMovement3DoF(
        movement=Movement3DoF(
            x=pose.x,
            y=pose.y,
            theta=pose.yaw,
        ),
        timestamp=timestamp,
    )


def slam_worker(input_queue: mp.Queue, output_queue: mp.Queue):
    seq_scan_matcher_config = get_config("yag-slam.seq_scan_matcher")
    loop_scan_matcher_config = get_config("yag-slam.loop_scan_matcher")

    seq_matcher = Scan2DMatcherCpp(seq_scan_matcher_config)
    loop_matcher = Scan2DMatcherCpp(loop_scan_matcher_config, loop=True)

    mapper = GraphSlam(
        seq_matcher,
        loop_matcher,
        scan_buffer_len=get_config("yag-slam.scan_buffer_len"),
        min_response_coarse=get_config("yag-slam.min_response_coarse"),
        min_response_fine=get_config("yag-slam.min_response_fine"),
    )

    map_counter = 0
    map_resolution = get_config("yag-slam.mapping.resolution")
    map_range_threshold = get_config("yag-slam.mapping.range_threshold")

    lidar_min_distance = get_config("yag-slam.lidar.min_distance")
    lidar_max_distance = get_config("yag-slam.lidar.max_distance")
    lidar_range_threshold = get_config("yag-slam.lidar.range_threshold")

    nq = 0
    nq_start_time = time.time()

    while True:
        nq += 1
        
        try:
            scan = input_queue.get(timeout=1)

            # shutdown signal
            if scan is None:
                break

            scan = datatypes.Lidar2D.decode(scan)

        except Empty:
            continue

        pose = scan.pos

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
            lidar_min_distance,
            lidar_max_distance,
            lidar_range_threshold,
            pose.x,
            pose.y,
            pose.theta,
        )

        res, closed = mapper.process_scan(data)
        if res is None or res.best_pose is None:
            continue

        movement_msg = pose2movement(res.best_pose, scan.timestamp)
        mmap_data = None
        
        if nq % 30 == 0:
            print(f"Running at {nq / (time.time() - nq_start_time)}Hz")

        if map_counter % 3 == 0:
            grid = mapper.make_occupancy_grid(
                map_resolution,
                map_range_threshold,
            )
            mmap_data = SLAMOffsetMap(
                grid=grid.image,
                width=grid.width,
                height=grid.height,
                offset_x=-int(grid.offset.x / map_resolution),
                offset_y=-int(grid.offset.y / map_resolution),
                resolution=map_resolution,
            )

            mmap_data = SLAMOffsetMap.encode(mmap_data)

        map_counter += 1
        output_queue.put((TimedMovement3DoF.encode(movement_msg), mmap_data))


class SLAMClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("slam", ip, port)

        self.dxy = 0
        self.dtheta = 0
        self.dt = 0

        self.scan_queue = mp.Queue(get_config("yag-slam.miniros.scan_queue_len"))
        self.result_queue = mp.Queue()

        self.last_pos: datatypes.Movement | None = None
        self.last_map: tuple | None = None

        self.running = asyncio.Event()
        self.slam_proc = None

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

    map_post_delay = get_config("yag-slam.miniros.map_post_delay")
    pose_post_delay = get_config("yag-slam.miniros.pose_post_delay")

    async def post_pos_job():
        await client.wait()
    
        # miniros allows you to send and receive data with any type you want
        # here we are sending pre-encoded Movement bytes but on the other end
        # we will parse it as Movement using @aparsedata
        pos_topic = await client.topic("pose", datatypes.Bytes)

        while client.running.is_set():
            if client.last_pos is not None:
                await pos_topic.post(client.last_pos)

            await asyncio.sleep(pose_post_delay)

    async def post_map_job():
        await client.wait()

        map_topic = await client.topic("map", datatypes.Bytes)

        while client.running.is_set():
            if client.last_map is not None:
                await map_topic.post(client.last_map)

            await asyncio.sleep(map_post_delay)

    async def run():
        await client.wait()
        # await client.anon("lidar", "ping", b"hi")

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
