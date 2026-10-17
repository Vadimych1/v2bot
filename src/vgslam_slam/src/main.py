import asyncio
from queue import Empty
import multiprocessing as mp

import time
from miniros_configurator import get_config
from miniros import AsyncROSClient, datatypes
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.datatypes import Movement3DoF, TimedMovement3DoF

import numpy as np
from vgslam import VGSLAM, PositionedCloud, scan_to_cloud

def slam_worker(input_queue: mp.Queue, output_queue: mp.Queue):
    map_counter = 0
    
    map_resolution = get_config("vgslam.map_resolution")
    voxel_resolution = get_config("vgslam.voxel_resolution")
    range_threshold = get_config("vgslam.range_threshold")
    occupied_threshold = get_config("vgslam.occupied_threshold")

    # TODO: add other arguments to the configuration
    mapper = VGSLAM(map_resolution=map_resolution)

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

        odometry_pose: datatypes.Vector = scan.pos
        angles = np.asarray(scan.angles)
        distances = np.asarray(scan.distances)
        pcl = scan_to_cloud(
            distances,
            angles,
            range_threshold
        )
        
        data = PositionedCloud(
            pcl,
            np.array([
                odometry_pose.x,
                odometry_pose.y,
                odometry_pose.z,
            ]),
            resolution=voxel_resolution,
        )

        pose, closed = mapper.process_scan(data)
        
        if pose is None:
            print("skipped scan")
            continue
        
        if closed:
            print("successful loop closure")

        movement_msg = TimedMovement3DoF(
            movement=Movement3DoF(
                x=pose[0],
                y=pose[1],
                theta=pose[2],
            ),
            timestamp=time.time()
        )
        mmap_data = None
        
        if nq % 30 == 0:
            print(f"Running at {nq / (time.time() - nq_start_time)}Hz")

        if map_counter % 3 == 0:
            grid = mapper.create_occupancy_grid(occupied_threshold)
            mmap_data = SLAMOffsetMap(
                grid=grid.grid,
                width=grid.width,
                height=grid.height,
                offset_x=grid.zero_offset_x_px,
                offset_y=grid.zero_offset_y_px,
                resolution=grid.resolution,
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

        self.scan_queue = mp.Queue(get_config("vgslam.miniros.scan_queue_len"))
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

    map_post_delay = get_config("vgslam.miniros.map_post_delay")
    pose_post_delay = get_config("vgslam.miniros.pose_post_delay")

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
