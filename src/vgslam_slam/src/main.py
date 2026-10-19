import asyncio
from queue import Empty
import multiprocessing as mp

import time
from miniros_configurator import get_config
from miniros import AsyncROSClient, datatypes, aparsedata
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
        try:
            scan = input_queue.get(timeout=1)

            # shutdown signal
            if scan is None:
                break

            x, y, theta, distances, angles = scan

        except Empty:
            continue

        nq += 1
        
        if (nq + 1) % 100 == 0:
            nq_start_time = time.time()
            nq = 0

        angles = np.asarray(scan.angles)
        distances = np.asarray(scan.distances)
        pcl = scan_to_cloud(
            distances,
            angles,
            range_threshold
        )
        
        data = PositionedCloud(
            pcl,
            np.array([x, y, theta]),
            resolution=voxel_resolution,
        )

        pose, closed = mapper.process_scan(data) # this takes ~1.2e-4 seconds to process in average on RPi5
        
        if pose is None:
            print("skipped scan", flush=True)
            continue
        
        if closed:
            print("successful loop closure", flush=True)

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
            print(f"Running at {nq / (time.time() - nq_start_time)}Hz", flush=True)

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


class FixedSizeList(list):
    def __init__(self, maxlen, iterable=()):
        self.maxlen = maxlen
        super().__init__(iterable)
        
        if len(self) > maxlen:
            del self[: len(self) - maxlen]

    def append(self, item):
        super().append(item)
        if len(self) > self.maxlen:
            self.pop(0)

    def insert(self, index, item):
        super().insert(index, item)
        if len(self) > self.maxlen:
            self.pop(0)

    def extend(self, iterable):
        super().extend(iterable)
        while len(self) > self.maxlen:
            self.pop(0)


class SLAMClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("slam", ip, port)

        self.dxy = 0
        self.dtheta = 0
        self.dt = 0

        self.scan_queue = mp.Queue(get_config("vgslam.miniros.scan_queue_len"))
        self.result_queue = mp.Queue()

        self.last_pos: bytes | None = None
        self.last_map: bytes | None = None

        self.running = asyncio.Event()
        self.slam_proc = None
        
        self._odometry_event = asyncio.Event()
        self._odometry = FixedSizeList(10)
        
        self._pos_version = 0
        self._map_version = 0

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
                self._pos_version += 1
                
                if mmap_data is not None:
                    self.last_map = mmap_data
                    self._map_version += 1

            except Empty:
                continue

    @aparsedata(datatypes.Lidar2D)
    async def on_lidar_lidar(self, data):
        print("got lidar")
        
        if self.scan_queue.full():
            try:
                self.scan_queue.get_nowait()

            except Empty:
                pass

        try:
            mov = await self.wait_for_odometry_at(data.timestamp, max_retries=3)
            
            if mov is None:
                print("ts None")
                return

            self.scan_queue.put_nowait((
                mov.x,
                mov.y,
                mov.theta,
                data.distances,
                data.angles,
            ))

        except Exception:
            pass

    @aparsedata(TimedMovement3DoF)
    async def on_motorcontroller_odometry(self, data):
        self._odometry.append(data)
        self._odometry_event.set()
    
    async def wait_for_odometry_at(self, ts: float, max_retries: int = 2):        
            for i, odom in enumerate(self._odometry):
                if odom.timestamp >= ts:
                    if i > 0:
                        prev_odom = self._odometry[i - 1]
                        return self._interpolate(odom, prev_odom, ts)
    
                    else:
                        return self._odometry[0]
    
            self._odometry_event.clear()
    
            for _ in range(max_retries):
                await self._odometry_event.wait()
                self._odometry_event.clear()
    
                if self._odometry[-1].timestamp >= ts:
                    return self._interpolate(
                        self._odometry[-1], self._odometry[-2], ts
                    )
    
            return None

    def _normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi

        while angle < -np.pi:
            angle += 2 * np.pi

        return angle
    
    def _interpolate(self, odom, prev_odom, timestamp):
        mov = odom.movement
        prev_mov = prev_odom.movement

        alpha = (timestamp - prev_odom.timestamp) / (odom.timestamp - prev_odom.timestamp)
        x = prev_mov.x + alpha * (mov.x - prev_mov.x)
        y = prev_mov.y + alpha * (mov.y - prev_mov.y)
        theta = prev_mov.theta + alpha * (mov.theta - prev_mov.theta)

        return Movement3DoF(
            x=x,
            y=y,
            theta=self._normalize_angle(theta),
        )

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

        last_seen_map = -1
        last_seen_pos = -1
        while client.running.is_set():
            if client.last_map is not None and last_seen_map != client._map_version:
                last_seen_map = client._map_version
                await map_topic.post(client.last_map)

            if last_seen_pos != client._pos_version:
                last_seen_pos = client._pos_version
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
