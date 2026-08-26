import asyncio
from queue import Empty
import multiprocessing as mp
import time

from rtabmap_py import RtabmapSLAM

from miniros_configurator import get_config
from miniros import AsyncROSClient, datatypes
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.datatypes import Movement, Vector


def slam_worker(input_queue: mp.Queue, output_queue: mp.Queue):
    map_resolution = get_config("rtab-slam.mapping.resolution")

    # float lidarMinRange, float lidarMaxRange, float resolution
    mapper = RtabmapSLAM(
        get_config("rtab-slam.lidar.min_distance"),
        get_config("rtab-slam.lidar.max_distance"),
        map_resolution,
    )

    map_counter = 0
    map_range_threshold = get_config("slam.mapping.range_threshold")

    while True:
        try:
            scan = input_queue.get(timeout=1)

            # shutdown signal
            if scan is None:
                break

            scan = datatypes.Lidar2D.decode(scan)

        except Empty:
            continue

        pose = scan.movement
        ranges, angles = scan.distances, scan.angles

        # py::arg("distances"),
        # py::arg("angles"),
        # py::arg("odomX"),
        # py::arg("odomY"),
        # py::arg("odomTheta"),
        # py::arg("timestamp")
        new_x, new_y, new_theta = mapper.process(
            ranges,
            angles,
            pose.x,
            pose.y,
            pose.theta,
            scan.timestamp,  # TODO: replace with LiDAR actual timestamp
        )

        movement_msg = datatypes.TimedMovement3DoF(
            timestamp=scan.timestamp,
            movement=datatypes.Movement3DoF(
                x=new_x,
                y=new_y,
                theta=new_theta,
            ),
        )
        mmap_data = None

        if map_counter % 3 == 0:
            x_min, y_min, grid = mapper.getOccupancyGrid()
            mmap_data = SLAMOffsetMap(
                grid=grid,
                width=grid.shape[1],
                height=grid.shape[0],
                offset_x=-int(x_min / map_resolution),
                offset_y=-int(y_min / map_resolution),
                resolution=map_resolution,
            )

            mmap_data = SLAMOffsetMap.encode(mmap_data)

        map_counter += 1
        output_queue.put((Movement.encode(movement_msg), mmap_data))


class SLAMClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("slam", ip, port)

        self.dxy = 0
        self.dtheta = 0
        self.dt = 0

        self.scan_queue = mp.Queue(get_config("rtab-slam.miniros.scan_queue_len"))
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

    map_post_delay = get_config("slam.miniros.map_post_delay")
    pose_post_delay = get_config("slam.miniros.pose_post_delay")

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
