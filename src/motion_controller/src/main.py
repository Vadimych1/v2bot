from miniros_algorithms.source.pathtracking import CurrentPathTracker as CPTAlgo
from miniros_algorithms.source.pathfinding import LocalPathPlanner as LPPAlgo
from miniros.util.datatypes import Vector, NumpyArray, LidarDatatype
from miniros_algorithms.source.pathfinding import (
    find_obstacles,
)  # TODO: make sure it works fine
from miniros_constants.main import PX_PER_METER, METER_PER_PX
from miniros_algorithms.source.pathtracking import PID
from miniros_slam.source.datatypes import SLAMMap
from miniros.util.decorators import aparsedata
from miniros import AsyncROSClient
from time import time
import numpy as np
import asyncio


class MotionController(AsyncROSClient):
    """
    Motion controller node for vbot

    subs to:
    - slam/map
    - slam/pose
    - lidar/lidar
    - pathplanner/globalpath

    posts:
    - motioncontroller/cmdvel

    Updates local path every 0.18 seconds (5.56hz) with chosen LocalPathPlanner (LPP) Algorithm

    Sends commands to motioncontroller/cmdvel based on local path planner output with chosen PathTracker (CPT) Algorithm every 0.1 seconds (10hz)

    Predicts current position based on speed integration and autocorrects
    """

    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("motioncontroller", ip, port, _parse_handlers)

        self.map = None
        self.projected_map = None
        self.pose = None
        self.predicted_pose = None
        self.current_goal = Vector(0, 0, 0)

        self.path_tracker = CPTAlgo(max_linear_speed=3.0, max_angular_speed=np.pi / 4.0)
        self.path_planner = LPPAlgo()
        self.planned_path = None

        self.last_pose_update = time()

    @aparsedata(SLAMMap)
    async def on_slam_map(self, map: SLAMMap):
        # decode map and copy
        # it for projection
        self.map = map.to_numpy(int(len(map.data) ** 0.5))  # map is a square
        self.projected_map = self.map

    @aparsedata(Vector)
    async def on_slam_pose(self, pose: Vector):
        # update pose and set last
        # update time
        self.pose = pose.pos_to_numpy()
        self.predicted_pose = np.copy(self.pose)
        self.last_pose_update = time()

    @aparsedata(LidarDatatype)
    async def on_lidar_lidar(self, scan: LidarDatatype):
        await self.anon("lidar", "ping", b"hi")

        # project lidar scan on map copy
        # based on predicted pose for
        # better local obstacle avoidance
        angles, distances = (
            np.deg2rad(scan.angles) + self.pose[2],
            scan.distances / 1000.0,
        )

        # copy current map
        self.projected_map = self.map.copy()

        xi, yi = (
            ((self.pose[0] + distances * np.cos(angles)) * PX_PER_METER).astype(int),
            ((self.pose[1] + distances * np.sin(angles)) * PX_PER_METER).astype(int),
        )

        h, w = self.map.shape
        mask = (xi >= 0) & (xi < w) & (yi >= 0) & (yi < h)

        xi = xi[mask]
        yi = yi[mask]

        radius = 2
        for x, y in zip(xi, yi):
            x_start = max(0, x - radius)
            x_end = min(w, x + radius + 1)
            y_start = max(0, y - radius)
            y_end = min(h, y + radius + 1)

            self.projected_map[y_start:y_end, x_start:x_end] = 255

    @aparsedata(NumpyArray)
    async def on_pathplanner_globalpath(self, path: np.ndarray):
        # set current global path
        self.planned_path = path


async def main():
    client = MotionController()
    local_path = np.array([])

    async def run_path_tracker():
        await client.wait()

        cmdvel_topic = await client.topic("cmdvel", Vector)

        upd_time = time()
        while True:
            await asyncio.sleep(0.1)

            # if last update was too much time ago
            # we wouldnt send anything to motorcontroller
            # so it will stop motors automatically after
            # some amount of seconds* but when we`re not
            # moving at all the PID`s I component still
            # intergating and we need to reset it
            #
            # *(see src/motor_controller/main.py)
            t = time()
            if t - client.last_pose_update > 1.0:
                if type(client.path_tracker) == PID:
                    client.path_tracker.i_angular = 0

                continue

            # calculate speeds and send them
            dt, upd_time = upd_time - t, t

            client.path_tracker.set_path(local_path)
            v, w = client.path_tracker.get_speeds(client.predicted_pose)

            await cmdvel_topic.post(Vector(v, w, 0))

            # integrate speeds for position
            (
                _,
                _,
                theta,
            ) = client.predicted_pose

            # TODO:
            # use optical flow to get current speed
            # or create learning algorithm that will
            # calculate difference between predicted
            # position and slam-detected position and
            # then correct speed based on this value
            dx = v * np.cos(theta) * dt
            dy = v * np.sin(theta) * dt
            dtheta = w * dt

            client.predicted_pose += np.array([dx, dy, dtheta])

    async def run_localpath_updator():
        nonlocal local_path
        await client.wait()

        while True:
            await asyncio.sleep(0.18)

            if (
                client.map != None
                and client.predicted_pose != None
                and client.planned_path != None
            ):
                # update current local path
                client.path_planner.pos = client.predicted_pose[:2] * PX_PER_METER
                client.path_planner.obstacles = find_obstacles(client.projected_map)
                local_path = (
                    np.array(client.path_planner.update_path(client.planned_path))
                    * METER_PER_PX  # convert to real coordinates
                )

    await asyncio.gather(client.run(), run_path_tracker(), run_localpath_updator())


asyncio.run(main())
