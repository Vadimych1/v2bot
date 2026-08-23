from scipy.ndimage import distance_transform_edt
import numpy as np
import math

import asyncio

from miniros.util.datatypes import NumpyArray, Vector
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.decorators import aparsedata
from miniros import AsyncROSClient
from miniros_configurator import get_config


class MotionControllerConfig:
    def __init__(self):
        self.max_linear_speed = get_config(
            "motion_controller.kinematics.max_linear_speed"
        )
        self.min_linear_speed = get_config(
            "motion_controller.kinematics.min_linear_speed"
        )
        self.max_angular_speed = get_config(
            "motion_controller.kinematics.angular_speed"
        )
        self.max_linear_accel = get_config(
            "motion_controller.kinematics.max_linear_accel"
        )
        self.max_angular_accel = get_config(
            "motion_controller.kinematics.max_angular_accel"
        )

        self.dt = get_config("motion_controller.time.delta")
        self.predict_time = get_config("motion_controller.time.prediction")

        self.num_v_samples = get_config("motion_controller.sampling.v_samples")
        self.num_w_samples = get_config("motion_controller.sampling.omega_samples")

        self.robot_radius = get_config("robot_radius")

        self.weight_heading = get_config("motion_controller.weights.heading")
        self.weight_dist = get_config("motion_controller.weights.distance")
        self.weight_speed = get_config("motion_controller.weights.speed")
        self.weight_obstacle = get_config("motion_controller.weights.obstacle")

        self.goal_tolerance = get_config("motion_controller.goal_tolerance")
        self.lookahead_distance = get_config("motion_controller.lookahead_distance")


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

    Predicts current position based on speed integration and autocorrects
    """

    def __init__(
        self,
        config: MotionControllerConfig,
        ip="localhost",
        port=3000,
        _parse_handlers=True,
    ):
        super().__init__("motioncontroller", ip, port, _parse_handlers)

        self.config = config

        # grid map retrieved from slam
        self.grid = None
        self.obstacle_distance_map = None

        # map size in pixels
        self.width = 0
        self.height = 0

        # (0, 0) offset on map
        # in pixels
        self.offset_x = 0
        self.offset_y = 0

        # map resolution (meters per pixel)
        self.resolution = 1

        # current pose, meters and radians
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0

        # linear and angular speeds
        self.current_v = 0.0
        self.current_w = 0.0

        # global path from path_planner
        self.global_path = None
        self.target_idx = 0

        self._obstacle_upper_threshold = get_config("obstacle_upper_threshold")

    def _world_to_pixel(self, wx: float, wy: float):
        px = math.floor(wx / self.resolution + self.offset_x)
        py = math.floor(wy / self.resolution + self.offset_y)

        return px, py

    def _is_collision(self, x: float, y: float):
        if self.obstacle_distance_map is None:
            return False

        clearance = self._get_obstacle_distance(x, y)
        return clearance <= self.config.robot_radius

    def _find_closest_path_idx(self):
        if self.global_path is None:
            return 0

        min_dist = float("inf")
        idx = 0
        for i, (px, py) in enumerate(self.global_path):
            d = np.hypot(px - self.robot_x, py - self.robot_y)
            if d < min_dist:
                min_dist = d
                idx = i

        return idx

    def _get_target_point(self):
        if self.global_path is None:
            return (self.robot_x, self.robot_y)

        if self.target_idx >= len(self.global_path):
            self.target_idx = len(self.global_path) - 1

        cumulative_dist = 0.0
        idx = self.target_idx
        while idx < len(self.global_path) - 1:
            px, py = self.global_path[idx]
            nx, ny = self.global_path[idx + 1]

            seg_dist = np.hypot(nx - px, ny - py)

            if cumulative_dist + seg_dist >= self.config.lookahead_distance:
                frac = (
                    (self.config.lookahead_distance - cumulative_dist) / seg_dist
                    if seg_dist > 0
                    else 0
                )

                target_x = px + frac * (nx - px)
                target_y = py + frac * (ny - py)

                return target_x, target_y

            cumulative_dist += seg_dist
            idx += 1

        if self.global_path is None or len(self.global_path) == 0:
            return (self.robot_x, self.robot_y)

        return self.global_path[-1]

    def _simulate_trajectory(self, v: float, w: float):
        states = []
        x, y, theta = self.robot_x, self.robot_y, self.robot_heading

        for _ in range(int(self.config.predict_time / self.config.dt)):
            x += v * np.cos(theta) * self.config.dt
            y += v * np.sin(theta) * self.config.dt
            theta += w * self.config.dt

            states.append((x, y, theta))

        return states

    def _get_obstacle_distance(self, x: float, y: float):
        if self.obstacle_distance_map is None:
            return 0.0

        px, py = self._world_to_pixel(x, y)

        # Outside the map is considered a collision.
        if px < 0 or px >= self.width or py < 0 or py >= self.height:
            return 0.0

        return self.obstacle_distance_map[py, px]

    def _evaluate_trajectory(
        self, states: list[tuple[float, float, float]], v: float, w: float
    ):
        if (
            self.global_path is None
            or len(states) <= 0
            or self.grid is None
            or self.obstacle_distance_map is None
        ):
            return float("inf")

        for x, y, _ in states:
            if self._is_collision(x, y):
                return float("inf")

        target_x, target_y = self._get_target_point()
        dx = target_x - states[-1][0]
        dy = target_y - states[-1][1]

        angle_to_target = np.atan2(dy, dx)
        heading_error = abs(self._normalize_angle(angle_to_target - states[-1][2]))
        heading_cost = self.config.weight_heading * heading_error

        dist_to_target = np.hypot(states[-1][0] - target_x, states[-1][1] - target_y)
        dist_cost = self.config.weight_dist * (
            dist_to_target / (self.config.lookahead_distance + 1.0)
        )

        # reward speed, moving backwards is allowed too
        speed_cost = -self.config.weight_speed * (v if v >= 0 else -v * 0.8)

        if 0 < v < 0.08:
            speed_cost += 0.08 / abs(v)

        # obstacle cost
        safety_margin = self.config.robot_radius
        influence_distance = (
            self.config.robot_radius + 0.05
        )  # TODO: get rid of magic value

        obstacle_cost = 0.0

        for i, (x, y, _) in enumerate(states):
            if i % 2 == 1:
                continue

            clearance = self._get_obstacle_distance(x, y)

            if clearance < safety_margin:
                obstacle_cost = float("inf")
                break

            if clearance > influence_distance:
                continue

            normalized = (influence_distance - clearance) / influence_distance
            local_cost = normalized * normalized
            temporal_weight = (i + 1) / len(states)

            obstacle_cost += local_cost * temporal_weight

        obstacle_cost *= self.config.weight_obstacle

        total_cost = heading_cost + dist_cost + speed_cost + obstacle_cost

        return total_cost

    def _normalize_angle(self, angle: float):
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def compute_control(self):
        grid = self.grid
        path = self.global_path
        x = self.robot_x
        y = self.robot_y
        v = self.current_v
        w = self.current_w

        if grid is None or path is None or len(path) == 0:
            return 0.0, 0.0

        # if we are close to the target
        last_x, last_y = path[-1]
        if np.hypot(x - last_x, y - last_y) < self.config.goal_tolerance:
            return 0.0, 0.0

        if self.target_idx < len(path) - 1:
            closest_idx = self._find_closest_path_idx()

            if closest_idx > self.target_idx:
                self.target_idx = closest_idx

            target_x, target_y = path[self.target_idx]

            if np.hypot(target_x - x, target_y - y) < self.config.goal_tolerance:
                self.target_idx += 1
                if self.target_idx >= len(path):
                    self.target_idx = len(path)

        v_min = max(
            self.config.min_linear_speed,
            v - self.config.max_linear_accel * self.config.dt,
        )
        v_max = min(
            self.config.max_linear_speed,
            v + self.config.max_linear_accel * self.config.dt,
        )

        w_min = max(
            -self.config.max_angular_speed,
            w - self.config.max_angular_accel * self.config.dt,
        )
        w_max = min(
            self.config.max_angular_speed,
            w + self.config.max_angular_accel * self.config.dt,
        )

        v_samples = np.linspace(v_min, v_max, self.config.num_v_samples)
        w_samples = np.linspace(w_min, w_max, self.config.num_w_samples)

        best_cost = float("inf")
        best_v = 0.0
        best_w = 0.0
        # best_trajectory = None

        for v in v_samples:
            for w in w_samples:
                states = self._simulate_trajectory(v, w)
                cost = self._evaluate_trajectory(states, v, w)

                if cost < best_cost:
                    best_cost = cost
                    best_v = v
                    best_w = w
                    # best_trajectory = states

        if best_cost == float("inf"):
            return 0.0, 0.0

        return best_v, best_w

    @aparsedata(SLAMOffsetMap)
    async def on_slam_map(self, map):  # SLAMOffsetMap
        self.grid = map.grid
        self.width = map.width
        self.height = map.height
        self.offset_x = map.offset_x
        self.offset_y = map.offset_y
        self.resolution = map.resolution

        # apply euclidian distance transform
        obstacles = self.grid < self._obstacle_upper_threshold
        euc = distance_transform_edt(~obstacles)

        if euc is None:
            return

        self.obstacle_distance_map = euc
        self.obstacle_distance_map *= self.resolution  # convert px to meters

    # @aparsedata(Movement)
    # async def on_slam_pose(self, pose: Movement):
    #     self.robot_x = pose.pos.x
    #     self.robot_y = pose.pos.y
    #     self.robot_heading = pose.ang.z

    @aparsedata(Vector)
    async def on_motorcontroller_odometry(self, odom: Vector):
        self.robot_x, self.robot_y, self.robot_heading = odom.x, odom.y, odom.z

    @aparsedata(NumpyArray)
    async def on_pathplanner_globalpath(self, path: np.ndarray):
        self.global_path = path
        self.target_idx = 0

    @aparsedata(Vector)
    async def on_motorcontroller_velocity(self, velocity: Vector):
        self.current_v = velocity.x
        self.current_w = velocity.y


async def main():
    client = MotionController(config=MotionControllerConfig())
    cmdvel_post_delay = get_config("motion_controller.miniros.cmdvel_post_delay")

    async def run_path_tracker():
        await client.wait()

        cmdvel_topic = await client.topic("cmdvel", Vector)
        # prev_v, prev_w = 0, 0

        while True:
            await asyncio.sleep(cmdvel_post_delay)

            v, w = await asyncio.to_thread(client.compute_control)
            await cmdvel_topic.post(Vector(v, -w, 0))

    path_track = asyncio.create_task(run_path_tracker())

    await asyncio.gather(client.run(), path_track)


asyncio.run(main())
