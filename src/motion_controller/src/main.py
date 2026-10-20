from scipy.ndimage import distance_transform_edt
import numpy as np
import math
import time

import asyncio
from miniros import AsyncROSClient
from miniros.util.datatypes import NumpyArray, Vector, TimedMovement3DoF
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.decorators import aparsedata
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
            "motion_controller.kinematics.max_angular_speed"
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
        self.weight_progress = get_config("motion_controller.weights.progress")

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

        self._map_snapshot = {
            "grid": None,
            "width": 0,
            "height": 0,
            "offset_x": 0,
            "offset_y": 0,
            "resolution": 1,
            "obstacle_distance_map": None,
        }
        
        # current pose, meters and radians
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        
        self.last_slam_pose = None
        self.last_odom_pose = None
        self.saved_odom_pose = None

        # linear and angular speeds
        self.current_v = 0.0
        self.current_w = 0.0

        # global path from path_planner
        self.global_path = None
        self.target_idx = 0

        self._obstacle_upper_threshold = get_config("obstacle_upper_threshold")

        self._last_map_ts = -1
        self._last_odom_ts = -1
        
        self._last_cmd_time = time.monotonic()
        
        self._edt_busy = False

    def _world_to_pixel(self, wx: float, wy: float, resolution, of_x, of_y):
        px = math.floor(wx / resolution + of_x)
        py = math.floor(wy / resolution + of_y)

        return px, py

    def _is_collision(self, x: float, y: float, obstacle_distance_map, resolution, of_x, of_y, width, height):
        if obstacle_distance_map is None:
            return False

        clearance = self._get_obstacle_distance(x, y, resolution, of_x, of_y, width, height, obstacle_distance_map)
        return clearance <= self.config.robot_radius

    def _find_closest_path_idx(self, path, robot_x, robot_y):
        if path is None:
            return 0

        min_dist = float("inf")
        idx = 0
        for i, (px, py) in enumerate(path):
            d = math.hypot(px - robot_x, py - robot_y)
            if d < min_dist:
                min_dist = d
                idx = i

        return idx

    def _get_target_point(self, path, target_idx, x, y):
        if path is None or len(path) == 0:
            return x, y

        cumulative_dist = 0.0
        idx = target_idx
        while idx < len(path) - 1:
            px, py = path[idx]
            nx, ny = path[idx + 1]

            seg_dist = math.hypot(nx - px, ny - py)

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
            
        return tuple(path[-1])

    def _simulate_trajectory(self, v: float, w: float, x, y, theta):
        states = []
        steps = max(1, round(self.config.predict_time / self.config.dt))
        for _ in range(steps):
            x += v * math.cos(theta) * self.config.dt
            y += v * math.sin(theta) * self.config.dt
            theta += w * self.config.dt

            states.append((x, y, theta))

        return states

    def _get_obstacle_distance(self, x: float, y: float, resolution, of_x, of_y, width, height, obstacle_distance_map):
        if obstacle_distance_map is None:
            return 0.0

        px, py = self._world_to_pixel(x, y, resolution, of_x, of_y)

        # Outside the map is considered a collision.
        if px < 0 or px >= width or py < 0 or py >= height:
            return 0.0

        return obstacle_distance_map[py, px]

    def _evaluate_trajectory(
        self, 
        states: list[tuple[float, float, float]], 
        target, 
        v: float, 
        obstacle_distance_map, 
        resolution, 
        of_x, 
        of_y, 
        width, 
        height,
        robot_x,
        robot_y
    ):
        if (
            len(states) <= 0
            or obstacle_distance_map is None
        ):
            return float("inf")

        for x, y, _ in states:
            if self._is_collision(x, y, obstacle_distance_map, resolution, of_x, of_y, width, height):
                return float("inf")

        target_x, target_y = target
        dx = target_x - states[-1][0]
        dy = target_y - states[-1][1]

        ### Heading cost
        angle_to_target = math.atan2(dy, dx)
        
        if v >= 0:
            movement_heading = states[-1][2]
        
        else:
            movement_heading = states[-1][2] + np.pi
        
        heading_error = abs(self._normalize_angle(angle_to_target - movement_heading))
        heading_cost = self.config.weight_heading * heading_error

        ### Progress reward
        start_dist = math.hypot(robot_x - target_x, robot_y - target_y)
        end_dist = math.hypot(states[-1][0] - target_x, states[-1][1] - target_y)
        
        progress = start_dist - end_dist
        progress_cost = -self.config.weight_progress * progress
        
        ### Distance cost
        dist_to_target = math.hypot(states[-1][0] - target_x, states[-1][1] - target_y)
        dist_cost = self.config.weight_dist * (
            dist_to_target / (self.config.lookahead_distance + 1.0)
        )

        ### Speed cost
        speed_cost = -self.config.weight_speed * (v if v >= 0 else -v * 0.9)

        if 0 < v < 0.1:
            t = abs(v) / 0.1
            speed_cost += (1.0 - t) * (1.0 - t) * 0.1

        # obstacle cost
        influence_distance = (
            self.config.robot_radius + 0.05
        )  # TODO: get rid of magic value

        obstacle_cost = 0.0
        
        ### Stationary penalty
        if abs(v) < 1e-4:
            stationary_penalty = 0.5
        else:
            stationary_penalty = 0.0

        for i, (x, y, _) in enumerate(states):
            if i % 2 == 1:
                continue

            clearance = self._get_obstacle_distance(x, y, resolution, of_x, of_y, width, height, obstacle_distance_map)
            if clearance > influence_distance:
                continue

            normalized = (influence_distance - clearance) / influence_distance
            local_cost = normalized * normalized
            temporal_weight = (i + 1) / len(states)

            obstacle_cost += local_cost * temporal_weight

        obstacle_cost *= self.config.weight_obstacle

        return heading_cost + dist_cost + speed_cost + obstacle_cost + progress_cost + stationary_penalty

    def _normalize_angle(self, angle: float):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def compute_control(self):
        snap = self._map_snapshot
        
        grid = snap["grid"]
        obstacle_distance_map = snap["obstacle_distance_map"]
        width = snap["width"]
        height = snap["height"]
        resolution = snap["resolution"]
        of_x = snap["offset_x"]
        of_y = snap["offset_y"]
        
        path = self.global_path
        
        x = self.robot_x
        y = self.robot_y
        theta = self.robot_heading
        v = self.current_v
        w = self.current_w

        if grid is None or path is None or len(path) == 0:
            return 0.0, 0.0

        # if we are close to the target
        last_x, last_y = path[-1]
        if math.hypot(x - last_x, y - last_y) < self.config.goal_tolerance:
            return 0.0, 0.0

        if self.target_idx < len(path) - 1:
            closest_idx = self._find_closest_path_idx(path, x, y)

            if closest_idx > self.target_idx:
                self.target_idx = closest_idx

            target_x, target_y = path[self.target_idx]

            if math.hypot(target_x - x, target_y - y) < self.config.goal_tolerance:
                self.target_idx += 1
                if self.target_idx >= len(path):
                    self.target_idx = len(path) - 1

        now = time.monotonic()
        control_dt = min(0.5, max(1e-3, now - self._last_cmd_time))
        self._last_cmd_time = now
        
        v_min = max(
            self.config.min_linear_speed,
            v - self.config.max_linear_accel * control_dt,
        )
        v_max = min(
            self.config.max_linear_speed,
            v + self.config.max_linear_accel * control_dt,
        )

        w_min = max(
            -self.config.max_angular_speed,
            w - self.config.max_angular_accel * control_dt,
        )
        w_max = min(
            self.config.max_angular_speed,
            w + self.config.max_angular_accel * control_dt,
        )

        v_samples = np.linspace(v_min, v_max, self.config.num_v_samples)
        w_samples = np.linspace(w_min, w_max, self.config.num_w_samples)

        best_cost = float("inf")
        best_v = 0.0
        best_w = 0.0
        # best_trajectory = None

        target = self._get_target_point(path, self.target_idx, x, y)
        for v in v_samples:
            for w in w_samples:
                states = self._simulate_trajectory(v, w, x, y, theta)
                cost = self._evaluate_trajectory(states, target, v, obstacle_distance_map, resolution, of_x, of_y, width, height, x, y)

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
        if self._edt_busy:
            return
        
        self._edt_busy = True
        
        try:
            obstacles = map.grid < self._obstacle_upper_threshold
            euc = await asyncio.to_thread(distance_transform_edt, ~obstacles)            
            euc *= map.resolution

            # single atomic swap
            self._map_snapshot = {
                "grid": map.grid,
                "width": map.width,
                "height": map.height,
                "offset_x": map.offset_x,
                "offset_y": map.offset_y,
                "resolution": map.resolution,
                "obstacle_distance_map": euc,
            }
            self._last_map_ts = time.time()
        
        finally:
            self._edt_busy = False

    @aparsedata(TimedMovement3DoF)
    async def on_slam_pose(self, pose):
        mov = pose.movement
        
        self.robot_x = mov.x
        self.robot_y = mov.y
        self.robot_heading = mov.theta
        
        self.last_slam_pose = mov
        self.saved_odom_pose = self.last_odom_pose
        
        # self._last_odom_ts = pose.timestamp

    @aparsedata(TimedMovement3DoF)
    async def on_motorcontroller_odometry(self, odom):
        self._last_odom_ts = odom.timestamp
        self.last_odom_pose = odom.movement
        
        if self.last_slam_pose is not None and self.saved_odom_pose != None:
            anchor_slam = self.last_slam_pose
            anchor_odom = self.saved_odom_pose
            cur_odom = self.last_odom_pose
            
            self.robot_x = anchor_slam.x + (cur_odom.x - anchor_odom.x)
            self.robot_y = anchor_slam.y + (cur_odom.y - anchor_odom.y)
            self.robot_heading = anchor_slam.theta + self._normalize_angle(cur_odom.theta - anchor_odom.theta)

    @aparsedata(Vector)
    async def on_motorcontroller_velocity(self, velocity: Vector):
        self.current_v = velocity.x
        self.current_w = velocity.y

    @aparsedata(NumpyArray)
    async def on_pathplanner_globalpath(self, path: np.ndarray):
        self.target_idx = 0
        self.global_path = path


async def main():
    client = MotionController(config=MotionControllerConfig())
    cmdvel_post_delay = get_config("motion_controller.miniros.cmdvel_post_delay")

    async def run_path_tracker():
        await client.wait()

        cmdvel_topic = await client.topic("cmdvel", Vector)
        map_max_age = get_config("motion_controller.map_max_age")
        odom_max_age = get_config("motion_controller.odometry_max_age")

        loop = asyncio.get_running_loop()
        next_tick = loop.time()
        while True:
            next_tick += cmdvel_post_delay
            delay = next_tick - loop.time()
            
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                next_tick = loop.time()
            
            ts = time.time()
            if ts - client._last_odom_ts >= odom_max_age or ts - client._last_map_ts >= map_max_age:
                await cmdvel_topic.post(Vector(0, 0, 0))
                continue

            try:
                v, w = await asyncio.to_thread(client.compute_control)
            except Exception as e:
                print(f"compute_control failed: {e}", flush=True)
                v, w = 0.0, 0.0
            
            await cmdvel_topic.post(Vector(v, w, 0)) # -w cause sits somewhere in the code; fix

    path_track = asyncio.create_task(run_path_tracker())

    await asyncio.gather(client.run(), path_track)


asyncio.run(main())
