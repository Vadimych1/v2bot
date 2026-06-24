from miniros.util.datatypes import Movement, NumpyArray, Vector
from miniros_algorithms.source.pathtracking import PID
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.decorators import aparsedata
from miniros import AsyncROSClient
from time import time
import numpy as np
import asyncio


class MotionControllerConfig:
    def __init__(self, max_linear_speed: float = 1.0, min_linear_speed: float = -1.0,
            max_angular_speed: float = 1.0, max_linear_accel: float = 0.5,
            max_angular_accel: float = 1.0, dt: float = 0.1, predict_time: float = 2.0,
            num_samples: int = 20, robot_radius: float = 0.4, weight_heading: float = 0.5,
            weight_dist: float = 0.2, weight_speed: float = 0.4, weight_obstacle: float = 0.5,
            goal_tolerance: float = 0.2, lookahead_distance: float = 2.5):
        self.max_linear_speed = max_linear_speed
        self.min_linear_speed = min_linear_speed
        self.max_angular_speed = max_angular_speed
        
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel

        self.dt = dt
        self.predict_time = predict_time
        self.num_samples = num_samples
        self.robot_radius = robot_radius

        self.weight_heading = weight_heading
        self.weight_dist = weight_dist
        self.weight_speed = weight_speed
        self.weight_obstacle = weight_obstacle

        self.goal_tolerance = goal_tolerance
        self.lookahead_distance = lookahead_distance

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

    def __init__(self, config: MotionControllerConfig, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("motioncontroller", ip, port, _parse_handlers)

        self.config = config

        # grid map retrieved from slam
        self.grid = None
        
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

        # self.Kp = 1.2
        # self.Ki = 0.0
        # self.Kd = 0.1

        self._integral_error = 0.0
        self._prev_error = 0.0

    def _world_to_pixel(self, wx: float, wy: float):
        px = int(wx / self.resolution + self.offset_x)
        py = int(wy / self.resolution + self.offset_y)

        return px, py
    
    def _is_collision(self, x: float, y: float):
        if self.grid is None:
            return False
        
        radius_px = self.config.robot_radius / self.resolution
        px, py = self._world_to_pixel(x, y)
        h, w = self.height, self.width

        if px < 0 or px >= w or py < 0 or py >= h:
            return True
        
        step = max(1, int(radius_px / 2))
        for dx in range(-step, step + 1):
            for dy in range(-step, step + 1):
                if dx * dx + dy * dy <= radius_px * radius_px:
                    ix = px + dx
                    iy = py + dy
                    if ix < 0 or ix >= w or iy < 0 or iy >= h:
                        return True
                    
                    if self.grid[iy, ix] < 90:
                        return True
        
        return False
    
    def _find_closest_path_idx(self):
        if self.global_path is None:
            return 0
        
        min_dist = float('inf')
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
                frac = (self.config.lookahead_distance - cumulative_dist) / seg_dist if seg_dist > 0 else 0
                
                target_x = px + frac * (nx - px)
                target_y = py + frac * (ny - py)

                return target_x, target_y

            cumulative_dist += seg_dist
            idx += 1

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
    
    def _evaluate_trajectory(self, states: list[tuple[float, float, float]], v: float, w: float):
        if len(states) <= 0:
            return float('inf')
        
        for (x, y, _) in states:
            if self._is_collision(x, y):
                return float('inf')
            
        target_x, target_y = self._get_target_point()
        dx = target_x - states[-1][0]
        dy = target_y - states[-1][1]

        angle_to_target = np.atan2(dy, dx)
        heading_error = abs(self._normalize_angle(angle_to_target - states[-1][2]))
        heading_cost = self.config.weight_heading * heading_error

        dist_to_target = np.hypot(states[-1][0] - target_x, states[-1][1] - target_y)
        dist_cost = self.config.weight_dist * (dist_to_target / (self.config.lookahead_distance + 1.0))

        speed_cost = -abs(self.config.weight_speed * v)

        obstacle_cost = 0.0
        h, w = self.height, self.width
        count_radius = 6
        for (x, y, _) in states:
            px, py = self._world_to_pixel(x, y)
            
            x_min = max(0, px - count_radius)
            x_max = min(w, px + count_radius + 1)
            y_min = max(0, py - count_radius)
            y_max = min(h, py + count_radius + 1)

            window = self.grid[y_min:y_max, x_min:x_max]
            fill = np.sum(window == 0)

            obstacle_cost += fill

        # TODO: implement weight cost
        obstacle_cost = self.config.weight_obstacle * obstacle_cost

        total_cost = heading_cost + dist_cost + speed_cost + obstacle_cost

        return total_cost
    
    def _normalize_angle(self, angle: float):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        
        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle
    
    def compute_control(self):
        if self.grid is None or self.global_path is None:
            return 0.0, 0.0
        
        if self.target_idx < len(self.global_path) - 1:
            target_x, target_y = self.global_path[self.target_idx]

            if np.hypot(target_x - self.robot_x, target_y - self.robot_y) < self.config.goal_tolerance:
                self.target_idx += 1
                if self.target_idx >= len(self.global_path):
                    self.target_idx = len(self.global_path)
        
        v_min = max(self.config.min_linear_speed, self.current_v - self.config.max_linear_accel * self.config.dt)
        v_max = min(self.config.max_linear_speed, self.current_v + self.config.max_linear_accel * self.config.dt)

        w_min = max(-self.config.max_angular_speed, self.current_w - self.config.max_angular_accel * self.config.dt)
        w_max = min(self.config.max_angular_speed, self.current_w + self.config.max_angular_accel * self.config.dt)

        v_samples = np.linspace(v_min, v_max, self.config.num_samples)
        w_samples = np.linspace(w_min, w_max, self.config.num_samples)

        best_cost = float('inf')
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
        
        if best_cost == float('inf'):
            return 0.0, 0.0

        return best_v, best_w

    # PID-based approach

    # def _normalize_angle(self, angle: float):
    #     while angle > np.pi:
    #         angle -= 2.0 * np.pi

    #     while angle < -np.pi:
    #         angle += 2.0 * np.pi

    #     return angle
    
    # def _find_closest_point_idx(self):
    #     if self.global_path is None: return -1

    #     min_dist = float('inf')
    #     closest_idx = 0

    #     for i, (px, py) in enumerate(self.global_path):
    #         d = np.hypot(px - self.robot_x, py - self.robot_y)
    #         if d < min_dist:
    #             min_dist = d
    #             closest_idx = i
        
    #     return closest_idx
    
    # def compute_control(self):
    #     if self.global_path is None:
    #         return 0.0, 0.0

    #     if self.target_idx is None:
    #         self.target_idx = self._find_closest_point_idx()

    #     if self.target_idx >= len(self.global_path):
    #         return 0.0, 0.0
        
    #     target_x, target_y = self.global_path[self.target_idx]
    #     distance = np.hypot(target_x - self.robot_x, target_y - self.robot_y)

    #     if distance <= self.config.goal_tolerance:
    #         self.target_idx += 1

    #         self._integral_error = 0.0
    #         self._prev_error = 0.0

    #         if self.target_idx >= len(self.global_path):
    #             return 0.0, 0.0

    #         target_x, target_y = self.global_path[self.target_idx]
    #         distance = np.hypot(target_x - self.robot_x, target_y - self.robot_y)

    #     desired_angle = np.atan2(target_y - self.robot_y, target_x - self.robot_x)
    #     error = self._normalize_angle(desired_angle - self.robot_heading)

    #     self._integral_error += error
    #     self._integral_error = np.clip(self._integral_error, -1, 1)

    #     derivative = error - self._prev_error
    #     self._prev_error = error

    #     angular_speed = self.Kp * error + self.Ki * self._integral_error + self.Kd * derivative
    #     angular_speed = np.clip(angular_speed, -self.config.max_angular_speed, self.config.max_angular_speed)

    #     angle_factor = max(0.1, 1.0 - abs(error) / np.pi)
    #     dist_factor = min(1.0, max(0.6, distance / 2))

    #     linear_speed = self.config.max_linear_speed * angle_factor * dist_factor
    #     linear_speed = np.clip(linear_speed, 0, self.config.max_linear_speed)

    #     return linear_speed, angular_speed

    @aparsedata(SLAMOffsetMap)
    async def on_slam_map(self, map): # SLAMOffsetMap
        self.grid = map.grid
        self.width = map.width
        self.height = map.height
        self.offset_x = map.offset_x
        self.offset_y = map.offset_y
        self.resolution = map.resolution

    @aparsedata(Movement)
    async def on_slam_pose(self, pose: Movement):
        self.robot_x = pose.pos.x
        self.robot_y = pose.pos.y
        self.robot_heading = pose.ang.z

    @aparsedata(NumpyArray)
    async def on_pathplanner_globalpath(self, path: np.ndarray):
        self.global_path = path


async def main():
    client = MotionController(
        config=MotionControllerConfig(
            # TODO: add speeds from odometry and remove these lines
            # (set current_v and current_w)
            max_angular_accel=10000,
            max_linear_accel=10000
        )
    )

    async def run_path_tracker():
        await client.wait()

        cmdvel_topic = await client.topic("cmdvel", Vector)
        prev_v, prev_w = 0, 0

        while True:
            await asyncio.sleep(0.1)
            v, w = client.compute_control()

            if prev_v != v or prev_w != w:
                await cmdvel_topic.post(Vector(v, w, 0))
                prev_v = v
                prev_w = w

    await asyncio.gather(client.run(), run_path_tracker())


asyncio.run(main())
