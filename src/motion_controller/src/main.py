from miniros.util.datatypes import Movement, NumpyArray, Vector
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.decorators import aparsedata
from miniros import AsyncROSClient
import numpy as np
import asyncio


class MotionControllerConfig:
    def __init__(
        self,
        max_linear_speed: float = 0.125,
        min_linear_speed: float = -0.125,
        max_angular_speed: float = 1.5,
        max_linear_accel: float = 1.5,
        max_angular_accel: float = 2.5,
        dt: float = 0.1,
        predict_time: float = 2.0,
        num_samples: int = 20,
        robot_radius: float = 0.22,
        weight_heading: float = 0.40,
        weight_dist: float = 0.4,
        weight_speed: float = 0.3,
        weight_obstacle: float = 0.55,
        goal_tolerance: float = 0.2,
        lookahead_distance: float = 2,
    ):
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
                
                print(f"Target: {target_x, target_y}")

                return target_x, target_y

            cumulative_dist += seg_dist
            idx += 1

        try:
            return self.global_path[-1]
        
        except:
            return (self.robot_x, self.robot_y)

        
    def _simulate_trajectory(self, v: float, w: float):
        states = []
        x, y, theta = self.robot_x, self.robot_y, self.robot_heading

        for _ in range(int(self.config.predict_time / self.config.dt)):
            x += v * np.cos(theta) * self.config.dt
            y += v * np.sin(theta) * self.config.dt
            theta += w * self.config.dt

            states.append((x, y, theta))

        return states

    def _evaluate_trajectory(
        self, states: list[tuple[float, float, float]], v: float, w: float
    ):
        if self.global_path is None or len(states) <= 0 or self.grid is None:
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

        speed_cost = -abs(self.config.weight_speed * v)

        obstacle_cost = 0.0
        h, w = self.height, self.width
        count_radius = 6
        for x, y, _ in states:
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
        if self.grid is None or self.global_path is None or len(self.global_path) == 0:
            return 0.0, 0.0

        # if we are close to the target
        last_x, last_y = self.global_path[-1]
        if (
            np.hypot(self.robot_x - last_x, self.robot_y - last_y)
            < self.config.goal_tolerance
        ):
            return 0.0, 0.0

        v_min = max(
            self.config.min_linear_speed,
            self.current_v - self.config.max_linear_accel * self.config.dt,
        )
        v_max = min(
            self.config.max_linear_speed,
            self.current_v + self.config.max_linear_accel * self.config.dt,
        )

        w_min = max(
            -self.config.max_angular_speed,
            self.current_w - self.config.max_angular_accel * self.config.dt,
        )
        w_max = min(
            self.config.max_angular_speed,
            self.current_w + self.config.max_angular_accel * self.config.dt,
        )

        v_samples = np.linspace(v_min, v_max, self.config.num_samples)
        w_samples = np.linspace(w_min, w_max, self.config.num_samples)

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

    async def run_path_tracker():
        await client.wait()

        cmdvel_topic = await client.topic("cmdvel", Vector)
        # prev_v, prev_w = 0, 0

        while True:
            await asyncio.sleep(0.1)
            v, w = await asyncio.to_thread(client.compute_control)

            # if prev_v != v or prev_w != w:
            
            await cmdvel_topic.post(Vector(v, -w, 0))
            
            # prev_v = v
            # prev_w = w
                
    path_track = asyncio.create_task(run_path_tracker())

    await asyncio.gather(client.run(), path_track)


asyncio.run(main())
