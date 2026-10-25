from scipy.ndimage import distance_transform_edt, gaussian_filter
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
        self.weight_path = get_config("motion_controller.weights.path")
        self.obstacle_side_factor = get_config("motion_controller.weights.obstacle_side_factor")
        self.min_viable_speed = get_config("motion_controller.kinematics.min_viable_speed")

        self.goal_tolerance = get_config("motion_controller.goal_tolerance")
        self.lookahead_distance = get_config("motion_controller.lookahead_distance")
        
        self.inf_mul = get_config("motion_controller.influence_distance_multiplier")


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
            "grad_x": None,
            "grad_y": None,
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

    def _simulate_trajectory(self, v: float, w: float, x, y, theta):
        states = []
        dt = self.config.dt

        pred_time = self.config.predict_time
        
        steps = max(1, round(pred_time / dt))
        for _ in range(steps):
            if abs(w) > 1e-6:
                x_new = x + (v/w) * (math.sin(theta + w*dt) - math.sin(theta))
                y_new = y - (v/w) * (math.cos(theta + w*dt) - math.cos(theta))
            else:
                x_new = x + v * math.cos(theta) * dt
                y_new = y + v * math.sin(theta) * dt
            x, y = x_new, y_new
            theta += w * dt

            states.append((x, y, theta))

        return states

    def _get_obstacle_distance(self, x: float, y: float, resolution, of_x, of_y, width, height, obstacle_distance_map):
        if obstacle_distance_map is None:
            return 0.0

        px, py = self._world_to_pixel(x, y, resolution, of_x, of_y)

        # outside the map is considered a collision
        if px < 0 or px >= width or py < 0 or py >= height:
            return 0.0

        return obstacle_distance_map[py, px]
    
    def _path_deviation(self, x: float, y: float, path, seg_i, max_lookahead: int = 20) -> float:
        if path is None or len(path) < 2:
            return 0.0
        
        lo = max(0, seg_i - 5)
        hi = min(len(path) - 1, seg_i + max_lookahead)
        
        # return distance to the end
        if hi <= lo:
            ax, ay = path[seg_i]
            return math.hypot(x - ax, y - ay)

        best = float('inf')
        for i in range(lo, hi):
            ax, ay = path[i]
            bx, by = path[i + 1]
            dx = bx - ax
            dy = by - ay
            
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-9:
                d = math.hypot(x - ax, y - ay)
            else:
                t = ((x - ax) * dx + (y - ay) * dy) / seg2
                t = max(0.0, min(1.0, t))
                d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            
            if d < best:
                best = d
        
        return best

    def _directional_factor(self, x: float, y: float, vx: float, vy: float, resolution, of_x, of_y, width, height, grad_x, grad_y) -> float:
        px, py = self._world_to_pixel(x, y, resolution, of_x, of_y)
        if px < 1 or px >= width - 1 or py < 1 or py >= height - 1:
            return 1.0
        
        gx = grad_x[py, px]
        gy = grad_y[py, px]
        gn = math.hypot(gx, gy)
        if gn < 1e-6:
            return 1.0
        
        gx /= gn
        gy /= gn
        
        vn = math.hypot(vx, vy)
        if vn < 1e-6:
            return 1.0
        
        dot = -(gx * vx + gy * vy)
        side = self.config.obstacle_side_factor
        
        return max(0.0, min(1.0, side + (1.0 - side) * dot))
    
    def _evaluate_trajectory(
        self, 
        states: list[tuple[float, float, float]], 
        target, 
        v: float,
        w: float,
        obstacle_distance_map, 
        resolution, 
        of_x, 
        of_y, 
        width, 
        height,
        robot_x,
        robot_y,
        path,
        target_seg_i,
        grad_x, grad_y,
    ):
        if len(states) <= 0 or obstacle_distance_map is None:
            return float("inf")

        for x, y, _ in states:
            if self._is_collision(x, y, obstacle_distance_map, resolution, of_x, of_y, width, height):
                return float("inf")

        target_x, target_y = target
        
        if v >= 0:
            movement_heading = states[-1][2]
        else:
            movement_heading = states[-1][2] + math.pi

        # heading
        ax, ay = path[target_seg_i]
        bx, by = path[target_seg_i + 1]
        path_heading = math.atan2(by - ay, bx - ax)
        
        heading_error = abs(self._normalize_angle(path_heading - movement_heading))

        heading_cost = self.config.weight_heading * heading_error

        # progress
        start_dist = math.hypot(robot_x - target_x, robot_y - target_y)
        end_dist = math.hypot(states[-1][0] - target_x, states[-1][1] - target_y)
        progress_cost = -self.config.weight_progress * (start_dist - end_dist)
        
        # distance
        # dists_to_target = [math.hypot(state[0] - target_x, state[1] - target_y) for state in states]
        # min_dist_to_target = min(dists_to_target)
        
        dist_to_target = math.hypot(states[-1][0] - target_x, states[-1][1] - target_y)
        dist_cost = self.config.weight_dist * (dist_to_target / (self.config.lookahead_distance + 1.0))

        # speed
        # speed_cost = -self.config.weight_speed * (v if v >= 0 else -v * 0.9)
        # if 0 < v < 0.1:
        #     tt = abs(v) / 0.1
        #     speed_cost += (1.0 - tt) * (1.0 - tt) * 0.1

        if start_dist < 0.3:
            speed_cost = 0.0
        else:
            speed_cost = self.config.weight_speed * (self.config.max_linear_speed - abs(v))

        min_viable_speed = self.config.min_viable_speed
        if 1e-3 < abs(v) < min_viable_speed:
            speed_cost += 3.0 * (1.0 - abs(v) / min_viable_speed)

        # # angular speed
        # w_speed_cost = self.config.weight_speed * (self.config.max_angular_speed - abs(w))

        # stationary
        stationary_penalty = 0.0
        w_dead = 0.10
        if abs(v) < self.config.min_viable_speed and abs(w) < w_dead:
            stationary_penalty = 5.0 * (1.0 - abs(w) / w_dead)

        # obstacle
        influence_distance = self.config.robot_radius * self.config.inf_mul + 0.15
        obstacle_cost = 0.0
        
        sign = 1.0 if v >= 0 else -1.0
        obstacle_cost = 0.0
        
        n = len(states)
        for i, (x, y, theta_i) in enumerate(states):
            if i % 2 == 1:
                continue

            clearance = self._get_obstacle_distance(x, y, resolution, of_x, of_y, width, height, obstacle_distance_map)
            if clearance > influence_distance:
                continue
            
            vx = sign * math.cos(theta_i)
            vy = sign * math.sin(theta_i)
            
            dir_factor = self._directional_factor(
                x, y, vx, vy,
                resolution, of_x, of_y, width, height, grad_x, grad_y
            )

            normalized = (influence_distance - clearance) / influence_distance
            temporal_weight = (i + 1) / n
            obstacle_cost += normalized * normalized * dir_factor * temporal_weight

        obstacle_cost *= self.config.weight_obstacle

        # path
        _, seg_i, _ = self._project_on_path(path, robot_x, robot_y)
        deviations = [self._path_deviation(sx, sy, path, seg_i) for sx, sy, _ in states]
        path_cost = self.config.weight_path * (sum(deviations) / len(deviations))

        return heading_cost + dist_cost + speed_cost \
            + obstacle_cost + progress_cost + path_cost \
            + stationary_penalty

    def _normalize_angle(self, angle: float):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _project_on_path(self, path, x, y):
        best = (float('inf'), 0, 0.0) # dist, seg_idx, dot
        for i in range(len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
            dx, dy = bx - ax, by - ay
            
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-9:
                d = math.hypot(x - ax, y - ay)
                t = 0.0
            
            else:
                t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
                d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            
            if d < best[0]:
                best = (d, i, t)
        
        return best
    
    def _target_from_proj(self, path, x, y):
        _, seg_i, t = self._project_on_path(path, x, y)
        
        ax, ay = path[seg_i]
        bx, by = path[seg_i + 1]
        
        px, py = ax + t * (bx - ax), ay + t * (by - ay)
        remaining = self.config.lookahead_distance
        
        seg_left = math.hypot(bx - px, by - py)
        if seg_left >= remaining:
            f = remaining / seg_left if seg_left > 0 else 0
            return px + f * (bx - px), py + f * (by - py), seg_i
        
        remaining -= seg_left
        
        for i in range(seg_i + 1, len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
            sl = math.hypot(bx - ax, by - ay)
            
            if sl >= remaining:
                f = remaining / sl if sl > 0 else 0
                return ax + f * (bx - ax), ay + f * (by - ay), i
            
            remaining -= sl
        
        return path[-1][0], path[-1][1], len(path) - 2

    def compute_control(self):
        snap = self._map_snapshot
        
        grid = snap["grid"]
        obstacle_distance_map = snap["obstacle_distance_map"]
        width = snap["width"]
        height = snap["height"]
        resolution = snap["resolution"]
        of_x = snap["offset_x"]
        of_y = snap["offset_y"]
        grad_x = snap["grad_x"]
        grad_y = snap["grad_y"]
        
        path = self.global_path
        
        x = self.robot_x
        y = self.robot_y
        theta = self.robot_heading
        v = self.current_v
        w = self.current_w

        if grid is None or path is None or len(path) < 2:
            return 0.0, 0.0

        # if we are close to the target
        last_x, last_y = path[-1]
        if math.hypot(x - last_x, y - last_y) < self.config.goal_tolerance:
            return 0.0, 0.0

        now = time.monotonic()
        control_dt = 0.05 if self._last_cmd_time is None else min(0.5, max(1e-3, now - self._last_cmd_time))
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

        # normal trajectory
        best_cost = float("inf")
        best_v = 0.0
        best_w = 0.0
        
        # penetration trajectory
        best_pen = float("inf")
        best_pen_v = 0.0
        best_pen_w = 0.0

        target_x, target_y, target_seg_i = self._target_from_proj(path, x, y)
        for v_cand in v_samples:
            for w_cand in w_samples:
                states = self._simulate_trajectory(v_cand, w_cand, x, y, theta)
                cost = self._evaluate_trajectory(states, (target_x, target_y), v_cand, w_cand, obstacle_distance_map, resolution, of_x, of_y, width, height, x, y, path, target_seg_i, grad_x, grad_y)

                if cost < best_cost:
                    best_cost = cost
                    best_v = v_cand
                    best_w = w_cand
                
                # if on current step all the costs were inf
                if best_cost == float("inf"):
                    pen = 0.0
                    start_c = self._get_obstacle_distance(x, y, resolution, of_x, of_y, width, height, obstacle_distance_map)
                    for sx, sy, _ in states:
                        c = self._get_obstacle_distance(sx, sy, resolution, of_x, of_y, width, height, obstacle_distance_map)
                        if c < self.config.robot_radius:
                            pen += self.config.robot_radius - c

                    # reward for coming from wall
                    end_c = self._get_obstacle_distance(states[-1][0], states[-1][1], resolution, of_x, of_y, width, height, obstacle_distance_map)
                    escape_reward = (end_c - start_c)
                    score = pen - 2.0 * escape_reward
                    if score < best_pen:
                        best_pen = score
                        best_pen_v, best_pen_w = v_cand, w_cand

        # instead of just standing still find the least-penetration trajectory
        if best_cost == float("inf"):
            return best_pen_v, best_pen_w

        return best_v, best_w

    @aparsedata(SLAMOffsetMap)
    async def on_slam_map(self, map):  # SLAMOffsetMap
        if self._edt_busy:
            return
        
        self._edt_busy = True
        
        try:
            obstacles = map.grid < self._obstacle_upper_threshold
            euc = (await asyncio.to_thread(distance_transform_edt, ~obstacles)).astype(np.float32)
            euc *= map.resolution
            
            gau = gaussian_filter(euc, sigma=1.0)
            gy, gx = np.gradient(gau, map.resolution)

            # single atomic swap
            self._map_snapshot = {
                "grid": map.grid,
                "width": map.width,
                "height": map.height,
                "offset_x": map.offset_x,
                "offset_y": map.offset_y,
                "resolution": map.resolution,
                "obstacle_distance_map": euc,
                "grad_x": gx,
                "grad_y": gy,
            }
            self._last_map_ts = time.monotonic()
        
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
        self._last_odom_ts = time.monotonic()
    
    # @aparsedata(TimedMovement3DoF)
    # async def on_motorcontroller_odometry(self, odom):
    #     self._last_odom_ts = time.monotonic()
    #     self.last_odom_pose = odom.movement
        
    #     if self.last_slam_pose is not None and self.saved_odom_pose != None:
    #         anchor_slam = self.last_slam_pose
    #         anchor_odom = self.saved_odom_pose
    #         cur_odom = self.last_odom_pose
            
    #         self.robot_x = anchor_slam.x + (cur_odom.x - anchor_odom.x)
    #         self.robot_y = anchor_slam.y + (cur_odom.y - anchor_odom.y)
    #         self.robot_heading = self._normalize_angle(
    #             anchor_slam.theta + self._normalize_angle(cur_odom.theta - anchor_odom.theta)
    #         )

    @aparsedata(Vector)
    async def on_motorcontroller_velocity(self, velocity: Vector):
        self.current_v = velocity.x
        self.current_w = velocity.y

    @aparsedata(NumpyArray)
    async def on_pathplanner_globalpath(self, path: np.ndarray):
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
        while client.client._is_running.is_set():
            next_tick += cmdvel_post_delay
            delay = next_tick - loop.time()
            
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                next_tick = loop.time()
            
            ts = time.monotonic()
            if ts - client._last_odom_ts >= odom_max_age or ts - client._last_map_ts >= map_max_age:
                await cmdvel_topic.post(Vector(0, 0, 0))
                continue

            try:
                v, w = await asyncio.to_thread(client.compute_control)
            except Exception as e:
                print(f"compute_control failed: {e}", flush=True)
                v, w = 0.0, 0.0
            
            await cmdvel_topic.post(Vector(v, w, 0))

    path_track = asyncio.create_task(run_path_tracker())

    try:
        await asyncio.gather(client.run(), path_track)

    finally:
        await client.stop()

asyncio.run(main())
