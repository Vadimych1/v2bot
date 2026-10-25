from scipy.ndimage import distance_transform_edt, gaussian_filter
import numpy as np
import traceback
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
        
        self._last_cmd_time = None
        
        self._edt_busy = False

    def _world_to_pixel(self, wx: float, wy: float, resolution, of_x, of_y):
        px = math.floor(wx / resolution + of_x)
        py = math.floor(wy / resolution + of_y)

        return px, py

    def _get_obstacle_distance(self, x: float, y: float, resolution, of_x, of_y, width, height, obstacle_distance_map):
        if obstacle_distance_map is None:
            return 0.0

        px, py = self._world_to_pixel(x, y, resolution, of_x, of_y)

        # outside the map is considered a collision
        if px < 0 or px >= width or py < 0 or py >= height:
            return 0.0

        return obstacle_distance_map[py, px]

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

        goal_x = float(path[-1][0])
        goal_y = float(path[-1][1])

        if math.hypot(x - goal_x, y - goal_y) < self.config.goal_tolerance:
            return 0.0, 0.0

        # DWA
        now = time.monotonic()
        control_dt = 0.05 if self._last_cmd_time is None else min(0.5, max(1e-3, now - self._last_cmd_time))
        self._last_cmd_time = now
        
        v_min = max(self.config.min_linear_speed, v - self.config.max_linear_accel * control_dt)
        v_max = min(self.config.max_linear_speed, v + self.config.max_linear_accel * control_dt)
        w_min = max(-self.config.max_angular_speed, w - self.config.max_angular_accel * control_dt)
        w_max = min(self.config.max_angular_speed, w + self.config.max_angular_accel * control_dt)

        v_samples = np.linspace(v_min, v_max, self.config.num_v_samples)
        w_samples = np.linspace(w_min, w_max, self.config.num_w_samples)

        target_x, target_y, target_seg_i = self._target_from_proj(path, x, y)

        ax, ay = path[target_seg_i]
        bx, by = path[target_seg_i + 1]
        path_heading_fwd = math.atan2(by - ay, bx - ax)

        window_lo = max(0, target_seg_i - 5)
        window_hi = min(len(path) - 1, target_seg_i + 20)
        if window_hi <= window_lo:
            window_lo = max(0, target_seg_i)
            window_hi = min(len(path) - 1, window_lo + 1)

        seg_ax = np.array([path[i][0] for i in range(window_lo, window_hi)], dtype=np.float64)
        seg_ay = np.array([path[i][1] for i in range(window_lo, window_hi)], dtype=np.float64)
        seg_bx = np.array([path[i + 1][0] for i in range(window_lo, window_hi)], dtype=np.float64)
        seg_by = np.array([path[i + 1][1] for i in range(window_lo, window_hi)], dtype=np.float64)

        seg_dx = seg_bx - seg_ax
        seg_dy = seg_by - seg_ay
        seg2 = seg_dx * seg_dx + seg_dy * seg_dy
        seg2_safe = np.where(seg2 < 1e-9, 1e-9, seg2)
        
        V, W = np.meshgrid(v_samples, w_samples, indexing="ij")
        V = V.ravel()
        W = W.ravel()
        N = V.size()

        dt = self.config.dt
        dist_to_goal = math.hypot(x - goal_x, y - goal_y)
        pred_time = self.config.predict_time * max(0.4, min(1.0, dist_to_goal / 1.0))
        steps = max(1, round(pred_time / dt))
        t_k = np.arange(1, steps + 1) * dt
        
        theta_ks = theta + W[:, None] * t_k[None, :]
        cos_t0 = math.cos(theta)
        sin_t0 = math.sin(theta)
        
        # straight paths (w ~ 0)
        x_straight = x + V[:, None] * t_k[None, :] * cos_t0
        y_straight = y + V[:, None] * t_k[None, :] * sin_t0

        # curved paths (w != 0)
        mask_w0 = np.abs(W) < 1e-6
        W_safe = np.where(mask_w0, 1e-6, W)
        
        ratio = V / W_safe
        x_curved = x + ratio[:, None] * (np.sin(theta_ks) - sin_t0)
        y_curved = y + ratio[:, None] * (np.cos(theta_ks) - cos_t0)
        
        m = mask_w0[:, None]
        x_ks = np.where(m, x_straight, x_curved)
        y_ks = np.where(m, y_straight, y_curved)
        
        px = np.floor(x_ks / resolution + of_x).astype(np.int32)
        py = np.floor(y_ks / resolution + of_y).astype(np.int32)
        inside = (px >= 0) & (px < width) & (py >= 0) & (py < height)
        
        px_c = np.clip(px, 0, width - 1)
        py_c = np.clip(py, 0, height - 1)
        
        # collision
        clearance = np.where(inside, obstacle_distance_map[py_c, px_c], 0.0)
        coll_thresh = self.config.robot_radius * 0.9
        collided = np.any(clearance <= coll_thresh, axis=1)
        
        # heading
        x_K = x_ks[:, -1]
        y_K = y_ks[:, -1]
        theta_K = theta_ks[:, -1]
        
        movement_heading = np.where(V >= 0, theta_K, theta_K + math.pi)
        mh_n = (movement_heading + np.pi) % (2 * np.pi) - np.pi
        
        if path_heading_fwd is not None:
            ph = np.where(V >= 0, path_heading_fwd, path_heading_fwd + np.pi)
            ph_n = (ph + np.pi) % (2 * np.pi) - np.pi
            heading_err = np.abs((ph_n - mh_n + np.pi) % (2 * np.pi) - np.pi)
        
        else:
            at = np.arctan2(target_y - y_K, target_x - x_K)
            at_n = (at + np.pi) % (2 * np.pi) - np.pi
            heading_err = np.abs((at_n - mh_n + np.pi) % (2 * np.pi) - np.pi)
        
        heading_cost = self.config.weight_heading * heading_err
        
        # progress & distance
        start_dist = math.hypot(x - target_x, y - target_y)
        end_dist = np.hypot(x_K - target_x, y_K - target_y)
        progress_cost = -self.config.weight_progress * (start_dist - end_dist)
        dist_cost = self.config.weight_dist * (end_dist / (self.config.lookahead_distance + 1.0))
        
        # speed cost & stationary penalty
        abs_v = np.abs(V)
        speed_cost = np.where(
            start_dist < 0.3, 0.0,
            self.config.weight_speed * (self.config.max_linear_speed - abs_v),
        )
        min_viable = self.config.min_viable_speed
        small_v = (abs_v > 1e-3) & (abs_v < min_viable)
        speed_cost = speed_cost + np.where(small_v, 3.0 * (1.0 - abs_v / min_viable), 0.0)
        
        stationary = (abs_v < 0.02) & (np.abs(W) < 1e-3)
        stationary_penalty = np.where(stationary, 5.0, 0.0)
        
        # obstacle cost
        influence_distance = self.config.robot_radius * self.config.inf_mul + 0.15
        sign = np.where(V >= 0, 1.0, -1.0)
        
        step_mask = np.zeros(steps, dtype=bool)
        step_mask[::2] = True
        
        clearance_sel = clearance[:, step_mask]
        # inside_sel =    inside[:,    step_mask]
        theta_sel =     theta_ks[:,  step_mask]
        px_sel =        px_c[:,      step_mask]
        py_sel =        py_c[:,      step_mask]

        gx = grad_x[py_sel, px_sel]
        gy = grad_y[py_sel, px_sel]
        gn = np.hypot(gx, gy)
        gn_safe = np.where(gn < 1e-6, 1.0, gn)
        gx_n = gx / gn_safe
        gy_n = gy / gn_safe
        
        vx = sign[:, None] * np.cos(theta_sel)
        vy = sign[:, None] * np.sin(theta_sel)
        dot = -(gx_n * vx + gy_n * vy)
        
        side = self.config.obstacle_side_factor
        dir_factor = np.clip(side + (1.0 - side) * dot, 0.0, 1.0)
        dir_factor = np.where(gn < 1e-6, 1.0, dir_factor)
        
        in_influence = clearance_sel <= influence_distance
        normalized = np.where(in_influence, (influence_distance - clearance_sel) / influence_distance, 0.0)
        obstacle_cost = self.config.weight_obstacle * np.sum(normalized * normalized * dir_factor, axis=1)
        
        # path cost
        tx = x_K[:, None] - seg_ax[None, :]
        ty = y_K[:, None] - seg_ay[None, :]

        t_param = (tx * seg_dx[None, :] + ty * seg_dy[None, :]) / seg2_safe[None, :]
        t_param = np.clip(t_param, 0.0, 1.0)

        cx = seg_ax[None, :] + t_param * seg_dx[None, :]
        cy = seg_ay[None, :] + t_param * seg_dy[None, :]

        dists = np.hypot(x_K[:, None] - cx, y_K[:, None] - cy)
        path_dev = np.min(dists, axis=1)
        path_cost = self.config.weight_path * path_dev
        
        total_cost = (
            heading_cost + dist_cost + speed_cost + obstacle_cost
            + progress_cost + path_cost + stationary_penalty
        )
        total_cost = np.where(collided, np.inf, total_cost)
        
        if np.any(np.isfinite(total_cost)):
            best = int(np.argmin(total_cost))
            return float(V[best]), float(W[best])
        
        # fallback if all costs were inf
        start_c = self._get_obstacle_distance(x, y, resolution, of_x, of_y, width, height, obstacle_distance_map)
        pen = np.sum(np.maximum(0.0, self.config.robot_radius - clearance), axis=1)
        end_c = clearance[:, -1]
        escape_reward = end_c - start_c
        score = pen - 2.0 * escape_reward + 0.5 * heading_err

        best = int(np.argmin(score))

        return float(V[best]), float(W[best])

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
            self.robot_heading = self._normalize_angle(
                anchor_slam.theta + self._normalize_angle(cur_odom.theta - anchor_odom.theta)
            )

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
            
            ts = time.time()
            if ts - client._last_odom_ts >= odom_max_age or ts - client._last_map_ts >= map_max_age:
                await cmdvel_topic.post(Vector(0, 0, 0))
                continue

            try:
                v, w = await asyncio.to_thread(client.compute_control)
            except Exception as e:
                print(f"compute_control failed: {traceback.format_exception(e)}", flush=True)
                v, w = 0.0, 0.0
            
            await cmdvel_topic.post(Vector(v, w, 0))

    path_track = asyncio.create_task(run_path_tracker())

    try:
        await asyncio.gather(client.run(), path_track)

    finally:
        await client.stop()

asyncio.run(main())
