from scipy.ndimage import distance_transform_edt, gaussian_filter
import numpy as np
import cv2 as cv
import skfmm
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
        self.weight_decision_deviation = get_config("motion_controller.weights.decision_deviation")
        self.weight_commitment = get_config("motion_controller.weights.commitment")
        self.weight_reverse = get_config("motion_controller.weights.reverse")
        
        self.commitment_w_threshold = get_config("motion_controller.weights.commitment_w_threshold")
        
        self.obstacle_side_factor = get_config("motion_controller.weights.obstacle_side_factor")
        self.min_viable_speed = get_config("motion_controller.kinematics.min_viable_speed")

        self.goal_tolerance = get_config("motion_controller.goal_tolerance")
        self.lookahead_distance = get_config("motion_controller.lookahead_distance")
        self.inf_mul = get_config("motion_controller.influence_distance_multiplier")

        self.alpha_v = get_config("motion_controller.smoothing.alpha_v")
        self.alpha_w = get_config("motion_controller.smoothing.alpha_w")

        self.fmm_map_change_threshold = get_config("motion_controller.fmm_map_change_threshold")

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
            "fma_speed_masked": None,
            "fma_phi": None,
        }
        
        self._fmm_snapshot = {
            "T": None, 
            "gx_T": None, 
            "gy_T": None, 
            "goal": None
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
        
        # smoothed control
        self._smoothed_w = 0.0
        self._smoothed_v = 0.0

        # global path from path_planner
        self.global_path = None

        self._obstacle_upper_threshold = get_config("obstacle_upper_threshold")
        self._free_lower_threshold = get_config("free_lower_threshold")

        self._last_map_ts = -1
        self._last_odom_ts = -1
        
        self._last_cmd_time = time.monotonic()
        
        self._edt_busy = False
        self._fmm_busy = False

    def _world_to_pixel(self, wx: float, wy: float, resolution, of_x, of_y):
        px = math.floor(wx / resolution + of_x)
        py = math.floor(wy / resolution + of_y)

        return px, py

    def _is_collision(self, x: float, y: float, obstacle_distance_map, resolution, of_x, of_y, width, height):
        if obstacle_distance_map is None:
            return False

        clearance = self._get_obstacle_distance(x, y, resolution, of_x, of_y, width, height, obstacle_distance_map)
        return clearance <= self.config.robot_radius

    def _simulate_trajectory(self, v: float, w: float, x, y, theta, clearance_now):
        states = []
        dt = self.config.dt

        scale = min(1.0, clearance_now / (2.0 * self.config.robot_radius))
        pred_time = self.config.predict_time * max(0.4, scale)
        
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
    
    def _get_cost_to_go(self, x, y, T, resolution, of_x, of_y, width, height):
        if T is None:
            return float('inf')
        
        px, py = self._world_to_pixel(x, y, resolution, of_x, of_y)
        if px < 0 or px >= width or py < 0 or py >= height:
            return float('inf')

        v = T[py, px]
        if not np.isfinite(v):
            return float('inf')

        return float(v)
    
    def _evaluate_trajectory(
        self, 
        states: list[tuple[float, float, float]], 
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
        prev_v, prev_w,
        T, gx_T, gy_T,
    ):
        if len(states) <= 0 or obstacle_distance_map is None:
            return float("inf")

        for x, y, _ in states:
            if self._is_collision(x, y, obstacle_distance_map, resolution, of_x, of_y, width, height):
                return float("inf")
        
        if v >= 0:
            movement_heading = states[-1][2]
        else:
            movement_heading = states[-1][2] + math.pi
        
        # decision deviation
        dev_v = (v - prev_v) / max(self.config.max_linear_speed, 1e-3)
        dev_w = (w - prev_w) / max(self.config.max_angular_speed, 1e-3)
        deviation_cost = (dev_v * dev_v + dev_w * dev_w) * self.config.weight_decision_deviation

        # heading
        ax, ay = path[target_seg_i]
        bx, by = path[target_seg_i + 1]
        path_heading = math.atan2(by - ay, bx - ax)
        
        T_heading = None
        px, py = self._world_to_pixel(states[-1][0], states[-1][1], resolution, of_x, of_y)
        if (gx_T is not None) and (gy_T is not None) and 0 <= px < width and 0 <= py < height:
            gx = gx_T[py, px]
            gy = gy_T[py, px]
            
            if np.isfinite(gx) and np.isfinite(gy) and (gx * gx + gy * gy) > 1e-8:
                T_heading = math.atan2(-gy, -gx)
        
        if T_heading is not None:
            geo_err = abs(self._normalize_angle(T_heading - movement_heading))
        
        else:
            geo_err = abs(self._normalize_angle(path_heading - movement_heading))

        heading_error = 0.5 * geo_err + 0.5 * abs(self._normalize_angle(path_heading - movement_heading))
        heading_cost = self.config.weight_heading * heading_error

        # commitment
        commitment_cost = 0.0
        if abs(prev_w) > self.config.commitment_w_threshold:
            same_dir = (w * prev_w) > 0.0
            magnitude = min(abs(prev_w), abs(w))
            
            if same_dir:
                commitment_cost = -self.config.weight_commitment * magnitude
            else:
                commitment_cost = +self.config.weight_commitment * magnitude

        # progress
        T_start = self._get_cost_to_go(robot_x, robot_y, T, resolution, of_x, of_y, width, height)
        T_end = self._get_cost_to_go(states[-1][0], states[-1][1], T, resolution, of_x, of_y, width, height)
        
        if not np.isfinite(T_end):
            return float('inf') # destination is not reachable
        
        if not np.isfinite(T_start):
            T_start = float(T_end) + 100.0
        
        progress = T_start - T_end
        progress_cost = -self.config.weight_progress * progress
        
        # distance
        ## iteration 1: min distance of every position
        # dists_to_target = [math.hypot(state[0] - target_x, state[1] - target_y) for state in states]
        # min_dist_to_target = min(dists_to_target)
        
        ## iteration 2: final distance
        # dist_to_target = math.hypot(states[-1][0] - target_x, states[-1][1] - target_y)
        # dist_cost = self.config.weight_dist * (dist_to_target / (self.config.lookahead_distance + 1.0))

        ## iteration 3: FMM-based distance
        dist_cost = self.config.weight_dist * (T_end / max(T_start, 0.1))

        # speed
        # speed_cost = -self.config.weight_speed * (v if v >= 0 else -v * 0.9)
        # if 0 < v < 0.1:
        #     tt = abs(v) / 0.1
        #     speed_cost += (1.0 - tt) * (1.0 - tt) * 0.1

        # if start_dist < 0.3:
        #     speed_cost = 0.0
        # else:
        #     speed_cost = self.config.weight_speed * (self.config.max_linear_speed - abs(v))

        speed_cost = self.config.weight_speed * max(0.0, self.config.max_linear_speed - abs(v))
        min_viable_speed = self.config.min_viable_speed
        if 1e-3 < abs(v) < min_viable_speed:
            speed_cost += 3.0 * (1.0 - abs(v) / min_viable_speed)

        # reverse
        reverse_cost = self.config.weight_reverse * abs(v) if v < 0.0 else 0.0

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
            
            # temporal_weight = (i + 1) / n
            temporal_weight = math.sqrt(1 / n)
            
            obstacle_cost += normalized * normalized * dir_factor * temporal_weight

        obstacle_cost *= self.config.weight_obstacle

        # path
        _, seg_i, _ = self._project_on_path(path, robot_x, robot_y)
        deviations = [self._path_deviation(sx, sy, path, seg_i) for sx, sy, _ in states]
        path_cost = self.config.weight_path * (sum(deviations) / len(deviations))

        return heading_cost   + dist_cost       + speed_cost \
             + obstacle_cost  + progress_cost   + path_cost \
             + deviation_cost + commitment_cost + reverse_cost \
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

    def _prepare_map(self, map):
        if self._edt_busy:
            return False
        
        self._edt_busy = True
        return_result = False
        try:
            if self._map_snapshot["grid"] is None or self._map_snapshot["grid"].shape != map.grid.shape:
                return_result = True
            
            elif np.count_nonzero(self._map_snapshot["grid"] - map.grid) > self.config.fmm_map_change_threshold:
                return_result = True
            
            grid = map.grid
            # robot_r = int(math.ceil((self.config.robot_radius / 2) / map.resolution))
            # kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (2 * robot_r + 1, 2 * robot_r + 1))

            # dilation
            # obstacle_mask = (grid < self._obstacle_upper_threshold).astype(np.uint8) * 255
            # dilated_mask = cv.dilate(obstacle_mask, kernel, iterations=1)
            # grid[dilated_mask == 255] = 0
            
            obstacles = grid < self._obstacle_upper_threshold
            euc = distance_transform_edt(~obstacles).astype(np.float32)
            euc *= map.resolution
            
            # fmm preparations
            speed = np.ones(grid.shape, dtype=float)
            # speed[euc < self.config.robot_radius * 0.1] = 0.5 # reduce speed near walls
            speed_masked = np.ma.MaskedArray(speed, mask=obstacles)
            
            # obstacle gradient
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
                "fma_speed_masked": speed_masked,
                "fma_mask": obstacles,
            }
            self._last_map_ts = time.monotonic()

        finally:
            self._edt_busy = False
    
        return return_result

    def _rebuild_fmm(self):
        snap = self._map_snapshot
        if snap["grid"] is None or self.global_path is None or len(self.global_path) < 2:
            return
        
        if self._fmm_busy:
            return
        
        self._fmm_busy = True

        try:
            goal_x, goal_y = self.global_path[-1]
            px, py = self._world_to_pixel(goal_x, goal_y, snap["resolution"], snap["offset_x"], snap["offset_y"])
            
            if not (0 <= px < snap["width"] and 0 <= py < snap["height"]):
                return
            
            speed_masked = snap["fma_speed_masked"]
            mask = snap["fma_mask"]
            
            phi = np.ones(snap["grid"].shape, dtype=float)
            
            if mask[py, px]:
                r = 10
                y0, y1 = max(0, py - r), min(mask.shape[0], py + r + 1)
                x0, x1 = max(0, px - r), min(mask.shape[0], px + r + 1)
                
                free = np.argwhere(~mask[y0:y1, x0:x1])
                
                if len(free) == 0:
                    return
                
                d = ((free - [py - y0, px - x0]) ** 2).sum(axis=1)
                dy, dx = free[np.argmin(d)]
                py, px = y0 + dy, x0 + dx
            
            phi[py, px] = -1.0
            phi_masked = np.ma.MaskedArray(phi, mask=mask)
            
            T_masked = skfmm.travel_time(phi_masked, speed_masked)
            T = T_masked.filled(np.inf).astype(np.float32)
            
            with np.errstate(invalid="ignore"):
                gy_T, gx_T = np.gradient(T, snap["resolution"])
            
            self._fmm_snapshot = {
                "T": T,
                "gx_T": gx_T,
                "gy_T": gy_T,
            }
        
        finally:
            self._fmm_busy = False

    def compute_control(self):
        snap = self._map_snapshot
        fmm_snap = self._fmm_snapshot
        
        grid = snap["grid"]
        obstacle_distance_map = snap["obstacle_distance_map"]
        width = snap["width"]
        height = snap["height"]
        resolution = snap["resolution"]
        of_x = snap["offset_x"]
        of_y = snap["offset_y"]
        grad_x = snap["grad_x"]
        grad_y = snap["grad_y"]
        
        T = fmm_snap["T"]
        gx_T = fmm_snap["gx_T"]
        gy_T = fmm_snap["gy_T"]
        
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

        clearance_now = self._get_obstacle_distance(x, y, resolution, of_x, of_y, width, height, obstacle_distance_map)
        _, _, target_seg_i = self._target_from_proj(path, x, y)
        for v_cand in v_samples:
            for w_cand in w_samples:
                # turn-or-move mode
                if not np.isclose(w_cand, 0) and not np.isclose(v_cand, 0):
                    continue
                
                states = self._simulate_trajectory(v_cand, w_cand, x, y, theta, clearance_now)
                cost = self._evaluate_trajectory(
                    states, 
                    v_cand, w_cand,
                    obstacle_distance_map, resolution, of_x, of_y, width, height,
                    x, y, path, target_seg_i,
                    grad_x, grad_y,
                    # smoothed variables are always equal to previously-used
                    self._smoothed_v, self._smoothed_w,
                    T, gx_T, gy_T
                )

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
            self._smoothed_w = self.config.alpha_w * best_pen_w + (1 - self.config.alpha_w) * self._smoothed_w
            self._smoothed_v = self.config.alpha_v * best_pen_v + (1 - self.config.alpha_v) * self._smoothed_v

        else:
            self._smoothed_w = self.config.alpha_w * best_w + (1 - self.config.alpha_w) * self._smoothed_w
            self._smoothed_v = self.config.alpha_v * best_v + (1 - self.config.alpha_v) * self._smoothed_v

        return self._smoothed_v, self._smoothed_w

    @aparsedata(SLAMOffsetMap)
    async def on_slam_map(self, map):  # SLAMOffsetMap        
        asyncio.create_task(self._handle_map(map))

    async def _handle_map(self, map):
        map_changed = await asyncio.to_thread(self._prepare_map, map)
        if map_changed:
            await asyncio.to_thread(self._rebuild_fmm)

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
        rebuild_fmm = self.global_path is None or np.linalg.norm(self.global_path[-1] - path[-1]) > 0.2  
        self.global_path = path

        if rebuild_fmm:
            await asyncio.to_thread(self._rebuild_fmm)


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
