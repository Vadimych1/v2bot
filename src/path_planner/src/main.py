import asyncio
import time

from scipy.spatial import KDTree
import math, random
import numpy as np
import cv2 as cv

from miniros import AsyncROSClient
from miniros.util.decorators import aparsedata
from miniros_slam.source.datatypes import SLAMOffsetMap
from miniros.util.datatypes import TimedMovement3DoF, NumpyArray, Vector
from miniros_configurator import get_config

from dataclasses import dataclass

# spatial index for RRT*
class _RRTIndex:
    __slots__ = ("rebuild_every", "_pos", "_kd", "_kd_size", "_pending")
    
    def __init__(self, rebuild_every: int = 64) -> None:
        self.rebuild_every = rebuild_every
        self._pos: list[tuple[float, float]] = []
        self._kd: KDTree | None = None
        self._kd_size = 0
        self._pending: list[int] = []
    
    def insert(self, pos) -> int:
        idx = len(self._pos)
        self._pos.append((float(pos[0]), float(pos[1])))
        self._pending.append(idx)
        if len(self._pending) >= self.rebuild_every:
            self._rebuild()
        return idx
    
    def _rebuild(self) -> None:
        self._kd = KDTree(self._pos)
        self._kd_size = len(self._pos)
        self._pending.clear()
    
    def nearest(self, pos) -> int:
        if not self._pos:
            return -1
        
        best_idx = -1
        best_d = float('inf')
        
        if self._kd is not None and self._kd_size > 0:
            d, i = self._kd.query(pos)
            best_d = float(d)
            best_idx = int(i)
        
        px, py = pos
        for idx in self._pending:
            x, y = self._pos[idx]
            d = math.hypot(px - x, py - y)
            if d < best_d:
                best_d = d
                best_idx = idx
        
        return best_idx
    
    def near(self, pos, radius: float) -> list[int]:
        if not self._pos:
            return []
        
        if self._kd is not None and self._kd_size > 0:
            result = list(self._kd.query_ball_point(pos, radius))
        else:
            result = []
        
        px, py = pos
        r2 = radius * radius
        for idx in self._pending:
            x, y = self._pos[idx]
            dx = px - x
            dy = py - y
            if dx * dx + dy * dy <= r2:
                result.append(idx)
        
        return result

@dataclass(slots=True)
class _RRTNode:
    pos: tuple[float, float]
    parent: int
    cost: float
    children: list[int]    

class PathPlanner(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("pathplanner", ip, port, _parse_handlers)

        self.robot_radius_m = get_config("robot_radius")

        # grid map retrieved from slam
        self.grid = None
        self.dilated_grid = None

        # map size in pixels
        self.width = 0
        self.height = 0
        self.d_width = 0
        self.d_height = 0

        # (0, 0) offset on map
        # in pixels
        self.offset_x = 0
        self.offset_y = 0

        # map resolution (meters per pixel)
        self.resolution = 1

        # start pose (x, y) in meters
        self.start_pos = None

        # target pos (x, y) in meters
        self.end_pos = None

        # world size
        world = get_config("path_planner.world")
        self.min_x = world["min_x"]
        self.min_y = world["min_y"]
        self.max_x = world["max_x"]
        self.max_y = world["max_y"]

        self._obstacle_upper_threshold = get_config("obstacle_upper_threshold")
        self._free_lower_threshold = get_config("free_lower_threshold")
        self._allow_unknown = False
        
        self._greedy_shortcut_max_lookahead = get_config("path_planner.pathfinder.greedy_shortcut_max_lookahead")
        self._path_smooth_iter = get_config("path_planner.pathfinder.path_smooth_iter")

    def _dilate_grid(self, grid) -> bool:
        if grid is None:
            return False

        robot_r = int(math.ceil(self.robot_radius_m / self.resolution))
        kernel = cv.getStructuringElement(
            cv.MORPH_ELLIPSE, (2 * robot_r + 1, 2 * robot_r + 1)
        )

        obstacle_mask = (self.grid < self._obstacle_upper_threshold).astype(
            np.uint8
        ) * 255
        dilated_mask = cv.dilate(obstacle_mask, kernel, iterations=1)

        self.dilated_grid = grid.copy()
        self.dilated_grid[dilated_mask == 255] = 0

        return True

    def _world_to_pixel(self, world_point: tuple[float, float]):
        wx, wy = world_point

        px = int(round(wx / self.resolution + self.offset_x))
        py = int(round(wy / self.resolution + self.offset_y))

        return px, py

    def _pixel_to_world(self, pixel_point: tuple[int, int]) -> tuple[float, float]:
        px, py = pixel_point

        wx = (px - self.offset_x) * self.resolution
        wy = (py - self.offset_y) * self.resolution

        return wx, wy

    def _is_free(self, pixel_point: tuple[int, int] | tuple[float, float]):
        px, py = pixel_point
        px, py = int(px), int(py)
        wx, wy = self._pixel_to_world((px, py))

        # if out-of-bounds, allow to move to at most +-d_width or +-d_height
        if px < 0 or px >= self.d_width or py < 0 or py >= self.d_height:
            return self.min_x < wx < self.max_x and self.min_y < wy < self.max_y

        # NOTE: those two conditions could make robot try to go through
        # the walls; although that should not happen while moving correctly,
        # note that this could be the possible problem; needs fixing

        # for robot to not stuck in the walls
        if (
            self.start_pos is not None
            and self._distance((wx, wy), self.start_pos) <= (self.robot_radius_m * 0.6)
        ):
            return True
        
        # for robot to be able to move to end_pos if it is near the wall
        if (
            self.end_pos is not None
            and self._distance((wx, wy), self.end_pos) <= (self.robot_radius_m * 0.6)
        ):
            return True

        if self.dilated_grid is None:
            return False

        val = self.dilated_grid[py, px]
        if val < self._obstacle_upper_threshold:
            return False
        
        if not self._allow_unknown and val < self._free_lower_threshold:
            return False
        
        return True

    def _line_of_sight(
        self,
        p1: tuple[float, float] | tuple[int, int],
        p2: tuple[float, float] | tuple[int, int],
    ):
        x1, y1 = p1
        x2, y2 = p2

        x1, y1 = int(x1), int(y1)
        x2, y2 = int(x2), int(y2)

        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1

        err = dx - dy

        while True:
            if not self._is_free((x1, y1)):
                return False

            if x1 == x2 and y1 == y2:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx

            if e2 < dx:
                err += dx
                y1 += sy

        return True

    def _distance(self, a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def _steer(
        self, from_pos: tuple[float, float], to_pos: tuple[float, float], step: float
    ) -> tuple[float, float]:
        dist = self._distance(from_pos, to_pos)

        if dist <= step:
            return to_pos

        ratio = step / dist

        new_x = from_pos[0] + (to_pos[0] - from_pos[0]) * ratio
        new_y = from_pos[1] + (to_pos[1] - from_pos[1]) * ratio

        return new_x, new_y

    def _propagate_cost_delta(self, tree, root_idx: int, delta: float) -> None:
        stack = [root_idx]
        while stack:
            i = stack.pop()
            tree[i]["cost"] += delta
            stack.extend(tree[i]["children"])

    def _greedy_shortcut(
        self, pixel_path: list[tuple[int, int]], max_lookahead: int = 6
    ):
        if len(pixel_path) <= 2:
            return pixel_path

        optimized_path = [pixel_path[0]]
        current_idx = 0
        n = len(pixel_path)

        while current_idx < n - 1:
            found_shortcut = False

            for target_idx in range(
                min(n - 1, current_idx + max_lookahead), current_idx + 1, -1
            ):
                dot_a = pixel_path[current_idx]
                dot_b = pixel_path[target_idx]

                if self._line_of_sight(dot_a, dot_b):
                    optimized_path.append(dot_b)

                    current_idx = target_idx
                    found_shortcut = True

                    break

            if not found_shortcut:
                current_idx += 1
                optimized_path.append(pixel_path[current_idx])

        return optimized_path
    
    def _smooth_path(self, path, iterations=3):
        for _ in range(iterations):
            new_path = [path[0]]
            for i in range(1, len(path) - 1):
                x = 0.25 * path[i - 1][0] + 0.5 * path[i][0] + 0.25 * path[i + 1][0]
                y = 0.25 * path[i - 1][1] + 0.5 * path[i][1] + 0.25 * path[i + 1][1]
                
                if self._line_of_sight(new_path[-1], (x, y)) and self._is_free((x, y)):
                    new_path.append((x, y))
                
                else:
                    new_path.append(path[i])
                
                new_path.append(path[-1])

        return path

    def find_path(
        self,
        max_iter: int = 2000,
        step_size: float = 10.0,
        goal_tolerance: float = 10.0,
        search_radius_factor: float = 1.0,
        allow_unknown: bool = False,
    ) -> list[tuple[float, float]] | None:
        """
        RRT* Pathfinding
        """
        
        self._allow_unknown = allow_unknown
        grid = self.grid
        start_pos = self.start_pos
        end_pos = self.end_pos

        if grid is None or start_pos is None or end_pos is None:
            return None

        # preprocess grid using dilation
        if not self._dilate_grid(grid):
            return None

        self.d_width = self.width
        self.d_height = self.height

        start_pixel = self._world_to_pixel(start_pos)
        end_pixel = self._world_to_pixel(end_pos)

        end_free = self._is_free(end_pixel)
        if not end_free:
            return None

        tree = [_RRTNode(pos=start_pixel, parent=-1, cost=0.0, children=[])]
        index = _RRTIndex(rebuild_every=64)
        index.insert(start_pixel)

        gamma_factor = get_config("path_planner.pathfinder.rrt_star_gamma")
        radius_floor = get_config("path_planner.pathfinder.rrt_star_radius_floor")
        radius_cap = get_config("path_planner.pathfinder.rrt_star_radius_cap")
        goal_bias = get_config("path_planner.pathfinder.goal_bias_chance")
        
        def get_radius(n: int) -> float:
            if n <= 1:
                return step_size * 2.0
            
            gamma = gamma_factor * search_radius_factor
            r_theoretical = gamma * math.sqrt(math.log(n) / n)
            
            r_floored = max(r_theoretical, step_size * radius_floor)
            return min(r_floored, step_size * radius_cap)

        for _ in range(max_iter):
            if random.random() < goal_bias:
                jitter = step_size * 2
                rand_x = end_pos[0] + random.uniform(-jitter, jitter)
                rand_x = end_pos[1] + random.uniform(-jitter, jitter)
            
            else:
                rand_x = random.uniform(self.min_x, self.max_x)
                rand_y = random.uniform(self.min_y, self.max_y)
            
            rand_point = self._world_to_pixel((rand_x, rand_y))

            nearest_idx = index.nearest(rand_point)
            nearest_node = tree[nearest_idx]
            nearest_pos = nearest_node.pos

            new_pos = self._steer(nearest_pos, rand_point, step_size)

            if not self._is_free(new_pos) or not self._line_of_sight(
                nearest_pos, new_pos
            ):
                continue

            radius = get_radius(len(tree))
            near_indices = index.near(new_pos, radius)

            # best parent selection
            min_cost = float("inf")
            best_parent = nearest_idx
            for idx in near_indices:
                node = tree[idx]
                if self._line_of_sight(node.pos, new_pos):
                    cost_candidate = node.cost + self._distance(node.pos, new_pos)
                    if cost_candidate < min_cost:
                        min_cost = cost_candidate
                        best_parent = idx

            # fallback: nearest not in near_indices (small radius, big step)
            if nearest_idx not in near_indices:
                cost_candidate = nearest_node.cost + self._distance(nearest_pos, new_pos)
                if cost_candidate < min_cost:
                    min_cost = cost_candidate
                    best_parent = nearest_idx

            if min_cost == float("inf"):
                continue

            # insert new node
            new_node = _RRTNode(pos=new_pos, parent=best_parent, cost=min_cost, children=[])
            tree.append(new_node)
            new_idx = len(tree) - 1
            tree[best_parent].children.append(new_idx)
            index.insert(new_pos)

            # rewire with subtree cost propagation
            for idx in near_indices:
                if idx == new_idx or idx == best_parent:
                    continue

                node = tree[idx]
                cost_via_new = new_node.cost + self._distance(new_pos, node.pos)

                if cost_via_new < node.cost and self._line_of_sight(new_pos, node.pos):
                    delta = cost_via_new - node.cost

                    old_parent = node.parent
                    if old_parent >= 0:
                        tree[old_parent].children.remove(idx)

                    node.parent = new_idx
                    tree[new_idx].children.append(idx)

                    self._propagate_cost_delta(tree, idx, delta)

            if self._distance(
                new_pos, end_pixel
            ) <= goal_tolerance and self._line_of_sight(new_pos, end_pixel):
                end_cost = new_node.cost + self._distance(new_pos, end_pixel)
                end_node = _RRTNode(pos=end_pixel, parent=new_idx, cost=end_cost, children=[])
                
                tree.append(end_node)
                end_idx = len(tree) - 1

                path_pixel = []
                current = end_idx

                while current != -1:
                    path_pixel.append(tree[current].pos)
                    current = tree[current].parent

                path_pixel.reverse()

                # optimize and convert
                path_pixel = self._greedy_shortcut(path_pixel, max_lookahead=self._greedy_shortcut_max_lookahead)
                path_pixel = self._smooth_path(path_pixel, iterations=self._path_smooth_iter)
                path_world = [self._pixel_to_world(p) for p in path_pixel]

                return path_world

        return None

    @aparsedata(SLAMOffsetMap)
    async def on_slam_map(self, map):
        """
        map: SLAMOffsetMap
        (see v2bot/src/*_slam/src/source/datatypes.py)
        """

        # "grid": NumpyArray,
        # "width": Int,
        # "height": Int,
        # "offset_x": Int,
        # "offset_y": Int,
        # "resolution": Float,
        self.grid = map.grid
        self.width = map.width
        self.height = map.height
        self.offset_x = map.offset_x
        self.offset_y = map.offset_y
        self.resolution = map.resolution

    @aparsedata(TimedMovement3DoF)
    async def on_slam_pose(self, pose):
        self.start_pos = (pose.movement.x, pose.movement.y)

    @aparsedata(Vector)
    async def on_goalmanager_currentgoal(self, goal: Vector):
        self.end_pos = (goal.x, goal.y)


async def main():
    client = PathPlanner()

    rebuild_check_time = get_config("path_planner.miniros.rebuild_check_time")
    max_no_rebuild_time = get_config("path_planner.miniros.max_no_rebuild_time")

    min_distance_from_start_to_goal = get_config(
        "path_planner.pathfinder.min_distance_to_rebuild"
    )
    min_distance_between_goals = get_config(
        "path_planner.pathfinder.min_distance_between_goals"
    )

    max_iter = get_config("path_planner.pathfinder.max_iter")
    step_size = get_config("path_planner.pathfinder.step_size_px")
    goal_tolerance = get_config("path_planner.pathfinder.goal_tolerance_px")
    search_radius_factor = get_config("path_planner.pathfinder.search_radius_factor")

    async def run():
        await client.wait()

        path_topic = await client.topic("globalpath", NumpyArray)

        prev_goal = None
        k = 0
        failed = 0
        
        while True:
            await asyncio.sleep(rebuild_check_time)
            k += 1

            needs_rebuild = client.end_pos is not None and (
                prev_goal is None
                or (
                    prev_goal is not None
                    and client._distance(client.end_pos, prev_goal)
                    > min_distance_between_goals
                )
                or (
                    k >= max_no_rebuild_time / rebuild_check_time
                    and client.start_pos is not None
                    and client._distance(client.start_pos, client.end_pos)
                    > min_distance_from_start_to_goal
                )
            )

            if needs_rebuild:
                build_start = time.monotonic()
                path = await asyncio.to_thread(
                    client.find_path,
                    max_iter=max_iter,
                    step_size=step_size,
                    goal_tolerance=goal_tolerance,
                    search_radius_factor=search_radius_factor,
                    allow_unknown=False,
                )
                build_end = time.monotonic()

                if path is None:
                    build_start = time.monotonic()
                    path = await asyncio.to_thread(
                        client.find_path,
                        max_iter=max_iter,
                        step_size=step_size,
                        goal_tolerance=goal_tolerance,
                        search_radius_factor=search_radius_factor,
                        allow_unknown=True
                    )
                    build_end = time.monotonic()

                k = 0

                if path is None:
                    failed += 1
                    
                    k = 30
                    if failed >= 3:
                        await path_topic.post([])
                        failed = 0
                    
                    print(f"[] Failed path in {build_end - build_start} seconds")

                else:
                    failed = 0
                    
                    await path_topic.post(np.asarray(path))
                    print(f"[] Built path in {build_end - build_start} seconds")

                prev_goal = client.end_pos

    await asyncio.gather(client.run(), run())


if __name__ == "__main__":
    asyncio.run(main())
