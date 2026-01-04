import numpy as np
from scipy.spatial import KDTree
from scipy.ndimage import binary_dilation, label, center_of_mass
from skimage import measure
from miniros_constants.main import ROBOT_SIZE_PX
from cv2 import resize, INTER_NEAREST


def dilate_map(
    map: np.ndarray, divider: float = 4, robot_size_px: int = 8
) -> np.ndarray:
    obstacle_mask = map == 0
    y, x = np.ogrid[
        -robot_size_px * 2 : robot_size_px * 2 + 1,
        -robot_size_px * 2 : robot_size_px * 2 + 1,
    ]
    kernel = (x * x + y * y <= (2 * robot_size_px) ** 2).astype(np.uint8)
    dilated = binary_dilation(obstacle_mask, structure=kernel)

    return (
        resize(
            dilated.astype(int) * 255,
            (np.array(dilated.shape) / divider).astype(int),
            interpolation=INTER_NEAREST,
        )
    ) == 255


def find_obstacles(map: np.ndarray, min_size: int = 30) -> np.ndarray:
    map = map == 0

    labeled_arr, num_features = label(map)
    obstacles = []

    for i in range(num_features):
        obstacle_mask = labeled_arr == i

        if np.sum(obstacle_mask) >= min_size:
            center = center_of_mass(obstacle_mask)

            indices = np.where(obstacle_mask)
            min_y, min_x = np.min(indices[0]), np.min(indices[1])
            max_y, max_x = np.max(indices[0]), np.max(indices[1])

            width = max_x - min_x
            height = max_y - min_y
            radius = max(width, height) / 2

            if radius > min_size * 10:
                continue

            # Конвертируем координаты
            center_x = center[1]
            center_y = center[0]

            obstacles.append([center_x, center_y, radius])

    return np.array(obstacles)

    # labeled_map = measure.label(map, connectivity=2)
    # regions = measure.regionprops(labeled_map)

    # obstacles = []
    # for region in regions:
    #     # Get centroid (y, x) - note: regionprops returns (row, col)
    #     y, x = region.centroid

    #     # Calculate equivalent radius from area
    #     area = region.area
    #     radius = np.sqrt(area / np.pi)

    #     # Get bounding box for more precise radius calculation
    #     min_row, min_col, max_row, max_col = region.bbox
    #     width = max_col - min_col
    #     height = max_row - min_row
    #     radius = max(width, height) / 2

    #     obstacles.append((x, y, radius))

    # return obstacles


class PathPlanner:
    def __init__(self):
        self.map: np.ndarray = None
        self.pos: np.ndarray = None
        self.goal: np.ndarray = None

    def build_path(self) -> list[tuple[float, float]]:
        raise NotImplemented

    def update_path(self, path: list[tuple[float, float]]) -> list[tuple[float, float]]:
        raise NotImplemented


class _RRTNode:
    def __init__(self, position):
        self.position = position
        self.parent = None
        self.cost = 0.0


class RRTStar(PathPlanner):
    def __init__(self):
        super().__init__()

        self.max_iter = 15000
        self.step_size = 10
        self.radius = 20

    def _check_collision(self, p1, p2, map):
        x1, y1 = p1
        x2, y2 = p2

        points = self._br_line(x1, y1, x2, y2)

        for x, y in points:
            if map[y, x] >= 1:
                return True

        return False

    def _br_line(self, x1, y1, x2, y2):
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1

        err = dx - dy

        while True:
            yield (x1, y1)

            if x1 == x2 and y1 == y2:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x1 += sx

            if e2 < dx:
                err += dx
                y1 += sy

    def _reconstruct_path(self, node: _RRTNode):
        path = []

        while node is not None:
            path.append(node.position)
            node = node.parent

        return np.array(path[::-1]) * 4

    def build_path(self) -> tuple[list[_RRTNode], _RRTNode, list[tuple[float, float]]]:
        if self.map is None or self.pos is None or self.goal is None:
            raise ValueError("Map, pos or goal are not presented")

        map = dilate_map(self.map, 4.0)

        h, w = map.shape
        start, end = (self.pos / 4).astype(int), (self.goal / 4).astype(int)
        nodes = [_RRTNode(start)]

        for _ in range(self.max_iter):
            if np.random.random() < 0.05:
                rand_point = end

            else:
                rand_point = np.array(
                    [np.random.randint(0, w), np.random.randint(0, h)]
                )

            kdtree = KDTree([node.position for node in nodes])
            _, idx = kdtree.query(rand_point)
            nearest_node = nodes[idx]

            direction = np.arctan2(*(rand_point - nearest_node.position))

            new_x = int(nearest_node.position[0] + self.step_size * np.cos(direction))
            new_y = int(nearest_node.position[1] + self.step_size * np.sin(direction))
            new_pos = np.array([new_x, new_y])

            if (
                0 <= new_x < w
                and 0 <= new_y < h
                and not self._check_collision(nearest_node.position, new_pos, map)
            ):
                new_node = _RRTNode(new_pos)
                neighbor_idxs = kdtree.query_ball_point(new_pos, self.radius)
                neighbors: list[_RRTNode] = [nodes[i] for i in neighbor_idxs]

                min_cost = nearest_node.cost + self.step_size
                best_parent = nearest_node

                for neighbor in neighbors:
                    if (
                        neighbor.cost + self.step_size < min_cost
                        and not self._check_collision(neighbor.position, new_pos, map)
                    ):
                        min_cost = neighbor.cost
                        best_parent = neighbor

                new_node.parent = best_parent
                new_node.cost = min_cost
                nodes.append(new_node)

                for neighbor in neighbors:
                    if (
                        new_node.cost + self.step_size < neighbor.cost
                        and not self._check_collision(
                            new_node.position, neighbor.position, map
                        )
                    ):
                        neighbor.parent = new_node
                        neighbor.cost = new_node.cost + self.step_size

                if np.linalg.norm(new_pos - end) < self.step_size:
                    end_node = _RRTNode(self.goal / 4)
                    end_node.parent = new_node
                    end_node.cost = new_node.cost + self.step_size

                    return nodes, end_node, map

        return nodes, None, map


class TimedElasticBand(PathPlanner):
    def __init__(self):
        super().__init__()

        self.max_iter = 100
        self.learning_rate = 0.01
        self.horizon = 25
        self.dt = 0.4
        self.obstacles = []

    def update_path(
        self, gn_path: list[tuple[float, float]]
    ) -> list[tuple[float, float]]:
        g_path = gn_path

        teb_band = self._init_teb(g_path, self.pos)

        for iteration in range(self.max_iter):
            total_cost = 0.0
            gradient = np.zeros_like(teb_band)

            for i in range(len(teb_band)):
                path_cost, path_grad = self._path_following_cost(teb_band[i], g_path)
                obstacle_cost, obstacle_grad = self._obstacle_avoidance_cost(
                    teb_band[i]
                )

                if i > 0:
                    kinem_cost, kinem_grad_1, kinem_grad_2 = self._kinematics_cost(
                        teb_band[i - 1], teb_band[i]
                    )
                    total_cost += kinem_cost
                    gradient[i - 1] += kinem_grad_1
                    gradient[i] += kinem_grad_2

                if i > 1:
                    smooth_cost, smooth_grad = self._smoothness_cost(
                        teb_band[i - 2 : i + 1]
                    )
                    total_cost += smooth_cost
                    gradient[i - 2 : i + 1] += smooth_grad

                total_cost += path_cost + obstacle_cost
                gradient[i] += path_grad + obstacle_grad 

            teb_band -= self.learning_rate * gradient

            if np.linalg.norm(gradient) < 1e-3:
                break

        return [x[:2] for x in teb_band] + [g_path[-1]]

    def _init_teb(self, global_path, current_pose):
        distances = [np.linalg.norm(point[:2] - current_pose) for point in global_path]
        start_idx = np.argmin(distances)
        end_idx = min(start_idx + self.horizon, len(global_path) - 1)

        teb_band = []
        for i in range(start_idx, end_idx):
            point = global_path[i]
            timed = np.array([point[0], point[1], 0, i * self.dt])
            teb_band.append(timed)

        return np.array(teb_band)

    def _path_following_cost(self, current_pose, global_path):
        distances = [
            np.linalg.norm(point[:2] - current_pose[:2]) for point in global_path
        ]

        min_idx = np.argmin(distances)
        target = global_path[min_idx]

        error = current_pose[:2] - target[:2]
        cost = 0.5 * np.dot(error, error)

        gradient = np.zeros_like(current_pose)
        gradient[:2] = error

        return cost, gradient

    # TODO: change obstacle detection logic
    def _obstacle_avoidance_cost(self, current_pose):
        cost = 0.0
        gradient = np.zeros_like(current_pose)

        for obstacle in self.obstacles:
            dist = np.linalg.norm(current_pose[:2] - obstacle[:2])
            safe_distance = obstacle[2] * 0.7 - ROBOT_SIZE_PX
 
            if dist < safe_distance:
                repulsive_force = 1.0 / max(dist, 0.1) - 1.0 / safe_distance
                cost += 0.5 * repulsive_force**2
 
                direction = (current_pose[:2] - obstacle[:2]) / max(dist, 1e-5)
                gradient[:2] += repulsive_force * direction / (max(dist, 0.1) ** 2)

        return cost, gradient

    def _kinematics_cost(self, p1, p2):
        pos_diff = p2[:2] - p1[:2]
        theta_diff = p2[2] - p1[2]
        distance = np.linalg.norm(pos_diff)
        
        while theta_diff > np.pi:
            theta_diff -= 2 * np.pi
            
        while theta_diff < -np.pi:
            theta_diff += 2 * np.pi

        time_diff = distance / 4.0
        
        curvature_cost = 0.0
        if distance > 1e-6:
            curvature = abs(theta_diff) / distance
            curvature_cost = 0.5 * curvature**2

        orientation_cost = 0.1 * theta_diff**2
        
        time_cost = 0.05 * time_diff**2
        
        cost = curvature_cost + orientation_cost + time_cost
        
        grad_p1 = np.zeros_like(p1)
        grad_p2 = np.zeros_like(p2)
        
        if distance > 1e-6:
            dcurvature_dtheta_diff = theta_diff / (distance**2) if theta_diff >= 0 else -theta_diff / (distance**2)
            dcurvature_ddistance = -abs(theta_diff) / (distance**2)
            
            grad_p1[2] += curvature * dcurvature_dtheta_diff * (-1)
            grad_p2[2] += curvature * dcurvature_dtheta_diff * (1)
            
            if distance > 0:
                ddist_dp1 = -pos_diff / distance
                ddist_dp2 = pos_diff / distance
                
                grad_p1[:2] += curvature * dcurvature_ddistance * ddist_dp1
                grad_p2[:2] += curvature * dcurvature_ddistance * ddist_dp2
        
        grad_p1[2] += 0.2 * theta_diff * (-1)
        grad_p2[2] += 0.2 * theta_diff * (1)
        
        if distance > 1e-6:
            dtime_ddistance = 1.0 / 4.0
            dtime_cost_ddistance = 0.1 * time_diff * dtime_ddistance
            
            grad_p1[:2] += dtime_cost_ddistance * (-pos_diff / distance)
            grad_p2[:2] += dtime_cost_ddistance * (pos_diff / distance)
        
        return cost, grad_p1, grad_p2

    def _smoothness_cost(self, poses):
        if len(poses) < 3:
            return 0.0, np.zeros_like(poses)

        accel = poses[0] - 2 * poses[1] + poses[2]
        cost = 0.5 * np.dot(accel, accel)

        gradient = np.zeros_like(poses)
        gradient[0] = accel
        gradient[1] = -2 * accel
        gradient[2] = accel

        return cost, gradient


## TODO: add A* algo
## TODO: add other local planners

# export
GlobalPathPlanner = RRTStar
LocalPathPlanner = TimedElasticBand

# test
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    planner = GlobalPathPlanner()

    map_size = (1000, 1000)
    obstacle_map = np.zeros(map_size, dtype=np.uint8)

    obstacle_map[400:600, 200:400] = 1
    obstacle_map[100:300, 500:800] = 1
    obstacle_map[700:900, 600:800] = 1

    start = (100, 100)
    goal = (900, 900)

    planner.pos = np.array(start)
    planner.goal = np.array(goal)
    planner.map = obstacle_map
    nodes, final_node, d_map = planner.build_path()

    if final_node:
        path = planner._reconstruct_path(final_node)
    else:
        path = []

    local_planner = LocalPathPlanner()
    local_planner.pos = np.array(start)
    local_planner.obstacles = find_obstacles(d_map)

    updated = local_planner.update_path(path)

    fig = plt.figure(figsize=(10, 10))
    plt.imshow(obstacle_map, cmap="gray", origin="lower")
    # plt.imshow(d_map, cmap="gray", origin="lower")
    plt.plot(start[0], start[1], "go", markersize=10)
    plt.plot(goal[0], goal[1], "ro", markersize=10)

    for node in nodes:
        if node.parent:
            plt.plot(
                [node.position[0] * 4, node.parent.position[0] * 4],
                [node.position[1] * 4, node.parent.position[1] * 4],
                "b-",
                alpha=1,
            )

    plt.plot(
        [x for x, _, _, _ in updated], [y for _, y, _, _ in updated], "r-", linewidth=2
    )
    plt.plot([x for x, _ in path], [y for _, y in path], "g-", linewidth=2)

    plt.title("RRT* Path Planning")
    plt.grid(False)
    plt.show()
