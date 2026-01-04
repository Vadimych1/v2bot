import numpy as np
from miniros_constants.main import PX_PER_METER


def normalize_angle(angle: float):
    while angle > np.pi:
        angle -= 2 * np.pi

    while angle < -np.pi:
        angle += 2 * np.pi

    return angle


# TODO: fix
class PathTracker:
    def __init__(
        self, max_linear_speed: float, max_angular_speed: float, target_radius: float = 0.3
    ):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.target_radius = target_radius
        self.path = []

    def get_speeds(self, pose: np.ndarray) -> np.ndarray:
        raise NotImplemented

    def is_goal_reached(self, current_pose, goal_threshold=0.1):
        if len(self.path) == 0:
            return True

        current_pos = np.array([current_pose[0], current_pose[1]])
        goal_pos = self.path[-1]

        return np.linalg.norm(goal_pos - current_pos) < self.target_radius


class PurePursuit(PathTracker):
    def __init__(
        self,
        max_linear_speed: float,
        max_angular_speed: float,
        target_radius: float = 0.3,
        lookahead_distance: float = PX_PER_METER * 3,
    ):
        super().__init__(max_linear_speed, max_angular_speed, target_radius)

        self.lookahead_distance = lookahead_distance
        self.current_waypoint_idx = 0

    def set_path(self, path: np.ndarray):
        self.path = path
        self.current_waypoint_idx = 0

    def find_lookahead_point(self, current_pose: np.ndarray):
        if len(self.path) == 0:
            return None

        current_pos = current_pose[:2]

        min_dist = float("inf")
        min_idx = self.current_waypoint_idx

        for i in range(self.current_waypoint_idx, len(self.path)):
            dist = np.linalg.norm(self.path[i] - current_pos)

            if dist < min_dist:
                min_dist = dist
                min_idx = i

        self.current_waypoint_idx = min_idx

        for i in range(self.current_waypoint_idx, len(self.path) - 1):
            segment_start = self.path[i]
            segment_end = self.path[i + 1]

            d = segment_end - segment_start
            f = segment_start - current_pos

            a = np.dot(d, d)
            b = 2 * np.dot(f, d)
            c = np.dot(f, f) - self.lookahead_distance**2

            disc = b**2 - 4 * a * c

            if disc >= 0:
                t1 = (-b + disc**0.5) / (2 * a)
                t2 = (-b - disc**0.5) / (2 * a)

                for t in [t1, t2]:
                    if 0 <= t <= 1:
                        lookahead_point = segment_start + t * d
                        self.current_waypoint_idx = i
                        return lookahead_point

        return self.path[-1]

    def get_speeds(self, pose: np.ndarray):
        if len(self.path) == 0:
            return 0.0, 0.0

        lookahead_point = self.find_lookahead_point(pose)
        if lookahead_point is None:
            return 0.0, 0.0

        x, y, theta = pose
        lookahead_x, lookahead_y = lookahead_point

        target_angle = np.atan2(lookahead_y - y, lookahead_x - x)

        angle_delta = normalize_angle(target_angle - theta)
        angular_vel = angle_delta * 2.0

        angular_vel = np.clip(
            angular_vel, -self.max_angular_speed, self.max_angular_speed
        )

        linear_vel = self.max_linear_speed * (1 - abs(angle_delta) / np.pi)
        linear_vel = max(0, linear_vel)

        return linear_vel, angular_vel


class PID(PathTracker):
    def __init__(
        self,
        max_linear_speed,
        max_angular_speed,
        kp_ang,
        ki_ang,
        kd_ang,
        kp_lin,
        target_radius=0.3,
    ):
        super().__init__(max_linear_speed, max_angular_speed, target_radius)

        self.kp_ang = kp_ang
        self.ki_ang = ki_ang
        self.kd_ang = kd_ang

        self.kp_lin = kp_lin

        self._prev_angular_error = 0
        self.i_angular = 0

        self.current_waypoint = 0

    def set_path(self, path: np.ndarray):
        self.path = path

    def get_speeds(self, pose: np.ndarray) -> np.ndarray:
        if self.path is None or self.current_waypoint >= len(self.path):
            return 0.0, 0.0

        cx, cy, ctheta = pose
        tx, ty = self.path[self.current_waypoint]

        distance = np.linalg.norm((tx - cx, ty - cy))

        if distance <= self.target_radius:
            self.current_waypoint += 1

            if self.current_waypoint >= len(self.path):
                return 0.0, 0.0

            tx, ty = self.path[self.current_waypoint]
            distance = np.linalg.norm((tx - cx, ty - cy))

        ttheta = np.atan2(ty - cy, tx - cx)
        angular_error = normalize_angle(ttheta - ctheta)

        self.i_angular += angular_error
        dtheta = angular_error - self._prev_angular_error

        angular_correction = (
            self.kp_ang * angular_error
            + self.ki_ang * self.i_angular
            + self.kd_ang * dtheta
        )

        angular_correction = np.clip(
            angular_correction, -self.max_angular_speed, self.max_angular_speed
        )

        self._prev_angular_error = angular_error

        # lin_speed = max(self.max_linear_speed / 4, self.kp_lin * distance)
        lin_speed = self.max_linear_speed * 0.9
        lin_speed = np.clip(lin_speed, -self.max_linear_speed, self.max_linear_speed)

        speed_reduction = 1.0 - min(abs(angular_correction) / np.pi, 0.5)
        lin_speed *= speed_reduction

        return lin_speed, angular_correction


# CurrentPathTracker = PID
CurrentPathTracker = PurePursuit

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # controller = CurrentPathTracker(
    #     lookahead_distance=3, max_linear_speed=0.3, max_angular_speed=0.7
    # )

    controller = CurrentPathTracker(
        max_linear_speed=0.3,
        max_angular_speed=0.7,
        kp_ang=1.5,
        ki_ang=0.01,
        kd_ang=0.3,
        kp_lin=0.8,
    )

    path = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0], [3.0, 3.0], [4.0, 4.0]])

    controller.set_path(path)
    current_pose = [0.0, 0.0, np.pi]  # [x, y, theta]

    fig = plt.figure(figsize=(10, 10))

    for dot in path:
        plt.plot(*dot, "go", markersize=10)

    poses = []
    for _ in range(400):
        if controller.is_goal_reached(current_pose):
            print("Цель достигнута!")
            break

        linear_vel, angular_vel = controller.get_speeds(current_pose)

        dt = 0.1
        x, y, theta = current_pose

        x += linear_vel * np.cos(theta) * dt
        y += linear_vel * np.sin(theta) * dt
        theta += angular_vel * dt

        current_pose = [x, y, theta]

        poses.append(current_pose)

    for i in range(1, len(poses)):
        plt.plot(poses[i - 1][:2], poses[i][:2], "r-")
        x = poses[i - 1][2]
        plt.plot(
            poses[i - 1][:2],
            [poses[i - 1][0] + np.sin(x), poses[i - 1][1] + np.cos(x)],
            "b-",
        )

    plt.title("RRT* Path Tracking")
    plt.grid(False)
    plt.show()
