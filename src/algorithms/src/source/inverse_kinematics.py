import numpy as np


class InverseKinematics:
    def calculate_wheel_speeds(self, v: float, omega: float) -> np.ndarray:
        raise NotImplemented


class TrackedRobotIK(InverseKinematics):
    def __init__(self, track_distance: float, wheel_radius: float):
        super().__init__()

        self.d = track_distance
        self.r = wheel_radius

    def calculate_wheel_speeds(self, v: float, omega: float) -> np.ndarray:
        speeds = np.array([v - omega * self.d / 2, v + omega * self.d / 2])
        speeds /= self.r

        return speeds

CurrentIK = TrackedRobotIK
