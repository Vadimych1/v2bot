import numpy as np

class Kinematics:
    def calculate_speeds(self, left_ticks: int, right_ticks: int, dt: float) -> np.ndarray:
        raise NotImplemented

class TrackedRobotK(Kinematics):
    def __init__(self, track_distance: float, wheel_radius: float, ticks_per_rev: int):
        super().__init__()
        
        self.track_distance = track_distance
        self.wheel_radius = wheel_radius
        self.ticks_per_rev = ticks_per_rev
    
    def calculate_speeds(self, left_ticks: int, right_ticks: int, dt: float) -> np.ndarray:
        d_left = (left_ticks / self.ticks_per_rev) * (2 * np.pi * self.wheel_radius)
        d_right = (right_ticks / self.ticks_per_rev) * (2 * np.pi * self.wheel_radius)
        
        linear_velocity = (d_left + d_right) / (2.0 * dt)
        angular_velocity = (d_right - d_left) / (self.track_distance * dt)
        
        return np.asarray([linear_velocity, angular_velocity])
    
CurrrentK = TrackedRobotK