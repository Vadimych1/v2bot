import imufusion
import numpy as np
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import ExtendedKalmanFilter as EKF
from miniros_algorithms.source.kinematics import CurrrentK

class SensorFusion:
    def __init__(self, track_distance: float, wheel_radius: float, ticks_per_rev: int):
        self.ahrs = imufusion.Ahrs()
        self.k = CurrrentK(track_distance, wheel_radius, ticks_per_rev)
        self.ang = 0.0
        
        dim_x = 5
        dim_z = 1
        
        # x y yaw v omega
        self.ekf = EKF(dim_x, dim_z)
        self.ekf.x = np.array([0.0, 10.0, 0.0, 0.0, 0.0])
        
        self.ekf.P = np.eye(dim_x) * 0.1
        self.ekf.P[3,3] = 0.01
        self.ekf.P[4,4] = 0.01
        
        self.ekf.Q[0:2, 0:2] = Q_discrete_white_noise(dim=2, dt=0.1, var=0.1)
        self.ekf.Q[2, 2] = 0.01
        self.ekf.Q[3, 3] = 0.5
        self.ekf.Q[4, 4] = 0.5
        
        self.ekf.R = np.array([[0.05]])
        
        
    def _move(self, x, dt, u):
        x_pos, y_pos, theta, _, _ = x
        v, omega = u
        
        new_x = x_pos + v * np.cos(theta) * dt
        new_y = x_pos + v * np.sin(theta) * dt
        new_theta = theta + omega * dt
        
        new_v = v
        new_omega = omega
        
        return np.array([new_x, new_y, new_theta, new_v, new_omega])
        
    def update_odom(self, left_ticks: int, right_ticks: int, dt: float):
        v, w = self.k.calculate_speeds(left_ticks, right_ticks, dt)
        
        self.ekf.update()
        self.ekf.predict(fx=self._move, dt=dt, u=(v, w))
        
    # merges gyro and acc data to get heading direction
    def update_imu(self, gyro: list[float], acc: list[float], dt: float) -> float:
        self.ahrs.update_no_magnetometer(gyro, acc, dt)
        self.ang = self.ahrs.quaternion.to_euler()[2]
        
        return self.ang
    
f = SensorFusion(0.3, 0.025, 2)
f.update_odom(10, -10, 1)