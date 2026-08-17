# motion_controller package 
## node: motioncontroller

### sends
Topics:
- motorcontroller/cmdvel (Vector: left, right, none)

### receives
Topics:
- slam/map (slam.SLAMMap)
- slam/pose (Vector: x, y, theta)
- lidar/lidar (LidarDatatype)
- pathplanner/globalpath (NumpyArray)