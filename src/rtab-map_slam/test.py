import rtabmap_py
import numpy as np
import cv2

            # py::arg("distances"),
            # py::arg("angles"),
            # py::arg("odomX"),
            # py::arg("odomY"),
            # py::arg("odomTheta"),
            # py::arg("timestamp")

slam = rtabmap_py.RtabmapSLAM(0.05, 12)
pos = slam.process(
    np.array([5, 5, 5, 5, 5, 5], dtype=np.float32),
    np.array([0, 1, 2, 3, 4, 5], dtype=np.float32) / 5 * np.pi,
    0, 0, 0,
    0,
)

print(pos)

map = slam.getOccupancyGrid()
cv2.imwrite("out.png", map)