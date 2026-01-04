"""
Vadimych1, 2025

This demo shows work of a pathfinding algorithm and than
simulates robot movement based on PurePursuit or PID
path tracking. You can change which algorithm to use
by commenting/uncommenting lines 18-27 and 29-35
"""

from source.pathfinding import GlobalPathPlanner, LocalPathPlanner, find_obstacles
from source.pathtracking import PurePursuit, PID
import matplotlib.pyplot as plt
import numpy as np

global_pp = GlobalPathPlanner()
local_pp = LocalPathPlanner()

## PID is sometimes slow but very accurate
# pt = PID(
#     max_linear_speed=40.0,
#     max_angular_speed=1.0,
#     kp_ang=1.5,
#     ki_ang=0.01,
#     kd_ang=0.3,
#     kp_lin=0.8,
#     target_radius=7,
# )

## PurePursuit gives better time perfomance (constant speed with path smoothing)
pt = PurePursuit(
    max_linear_speed=40.0,
    max_angular_speed=1.0,
    lookahead_distance=50.0,
    target_radius=7.0,
)

map_size = (1000, 1000)
obstacle_map = np.ones(map_size, dtype=np.uint8)

obstacle_map[400:600, 200:400] = 0
obstacle_map[100:300, 500:800] = 0
obstacle_map[700:900, 600:800] = 0

start = (100, 100)
goal = (900, 900)

global_pp.pos = np.array(start)
global_pp.goal = np.array(goal)
global_pp.map = obstacle_map
nodes, final_node, d_map = global_pp.build_path()

if final_node:
    path = global_pp._reconstruct_path(final_node)
else:
    path = []

local_pp.pos = np.array(start)
local_pp.obstacles = find_obstacles(obstacle_map)
updated = local_pp.update_path(path)

fig = plt.figure(figsize=(10, 10))
plt.imshow(obstacle_map, cmap="gray", origin="lower")
plt.plot(start[0], start[1], "go", markersize=10)
plt.plot(goal[0], goal[1], "ro", markersize=10)

for node in nodes:
    if node.parent is not None:
        plt.plot(
            [node.position[0] * 4, node.parent.position[0] * 4],
            [node.position[1] * 4, node.parent.position[1] * 4],
            "b-",
            alpha=1,
        )

plt.plot([x for x, _ in updated], [y for _, y in updated], "r-", linewidth=2)
plt.plot([x for x, _ in path], [y for _, y in path], "g-", linewidth=2)

print(local_pp.obstacles)
for obstacle in local_pp.obstacles:
    x, y, r = obstacle

    theta = np.linspace(0, 2 * np.pi, 100)
    a = r * np.cos(theta) + x
    b = r * np.sin(theta) + y

    plt.plot(a, b)

plt.title("RRT* Path Planning")
plt.grid(False)
plt.show()

plt.ion()

pt.set_path(updated)
current_pose = np.array([100.0, 100.0, -np.pi])
fig2, ax = plt.subplots()
(line,) = ax.plot(*start, "b-", linewidth=5)
(line2,) = ax.plot([0, 0], [0, 0], "r-")

ax.imshow(obstacle_map, cmap="gray", origin="lower")
for dot in updated:
    ax.plot(*dot, "go", markersize=2)


x_dots = []
y_dots = []
for i in range(2000):
    if pt.is_goal_reached(current_pose):
        print("Goal reached")
        fig2.canvas.draw()
        break

    linear_vel, angular_vel = pt.get_speeds(current_pose)

    dt = 0.1
    x, y, theta = current_pose

    x += linear_vel * np.cos(theta) * dt
    y += linear_vel * np.sin(theta) * dt
    theta += angular_vel * dt

    current_pose = [x, y, theta]

    x_dots.append(x)
    y_dots.append(y)

    line.set_xdata(x_dots)
    line.set_ydata(y_dots)

    line2.set_xdata([x, x + 100 * np.cos(theta)])
    line2.set_ydata([y, y + 100 * np.sin(theta)])
    
    fig2.canvas.draw()
    plt.pause(0.002)

plt.ioff()
plt.show()
