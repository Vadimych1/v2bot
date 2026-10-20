"""Standalone RRT* test for the PathPlanner class.

Builds a synthetic occupancy grid with rectangular obstacles, runs
PathPlanner.find_path(...) against it, and dumps the result to PNG.

Run from a directory where `from miniros_pathplanner import PathPlanner`
resolves — i.e. the same environment you launch the real node from.
"""

import time
import numpy as np
import cv2 as cv

from miniros_path_planner.main import PathPlanner


# --------------------------------------------------------------------------- #
# Planner construction (no ROS, no config server)
# --------------------------------------------------------------------------- #

def build_test_planner(seed: int = 42) -> PathPlanner:
    """
    Build a PathPlanner whose fields are set directly.

    PathPlanner.__init__ is bypassed because it (a) calls get_config(...)
    which needs miniros_configurator, and (b) calls AsyncROSClient.__init__
    which needs a live MiniROS server. find_path() only reads the fields
    populated below, so this is safe.
    """
    np.random.seed(seed)

    planner = object.__new__(PathPlanner)

    # ---- map geometry ----
    width, height = 300, 300
    resolution = 0.02          # m / pixel
    offset_x, offset_y = 150, 150   # pixel that maps to world (0, 0)

    # values: 100 = free, 0 = obstacle (any value below threshold is obstacle)
    grid = np.full((height, width), 100, dtype=np.int16)

    # Rectangular obstacles: grid[row_start:row_end, col_start:col_end] = 0
    grid[100:120,  50:300] = 0      # upper horizontal wall
    grid[200:220, 100:230] = 0      # lower horizontal wall
    grid[220:300, 100:120] = 0

    world_half = (min(width, height) * resolution) / 2.0   # 3.0 m for 300 px @ 0.02

    # ---- fields required by find_path ----
    planner.robot_radius_m = 0.15
    planner.grid = grid
    planner.dilated_grid = None
    planner.width = width
    planner.height = height
    planner.d_width = width        # set again inside find_path, harmless
    planner.d_height = height
    planner.offset_x = offset_x
    planner.offset_y = offset_y
    planner.resolution = resolution

    # world coords, meters: start bottom-left, goal top-right
    planner.start_pos = (-2.0, -2.0)
    planner.end_pos   = ( 2.0,  2.0)

    planner.min_x = -world_half
    planner.max_x =  world_half
    planner.min_y = -world_half
    planner.max_y =  world_half

    planner._obstacle_upper_threshold = 50
    planner._greedy_shortcut_max_lookahead = 15

    return planner


# --------------------------------------------------------------------------- #
# Visualization
# --------------------------------------------------------------------------- #

def visualize(
    planner: PathPlanner,
    path: list[tuple[float, float]] | None,
    output_path: str,
    upscale: int = 2,
) -> None:
    """Render obstacles, safety margin, start, goal, and (if any) the path."""
    h, w = planner.grid.shape
    threshold = planner._obstacle_upper_threshold

    # BGR image, all white (free space)
    img = np.full((h, w, 3), 255, dtype=np.uint8)

    # obstacles -> black
    img[planner.grid < threshold] = (0, 0, 0)

    # dilated safety margin -> light gray
    if planner.dilated_grid is not None:
        safety = (planner.dilated_grid < 40) & (planner.grid >= threshold)
        img[safety] = (200, 200, 200)

    # pixel helper (same math as PathPlanner._world_to_pixel)
    def w2p(point: tuple[float, float]) -> tuple[int, int]:
        wx, wy = point
        px = int(round(wx / planner.resolution + planner.offset_x))
        py = int(round(wy / planner.resolution + planner.offset_y))
        return px, py

    # path -> orange polyline
    if path is not None and len(path) >= 2:
        pts = np.array([w2p(p) for p in path], dtype=np.int32)
        for i in range(len(pts) - 1):
            cv.line(img, tuple(pts[i]), tuple(pts[i + 1]),
                    (0, 165, 255), 2, cv.LINE_AA)
        for x, y in pts:
            cv.circle(img, (int(x), int(y)), 2, (0, 165, 255), -1)

    # start (green), goal (red)
    sx, sy = w2p(planner.start_pos)
    gx, gy = w2p(planner.end_pos)
    cv.circle(img, (sx, sy), 6, (0, 200, 0), -1, cv.LINE_AA)
    cv.circle(img, (gx, gy), 6, (0, 0, 255), -1, cv.LINE_AA)

    # annotate outcome
    label = "path found" if path is not None else "NO PATH"
    color = (0, 128, 0) if path is not None else (0, 0, 200)
    cv.putText(img, label, (8, 20), cv.FONT_HERSHEY_SIMPLEX,
               0.55, color, 1, cv.LINE_AA)

    if upscale > 1:
        img = cv.resize(img, None, fx=upscale, fy=upscale,
                        interpolation=cv.INTER_NEAREST)

    cv.imwrite(output_path, img)
    print(f"wrote {output_path}")


# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #

def main() -> None:
    planner = build_test_planner()

    t0 = time.perf_counter()
    path = planner.find_path(
        max_iter=3000,
        step_size=8.0,
        goal_tolerance=10.0,
        search_radius_factor=1.0,
    )
    elapsed = time.perf_counter() - t0

    if path is None:
        print(f"no path found in {elapsed:.2f} s")
    else:
        length = sum(
            np.hypot(path[i + 1][0] - path[i][0],
                     path[i + 1][1] - path[i][1])
            for i in range(len(path) - 1)
        )
        print(f"path: {len(path)} points, {length:.3f} m, "
              f"planned in {elapsed:.2f} s")

    visualize(planner, path, "rrt_star_test.png")


if __name__ == "__main__":
    main()