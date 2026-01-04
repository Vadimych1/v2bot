import time
import asyncio
import numpy as np
from miniros import AsyncROSClient
from miniros.util.decorators import aparsedata
from miniros_slam.source.datatypes import SLAMMap
from miniros.util.datatypes import Vector, NumpyArray
from miniros_algorithms.source.pathfinding import GlobalPathPlanner as GPPAlgo


class PathPlanner(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("path_planner", ip, port, _parse_handlers)

        self.map = None
        self.pose = None
        self.current_goal = Vector(0, 0, 0)

        self.path_planner = GPPAlgo()
        self.planned_path = np.array([])

    @aparsedata(SLAMMap)
    def on_slam_map(self, map: SLAMMap):
        self.map = map.to_numpy(int(len(map.data) ** 0.5))  # map is a square

    @aparsedata(Vector)
    def on_slam_pose(self, pose: Vector):
        self.pose = pose.pos_to_numpy()

    @aparsedata
    def on_goalmanager_currentgoal(self, goal: Vector):
        # TODO: adaptive distance
        if np.linalg.norm([goal.x - self.pose[0], goal.y - self.pose[1]]) > 0.3:
            self.current_goal = goal


async def main():
    client = PathPlanner()

    async def run():
        await client.wait()

        path_topic = await client.topic("globalpath", NumpyArray)

        prev_goal = Vector(0, 0, 0)
        k = 0
        while True:
            await asyncio.sleep(0.1)
            k += 1

            # rebuild path only when:
            # 1) goal changed too much TODO: change 5 to better max distance
            # 2) every 17 seconds TODO: change this value to more real-like
            needs_rebuild = (client.current_goal - prev_goal).norm() > 5 or k >= 170

            if needs_rebuild and client.map != None and client.pose != None:
                build_start = time.time()

                client.path_planner.map = client.map
                client.path_planner.goal = client.current_goal
                client.path_planner.pos = client.pose[2:]
                
                prev_goal = client.current_goal

                nodes, end_node, dilated_map = (
                    client.path_planner.build_path()
                )  # TODO: save graph and map?
                path = client.path_planner._reconstruct_path(end_node)

                client.planned_path = path

                build_end = time.time()

                k = max(0, (build_end - build_start) / 0.1 - 50)

                await path_topic.post(path)

    await asyncio.gather(client.run(), run())


asyncio.run(main())
