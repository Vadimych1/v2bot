import psutil, time
import asyncio
import base64
import time

from miniros import AsyncROSClient, datatypes, naming, aparsedata
from miniros_configurator import get_config

# !!!!
# latencies = [...]
# p50, p90, p95, p99 = np.percentile(latencies, [50, 90, 95, 99])


stats = {}
last_timestamps = []
class StatsClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("stats", ip, port)
        
        self.add_topic_handler(self.on_slam_map,
            naming.Node.SLAM,
            naming.Topic.SLAM_OCCUPANCY_GRID,
        )
        
        self.add_topic_handler(self.on_slam_pose,
            naming.Node.SLAM,
            naming.Topic.SLAM_POSE,
        )
        
        self.add_topic_handler(self.on_motorcontroller_odometry,
            naming.Node.MOTOR_CONTROLLER,
            naming.Topic.O_POSE,
        )
        
        self.add_topic_handler(self.on_motorcontroller_speeds,
            naming.Node.MOTOR_CONTROLLER,
            naming.Topic.O_VELOCITY,
        )
        
        self.add_topic_handler(self.on_motioncontroller_cmdvel,
            naming.Node.MOTION_CONTROLLER,
            naming.Topic.MC_CMDVEL,
        )
        
        self.add_topic_handler(self.on_pathplanner_path,
            naming.Node.PATH_PLANNER,
            naming.Topic.PP_PATH,
        )
        
        self.add_topic_handler(self.on_lidar_lidar,
            naming.Node.LIDAR,
            naming.Topic.L_SCAN,
        )
        
        self.add_topic_handler(self.on_goalmanager_currentgoal,
            naming.Node.GOAL_MANAGER,
            naming.Topic.GM_GOAL,
        )
    
    def _log_data(self, data: bytes | bytearray, type: str, timestamp: float):
        timestamp = round(timestamp, 2)
        if timestamp not in last_timestamps:
            stats[timestamp] = []
        
        else:
            last_timestamps.append(timestamp)
            if len(last_timestamps) > 20:
                last_timestamps.pop(0)
        
        stats[timestamp].append({
            "type": type,
            "data": base64.encodebytes(data) \
                .decode("utf-8")
        })

    async def on_slam_map(self, map):
        ts = time.time()
        self._log_data(map, "slam_map", ts)
    
    @aparsedata(datatypes.TimedPose3DoF)
    async def on_slam_pose(self, pose):
        ts = pose.timestamp
        self._log_data(
            datatypes.TimedPose3DoF.encode(pose),
            "slam_pose",
            ts,
        )
    
    @aparsedata(datatypes.TimedOdometry3DoF)
    async def on_motorcontroller_odometry(self, odom):
        ts = odom.timestamp
        self._log_data(
            datatypes.TimedOdometry3DoF.encode(odom),
            "odom_pose",
            ts,
        )
    
    async def on_motorcontroller_speeds(self, speeds):
        ts = time.time()
        self._log_data(
            speeds,
            "odom_speeds",
            ts,
        )
    
    async def on_motioncontroller_cmdvel(self, control):
        ts = time.time()
        self._log_data(
            control,
            "cmdvel",
            ts,
        )
    
    async def on_pathplanner_path(self, path):
        ts = time.time()
        self._log_data(
            path,
            "path",
            ts,
        )
    
    @aparsedata(datatypes.Lidar2D)
    async def on_lidar_lidar(self, lidar):
        ts = lidar.timestamp
        self._log_data(
            datatypes.Lidar2D.encode(lidar),
            "lidar",
            ts,
        )
    
    async def on_goalmanager_currentgoal(self, goal):
        ts = time.time()
        self._log_data(
            goal,
            "goal",
            ts,
        )

async def main():
    client = StatsClient()
    
    async def run():
        await client.wait()
        
        miniros_pids = [p.pid for p in psutil.process_iter(['name']) if 'miniros' in p.info['name'].lower()]
        processes = [psutil.Process(p) for p in miniros_pids]
        
        for p in processes:
            p.cpu_percent(interval=None)
        await asyncio.sleep(1)
        
        while True:
            cpu_perc = sum([p.cpu_percent(interval=None) for p in processes])
            mem = sum([p.memory_info().rss / 1024 ** 2 for p in processes])
            ts = time.time()
            
            client._log_data(
                (f"{cpu_perc}__{mem}").encode("utf-8"),
                "cpu_mem",
                ts,
            )
            
            await asyncio.sleep(1)
    
    rp = asyncio.create_task(run())
    
    await asyncio.gather(
        client.run(),
        rp,
    )


if __name__ == "__main__":
    if not get_config("enable_stats"):
        raise Exception("'enable_stats' is set to 'false' in config; enable to collect stats")
    
    try:
        asyncio.run(main())

    except KeyboardInterrupt:
        import json
        
        with open("miniros.bag", "w", -1, "utf-8") as f:
            json.dump(stats, f)
