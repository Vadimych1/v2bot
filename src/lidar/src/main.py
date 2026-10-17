import pyrplidarsdk
import time

import platform
import asyncio
import signal
import multiprocessing as mp
from queue import Empty

from miniros import AsyncROSClient, datatypes
from miniros_configurator import get_config


def scan_worker(port: str, baudrate: int, running, queue: mp.Queue):
    lidar = pyrplidarsdk.RplidarDriver(
        port=port,
        baudrate=baudrate,
    )
    
    if not lidar.connect():
        print("[] failed to connect", flush=True)
        exit(1)
    
    lidar.start_scan()

    while running.is_set():
        dat = lidar.get_scan_data()
        
        if dat is not None:
            angles, distances, quality = dat
            timestamp = time.time()
            
            try:
                if queue.full():
                    try:
                        queue.get_nowait()
                    except Empty:
                        pass
                
                queue.put_nowait((angles, distances, timestamp))
            
            except Exception:
                pass
        
        else:
            print("[] null scan", flush=True)
    
    lidar.stop_scan()
    lidar.disconnect()


class LidarClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("lidar", ip, port)

        win_port = get_config("lidar.win_port")
        ldr_port = get_config("lidar.port")
        self.port = win_port if platform.system() == "Windows" else ldr_port
        self.baudrate = get_config("lidar.baudrate")

        self.lidar_proc = mp.Process(target=scan_worker)

        self.last_ping_time = time.time()
        self.lidar_queue = mp.Queue(10)

        self.running = mp.Event()

    def scans_job(self):
        if self.lidar_proc is not None and self.lidar_proc.is_alive():
            return
        
        self.running.set()
        self.lidar_proc = mp.Process(target=scan_worker, args=(self.port, self.baudrate, self.running, self.lidar_queue))
        self.lidar_proc.start()
        
    async def get_scan_async(self):
        loop = asyncio.get_running_loop()
        
        try:
            data = await loop.run_in_executor(None, self.lidar_queue.get, True, 3)
            return data
        except Empty:
            return None

    async def on_ping(self, _, node):
        self.last_ping_time = time.time()


async def main():
    nq_start_time = time.time()

    client = LidarClient()
    client.scans_job()

    def shutdown(sig, frame):
        client.running.clear()

    async def run():
        nq = 0
        await client.wait()

        ldr_topic = await client.topic("lidar", datatypes.Lidar2D)

        while client.running.is_set():
            nq += 1
            
            lidar = await client.get_scan_async()

            if lidar is None:
                continue

            angles, distances, ts = lidar

            await ldr_topic.post(
                datatypes.Lidar2D(
                    distances=distances,
                    angles=angles,
                    timestamp=ts,
                )
            )
            
            if nq % 30 == 0:
                print(f"Running at {nq / (time.time() - nq_start_time)}Hz")

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        await asyncio.gather(
            client.run(),
            run(),
        )

    except (KeyboardInterrupt, asyncio.QueueShutDown):
        pass


if __name__ == "__main__":
    asyncio.run(main())
