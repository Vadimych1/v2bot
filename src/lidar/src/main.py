import pyrplidarsdk
import time

import platform
import asyncio
import signal
import multiprocessing as mp
from queue import Empty

from miniros import AsyncROSClient, datatypes
from miniros_configurator import get_config


def scan_worker(port: str, baudrate: int, running, conn):
    lidar = pyrplidarsdk.RplidarDriver(
        port=port,
        baudrate=baudrate,
    )
    
    if not lidar.connect():
        raise Exception("[] failed to connect")
    
    lidar.start_scan()
    
    try:        
        while running.is_set():
            dat = lidar.get_scan_data()
            
            if dat is not None:
                angles, distances, _ = dat # _ is quality
                timestamp = time.time()
                
                payload = datatypes.Lidar2D.encode(
                    datatypes.Lidar2D(
                        distances=distances,
                        angles=angles,
                        timestamp=timestamp,
                    )
                )
                conn.send_bytes(payload)
            
            else:
                print("[] null scan", flush=True)
    
    finally:
        lidar.stop_scan()
        lidar.disconnect()


class LidarClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("lidar", ip, port)

        win_port = get_config("lidar.win_port")
        ldr_port = get_config("lidar.port")
        self.port = win_port if platform.system() == "Windows" else ldr_port
        self.baudrate = get_config("lidar.baudrate")

        self.lidar_proc: mp.Process | None = None

        self.last_ping_time = time.time()
        # self.lidar_queue = mp.Queue(10)

        self.parent_conn, self.child_conn = mp.Pipe(duplex=False)

        self.running = mp.Event()

    def scans_job(self):
        if self.lidar_proc is not None and self.lidar_proc.is_alive():
            return
        
        self.running.set()
        self.lidar_proc = mp.Process(target=scan_worker, args=(self.port, self.baudrate, self.running, self.child_conn), daemon=True)
        self.lidar_proc.start()
    
    def get_nowait_bytes(self):
        if self.parent_conn.poll(0):
            return self.parent_conn.recv_bytes()
        
        raise Empty
    
    async def get_scan_async(self, timeout: float = 1.0) -> bytes | None:
        deadline = time.monotonic() + timeout
        
        while time.monotonic() < deadline:
            if not self.running.is_set():
                return None
            
            try:
                return self.get_nowait_bytes()
            
            except Empty:
                await asyncio.sleep(0.005)
        
        return None
        
        # loop = asyncio.get_running_loop()
        
        # try:
        #     data = await loop.run_in_executor(None, self.lidar_queue.get, True, 3)
        #     return data
        # except Empty:
        #     return None

    async def on_ping(self, _, node):
        self.last_ping_time = time.time()


async def main():
    client = LidarClient()
    client.scans_job()

    def shutdown(sig, frame):
        client.running.clear()
        
        if client.lidar_proc is not None:
            client.lidar_proc.join(timeout=3)
            
            if client.lidar_proc.is_alive():
                client.lidar_proc.terminate()
                client.lidar_proc.join(timeout=1)

    async def run():
        nq_start_time = time.time()
        nq = 0
        
        await client.wait()

        ldr_topic = await client.topic("lidar", datatypes.Bytes)

        while client.running.is_set():
            if client.lidar_proc is None or not client.lidar_proc.is_alive():
                print("[] lidar worker died — restarting", flush=True)
                client.scans_job()
                await asyncio.sleep(0.5)
                continue
            
            lidar = await client.get_scan_async()

            if lidar is None:
                continue
            
            nq += 1
            
            await ldr_topic.post(lidar)
            
            if (nq + 1) % 30 == 0:
                print(f"Running at {nq / (time.time() - nq_start_time)}Hz")

            if (nq + 1) % 100 == 0:
                nq = 0
                nq_start_time = time.time()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        await asyncio.gather(
            client.run(),
            run(),
        )

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    asyncio.run(main())
