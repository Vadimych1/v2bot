import pyrplidarsdk
import time

import platform
import asyncio
import signal

from miniros import AsyncROSClient, datatypes, naming
from miniros import MinirosPipe, ProcessPipe, PipeProtocol
from miniros_configurator import get_config

protocol = PipeProtocol([datatypes.Lidar2D])

def scan_worker(pipe: ProcessPipe, running, port, baudrate):
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
                
                data = datatypes.Lidar2D(
                    distances=distances,
                    angles=angles,
                    timestamp=timestamp,
                )
                
                pipe.write(data)
            
            else:
                print("[] null scan", flush=True)
    
    finally:
        lidar.stop_scan()
        lidar.disconnect()


class LidarClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__(naming.Node.LIDAR, ip, port)

        win_port = get_config("lidar.win_port")
        ldr_port = get_config("lidar.port")
        self.port = win_port if platform.system() == "Windows" else ldr_port
        self.baudrate = get_config("lidar.baudrate")

        self.last_ping_time = time.time()
        self.pipe = MinirosPipe(protocol, scan_worker, False)

    def startup(self):
        self.pipe.start(self.port, self.baudrate)
    
    def shutdown(self):
        self.pipe.stop()
    
    async def get_scan_async(self, timeout: float = 1.0) -> bytes | None:
        return await self.pipe.read_async(timeout=timeout)
    
    async def on_ping(self, _, __):
        self.last_ping_time = time.time()


async def main():
    client = LidarClient()
    client.startup()
    
    running = True

    def shutdown(sig, frame):
        nonlocal running
        
        running = False
        
        asyncio.create_task(client.stop())
        client.shutdown()
        
    async def run():
        nq_start_time = time.time()
        nq = 0
        
        await client.wait()

        ldr_topic = await client.topic(naming.Topic.L_SCAN, datatypes.Lidar2D)

        while running:
            if not client.pipe.is_alive():
                print("[] lidar worker died. restarting", flush=True)
                client.startup()
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
    
    lidar_run_task = asyncio.create_task(run())

    try:
        await asyncio.gather(
            client.run(),
            lidar_run_task,
        )

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    asyncio.run(main())
