import numpy as np

import signal
import platform
import asyncio
import time

from miniros_motor_controller.source.vserial import ArduinoSerial
from miniros.util.decorators import aparsedata
from miniros import AsyncROSClient, datatypes
from miniros_configurator import get_config


class TrackedRobotIK:
    def __init__(self, track_distance: float, wheel_radius: float):
        super().__init__()

        self.d = track_distance
        self.r = wheel_radius

    def calculate_wheel_speeds(self, v: float, omega: float) -> np.ndarray:
        speeds = np.array([v - omega * self.d / 2, v + omega * self.d / 2])
        speeds /= self.r

        return speeds


class MotorControllerClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("motorcontroller", ip, port, _parse_handlers)

        # get port based on system
        # for testing compatibility
        # TODO: maybe add port definition
        # for MacOS
        port = ""
        baudrate = get_config("motor_controller.serial.baudrate")

        match platform.system():
            case "Windows":
                port = get_config("motor_controller.serial.win_port")

            case _:
                port = get_config("motor_controller.serial.port")

        track_distance = get_config("motor_controller.robot.track_distance")
        wheel_radius = get_config("motor_controller.robot.wheel_radius")

        self.ik = TrackedRobotIK(track_distance, wheel_radius)
        self.serial = ArduinoSerial(port, baudrate)
        self.last_update = time.time()
        self.n = 0

        self.latest_cmd: tuple[float, float] | None = None
        self.cmd_event = asyncio.Event()

    @aparsedata(datatypes.Vector)
    async def on_motioncontroller_cmdvel(self, data: datatypes.Vector):
        self.last_update = time.time()

        v, w = data.x, data.y
        self.latest_cmd = (v, w)
        self.cmd_event.set()
        
    async def open_port(self):
        await self.serial.serial.open()


async def main():
    client = MotorControllerClient()
    is_running = True

    await client.open_port()

    def shutdown(sig, frame):
        nonlocal is_running

        is_running = False
        asyncio.create_task(client.serial.close())

    async def cmdvel_write_task():
        last_sent = None
        while is_running:
            await client.cmd_event.wait()
            client.cmd_event.clear()
            
            cmd = client.latest_cmd
            client.latest_cmd = None
            
            if cmd is not None and cmd != last_sent:
                v, w = cmd
                try:
                    l, r = client.ik.calculate_wheel_speeds(v, -w)
                    await client.serial.set_speeds(l, -r)
                    last_sent = cmd
                
                except Exception as e:
                    print(f"serial write failed: {e}", flush=True)
                    await asyncio.sleep(0.1)

    async def run():
        await client.wait()

        odometry_topic = await client.topic("odometry", datatypes.TimedMovement3DoF)
        speeds_topic = await client.topic("velocity", datatypes.Vector)

        while is_running:
            await client.serial.odometry_event.wait()
            client.serial.odometry_event.clear()

            odom = client.serial.last_odometry
            client.serial.last_odometry = None

            if odom != None:                
                x, y, theta, v, w = odom

                await odometry_topic.post(
                    datatypes.TimedMovement3DoF(
                        timestamp=time.time(),
                        movement=datatypes.Movement3DoF(
                            x=x,
                            y=y,
                            theta=-theta,
                        ),
                    )
                )
                await speeds_topic.post(datatypes.Vector(v, w, 0))

        await client.stop()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    
    run_task = asyncio.create_task(run())
    write_task = asyncio.create_task(cmdvel_write_task())
    fetch_task = asyncio.create_task(client.serial.fetch_task())

    try:
        await asyncio.gather(
            client.run(),
            fetch_task,
            run_task,
            write_task,
        )

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    asyncio.run(main())
