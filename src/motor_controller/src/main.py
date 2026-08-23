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

        # optimizations
        self.prev_v = 0
        self.prev_w = 0

    @aparsedata(datatypes.Vector)
    async def on_motioncontroller_cmdvel(self, data: datatypes.Vector):
        self.last_update = time.time()

        v, w = data.x, data.y
        if self.prev_v == v and self.prev_w == w:
            return

        self.prev_v = v
        self.prev_w = w

        l, r = self.ik.calculate_wheel_speeds(v, w)

        await self.serial.set_speeds(l, r)

    @aparsedata(datatypes.Movement)
    async def on_slam_pose(self, data: datatypes.Movement):
        self.n += 1

        if self.n % 3 == 0:
            x, y = data.pos.x, data.pos.y
            theta = data.ang.z

            await self.serial.reset_position(x, y, theta)

    async def open_port(self):
        await self.serial.serial.open()


async def main():
    client = MotorControllerClient()
    odometry_post_delay = get_config("motor_controller.miniros.odometry_post_delay")

    is_running = True

    await client.open_port()

    def shutdown(sig, frame):
        nonlocal is_running

        is_running = False
        asyncio.create_task(client.serial.close())

    async def run():
        await client.wait()

        odometry_topic = await client.topic("odometry", datatypes.Vector)
        speeds_topic = await client.topic("velocity", datatypes.Vector)

        while is_running:
            if client.serial.last_odometry is not None:
                x, y, t, v, w = client.serial.last_odometry

                await odometry_topic.post(datatypes.Vector(x, y, t))
                await speeds_topic.post(datatypes.Vector(v, w, 0))

            await asyncio.sleep(odometry_post_delay)

        await client.stop()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        await asyncio.gather(
            client.run(),
            # crash_prevent(),
            client.serial.fetch_task(),
            run(),
        )

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    asyncio.run(main())
