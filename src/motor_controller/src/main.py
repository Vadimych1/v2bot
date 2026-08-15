from miniros_motor_controller.source.vserial import ArduinoSerial
from miniros_algorithms.source.inverse_kinematics import CurrentIK
from miniros.util.decorators import aparsedata
from miniros import AsyncROSClient, datatypes
import signal
import platform
import asyncio
import time


class MotorControllerClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("motorcontroller", ip, port, _parse_handlers)

        # get port based on system
        # for testing compatibility
        # TODO: maybe add port definition
        # for MacOS
        port = ""
        match platform.system():
            case "Windows":
                port = "COM6"

            case _:
                port = "/dev/ttyUSB0"

        self.serial = ArduinoSerial(port, 115200)
        self.last_update = time.time()
        self.ik = CurrentIK(0.19, 0.022)
        self.n = 0
    
    @aparsedata(datatypes.Vector)
    async def on_motioncontroller_cmdvel(self, data: datatypes.Vector):        
        self.last_update = time.time()

        v, w = data.x, data.y
        l, r = self.ik.calculate_wheel_speeds(v, w)

        await self.serial.set_speeds(l, r)
        
        print(l, r)

    @aparsedata(datatypes.Movement)
    async def on_slam_pose(self, data: datatypes.Movement):
        self.n += 1

        if self.n % 3 == 0:
            x, y = data.pos.x, data.pos.y
            theta = data.ang.z
            
            print("SLAM:", x, y, theta)

            await self.serial.reset_position(x, y, theta)
            
    async def open_port(self):
        await self.serial.serial.open()
        

async def main():
    client = MotorControllerClient()
    await client.open_port()

    def shutdown(sig, frame):
        asyncio.create_task(client.serial.close()).add_done_callback(lambda _: quit(0))

    async def run():
        await client.wait()

        odometry_topic = await client.topic("odometry", datatypes.Vector)
        speeds_topic = await client.topic("velocity", datatypes.Vector)

        while True:
            x, y, t, v, w = await client.serial.odometry_speeds_queue.get()

            await odometry_topic.post(datatypes.Vector(x, y, t))
            await speeds_topic.post(datatypes.Vector(v, w, 0))

    # # async def crash_prevent():
    # #     while True:
    # #         await asyncio.sleep(0.1)

    # #         # time limit from last speeds
    # #         # update to prevent crashes
    # #         if time.time() - client.last_update > 0.5:
    # #             client.serial.set_speeds(0, 0)

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
