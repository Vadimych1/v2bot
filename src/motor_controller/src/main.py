from miniros_motor_controller.source.vserial import ArduinoSerial
from miniros_algorithms.source.inverse_kinematics import CurrentIK
from miniros.util.decorators import aparsedata
from miniros import AsyncROSClient, datatypes
import platform
import asyncio
import time


class MotorControllerClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("motorcontroller", ip, port, _parse_handlers)

        # get port based on system
        # for testing compability
        # TODO: maybe add port definition
        # for MacOS
        port = ""
        match platform.system():
            case "Windows":
                port = "COM5"

            case _:
                port = "/dev/ttyArduino"  # TODO: change to the actual arduino port name

        self.serial = ArduinoSerial(port, 115200)
        self.last_update = time.time()
        self.ik = CurrentIK(0.15, 0.02)
        
        self.serial_sync_lock = asyncio.Lock()

    @aparsedata(datatypes.Vector)
    async def on_motioncontroller_cmdvel(self, data: datatypes.Vector):
        self.last_update = time.time()

        # calculate speeds based on values
        v, w = data.x, data.y
        l, r = self.ik.calculate_wheel_speeds(v, w)
        
        await self.serial_sync_lock.acquire()
        self.serial.send_speeds(l, r)
        self.serial_sync_lock.release()


async def main():
    client = MotorControllerClient()
    client.serial.run_deltas_fetch()

    async def run():
        await client.wait()

        odometry_topic = await client.topic("odometry", datatypes.Vector)

        while True:
            await asyncio.sleep(0.19)

            # time limit from last speeds
            # update to prevent crashes
            if time.time() - client.last_update > 1.0:
                await client.serial_sync_lock.acquire()
                client.serial.send_floats(0, 0)
                client.serial_sync_lock.release()

            while len(client.serial.odometry_queue) > 0:
                dat = client.serial.odometry_queue.popleft()
                await odometry_topic.post(datatypes.Vector(dat[0], dat[1], dat[2]))
                await asyncio.sleep(0.03) # a small delay

    await asyncio.gather(
        client.run(),
        run(),
    )


asyncio.run(main())
