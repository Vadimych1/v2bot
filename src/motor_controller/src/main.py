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
        port = ''
        match platform.system():
            case "Windows":
                port = 'COM3'
                
            case _:
                port = '/dev/ttyArduino' # TODO: change to the actual arduino port
            
        self.serial = ArduinoSerial(port, 9600)
        self.last_update = time.time()
        self.ik = CurrentIK(0.15, 0.02)
        
    @aparsedata(datatypes.Vector)
    async def on_motioncontroller_cmdvel(self, data: datatypes.Vector):
        self.last_update = time.time()
        
        # calculate speeds based on values
        v, w = data.x, data.y
        l, r = self.ik.calculate_wheel_speeds(v, w)
        self.serial.send_speeds(l, r)

async def main():
    client = MotorControllerClient()
    
    async def run():
        await client.wait()
        
        while True:
            await asyncio.sleep(0.5)
            
            # time limit from last speeds
            # update to prevent crashes
            if time.time() - client.last_update > 1.0:
                client.serial.send_floats(0, 0)
    
    await asyncio.gather(
        client.run(),
        run(),
    )

asyncio.run(main())