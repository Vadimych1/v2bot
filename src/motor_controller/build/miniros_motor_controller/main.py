from miniros import AsyncROSClient
from miniros_motor_controller.source.vserial import ArduinoSerial
import platform
import asyncio

class MotorControllerClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("motorcontroller", ip, port, _parse_handlers)

async def main():
    port = ''
    
    match platform.system():
        case "Windows":
            port = 'COM3'
            
        case _:
            port = '/dev/ttyUSBArduino'
    
    client = MotorControllerClient()
    serial = ArduinoSerial('COM')

# Usage example
if __name__ == "__main__":
    # Change COM port as needed (COM3 for Windows, /dev/ttyUSB0 or /dev/ttyACM0 for Linux/Mac)
    arduino = ArduinoSerial('COM3', 9600)
    
    try:
        # 1. Send PING
        if arduino.send_ping():
            print("Arduino is alive!")
        
        arduino._sound_startup()
        
        # 2. Send float data
        arduino.send_floats(2, 2)
        time.sleep(2)
        arduino.send_floats(-2, -2)
        time.sleep(2)
        arduino.send_floats(2, 0)
        time.sleep(2)
        arduino.send_floats(0, 2)
        time.sleep(2)
        arduino.send_floats(0, 0)
        
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        arduino.close()