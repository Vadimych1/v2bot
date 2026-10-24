from miniros import AsyncROSClient, datatypes
from miniros_configurator import get_config

import cv2 as cv
import asyncio


class CameraClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("camera", ip, port, _parse_handlers)


async def main():
    client = CameraClient()

    async def run():
        await client.wait()
        
        camera_topic = await client.topic("image", datatypes.OpenCVImage)
        cap = cv.VideoCapture(get_config("camera.device_idx"))

        try:
            while cap.isOpened():
                ret, frame = await asyncio.to_thread(cap.read)
                
                if not ret:
                    break
                
                await camera_topic.post(frame)
                await asyncio.sleep(0.001)

        except Exception as e:
            print(f"camera exception: {e}")
        
        finally:
            cap.release()
            await client.stop()
    
    run_task = asyncio.create_task(run())

    await asyncio.gather(client.run(), run_task)

if __name__ == "__main__":
    asyncio.run(main())
