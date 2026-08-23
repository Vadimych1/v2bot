import asyncio
import base64
import io
import importlib.resources
import uvicorn
import numpy as np
from PIL import Image
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from miniros import AsyncROSClient, datatypes, aparsedata
from miniros_slam.source.datatypes import SLAMOffsetMap

app = FastAPI()


class GoalManagerClient(AsyncROSClient):

    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("goalmanager", ip, port, _parse_handlers)

        self.goal_topic = None

        self.latest_map = None
        self.latest_pose = None
        self.latest_path = None
        self.latest_cmdvel = None

        self.web_clients: set[WebSocket] = set()

    @aparsedata(datatypes.Vector)
    async def on_motioncontroller_cmdvel(
        self,
        data: datatypes.Vector
    ):
        self.latest_cmdvel = {
            "linear": float(data.x),
            "angular": float(data.y)
        }

        await self.broadcast({
            "type": "cmdvel",
            "cmdvel": self.latest_cmdvel
        })

    @aparsedata(SLAMOffsetMap)
    async def on_slam_map(self, data):
        self.latest_map = data
        await self.broadcast_map(data)

    @aparsedata(datatypes.Vector)
    async def on_motorcontroller_odometry(self, data: datatypes.Vector):
        self.latest_pose = {
            "x": float(data.x),
            "y": float(data.y),
            "heading": float(data.z),
        }

        await self.broadcast({"type": "pose", "pose": self.latest_pose})

    @aparsedata(datatypes.NumpyArray)
    async def on_pathplanner_globalpath(self, path: np.ndarray):
        self.latest_path = [
            {"x": float(point[0]), "y": float(point[1])} for point in path
        ]

        await self.broadcast({"type": "path", "path": self.latest_path})

    def encode_map(self, data):
        grid = np.asarray(data.grid)

        width = int(data.width)
        height = int(data.height)

        image = np.full((height, width), 127, dtype=np.uint8)

        image[grid > 40] = 255
        image[grid <= 40] = 0

        pil_image = Image.fromarray(image, mode="L")

        buffer = io.BytesIO()

        pil_image.save(buffer, format="PNG")

        encoded = base64.b64encode(buffer.getvalue()).decode("ascii")

        return {
            "image": encoded,
            "width": width,
            "height": height,
            # IMPORTANT:
            # These are PIXEL offsets of world (0, 0).
            "offset_x": float(data.offset_x),
            "offset_y": float(data.offset_y),
            "resolution": float(data.resolution),
        }

    async def broadcast_map(self, data):
        message = {"type": "map", "map": self.encode_map(data)}
        await self.broadcast(message)

    async def broadcast(self, message):
        if not self.web_clients:
            return

        disconnected = []

        for websocket in self.web_clients:
            try:
                await websocket.send_json(message)

            except Exception:
                disconnected.append(websocket)

        for websocket in disconnected:
            self.web_clients.discard(websocket)


client = GoalManagerClient()


@app.get("/")
async def index():
    return FileResponse(
        str(
            importlib.resources.files("miniros_visual_goal_manager").joinpath(
                "web/index.html"
            )
        )
    )


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()

    client.web_clients.add(websocket)

    try:
        if client.latest_map is not None:
            await websocket.send_json(
                {"type": "map", "map": client.encode_map(client.latest_map)}
            )

        if client.latest_pose is not None:
            await websocket.send_json({"type": "pose", "pose": client.latest_pose})

        if client.latest_path is not None:
            await websocket.send_json({"type": "path", "path": client.latest_path})
            
        if client.latest_cmdvel is not None:
            await websocket.send_json({"type": "cmdvel", "cmdvel": client.latest_cmdvel})

        while True:
            message = await websocket.receive_json()
            message_type = message.get("type")

            if message_type == "goal":
                x = float(message["x"])
                y = float(message["y"])

                if client.goal_topic is not None:
                    await client.goal_topic.post(datatypes.Vector(x, y, 0))

    except WebSocketDisconnect:
        pass

    except Exception as e:
        print("WebSocket error:", repr(e))

    finally:
        client.web_clients.discard(websocket)


async def ros_task():
    await client.run()


async def web_task():
    config = uvicorn.Config(app, host="0.0.0.0", port=8080)
    server = uvicorn.Server(config)

    await server.serve()


async def main():
    async def _job():
        await client.wait()
        client.goal_topic = await client.topic("currentgoal", datatypes.Vector)

    await asyncio.gather(ros_task(), web_task(), _job())


if __name__ == "__main__":
    asyncio.run(main())
