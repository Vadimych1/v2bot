import asyncio
import aioconsole as aioc
from miniros import AsyncROSClient
from miniros.util.datatypes import Vector


class GoalManagerClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("goalmanager", ip, port, _parse_handlers)


async def main():
    client = GoalManagerClient()

    async def run():
        await client.wait()

        currentgoal_topic = await client.topic("currentgoal", Vector)

        while True:
            try:
                x, y = map(float, (await aioc.ainput("X Y > ")).split())
                await currentgoal_topic.post(Vector(x, y, 0))

            except Exception as e:
                print("[e]", e)

    await asyncio.gather(client.run(), run())


if __name__ == "__main__":
    asyncio.run(main())
