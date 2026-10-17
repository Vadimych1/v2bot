# from tiny_tf.tf import Transform
# from tiny_tf.transformations import quaternion_from_euler
from karto_scanmatcher import Pose2
from miniros.util.datatypes import Movement3DoF, TimedMovement3DoF

class FixedSizeList(list):
    def __init__(self, maxlen, iterable=()):
        self.maxlen = maxlen
        super().__init__(iterable)
        
        if len(self) > maxlen:
            del self[: len(self) - maxlen]

    def append(self, item):
        super().append(item)
        if len(self) > self.maxlen:
            self.pop(0)

    def insert(self, index, item):
        super().insert(index, item)
        if len(self) > self.maxlen:
            self.pop(0)

    def extend(self, iterable):
        super().extend(iterable)
        while len(self) > self.maxlen:
            self.pop(0)
            
# def movement2transform(msg) -> Transform:
#     return Transform(msg.x, msg.y, 0, *quaternion_from_euler(0, 0, msg.theta))

def pose2movement(pose: Pose2, timestamp: float):
    return TimedMovement3DoF(
        movement=Movement3DoF(
            x=pose.x,
            y=pose.y,
            theta=pose.yaw,
        ),
        timestamp=timestamp,
    )