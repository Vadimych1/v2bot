from miniros.util.datatypes import Datatype, Movement, Int, Dict, Vector
import numpy as np

class SLAMMap(Datatype):
    def __init__(self, mapdata: bytearray):
        super().__init__()
        self.data = mapdata

    @staticmethod
    def encode(data: "SLAMMap"):
        return data.data
    
    @staticmethod
    def decode(data: bytearray) -> "SLAMMap":
        return SLAMMap(data)
    
    def to_numpy(self, mapsize: int) -> np.ndarray:
        return np.frombuffer(self.data, dtype=np.uint8).reshape((mapsize, mapsize))

class SLAMPosition(Datatype):
    _len = 24
    
    def __init__(self, pos: Vector, ang: Vector):
        self.pos = pos
        self.ang = ang
    
    @staticmethod
    def decode(data: bytes):
        if len(data) < SLAMPosition._len: return SLAMPosition(Vector(0, 0, 0), Vector(0, 0, 0))
        
        a, b = data[:12], data[12:]
        return SLAMPosition(Vector.decode(a), Vector.decode(b))
        
    @staticmethod
    def encode(data: "SLAMPosition"):
        a, b = Vector.encode(data.pos), Vector.encode(data.ang)
        return a + b


    def pos_to_numpy(self) -> np.ndarray:
        return np.array([self.pos.x, self.pos.y, self.pos.z])

    def rot_to_numpy(self) -> np.ndarray:
        return np.array([self.ang.x, self.ang.y, self.ang.z])

class SLAMAnonSave(Int): ...
class SLAMAnonLoad(Int): ...