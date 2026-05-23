from miniros.util.datatypes import Datatype, Int
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


class SLAMAnonSave(Int): ...


class SLAMAnonLoad(Int): ...
