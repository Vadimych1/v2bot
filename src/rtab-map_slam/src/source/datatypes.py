from miniros.util.datatypes import NamedComposedDatatype, NumpyArray, Int, Float

SLAMOffsetMap = NamedComposedDatatype({
    "grid": NumpyArray,
    "width": Int,
    "height": Int,
    "offset_x": Int,
    "offset_y": Int,
    "resolution": Float,
}, "SLAMOffsetMap")