import importlib.resources as res
from functools import lru_cache
from typing import Any
import json

config = json.loads(res.files("miniros_configurator").joinpath("config.json").read_text())
_NoDefaultValue = {}

@lru_cache(maxsize=128)
def get_config(path: str, default: Any | None = _NoDefaultValue):
    """
    @param path: dot-separated path to value (ex. slam.search.distance_fine)
    """
    
    try:
        q = config
        for val in path.split("."):
            q = q[val]
    
    except Exception as e:
        if default is _NoDefaultValue:
            raise e
        
        return default
    
    return q
