import importlib.resources as res
import json

config = json.loads(res.files("miniros_configurator").joinpath("config.json").read_text())

def get_config(path: str):
    """
    @param path: dot-separated path to value (ex. slam.search.distance_fine)
    """
    q = config
    for val in path.split("."):
        q = q[val]
    return q