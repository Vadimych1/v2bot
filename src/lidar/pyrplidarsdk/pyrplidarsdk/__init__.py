"""
PyRPLIDARSDK - Python wrapper for RPLIDAR SDK

This package provides a Python interface to the Slamtec RPLIDAR series of 
2D laser range scanners using nanobind to wrap the official C++ SDK.

Copyright (C) 2025 Dexmate Inc.
Licensed under MIT License (see LICENSE for details)
"""

from .rplidar_wrapper import RplidarDriver, DeviceInfo, DeviceHealth

__version__ = "0.1.0"
__author__ = "Dexmate Inc."
__email__ = "contact@dexmate.ai"

__all__ = [
    "RplidarDriver",
    "DeviceInfo", 
    "DeviceHealth",
] 