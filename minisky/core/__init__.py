"""Core simulation infrastructure of MiniSky.

Contains the per-aircraft array bookkeeping (trafficarrays), the settings
loader (settings), and the variable explorer (varexplorer). The classes
TrafficArrays and RegisterElementParameters, which all traffic-related
simulation entities build on, are re-exported here for convenience.
"""

from minisky.core.trafficarrays import RegisterElementParameters, TrafficArrays

from . import settings, trafficarrays, varexplorer

__all__ = [
    "RegisterElementParameters",
    "TrafficArrays",
    "settings",
    "trafficarrays",
    "varexplorer",
]
