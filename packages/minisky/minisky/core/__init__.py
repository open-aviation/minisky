"""Core simulation infrastructure of MiniSky.

Contains configuration loading, per-aircraft array bookkeeping, and the
variable explorer. The classes TrafficArrays and RegisterElementParameters,
which all traffic-related simulation entities build on, are re-exported here
for convenience.
"""

from __future__ import annotations

from minisky.core.trafficarrays import RegisterElementParameters, TrafficArrays

from . import config, trafficarrays, varexplorer

__all__ = (
    "RegisterElementParameters",
    "TrafficArrays",
    "config",
    "trafficarrays",
    "varexplorer",
)
