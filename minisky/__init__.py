"""MiniSky air traffic simulator.

[`MiniSky`][minisky.runtime.MiniSky] is the explicit ownership root for one
simulator runtime. Construct it with validated [`MiniSkySettings`][] and access
simulation components through that instance.
"""

from minisky.core.settings import DEFAULT_SETTINGS_FILE, MiniSkySettings
from minisky.runtime import MiniSky
from minisky.simulation import SimulationState

BS_OK = 0
BS_ARGERR = 1
BS_FUNERR = 2
BS_CMDERR = 4

__all__ = (
    "BS_ARGERR",
    "BS_CMDERR",
    "BS_FUNERR",
    "BS_OK",
    "DEFAULT_SETTINGS_FILE",
    "MiniSky",
    "MiniSkySettings",
    "SimulationState",
)
