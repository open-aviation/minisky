"""MiniSky air traffic simulator.

[`MiniSky`][minisky.runtime.MiniSky] is the explicit ownership root for a
simulator runtime. Construct it without arguments to use the optional default
user config, or pass a validated [`MiniSkyConfig`][] explicitly.
"""

from minisky.core.config import (
    MiniSkyConfig,
    default_user_config_dir,
    default_user_config_toml_path,
)
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
    "default_user_config_dir",
    "default_user_config_toml_path",
    "MiniSky",
    "MiniSkyConfig",
    "SimulationState",
)
