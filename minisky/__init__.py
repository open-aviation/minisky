"""MiniSky air traffic simulator.

`MiniSky` is the explicit owner of a simulator
runtime. The module-level `traf`, `sim`, `scr`, `runner`, and `navdb` names are
temporary compatibility aliases for the active runtime.
"""

from __future__ import annotations

from minisky import core, plugin, stack, tools
from minisky.core.settings import MiniSkySettings, data, filename_settings
from minisky.simulation import ConsoleIO, Runner, Simulation
from minisky.simulation.simulation import END, HOLD, INIT, OP
from minisky.tools.navdata import Navdatabase

# isort: split
# traffic remains last because importing the performance model reads
# `minisky.data` during module initialization.
from minisky import traffic
from minisky.traffic import Traffic

# Constants
BS_OK = 0
BS_ARGERR = 1
BS_FUNERR = 2
BS_CMDERR = 4

_current: MiniSky | None = None
runner: Runner = None  # type: ignore[assignment]
traf: Traffic = None  # type: ignore[assignment]
navdb: Navdatabase = None  # type: ignore[assignment]
sim: Simulation = None  # type: ignore[assignment]
scr: ConsoleIO = None  # type: ignore[assignment]


def _activate(instance: MiniSky) -> None:
    """Point the compatibility aliases at an active runtime."""
    global _current, runner, traf, navdb, sim, scr

    _current = instance
    runner = instance.runner
    traf = instance.traffic
    navdb = instance.navigation
    sim = instance.simulation
    scr = instance.console
    stack._activate(instance.commands)
    core.varexplorer._activate(instance.variables)
    tools.areafilter._activate(instance.areas)


from minisky.runtime import MiniSky  # noqa: E402


def init(
    scenario: str | None = None,
    settings: MiniSkySettings | None = None,
) -> MiniSky:
    """Construct and activate a MiniSky runtime.

    This function is a compatibility adapter. New code should construct
    `MiniSky` directly with explicit settings.
    """
    if settings is None:
        settings = MiniSkySettings.from_file(filename_settings)

    instance = MiniSky(settings, scenario)

    # plugin discovery remains part of the legacy startup path for now.
    plugin.discover()
    return instance


def load_plugins() -> None:
    """Load plugins enabled by the compatibility settings module."""
    plugin.load_enabled()
