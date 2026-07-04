"""MiniSky: a minimal fork of BlueSky, the open-source ATM simulator.

This package is the top-level entry point of the simulator. It exposes the
main singleton objects that together make up a running simulation:

- ``traf``: the :class:`~minisky.traffic.Traffic` object holding all aircraft states
- ``sim``: the :class:`~minisky.simulation.Simulation` object controlling sim time and state
- ``scr``: the :class:`~minisky.simulation.ConsoleIO` object buffering console output
- ``runner``: the :class:`~minisky.simulation.Runner` driving the asyncio simulation loop
- ``navdb``: the :class:`~minisky.tools.navdata.Navdatabase` with navaids, airports, and airways

It also defines the shared return codes for stack commands (``BS_OK``,
``BS_ARGERR``, ``BS_FUNERR``, ``BS_CMDERR``) and the simulation state
constants (``INIT``, ``HOLD``, ``OP``, ``END``).

Call :func:`init` once to construct these singletons, then optionally
:func:`load_plugins` to activate the plugins enabled in the settings.
"""

from minisky import core, plugin, stack, tools
from minisky.core import varexplorer
from minisky.core.settings import data
from minisky.simulation import ConsoleIO, Runner, Simulation
from minisky.tools.navdata import Navdatabase
from minisky.traffic import Traffic

# Constants
BS_OK = 0
BS_ARGERR = 1
BS_FUNERR = 2
BS_CMDERR = 4

# simulation states
INIT, HOLD, OP, END = (0, 1, 2, 3)

# Main singleton objects in BlueSky. They are None until init() constructs them,
# but are annotated with their concrete types so downstream code type-checks
# against the real objects (init() must be called before any of them are used).
runner: Runner = None  # type: ignore[assignment]
traf: Traffic = None  # type: ignore[assignment]
navdb: Navdatabase = None  # type: ignore[assignment]
sim: Simulation = None  # type: ignore[assignment]
scr: ConsoleIO = None  # type: ignore[assignment]


def init(scenario: str | None = None) -> None:
    """Initialize all MiniSky modules and singletons.

    Constructs the navigation database, traffic, simulation, console I/O and
    runner singletons, initializes the tools and variable explorer, and
    discovers available plugins (via AST parsing, without importing them).
    Must be called once before the simulation is stepped or run.

    If a scenario filename is given it is loaded onto the command stack with
    the ``IC`` command; otherwise the runner is configured to stay alive even
    when a ``QUIT``/``STOP`` command is issued, so an idle simulator keeps
    accepting commands.

    Args:
        scenario: Optional path to a scenario (.scn) file to load at startup.
            When omitted, the simulator starts empty and shutdown is prevented.
    """
    global traf, sim, scr, runner
    global navdb

    # Initialise tools
    tools.init()

    navdb = Navdatabase()

    # Initialize singletons
    traf = Traffic()
    sim = Simulation()
    scr = ConsoleIO()
    runner = Runner()

    # Initialize remaining modules
    varexplorer.init()

    if scenario:
        stack.stack(f"IC {scenario}")
    else:
        # without scenario, sim shall be up
        runner.prevent_shutdown()

    stack.init()

    # Discover available plugins (AST parsing only, no imports)
    plugin.discover()


def load_plugins() -> None:
    """Load the plugins enabled in the settings.

    Imports and initializes every plugin listed under ``enabled_plugins`` in
    the settings, registering their timed functions and stack commands.
    Must be called after :func:`init`, since plugins may rely on the traffic
    and simulation singletons during initialization.
    """
    plugin.load_enabled()
