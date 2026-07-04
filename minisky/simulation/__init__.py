"""Simulation control subpackage of MiniSky.

Bundles the three objects that drive a simulation run:

- :class:`Simulation`: owns simulation time, state (INIT/HOLD/OP/END) and
  performs one timestep per call to :meth:`Simulation.step`.
- :class:`Runner`: the asyncio loop that repeatedly steps the simulation at a
  configurable real-time speed, with support for fast-forward jumps.
- :class:`ConsoleIO`: collects console/echo output from the simulation so it
  can be printed and forwarded to remote clients (e.g. the HTTP API).
"""

from .console import ConsoleIO
from .runner import Runner
from .simulation import Simulation
