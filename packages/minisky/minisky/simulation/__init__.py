"""Simulation module of MiniSky.

Bundles the three objects that drive a simulation run:

- [`Simulation`][minisky.simulation.simulation.Simulation]: controls simulation
  time and performs one timestep per call to
  [`Simulation.step`][minisky.simulation.simulation.Simulation.step].
- [`SimulationState`][minisky.simulation.simulation.SimulationState]: the
  lifecycle state — `INIT` before the first traffic or scenario command, `OP`
  while time advances, `HOLD` when paused, `END` once stopped.
- [`Runner`][minisky.simulation.runner.Runner]: the asyncio loop that repeatedly steps the simulation at a
  configurable real-time speed, with support for fast-forward jumps.
- [`ConsoleIO`][minisky.simulation.console.ConsoleIO]: collects console/echo output from the simulation so it
  can be printed and forwarded to remote clients (e.g. the HTTP API).
"""

from .console import ConsoleIO
from .runner import Runner
from .simulation import Simulation, SimulationState

__all__ = ("ConsoleIO", "Runner", "Simulation", "SimulationState")
