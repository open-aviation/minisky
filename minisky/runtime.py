"""Explicit ownership root for one MiniSky runtime."""

from __future__ import annotations

from minisky import stack, tools
from minisky.core import varexplorer
from minisky.core.settings import MiniSkySettings
from minisky.simulation import ConsoleIO, Runner, Simulation
from minisky.tools.navdata import Navdatabase
from minisky.traffic import Traffic


class MiniSky:
    """Own the primary objects that make up one simulator runtime."""

    def __init__(self, settings: MiniSkySettings, scenario: str | None = None) -> None:
        self.settings = settings
        tools.init()

        self.navigation = Navdatabase()
        self.traffic = Traffic(settings)
        self.simulation = Simulation()
        self.console = ConsoleIO()
        self.runner = Runner()

        # compatibility facade must be active before modules register
        # commands and variable-explorer parents against the runtime.
        import minisky

        minisky._activate(self)
        varexplorer.init()

        if scenario:
            stack.stack(f"IC {scenario}")
        else:
            self.runner.prevent_shutdown()

        stack.init()

    async def run(self) -> None:
        """Run the simulation until its runner stops."""
        await self.runner.run()
