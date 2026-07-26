"""Explicit ownership root for one MiniSky runtime."""

from __future__ import annotations

from minisky import tools
from minisky.core import varexplorer
from minisky.core.settings import MiniSkySettings, data
from minisky.simulation import ConsoleIO, Runner, Simulation
from minisky.simulation.simulation import OP
from minisky.stack import CommandStack
from minisky.tools.navdata import Navdatabase
from minisky.traffic import Traffic


class MiniSky:
    """Own the primary objects that make up one simulator runtime."""

    def __init__(self, settings: MiniSkySettings, scenario: str | None = None) -> None:
        self.settings = settings
        tools.init()

        self.console = ConsoleIO(lambda: self.simulation.state == OP)
        self.navigation = Navdatabase(data("navigation"), self.console)
        self.traffic = Traffic(settings)
        self.commands = CommandStack(
            traffic=self.traffic,
            navigation=self.navigation,
            console=self.console,
            get_simulation=lambda: self.simulation,
            get_runner=lambda: self.runner,
        )
        self.simulation = Simulation(
            traffic=self.traffic,
            navigation=self.navigation,
            console=self.console,
            command_stack=self.commands,
            stop_runner=self._stop_runner,
        )
        self.runner = Runner(self.simulation, self.console)

        # the compatibility facade must be active before commands and variable
        # explorer parents are registered against this runtime.
        import minisky

        minisky._activate(self)
        varexplorer.init()
        self.commands.init()

        if scenario:
            self.commands.stack(f"IC {scenario}")
        else:
            self.runner.prevent_shutdown()

    def _stop_runner(self) -> None:
        self.runner.stop()

    async def run(self) -> None:
        """Run the simulation until its runner stops."""
        await self.runner.run()
