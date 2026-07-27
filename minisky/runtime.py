"""Explicit ownership root for one MiniSky runtime."""

from __future__ import annotations

from minisky import tools
from minisky.core.settings import MiniSkySettings, data
from minisky.core.varexplorer import VariableExplorer
from minisky.simulation import ConsoleIO, Runner, Simulation
from minisky.simulation.simulation import OP
from minisky.stack import CommandStack
from minisky.streaming import StreamHub, build_snapshot
from minisky.tools.areafilter import AreaFilter
from minisky.tools.navdata import Navdatabase
from minisky.traffic import Traffic


class MiniSky:
    """Own the primary objects that make up one simulator runtime."""

    def __init__(self, settings: MiniSkySettings, scenario: str | None = None) -> None:
        self.settings = settings
        tools.init()

        self.console = ConsoleIO(lambda: self.simulation.state == OP)
        self.navigation = Navdatabase(data("navigation"), self.console)
        self.areas = AreaFilter()
        self.variables = VariableExplorer()
        self.traffic = Traffic(
            settings=settings,
            areas=self.areas,
            navigation=self.navigation,
            console=self.console,
            get_simulation=lambda: self.simulation,
            stack_command=lambda *args, **kwargs: self.commands.stack(*args, **kwargs),
            get_command_registry=lambda: self.commands.cmddict,
            select_implementation=lambda base, impl: self.commands.select_implementation(
                base, impl
            ),
        )
        self.commands = CommandStack(
            traffic=self.traffic,
            navigation=self.navigation,
            console=self.console,
            areas=self.areas,
            variables=self.variables,
            get_simulation=lambda: self.simulation,
            get_runner=lambda: self.runner,
        )
        self.streaming = StreamHub(
            lambda: build_snapshot(self.simulation, self.traffic, self.runner, self.commands)
        )
        self.simulation = Simulation(
            traffic=self.traffic,
            navigation=self.navigation,
            console=self.console,
            command_stack=self.commands,
            areas=self.areas,
            stop_runner=self._stop_runner,
            publish_tick=self.streaming.publish_tick,
        )
        self.runner = Runner(self.simulation, self.console)
        self.variables.init(self.simulation, self.traffic)

        # the compatibility facade must be active before commands and variable
        # explorer parents are registered against this runtime.
        import minisky

        minisky._activate(self)
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
