"""Explicit ownership root for one MiniSky runtime."""

from __future__ import annotations

import asyncio
from contextlib import suppress

from minisky.core.settings import MiniSkySettings, data
from minisky.core.trafficarrays import ReplaceableManager
from minisky.core.varexplorer import VariableExplorer
from minisky.plugin import PluginManager
from minisky.simulation import ConsoleIO, Runner, Simulation, SimulationState
from minisky.stack import CommandStack
from minisky.streaming import StreamHub, build_snapshot
from minisky.tools.areafilter import AreaFilter
from minisky.tools.navdata import Navdatabase
from minisky.traffic import Traffic


class MiniSky:
    """Own the primary objects that make up one simulator runtime."""

    def __init__(self, settings: MiniSkySettings, scenario: str | None = None) -> None:
        self.settings = settings
        self._run_task: asyncio.Task[None] | None = None
        self._closed = False
        self.console = ConsoleIO(
            lambda: self.simulation.state == SimulationState.OP
        )
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
            select_implementation=lambda base, impl: self.replaceables.select(base, impl),
        )
        self.replaceables = ReplaceableManager(self.traffic, lambda: self.commands.cmddict)
        self.plugins = PluginManager(
            settings=settings,
            console=self.console,
            variables=self.variables,
            get_runtime=lambda: self,
            get_simulation=lambda: self.simulation,
            get_command_stack=lambda: self.commands,
        )
        self.commands = CommandStack(
            traffic=self.traffic,
            navigation=self.navigation,
            console=self.console,
            areas=self.areas,
            variables=self.variables,
            plugins=self.plugins,
            replaceables=self.replaceables,
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
            plugins=self.plugins,
            replaceables=self.replaceables,
            stop_runner=self._stop_runner,
            publish_tick=self.streaming.publish_tick,
        )
        self.runner = Runner(self.simulation, self.console)
        self.variables.init(self.simulation, self.traffic)

        self.commands.init()
        self.plugins.discover()

        if scenario:
            self.commands.stack(f"IC {scenario}")
        else:
            self.runner.prevent_shutdown()

    def _stop_runner(self) -> None:
        self.runner.stop()

    def load_plugins(self) -> None:
        """Load plugins enabled in this runtime's settings."""
        self.plugins.load_enabled()

    async def run(self) -> None:
        """Run the simulation until its runner stops."""
        if self._closed:
            raise RuntimeError("MiniSky runtime is closed")
        await self.runner.run()

    def start(self) -> asyncio.Task[None]:
        """Start the simulation runner in an owned asyncio task."""
        if self._closed:
            raise RuntimeError("MiniSky runtime is closed")
        if self._run_task is None or self._run_task.done():
            self._run_task = asyncio.create_task(self.run())
        return self._run_task

    def close(self) -> None:
        """Release synchronous resources owned by this runtime."""
        if self._closed:
            return
        self.runner.shutdown()
        self.streaming.close()
        try:
            self.plugins.shutdown()
        finally:
            self._closed = True

    async def aclose(self) -> None:
        """Stop the runner task and release all runtime-owned resources."""
        error: BaseException | None = None
        try:
            self.close()
        except BaseException as exc:  # cleanup the runner task before re-raising
            error = exc

        task = self._run_task
        if task is not None and task is not asyncio.current_task() and not task.done():
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
        self._run_task = None

        if error is not None:
            raise error

    def __enter__(self) -> MiniSky:
        """Enter a synchronous runtime lifecycle context."""
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """Close the runtime when leaving a synchronous context."""
        self.close()

    async def __aenter__(self) -> MiniSky:
        """Enter an asynchronous runtime lifecycle context."""
        return self

    async def __aexit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """Close the runtime when leaving an asynchronous context."""
        await self.aclose()
