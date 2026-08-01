"""Explicit ownership root for a MiniSky runtime."""

from __future__ import annotations

import asyncio
from random import Random

import numpy as np

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
from minisky.traffic.asas import MVP, ConflictDetection, ConflictResolution
from minisky.traffic.autopilot import Autopilot
from minisky.traffic.performance.perfoap import OpenAP


class MiniSky:
    """Own the primary objects that make up a simulator runtime."""

    def __init__(self, settings: MiniSkySettings, scenario: str | None = None) -> None:
        self.settings = settings
        self._run_task: asyncio.Task[None] | None = None
        self._closed = False
        self.python_random = Random()
        self.numpy_random = np.random.RandomState()
        self.console = ConsoleIO(lambda: self.simulation.state == SimulationState.OP)
        self.navigation = Navdatabase(data("navigation"), self.console)
        self.areas = AreaFilter()
        self.variables = VariableExplorer()
        self.traffic = Traffic(
            settings=settings,
            python_random=self.python_random,
            numpy_random=self.numpy_random,
            areas=self.areas,
            navigation=self.navigation,
            console=self.console,
            get_simulation=lambda: self.simulation,
            stack_command=lambda *args, **kwargs: self.commands.stack(*args, **kwargs),
            get_command_registry=lambda: self.commands.cmddict,
            select_implementation=lambda base, impl: self.replaceables.select(base, impl),
        )
        self.replaceables = ReplaceableManager(
            self.traffic,
            lambda: self.commands.cmddict,
            bases=(Autopilot, ConflictDetection, ConflictResolution, OpenAP),
            core=(MVP,),
        )
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
            python_random=self.python_random,
            numpy_random=self.numpy_random,
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

    async def run(self) -> None:
        """Run the simulation until its runner stops."""
        if self._closed:
            raise RuntimeError("MiniSky runtime is closed")
        await self.runner.run()

    # TODO(abraham): move background task ownership to callers and remove start()
    def start(self) -> asyncio.Task[None]:
        """Start the simulation runner in an owned asyncio task."""
        if self._closed:
            raise RuntimeError("MiniSky runtime is closed")
        task = self._run_task
        if task is not None:
            if not task.done():
                return task
            self._run_task = None
            task.result()

        self._run_task = asyncio.create_task(self.run())
        return self._run_task

    @staticmethod
    def _raise_errors(message: str, errors: list[Exception]) -> None:
        if len(errors) == 1:
            raise errors[0]
        if errors:
            raise ExceptionGroup(message, errors)

    def close(self) -> None:
        """Close a runtime without active plugin lifespans."""
        if self._closed:
            return
        if self.plugins.requires_async_close:
            raise RuntimeError("active plugins require await runtime.aclose()")

        errors: list[Exception] = []
        for cleanup in (self.runner.shutdown, self.streaming.close, self.plugins.close):
            try:
                cleanup()
            except Exception as exc:
                errors.append(exc)

        task = self._run_task
        try:
            current = asyncio.current_task()
        except RuntimeError:
            current = None
        if task is not None and task is not current:
            if task.done():
                self._run_task = None
                try:
                    task.result()
                except asyncio.CancelledError:
                    pass
                except Exception as exc:
                    errors.append(exc)
            else:
                task.cancel()

        self._closed = True
        self._raise_errors("MiniSky cleanup failed", errors)

    async def aclose(self) -> None:
        """Stop the runner and close plugin lifespans and synchronous resources."""
        if self._closed:
            return
        errors: list[Exception] = []
        self.runner.shutdown()

        task = self._run_task
        if task is not None and task is not asyncio.current_task():
            if not task.done():
                task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception as exc:
                errors.append(exc)
            finally:
                self._run_task = None

        try:
            await self.plugins.aclose()
        except Exception as exc:
            errors.append(exc)
        try:
            self.streaming.close()
        except Exception as exc:
            errors.append(exc)

        self._closed = True
        self._raise_errors("MiniSky shutdown failed", errors)

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
