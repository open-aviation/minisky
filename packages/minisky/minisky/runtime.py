"""Explicit ownership root for a MiniSky runtime."""

from __future__ import annotations

from random import Random
from typing import Self

import numpy as np

from minisky.core.config import MiniSkyConfig, data, default_user_config_toml_path
from minisky.core.trafficarrays import ReplaceableManager
from minisky.core.varexplorer import VariableExplorer
from minisky.plugin import PluginManager
from minisky.simulation import ConsoleIO, Runner, Simulation, SimulationState
from minisky.stack import CommandStack
from minisky.streaming import StreamHub, build_snapshot
from minisky.tools.areafilter import AreaFilter
from minisky.tools.navdata import Navdatabase
from minisky.traffic import Traffic
from minisky.traffic.activewpdata import ActiveWaypoint
from minisky.traffic.aporasas import APorASAS
from minisky.traffic.asas import MVP, ConflictDetection, ConflictResolution
from minisky.traffic.autopilot import Autopilot
from minisky.traffic.kinematics import Kinematics
from minisky.traffic.performance.perfoap import OpenAP


class MiniSky:
    """Own the primary objects that make up a simulator runtime.

    When `config` is omitted, MiniSky loads the optional default user
    config and otherwise falls back to [`MiniSkyConfig`][minisky.MiniSkyConfig]
    defaults.
    """

    def __init__(
        self,
        config: MiniSkyConfig | None = None,
        scenario: str | None = None,
    ) -> None:
        if config is None:
            try:
                config = MiniSkyConfig.from_path(default_user_config_toml_path())
            except FileNotFoundError:
                config = MiniSkyConfig()
        self.config = config
        self._closed = False
        self.python_random = Random()
        self.numpy_random = np.random.RandomState()
        self.console = ConsoleIO(lambda: self.simulation.state == SimulationState.OP)
        self.navigation = Navdatabase(data("navigation"), self.console)
        self.areas = AreaFilter()
        self.variables = VariableExplorer()
        self.traffic = Traffic(
            config=config,
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
            bases=(
                ActiveWaypoint,
                APorASAS,
                Autopilot,
                ConflictDetection,
                ConflictResolution,
                Kinematics,
                OpenAP,
            ),
            core=(MVP,),
        )
        self.plugins = PluginManager(
            config=config,
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

    @staticmethod
    def _raise_errors(message: str, errors: list[Exception]) -> None:
        if len(errors) == 1:
            raise errors[0]
        if errors:
            raise ExceptionGroup(message, errors)

    def close(self) -> None:
        """Close synchronous resources when no plugin lifespan is active."""
        if self._closed:
            return
        if self.commands.command_pending or self.plugins.requires_async_close:
            raise RuntimeError("active asynchronous work requires await runtime.aclose()")

        errors: list[Exception] = []
        for cleanup in (self.runner.shutdown, self.streaming.close, self.plugins.close):
            try:
                cleanup()
            except Exception as exc:  # ruff: ignore[BLE001] aggregate resource failures
                errors.append(exc)

        self._closed = True
        self._raise_errors("MiniSky cleanup failed", errors)

    async def aclose(self) -> None:
        """Stop the runner and close runtime-owned asynchronous resources."""
        if self._closed:
            return
        errors: list[Exception] = []
        self.runner.shutdown()

        try:
            await self.commands.aclose()
        except Exception as exc:  # ruff: ignore[BLE001] aggregate resource failures
            errors.append(exc)
        try:
            await self.plugins.aclose()
        except Exception as exc:  # ruff: ignore[BLE001] aggregate resource failures
            errors.append(exc)
        try:
            self.streaming.close()
        except Exception as exc:  # ruff: ignore[BLE001] aggregate resource failures
            errors.append(exc)

        self._closed = True
        self._raise_errors("MiniSky shutdown failed", errors)

    def __enter__(self) -> Self:
        """Enter a synchronous runtime lifecycle context."""
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """Close the runtime when leaving a synchronous context."""
        self.close()

    async def __aenter__(self) -> Self:
        """Enter an asynchronous runtime lifecycle context."""
        return self

    async def __aexit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """Close the runtime when leaving an asynchronous context."""
        await self.aclose()
