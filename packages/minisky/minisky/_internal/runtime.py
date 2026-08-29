"""Explicit ownership root for a MiniSky runtime."""

from __future__ import annotations

from random import Random
from typing import Self

import numpy as np

from minisky._internal.active_waypoint import ActiveWaypoint
from minisky._internal.autopilot import Autopilot
from minisky._internal.config import MiniSkyConfig, data, default_user_config_toml_path
from minisky._internal.conflict.detection import ConflictDetection
from minisky._internal.conflict.mvp import MVP
from minisky._internal.conflict.resolution import ConflictResolution
from minisky._internal.console import ConsoleIO
from minisky._internal.geo_commands import GeoCommands
from minisky._internal.guidance import APorASAS
from minisky._internal.kinematics import Kinematics
from minisky._internal.navigation import NavData, Waypoints, load_navdata
from minisky._internal.performance.openap import OpenAP
from minisky._internal.plugin import PluginManager
from minisky._internal.route import RouteCommands
from minisky._internal.runner import Runner
from minisky._internal.shapes import Shapes
from minisky._internal.simulation import Simulation, SimulationState
from minisky._internal.stack import CommandStack
from minisky._internal.streaming import StreamHub, build_snapshot
from minisky._internal.traffic import Traffic
from minisky._internal.traffic_arrays import ReplaceableManager
from minisky._internal.variables import VariableExplorer
from minisky.geo import MagneticDeclination, MagneticDeclinationGrid


class MiniSky:
    """Own the primary objects that make up a simulator runtime.

    When `config` is omitted, MiniSky loads the optional default user
    config and otherwise falls back to [`MiniSkyConfig`][minisky.MiniSkyConfig]
    defaults. Navigation data defaults to the bundled core dataset.
    Magnetic declination defaults to the bundled grid.
    """

    def __init__(
        self,
        config: MiniSkyConfig | None = None,
        scenario: str | None = None,
        *,
        navdata: NavData | None = None,
        magnetic_declination: MagneticDeclination | None = None,
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
        if navdata is None:
            navdata = load_navdata(data("navigation"))
        if magnetic_declination is None:
            magnetic_declination = MagneticDeclinationGrid.load_default()
        self.magnetic_declination = magnetic_declination
        self.waypoints = Waypoints(navdata.waypoints)
        self.airports = navdata.airports
        self.airways = navdata.airways
        self.firs = navdata.firs
        self.countries = navdata.countries
        self.runway_thresholds = navdata.runway_thresholds
        self.shapes = Shapes()
        self.variables = VariableExplorer()
        self.traffic = Traffic(
            config=config,
            python_random=self.python_random,
            numpy_random=self.numpy_random,
            shapes=self.shapes,
            waypoints=self.waypoints,
            airports=self.airports,
            airways=self.airways,
            countries=self.countries,
            runway_thresholds=self.runway_thresholds,
            magnetic_declination=self.magnetic_declination,
            console=self.console,
            get_simulation=lambda: self.simulation,
            stack_command=lambda *args, **kwargs: self.commands.stack(*args, **kwargs),
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
            waypoints=self.waypoints,
            airports=self.airports,
            runway_thresholds=self.runway_thresholds,
            console=self.console,
            shapes=self.shapes,
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
            waypoints=self.waypoints,
            python_random=self.python_random,
            numpy_random=self.numpy_random,
            console=self.console,
            command_stack=self.commands,
            shapes=self.shapes,
            plugins=self.plugins,
            replaceables=self.replaceables,
            stop_runner=self._stop_runner,
            publish_tick=self.streaming.publish_tick,
        )
        self.runner = Runner(self.simulation, self.console)
        self.route_commands = RouteCommands(self.traffic)
        self.geo_commands = GeoCommands(self.magnetic_declination)
        self.variables.init(self.simulation, self.traffic)
        self.commands.mount_components(
            (
                self.console,
                self.waypoints,
                self.shapes,
                self.variables,
                self.traffic,
                self.traffic.cond,
                self.traffic.wind,
                self.traffic.cd,
                self.traffic.cr,
                self.traffic.ap,
                self.traffic.perf,
                self.traffic.groups,
                self.replaceables,
                self.plugins,
                self.commands,
                self.route_commands,
                self.geo_commands,
                self.simulation,
                self.runner,
            )
        )

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
