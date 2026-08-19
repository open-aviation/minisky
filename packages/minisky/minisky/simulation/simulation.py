"""Simulation clock and state machine that steps the simulator.

Defines the `Simulation`. Each timestep processes the command stack,
advances simulation time, runs the plugin `preupdate` hooks, updates all
aircraft in the traffic object, and runs the plugin `update` hooks. Each
`MiniSky` runtime owns an instance.
"""

from __future__ import annotations

import datetime
import time
from collections.abc import Callable
from enum import IntEnum
from random import Random
from typing import TYPE_CHECKING, Annotated, Literal

import numpy as np
from annotated_types import Ge, Le

from minisky import quantities as q
from minisky.result import Err, Ok, Result
from minisky.stack import ScenarioData
from minisky.stack_command import (
    CmdParser,
    OnOff,
    command,
)

if TYPE_CHECKING:
    from minisky.core.trafficarrays import ReplaceableManager
    from minisky.plugin.plugin import PluginManager
    from minisky.simulation.console import ConsoleIO
    from minisky.stack import CommandStack
    from minisky.tools.navdata import Navdatabase
    from minisky.tools.shapes import Shapes
    from minisky.traffic.traffic import Traffic


class SimulationState(IntEnum):
    """Simulation lifecycle states."""

    INIT = 0
    """Freshly created or reset, waiting for the first traffic or scenario command."""
    HOLD = 1
    """Paused: commands still run, but simulated time is frozen."""
    OP = 2
    """Running: simulated time advances and traffic is updated each step."""
    END = 3
    """Stopped for good: the run is over and time never advances again."""


MINSLEEP: q.DurationS[float] = 1e-3
"""Minimum sleep interval"""


Day = Annotated[int, Ge(1), Le(31)]
Month = Annotated[int, Ge(1), Le(12)]
Year = Annotated[int, Ge(1)]


def _parse_clock_time(value: str) -> datetime.time:
    return (
        datetime.datetime.strptime(value, "%H:%M:%S.%f" if "." in value else "%H:%M:%S")
        .replace(tzinfo=datetime.UTC)
        .time()
    )


ClockTimeArg = Annotated[datetime.time, CmdParser.value(_parse_clock_time, "a UTC time")]


def _calendar_datetime(
    day: int, month: int, year: int, clock: datetime.time = datetime.time()
) -> Result[datetime.datetime, str]:
    try:
        date = datetime.date(year, month, day)
    except ValueError as exc:
        return Err(str(exc))
    return Ok(datetime.datetime.combine(date, clock, tzinfo=datetime.UTC))


class Simulation:
    """Clock, state machine and per-step update driver of the simulator.

    Each call to `step` processes the pending stack commands and, while in
    `OP`, advances `simt` and the simulated UTC clock by `simdt`, runs the
    plugin `preupdate` hooks, updates the traffic and runs the plugin
    `update` hooks. State transitions are driven by the `OP`/`HOLD`/`RESET`
    and `QUIT` stack commands, which map onto `op`, `hold`, `reset` and
    `stop`.
    """

    def __init__(
        self,
        traffic: Traffic,
        navigation: Navdatabase,
        python_random: Random,
        numpy_random: np.random.RandomState,
        console: ConsoleIO,
        command_stack: CommandStack,
        shapes: Shapes,
        plugins: PluginManager,
        replaceables: ReplaceableManager,
        stop_runner: Callable[[], None],
        publish_tick: Callable[[], None],
    ) -> None:
        self.traffic = traffic
        self.navigation = navigation
        self.python_random = python_random
        self.numpy_random = numpy_random
        self.console = console
        self.commands = command_stack
        self.shapes = shapes
        self.plugins = plugins
        self.replaceables = replaceables
        self.stop_runner = stop_runner
        self.publish_tick = publish_tick
        self.state = SimulationState.INIT
        self.prevstate: SimulationState | None = None
        """Previous simulation state; currently unused."""

        self.simt: q.SimulationTimeS[float] = 0.0  # pyright: ignore[reportGeneralTypeIssues]
        self.simdt: q.DurationS[float] = 1.0  # pyright: ignore[reportGeneralTypeIssues]
        self.syst: q.WallClockTimeS[float] = 0.0  # pyright: ignore[reportGeneralTypeIssues]
        """Wall-clock pacing reference; currently unused by the runner."""

        self.utc = datetime.datetime.now(datetime.UTC)
        """Simulated UTC clock, advanced by `simdt` and settable with `TIME` and `DATE`."""

        self.rtmode: bool = False
        """Realtime request flag; currently does not alter the timestep."""

    def step(self) -> bool:
        """Perform one simulation timestep.

        Call this directly to advance the simulation yourself; `Runner` calls
        it on a wall-clock schedule instead.

        A step consists of:

        1. Auto-start: while in `INIT`, switch to `OP` as soon as there is
           traffic or there are pending scenario commands.
        2. Process the command stack (always, in every state). An awaitable
           command that has not finished ends the step here.
        3. While in `OP`: advance `simt` and the simulated UTC clock by
           `simdt` seconds, run plugin `preupdate` hooks (including
           timers), update all aircraft, then run plugin `update` hooks.
        4. Publish the runtime stream snapshot (a no-op without subscribers,
           and rate-capped), on this and on the early-return path.

        Returns:
            `False` while an awaitable stack command still owns the step boundary,
            meaning simulation time did not advance; otherwise `True`.
        """
        if self.state == SimulationState.INIT and (
            self.traffic.ntraf > 0 or self.commands.get_scendata().commands
        ):
            self.op()

        if not self.commands.process():
            self.publish_tick()
            return False

        if self.state == SimulationState.OP:
            self.simt += self.simdt

            self.utc += datetime.timedelta(seconds=self.simdt)

            # timers + preupdate hooks
            self.plugins.preupdate()

            self.traffic.update()

            # post-update hooks
            self.plugins.update()

        # Publish after command and state processing in every simulation state.
        # This is a no-op when the runtime has no stream subscribers.
        self.publish_tick()
        return True

    @command(name="QUIT", aliases=("CLOSE", "END", "EXIT", "Q", "STOP"))
    def stop(self) -> None:
        """Stop the simulation.

        Sets the simulation state to `END` and asks the runner to exit its
        loop. If the runner was configured with
        `minisky.simulation.runner.Runner.prevent_shutdown`, the loop
        keeps running and only the state changes.
        """
        self.state = SimulationState.END
        self.stop_runner()

    @command(name="OP", aliases=("CONTINUE", "RUN", "START"))
    def op(self) -> None:
        """Set simulation state to OPERATE.

        Resumes (or starts) advancing simulation time. Also re-anchors the
        system time reference `syst` to the current wall-clock time plus one
        simulation step.
        """
        self.syst = time.time() + self.simdt
        self.state = SimulationState.OP
        self.console.echo("Simulation running")

    @command(name="HOLD", aliases=("PAUSE",))
    def hold(self) -> None:
        """Set simulation state to HOLD.

        Pauses the advance of simulation time and triggers the plugin `hold`
        hooks. Stack commands are still processed while holding, so the
        simulation can be resumed with the `OP` command.
        """
        self.syst = time.time() + self.simdt
        self.state = SimulationState.HOLD
        self.plugins.hold()
        self.console.echo("Simulation paused")

    @command(name="RESET")
    def reset(self) -> None:
        """Reset all simulation objects (stack `RESET` command).

        Returns the simulation to its initial state: simulation time back to
        0 s, timestep back to 1 s, the simulated UTC clock to today at
        00:00:00, and all traffic, stack, navigation database, area filters,
        console output, replaceable entities (autopilot, performance models,
        etc.) and plugin timers/hooks reset to their defaults.
        """
        self.state = SimulationState.INIT
        self.syst = 0
        self.simt = 0
        self.simdt = 1
        self.utc = datetime.datetime.now(datetime.UTC).replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        self.navigation.reset()
        self.traffic.reset()
        self.commands.reset()
        self.shapes.reset()
        self.console.reset()
        self.replaceables.reset()
        self.plugins.reset()
        self.console.echo("Simulation reset")

    @command(name="REALTIME", aliases=("RT",))
    def realtime_status(self) -> Result[str, str]:
        """Report realtime mode."""
        return Ok(f"Realtime mode is {'on' if self.rtmode else 'off'}")

    @command(name="REALTIME")
    def set_realtime(self, flag: OnOff) -> Result[str, str]:
        """Enable or disable realtime mode."""
        self.rtmode = flag
        return Ok(f"Realtime mode is {'on' if self.rtmode else 'off'}")

    def event(
        self,
        eventname: bytes,
        eventdata: str | ScenarioData,
        sender_rte: bytes | None,
    ) -> bool:
        """Handle events coming from the network.

        Supports two event types: `b"STACK"`, which appends a single stack
        command line to the command stack, and `b"BATCH"`, which resets the
        simulation, installs a full scenario (times + commands) on the stack,

        Args:
            eventname: Event tag: `b"STACK"` or `b"BATCH"`.
            eventdata: Command text for `STACK`, or scenario data for `BATCH`.
            sender_rte: Sending client route, forwarded as the stack command sender id.

        Returns:
            Whether the event tag was recognized and processed.
        """
        event_processed = False

        if eventname == b"STACK":
            if not isinstance(eventdata, str):
                raise TypeError("STACK event data must be command text")
            self.commands.stack(eventdata, sender_id=sender_rte)
            event_processed = True

        elif eventname == b"BATCH":
            if not isinstance(eventdata, ScenarioData):
                raise TypeError("BATCH event data must be ScenarioData")
            self.reset()
            self.commands.set_scendata(eventdata)
            self.op()
            event_processed = True

        return event_processed

    @command(name="TIME")
    def report_time(self) -> Result[str, str]:
        """Report the current simulation UTC timestamp."""
        return Ok(f"Simulation UTC {self.utc}")

    @command(name="TIME")
    def set_time_run(self, _source: Literal["RUN"]) -> Result[str, str]:
        """Set simulation UTC to the start of the current UTC day."""
        self.utc = datetime.datetime.now(datetime.UTC).replace(
            hour=0, minute=0, second=0, microsecond=0
        )
        return Ok(f"Simulation UTC {self.utc}")

    @command(name="TIME")
    def set_time_real(self, _source: Literal["REAL"]) -> Result[str, str]:
        """Set simulation UTC from the current local wall clock."""
        self.utc = datetime.datetime.now(datetime.UTC).astimezone().replace(microsecond=0)
        return Ok(f"Simulation UTC {self.utc}")

    @command(name="TIME")
    def set_time_utc(self, _source: Literal["UTC"]) -> Result[str, str]:
        """Set simulation UTC from the current UTC wall clock."""
        self.utc = datetime.datetime.now(datetime.UTC).replace(microsecond=0)
        return Ok(f"Simulation UTC {self.utc}")

    @command(name="TIME")
    def set_time_value(self, time: ClockTimeArg) -> Result[str, str]:
        """Set only the time-of-day, preserving BlueSky's 1900-01-01 date behavior."""
        self.utc = datetime.datetime.combine(datetime.date(1900, 1, 1), time, tzinfo=datetime.UTC)
        return Ok(f"Simulation UTC {self.utc}")

    @command(name="DATE")
    def report_date(self) -> Result[str, str]:
        """Report the current simulation UTC timestamp."""
        return Ok(f"Simulation UTC {self.utc}")

    @command(name="DATE")
    def set_date(self, day: Day, month: Month, year: Year) -> Result[str, str]:
        """Set the calendar date at midnight UTC."""
        result = _calendar_datetime(day, month, year)
        if isinstance(result, Err):
            return result
        self.utc = result.ok()
        return Ok(f"Simulation UTC {self.utc}")

    @command(name="DATE")
    def set_date_time(
        self, day: Day, month: Month, year: Year, time: ClockTimeArg
    ) -> Result[str, str]:
        """Set the calendar date and UTC time."""
        result = _calendar_datetime(day, month, year, time)
        if isinstance(result, Err):
            return result
        self.utc = result.ok()
        return Ok(f"Simulation UTC {self.utc}")

    @command(name="SEED")
    def setseed(self, value: int) -> None:
        """Set the random seed for this simulation (stack SEED command).

        Seeds this runtime's Python and NumPy generators so stochastic
        scenario elements are reproducible without affecting other runtimes.
        """
        self.python_random.seed(value)
        self.numpy_random.seed(value)
        self.console.echo("random seed set")
