"""BlueSky simulation control object.

Defines the `Simulation` class, the central clock and state machine of
the simulator. It advances simulation time, processes the command stack,
triggers plugin pre-/post-update hooks, and updates all aircraft in the
traffic object once per timestep. Each `MiniSky` runtime owns an instance.
"""

from __future__ import annotations

import datetime
import time
from collections.abc import Callable
from enum import IntEnum
from random import Random
from typing import TYPE_CHECKING, Annotated, Any, Literal

import numpy as np
from annotated_types import Ge, Le

from minisky.command import (
    ArgumentIssue,
    CmdParser,
    OnOff,
    Parsed,
    ParseResult,
    command,
    next_argument,
)
from minisky.result import Err, Ok, Result

if TYPE_CHECKING:
    from minisky.core.trafficarrays import ReplaceableManager
    from minisky.plugin import PluginManager
    from minisky.simulation.console import ConsoleIO
    from minisky.stack import CommandStack
    from minisky.tools.areafilter import AreaFilter
    from minisky.tools.navdata import Navdatabase
    from minisky.traffic import Traffic


class SimulationState(IntEnum):
    """Simulation lifecycle states."""

    INIT = 0
    HOLD = 1
    OP = 2
    END = 3


# Minimum sleep interval
MINSLEEP = 1e-3


Day = Annotated[int, Ge(1), Le(31)]
Month = Annotated[int, Ge(1), Le(12)]
Year = Annotated[int, Ge(1)]


def _clock_time(value: str) -> Result[datetime.time, ArgumentIssue]:
    try:
        parsed = (
            datetime.datetime.strptime(value, "%H:%M:%S.%f" if "." in value else "%H:%M:%S")
            .replace(tzinfo=datetime.UTC)
            .time()
        )
    except ValueError:
        return Err(ArgumentIssue.expected("a UTC time", value))
    return Ok(parsed)


def _parse_clock_time(text: str) -> ParseResult[datetime.time]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    if isinstance(clock := _clock_time(token.value), Err):
        return Err(ArgumentIssue(clock.err().message, token.span))
    return Ok(Parsed(clock.ok(), token.remainder, token.span))


ClockTimeArg = Annotated[datetime.time, CmdParser(_parse_clock_time)]


def _calendar_datetime(
    day: int, month: int, year: int, clock: datetime.time = datetime.time()
) -> Result[datetime.datetime, str]:
    try:
        date = datetime.date(year, month, day)
    except ValueError as exc:
        return Err(str(exc))
    return Ok(datetime.datetime.combine(date, clock, tzinfo=datetime.UTC))


class Simulation:
    """The simulation object: clock, state machine, and per-step update driver.

    Holds simulation time and state, and advances the simulation one timestep
    at a time. Each `step` processes pending stack commands and, while
    operating, increments simulation time, triggers plugin hooks and updates
    the traffic. State transitions are driven by the `OP`/`HOLD`/`RESET`
    and `QUIT` stack commands, which map onto `op`, `hold`,
    `reset` and `stop`.

    Attributes:
        state: Current [`SimulationState`][minisky.simulation.simulation.SimulationState] value.
        prevstate: Previous simulation state (unused placeholder).
        simt: Elapsed simulation time [s].
        simdt: Simulation timestep [s].
        syst: System (wall-clock) time reference [s].
        utc: Simulated UTC clock time as a `datetime`; settable with the
            `TIME` and `DATE` commands.
        rtmode: Flag indicating whether the timestep may be varied to keep
            the simulation running in real time.
        clients: Set of known client identifiers connected to this simulation.
    """

    def __init__(
        self,
        traffic: Traffic,
        navigation: Navdatabase,
        python_random: Random,
        numpy_random: np.random.RandomState,
        console: ConsoleIO,
        command_stack: CommandStack,
        areas: AreaFilter,
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
        self.areas = areas
        self.plugins = plugins
        self.replaceables = replaceables
        self.stop_runner = stop_runner
        self.publish_tick = publish_tick
        self.state = SimulationState.INIT
        self.prevstate: SimulationState | None = None

        # Simulation time [seconds]
        self.simt: float = 0

        # Simulation timestep [seconds]
        self.simdt: float = 1

        # System time [seconds]
        self.syst: float = 0

        # simulated utc clock time (timezone-aware), set by time/date commands
        self.utc: datetime.datetime = datetime.datetime.now(datetime.UTC)

        # Flag indicating running at fixed rate or fast time

        # Flag indicating whether timestep can be varied to ensure realtime op
        self.rtmode: bool = False

        # Keep track of known clients
        self.clients: set[Any] = set()

    def step(self) -> bool:
        """Perform one simulation timestep.

        Call this function instead of update if you don't want to run with a fixed
        real-time rate.

        A step consists of:

        1. Auto-start: while in `INIT`, switch to `OP` as soon as there is
           traffic or there are pending scenario commands.
        2. Process the command stack (always, in every state).
        3. While in `OP`: advance `simt` and the simulated UTC clock by
           `simdt` seconds, run plugin `preupdate` hooks (including
           timers), update all aircraft, then run plugin `update` hooks.
        4. Publish the runtime stream snapshot when subscribers are present.
        """
        # Simulation starts as soon as there is traffic, or pending commands
        if self.state == SimulationState.INIT and (
            self.traffic.ntraf > 0 or len(self.commands.get_scendata()[0]) > 0
        ):
            self.op()

        # An awaitable stack command owns this boundary until it completes.
        if not self.commands.process():
            self.publish_tick()
            return False

        if self.state == SimulationState.OP:
            self.simt += self.simdt

            # Update UTC time
            self.utc += datetime.timedelta(seconds=self.simdt)

            # Plugin pre-update (timers + preupdate hooks)
            self.plugins.preupdate()

            self.traffic.update()

            # Plugin post-update hooks
            self.plugins.update()

        # Publish after command and state processing in every simulation state.
        # This is a no-op when the runtime has no stream subscribers.
        self.publish_tick()
        return True

    def stop(self) -> None:
        """Stop the simulation (stack STOP/QUIT command).

        Sets the simulation state to `END` and asks the runner to exit its
        loop. If the runner was configured with
        `minisky.simulation.runner.Runner.prevent_shutdown`, the loop
        keeps running and only the state changes.
        """
        self.state = SimulationState.END
        self.stop_runner()

    def op(self) -> None:
        """Set simulation state to OPERATE (stack OP command).

        Resumes (or starts) advancing simulation time. Also re-anchors the
        system time reference `syst` to the current wall-clock time plus one
        timestep [s].
        """
        self.syst = time.time() + self.simdt
        self.state = SimulationState.OP
        self.console.echo("Simulation running")

    def hold(self) -> None:
        """Set simulation state to HOLD (stack HOLD command).

        Pauses the advance of simulation time and triggers the plugin `hold`
        hooks. Stack commands are still processed while holding, so the
        simulation can be resumed with the `OP` command.
        """
        self.syst = time.time() + self.simdt
        self.state = SimulationState.HOLD
        self.plugins.hold()
        self.console.echo("Simulation paused")

    def reset(self) -> None:
        """Reset all simulation objects (stack RESET command).

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
        self.areas.reset()
        self.console.reset()
        # Reset replaceables (Autopilot, PerfBase, etc.) to defaults
        self.replaceables.reset()
        # Reset plugins (timers + reset hooks)
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

    def event(self, eventname: bytes, eventdata: Any, sender_rte: Any) -> bool:
        """Handle events coming from the network.

        Supports two event types: `b"STACK"`, which appends a single stack
        command line to the command stack, and `b"BATCH"`, which resets the
        simulation, installs a full scenario (times + commands) on the stack,
        and immediately starts operating.

        Args:
            eventname: Event type identifier as bytes (`b"STACK"` or
                `b"BATCH"`).
            eventdata: Event payload; the command string for `STACK`, or a
                dict with `scentime` (command times [s]) and `scencmd`
                (command strings) for `BATCH`.
            sender_rte: Route/identifier of the sending client, passed on as
                the stack command's sender id.

        Returns:
            bool: True if the event was recognized and processed.
        """
        # Keep track of event processing
        event_processed = False

        if eventname == b"STACK":
            # We received a single stack command. Add it to the existing stack
            self.commands.stack(eventdata, sender_id=sender_rte)
            event_processed = True

        elif eventname == b"BATCH":
            # We are in a batch simulation, and received an entire scenario. Assign it to the stack.
            self.reset()
            self.commands.set_scendata(eventdata["scentime"], eventdata["scencmd"])
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
        """Set only the time-of-day using the historical 1900-01-01 date."""
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

        Args:
            value: Integer seed value.
        """
        self.python_random.seed(value)
        self.numpy_random.seed(value)
        self.console.echo("random seed set")
