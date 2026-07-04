"""BlueSky simulation control object.

Defines the :class:`Simulation` class, the central clock and state machine of
the simulator. It advances simulation time, processes the command stack,
triggers plugin pre-/post-update hooks, and updates all aircraft in the
traffic object once per timestep. A single instance is created by
:func:`minisky.init` and made available as ``minisky.sim``.
"""

import datetime
import time
from random import seed
from typing import Any

import numpy as np

# Local imports
import minisky
from minisky.core.trafficarrays import reset_replaceables
from minisky.plugin import PluginManager
from minisky.tools import areafilter

# Minimum sleep interval
MINSLEEP = 1e-3


class Simulation:
    """The simulation object: clock, state machine, and per-step update driver.

    Holds simulation time and state, and advances the simulation one timestep
    at a time. Each :meth:`step` processes pending stack commands and, while
    operating, increments simulation time, triggers plugin hooks and updates
    the traffic. State transitions are driven by the ``OP``/``HOLD``/``RESET``
    and ``QUIT`` stack commands, which map onto :meth:`op`, :meth:`hold`,
    :meth:`reset` and :meth:`stop`.

    Attributes:
        state: Current simulation state, one of ``minisky.INIT``,
            ``minisky.HOLD``, ``minisky.OP`` or ``minisky.END``.
        prevstate: Previous simulation state (unused placeholder).
        simt: Elapsed simulation time [s].
        simdt: Simulation timestep [s].
        syst: System (wall-clock) time reference [s].
        utc: Simulated UTC clock time as a ``datetime``; settable with
            :meth:`setutc`.
        rtmode: Flag indicating whether the timestep may be varied to keep
            the simulation running in real time.
        clients: Set of known client identifiers connected to this simulation.
    """

    def __init__(self) -> None:
        self.state = minisky.INIT
        self.prevstate = None

        # Simulation time [seconds]
        self.simt: float = 0

        # Simulation timestep [seconds]
        self.simdt: float = 1

        # System time [seconds]
        self.syst: float = 0

        # Simulated UTC clock time, can be set by setutc()
        self.utc: datetime.datetime = datetime.datetime.today()

        # Flag indicating running at fixed rate or fast time

        # Flag indicating whether timestep can be varied to ensure realtime op
        self.rtmode: bool = False

        # Keep track of known clients
        self.clients: set[Any] = set()

    def step(self) -> None:
        """Perform one simulation timestep.

        Call this function instead of update if you don't want to run with a fixed
        real-time rate.

        A step consists of:

        1. Auto-start: while in ``INIT``, switch to ``OP`` as soon as there is
           traffic or there are pending scenario commands.
        2. Process the command stack (always, in every state).
        3. While in ``OP``: advance ``simt`` and the simulated UTC clock by
           ``simdt`` seconds, run plugin ``preupdate`` hooks (including
           timers), update all aircraft, then run plugin ``update`` hooks.
        """
        # Simulation starts as soon as there is traffic, or pending commands
        if self.state == minisky.INIT and (
            minisky.traf.ntraf > 0 or len(minisky.stack.get_scendata()[0]) > 0
        ):
            self.op()

        # Always update stack
        minisky.stack.process()

        if self.state == minisky.OP:
            self.simt += self.simdt

            # Update UTC time
            self.utc += datetime.timedelta(seconds=self.simdt)

            # Plugin pre-update (timers + preupdate hooks)
            PluginManager.preupdate()

            minisky.traf.update()

            # Plugin post-update hooks
            PluginManager.update()

    def stop(self) -> None:
        """Stop the simulation (stack STOP/QUIT command).

        Sets the simulation state to ``END`` and asks the runner to exit its
        loop. If the runner was configured with
        :meth:`~minisky.simulation.runner.Runner.prevent_shutdown`, the loop
        keeps running and only the state changes.
        """
        self.state = minisky.END
        minisky.runner.stop()

    def op(self) -> None:
        """Set simulation state to OPERATE (stack OP command).

        Resumes (or starts) advancing simulation time. Also re-anchors the
        system time reference ``syst`` to the current wall-clock time plus one
        timestep [s].
        """
        self.syst = time.time() + self.simdt
        self.state = minisky.OP
        minisky.scr.echo("Simulation running")

    def hold(self) -> None:
        """Set simulation state to HOLD (stack HOLD command).

        Pauses the advance of simulation time and triggers the plugin ``hold``
        hooks. Stack commands are still processed while holding, so the
        simulation can be resumed with the ``OP`` command.
        """
        self.syst = time.time() + self.simdt
        self.state = minisky.HOLD
        PluginManager.hold()
        minisky.scr.echo("Simulation paused")

    def reset(self) -> None:
        """Reset all simulation objects (stack RESET command).

        Returns the simulation to its initial state: simulation time back to
        0 s, timestep back to 1 s, the simulated UTC clock to today at
        00:00:00, and all traffic, stack, navigation database, area filters,
        console output, replaceable entities (autopilot, performance models,
        etc.) and plugin timers/hooks reset to their defaults.
        """
        self.state = minisky.INIT
        self.syst = 0
        self.simt = 0
        self.simdt = 1
        self.utc = datetime.datetime.utcnow().replace(hour=0, minute=0, second=0, microsecond=0)
        minisky.navdb.reset()
        minisky.traf.reset()
        minisky.stack.reset()
        areafilter.reset()
        minisky.scr.reset()
        # Reset replaceables (Autopilot, PerfBase, etc.) to defaults
        reset_replaceables()
        # Reset plugins (timers + reset hooks)
        PluginManager.reset()
        minisky.scr.echo("Simulation reset")

    def realtime(self, flag: bool | None = None) -> tuple[bool, str]:
        """Get or set realtime mode (stack REALTIME command).

        In realtime mode the timestep may be varied to keep the simulation
        synchronized with the wall clock.

        Args:
            flag: ``True``/``False`` to enable or disable realtime mode, or
                ``None`` to only report the current setting.

        Returns:
            Tuple of (success flag, message stating whether realtime mode is
            on or off).
        """
        if flag is not None:
            self.rtmode = flag

        return True, "Realtime mode is o" + ("n" if self.rtmode else "ff")

    def event(self, eventname: bytes, eventdata: Any, sender_rte: Any) -> bool:
        """Handle events coming from the network.

        Supports two event types: ``b"STACK"``, which appends a single stack
        command line to the command stack, and ``b"BATCH"``, which resets the
        simulation, installs a full scenario (times + commands) on the stack,
        and immediately starts operating.

        Args:
            eventname: Event type identifier as bytes (``b"STACK"`` or
                ``b"BATCH"``).
            eventdata: Event payload; the command string for ``STACK``, or a
                dict with ``scentime`` (command times [s]) and ``scencmd``
                (command strings) for ``BATCH``.
            sender_rte: Route/identifier of the sending client, passed on as
                the stack command's sender id.

        Returns:
            bool: True if the event was recognized and processed.
        """
        # Keep track of event processing
        event_processed = False

        if eventname == b"STACK":
            # We received a single stack command. Add it to the existing stack
            minisky.stack.stack(eventdata, sender_id=sender_rte)
            event_processed = True

        elif eventname == b"BATCH":
            # We are in a batch simulation, and received an entire scenario. Assign it to the stack.
            self.reset()
            minisky.stack.set_scendata(eventdata["scentime"], eventdata["scencmd"])
            self.op()
            event_processed = True

        return event_processed

    def setutc(self, *args) -> tuple[bool, str]:
        """Set the simulated UTC clock time (stack UTC/DATE command).

        Usage: UTC [RUN | REAL | UTC | HH:MM:SS[.ff] | day month year [HH:MM:SS[.ff]]]

        Accepted argument forms:

        - no arguments: leave the clock unchanged (the new value is reported).
        - ``RUN``: today's date at 00:00:00 UTC.
        - ``REAL``: current local date and time.
        - ``UTC``: current UTC date and time.
        - a time string ``HH:MM:SS`` or ``HH:MM:SS.ff``: set the clock time.
        - ``day month year``: set the date (three integers).
        - ``day month year timestring``: set both date and time.

        Args:
            *args: Zero, one, three, or four arguments as described above.

        Returns:
            Tuple of (success flag, message with the resulting simulation UTC
            time, or an error message when parsing failed).
        """
        if not args:
            pass  # avoid error message, just give time

        elif len(args) == 1:
            if args[0].upper() == "RUN":
                self.utc = datetime.datetime.utcnow().replace(
                    hour=0, minute=0, second=0, microsecond=0
                )

            elif args[0].upper() == "REAL":
                self.utc = datetime.datetime.today().replace(microsecond=0)

            elif args[0].upper() == "UTC":
                self.utc = datetime.datetime.utcnow().replace(microsecond=0)

            else:
                try:
                    self.utc = datetime.datetime.strptime(
                        args[0], "%H:%M:%S.%f" if "." in args[0] else "%H:%M:%S"
                    )
                except ValueError:
                    return False, "Input time invalid"

        elif len(args) == 3:
            day, month, year = args
            try:
                self.utc = datetime.datetime(year, month, day)
            except ValueError:
                return False, "Input date invalid."
        elif len(args) == 4:
            day, month, year, timestring = args
            try:
                self.utc = datetime.datetime.strptime(
                    f"{year},{month},{day},{timestring}",
                    ("%Y,%m,%d,%H:%M:%S.%f" if "." in timestring else "%Y,%m,%d,%H:%M:%S"),
                )
            except ValueError:
                return False, "Input date invalid."
        else:
            return False, "Syntax error"

        return True, "Simulation UTC " + str(self.utc)

    @staticmethod
    def setseed(value: int) -> None:
        """Set the random seed for this simulation (stack SEED command).

        Seeds both Python's :mod:`random` module and NumPy's random generator
        so that stochastic scenario elements are reproducible.

        Args:
            value: Integer seed value.
        """
        seed(value)
        np.random.seed(value)
        minisky.scr.echo("random seed set")
