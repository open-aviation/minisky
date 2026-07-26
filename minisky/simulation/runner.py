"""Node encapsulates the sim process, and manages process I/O.

Defines the `Runner`, the asyncio-based main loop of MiniSky. It calls
`self.simulation.step()` repeatedly at an interval derived from the requested
simulation speed, and supports fast-forward jumps where the sleep interval is
reduced to a minimum until a target simulation time is reached. A single
instance is owned by `MiniSky` and temporarily available as `minisky.runner`
through the compatibility facade.
"""

from __future__ import annotations

import asyncio
import os
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from minisky.simulation.console import ConsoleIO
    from minisky.simulation.simulation import Simulation

MIN_UPDATE_INTERVAL = 0.0001


class Runner:
    """Asyncio loop that drives the simulation at a configurable speed.

    Each loop iteration performs one call to `self.simulation.step()` (which
    advances simulation time by one `simdt`) and then sleeps so that steps
    occur every `1 / speed` wall-clock seconds. During a fast-forward jump
    (see `forward`) the sleep is shortened to the minimum interval so
    the target simulation time is reached as fast as possible.

    Attributes:
        node_id: Random 5-byte identifier for this simulation node.
        host_id: Identifier of the host this node belongs to (empty by default).
        running: True while the run loop is active.
        allow_shutdown: If False, `stop` is ignored and the loop keeps
            running (used when the simulator should idle without a scenario).
        speed: Simulation speed factor relative to real time; the loop targets
            one simulation step every `1 / speed` wall-clock seconds.
        jump: Remaining fast-forward request [s of simulation time]; 0 when
            no jump is active.
        jump_to: Target simulation time of the active fast-forward jump [s].
    """

    def __init__(self, simulation: Simulation, console: ConsoleIO, speed: float = 1) -> None:
        """Initialize the runner.

        Args:
            simulation: Simulation stepped by the run loop.
            console: Output channel used for lifecycle messages.
            speed: Simulation speed factor relative to real time, default 1.
        """
        self.simulation = simulation
        self.console = console
        self.node_id: bytes = b"\x00" + os.urandom(4)
        self.host_id: bytes = b""
        self.running: bool = False
        self.allow_shutdown: bool = True
        self.speed = speed
        self.jump: float = 0
        self.jump_to: float = 0

    def forward(self, seconds: float) -> None:
        """Fast-forward the simulation by a number of simulation seconds.

        Activates a jump: the run loop switches to the minimum sleep interval
        until simulation time reaches the target. The target is set 2 s short
        of the full jump as an action margin.

        Args:
            seconds: Amount of simulation time to jump forward [s].
        """
        self.jump_to = self.simulation.simt + seconds - 2  # -2 for the action margin
        self.jump = seconds

    def setspeed(self, mult: float) -> tuple[bool, str]:
        """Set the simulation speed multiplier (stack DTMULT command).

        The loop targets one simulation step every `1 / speed` wall-clock
        seconds, so a larger multiplier makes simulated time advance faster
        relative to the wall clock. This is the wall-clock-pacing equivalent of
        BlueSky's `DTMULT`.

        Args:
            mult: Simulation speed factor relative to real time; must be
                positive.

        Returns:
            Tuple of (success flag, message reporting the new speed, or an
            error message when the multiplier is not positive).
        """
        if mult <= 0:
            return False, "DTMULT: speed multiplier must be positive"
        self.speed = mult
        return True, f"Simulation speed set to {mult}x"

    def prevent_shutdown(self) -> None:
        """Disable shutdown so that `stop` requests are ignored.

        Used when the simulator runs without a scenario (e.g. behind the HTTP
        API) and should keep accepting commands even after a scenario ends or
        a QUIT/STOP command is issued.
        """
        self.allow_shutdown = False

    async def run(self) -> None:
        """Run the main simulation loop until stopped.

        Repeatedly steps the simulation, sleeping between steps so that steps
        occur every `1 / speed` wall-clock seconds. While a fast-forward
        jump is active the sleep interval is reduced to the minimum until the
        target simulation time is reached. The loop exits when
        `stop` sets `running` to False (and shutdown is allowed).
        """
        self.console.echo("Starting simulation")
        self.running = True

        while self.running:
            # Check if jump is active
            if self.jump > 0:
                update_interval = MIN_UPDATE_INTERVAL

                # Check if jump is completed
                if self.jump_to <= self.simulation.simt:
                    self.jump = 0
                    self.jump_to = 0
            else:
                update_interval = 1 / self.speed

            next_time = asyncio.get_event_loop().time() + update_interval

            self.simulation.step()

            current_time = asyncio.get_event_loop().time()

            sleep_time = max(MIN_UPDATE_INTERVAL, next_time - current_time)

            await asyncio.sleep(sleep_time)

        self.console.echo("Simulation completed")

    def stop(self) -> None:
        """Request the run loop to stop.

        Has no effect when shutdown has been disabled with
        `prevent_shutdown`; in that case a message is printed and the
        loop keeps running.
        """
        if self.allow_shutdown:
            self.running = False
        else:
            self.console.echo("Shutdown is prevented")
