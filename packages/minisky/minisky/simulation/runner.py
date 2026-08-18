"""Asyncio main loop that drives the simulation at a configurable speed.

Defines the `Runner`. It calls `self.simulation.step()` repeatedly at an
interval derived from the requested simulation speed, and supports
fast-forward jumps where the sleep interval is reduced to a minimum until a
target simulation time is reached. Each `MiniSky` runtime owns an instance.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

from minisky import quantities as q
from minisky.command import PositiveFiniteFloat, command
from minisky.result import Ok, Result

if TYPE_CHECKING:
    from minisky.simulation.console import ConsoleIO
    from minisky.simulation.simulation import Simulation

# TODO(abraham): make this configurable.

MIN_UPDATE_INTERVAL: q.DurationS[float] = 0.0001

# A fast-forward jump ends this many timesteps short of the requested time, so
# that the final approach to the target runs at the configured speed.
JUMP_SETTLE_STEPS = 2


class Runner:
    """Asyncio loop that drives the simulation at a configurable speed.

    Each loop iteration performs a call to `self.simulation.step()` (which
    advances simulation time by `simdt`) and then sleeps so that simulated
    time advances `speed` times faster than wall-clock time, i.e. a step
    every `simdt / speed` wall-clock seconds. During a fast-forward jump
    (see `forward`) the sleep is shortened to the minimum interval so
    the target simulation time is reached as fast as possible.
    """

    def __init__(self, simulation: Simulation, console: ConsoleIO, speed: float = 1) -> None:
        self.simulation = simulation
        self.console = console
        self.running: bool = False
        self.allow_shutdown: bool = True
        """Whether `stop` may end the runner loop."""
        self.speed = speed
        """Simulation speed factor relative to wall-clock time."""
        self.jumping: bool = False
        self.jump_to: q.SimulationTimeS[float] = 0.0  # pyright: ignore[reportGeneralTypeIssues]

    def forward(self, seconds: q.DurationS[float]) -> None:
        """Fast-forward the simulation by a number of simulation seconds.

        Activates a jump: the run loop switches to the minimum sleep interval
        until simulation time reaches the target. The jump stops
        `JUMP_SETTLE_STEPS` timesteps short of the requested time, so the
        final approach runs at the configured speed.

        Args:
            seconds: Duration of simulated time to jump forward [s].
        """
        self.jump_to = self.simulation.simt + seconds - JUMP_SETTLE_STEPS * self.simulation.simdt
        self.jumping = True

    @command(name="DTMULT", aliases=("RTF",))
    def setspeed(self, mult: PositiveFiniteFloat) -> Result[str, str]:
        """Set the simulation speed multiplier.

        The loop targets a simulation step every `simdt / speed` wall-clock
        seconds, so a larger multiplier makes simulated time advance faster
        relative to the wall clock.

        Args:
            mult: Simulation speed factor relative to real time; must be
                positive.
        """
        self.speed = mult
        return Ok(f"Simulation speed set to {mult}x")

    def prevent_shutdown(self) -> None:
        """Disable shutdown so that `stop` requests are ignored.

        Used when the runtime starts without a scenario (e.g. behind the HTTP
        API) and should keep accepting commands even after a `QUIT` command is
        issued. The simulation still moves to `END` and simulated time stops
        advancing; resume with the `OP` command.
        """
        self.allow_shutdown = False

    async def run(self) -> None:
        """Run the main simulation loop until stopped.

        Repeatedly steps the simulation, sleeping between steps so that steps
        occur every `simdt / speed` wall-clock seconds. While a fast-forward
        jump is active the sleep interval is reduced to the minimum until the
        target simulation time is reached. The loop exits when `running` is
        cleared either by `shutdown` or by `stop` if shutdown has not been
        disabled with `prevent_shutdown`.
        """
        if self.running:
            raise RuntimeError("Simulation runner is already running")

        self.console.echo("Starting simulation")
        self.running = True
        try:
            while self.running:
                if self.jumping:
                    update_interval = MIN_UPDATE_INTERVAL

                    if self.jump_to <= self.simulation.simt:
                        self.jumping = False
                        self.jump_to = 0
                else:
                    update_interval = self.simulation.simdt / self.speed

                next_time = asyncio.get_event_loop().time() + update_interval

                self.simulation.step()

                current_time = asyncio.get_event_loop().time()

                sleep_time = max(MIN_UPDATE_INTERVAL, next_time - current_time)

                await asyncio.sleep(sleep_time)
        finally:
            self.running = False

        self.console.echo("Simulation completed")

    def shutdown(self) -> None:
        """Stop the run loop regardless of scenario shutdown policy."""
        self.running = False

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
