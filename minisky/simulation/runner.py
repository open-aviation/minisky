"""Node encapsulates the sim process, and manages process I/O.

Defines the :class:`Runner`, the asyncio-based main loop of MiniSky. It calls
``minisky.sim.step()`` repeatedly at an interval derived from the requested
simulation speed, and supports fast-forward jumps where the sleep interval is
reduced to a minimum until a target simulation time is reached. A single
instance is created by :func:`minisky.init` and available as
``minisky.runner``.
"""

import asyncio
import os
from typing import Any

import minisky

MIN_UPDATE_INTERVAL = 0.0001


class Runner:
    """Asyncio loop that drives the simulation at a configurable speed.

    Each loop iteration performs one call to ``minisky.sim.step()`` (which
    advances simulation time by one ``simdt``) and then sleeps so that steps
    occur every ``1 / speed`` wall-clock seconds. During a fast-forward jump
    (see :meth:`forward`) the sleep is shortened to the minimum interval so
    the target simulation time is reached as fast as possible.

    Attributes:
        node_id: Random 5-byte identifier for this simulation node.
        host_id: Identifier of the host this node belongs to (empty by default).
        running: True while the run loop is active.
        allow_shutdown: If False, :meth:`stop` is ignored and the loop keeps
            running (used when the simulator should idle without a scenario).
        speed: Simulation speed factor relative to real time; the loop targets
            one simulation step every ``1 / speed`` wall-clock seconds.
        jump: Remaining fast-forward request [s of simulation time]; 0 when
            no jump is active.
        jump_to: Target simulation time of the active fast-forward jump [s].
    """

    def __init__(self, **kwargs: Any) -> None:
        """Initialize the runner.

        Args:
            **kwargs: Optional settings. Supports ``speed`` (simulation speed
                factor relative to real time, default 1).
        """
        self.node_id: bytes = b"\x00" + os.urandom(4)
        self.host_id: bytes = b""
        self.running: bool = False
        self.allow_shutdown: bool = True
        self.speed = kwargs.get("speed", 1)
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
        self.jump_to = minisky.sim.simt + seconds - 2  #  -2 for the action margin
        self.jump = seconds

    def prevent_shutdown(self) -> None:
        """Disable shutdown so that :meth:`stop` requests are ignored.

        Used when the simulator runs without a scenario (e.g. behind the HTTP
        API) and should keep accepting commands even after a scenario ends or
        a QUIT/STOP command is issued.
        """
        self.allow_shutdown = False

    async def run(self) -> None:
        """Run the main simulation loop until stopped.

        Repeatedly steps the simulation, sleeping between steps so that steps
        occur every ``1 / speed`` wall-clock seconds. While a fast-forward
        jump is active the sleep interval is reduced to the minimum until the
        target simulation time is reached. The loop exits when
        :meth:`stop` sets ``running`` to False (and shutdown is allowed).
        """
        print("staring simulation")
        self.running = True

        while self.running:
            # Check if jump is active
            if self.jump > 0:
                update_interval = MIN_UPDATE_INTERVAL

                # Check if jump is completed
                if self.jump_to <= minisky.sim.simt:
                    self.jump = 0
                    self.jump_to = 0
            else:
                update_interval = 1 / self.speed

            next_time = asyncio.get_event_loop().time() + update_interval

            minisky.sim.step()

            current_time = asyncio.get_event_loop().time()

            sleep_time = max(MIN_UPDATE_INTERVAL, next_time - current_time)

            await asyncio.sleep(sleep_time)

        print("simulation completed")

    def stop(self) -> None:
        """Request the run loop to stop.

        Has no effect when shutdown has been disabled with
        :meth:`prevent_shutdown`; in that case a message is printed and the
        loop keeps running.
        """
        if self.allow_shutdown:
            self.running = False
        else:
            print("Shutdown is prevented")
