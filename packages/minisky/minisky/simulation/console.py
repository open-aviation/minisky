"""Console I/O for a MiniSky runtime."""

from __future__ import annotations

import asyncio
import io
import sys
import traceback
from collections.abc import Callable
from typing import Self

from colorama import Fore, Style

from minisky.command import Text, command
from minisky.values import LatLonDegrees

ConsoleCallback = Callable[[str], None]


class ConsoleSubscription:
    """Owned console callback registration."""

    def __init__(self, console: ConsoleIO, token: int) -> None:
        self._console = console
        self._token = token
        self._closed = False

    def close(self) -> None:
        """Unregister the callback. Closing an already closed subscription does nothing."""
        if self._closed:
            return
        self._closed = True
        self._console._unsubscribe(self._token)

    def __enter__(self) -> Self:
        """Return the subscription itself, for use as a context manager."""
        return self

    def __exit__(self, *_args: object) -> None:
        """Close the subscription on leaving the `with` block."""
        self.close()


class ConsoleIO:
    """Text output and subscriptions owned by a runtime.

    Simulation code never prints directly, it calls `echo`. Each message goes
    to stdout with a prefix, replaces `output_buffer` until
    `read_output_buffer` drains it, and reaches every subscriber registered
    with `subscribe`.

    Attributes:
        is_operating: Predicate returning whether the simulation is in
            [`SimulationState.OP`][minisky.simulation.simulation.SimulationState],
            checked once per `update`.
        prefix: Prefix for the stdout copy of echoed text, aligned with
            uvicorn's `INFO:     ` column. Only the terminal print gets it;
            the output buffer read by remote clients stays unprefixed.
        siminfo_rate: Update rate of simulation info messages [Hz]. It records
            the intended rate only — nothing reads it.
        acupdate_rate: Update rate of aircraft update messages [Hz]. It records
            the intended rate only — nothing reads it.
        prevtime: Wall-clock reference of the last rate measurement [s],
            cleared by `reset`; nothing reads it.
        samplecount: Number of operating steps counted by `update` since the
            last `reset`.
        prevcount: Sample count at the last rate measurement; nothing reads it.
        output_buffer: Buffer holding the most recently echoed text until
            `read_output_buffer` drains it.
        event: Set on every `echo`, so waiters can pick up new output.
    """

    prefix: str = "MINISKY:  "
    siminfo_rate: int = 1
    acupdate_rate: int = 5

    def __init__(self, is_operating: Callable[[], bool]) -> None:
        self.is_operating = is_operating
        self.prevtime: float = 0.0
        self.samplecount: int = 0
        self.prevcount: int = 0
        self.output_buffer: io.StringIO = io.StringIO()
        self.event: asyncio.Event = asyncio.Event()
        self._subscribers: dict[int, ConsoleCallback] = {}
        self._next_subscription = 0

    def update(self) -> None:
        """Count one simulation sample while the simulation is operating.

        Increments `samplecount` only when the simulation is in
        [`SimulationState.OP`][minisky.simulation.simulation.SimulationState].
        Nothing calls it, and nothing reads the counter it keeps.
        """
        if self.is_operating():
            self.samplecount += 1

    def reset(self) -> None:
        """Reset the timing bookkeeping counters to their initial values."""
        self.samplecount = 0
        self.prevcount = 0
        self.prevtime = 0.0

    @command(name="ECHO", aliases=("PRINT",))
    def echo(self, text: Text, *, flag: int = 0) -> None:
        """Print a console message.

        The message goes to stdout with `prefix`, and to every callback
        registered with `subscribe`. It also overwrites `output_buffer`, so an
        earlier message `read_output_buffer` has not drained yet is lost.

        Args:
            text: Message to echo; each line is printed on its own.
            flag: Accepted and ignored.
        """
        del flag
        self.output_buffer.truncate(0)
        self.output_buffer.seek(0)
        prefix = self.prefix
        if sys.stdout.isatty():
            # Blue to stand apart from uvicorn's green INFO:, only for
            # interactive terminals so redirected logs stay free of ANSI codes.
            tag = self.prefix.rstrip()
            prefix = f"{Fore.BLUE}{tag}{Style.RESET_ALL}{self.prefix[len(tag) :]}"
        for line in text.splitlines() or [""]:
            print(f"{prefix}{line}")
        print(text, file=self.output_buffer, end="")
        self.event.set()

        for token, callback in tuple(self._subscribers.items()):
            try:
                callback(text)
            except Exception as exc:  # ruff: ignore[BLE001] subscribers are arbitrary
                self._subscribers.pop(token, None)
                traceback.print_exception(exc)

    def subscribe(self, callback: ConsoleCallback) -> ConsoleSubscription:
        """Register a callback receiving every future `echo`.

        Returns:
            A subscription; close it, or use it as a context manager, to
            unregister the callback.
        """
        token = self._next_subscription
        self._next_subscription += 1
        self._subscribers[token] = callback
        return ConsoleSubscription(self, token)

    def _unsubscribe(self, token: int) -> None:
        self._subscribers.pop(token, None)

    def getviewctr(self) -> LatLonDegrees:
        """Return the reference point for navigation lookups.

        Stub for non-GUI mode: with no map view there is no centre to report,
        so lookups that pick the nearest of several same-named waypoints
        always measure from the origin.
        """
        return LatLonDegrees(0.0, 0.0)

    def addnavwpt(self, name: str, lat: float, lon: float) -> None:
        """Do nothing. Stub for non-GUI mode, which draws no waypoint markers."""

    def removenavwpt(self, name: str) -> None:
        """Do nothing. Stub for non-GUI mode, which draws no waypoint markers."""

    def read_output_buffer(self) -> str:
        """Return and clear buffered console output."""
        text = self.output_buffer.getvalue()
        self.output_buffer.truncate(0)
        self.output_buffer.seek(0)
        return text
