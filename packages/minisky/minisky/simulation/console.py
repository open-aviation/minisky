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
        if self._closed:
            return
        self._closed = True
        self._console._unsubscribe(self._token)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()


class ConsoleIO:
    """Text output and subscriptions owned by a runtime."""

    # Prefix for the stdout copy of echoed text, aligned with uvicorn's
    # "INFO:     " column. Only the terminal print gets it; the output
    # buffer read by remote clients stays unprefixed.
    prefix: str = "MINISKY:  "

    # Update rate of simulation info messages [Hz]
    siminfo_rate: int = 1

    # Update rate of aircraft update messages [Hz]
    acupdate_rate: int = 5

    def __init__(self, is_operating: Callable[[], bool]) -> None:
        self.is_operating = is_operating

        # Timing bookkeeping counters
        self.prevtime: float = 0.0
        self.samplecount: int = 0
        self.prevcount: int = 0
        self.output_buffer = io.StringIO()
        self.event = asyncio.Event()
        self._subscribers: dict[int, ConsoleCallback] = {}
        self._next_subscription = 0

    def update(self) -> None:
        """Count one simulation sample while the simulation is operating.

        Increments the sample counter only when the simulation state is
        [`SimulationState.OP`][minisky.simulation.simulation.SimulationState];
        used for bookkeeping of the effective update rate.
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
        """Print, buffer, and publish a console message."""
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
        """Subscribe to future console messages."""
        token = self._next_subscription
        self._next_subscription += 1
        self._subscribers[token] = callback
        return ConsoleSubscription(self, token)

    def _unsubscribe(self, token: int) -> None:
        self._subscribers.pop(token, None)

    def getviewctr(self) -> LatLonDegrees:
        """Return the current view center. Stub for non-GUI mode."""
        return LatLonDegrees(0.0, 0.0)

    def addnavwpt(self, name: str, lat: float, lon: float) -> None:
        """Add a waypoint marker. Stub for non-GUI mode."""

    def removenavwpt(self, name: str) -> None:
        """Remove a waypoint marker. Stub for non-GUI mode."""

    def read_output_buffer(self) -> str:
        """Return and clear buffered console output."""
        text = self.output_buffer.getvalue()
        self.output_buffer.truncate(0)
        self.output_buffer.seek(0)
        return text
