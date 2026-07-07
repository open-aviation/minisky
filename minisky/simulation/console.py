"""Console I/O for the MiniSky simulation.

Defines :class:`ConsoleIO`, the text output channel of the simulator. Stack
commands and simulation state changes report back through its :meth:`echo`
method, which prints to stdout and stores the message in a buffer that remote
clients (such as the HTTP API served by ``minisky server``) can read asynchronously.
A single instance is created by :func:`minisky.init` and available as
``minisky.scr``.
"""

import asyncio
import io

import minisky


class ConsoleIO:
    """Class within sim task which sends/receives data to/from GUI task.

    Acts as the simulator's screen/console object (``minisky.scr``). Output
    produced with :meth:`echo` is printed to stdout and kept in an in-memory
    buffer; an :class:`asyncio.Event` is set on every echo so that awaiting
    consumers (e.g. the HTTP API's ``/stack`` endpoint) know new output is
    available and can collect it with :meth:`read_output_buffer`.

    Attributes:
        siminfo_rate: Update rate of simulation info messages [Hz].
        acupdate_rate: Update rate of aircraft update messages [Hz].
        prevtime: Simulation time of the previous info update [s].
        samplecount: Number of simulation samples counted while operating.
        prevcount: Sample count at the previous info update.
        output_buffer: ``StringIO`` buffer holding the latest echoed text.
        event: ``asyncio.Event`` set whenever new output has been echoed.
    """

    # Update rate of simulation info messages [Hz]
    siminfo_rate: int = 1

    # Update rate of aircraft update messages [Hz]
    acupdate_rate: int = 5

    def __init__(self) -> None:
        # Timing bookkeeping counters
        self.prevtime: float = 0.0
        self.samplecount: int = 0
        self.prevcount: int = 0

        self.output_buffer: io.StringIO = io.StringIO()
        self.event: asyncio.Event = asyncio.Event()

    def update(self) -> None:
        """Count one simulation sample while the simulation is operating.

        Increments the sample counter only when the simulation state is
        ``OP``; used for bookkeeping of the effective update rate.
        """
        if minisky.sim.state == minisky.OP:
            self.samplecount += 1

    def reset(self) -> None:
        """Reset the timing bookkeeping counters to their initial values."""
        self.samplecount = 0
        self.prevcount = 0
        self.prevtime = 0.0

    def echo(self, text: str = "", flag: int = 0) -> None:
        """Print a message and store it in the output buffer.

        The previous buffer contents are discarded, the text is written both
        to stdout and to the buffer, and the output event is set to wake up
        any consumer awaiting new output.

        Args:
            text: Message text to output.
            flag: Message flag (accepted for interface compatibility, unused).
        """
        self.output_buffer.truncate(0)
        self.output_buffer.seek(0)
        print(text)
        print(text, file=self.output_buffer, end="")
        self.event.set()

    def getviewctr(self) -> tuple[float, float]:
        """Return the current view center (lat, lon). Stub for non-GUI mode."""
        return 0.0, 0.0

    def addnavwpt(self, name: str, lat: float, lon: float) -> None:
        """Add a nav waypoint marker to the display. Stub for non-GUI mode."""
        pass

    def removenavwpt(self, name: str) -> None:
        """Remove a nav waypoint marker from the display. Stub for non-GUI mode."""
        pass

    def read_output_buffer(self) -> str:
        """Return the buffered console output and clear the buffer.

        Returns:
            str: All text echoed since the last read (empty string if none).
        """
        text = self.output_buffer.getvalue()
        self.output_buffer.truncate(0)
        self.output_buffer.seek(0)
        return text
