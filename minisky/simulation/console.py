import asyncio
import io

import minisky


class ConsoleIO:
    """Class within sim task which sends/receives data to/from GUI task"""

    # Update rate of simulation info messages [Hz]
    siminfo_rate = 1

    # Update rate of aircraft update messages [Hz]
    acupdate_rate = 5

    def __init__(self):
        # Timing bookkeeping counters
        self.prevtime = 0.0
        self.samplecount = 0
        self.prevcount = 0

        self.output_buffer = io.StringIO()
        self.event = asyncio.Event()

    def update(self):
        if minisky.sim.state == minisky.OP:
            self.samplecount += 1

    def reset(self):
        self.samplecount = 0
        self.prevcount = 0
        self.prevtime = 0.0

    def echo(self, text: str = "", flag=0):
        self.output_buffer.truncate(0)
        self.output_buffer.seek(0)
        print(text)
        print(text, file=self.output_buffer, end="")
        self.event.set()

    def read_output_buffer(self):
        text = self.output_buffer.getvalue()
        self.output_buffer.truncate(0)
        self.output_buffer.seek(0)
        return text
