"""Node encapsulates the sim process, and manages process I/O."""

import asyncio
import os

import minisky
from minisky.core.walltime import Timer


class Runner:
    def __init__(self, *args):
        self.node_id = b"\x00" + os.urandom(4)
        self.host_id = b""
        self.running = False
        self.allow_shutdown = True

    def prevent_shutdown(self):
        self.allow_shutdown = False

    async def run(self):
        print("staring simulation")
        self.running = True
        while self.running:
            Timer.update_timers()
            minisky.sim.step()
            await asyncio.sleep(0.01)
        print("simulation completed")

    def stop(self):
        if self.allow_shutdown:
            self.running = False
        else:
            print("Shutdown is prevented")
