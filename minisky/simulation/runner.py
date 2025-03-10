"""Node encapsulates the sim process, and manages process I/O."""

import asyncio
import os

import minisky


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
            target_dt = minisky.sim.simdt / minisky.sim.dtmult
            next_time = asyncio.get_event_loop().time() + target_dt

            minisky.sim.step()

            # Calculate sleep time to compensate for processing time
            current_time = asyncio.get_event_loop().time()
            sleep_time = max(0, next_time - current_time)
            next_time = next_time + target_dt  # Schedule next step

            await asyncio.sleep(sleep_time)
        print("simulation completed")

    def stop(self):
        if self.allow_shutdown:
            self.running = False
        else:
            print("Shutdown is prevented")
