"""Node encapsulates the sim process, and manages process I/O."""

import asyncio
import os

import minisky

MIN_UPDATE_INTERVAL = 0.0001


class Runner:
    def __init__(self, **kwargs):
        self.node_id = b"\x00" + os.urandom(4)
        self.host_id = b""
        self.running = False
        self.allow_shutdown = True
        self.speed = kwargs.get("speed", 1)
        self.jump = 0
        self.jump_to = 0

    def forward(self, seconds):
        self.jump_to = minisky.sim.simt + seconds - 2  #  -2 for the action margin
        self.jump = seconds

    def prevent_shutdown(self):
        self.allow_shutdown = False

    async def run(self):
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

    def stop(self):
        if self.allow_shutdown:
            self.running = False
        else:
            print("Shutdown is prevented")
