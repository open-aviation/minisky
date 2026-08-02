"""Example runtime-local MiniSky plugin."""

from __future__ import annotations

from random import Random

import numpy as np
from minisky import plugin as plugin_api
from minisky.result import Err, Ok, Result


# --8<-- [start:declaration]
def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
    context.mount(Example(context.python_random))
    return context.finish()


plugin = plugin_api.Plugin(build=build)
# --8<-- [end:declaration]


# --8<-- [start:entity]
class Example(plugin_api.Entity):
    """Track passenger count for every aircraft in the owning runtime."""

    def __init__(self, random: Random) -> None:
        super().__init__()
        self.random = random
        self.updates = 0
        with self.settrafarrays():
            self.npassengers = np.array([])

    def create(self, n: int = 1) -> None:
        super().create(n)
        self.npassengers[-n:] = [self.random.randint(50, 250) for _ in range(n)]

    @plugin_api.hook(interval=5.0)
    def update(self) -> None:
        """Count periodic execution."""
        self.updates += 1

    @plugin_api.command(arguments="txt,[int]")
    def passengers(self, callsign: str, count: int = -1) -> Result[str, str]:
        """Set or get the number of passengers on an aircraft."""
        callsign = callsign.upper()
        if callsign not in self.traffic.callsign:
            return Err(f"Aircraft {callsign} not found")
        index = self.traffic.callsign.index(callsign)
        if count < 0:
            return Ok(f"Aircraft {callsign} has {int(self.npassengers[index])} passengers")
        self.npassengers[index] = count
        return Ok(f"Set {callsign} passengers to {count}")


# --8<-- [end:entity]
