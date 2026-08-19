"""Example runtime-local MiniSky plugin."""

from __future__ import annotations

from random import Random
from typing import Annotated

import numpy as np
from annotated_types import Ge, Le
from minisky import AcId, Entity, Ok, Plugin, PluginContext, PluginSpec, Result, command, hook


# --8<-- [start:declaration]
def build(context: PluginContext[object]) -> PluginSpec:
    context.mount(Example(context.python_random))
    return context.finish()


plugin = Plugin(build=build)
# --8<-- [end:declaration]


# --8<-- [start:entity]
class Example(Entity):
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

    @hook(interval=5.0)
    def update(self) -> None:
        """Count periodic execution."""
        self.updates += 1

    @command(name="PASSENGERS")
    def passenger_count(self, index: AcId) -> Result[str, str]:
        """Show the number of passengers on an aircraft."""
        callsign = self.traffic.callsign[index]
        return Ok(f"Aircraft {callsign} has {int(self.npassengers[index])} passengers")

    @command(name="PASSENGERS")
    def set_passenger_count(
        self, index: AcId, count: Annotated[int, Ge(0), Le(500)]
    ) -> Result[str, str]:
        """Set the number of passengers on an aircraft."""
        callsign = self.traffic.callsign[index]
        self.npassengers[index] = count
        return Ok(f"Set {callsign} passengers to {count}")


# --8<-- [end:entity]
