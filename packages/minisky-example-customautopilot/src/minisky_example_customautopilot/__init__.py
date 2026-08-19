"""Example runtime-local autopilot replacement."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

from minisky import Autopilot, Plugin, PluginContext, PluginSpec, replacement

if TYPE_CHECKING:
    from minisky import Simulation, Traffic


# --8<-- [start:replacement]
@replacement
class CustomAutoPilot(Autopilot):
    """Extend the base autopilot with an example value."""

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__(traffic, get_simulation)
        self.new_variable = 10

    def update(self) -> None:
        super().update()
        self.new_variable += 1


def build(context: PluginContext[object]) -> PluginSpec:
    return context.finish(replacements=(CustomAutoPilot,))


plugin = Plugin(build=build)
# --8<-- [end:replacement]
