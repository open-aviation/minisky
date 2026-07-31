"""Example runtime-local autopilot replacement."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

from minisky import plugin as plugin_api
from minisky.traffic.autopilot import Autopilot

if TYPE_CHECKING:
    from minisky.simulation import Simulation
    from minisky.traffic import Traffic


def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
    return context.finish(replacements=(CustomAutoPilot,))


plugin = plugin_api.Plugin(build=build)


@plugin_api.replacement
class CustomAutoPilot(Autopilot):
    """Extend the base autopilot with an example value."""

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__(traffic, get_simulation)
        self.new_variable = 10

    def update(self) -> None:
        super().update()
        self.new_variable += 1
