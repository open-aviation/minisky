"""MiniSky example plugin.

This plugin demonstrates the plugin system capabilities:

- registering per-aircraft data arrays;
- periodic update functions;
- stack commands bound to runtime-owned plugin state.
"""

from __future__ import annotations

from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np

from minisky import plugin

if TYPE_CHECKING:
    from minisky import MiniSky
    from minisky.traffic import Traffic


def init_plugin(
    runtime: MiniSky,
) -> tuple[dict[str, Any], dict[str, list[Any]]]:
    """Initialize the plugin for one MiniSky runtime.

    This function is required for all plugins. It returns a configuration
    dictionary and, optionally, a second dictionary of stack functions. The
    runtime argument makes ownership explicit: every plugin load creates a new
    entity and command binding for that runtime.

    Args:
        runtime: MiniSky runtime loading this plugin.

    Returns:
        A `(config, stack_functions)` tuple consumed by the runtime's plugin
        manager.
    """
    # Instantiate the example entity on this runtime's traffic-array tree.
    instance = Example(runtime.traffic)

    # Configuration parameters and lifecycle callbacks.
    config = {
        "plugin_name": "EXAMPLE",
        "update_interval": 5,  # Update every 5 seconds of simulation time.
        "update": instance.update,
        "state": instance,
    }

    # Bind the PASSENGERS command directly to this runtime's entity instance.
    stack_functions = {
        "PASSENGERS": [
            instance.passengers,
            "txt,[int]",
            "PASSENGERS callsign, [count]",
            "Set or get the number of passengers on an aircraft.",
        ]
    }
    return config, stack_functions


class Example(plugin.Entity):
    """Example entity that tracks passenger count per aircraft.

    Each loaded runtime owns a separate `Example` instance. Its passenger
    array remains index-aligned with the aircraft arrays of the traffic object
    passed to the constructor.
    """

    def __init__(self, traffic: Traffic) -> None:
        """Attach the entity to `traffic` and register its passenger array."""
        super().__init__(traffic)

        # Register per-aircraft data arrays. These automatically resize when
        # aircraft are created or deleted in the owning runtime.
        with self.settrafarrays():
            self.npassengers = np.array([])

    def create(self, n: int = 1) -> None:
        """Initialize passenger counts for newly created aircraft.

        Called automatically by the traffic-array tree whenever `n` aircraft
        are added to the owning traffic object.

        Args:
            n: Number of newly created aircraft.
        """
        super().create(n)
        self.npassengers[-n:] = [randint(50, 250) for _ in range(n)]

    def update(self) -> None:
        """Periodic update function called every five simulation seconds."""
        if self.traffic.ntraf > 0:
            total = int(sum(self.npassengers))
            print(
                f"Example plugin: {self.traffic.ntraf} aircraft, "
                f"{total} total passengers"
            )

    def passengers(self, callsign: str, count: int = -1) -> tuple[bool, str]:
        """Set or get the number of passengers on an aircraft.

        Args:
            callsign: Aircraft callsign.
            count: Passenger count to set. Omit it, or pass a negative value,
                to query the current count.

        Returns:
            A `(success, message)` tuple suitable for the stack command.
        """
        callsign = callsign.upper()

        # Find the aircraft index in this runtime's traffic object.
        if callsign not in self.traffic.callsign:
            return False, f"Aircraft {callsign} not found"

        index = self.traffic.callsign.index(callsign)
        if count < 0:
            return (
                True,
                f"Aircraft {callsign} has {int(self.npassengers[index])} passengers",
            )

        self.npassengers[index] = count
        return True, f"Set {callsign} passengers to {count}"
