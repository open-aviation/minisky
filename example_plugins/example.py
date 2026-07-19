"""MiniSky example plugin.

This plugin demonstrates the plugin system capabilities:
- Registering per-aircraft data arrays
- Periodic update functions
- Stack commands
"""

from random import randint

import numpy as np

import minisky
from minisky import plugin, stack

# Global reference to the example instance
example: "Example | None" = None


def init_plugin():
    """Plugin initialization function.

    This function is required for all plugins. It should return a configuration
    dictionary, and optionally a second dictionary of stack functions.
    """
    global example

    # Instantiate our example entity
    example = instance = Example()

    # Configuration parameters
    config = {
        "plugin_name": "EXAMPLE",
        "update_interval": 5,  # Update every 5 seconds
        "update": instance.update,  # Register update function via config
    }

    return config


class Example(plugin.Entity):
    """Example entity that tracks passenger count per aircraft."""

    def __init__(self):
        super().__init__()
        # Register per-aircraft data arrays
        # These automatically resize when aircraft are created/deleted
        with self.settrafarrays():
            self.npassengers = np.array([])

    def create(self, n=1):
        """Called automatically when new aircraft are created."""
        super().create(n)
        # Set passenger count for new aircraft
        self.npassengers[-n:] = [randint(50, 250) for _ in range(n)]

    def update(self):
        """Periodic update function called every 5 simulation seconds."""
        if minisky.traf.ntraf > 0:
            total = int(sum(self.npassengers))
            print(f"Example plugin: {minisky.traf.ntraf} aircraft, {total} total passengers")


# Stack command for passengers - defined as module-level function
@stack.command(name="PASSENGERS", arguments="txt,[int]")
def passengers(callsign: str, count: int = -1):
    """Set or get the number of passengers on an aircraft.

    Arguments:
    - callsign: Aircraft callsign
    - count: Number of passengers (optional, omit to query)
    """
    if example is None:
        return False, "Example plugin not initialised"

    callsign = callsign.upper()

    # Find aircraft index
    if callsign not in minisky.traf.callsign:
        return False, f"Aircraft {callsign} not found"

    idx = minisky.traf.callsign.index(callsign)

    if count < 0:
        return True, f"Aircraft {callsign} has {int(example.npassengers[idx])} passengers"

    example.npassengers[idx] = count
    return True, f"Set {callsign} passengers to {count}"
