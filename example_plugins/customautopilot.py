"""Custom Autopilot Plugin - Example of the Replaceable Pattern.

This plugin demonstrates how to create a custom autopilot by subclassing
the base Autopilot class. MiniSky's replaceable pattern allows you to
swap implementations at runtime without modifying core code.

How it works:
1. Subclass a TrafficArrays-derived class (e.g., Autopilot, PerfBase)
2. Your subclass is automatically registered by name (uppercase class name)
3. Select your implementation via scenario command or programmatically:
   - Scenario: SELECTIMPL AUTOPILOT CUSTOMAUTOPILOT
   - Python:   CustomAutoPilot.select()
4. On simulation reset, implementations revert to defaults

The SELECTIMPL command replaces the existing instance on traf immediately,
so you can switch implementations mid-simulation if needed.

Available base classes for replacement:
- Autopilot: Aircraft guidance logic (traf.ap)
- PerfBase: Performance model (traf.perf)
- ConflictDetection: CD algorithm (traf.cd)
- ConflictResolution: CR algorithm (traf.cr)
"""

from minisky.traffic.autopilot import Autopilot


def init_plugin():
    config = {"plugin_name": "CUSTOMAUTOPILOT"}
    return config


class CustomAutoPilot(Autopilot):
    """Custom autopilot implementation.

    Subclassing Autopilot automatically registers this class as 'CUSTOMAUTOPILOT'.
    Select it with: SELECTIMPL AUTOPILOT CUSTOMAUTOPILOT
    """

    def __init__(self):
        super().__init__()
        # Add custom instance variables here
        self.new_variable = 10

    def update(self):
        # Option 1: Extend base behavior - call super first, then add custom logic
        super().update()
        self.new_variable += 1

        # Option 2: Replace base behavior entirely - don't call super().update()
        # and implement your own autopilot logic from scratch
