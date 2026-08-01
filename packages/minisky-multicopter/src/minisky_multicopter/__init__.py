"""MULTICOPTER — simulate small electric multirotors.

Makes MiniSky fly DJI MAVIC/M600/PHAN4-class and Amazon/Matternet-style
delivery drones with multicopter behaviour: hover and yaw at zero airspeed,
and a velocity vector decoupled from the body heading (the aircraft can
strafe — change course without rotating the nose).

Everything is implemented as replaceable subclasses of core entities, which
the plugin registers on load and keeps selected through its hooks (the first
simulation step after loading, and every reset):

- ``KINEMATICS``      -> :class:`MulticopterKinematics` — yaw-rate-limited
  heading, track-driven velocity vector.
- ``APORASAS``        -> :class:`MulticopterAPorASAS` — no track-to-heading
  coupling for multicopter rows.
- ``AUTOPILOT``       -> :class:`MulticopterAutopilot` — HOVER primitive,
  HDG-yaws-the-nose semantics, fly-over route defaults.
- ``ACTIVEWAYPOINT``  -> :class:`MulticopterActiveWaypoint` — fixed waypoint
  capture radius (the bank-angle turn distance degenerates at hover speeds).

Fixed-wing aircraft in the same simulation are untouched: every override
calls ``super()`` and adjusts only the multicopter rows. Helicopters are out
of scope — membership is a typecode set, not ``LIFT_ROTOR``.

Stack commands: MCOPT, YAW, YAWRATE, HOVER.
"""

from __future__ import annotations

from minisky import plugin as plugin_api
from minisky_multicopter.activewp import MulticopterActiveWaypoint
from minisky_multicopter.aporasas import MulticopterAPorASAS
from minisky_multicopter.autopilot import MulticopterAutopilot
from minisky_multicopter.entity import Multicopter, get_multicopter
from minisky_multicopter.kinematics import MulticopterKinematics

__all__ = (
    "Multicopter",
    "MulticopterAPorASAS",
    "MulticopterActiveWaypoint",
    "MulticopterAutopilot",
    "MulticopterKinematics",
    "get_multicopter",
    "plugin",
)


def build(context: plugin_api.PluginContext[object]) -> plugin_api.PluginSpec:
    """Build the multicopter components for one MiniSky runtime.

    Mounts the per-aircraft state entity (which also carries the stack
    commands and the implementation-selection hooks) and registers the
    multicopter implementations as runtime-local replacements.
    """
    context.mount(Multicopter())
    return context.finish(
        replacements=(
            MulticopterKinematics,
            MulticopterAPorASAS,
            MulticopterAutopilot,
            MulticopterActiveWaypoint,
        )
    )


plugin = plugin_api.Plugin(build=build)
