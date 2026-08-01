"""MULTICOPTER — simulate small electric multirotors.

Makes MiniSky fly DJI MAVIC/M600/PHAN4-class and Amazon/Matternet-style
delivery drones with multicopter behaviour: hover and yaw at zero airspeed,
and a velocity vector decoupled from the body heading (the aircraft can
strafe — change course without rotating the nose).

Everything is implemented as replaceable subclasses of core entities, which
the plugin selects on load:

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

from typing import TYPE_CHECKING, Any

from .activewp import MulticopterActiveWaypoint
from .aporasas import MulticopterAPorASAS
from .autopilot import MulticopterAutopilot
from .entity import Multicopter
from .kinematics import MulticopterKinematics

if TYPE_CHECKING:
    from minisky import MiniSky
    from minisky.traffic import Traffic

#: Replaceable base -> multicopter implementation, selected on load and reset.
IMPLEMENTATIONS = (
    ("KINEMATICS", MulticopterKinematics),
    ("APORASAS", MulticopterAPorASAS),
    ("AUTOPILOT", MulticopterAutopilot),
    ("ACTIVEWAYPOINT", MulticopterActiveWaypoint),
)


def init_plugin(runtime: MiniSky) -> tuple[dict[str, Any], dict[str, list[Any]]]:
    """Initialise the multicopter plugin for one MiniSky runtime.

    Creates the per-aircraft state entity on the runtime's traffic tree,
    swaps in the multicopter implementations, and returns the stack commands.

    Args:
        runtime: MiniSky runtime loading this plugin.

    Returns:
        A `(config, stack_functions)` tuple consumed by the plugin manager.
    """
    mc = Multicopter(runtime.traffic)
    _select_implementations(runtime)
    if not isinstance(runtime.traffic.ap, MulticopterAutopilot):
        raise RuntimeError("MULTICOPTER: could not select the multicopter implementations")

    config = {
        "plugin_name": "MULTICOPTER",
        "reset": lambda: _select_implementations(runtime),
        "state": mc,
    }
    return config, _stack_functions(runtime.traffic, mc)


def _select_implementations(runtime: MiniSky) -> None:
    """Swap the multicopter implementations onto the runtime's ``traf``.

    Equivalent to issuing ``SELECTIMPL <BASE> <IMPL>`` for each entry of
    :data:`IMPLEMENTATIONS`; replaces the live instance immediately and
    rebinds any stack commands bound to the old one. Also called from the
    reset hook, since a reset reverts every replaceable to its core default.
    """
    for basename, impl in IMPLEMENTATIONS:
        runtime.replaceables.select(basename, impl.__name__)


def _stack_functions(traffic: Traffic, mc: Multicopter) -> dict[str, list[Any]]:
    """Build the plugin's stack-command table.

    HOVER is a free function that looks up ``traffic.ap`` at call time
    rather than a bound method: a reset replaces the autopilot instance
    twice (revert to base, then the reset hook reselects), and the command
    rebinding cannot follow methods that only exist on the subclass across
    that double swap. The entity commands bind to ``mc`` directly, which
    lives for the whole plugin lifetime.

    Argument specifications are given explicitly, so they override the plain
    Python annotations on the callbacks.
    """

    def hover(
        idx: int, duration: float | None = None, alt: float | None = None
    ) -> tuple[bool, str]:
        ap = traffic.ap
        if not isinstance(ap, MulticopterAutopilot):
            return False, "HOVER: SELECTIMPL AUTOPILOT MULTICOPTERAUTOPILOT first"
        return ap.hover(idx, duration, alt)

    return {
        "MCOPT": [mc.mcopt, "callsign,[onoff]", "MCOPT callsign,[ON/OFF]", mc.mcopt.__doc__],
        "YAW": [mc.yaw, "callsign,hdg", "YAW callsign,hdg", mc.yaw.__doc__],
        "YAWRATE": [
            mc.setyawrate,
            "callsign,[float]",
            "YAWRATE callsign,[rate]",
            mc.setyawrate.__doc__,
        ],
        "HOVER": [
            hover,
            "callsign,[time,alt]",
            "HOVER callsign,[time,alt]",
            MulticopterAutopilot.hover.__doc__,
        ],
    }
