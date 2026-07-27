"""MULTICOPTER — simulate small electric multirotors.

Makes MiniSky fly DJI MAVIC/M600/PHAN4-class and Amazon/Matternet-style
delivery drones with multicopter behaviour: hover and yaw at zero airspeed,
and a velocity vector decoupled from the body heading (the aircraft can
strafe — change course without rotating the nose).

Everything is implemented as replaceable subclasses of core entities, which
the plugin selects on load:

- ``KINEMATICS`` -> :class:`MulticopterKinematics` — yaw-rate-limited
  heading, track-driven velocity vector.
- ``APORASAS``   -> :class:`MulticopterAPorASAS` — no track-to-heading
  coupling for multicopter rows.
- ``AUTOPILOT``  -> :class:`MulticopterAutopilot` — HOVER/DELIVER mission
  primitives and a fixed waypoint capture radius.
- ``OPENAP``     -> :class:`MulticopterPerf` — electric performance: power
  from a propeller/motor map, battery state of charge, sagging envelope.

Fixed-wing aircraft in the same simulation are untouched: every override
calls ``super()`` and adjusts only the multicopter rows. Helicopters are out
of scope — membership is a typecode set, not ``LIFT_ROTOR``.

Stack commands: MCOPT, YAW, YAWRATE, HOVER, DELIVER, BATT.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from .aporasas import MulticopterAPorASAS
from .autopilot import MulticopterAutopilot
from .entity import Multicopter
from .kinematics import MulticopterKinematics
from .perf import MulticopterPerf

if TYPE_CHECKING:
    from minisky import MiniSky

#: Replaceable base -> multicopter implementation, selected on load and reset.
IMPLEMENTATIONS = (
    ("KINEMATICS", MulticopterKinematics),
    ("APORASAS", MulticopterAPorASAS),
    ("AUTOPILOT", MulticopterAutopilot),
    ("OPENAP", MulticopterPerf),
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

    # The instances just selected onto traf; commands bind to these
    ap = runtime.traffic.ap
    perf = runtime.traffic.perf
    if not isinstance(ap, MulticopterAutopilot) or not isinstance(perf, MulticopterPerf):
        raise RuntimeError("MULTICOPTER: could not select the multicopter implementations")

    config = {
        "plugin_name": "MULTICOPTER",
        "reset": lambda: _select_implementations(runtime),
        "state": mc,
    }
    return config, _stack_functions(mc, ap, perf)


def _select_implementations(runtime: MiniSky) -> None:
    """Swap the multicopter implementations onto the runtime's ``traf``.

    Equivalent to issuing ``SELECTIMPL <BASE> <IMPL>`` for each entry of
    :data:`IMPLEMENTATIONS`; replaces the live instance immediately and
    rebinds any stack commands bound to the old one. Also called from the
    reset hook, since a reset reverts every replaceable to its core default.
    """
    for basename, impl in IMPLEMENTATIONS:
        runtime.replaceables.select(basename, impl.__name__)


def _stack_functions(
    mc: Multicopter, ap: MulticopterAutopilot, perf: MulticopterPerf
) -> dict[str, list[Any]]:
    """Build the plugin's stack-command table.

    Bound to the freshly selected instances; a later SELECTIMPL swap rebinds
    them to the new instance. Argument specifications are given explicitly,
    so they override the plain Python annotations on the callbacks.
    """
    return {
        "MCOPT": [mc.mcopt, "callsign,[onoff]", "MCOPT callsign,[onoff]", mc.mcopt.__doc__],
        "YAW": [mc.yaw, "callsign,hdg", "YAW callsign,hdg", mc.yaw.__doc__],
        "YAWRATE": [
            mc.setyawrate,
            "callsign,[float]",
            "YAWRATE callsign,[rate]",
            mc.setyawrate.__doc__,
        ],
        "HOVER": [ap.hover, "callsign,[time]", "HOVER callsign,[time]", ap.hover.__doc__],
        "DELIVER": [
            ap.deliver,
            "callsign,alt,[time]",
            "DELIVER callsign,alt,[time]",
            ap.deliver.__doc__,
        ],
        "BATT": [perf.batt, "callsign", "BATT callsign", perf.batt.__doc__],
    }
