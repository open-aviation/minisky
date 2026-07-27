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

Fixed-wing aircraft in the same simulation are untouched: every override
calls ``super()`` and adjusts only the multicopter rows. Helicopters are out
of scope — membership is a typecode set, not ``LIFT_ROTOR``.

Stack commands: MCOPT, YAW, YAWRATE, HOVER, DELIVER.
"""

from __future__ import annotations

import minisky
from minisky import stack
from minisky.core.trafficarrays import select_implementation

from .aporasas import MulticopterAPorASAS
from .autopilot import MulticopterAutopilot
from .entity import Multicopter
from .kinematics import MulticopterKinematics

#: The plugin's per-aircraft state entity; None until init_plugin() ran.
multicopter: Multicopter | None = None

#: Replaceable base -> multicopter implementation, selected on load and reset.
IMPLEMENTATIONS = (
    ("KINEMATICS", MulticopterKinematics),
    ("APORASAS", MulticopterAPorASAS),
    ("AUTOPILOT", MulticopterAutopilot),
)


def init_plugin():
    """Initialise the multicopter plugin.

    Creates the per-aircraft state entity, swaps in the three multicopter
    implementations, and registers the stack commands.
    """
    global multicopter

    multicopter = mc = Multicopter()
    _select_implementations()

    ap = minisky.traf.ap  # the MulticopterAutopilot instance just selected
    if not isinstance(ap, MulticopterAutopilot):
        raise RuntimeError("MULTICOPTER: could not select MulticopterAutopilot")
    _register_commands(mc, ap)

    config = {
        "plugin_name": "MULTICOPTER",
        "reset": reset,
    }
    return config


def _select_implementations() -> None:
    """Swap the multicopter implementations onto ``traf``.

    Equivalent to issuing ``SELECTIMPL <BASE> <IMPL>`` for each entry of
    :data:`IMPLEMENTATIONS`; replaces the live instance immediately and
    rebinds any stack commands bound to the old one.
    """
    for basename, impl in IMPLEMENTATIONS:
        select_implementation(basename, impl.__name__)


def _register_commands(mc: Multicopter, ap: MulticopterAutopilot) -> None:
    """Register the plugin's stack commands.

    Registered as bound methods, so a later SELECTIMPL swap rebinds them to
    the new instance. Argument specifications are given explicitly, so they
    override the plain Python annotations on the callbacks.
    """
    stack.command(mc.mcopt, name="MCOPT", arguments="callsign,[onoff]")
    stack.command(mc.yaw, name="YAW", arguments="callsign,hdg")
    stack.command(mc.setyawrate, name="YAWRATE", arguments="callsign,[float]")
    stack.command(ap.hover, name="HOVER", arguments="callsign,[time]")
    stack.command(ap.deliver, name="DELIVER", arguments="callsign,alt,[time]")


def reset() -> None:
    """Re-arm the plugin after a simulation reset.

    A reset reverts every replaceable to its core default, so the
    multicopter implementations have to be selected again.
    """
    _select_implementations()
