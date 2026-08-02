"""Multicopter membership and per-aircraft state.

Holds the plugin-owned per-aircraft arrays that mark which aircraft are
multicopters and carry their decoupled body heading and yaw rate, plus the
stack commands that read and write them (``MCOPT``, ``YAW``, ``YAWRATE``,
``HOVER``) and the hooks that keep the multicopter implementations selected
(on the first simulation step after loading, and again after every reset,
which reverts all replaceables to their core defaults).

Membership is deliberately *not* ``traf.perf.lifttype == LIFT_ROTOR``: that
set also contains the EC35, a crewed helicopter, which this plugin does not
model. It is a fixed typecode set instead, overridable per aircraft.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from minisky import plugin as plugin_api

if TYPE_CHECKING:
    from minisky.traffic import Traffic

#: OpenAP rotor typecodes minus helicopters (the EC35 is excluded on purpose).
MULTICOPTER_TYPES = frozenset(
    {"MAVIC", "PHAN4", "M100", "M200", "M600", "MNET", "AMZN", "HORSEFLY"}
)

#: Default yaw rate for a newly created multicopter [deg/s].
DEFAULT_YAWRATE = 90.0

#: Replaceable base -> multicopter implementation, selected on load and reset.
#: Kept as names (not classes) because every implementation module imports
#: this one for `get_multicopter`.
IMPLEMENTATIONS = (
    ("KINEMATICS", "MULTICOPTERKINEMATICS"),
    ("APORASAS", "MULTICOPTERAPORASAS"),
    ("AUTOPILOT", "MULTICOPTERAUTOPILOT"),
    ("ACTIVEWAYPOINT", "MULTICOPTERACTIVEWAYPOINT"),
)


def get_multicopter(traffic: Traffic) -> Multicopter | None:
    """Return the Multicopter entity attached to a traffic tree, if any.

    The entity is mounted by the plugin build as a child node of ``traffic``.
    The replaceable subclasses use this lookup so that, when one of them is
    selected without the plugin loaded, they degrade to base behaviour
    instead of crashing.
    """
    return next(
        (child for child in traffic._children if isinstance(child, Multicopter)),
        None,
    )


class Multicopter(plugin_api.Entity):
    """Per-aircraft multicopter state.

    Attributes:
        ismulticopter (ndarray): Bool switch: aircraft is flown as a
            multicopter (yaw-rate-limited heading, track-driven velocity).
        selhdg (ndarray): Commanded body heading [deg], decoupled from track.
        swselhdg (ndarray): Bool switch: a body heading was commanded. While
            False the nose follows the track (nose-along-course default).
        yawrate (ndarray): Maximum yaw rate [deg/s].
    """

    def __init__(self) -> None:
        super().__init__()
        self._selected = False
        with self.settrafarrays():
            self.ismulticopter = np.array([], dtype=bool)
            self.selhdg = np.array([])
            self.swselhdg = np.array([], dtype=bool)
            self.yawrate = np.array([])

    def create(self, n: int = 1) -> None:
        """Seed multicopter state for n newly created aircraft.

        Membership follows from the typecode; the body heading starts
        unconstrained (nose follows track) at the default yaw rate.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)
        self.ismulticopter[-n:] = [
            typecode.upper() in MULTICOPTER_TYPES for typecode in self.traffic.typecode[-n:]
        ]
        self.selhdg[-n:] = self.traffic.hdg[-n:]
        self.swselhdg[-n:] = False
        self.yawrate[-n:] = DEFAULT_YAWRATE

    def mask(self) -> np.ndarray:
        """Return the boolean row mask of aircraft flown as multicopters."""
        return self.ismulticopter

    def select_implementations(self) -> None:
        """Swap the multicopter implementations onto the owning traffic.

        Equivalent to issuing ``SELECTIMPL <BASE> <IMPL>`` for each entry of
        :data:`IMPLEMENTATIONS`; replaces the live instance immediately.
        """
        for basename, implname in IMPLEMENTATIONS:
            result = self.traffic.select_implementation(basename, implname)
            if result.is_err():
                raise RuntimeError(f"MULTICOPTER: {result.err()}")
        self._selected = True

    @plugin_api.hook("preupdate")
    def ensure_implementations(self) -> None:
        """Select the multicopter implementations on the first step after load.

        Replacements are installed when the plugin loads but can only be
        selected once the plugin is published, so the initial selection
        happens here. A manual ``SELECTIMPL`` afterwards is respected until
        the next reset.
        """
        if not self._selected:
            self.select_implementations()

    @plugin_api.hook("reset")
    def reselect_implementations(self) -> None:
        """Re-select the multicopter implementations after a reset.

        A reset reverts every replaceable to its core default; this hook runs
        afterwards and restores the multicopter set.
        """
        self.select_implementations()

    @plugin_api.command(arguments="callsign,[onoff]")
    def mcopt(self, idx: int, flag: bool | None = None) -> tuple[bool, str]:
        """Mark an aircraft as a multicopter (or report its current setting).

        Arguments:
        - idx: Aircraft callsign
        - flag: ON to fly it as a multicopter, OFF for normal fixed-wing
          kinematics (optional, omit to query)
        """
        callsign = self.traffic.callsign[idx]
        if flag is None:
            return True, f"MCOPT {callsign}: {'ON' if self.ismulticopter[idx] else 'OFF'}"

        self.ismulticopter[idx] = flag
        # Multicopters fly point-to-point: newly added waypoints default to
        # fly-over (restored to fly-by when switched back off).
        self.traffic.ap.route[idx].swflyby = not flag
        if flag:
            # Start with the nose unconstrained, following the track.
            self.selhdg[idx] = self.traffic.hdg[idx]
            self.swselhdg[idx] = False
        return True, f"MCOPT {callsign}: {'ON' if flag else 'OFF'}"

    @plugin_api.command(arguments="callsign,hdg")
    def yaw(self, idx: int, hdg: float) -> tuple[bool, str]:
        """Command the body heading (nose direction) of a multicopter.

        The velocity vector keeps following the track command from the FMS
        or conflict resolution, so this rotates the aircraft in place.

        Arguments:
        - idx: Aircraft callsign
        - hdg: Commanded body heading [deg]
        """
        if not self.ismulticopter[idx]:
            callsign = self.traffic.callsign[idx]
            return False, f"YAW: {callsign} is not a multicopter (use MCOPT {callsign} ON)"

        self.selhdg[idx] = hdg % 360.0
        self.swselhdg[idx] = True
        return True, f"YAW {self.traffic.callsign[idx]}: nose to {hdg % 360.0:.0f} deg"

    @plugin_api.command(name="YAWRATE", arguments="callsign,[float]")
    def setyawrate(self, idx: int, yawrate: float | None = None) -> tuple[bool, str]:
        """Set or report the maximum yaw rate of a multicopter.

        Arguments:
        - idx: Aircraft callsign
        - yawrate: Maximum yaw rate [deg/s] (optional, omit to query)
        """
        callsign = self.traffic.callsign[idx]
        if yawrate is None:
            return True, f"YAWRATE {callsign}: {self.yawrate[idx]:.0f} deg/s"
        if yawrate <= 0.0:
            return False, "YAWRATE: yaw rate must be positive"

        self.yawrate[idx] = yawrate
        return True, f"YAWRATE {callsign}: {yawrate:.0f} deg/s"

    @plugin_api.command(arguments="callsign,[time,alt]")
    def hover(
        self, idx: int, duration: float | None = None, alt: float | None = None
    ) -> tuple[bool, str]:
        """Hold position, optionally for a fixed time at a given altitude.

        Suspends LNAV/VNAV, commands zero ground speed, and holds the given
        altitude (the current one when omitted) — with an altitude the
        aircraft moves there vertically, at a fixed position. With a
        duration, the route resumes once position and altitude have been
        held that long; without one, the aircraft hovers until LNAV is
        re-engaged. Repeating the command while hovering updates the hold
        time and altitude, and a plain ALT command changes the hover
        altitude as well.

        Arguments:
        - idx: Aircraft callsign
        - duration: Hold time [s] (optional, omit to hover indefinitely)
        - alt: Hover altitude [ft or FL] (optional, default: hold current)
        """
        # Deferred import: the autopilot module imports this one.
        from minisky_multicopter.autopilot import MulticopterAutopilot

        ap = self.traffic.ap
        if not isinstance(ap, MulticopterAutopilot):
            return False, "HOVER: SELECTIMPL AUTOPILOT MULTICOPTERAUTOPILOT first"
        return ap.hover(idx, duration, alt)
