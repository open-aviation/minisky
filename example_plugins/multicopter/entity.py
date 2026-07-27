"""Multicopter membership and per-aircraft state.

Holds the plugin-owned per-aircraft arrays that mark which aircraft are
multicopters and carry their decoupled body heading and yaw rate, plus the
stack commands that read and write them (``MCOPT``, ``YAW``, ``YAWRATE``).

Membership is deliberately *not* ``traf.perf.lifttype == LIFT_ROTOR``: that
set also contains the EC35, a crewed helicopter, which this plugin does not
model. It is a fixed typecode set instead, overridable per aircraft.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from minisky import plugin

if TYPE_CHECKING:
    from minisky.traffic import Traffic

#: OpenAP rotor typecodes minus helicopters (the EC35 is excluded on purpose).
MULTICOPTER_TYPES = frozenset(
    {"MAVIC", "PHAN4", "M100", "M200", "M600", "MNET", "AMZN", "HORSEFLY"}
)

#: Default yaw rate for a newly created multicopter [deg/s].
DEFAULT_YAWRATE = 90.0


class Multicopter(plugin.Entity):
    """Per-aircraft multicopter state.

    Attributes:
        ismulticopter (ndarray): Bool switch: aircraft is flown as a
            multicopter (yaw-rate-limited heading, track-driven velocity).
        selhdg (ndarray): Commanded body heading [deg], decoupled from track.
        swselhdg (ndarray): Bool switch: a body heading was commanded. While
            False the nose follows the track (nose-along-course default).
        yawrate (ndarray): Maximum yaw rate [deg/s].
    """

    def __init__(self, traffic: Traffic) -> None:
        super().__init__(traffic)
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
        # TODO: set ismulticopter from self.traffic.typecode[-n:], seed
        # selhdg from the current heading, swselhdg False, yawrate default.

    def mask(self) -> np.ndarray:
        """Return the boolean row mask of aircraft flown as multicopters."""
        return self.ismulticopter

    def mcopt(self, idx: int, flag: bool | None = None) -> tuple[bool, str]:
        """Mark an aircraft as a multicopter (or report its current setting).

        Arguments:
        - idx: Aircraft callsign
        - flag: ON to fly it as a multicopter, OFF for normal fixed-wing
          kinematics (optional, omit to query)
        """
        # TODO: report or set self.ismulticopter[idx]
        return False, "MCOPT: not implemented yet"

    def yaw(self, idx: int, hdg: float) -> tuple[bool, str]:
        """Command the body heading (nose direction) of a multicopter.

        The velocity vector keeps following the track command from the FMS
        or conflict resolution, so this rotates the aircraft in place.

        Arguments:
        - idx: Aircraft callsign
        - hdg: Commanded body heading [deg]
        """
        # TODO: set self.selhdg[idx] / self.swselhdg[idx]
        return False, "YAW: not implemented yet"

    def setyawrate(self, idx: int, yawrate: float | None = None) -> tuple[bool, str]:
        """Set or report the maximum yaw rate of a multicopter.

        Arguments:
        - idx: Aircraft callsign
        - yawrate: Maximum yaw rate [deg/s] (optional, omit to query)
        """
        # TODO: report or set self.yawrate[idx]
        return False, "YAWRATE: not implemented yet"
