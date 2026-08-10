"""Multicopter waypoint capture.

The stock waypoint-switching criterion turns at a distance derived from the
bank-angle turn radius, which degenerates at multicopter speeds: it shrinks
to nothing at creeping speeds, and a hovering aircraft sitting on top of its
waypoint would never switch at all. Multicopter rows use a fixed capture
radius instead.

This must live in an :class:`ActiveWaypoint` subclass (selected with
``SELECTIMPL ACTIVEWAYPOINT MULTICOPTERACTIVEWAYPOINT``) because
:meth:`ActiveWaypoint.reached` recomputes ``turndist`` from the bank-angle
formula every step — clamping it from the autopilot update would be
overwritten before it is ever used.
"""

from __future__ import annotations

import numpy as np
from minisky import plugin as plugin_api
from minisky import quantities as q
from minisky.traffic.activewpdata import ActiveWaypoint

from minisky_multicopter.entity import get_multicopter

#: Waypoint capture radius for multicopters [m].
CAPTURE_RADIUS: q.DistanceM[float] = 10.0


@plugin_api.replacement
class MulticopterActiveWaypoint(ActiveWaypoint):
    """Active-waypoint data with a fixed capture radius for multicopters."""

    def reached(
        self,
        qdr: q.BearingDeg[np.ndarray],
        dist: q.DistanceM[np.ndarray],
        flyby: np.ndarray,
        flyturn: np.ndarray,
        turnrad: q.TurnRadiusM[np.ma.MaskedArray],
        turnhdgr: q.TurnRateDegPerS[np.ma.MaskedArray],
        swlastwp: np.ndarray,
    ) -> np.ndarray:
        """Determine which aircraft have reached their active waypoint.

        Runs the base criterion for the whole fleet, then overrides the turn
        distance of multicopter rows with the fixed capture radius and also
        counts them as reached when within it.

        `turnrad` and `turnhdgr` are masked when unspecified. `swlastwp`
        marks aircraft whose active waypoint is their final waypoint.

        Returns the indices of aircraft that reached their waypoint.
        """
        swreached = super().reached(qdr, dist, flyby, flyturn, turnrad, turnhdgr, swlastwp)
        mc = get_multicopter(self.traffic)
        if mc is None or not mc.ismulticopter.any():
            return swreached

        m = mc.ismulticopter
        self.turndist[m] = CAPTURE_RADIUS
        captured = np.where(m & self.traffic.swlnav & (dist < CAPTURE_RADIUS))[0]
        return np.union1d(swreached, captured).astype(int)
