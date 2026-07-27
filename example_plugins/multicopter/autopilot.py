"""Multicopter mission autopilot.

A thin subclass of the core :class:`Autopilot`: LNAV already emits a *track*
command, which is exactly what the decoupled multicopter kinematics
consumes, so no guidance rewrite is needed. What the stock FMS cannot
express is added here — the ``HOVER`` and ``DELIVER`` mission primitives,
a fixed waypoint capture radius (the bank- and speed-based turn distance
degenerates at creeping speeds), and fly-over route defaults.

Selected with ``SELECTIMPL AUTOPILOT MULTICOPTERAUTOPILOT``.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

from minisky.traffic.autopilot import Autopilot

if TYPE_CHECKING:
    from minisky.simulation import Simulation
    from minisky.traffic import Traffic

#: Waypoint capture radius for multicopters [m].
CAPTURE_RADIUS = 10.0

# Mission states
MISSION_NONE = 0
MISSION_HOVER = 1
MISSION_DELIVER = 2


class MulticopterAutopilot(Autopilot):
    """Autopilot with multicopter mission primitives.

    Attributes:
        mission (ndarray): Current mission state (one of ``MISSION_*``).
        missiontimer (ndarray): Remaining dwell time of the active mission
            state [s].
        missionalt (ndarray): Target altitude of an active DELIVER [m].
        resumealt (ndarray): Altitude to climb back to after a DELIVER [m].
    """

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__(traffic, get_simulation)
        with self.settrafarrays():
            self.mission = np.array([], dtype=int)
            self.missiontimer = np.array([])
            self.missionalt = np.array([])
            self.resumealt = np.array([])

    def create(self, n: int = 1) -> None:
        """Seed the mission state of n newly created aircraft.

        New aircraft start with no mission active.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)
        # TODO: mission = MISSION_NONE, timers/altitudes zeroed.

    def update(self) -> None:
        """Run the FMS, then the multicopter mission state machine.

        After the base autopilot update, advances any active HOVER/DELIVER
        state (counting the dwell timers down and resuming the route when
        they expire) and clamps the waypoint turn distance of multicopter
        rows to a fixed capture radius.
        """
        super().update()
        # TODO: advance mission timers/state machine for multicopter rows.
        # TODO: clamp actwp.turndist[m] to CAPTURE_RADIUS.

    def hover(self, idx: int, duration: float | None = None) -> tuple[bool, str]:
        """Hold position, optionally for a fixed duration.

        Suspends LNAV and commands zero ground speed. With a duration, the
        route resumes automatically once it has elapsed; without one, the
        aircraft hovers until LNAV is re-engaged.

        Arguments:
        - idx: Aircraft callsign
        - duration: Hold time [s] (optional, omit to hover indefinitely)
        """
        # TODO: enter MISSION_HOVER for idx
        return False, "HOVER: not implemented yet"

    def deliver(self, idx: int, alt: float, dwell: float | None = None) -> tuple[bool, str]:
        """Descend vertically to an altitude, dwell, climb back, resume route.

        The horizontal position is held throughout, so lat/lon are unchanged
        for the whole manoeuvre.

        Arguments:
        - idx: Aircraft callsign
        - alt: Delivery altitude [ft or FL]
        - dwell: Time spent at the delivery altitude [s] (optional)
        """
        # TODO: enter MISSION_DELIVER for idx
        return False, "DELIVER: not implemented yet"
