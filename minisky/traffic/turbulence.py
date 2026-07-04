"""Simple turbulence implementation.

Adds zero-mean Gaussian position perturbations to all aircraft each
simulation step, scaled with the square root of the time step. Turbulence
is part of the trajectory noise that is switched on/off with the NOISE
stack command (see Traffic.setnoise()).
"""

from typing import Any

import numpy as np

import minisky
from minisky.core.trafficarrays import TrafficArrays
from minisky.tools.aero import Rearth


class Turbulence(TrafficArrays):
    """Simple stochastic turbulence model.

    When active, random displacements are drawn per aircraft in the
    body-related axes (along track, across track, vertical) and applied
    directly to the aircraft positions and altitudes.

    Attributes:
        active (bool): Whether turbulence is applied.
        sd (ndarray): Turbulence standard deviations [m/s] in (horizontal
            flight direction, horizontal wing direction, vertical);
            clipped to a small positive minimum.
    """

    def __init__(self) -> None:
        self.active = False
        self.sd = np.array([])

    def reset(self) -> None:
        """Switch turbulence off and restore the default standard deviations."""
        self.active = False
        self.SetStandards([0, 0.1, 0.1])

    def setnoise(self, flag: bool) -> None:
        """Switch the turbulence model on or off (part of the NOISE command).

        Args:
            flag: True to enable turbulence, False to disable it.
        """
        self.active = flag

    def SetStandards(self, s: Any) -> None:
        """Set the turbulence standard deviations.

        Args:
            s: Sequence of three standard deviations [m/s]: (horizontal
                flight direction, horizontal wing direction, vertical).
                Values are clipped to a small positive minimum.
        """
        self.sd = np.array(s)  # m/s standard turbulence  (nonnegative)
        # in (horizontal flight direction, horizontal wing direction, vertical)
        self.sd = np.where(self.sd > 1e-6, self.sd, 1e-6)

    def update(self) -> None:
        """Apply one time step of random turbulence displacements.

        Draws zero-mean Gaussian displacements [m] per aircraft (scaled
        with sqrt(simdt)), rotates the horizontal components from the
        body axes to north/east using the current track, and adds them
        to the aircraft latitude, longitude and altitude. Does nothing
        when turbulence is inactive.
        """
        if not self.active:
            return

        timescale = np.sqrt(minisky.sim.simdt)
        # Horizontal flight direction
        turbhf = np.random.normal(0, self.sd[0] * timescale, minisky.traf.ntraf)  # [m]

        # Horizontal wing direction
        turbhw = np.random.normal(0, self.sd[1] * timescale, minisky.traf.ntraf)  # [m]

        # Vertical direction
        turbalt = np.random.normal(0, self.sd[2] * timescale, minisky.traf.ntraf)  # [m]

        trkrad = np.radians(minisky.traf.trk)
        # Lateral, longitudinal direction
        turblat = np.cos(trkrad) * turbhf - np.sin(trkrad) * turbhw  # [m]
        turblon = np.sin(trkrad) * turbhf + np.cos(trkrad) * turbhw  # [m]

        # Update the aircraft locations
        minisky.traf.alt = minisky.traf.alt + turbalt
        minisky.traf.lat = minisky.traf.lat + np.degrees(turblat / Rearth)
        minisky.traf.lon = minisky.traf.lon + np.degrees(turblon / Rearth / minisky.traf.coslat)
