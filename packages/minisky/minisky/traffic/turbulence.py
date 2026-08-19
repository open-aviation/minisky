"""Simple turbulence implementation.

Adds zero-mean Gaussian position perturbations to all aircraft each
simulation step, scaled with the square root of the time step. Turbulence
is part of the trajectory noise that is switched on/off with the NOISE
stack command (see Traffic.setnoise()).
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

from minisky import quantities as q
from minisky.core.trafficarrays import TrafficArrays
from minisky.tools.aero import Rearth

if TYPE_CHECKING:
    from minisky.simulation.simulation import Simulation
    from minisky.traffic.traffic import Traffic


class Turbulence(TrafficArrays):
    """Simple stochastic turbulence model.

    When active, random displacements are drawn per aircraft in the
    body-related axes (along track, across track, vertical) and applied
    directly to the aircraft positions and altitudes.
    """

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__(traffic)
        self.traffic = traffic
        self._get_simulation = get_simulation
        self.active = False
        self.set_standards([0, 0.1, 0.1])

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic and simulation."""
        return implementation(self.traffic, self._get_simulation)

    def reset(self) -> None:
        """Switch turbulence off and restore the default standard deviations."""
        self.active = False
        self.set_standards([0, 0.1, 0.1])

    def setnoise(self, flag: bool) -> None:
        """Switch the turbulence model on or off (part of the NOISE command)."""
        self.active = flag

    def set_standards(self, s: q.PositionDiffusionMPerSqrtS) -> None:
        """Set the turbulence standard deviations.

        Values are ordered horizontal-flight, horizontal-wing, vertical and
        clipped to a small positive minimum.
        """
        self.sd: q.PositionDiffusionMPerSqrtS[np.ndarray] = np.asarray(s, dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        """Diffusion amplitudes along-track, cross-track, and vertical."""
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

        timescale = np.sqrt(self._get_simulation().simdt)
        # Horizontal flight direction
        turbhf = self.traffic.numpy_random.normal(
            0, self.sd[0] * timescale, self.traffic.ntraf
        )  # [m]

        # Horizontal wing direction
        turbhw = self.traffic.numpy_random.normal(
            0, self.sd[1] * timescale, self.traffic.ntraf
        )  # [m]

        # Vertical direction
        turbalt = self.traffic.numpy_random.normal(
            0, self.sd[2] * timescale, self.traffic.ntraf
        )  # [m]

        trkrad = np.radians(self.traffic.trk)
        turblat = np.cos(trkrad) * turbhf - np.sin(trkrad) * turbhw  # [m]
        turblon = np.sin(trkrad) * turbhf + np.cos(trkrad) * turbhw  # [m]

        self.traffic.alt = self.traffic.alt + turbalt
        self.traffic.lat = self.traffic.lat + np.degrees(turblat / Rearth)
        self.traffic.lon = self.traffic.lon + np.degrees(turblon / Rearth / self.traffic.coslat)
