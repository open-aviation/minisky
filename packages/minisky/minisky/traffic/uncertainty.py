"""ADS-B model. Implements real-life limitations of ADS-B communication.

Maintains, per aircraft, the most recently "broadcast" state as it would
be received via ADS-B: position and altitude with optional transmission
noise, and updates limited to a configurable truncation interval. This
surveillance noise is part of the trajectory noise switched on/off with
the NOISE stack command (see Traffic.setnoise()).
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

from minisky import quantities as q
from minisky.core.trafficarrays import TrafficArrays

if TYPE_CHECKING:
    from minisky.simulation import Simulation
    from minisky.traffic import Traffic


class SurveillanceUncertainty(TrafficArrays):
    """ADS-B model. Implements real-life limitations of ADS-B communication.

    Keeps a noisy, periodically refreshed copy of the true aircraft state,
    representing what surveillance-based systems would observe. Available
    as [`runtime.traffic.noise`][minisky.traffic.uncertainty.SurveillanceUncertainty].
    """

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__(traffic)
        self.traffic = traffic
        self._get_simulation = get_simulation
        with self.settrafarrays():
            self.lastupdate: q.SimulationTimeS[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.lat: q.LatitudeDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.lon: q.LongitudeDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.alt: q.PressureAltitudeM[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.trk: q.GroundTrackDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.tas: q.TrueAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.gs: q.GroundSpeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.vs: q.VerticalRateMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]

        self.setnoise(False)

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic and simulation."""
        return implementation(self.traffic, self._get_simulation)

    def setnoise(self, n: bool) -> None:
        """Switch surveillance noise on or off (part of the NOISE command)."""
        self.transnoise: bool = n
        """Whether transmission noise is added to surveillance updates."""
        self.truncated: bool = n
        """Whether surveillance refreshes are limited by `trunctime`."""
        self.position_noise_std: q.AngleDeg[float] = 1e-4  # pyright: ignore[reportGeneralTypeIssues]
        self.altitude_noise_std: q.VerticalDistanceM[float] = q.ft_to_m(100.0)  # pyright: ignore[reportGeneralTypeIssues]
        self.trunctime: q.DurationS[float] = 0.0  # pyright: ignore[reportGeneralTypeIssues]

    def create(self, n: int = 1) -> None:
        """Initialize broadcast data for n newly created aircraft.

        Copies the true state as the first broadcast and randomizes the
        initial update times so aircraft do not all broadcast in the same
        simulation step.

        Args:
            n: Number of aircraft appended to the traffic arrays.
        """
        super().create(n)

        self.lastupdate[-n:] = -self.trunctime * self.traffic.numpy_random.rand(n)
        self.lat[-n:] = self.traffic.lat[-n:]
        self.lon[-n:] = self.traffic.lon[-n:]
        self.alt[-n:] = self.traffic.alt[-n:]
        self.trk[-n:] = self.traffic.trk[-n:]
        self.tas[-n:] = self.traffic.tas[-n:]
        self.gs[-n:] = self.traffic.gs[-n:]
        self.vs[-n:] = self.traffic.vs[-n:]

    def update(self) -> None:
        """Refresh the broadcast state of aircraft that are due an update.

        Called every simulation step. For aircraft whose last broadcast is
        older than the truncation interval, the broadcast position and
        altitude are copied from the true state, with Gaussian transmission
        noise added when enabled; track and speeds are copied unmodified.
        """
        up = np.where(self.lastupdate + self.trunctime < self._get_simulation().simt)
        nup = len(up[0])
        if self.transnoise:
            self.lat[up] = self.traffic.lat[up] + self.traffic.numpy_random.normal(
                0, self.position_noise_std, nup
            )
            self.lon[up] = self.traffic.lon[up] + self.traffic.numpy_random.normal(
                0, self.position_noise_std, nup
            )
            self.alt[up] = self.traffic.alt[up] + self.traffic.numpy_random.normal(
                0, self.altitude_noise_std, nup
            )
        else:
            self.lat[up] = self.traffic.lat[up]
            self.lon[up] = self.traffic.lon[up]
            self.alt[up] = self.traffic.alt[up]
        self.trk[up] = self.traffic.trk[up]
        self.tas[up] = self.traffic.tas[up]
        self.gs[up] = self.traffic.gs[up]
        self.vs[up] = self.traffic.vs[up]
        self.lastupdate[up] = self.lastupdate[up] + self.trunctime
