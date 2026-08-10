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

    Attributes:
        lastupdate (ndarray): Simulation time of the last broadcast per
            aircraft [s].
        lat (ndarray): Last broadcast latitude [deg].
        lon (ndarray): Last broadcast longitude [deg].
        alt (ndarray): Last broadcast altitude [m].
        trk (ndarray): Last broadcast track angle [deg].
        tas (ndarray): Last broadcast true airspeed [m/s].
        gs (ndarray): Last broadcast ground speed [m/s].
        vs (ndarray): Last broadcast vertical speed [m/s].
        transnoise (bool): Whether transmission noise is added.
        truncated (bool): Whether updates are truncated to the update
            interval.
        transerror (list): Standard deviations of the transmission noise:
            [lat/lon error [deg], altitude error [m]].
        trunctime (float): Minimum time between broadcast updates [s].
    """

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__(traffic)
        self.traffic = traffic
        self._get_simulation = get_simulation
        # From here, define object arrays
        with self.settrafarrays():
            # Most recent broadcast data
            self.lastupdate = np.array([])
            self.lat = np.array([])
            self.lon = np.array([])
            self.alt = np.array([])
            self.trk = np.array([])
            self.tas = np.array([])
            self.gs = np.array([])
            self.vs = np.array([])

        self.setnoise(False)

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic and simulation."""
        return implementation(self.traffic, self._get_simulation)

    def setnoise(self, n: bool) -> None:
        """Switch surveillance noise on or off (part of the NOISE command).

        Args:
            n: True to enable transmission noise and truncation, False to
                disable them.
        """
        self.transnoise = n
        self.truncated = n
        self.transerror = [
            1e-4,
            q.ft_to_m(100.0),
        ]  # [degree,m] standard lat/lon distance, altitude error
        self.trunctime = 0  # [s]

    def create(self, n: int = 1) -> None:
        """Initialize broadcast data for n newly created aircraft.

        Copies the true state as the first broadcast and randomizes the
        initial update times so aircraft do not all broadcast in the same
        simulation step.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)

        self.lastupdate[-n:] = -self.trunctime * self.traffic.numpy_random.rand(n)
        self.lat[-n:] = self.traffic.lat[-n:]
        self.lon[-n:] = self.traffic.lon[-n:]
        self.alt[-n:] = self.traffic.alt[-n:]
        self.trk[-n:] = self.traffic.trk[-n:]
        self.tas[-n:] = self.traffic.tas[-n:]
        self.gs[-n:] = self.traffic.gs[-n:]

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
                0, self.transerror[0], nup
            )
            self.lon[up] = self.traffic.lon[up] + self.traffic.numpy_random.normal(
                0, self.transerror[0], nup
            )
            self.alt[up] = self.traffic.alt[up] + self.traffic.numpy_random.normal(
                0, self.transerror[1], nup
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
