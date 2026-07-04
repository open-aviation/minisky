"""ADS-B model. Implements real-life limitations of ADS-B communication.

Maintains, per aircraft, the most recently "broadcast" state as it would
be received via ADS-B: position and altitude with optional transmission
noise, and updates limited to a configurable truncation interval. This
surveillance noise is part of the trajectory noise switched on/off with
the NOISE stack command (see Traffic.setnoise()).
"""

import numpy as np

import minisky
from minisky.core.trafficarrays import TrafficArrays
from minisky.tools.aero import ft


class SurveillanceUncertainty(TrafficArrays):
    """ADS-B model. Implements real-life limitations of ADS-B communication.

    Keeps a noisy, periodically refreshed copy of the true aircraft state,
    representing what surveillance-based systems would observe. Available
    at runtime as ``minisky.traf.noise``.

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

    def __init__(self) -> None:
        super().__init__()
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
            100 * ft,
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

        self.lastupdate[-n:] = -self.trunctime * np.random.rand(n)
        self.lat[-n:] = minisky.traf.lat[-n:]
        self.lon[-n:] = minisky.traf.lon[-n:]
        self.alt[-n:] = minisky.traf.alt[-n:]
        self.trk[-n:] = minisky.traf.trk[-n:]
        self.tas[-n:] = minisky.traf.tas[-n:]
        self.gs[-n:] = minisky.traf.gs[-n:]

    def update(self) -> None:
        """Refresh the broadcast state of aircraft that are due an update.

        Called every simulation step. For aircraft whose last broadcast is
        older than the truncation interval, the broadcast position and
        altitude are copied from the true state, with Gaussian transmission
        noise added when enabled; track and speeds are copied unmodified.
        """
        up = np.where(self.lastupdate + self.trunctime < minisky.sim.simt)
        nup = len(up)
        if self.transnoise:
            self.lat[up] = minisky.traf.lat[up] + np.random.normal(0, self.transerror[0], nup)
            self.lon[up] = minisky.traf.lon[up] + np.random.normal(0, self.transerror[0], nup)
            self.alt[up] = minisky.traf.alt[up] + np.random.normal(0, self.transerror[1], nup)
        else:
            self.lat[up] = minisky.traf.lat[up]
            self.lon[up] = minisky.traf.lon[up]
            self.alt[up] = minisky.traf.alt[up]
        self.trk[up] = minisky.traf.trk[up]
        self.tas[up] = minisky.traf.tas[up]
        self.gs[up] = minisky.traf.gs[up]
        self.vs[up] = minisky.traf.vs[up]
        self.lastupdate[up] = self.lastupdate[up] + self.trunctime
