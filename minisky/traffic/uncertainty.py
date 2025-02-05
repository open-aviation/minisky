"""ADS-B model. Implements real-life limitations of ADS-B communication."""

import numpy as np

import minisky
from minisky.core.trafficarrays import TrafficArrays
from minisky.tools.aero import ft


class SurveillanceUncertainty(TrafficArrays):
    """ADS-B model. Implements real-life limitations of ADS-B communication."""

    def __init__(self):
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

    def setnoise(self, n):
        self.transnoise = n
        self.truncated = n
        self.transerror = [
            1e-4,
            100 * ft,
        ]  # [degree,m] standard lat/lon distance, altitude error
        self.trunctime = 0  # [s]

    def create(self, n=1):
        super().create(n)

        self.lastupdate[-n:] = -self.trunctime * np.random.rand(n)
        self.lat[-n:] = minisky.traf.lat[-n:]
        self.lon[-n:] = minisky.traf.lon[-n:]
        self.alt[-n:] = minisky.traf.alt[-n:]
        self.trk[-n:] = minisky.traf.trk[-n:]
        self.tas[-n:] = minisky.traf.tas[-n:]
        self.gs[-n:] = minisky.traf.gs[-n:]

    def update(self):
        up = np.where(self.lastupdate + self.trunctime < minisky.sim.simt)
        nup = len(up)
        if self.transnoise:
            self.lat[up] = minisky.traf.lat[up] + np.random.normal(
                0, self.transerror[0], nup
            )
            self.lon[up] = minisky.traf.lon[up] + np.random.normal(
                0, self.transerror[0], nup
            )
            self.alt[up] = minisky.traf.alt[up] + np.random.normal(
                0, self.transerror[1], nup
            )
        else:
            self.lat[up] = minisky.traf.lat[up]
            self.lon[up] = minisky.traf.lon[up]
            self.alt[up] = minisky.traf.alt[up]
        self.trk[up] = minisky.traf.trk[up]
        self.tas[up] = minisky.traf.tas[up]
        self.gs[up] = minisky.traf.gs[up]
        self.vs[up] = minisky.traf.vs[up]
        self.lastupdate[up] = self.lastupdate[up] + self.trunctime
