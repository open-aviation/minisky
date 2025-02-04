"""Simple turbulence implementation."""

import numpy as np

import minisky
from minisky.core import Entity
from minisky.tools.aero import Rearth


class Turbulence(Entity, replaceable=True):
    """Simple turbulence implementation."""

    def __init__(self):
        self.active = False
        self.sd = np.array([])

    def reset(self):
        self.active = False
        self.SetStandards([0, 0.1, 0.1])

    def setnoise(self, flag):
        self.active = flag

    def SetStandards(self, s):
        self.sd = np.array(s)  # m/s standard turbulence  (nonnegative)
        # in (horizontal flight direction, horizontal wing direction, vertical)
        self.sd = np.where(self.sd > 1e-6, self.sd, 1e-6)

    def update(self):
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
        minisky.traf.lon = minisky.traf.lon + np.degrees(
            turblon / Rearth / minisky.traf.coslat
        )
