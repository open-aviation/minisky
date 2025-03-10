"""This module provides PerfBase, the base class for aircraft
performance implementations.
"""

import numpy as np

from minisky.core.trafficarrays import TrafficArrays


class PerfBase(TrafficArrays):
    """Base class for BlueSky aircraft performance implementations."""

    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            # --- fixed parameters ---
            self.actype = np.array([], dtype=str)  # aircraft type
            self.Sref = np.array([])  # wing reference surface area [m^2]
            self.engtype = np.array([])  # integer, aircraft.ENG_TF...

            # --- dynamic parameters ---
            self.mass = np.array([])  # effective mass [kg]
            self.phase = np.array([])
            self.cd0 = np.array([])
            self.k = np.array([])
            self.bank = np.array([])
            self.thrust = np.array([])  # thrust
            self.drag = np.array([])  # drag
            self.fuelflow = np.array([])  # fuel flow

            # Envelope limits per aircraft
            self.hmax = np.array([])  # Flight ceiling [m]
            self.vmin = np.array([])  # Minimum operating speed [m/s]
            self.vmax = np.array([])  # Maximum operating speed [m/s]
            self.vsmin = np.array([])  # Maximum descent speed [m/s]
            self.vsmax = np.array([])  # Maximum climb speed [m/s]
            self.axmax = np.array([])  # Max/min acceleration [m/s2]

    def create(self, n):
        super().create(n=n)
        # Set wide default limits, so that no envelope limiting occurs
        # when the actual performance model used doesn't support it
        self.axmax[-n:] = 2.0  # Default acceleration limit is 2 m/s2
        self.hmax[-n:] = 1e6
        self.vmin[-n:] = -1e6
        self.vmax[-n:] = 1e6
        self.vsmin[-n:] = -1e6
        self.vsmax[-n:] = 1e6

    def update(self, dt=1):
        """implement this method"""
        pass

    def limits(self, intent_v, intent_vs, intent_h, ax):
        """implement this method"""
        return intent_v, intent_vs, intent_h

    def currentlimits(self):
        """implement this method"""
        # Get current kinematic performance envelop of all aircraft
        pass
