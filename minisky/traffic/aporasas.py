"""Pilot logic.

Selects, per aircraft and per control channel, whether the aircraft
follows the autopilot/FMS command or the conflict-resolution (ASAS)
command. The resulting desired states are the setpoints that
:class:`~minisky.traffic.traffic.Traffic` flies towards each time step
(after being limited by the performance model).
"""

import numpy as np

import minisky
from minisky.core import TrafficArrays


class APorASAS(TrafficArrays):
    """Selection between autopilot (AP) and conflict resolution (ASAS).

    For each control channel (track, speed, altitude, vertical speed) the
    ASAS command is used when the corresponding conflict-resolution channel
    is active, otherwise the autopilot command is used. The desired heading
    is derived from the desired track with a wind-drift correction.
    Available at runtime as ``minisky.traf.aporasas``.

    Attributes:
        alt (ndarray): Desired altitude [m].
        hdg (ndarray): Desired heading [deg].
        trk (ndarray): Desired track angle [deg].
        vs (ndarray): Desired vertical speed (magnitude) [m/s].
        tas (ndarray): Desired true airspeed [m/s].
    """

    def __init__(self) -> None:
        super().__init__()
        with self.settrafarrays():
            # Desired aircraft states
            self.alt = np.array([])  # desired altitude [m]
            self.hdg = np.array([])  # desired heading [deg]
            self.trk = np.array([])  # desired track angle [deg]
            self.vs = np.array([])  # desired vertical speed [m/s]
            self.tas = np.array([])  # desired speed [m/s]

    def create(self, n: int = 1) -> None:
        """Initialize desired states for n newly created aircraft.

        The desired altitude, speed, heading and track are copied from the
        current traffic state so new aircraft start in steady flight.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)
        self.alt[-n:] = minisky.traf.alt[-n:]
        self.tas[-n:] = minisky.traf.tas[-n:]
        self.hdg[-n:] = minisky.traf.hdg[-n:]
        self.trk[-n:] = minisky.traf.trk[-n:]

    def update(self) -> None:
        """Select the desired aircraft states from autopilot or ASAS.

        Called every simulation step, after conflict resolution and before
        the traffic state integration. Per channel (track, speed, altitude,
        vertical speed) the conflict-resolution command is selected when
        that resolution channel is active, otherwise the autopilot command.
        The ASAS speed advisory (a ground speed) is converted to TAS using
        the local wind, the vertical speed is stored as a magnitude, and
        the desired heading is computed from the desired track with a
        wind-drift correction.
        """
        # --------- Input to Autopilot settings to follow: destination or ASAS ----------
        # Convert the ASAS commanded speed from ground speed to TAS
        if minisky.traf.wind.winddim > 0:
            vwn, vwe = minisky.traf.wind.getdata(
                minisky.traf.lat, minisky.traf.lon, minisky.traf.alt
            )
            asastasnorth = minisky.traf.cr.tas * np.cos(np.radians(minisky.traf.cr.trk)) - vwn
            asastaseast = minisky.traf.cr.tas * np.sin(np.radians(minisky.traf.cr.trk)) - vwe
            asastas = np.sqrt(asastasnorth**2 + asastaseast**2)
        # no wind, then ground speed = TAS
        else:
            asastas = minisky.traf.cr.tas  # TAS [m/s]

        # Select asas if there is a conflict AND resolution is on
        # Determine desired states per channel whether to use value from ASAS or AP.
        # minisky.traf.cr.active may be used as well, will set all of these channels
        self.trk = np.where(minisky.traf.cr.hdgactive, minisky.traf.cr.trk, minisky.traf.ap.trk)
        self.tas = np.where(minisky.traf.cr.tasactive, asastas, minisky.traf.ap.tas)
        self.alt = np.where(minisky.traf.cr.altactive, minisky.traf.cr.alt, minisky.traf.ap.alt)
        self.vs = np.where(minisky.traf.cr.vsactive, minisky.traf.cr.vs, minisky.traf.ap.vs)

        # ASAS can give positive and negative VS, but the sign of VS is determined using delalt in Traf.ComputeAirSpeed
        # Therefore, ensure that pilot.vs is always positive to prevent opposite signs of delalt and VS in Traf.ComputeAirSpeed
        self.vs = np.abs(self.vs)

        # Compute the desired heading needed to compensate for the wind
        if minisky.traf.wind.winddim > 0:
            # Calculate wind correction
            vwn, vwe = minisky.traf.wind.getdata(
                minisky.traf.lat, minisky.traf.lon, minisky.traf.alt
            )
            Vw = np.sqrt(vwn * vwn + vwe * vwe)
            winddir = np.arctan2(vwe, vwn)
            drift = np.radians(self.trk) - winddir  # [rad]
            steer = np.arcsin(
                np.minimum(
                    1.0,
                    np.maximum(-1.0, Vw * np.sin(drift) / np.maximum(0.001, minisky.traf.tas)),
                )
            )
            # desired heading
            self.hdg = (self.trk + np.degrees(steer)) % 360.0
        else:
            self.hdg = self.trk % 360.0
