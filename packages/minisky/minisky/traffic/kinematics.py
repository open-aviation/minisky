"""Aircraft kinematics integration.

Defines :class:`Kinematics`, the first-level :class:`TrafficArrays` entity
that numerically integrates airspeed, heading, vertical speed, ground speed
and position of all aircraft each simulation step. The flight-integration
behaviour is a replaceable implementation: plugins may subclass it and
select it with ``SELECTIMPL KINEMATICS <IMPL>`` to change how (a subset of)
aircraft fly (e.g. yaw-rate-limited hover, thrust-redirected translation).

The base implementation owns the acceleration and turn/altitude-select state
arrays (``ax``, ``az``, ``swhdgsel``, ``swaltsel``); all other per-aircraft
state is read and written on the owning traffic object.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import numpy as np

from minisky import quantities as q
from minisky.core.trafficarrays import TrafficArrays
from minisky.tools.aero import Rearth, g0, vtas2cas, vtas2mach

if TYPE_CHECKING:
    from minisky.simulation import Simulation
    from minisky.traffic import Traffic


class Kinematics(TrafficArrays):
    """Integrate airspeed, heading, vertical speed and position each step.

    Replaceable via `SELECTIMPL KINEMATICS <IMPL>`; plugins may subclass
    to change how aircraft fly. Available at runtime as
    `runtime.traffic.kinematics`.
    """

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__(traffic)
        self.traffic = traffic
        self._get_simulation = get_simulation
        with self.settrafarrays():
            self.ax: q.AccelerationMps2[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.az: q.AccelerationMps2[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.swhdgsel = np.array([], dtype=bool)
            """Whether each aircraft is actively turning toward a selected heading."""
            self.swaltsel = np.array([], dtype=bool)
            """Whether each aircraft is actively capturing a selected altitude."""

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic and simulation."""
        return implementation(self.traffic, self._get_simulation)

    def create(self, n: int = 1) -> None:
        """Initialize the integration state for n newly created aircraft.

        New aircraft start in steady flight: no longitudinal or vertical
        acceleration, and neither the turn nor the altitude-capture mode
        engaged. Called from ``Traffic.create_children()`` after the traffic
        state of the new aircraft has been set, so subclasses may seed their
        own state from the owning traffic object.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)
        self.ax[-n:] = 0.0
        self.az[-n:] = 0.0
        self.swhdgsel[-n:] = False
        self.swaltsel[-n:] = False

    def update(self) -> None:
        """Integrate airspeed, heading, ground speed and position one step.

        Runs the three integration stages in order. Subclasses that only
        change how the aircraft accelerates or steers should override
        `update_airspeed` / `update_groundspeed` and let a single
        `update_pos` pass integrate the resulting velocity.
        """
        self.update_airspeed()
        self.update_groundspeed()
        self.update_pos()

    def update_airspeed(self) -> None:
        """Integrate true airspeed, heading and vertical speed over one step.

        Accelerates or decelerates towards the commanded TAS using the
        performance-limited longitudinal acceleration, turns towards the
        commanded heading with a turn rate that follows from the bank angle
        (commanded turn bank or default bank limit), and updates the vertical
        speed for the altitude select/capture/hold autopilot logic. Also
        refreshes the derived CAS and Mach values.
        """
        traf = self.traffic
        simdt = self._get_simulation().simdt
        delta_spd = traf.aporasas.tas - traf.tas
        need_ax = np.abs(delta_spd) > np.abs(simdt * traf.perf.axmax)
        self.ax = need_ax * np.sign(delta_spd) * traf.perf.axmax
        traf.tas = np.where(need_ax, traf.tas + self.ax * simdt, traf.aporasas.tas)
        traf.cas = vtas2cas(traf.tas, traf.alt)
        traf.M = vtas2mach(traf.tas, traf.alt)

        # Turning bank triangle
        # tan phi = a centrigugal/a grav = omega^2 * R / g = omega * V /g
        # => omega = (g tan phi)/V
        turnrate = np.degrees(
            g0
            * np.tan(
                np.where(
                    traf.ap.turnphi > traf.eps * traf.eps,
                    traf.ap.turnphi,
                    traf.ap.bankdef,
                )
            )
            / np.maximum(traf.tas, traf.eps)
        )
        delhdg = (traf.aporasas.hdg - traf.hdg + 180) % 360 - 180  # [deg]
        self.swhdgsel = np.abs(delhdg) > np.abs(simdt * turnrate)

        traf.hdg = (
            np.where(
                self.swhdgsel,
                traf.hdg + simdt * turnrate * np.sign(delhdg),
                traf.aporasas.hdg,
            )
            % 360.0
        )

        delta_alt = traf.aporasas.alt - traf.alt
        # Old dead band version:
        #        self.swaltsel = np.abs(delta_alt) > np.maximum(
        #            10 * ft, np.abs(2 * simdt * self.vs))

        # Update version: time based engage of altitude capture (to adapt for UAV vs airliner scale)
        self.swaltsel = np.abs(delta_alt) > 1.05 * np.maximum(
            np.abs(simdt * traf.aporasas.vs),
            np.abs(simdt * traf.vs),
        )
        target_vs = self.swaltsel * np.sign(delta_alt) * np.abs(traf.aporasas.vs)
        delta_vs = target_vs - traf.vs
        vertical_acceleration: q.AccelerationMps2[float] = q.fpm_per_s_to_mps2(300.0)
        need_az = np.abs(delta_vs) > vertical_acceleration * simdt
        self.az = need_az * np.sign(delta_vs) * vertical_acceleration
        traf.vs = np.where(need_az, traf.vs + self.az * simdt, target_vs)
        traf.vs = np.where(np.isfinite(traf.vs), traf.vs, 0)  # fix vs nan issue

    def update_groundspeed(self) -> None:
        """Compute ground speed and track from heading, airspeed and wind.

        Without wind, ground speed equals TAS and track equals heading. With
        a wind field defined, the wind vector at each aircraft position is
        added to the airspeed vector (only when airborne, above 50 ft). Also
        accumulates the work done by the engines [J] along the flown path.
        """
        traf = self.traffic
        simdt = self._get_simulation().simdt
        # Compute ground speed and track from heading, airspeed and wind
        if not traf.wind.has_wind:  # no wind
            traf.gsnorth = traf.tas * np.cos(np.radians(traf.hdg))
            traf.gseast = traf.tas * np.sin(np.radians(traf.hdg))

            traf.gs = traf.tas
            traf.trk = traf.hdg
            traf.windnorth[:], traf.windeast[:] = 0.0, 0.0

        else:
            applywind = traf.alt > q.ft_to_m(50.0)  # Only apply wind when airborne

            vnwnd, vewnd = traf.wind.getdata(traf.lat, traf.lon, traf.alt)
            traf.windnorth[:], traf.windeast[:] = vnwnd, vewnd
            traf.gsnorth = traf.tas * np.cos(np.radians(traf.hdg)) + traf.windnorth * applywind
            traf.gseast = traf.tas * np.sin(np.radians(traf.hdg)) + traf.windeast * applywind

            traf.gs = np.logical_not(applywind) * traf.tas + applywind * np.sqrt(
                traf.gsnorth**2 + traf.gseast**2
            )

            traf.trk = (
                np.logical_not(applywind) * traf.hdg
                + applywind * np.degrees(np.arctan2(traf.gseast, traf.gsnorth)) % 360.0
            )

        traf.work += traf.perf.thrust * simdt * np.sqrt(traf.gs * traf.gs + traf.vs * traf.vs)

    def update_pos(self) -> None:
        """Integrate altitude and lat/lon position over one time step.

        Altitude follows the vertical speed while the altitude-select mode is
        engaged, and snaps to the commanded altitude otherwise. Latitude and
        longitude are advanced with the ground speed components using a
        spherical-Earth approximation, and the flown distance is accumulated.
        """
        traf = self.traffic
        simdt = self._get_simulation().simdt
        traf.alt = np.where(
            self.swaltsel,
            np.round(traf.alt + traf.vs * simdt, 6),
            traf.aporasas.alt,
        )
        traf.lat = traf.lat + np.degrees(simdt * traf.gsnorth / Rearth)
        traf.coslat = np.cos(np.deg2rad(traf.lat))
        traf.lon = traf.lon + np.degrees(simdt * traf.gseast / traf.coslat / Rearth)
        traf.distflown += traf.gs * simdt
