"""Conflict resolution base class.

This module provides [`ConflictResolution`][minisky.traffic.asas.resolution.ConflictResolution], the base class for all
conflict resolution (CR) implementations in MiniSky. It manages the shared
resolution machinery: per-aircraft resolution advisories (heading, speed,
vertical speed, altitude), resolution zone margins relative to the detection
protected zone, priority rules, per-aircraft opt-outs (NORESO/RESOOFF), and
the logic that decides when an aircraft may resume normal navigation after a
conflict has been resolved ([`ConflictResolution.resumenav`][minisky.traffic.asas.resolution.ConflictResolution.resumenav]).

Actual resolution algorithms (e.g. the Modified Voltage Potential method in
`minisky.traffic.asas.mvp`) subclass this class and override
[`ConflictResolution.resolve`][minisky.traffic.asas.resolution.ConflictResolution.resolve].
"""

from __future__ import annotations

from collections.abc import Callable
from enum import Enum
from typing import TYPE_CHECKING, Annotated, Literal, NamedTuple

import numpy as np
from annotated_types import Ge

from minisky import quantities as q
from minisky.command import (
    AcIdSelection,
    DistanceM,
    OnOff,
    PositiveFiniteFloat,
    aircraft_indices,
    command,
)
from minisky.core.config import MiniSkyConfig
from minisky.core.trafficarrays import TrafficArrays
from minisky.result import Err, Ok, Result
from minisky.traffic import route

if TYPE_CHECKING:
    from minisky.traffic import Traffic

    from .detection import ConflictDetection


class PriorityCode(Enum):
    FF1 = "FF1"
    FF2 = "FF2"
    FF3 = "FF3"
    LAY1 = "LAY1"
    LAY2 = "LAY2"


PriorityCodeArg = Literal["FF1", "FF2", "FF3", "LAY1", "LAY2"]
ResolutionRadiusM = Annotated[DistanceM, Ge(0)]
ResolutionHeightM = Annotated[DistanceM, Ge(0)]
HorizontalResolutionMethod = Literal["BOTH", "SPD", "HDG", "NONE", "ON", "OFF", "OF"]
VerticalResolutionMethod = Literal["NONE", "ON", "OFF", "OF", "V/S"]


class ConflictResolution(TrafficArrays):
    """Base class for Conflict Resolution implementations.

    Each update step, when resolution is active and conflicts are detected,
    [`ConflictResolution.resolve`][minisky.traffic.asas.resolution.ConflictResolution.resolve] is called to compute resolution advisories for all
    aircraft. These advisories are stored in the per-aircraft arrays below and
    are followed by the autopilot for aircraft whose `active` flag is True.
    [`ConflictResolution.resumenav`][minisky.traffic.asas.resolution.ConflictResolution.resumenav] then decides per aircraft whether to keep following the
    resolution or to resume the flight plan (after the conflict pair has
    passed its closest point of approach).

    The base class itself performs no avoidance: its [`ConflictResolution.resolve`][minisky.traffic.asas.resolution.ConflictResolution.resolve] simply
    returns the autopilot values. Subclasses implement an actual algorithm.

    Attributes:
        activate (bool): Whether conflict resolution is switched on.
        priority_code: Selected priority rule set, or None when priority is off.
        resopairs (set): Conflict pairs that are being resolved and have not
            yet passed their CPA.
        resofach (float): Horizontal resolution zone factor relative to the
            detection zone radius [-].
        resofacv (float): Vertical resolution zone factor relative to the
            detection zone height [-].
        resooffac (ndarray): Per-aircraft flag, True for aircraft that do not
            perform resolutions themselves [-].
        noresoac (ndarray): Per-aircraft flag, True for aircraft that others
            do not avoid [-].
        active (ndarray): Per-aircraft flag, True while the autopilot follows
            the resolution advisory instead of the flight plan [-].
        trk (ndarray): Resolution heading advisory [deg].
        gs (ndarray): Resolution ground-speed advisory [m/s].
        alt (ndarray): Resolution altitude advisory [m].
        vs (ndarray): Resolution vertical speed advisory [m/s].
    """

    trk: q.GroundTrackDeg[np.ndarray]
    gs: q.GroundSpeedMps[np.ndarray]
    alt: q.PressureAltitudeM[np.ndarray]
    vs: q.VerticalRateMps[np.ndarray]

    def __init__(
        self,
        config: MiniSkyConfig,
        traffic: Traffic,
        select_implementation: Callable[[str, str], Result[str, str]],
    ) -> None:
        super().__init__()
        self.config = config
        self.traffic = traffic
        self.select_implementation = select_implementation
        self.activate = False

        self.priority_code: PriorityCode | None = None
        self.resopairs = set()  # Resolved conflicts that are still before CPA

        # Resolution factors:
        # set < 1 to maneuver only a fraction of the resolution
        # set > 1 to add a margin to separation values
        self.resofach = self.config.asas_marh
        self.resofacv = self.config.asas_marv

        # Switches to guarantee last reso zone commands keep valid if cd zone changes
        self.resodhrelative = (
            True  # Size of resolution zone dh, vertically, set relative to CD zone
        )
        self.resorrelative = True  # Size of resolution zone r, vertically, set relative to CD zone

        with self.settrafarrays():
            self.resooffac = np.array([], dtype=bool)
            self.noresoac = np.array([], dtype=bool)
            # whether the autopilot follows ASAS or not
            self.active = np.array([], dtype=bool)
            self.trk = np.array([])
            self.gs = np.array([])
            self.alt = np.array([])
            self.vs = np.array([])

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic and selector."""
        return implementation(self.config, self.traffic, self.select_implementation)

    def switch(self, flag: bool | None = None) -> None:
        """Turn conflict resolution on or off."""
        self.activate = flag

    def reset(self) -> None:
        """Reset the conflict resolution state to defaults.

        Called on simulation reset: clears priority settings and pending
        resolution pairs, and restores the resolution zone factors from the
        simulation config.
        """
        super().reset()
        self.priority_code = None
        self.resopairs.clear()
        self.resofach = self.config.asas_marh
        self.resofacv = self.config.asas_marv
        self.resodhrelative = True
        self.resorrelative = True

    # By default all channels are controlled by self.active,
    # but they can be overloaded with separate variables or functions in a
    # derived ASAS Conflict Resolution class (@property decorator takes away
    # need for brackets when calling it so it can be overloaded by a variable)
    @property
    def hdgactive(self) -> np.ndarray:
        """Return a boolean array sized according to the number of aircraft
        with True for all elements where heading is currently controlled by
        the conflict resolution algorithm.
        """
        return self.active

    @property
    def vsactive(self) -> np.ndarray:
        """Return a boolean array sized according to the number of aircraft
        with True for all elements where vertical speed is currently
        controlled by the conflict resolution algorithm.
        """
        return self.active

    @property
    def altactive(self) -> np.ndarray:
        """Return a boolean array sized according to the number of aircraft
        with True for all elements where altitude is currently controlled by
        the conflict resolution algorithm.
        """
        return self.active

    @property
    def gsactive(self) -> np.ndarray:
        """Return a boolean array sized according to the number of aircraft
        with True for all elements where speed is currently controlled by
        the conflict resolution algorithm.
        """
        return self.active

    class ResolutionAdvisories(NamedTuple):
        track: q.GroundTrackDeg[np.ndarray]
        ground_speed: q.GroundSpeedMps[np.ndarray]
        vertical_speed: q.VerticalRateMps[np.ndarray]
        altitude: q.PressureAltitudeM[np.ndarray]

    def resolve(
        self, conf: ConflictDetection, ownship: Traffic, intruder: Traffic
    ) -> ResolutionAdvisories:
        """Resolve all current conflicts.

        This function should be reimplemented in a subclass for actual
        resolution of conflicts. See for instance minisky.traffic.asas.mvp.
        The base implementation returns the autopilot values, i.e. no
        avoidance manoeuvre.

        Args:
            conf: Conflict detector containing the current conflicts.
            ownship: Ownship traffic state.
            intruder: Intruder traffic state.
        """
        # If resolution is off, and detection is on, and a conflict is detected
        # then asas will be active for that airplane. Since resolution is off, it
        # should then follow the auto pilot instructions.
        return self.ResolutionAdvisories(ownship.ap.trk, ownship.gs, ownship.ap.vs, ownship.ap.alt)

    def update(self, conf: ConflictDetection, ownship: Traffic, intruder: Traffic) -> None:
        """Perform an update step of the Conflict Resolution implementation.

        When resolution is active, computes new resolution advisories with
        [`ConflictResolution.resolve`][minisky.traffic.asas.resolution.ConflictResolution.resolve] if there are current conflicts, and updates which
        aircraft should keep following the resolution with [`ConflictResolution.resumenav`][minisky.traffic.asas.resolution.ConflictResolution.resumenav].

        Args:
            conf: Conflict detector containing the current conflicts.
            ownship: Ownship traffic state.
            intruder: Intruder traffic state.
        """
        if self.activate:
            if conf.confpairs:
                advisories = self.resolve(conf, ownship, intruder)
                # TODO(abraham): consider storing the entire advisories result
                self.trk = advisories.track
                self.gs = advisories.ground_speed
                self.vs = advisories.vertical_speed
                self.alt = advisories.altitude
            self.resumenav(conf, ownship, intruder)

    def resumenav(self, conf: ConflictDetection, ownship: Traffic, intruder: Traffic) -> None:
        """Decide for each aircraft in the conflict list whether the ASAS
        should be followed or not, based on if the aircraft pairs passed
        their CPA.

        An aircraft keeps following the resolution while its conflict pair
        has not yet passed the closest point of approach, while there still
        is horizontal loss of separation, or while the conflict is
        "bouncing" (near-parallel tracks repeatedly moving in and out of
        conflict). Once none of its conflicts require resolution anymore,
        the aircraft is released and directed back to its next active
        flight-plan waypoint.

        Args:
            conf: Conflict detector containing the current and recent conflict pairs.
            ownship: Ownship traffic state.
            intruder: Intruder traffic state.
        """
        # Add new conflicts to resopairs and confpairs_all and new losses to lospairs_all
        self.resopairs.update(conf.confpairs)

        # Conflict pairs to be deleted
        delpairs = set()
        changeactive = {}

        # smallest relative angle between vectors of heading a and b
        def anglediff(a: q.AngleDeg, b: q.AngleDeg) -> q.AngleDeg:
            d = a - b
            if d > 180:
                return anglediff(a, b + 360)
            elif d < -180:
                return anglediff(a + 360, b)
            else:
                return d

        # Look at all conflicts, also the ones that are solved but CPA is yet to come
        for conflict in self.resopairs:
            past_cpa = False
            hor_los = False
            is_bouncing = False
            idx1, idx2 = self.traffic.idx(conflict)
            # If the ownship aircraft is deleted remove its conflict from the list
            if idx1 is None:
                delpairs.add(conflict)
                continue

            if idx2 is not None:
                # Distance vector using flat earth approximation
                re: q.LengthM[float] = 6371000.0
                dist = re * np.array(
                    [
                        np.radians(intruder.lon[idx2] - ownship.lon[idx1])
                        * np.cos(0.5 * np.radians(intruder.lat[idx2] + ownship.lat[idx1])),
                        np.radians(intruder.lat[idx2] - ownship.lat[idx1]),
                    ]
                )

                # Relative velocity vector
                vrel = np.array(
                    [
                        intruder.gseast[idx2] - ownship.gseast[idx1],
                        intruder.gsnorth[idx2] - ownship.gsnorth[idx1],
                    ]
                )

                # Check if conflict is past CPA
                past_cpa = np.dot(dist, vrel) > 0.0

                rpz = np.max(conf.rpz[[idx1, idx2]])
                # hor_los:
                # Aircraft should continue to resolve until there is no horizontal
                # LOS. This is particularly relevant when vertical resolutions
                # are used.
                hdist = np.linalg.norm(dist)
                hor_los = hdist < rpz

                # Bouncing conflicts:
                # If two aircraft are getting in and out of conflict continously,
                # then they it is a bouncing conflict. ASAS should stay active until
                # the bouncing stops.
                is_bouncing = (
                    abs(anglediff(ownship.trk[idx1], intruder.trk[idx2])) < 30.0
                    and hdist < rpz * self.resofach
                )

            # Start recovery for ownship if intruder is deleted, or if past CPA
            # and not in horizontal LOS or a bouncing conflict
            if idx2 is not None and (not past_cpa or hor_los or is_bouncing):
                # Enable ASAS for this aircraft
                changeactive[idx1] = True
            else:
                # Switch ASAS off for ownship if there are no other conflicts
                # that this aircraft is involved in.
                changeactive[idx1] = changeactive.get(idx1, False)
                # If conflict is solved, remove it from the resopairs list
                delpairs.add(conflict)

        for idx, active in changeactive.items():
            # Loop a second time: this is to avoid that ASAS resolution is
            # turned off for an aircraft that is involved simultaneously in
            # multiple conflicts, where the first, but not all conflicts are
            # resolved.
            self.active[idx] = active
            if not active:
                # Waypoint recovery after conflict: Find the next active waypoint
                # and send the aircraft to that waypoint.
                iwpid = self.traffic.ap.route[idx].findact(idx)
                if iwpid is not None:  # To avoid problems if there are no waypoints
                    route.direct(self.traffic, idx, self.traffic.ap.route[idx].wpname[iwpid])

        # Remove pairs from the list that are past CPA or have deleted aircraft
        self.resopairs -= delpairs

    def priority_status(self) -> Result[str, str]:
        if self.__class__ is ConflictResolution:
            return Err("No conflict resolution enabled.")
        return Err(f"Resolution algorithm {self.__class__.__name__} hasn't implemented priority.")

    def configure_priority(self, flag: bool, priority_code: PriorityCode) -> Result[str, str]:
        self.priority_code = priority_code if flag else None
        return Ok("")

    @command(name="PRIORULES")
    def show_priority_rules(self) -> Result[str, str]:
        """Show priority-rule support and state."""
        return self.priority_status()

    @command(name="PRIORULES")
    def set_priority_rules(self, flag: OnOff, priority_code: PriorityCodeArg) -> Result[str, str]:
        """Enable or disable priority rules using a selected rule code."""
        return self.configure_priority(flag, PriorityCode(priority_code))

    @command(name="NORESO")
    def noreso_status(self) -> Result[str, str]:
        """Show aircraft that nobody will avoid."""
        aircraft = ", ".join(np.array(self.traffic.callsign)[self.noresoac])
        return Ok(
            "NORESO [ACID, ... ] OR NORESO [GROUPID]"
            f"\nCurrent list of aircraft nobody will avoid:{aircraft}"
        )

    @command(name="NORESO")
    def toggle_noreso(self, first: AcIdSelection, *additional: AcIdSelection) -> Result[str, str]:
        """Toggle the nobody-avoids flag for selected aircraft."""
        indices = aircraft_indices((first, *additional))
        self.noresoac[indices] = np.logical_not(self.noresoac[indices])
        return Ok("")

    @command(name="RESOOFF")
    def resooff_status(self) -> Result[str, str]:
        """Show aircraft that perform no resolution manoeuvres."""
        aircraft = ", ".join(np.array(self.traffic.callsign)[self.resooffac])
        return Ok(
            "RESOOFF [ACID, ... ] OR RESOOFF [GROUPID]"
            f"\nCurrent list of aircraft will not avoid anybody:{aircraft}"
        )

    @command(name="RESOOFF")
    def toggle_resooff(self, first: AcIdSelection, *additional: AcIdSelection) -> Result[str, str]:
        """Toggle resolution manoeuvres for selected aircraft."""
        indices = aircraft_indices((first, *additional))
        self.resooffac[indices] = np.logical_not(self.resooffac[indices])
        return Ok("")

    @command(name="RFACH", aliases=("RESOFACH", "HRFAC", "HRESOFAC"))
    def horizontal_resolution_factor(self) -> Result[str, str]:
        """Show the horizontal resolution factor."""
        return Ok(f"RFACH [FACTOR]\nCurrent horizontal resolution factor is: {self.resofach}")

    @command(name="RFACH")
    def set_horizontal_resolution_factor(self, factor: PositiveFiniteFloat) -> Result[str, str]:
        """Set the horizontal resolution factor."""
        self.resofach = factor
        self.resorrelative = True  # Size of resolution zone r, vertically, set relative to CD zone
        return Ok(f"Horizontal resolution factor set to {self.resofach}")

    @command(name="RFACV", aliases=("RESOFACV",))
    def vertical_resolution_factor(self) -> Result[str, str]:
        """Show the vertical resolution factor."""
        return Ok(f"RFACV [FACTOR]\nCurrent vertical resolution factor is: {self.resofacv}")

    @command(name="RFACV")
    def set_vertical_resolution_factor(self, factor: PositiveFiniteFloat) -> Result[str, str]:
        """Set the vertical resolution factor."""
        self.resofacv = factor
        # Size of resolution zone dh, vertically, set relative to CD zone
        self.resodhrelative = True
        return Ok(f"Vertical resolution factor set to {self.resofacv}")

    def _horizontal_absolute_zone_available(self) -> Result[None, str]:
        if self.traffic.cd.global_rpz:
            return Ok(None)
        self.resorrelative = True
        return Err(
            "RSZONER [radius], e.g. RSZONER 7.5NM\nCan only set resolution factor when simulation contains aircraft with different RPZ,\nUse RFACH instead."
        )

    @command(name="RSZONER", aliases=("RESOZONER",))
    def horizontal_resolution_zone(self) -> Result[str, str]:
        """Show the absolute horizontal resolution zone."""
        if isinstance(available := self._horizontal_absolute_zone_available(), Err):
            return available
        return Ok(
            f"RSZONER [radius], e.g. RSZONER 7.5NM\nCurrent horizontal resolution factor is: {self.resofach}, resulting in radius: {q.m_to_nmi(self.resofach * self.traffic.cd.rpz_def)} nm"
        )

    @command(name="RSZONER")
    def set_horizontal_resolution_zone(self, zoner: ResolutionRadiusM) -> Result[str, str]:
        """Set the absolute horizontal resolution-zone radius."""
        if isinstance(available := self._horizontal_absolute_zone_available(), Err):
            return available
        self.resofach = zoner / self.traffic.cd.rpz_def
        # Size of resolution zone r, vertically, no longer relative to CD zone
        self.resorrelative = False
        return Ok(
            f"Horizontal resolution factor updated to {self.resofach}, resulting in radius: {q.m_to_nmi(zoner)} nm"
        )

    def _vertical_absolute_zone_available(self) -> Result[None, str]:
        if self.traffic.cd.global_hpz:
            return Ok(None)
        self.resodhrelative = True
        return Err(
            "RSZONEH [height], e.g. RSZONEH 1500FT\nCan only set resolution factor when simulation contains aircraft with different HPZ,\nUse RFACV instead."
        )

    @command(name="RSZONEDH", aliases=("RESOZONEDH",))
    def vertical_resolution_zone(self) -> Result[str, str]:
        """Show the absolute vertical resolution zone."""
        if isinstance(available := self._vertical_absolute_zone_available(), Err):
            return available
        return Ok(
            f"RSZONEDH [height], e.g. RSZONEDH 1500FT\nCurrent vertical resolution factor is: {self.resofacv}, resulting in height: {q.m_to_ft(self.resofacv * self.traffic.cd.hpz_def)} ft"
        )

    @command(name="RSZONEDH")
    def set_vertical_resolution_zone(self, zonedh: ResolutionHeightM) -> Result[str, str]:
        """Set the absolute vertical resolution-zone height."""
        if isinstance(available := self._vertical_absolute_zone_available(), Err):
            return available
        self.resofacv = zonedh / self.traffic.cd.hpz_def
        # Size of resolution zone dh, vertically, no longer relative to CD zone
        self.resodhrelative = False
        return Ok(
            f"Vertical resolution factor updated to {self.resofacv}, resulting in height: {q.m_to_ft(zonedh)} ft"
        )

    @command(name="RESO")
    def resolution_method(self) -> Result[str, str]:
        """Show the current and available conflict-resolution methods."""
        curname = type(self.traffic.cr).__name__ if self.traffic.cr.activate else "OFF"
        return Ok(f"Current CR method: {curname}\nAvailable CR methods: OFF, MVP")

    @command(name="RESO")
    def disable_resolution(self, _method: Literal["OFF"]) -> Result[str, str]:
        """Disable conflict resolution."""
        self.traffic.cr.switch(False)
        return Ok("Conflict Resolution turned off.")

    @command(name="RESO")
    def enable_mvp_resolution(self, _method: Literal["MVP"]) -> Result[str, str]:
        """Select and enable MVP conflict resolution."""
        match self.select_implementation("CONFLICTRESOLUTION", "MVP"):
            case Err() as error:
                return error
            case Ok():
                self.traffic.cr.switch(True)
                return Ok("Selected MVP as Conflict Resolution method.")

    def horizontal_method_status(self) -> Result[str, str]:
        return Err(f"RMETHH is not available for CR method {type(self).__name__}")

    def configure_horizontal_method(self, value: HorizontalResolutionMethod) -> Result[str, str]:
        return Err(f"RMETHH is not available for CR method {type(self).__name__}")

    @command(name="RMETHH")
    def show_horizontal_method(self) -> Result[str, str]:
        """Show horizontal resolution-method settings."""
        return self.horizontal_method_status()

    @command(name="RMETHH")
    def set_horizontal_method(self, value: HorizontalResolutionMethod) -> Result[str, str]:
        """Configure horizontal resolution degrees of freedom."""
        return self.configure_horizontal_method(value)

    def vertical_method_status(self) -> Result[str, str]:
        return Err(f"RMETHV is not available for CR method {type(self).__name__}")

    def configure_vertical_method(self, value: VerticalResolutionMethod) -> Result[str, str]:
        return Err(f"RMETHV is not available for CR method {type(self).__name__}")

    @command(name="RMETHV")
    def show_vertical_method(self) -> Result[str, str]:
        """Show vertical resolution-method settings."""
        return self.vertical_method_status()

    @command(name="RMETHV")
    def set_vertical_method(self, value: VerticalResolutionMethod) -> Result[str, str]:
        """Configure vertical resolution degrees of freedom."""
        return self.configure_vertical_method(value)
