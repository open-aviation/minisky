"""Conflict resolution base class.

This module provides :class:`ConflictResolution`, the base class for all
conflict resolution (CR) implementations in MiniSky. It manages the shared
resolution machinery: per-aircraft resolution advisories (heading, speed,
vertical speed, altitude), resolution zone margins relative to the detection
protected zone, priority rules, per-aircraft opt-outs (NORESO/RESOOFF), and
the logic that decides when an aircraft may resume normal navigation after a
conflict has been resolved (:meth:`ConflictResolution.resumenav`).

Actual resolution algorithms (e.g. the Modified Voltage Potential method in
``minisky.traffic.asas.mvp``) subclass this class and override
:meth:`ConflictResolution.resolve`.
"""

from typing import Any

import numpy as np

import minisky
from minisky.core.trafficarrays import TrafficArrays
from minisky.stack.argparser import Txt
from minisky.tools.aero import ft, nm


class ConflictResolution(TrafficArrays):
    """Base class for Conflict Resolution implementations.

    Each update step, when resolution is active and conflicts are detected,
    :meth:`resolve` is called to compute resolution advisories for all
    aircraft. These advisories are stored in the per-aircraft arrays below and
    are followed by the autopilot for aircraft whose ``active`` flag is True.
    :meth:`resumenav` then decides per aircraft whether to keep following the
    resolution or to resume the flight plan (after the conflict pair has
    passed its closest point of approach).

    The base class itself performs no avoidance: its :meth:`resolve` simply
    returns the autopilot values. Subclasses implement an actual algorithm.

    Attributes:
        activate (bool): Whether conflict resolution is switched on.
        swprio (bool): Whether priority (right-of-way) rules are applied.
        priocode (str): Selected priority rule set (e.g. "FF1".."FF3",
            "LAY1", "LAY2").
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
        tas (ndarray): Resolution speed advisory [m/s].
        alt (ndarray): Resolution altitude advisory [m].
        vs (ndarray): Resolution vertical speed advisory [m/s].
    """

    def __init__(self) -> None:
        super().__init__()
        self.activate = False

        # [-] switch to activate priority rules for conflict resolution
        self.swprio = False  # switch priority on/off
        self.priocode = ""  # select priority mode
        self.resopairs = set()  # Resolved conflicts that are still before CPA

        # Resolution factors:
        # set < 1 to maneuver only a fraction of the resolution
        # set > 1 to add a margin to separation values
        self.resofach = minisky.core.settings.asas_marh
        self.resofacv = minisky.core.settings.asas_marv

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
            self.trk = np.array([])  # heading provided by the ASAS [deg]
            self.tas = np.array([])  # speed provided by the ASAS (eas) [m/s]
            self.alt = np.array([])  # alt provided by the ASAS [m]
            self.vs = np.array([])  # vspeed provided by the ASAS [m/s]

    def switch(self, flag: bool | None = None) -> None:
        """Turn conflict resolution on or off.

        Args:
            flag (bool): True to activate resolution, False to deactivate.
        """
        self.activate = flag

    def reset(self) -> None:
        """Reset the conflict resolution state to defaults.

        Called on simulation reset: clears priority settings and pending
        resolution pairs, and restores the resolution zone factors from the
        simulation settings.
        """
        super().reset()
        self.swprio = False
        self.priocode = ""
        self.resopairs.clear()
        self.resofach = minisky.core.settings.asas_marh
        self.resofacv = minisky.core.settings.asas_marv
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
    def tasactive(self) -> np.ndarray:
        """Return a boolean array sized according to the number of aircraft
        with True for all elements where speed is currently controlled by
        the conflict resolution algorithm.
        """
        return self.active

    def resolve(self, conf: Any, ownship: Any, intruder: Any) -> tuple:
        """Resolve all current conflicts.

        This function should be reimplemented in a subclass for actual
        resolution of conflicts. See for instance minisky.traffic.asas.mvp.
        The base implementation returns the autopilot values, i.e. no
        avoidance manoeuvre.

        Args:
            conf: The ConflictDetection instance with the current conflicts.
            ownship: Traffic object with ownship states.
            intruder: Traffic object with intruder states.

        Returns:
            tuple: Per-aircraft advisories (newtrk [deg], newtas [m/s],
                newvs [m/s], newalt [m]).
        """
        # If resolution is off, and detection is on, and a conflict is detected
        # then asas will be active for that airplane. Since resolution is off, it
        # should then follow the auto pilot instructions.
        return ownship.ap.trk, ownship.ap.tas, ownship.ap.vs, ownship.ap.alt

    def update(self, conf: Any, ownship: Any, intruder: Any) -> None:
        """Perform an update step of the Conflict Resolution implementation.

        When resolution is active, computes new resolution advisories with
        :meth:`resolve` if there are current conflicts, and updates which
        aircraft should keep following the resolution with :meth:`resumenav`.

        Args:
            conf: The ConflictDetection instance with the current conflicts.
            ownship: Traffic object with ownship states.
            intruder: Traffic object with intruder states.
        """
        if self.activate:
            if conf.confpairs:
                self.trk, self.tas, self.vs, self.alt = self.resolve(conf, ownship, intruder)
            self.resumenav(conf, ownship, intruder)

    def resumenav(self, conf: Any, ownship: Any, intruder: Any) -> None:
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
            conf: The ConflictDetection instance with the current conflicts.
            ownship: Traffic object with ownship states.
            intruder: Traffic object with intruder states.
        """
        # Add new conflicts to resopairs and confpairs_all and new losses to lospairs_all
        self.resopairs.update(conf.confpairs)

        # Conflict pairs to be deleted
        delpairs = set()
        changeactive = {}

        # smallest relative angle between vectors of heading a and b
        def anglediff(a: float, b: float) -> float:
            d = a - b
            if d > 180:
                return anglediff(a, b + 360)
            elif d < -180:
                return anglediff(a + 360, b)
            else:
                return d

        # Look at all conflicts, also the ones that are solved but CPA is yet to come
        for conflict in self.resopairs:
            idx1, idx2 = minisky.traf.idx(conflict)
            # If the ownship aircraft is deleted remove its conflict from the list
            if idx1 < 0:
                delpairs.add(conflict)
                continue

            if idx2 >= 0:
                # Distance vector using flat earth approximation
                re = 6371000.0
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
            if idx2 >= 0 and (not past_cpa or hor_los or is_bouncing):
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
                iwpid = minisky.traf.ap.route[idx].findact(idx)
                if iwpid != -1:  # To avoid problems if there are no waypoints
                    minisky.traffic.route.direct(idx, minisky.traf.ap.route[idx].wpname[iwpid])

        # Remove pairs from the list that are past CPA or have deleted aircraft
        self.resopairs -= delpairs

    def setprio(self, flag: bool | None = None, priocode="") -> "bool | tuple":
        """Define priority rules (right of way) for conflict resolution.

        Implements the PRIORULES stack command. The base class only stores
        the settings; interpretation of the priority code is up to the
        resolution algorithm (see e.g. ``MVP.applyprio``).

        Args:
            flag (bool): True to enable priority rules, False to disable.
                When None, an informational message is returned.
            priocode (str): Identifier of the priority rule set to use.

        Returns:
            True on success, or (False, message) when not applicable.
        """
        if flag is None:
            if self.__class__ is ConflictResolution:
                return False, "No conflict resolution enabled."
            return (
                False,
                f"Resolution algorithm {self.__class__.__name__} hasn't implemented priority.",
            )

        self.swprio = flag
        self.priocode = priocode
        return True

    def setnoreso(self, *idx: int) -> "bool | tuple":
        """ADD or Remove aircraft that nobody will avoid.
        Multiple aircraft can be sent to this function at once.

        Implements the NORESO stack command: toggles the ``noresoac`` flag
        for the given aircraft. Flagged aircraft still avoid others, but
        other aircraft will not avoid them.

        Args:
            *idx: Aircraft indices to toggle. When empty, the current list of
                flagged aircraft is reported.

        Returns:
            True on success, or (True, message) when reporting.
        """
        if not idx:
            return (
                True,
                "NORESO [ACID, ... ] OR NORESO [GROUPID]"
                + "\nCurrent list of aircraft nobody will avoid:"
                + ", ".join(np.array(minisky.traf.callsign)[self.noresoac]),
            )
        indices = list(idx)
        self.noresoac[indices] = np.logical_not(self.noresoac[indices])
        return True

    def setresooff(self, *idx: int) -> "bool | tuple":
        """ADD or Remove aircraft that will not avoid anybody else.
        Multiple aircraft can be sent to this function at once.

        Implements the RESOOFF stack command: toggles the ``resooffac`` flag
        for the given aircraft. Flagged aircraft perform no resolution
        manoeuvres themselves, but others may still avoid them.

        Args:
            *idx: Aircraft indices to toggle. When empty, the current list of
                flagged aircraft is reported.

        Returns:
            True on success, or (True, message) when reporting.
        """
        if not idx:
            return (
                True,
                "RESOOFF [ACID, ... ] OR RESOOFF [GROUPID]"
                + "\nCurrent list of aircraft will not avoid anybody:"
                + ", ".join(np.array(minisky.traf.callsign)[self.resooffac]),
            )
        else:
            indices = list(idx)
            self.resooffac[indices] = np.logical_not(self.resooffac[indices])
            return True

    def setresofach(self, factor: float | None = None) -> tuple:
        """Set resolution factor horizontal
        (to maneuver only a fraction of a resolution vector).

        Implements the RFACH stack command. The horizontal resolution zone
        radius is ``resofach`` times the detection protected zone radius:
        values below 1 manoeuvre only a fraction of the resolution, values
        above 1 add a separation margin.

        Args:
            factor (float): Horizontal resolution factor [-]. When None, the
                current factor is reported.

        Returns:
            tuple: (success (bool), message (str)) for the command stack.
        """
        if factor is None:
            return (
                True,
                f"RFACH [FACTOR]\nCurrent horizontal resolution factor is: {self.resofach}",
            )
        else:
            self.resofach = factor
            self.resorrelative = (
                True  # Size of resolution zone r, vertically, set relative to CD zone
            )
            return True, f"Horizontal resolution factor set to {self.resofach}"

    def setresofacv(self, factor: float | None = None) -> tuple:
        """Set resolution factor vertical (to maneuver only a fraction of a resolution vector).

        Implements the RFACV stack command. The vertical resolution zone
        height is ``resofacv`` times the detection protected zone height.

        Args:
            factor (float): Vertical resolution factor [-]. When None, the
                current factor is reported.

        Returns:
            tuple: (success (bool), message (str)) for the command stack.
        """
        if factor is None:
            return (
                True,
                f"RFACV [FACTOR]\nCurrent vertical resolution factor is: {self.resofacv}",
            )
        self.resofacv = factor
        # Size of resolution zone dh, vertically, set relative to CD zone
        self.resodhrelative = True
        return True, f"Vertical resolution factor set to {self.resofacv}"

    def setresozoner(self, zoner: float | None = None) -> tuple:
        """Set resolution factor horizontal, but then with absolute value
        (to maneuver only a fraction of a resolution vector).

        Implements the RSZONER stack command: sets the horizontal resolution
        zone as an absolute radius, from which ``resofach`` is derived. Only
        available when all aircraft share the same (global) protected zone
        radius.

        Args:
            zoner (float): Resolution zone radius [NM]. When None, the current
                factor and resulting radius are reported.

        Returns:
            tuple: (success (bool), message (str)) for the command stack.
        """
        if not minisky.traf.cd.global_rpz:
            self.resorrelative = True
            return (
                False,
                "RSZONER [radiusnm]\nCan only set resolution factor when simulation contains aircraft with different RPZ,\nUse RFACH instead.",
            )
        if zoner is None:
            return (
                True,
                f"RSZONER [radiusnm]\nCurrent horizontal resolution factor is: {self.resofach}, resulting in radius: {self.resofach * minisky.traf.cd.rpz_def / nm} nm",
            )

        self.resofach = zoner / minisky.traf.cd.rpz_def * nm
        # Size of resolution zone r, vertically, no longer relative to CD zone
        self.resorrelative = False
        return (
            True,
            f"Horizontal resolution factor updated to {self.resofach}, resulting in radius: {zoner} nm",
        )

    def setresozonedh(self, zonedh: float | None = None) -> tuple:
        """Set resolution factor vertical (to maneuver only a fraction of a
        resolution vector), but then with absolute value.

        Implements the RSZONEDH stack command: sets the vertical resolution
        zone as an absolute height, from which ``resofacv`` is derived. Only
        available when all aircraft share the same (global) protected zone
        height.

        Args:
            zonedh (float): Resolution zone height [ft]. When None, the
                current factor and resulting height are reported.

        Returns:
            tuple: (success (bool), message (str)) for the command stack.
        """
        if not minisky.traf.cd.global_hpz:
            self.resodhrelative = True
            return (
                False,
                "RSZONEH [zonedhft]\nCan only set resolution factor when simulation contains aircraft with different HPZ,\nUse RFACV instead.",
            )
        if zonedh is None:
            return (
                True,
                f"RSZONEDH [zonedhft]\nCurrent vertical resolution factor is: {self.resofacv}, resulting in height: {self.resofacv * minisky.traf.cd.hpz_def / ft} ft",
            )

        self.resofacv = zonedh / minisky.traf.cd.hpz_def * ft
        # Size of resolution zone dh, vertically, no longer relative to CD zone
        self.resodhrelative = False
        return (
            True,
            f"Vertical resolution factor updated to {self.resofacv}, resulting in height: {zonedh} ft",
        )

    @staticmethod
    def setmethod(name: Txt = "") -> tuple:
        """Select a Conflict Resolution method.

        Implements the RESO stack command. Selecting "MVP" replaces the
        traffic object's resolution instance (``minisky.traf.cr``) with a new
        MVP instance and activates it.

        Args:
            name (str): "OFF", "MVP", or empty to report available methods.

        Returns:
            tuple: (success (bool), message (str)) for the command stack.
        """

        names = ["OFF", "MVP"]

        if not name:
            curname = type(minisky.traf.cr).__name__ if minisky.traf.cr.activate else "OFF"
            return (
                True,
                f"Current CR method: {curname}" + f"\nAvailable CR methods: {', '.join(names)}",
            )

        if name == "OFF":
            minisky.traf.cr.switch(False)
            return True, "Conflict Resolution turned off."

        if name == "MVP":
            from minisky.traffic.asas.mvp import MVP

            # Replace the current conflict resolution instance with MVP
            minisky.traf.cr = MVP()
            minisky.traf.cr.switch(True)
            return True, "Selected MVP as Conflict Resolution method."

        return False, f"Unknown method: {name}. Available: {', '.join(names)}"


# Module-level dispatchers for the stack commands below. RESO can replace
# minisky.traf.cr with a new instance (e.g. MVP), which would leave commands
# registered as bound methods pointing at the stale, replaced object. These
# wrappers resolve the current instance at call time instead.


def setprio(flag: bool | None = None, priocode="") -> "bool | tuple":
    """PRIORULES stack command; see ConflictResolution.setprio()."""
    return minisky.traf.cr.setprio(flag, priocode)


def setnoreso(*idx: int) -> "bool | tuple":
    """NORESO stack command; see ConflictResolution.setnoreso()."""
    return minisky.traf.cr.setnoreso(*idx)


def setresooff(*idx: int) -> "bool | tuple":
    """RESOOFF stack command; see ConflictResolution.setresooff()."""
    return minisky.traf.cr.setresooff(*idx)


def setresofach(factor: float | None = None) -> tuple:
    """RFACH stack command; see ConflictResolution.setresofach()."""
    return minisky.traf.cr.setresofach(factor)


def setresofacv(factor: float | None = None) -> tuple:
    """RFACV stack command; see ConflictResolution.setresofacv()."""
    return minisky.traf.cr.setresofacv(factor)


def setresozoner(zoner: float | None = None) -> tuple:
    """RSZONER stack command; see ConflictResolution.setresozoner()."""
    return minisky.traf.cr.setresozoner(zoner)


def setresozonedh(zonedh: float | None = None) -> tuple:
    """RSZONEDH stack command; see ConflictResolution.setresozonedh()."""
    return minisky.traf.cr.setresozonedh(zonedh)


def setresometh(value: Txt = "") -> tuple:
    """RMETHH stack command; see MVP.setresometh()."""
    from minisky.traffic.asas.mvp import MVP

    cr = minisky.traf.cr
    if not isinstance(cr, MVP):
        return False, f"RMETHH is not available for CR method {type(cr).__name__}"
    return cr.setresometh(value)


def setresometv(value: Txt = "") -> tuple:
    """RMETHV stack command; see MVP.setresometv()."""
    from minisky.traffic.asas.mvp import MVP

    cr = minisky.traf.cr
    if not isinstance(cr, MVP):
        return False, f"RMETHV is not available for CR method {type(cr).__name__}"
    return cr.setresometv(value)
