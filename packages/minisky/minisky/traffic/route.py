"""Route implementation for the BlueSky FMS.

Contains the per-aircraft [`Route`][minisky.traffic.route.Route] class (the flight plan: an ordered
list of waypoints with optional altitude, speed, RTA and turn constraints)
plus the route-editing functions used by [`RouteCommands`][minisky.traffic.route.RouteCommands],
the runtime-owned command component for ADDWPT, ADDWPTMODE, AFTER, BEFORE,
AT, DIRECT, RTA, LISTRTE, DELRTE and DELWPT.

The route itself is passive data with flight-plan pre-calculations
(calcfp()); the actual guidance along the route is performed by
[`Autopilot`][minisky.traffic.autopilot.Autopilot], which pulls waypoint data
into the vectorized [`ActiveWaypoint`][minisky.traffic.activewpdata.ActiveWaypoint]
arrays via getnextwp()/getnextturnwp().
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING, Annotated, Literal, NamedTuple, TypeAlias

import numpy as np

from minisky.command import (
    AcId,
    AltM,
    ArgumentIssue,
    CmdParser,
    CommandParseContext,
    CoordinateWaypoint,
    Keyword,
    LatLonDegrees,
    NamedWaypoint,
    Omitted,
    Parsed,
    ParseResult,
    PositiveFiniteFloat,
    RunwayPosition,
    SpeedMpsOrMach,
    Text,
    TimeS,
    Wpt,
    command,
    next_argument,
    parse_altitude_value,
    parse_keyword,
    parse_resolved_position,
    parse_speed_value,
)
from minisky.result import Err, Ok, Result

# from minisky.core import Replaceable
from minisky.tools import geo
from minisky.tools.aero import casormach2tas, ft, g0, kts, mach2cas, nm
from minisky.tools.convert import degto180
from minisky.tools.position import txt2pos

if TYPE_CHECKING:
    from minisky.traffic import Traffic


class TurnRadius(NamedTuple):
    radius: float
    """Turn radius [m]."""


class TurnHeadingRate(NamedTuple):
    heading_rate: float
    """Turn heading rate [deg/s]."""


TurnGeometry: TypeAlias = TurnRadius | TurnHeadingRate


class TurnParameters(NamedTuple):
    geometry: TurnGeometry | None = None
    speed: float | None = None


class NextTurn(NamedTuple):
    latitude: float
    longitude: float
    turn: TurnParameters
    waypoint_index: int


class AltitudeTarget(NamedTuple):
    altitude: float
    """Target altitude [m]."""
    distance: float
    """Distance from the active waypoint to the constraint [m]."""


class RtaTarget(NamedTuple):
    time: float
    """Required arrival time in simulation time [s]."""
    distance: float
    """Distance from the active waypoint to the RTA waypoint [m]."""


class RouteProfile(NamedTuple):
    altitude: AltitudeTarget | None = None
    rta: RtaTarget | None = None


class Route:
    """Flight plan (route) of a single aircraft: basic FMS functionality.

    A Route is an ordered list of waypoints, each with an optional altitude
    constraint, speed constraint, required time of arrival (RTA), turn
    specification (fly-by/fly-over/fly-turn with radius, speed or heading
    rate) and stack commands to execute when the waypoint is passed. One
    Route object is kept per aircraft in [`runtime.traffic.ap.route`][minisky.traffic.route.Route].

    Waypoints from the navigation database are resolved to the entry
    closest to the given lat/lon. For plain lat/lon waypoints the aircraft
    callsign is used as waypoint name, with a number appended.

    Attributes:
        acid (str): Callsign of the aircraft this route belongs to.
        wpname (list): Waypoint names.
        wptype (list): Waypoint types (wplatlon, wpnav, orig, dest,
            calcwp, runway).
        wplat (list): Waypoint latitudes [deg].
        wplon (list): Waypoint longitudes [deg].
        wpalt (list): Optional altitude constraints [m].
        wpspd (list): Optional speed constraints, CAS [m/s] or Mach [-].
        wprta (list): Optional required times of arrival [s].
        wpflyby (list): Fly-by (True) / fly-over (False) switch.
        wpturn (list): Optional fly-turn parameters per waypoint.
        wpstack (list): Stack command lines executed when passing each
            waypoint (AT ... DO).
        iactwp (int): Index of the currently active waypoint (-1 = none).
        swflyby (bool): Default fly-by mode for newly added waypoints.
        swflyturn (bool): Default fly-turn mode for newly added waypoints.
        bank (float): Default bank angle for turn calculations [deg].
        flag_landed_runway (bool): True after touchdown on a runway; the
            aircraft then keeps the runway heading.
        wpdirfrom (list): Direction of the leg leaving each waypoint [deg].
        wpdirto (list): Direction of the leg to each waypoint [deg].
        wpdistto (list): Length of the leg to each waypoint [nm].
        wpprofile (list): Optional altitude and RTA guidance targets from each waypoint.

    Created by: Jacco M. Hoekstra
    """

    # Waypoint types:
    wplatlon = 0  # lat/lon waypoint
    wpnav = 1  # VOR/nav database waypoint
    orig = 2  # Origin airport
    dest = 3  # Destination airport
    calcwp = 4  # Calculated waypoint (T/C, T/D, A/C)
    runway = 5  # Runway: Copy name and positions

    # # Aircraft route objects
    # _routes: WeakValueDictionary[str, "Route"] = WeakValueDictionary()

    def __init__(self, traffic: Traffic, acid: str) -> None:
        self.traffic = traffic
        self.navigation = traffic.navigation
        self.acid = acid

        # Waypoint data
        self.wpname = []  # List of waypoint names for this flight plan
        self.wptype = []  # List of waypoint types
        self.wplat = []  # List of waypoint latitudes
        self.wplon = []  # List of waypoint longitudes
        self.wpalt: list[float | None] = []  # [m]
        self.wpspd: list[float | None] = []  # [m/s] CAS or Mach
        self.wprta: list[float | None] = []  # [s]
        self.wpflyby = []  # Flyby (True)/flyover(False) switch
        self.wpstack = []  # Stack with command execured when passing this waypoint

        # Made for drones: fly turn mode, means use specified turn radius and optionally turn speed
        self.wpturn: list[TurnParameters | None] = []

        # Current actual waypoint
        self.iactwp = -1

        # FIXME(abraham): BlueSky 01ea1f9 (2020-04-03) introduced this two-boolean mode state;
        # swflyby and swflyturn can both be true even though only three modes are valid.
        self.swflyby = True  # Default waypoints are flyby waypoint
        self.swflyturn = False  # Default waypoints are waypoints w/o specified turn

        self.bank = 25.0  # [deg] Default bank angle
        self.turn = TurnParameters()

        # if the aircraft lands on a runway, the aircraft should keep the
        # runway heading
        # default: False
        self.flag_landed_runway = False

        self.wpdirfrom = []  # [deg] direction leg to wp
        self.wpdirto = []  # [deg] direction leg from wp
        # TODO(abraham): migrate this to SI!
        self.wpdistto = []  # [nm] leg length to wp
        self.wpprofile: list[RouteProfile] = []

    def insert_wpt_data(
        self,
        wpidx: int,
        wpname: str,
        wplat: float,
        wplon: float,
        wptype: int,
        wpalt: float | None,
        wpspd: float | None,
    ) -> None:
        """Insert a new waypoint record at a given index in the route.

        All per-waypoint lists are updated consistently; the current default
        fly-by/fly-turn mode and turn parameters of the route are applied to
        the new waypoint, and no RTA is set.

        Args:
            wpidx: List index at which to insert the waypoint.
            wpname: Waypoint name.
            wplat: Waypoint latitude [deg].
            wplon: Waypoint longitude [deg].
            wptype: Waypoint type (see the Route class constants).
            wpalt: Optional altitude constraint [m].
            wpspd: Optional speed constraint, CAS [m/s] or Mach [-].
        """

        self.wpname.insert(wpidx, wpname)
        self.wplat.insert(wpidx, wplat)
        self.wplon.insert(wpidx, wplon)
        self.wpalt.insert(wpidx, wpalt)
        self.wpspd.insert(wpidx, wpspd)
        self.wptype.insert(wpidx, wptype)
        self.wpflyby.insert(wpidx, self.swflyby)
        self.wpturn.insert(wpidx, self.turn if self.swflyturn else None)
        self.wprta.insert(wpidx, None)
        self.wpstack.insert(wpidx, [])

    def add_waypoint(
        self,
        iac: int,
        name: str,
        wptype: int,
        lat: float,
        lon: float,
        alt: float | None = None,
        spd: float | None = None,
        afterwp: str = "",
        beforewp: str = "",
    ) -> int:
        """Add a waypoint to the route and update the flight plan.

        Handles all waypoint types: origin/destination airports (placed at
        the start/end of the route, overwriting an existing orig/dest),
        navigation-database waypoints (resolved closest to the given
        position), runways and plain lat/lon waypoints. The insertion point
        can be steered with afterwp/beforewp; by default waypoints are
        appended just before the destination. Afterwards the flight-plan
        tables are recalculated (calcfp()) and, when a waypoint is active,
        the guidance towards it is refreshed.

        Args:
            iac: Aircraft index.
            name: Waypoint name (or callsign for lat/lon waypoints).
            wptype: Waypoint type (see the Route class constants).
            lat: Waypoint latitude [deg].
            lon: Waypoint longitude [deg].
            alt: Optional altitude constraint [m].
            spd: Optional speed constraint, CAS [m/s] or Mach [-].
            afterwp: Optional name of the waypoint after which to insert.
            beforewp: Optional name of the waypoint before which to insert.

        Returns:
            int: Index of the added waypoint in the route, or -1 on failure.
        """

        # For safety
        n_wpt = len(self.wplat)

        name = name.upper().strip()
        spd = None if spd is None or spd <= 0.0 else spd

        wplat = (lat + 90.0) % 180.0 - 90.0
        wplon = (lon + 180.0) % 360.0 - 180.0

        wpok = True  # switch for waypoint check

        # Check if name already exists, if so add integer 01, 02, 03 etc.
        wprtename = get_available_name(self.wpname, name, self.traffic.callsign)
        # Select on wptype
        # ORIGIN: Wptype is origin/destination?
        if wptype == Route.orig or wptype == Route.dest:
            orig = wptype == Route.orig
            wpidx = 0 if orig else -1
            suffix = "ORIG" if orig else "DEST"

            if name != self.traffic.callsign[iac] + suffix:  # published identifier
                i = self.navigation.getaptidx(name)
                if i is not None:
                    wplat = self.navigation.aptlat[i]
                    wplon = self.navigation.aptlon[i]

            if not orig and alt is None:
                alt = 0.0

            # Overwrite existing origin/dest
            if n_wpt > 0 and self.wptype[wpidx] == wptype:
                self.wpname[wpidx] = wprtename
                self.wplat[wpidx] = wplat
                self.wplon[wpidx] = wplon
                self.wpalt[wpidx] = alt
                self.wpspd[wpidx] = spd
                self.wptype[wpidx] = wptype
                # also apply other current settings
                self.wpflyby[wpidx] = self.swflyby
                self.wpturn[wpidx] = self.turn if self.swflyturn else None
                self.wprta[wpidx] = None
                self.wpstack[wpidx] = []

            # Or add before first waypoint/append to end
            else:
                if not orig:
                    wpidx = len(self.wplat)

                self.insert_wpt_data(wpidx, wprtename, wplat, wplon, wptype, alt, spd)

                n_wpt += 1
                if orig and self.iactwp >= 0:
                    self.iactwp += 1
                elif not orig and self.iactwp < 0 and n_wpt == 1:
                    # When only waypoint: adjust pointer to point to destination
                    self.iactwp = 0

            idx = 0 if orig else n_wpt - 1

        # NORMAL: Wptype is normal waypoint? (lat/lon or nav)
        else:
            # Lat/lon: wpname is then call sign of aircraft: add number
            if wptype == Route.wplatlon:
                newname = get_available_name(self.wpname, name, self.traffic.callsign, 3)

            # Else make data complete with nav database and closest to given lat,lon
            else:  # so wptypewpnav
                newname = wprtename

                if wptype != Route.runway:
                    i = self.navigation.getwpidx(name, LatLonDegrees(lat, lon))
                    wpok = i is not None

                    if i is not None:
                        wplat = self.navigation.wplat[i]
                        wplon = self.navigation.wplon[i]
                    else:
                        i = self.navigation.getaptidx(name)
                        wpok = i is not None
                        if i is not None:
                            wplat = self.navigation.aptlat[i]
                            wplon = self.navigation.aptlon[i]

            # Check if afterwp or beforewp is specified and found:
            aftwp = afterwp.upper().strip()  # Remove space, upper case
            bfwp = beforewp.upper().strip()

            if wpok:
                if (afterwp and self.wpname.count(aftwp) > 0) or (
                    beforewp and self.wpname.count(bfwp) > 0
                ):
                    wpidx = self.wpname.index(aftwp) + 1 if afterwp else self.wpname.index(bfwp)

                    self.insert_wpt_data(wpidx, newname, wplat, wplon, wptype, alt, spd)

                    if afterwp and self.iactwp >= wpidx:
                        self.iactwp += 1

                # No afterwp: append, just before dest if there is a dest
                else:
                    # Is there a destination?
                    wpidx = n_wpt - 1 if n_wpt > 0 and self.wptype[-1] == Route.dest else n_wpt

                    self.insert_wpt_data(wpidx, newname, wplat, wplon, wptype, alt, spd)

                idx = wpidx
                n_wpt += 1

            else:
                idx = -1
                if len(self.wplat) == 1:
                    self.iactwp = 0

        # update qdr and "last waypoint switch" in traffic
        if idx >= 0:
            next_qdr = self.getnextqdr()
            self.traffic.actwp.next_qdr[iac] = np.ma.masked if next_qdr is None else next_qdr
            self.traffic.actwp.swlastwp[iac] = self.iactwp == n_wpt - 1

        # Update waypoints
        if wptype != Route.calcwp:
            self.calcfp()

        # Update autopilot settings
        if wpok and 0 <= self.iactwp < n_wpt:
            direct(self.traffic, iac, self.wpname[self.iactwp])

        return idx

    def getnextturnwp(self) -> NextTurn | None:
        """Return the next fly-turn waypoint at or after the active waypoint."""
        for waypoint_index in range(max(self.iactwp, 0), len(self.wpturn)):
            turn = self.wpturn[waypoint_index]
            if turn is not None:
                return NextTurn(
                    self.wplat[waypoint_index],
                    self.wplon[waypoint_index],
                    turn,
                    waypoint_index,
                )
        return None

    class WaypointTransition(NamedTuple):
        """Guidance state produced when a route activates its next waypoint."""

        position: LatLonDegrees
        """Active waypoint position [deg]."""
        speed: float | None
        """Optional speed constraint, calibrated airspeed [m/s] or Mach number [-]."""
        profile: RouteProfile
        """Altitude and RTA guidance targets ahead of the active waypoint."""
        lnav_enabled: bool
        """Whether lateral navigation remains enabled."""
        fly_by: bool
        """Whether the waypoint uses fly-by switching."""
        turn: TurnParameters | None
        """Fly-turn parameters, or None when this is not a fly-turn waypoint."""
        next_leg: LatLonDegrees | None
        """Endpoint of the leg after the active waypoint, or None when there is no next leg."""
        last_waypoint: bool
        """Whether this is the final waypoint."""

    def getnextwp(self) -> WaypointTransition:
        """Activate the next waypoint in the route and return its guidance state.

        Called by the autopilot when the active waypoint has been passed.
        Advances iactwp (unless the last waypoint was reached, in which case
        the returned LNAV switch is False). When the new active waypoint is
        a runway used for landing, a fixed runway heading is commanded and
        deceleration plus deletion of the aircraft are scheduled via the
        stack.
        """
        n_wpt = len(self.wpname)

        if self.flag_landed_runway:
            # when landing, LNAV is switched off
            lnavon = False

            # no further waypoint; the aircraft only needs a fixed runway heading
            name = self.wpname[self.iactwp]

            # Change RW06,RWY18C,RWY24001 to resp. 06,18C,24
            if "RWY" in name:
                rwykey = name[8:10]
                if len(name) > 10 and not name[10].isdigit():
                    rwykey = name[8:11]
            # also if it is only RW
            else:
                rwykey = name[7:9]
                if len(name) > 9 and not name[9].isdigit():
                    rwykey = name[7:10]

            # Use this code to look up runway heading
            wphdg = self.navigation.rwythresholds[name[:4]][rwykey][2]

            # keep constant runway heading
            self.traffic.stack_command("HDG " + str(self.acid) + " " + str(wphdg))

            # start decelerating
            self.traffic.stack_command("DELAY " + "10 " + "SPD " + str(self.acid) + " " + "10")

            # delete aircraft
            self.traffic.stack_command("DELAY " + "42 " + "DEL " + str(self.acid))

            return self.WaypointTransition(
                LatLonDegrees(self.wplat[self.iactwp], self.wplon[self.iactwp]),
                self.wpspd[self.iactwp],
                self.wpprofile[self.iactwp],
                lnavon,
                self.wpflyby[self.iactwp],
                self.wpturn[self.iactwp],
                None,
                self.iactwp == n_wpt - 1,
            )

        # Switch LNAV off when last waypoint has been passed
        lnavon = self.iactwp < n_wpt - 1

        # if LNAV on: increase counter
        if lnavon:
            self.iactwp += 1

        # Activate switch to indicate that this is the last waypoint (for lenient passing logic in actwp.Reached function)
        swlastwp = self.iactwp == n_wpt - 1

        # Endpoint of the leg after the new active waypoint; the autopilot
        # computes the next-leg bearings for all switching aircraft in one
        # vectorised qdrdist call (see wppassingcheck).
        next_leg = self.getnextleg()

        # in case that there is a runway, the aircraft should remain on it
        # instead of deviating to the airport centre
        # When there is a destination: current = runway, next  = Dest
        # Else: current = runway and this is also the last waypoint
        if (self.wptype[self.iactwp] == 5 and self.wpname[self.iactwp] == self.wpname[-1]) or (
            self.wptype[self.iactwp] == 5
            and self.iactwp + 1 < n_wpt
            and self.wptype[self.iactwp + 1] == 3
        ):
            self.flag_landed_runway = True

        return self.WaypointTransition(
            LatLonDegrees(self.wplat[self.iactwp], self.wplon[self.iactwp]),
            self.wpspd[self.iactwp],
            self.wpprofile[self.iactwp],
            lnavon,
            self.wpflyby[self.iactwp],
            self.wpturn[self.iactwp],
            next_leg,
            swlastwp,
        )

    def runactwpstack(self) -> None:
        """Execute the stack commands stored for the active waypoint.

        Commands are attached to waypoints with the AT ... DO/STACK command
        and are issued when the aircraft passes the waypoint.
        """
        for cmdline in self.wpstack[self.iactwp]:
            self.traffic.stack_command(cmdline)
            # debug
            # stack.stack("ECHO "+self.acid+" AT "+self.wpname[self.iactwp]+" command issued:"+cmdline)

    def insertcalcwp(self, i: int, name: str) -> None:
        """Insert an empty calculated waypoint (T/C, T/D) at location i."""

        self.wpname.insert(i, name)
        self.wplat.insert(i, 0.0)
        self.wplon.insert(i, 0.0)
        self.wpalt.insert(i, None)
        self.wpspd.insert(i, None)
        self.wptype.insert(i, Route.calcwp)

    def calcfp(self) -> None:
        """Current Flight Plan calculations, which actualize based on flight condition

        This routine prepares data for this by adding a "ruler" along the flight
        plan in the form of distance at wp to next altitude constraint (xtoalt),
        its index ial and the value (toalt). Same logic is used for time constraint.

        Note: No Top of Descent or Top of Climb can inserted here as this depends on
        the speed, which might be undefined (often is). Guidance in autpilot.py takes
        care of ToD and ToC logic while flying using current speed.

        Recomputes, per waypoint: leg directions [deg] and lengths [nm]
        (wpdirfrom, wpdirto, wpdistto), plus typed altitude and RTA guidance
        targets in wpprofile.
        """

        # Direction to waypoint
        n_wpt = len(self.wpname)

        # Create cleared flight plan calculation table
        # [deg] Direction of leg laving this waypoint
        self.wpdirfrom = n_wpt * [0.0]

        # [deg] Direction of leg ot this waypoint (if it exists)
        self.wpdirto = n_wpt * [0.0]

        # [nm] Distance of leg to this waypoint in nm
        self.wpdistto = n_wpt * [0.0]

        self.wpprofile = [RouteProfile() for _ in range(n_wpt)]

        # No waypoints: make empty variables to be safe and return: nothing to do
        if n_wpt == 0:
            return

        # Calculate lateral leg data
        # LNAV: Calculate leg distances and directions

        for i in range(n_wpt - 1):
            qdr, dist = geo.qdrdist(
                self.wplat[i], self.wplon[i], self.wplat[i + 1], self.wplon[i + 1]
            )
            self.wpdirfrom[i] = float(qdr)  # [deg]
            self.wpdistto[i + 1] = float(dist)  # [nm] distto is in nautical miles

        # Also add "from direction" as to directions so no need to shift for actwpdata
        # direction to will be overwritten in actwpdata in case of a direct to
        # Add current pos to first waypoint as default value for direction to 1st waypoint
        iac = self.traffic.callsign.index(self.acid)
        qdr, _dist = geo.qdrdist(
            self.traffic.lat[iac], self.traffic.lon[iac], self.wplat[0], self.wplon[0]
        )
        self.wpdirto = [qdr, *self.wpdirfrom[0:-1]]  # [deg] Direction to waypoints

        # Continue flying in the saem direction
        if n_wpt > 1:
            self.wpdirfrom[-1] = self.wpdirfrom[-2]

        # Calculate longitudinal leg data
        # VNAV: calc next altitude constraint and distance to it
        altitude_target: AltitudeTarget | None = None
        for i in range(n_wpt - 1, -1, -1):
            # waypoint with altitude constraint (dest of all specified)
            if self.wptype[i] == Route.dest:
                altitude_target = AltitudeTarget(0.0, 0.0)
            elif (altitude := self.wpalt[i]) is not None:
                altitude_target = AltitudeTarget(altitude, 0.0)
            # waypoint with no altitude constraint: keep counting
            elif altitude_target is not None and i != n_wpt - 1:
                altitude_target = AltitudeTarget(
                    altitude_target.altitude,
                    altitude_target.distance + self.wpdistto[i + 1] * nm,
                )
            else:
                altitude_target = None
            self.wpprofile[i] = RouteProfile(altitude=altitude_target)

        # RTA: calc next rta constraint and distance to it
        rta_target: RtaTarget | None = None
        for i in range(n_wpt - 1, -1, -1):
            # waypoint with rta: reset counter, update target
            if (rta := self.wprta[i]) is not None:
                rta_target = RtaTarget(rta, 0.0)
            elif rta_target is not None and i != n_wpt - 1:
                # No speed or rta constraint: add to distance
                if (speed := self.wpspd[i]) is None:
                    rta_target = RtaTarget(
                        rta_target.time,
                        rta_target.distance + self.wpdistto[i + 1] * nm,
                    )
                else:
                    # speed constraint on this leg: shift RTA time to account for this
                    # altitude unknown; use the next altitude constraint for this waypoint
                    # or default to 10000 ft when no altitude constraint is present
                    altitude_target = self.wpprofile[i].altitude
                    altitude = (
                        altitude_target.altitude
                        if altitude_target is not None and altitude_target.altitude > 0.0
                        else 10000.0 * ft
                    )
                    legtas = casormach2tas(speed, altitude, self.traffic.casmach_threshold)
                    # TODO(abraham): account for wind at this position by adding wind vectors to waypoints?

                    # This fixed-speed leg is excluded from RTA distance, so subtract its
                    # travel time from the target time instead.
                    # FIXME(abraham): BlueSky f352d73 (2019-07-14) computes this leg time
                    # from nautical miles / m/s, missing the nm conversion.
                    legtime = self.wpdistto[i + 1] / legtas
                    rta_target = RtaTarget(rta_target.time - legtime, rta_target.distance)
            else:
                rta_target = None

            self.wpprofile[i] = self.wpprofile[i]._replace(rta=rta_target)

    def findact(self, i: int) -> int | None:
        """Find the best default active waypoint for an aircraft.

        Called when LNAV is (re-)engaged. Selects the waypoint closest to
        the aircraft, without walking back to earlier waypoints, and skips
        to the next waypoint when the closest one cannot be reached with
        the required heading change (turn time exceeds straight flight
        time).

        Args:
            i: Aircraft index.

        Returns:
            Index of the suggested active waypoint in this route, or None
            for an empty route.
        """

        n_wpt = len(self.wpname)

        # Check for easy answers first
        if n_wpt <= 0:
            return None

        elif n_wpt == 1:
            return 0

        # Find closest
        wplat = np.array(self.wplat)
        wplon = np.array(self.wplon)
        dy = wplat - self.traffic.lat[i]
        dx = (wplon - self.traffic.lon[i]) * self.traffic.coslat[i]
        dist2 = dx * dx + dy * dy
        # Note: the max() prevents walking back, even in cases when this might be apropriate,
        # such as when previous waypoints have been deleted

        iwpnear = max(self.iactwp, np.argmin(dist2))

        # Unless behind us, next waypoint?
        if iwpnear + 1 < n_wpt:
            qdr = math.degrees(math.atan2(dx[iwpnear], dy[iwpnear]))
            delhdg = abs(degto180(self.traffic.trk[i] - qdr))

            # we only turn to the first waypoint if we can reach the required
            # heading before reaching the waypoint
            time_turn = (
                max(0.01, self.traffic.tas[i])
                * math.radians(delhdg)
                / (g0 * math.tan(self.traffic.ap.bankdef[i]))
            )
            time_straight = math.sqrt(dist2[iwpnear]) * 60.0 * nm / max(0.01, self.traffic.tas[i])

            if time_turn > time_straight:
                iwpnear += 1

        return int(iwpnear)

    def getnextleg(self) -> LatLonDegrees | None:
        """Return the endpoint of the leg after the active waypoint."""
        if -1 < self.iactwp < len(self.wpname) - 1:
            return LatLonDegrees(self.wplat[self.iactwp + 1], self.wplon[self.iactwp + 1])
        return None

    def getnextqdr(self) -> float | None:
        """Return the bearing of the leg after the active waypoint [deg]."""
        # get qdr for next leg
        next_leg = self.getnextleg()
        if next_leg is None:
            return None
        nextqdr, _dist = geo.qdrdist(
            self.wplat[self.iactwp],
            self.wplon[self.iactwp],
            next_leg.lat,
            next_leg.lon,
        )
        return float(nextqdr)


# ---- following are functions managing the routes ----


def get_available_name(data: list, name_: str, callsigns: list[str], len_: int = 2) -> str:
    """Make a waypoint name unique by appending a zero-padded number.

    Checks if the name already exists in the given list (or matches an
    aircraft callsign); if so, appends/increments an integer suffix
    (01, 02, 03, ...) until the name is unique.

    Args:
        data: Existing names (e.g. the wpname list of a route).
        name_: Requested base name.
        len_: Number of digits of the appended counter (default 2).

    Returns:
        str: A name that does not yet occur in data.
    """
    appi = 0  # appended integer to name starts at zero (=nothing)
    # Use Python 3 formatting syntax: "{:03d}".format(7) => "007"
    fmt_ = "{:0" + str(len_) + "d}"

    # Avoid using call sign without number
    if callsigns.count(name_) > 0:
        appi = 1
        name_ = name_ + fmt_.format(appi)

    while data.count(name_) > 0:
        appi += 1
        name_ = name_[:-len_] + fmt_.format(appi)
    return name_


class WaypointMode(Enum):
    FLYBY = auto()
    FLYOVER = auto()
    FLYTURN = auto()


class TurnParameter(Enum):
    RADIUS = auto()
    SPEED = auto()
    HEADING_RATE = auto()


@dataclass(frozen=True, slots=True)
class InsertAfter:
    waypoint: str


@dataclass(frozen=True, slots=True)
class InsertBefore:
    waypoint: str


WaypointInsertion: TypeAlias = InsertAfter | InsertBefore


_WAYPOINT_MODES = {
    "FLYBY": WaypointMode.FLYBY,
    "FLY-BY": WaypointMode.FLYBY,
    "FLYOVER": WaypointMode.FLYOVER,
    "FLY-OVER": WaypointMode.FLYOVER,
    "FLYTURN": WaypointMode.FLYTURN,
    "FLY-TURN": WaypointMode.FLYTURN,
}
_TURN_PARAMETERS = {
    "TURNRAD": TurnParameter.RADIUS,
    "TURNRADIUS": TurnParameter.RADIUS,
    "TURNSPD": TurnParameter.SPEED,
    "TURNSPEED": TurnParameter.SPEED,
    "TURNHDG": TurnParameter.HEADING_RATE,
    "TURNHDGR": TurnParameter.HEADING_RATE,
    "TURNHDGRATE": TurnParameter.HEADING_RATE,
}
# bluesky also accepts TURNBANK/TURNPHI. minisky intentionally rejects them:
# its route model has no per-waypoint bank state, so accepting the syntax would
# silently discard behavior. add them only when that state is represented.


def _parse_waypoint_mode(context: CommandParseContext, text: str) -> ParseResult[WaypointMode]:
    if isinstance(result := parse_keyword(context, text), Err):
        return result
    token = result.ok()
    mode = _WAYPOINT_MODES.get(token.value)
    if mode is None:
        return Err(ArgumentIssue.expected("FLYBY, FLYOVER, or FLYTURN", token.value, token.span))
    return Ok(Parsed(mode, token.remainder, token.span))


WaypointModeArg = Annotated[
    WaypointMode,
    CmdParser.literals(_parse_waypoint_mode, tuple(_WAYPOINT_MODES)),
]


def _parse_turn_parameter(context: CommandParseContext, text: str) -> ParseResult[TurnParameter]:
    if isinstance(result := parse_keyword(context, text), Err):
        return result
    token = result.ok()
    parameter = _TURN_PARAMETERS.get(token.value)
    if parameter is None:
        return Err(ArgumentIssue.expected("a supported turn parameter", token.value, token.span))
    return Ok(Parsed(parameter, token.remainder, token.span))


TurnParameterArg = Annotated[
    TurnParameter,
    CmdParser.literals(_parse_turn_parameter, tuple(_TURN_PARAMETERS)),
]


def _parse_runway(context: CommandParseContext, text: str) -> ParseResult[RunwayPosition]:
    if isinstance(result := parse_resolved_position(context, text), Err):
        return result
    parsed = result.ok()
    if not isinstance(parsed.value, RunwayPosition):
        return Err(ArgumentIssue.expected("a runway", text, parsed.span))
    return Ok(Parsed(parsed.value, parsed.remainder, parsed.span))


RunwayArg = Annotated[RunwayPosition, CmdParser(_parse_runway)]


def _waypoint_mode_status(traffic: Traffic, acidx: int) -> Result[str, str]:
    acrte = traffic.ap.route[acidx]
    if acrte.swflyturn:
        mode = "FLYTURN"
    elif acrte.swflyby:
        mode = "FLYBY"
    else:
        mode = "FLYOVER"
    return Ok(f"Current ADDWPT mode is {mode}.")


def _set_waypoint_mode(traffic: Traffic, acidx: int, mode: WaypointMode) -> Result[str, str]:
    """Set fly-by/fly-over/fly-turn behavior for newly added route waypoints."""
    acrte = traffic.ap.route[acidx]
    acrte.swflyby = mode is WaypointMode.FLYBY
    acrte.swflyturn = mode is WaypointMode.FLYTURN
    return Ok("")


def _set_turn_parameter(
    traffic: Traffic,
    acidx: int,
    parameter: TurnParameter,
    value: PositiveFiniteFloat | None,
) -> Result[str, str]:
    """Set or clear a fly-turn parameter using the units written in SCN text.

    BlueSky parsed these values through its waypoint-altitude path and later
    undid that conversion. MiniSky keeps the command value in its natural
    unit here: nautical miles for radius, knots for speed, and degrees/second
    for heading rate.
    """
    acrte = traffic.ap.route[acidx]
    if parameter is TurnParameter.RADIUS:
        geometry = acrte.turn.geometry
        if value is not None:
            geometry = TurnRadius(value * nm)
        elif isinstance(geometry, TurnRadius):
            geometry = None
        acrte.turn = acrte.turn._replace(geometry=geometry)
    elif parameter is TurnParameter.SPEED:
        acrte.turn = acrte.turn._replace(speed=None if value is None else value * kts)
    else:
        geometry = acrte.turn.geometry
        if value is not None:
            geometry = TurnHeadingRate(value)
        elif isinstance(geometry, TurnHeadingRate):
            geometry = None
        acrte.turn = acrte.turn._replace(geometry=geometry)
    acrte.swflyby = False
    acrte.swflyturn = True
    return Ok("")


def _add_takeoff_waypoint(
    traffic: Traffic, acidx: int, runway: RunwayPosition | None = None
) -> Result[str, str]:
    callsign = traffic.callsign[acidx]
    acrte = traffic.ap.route[acidx]
    rwyrteidx = next((i for i, name in enumerate(acrte.wpname) if "/" in name), -1)

    if runway is not None:
        rwylat = runway.coordinates.lat
        rwylon = runway.coordinates.lon
        rwyhdg = runway.runway_heading
    elif rwyrteidx > 0:
        rwylat = acrte.wplat[rwyrteidx]
        rwylon = acrte.wplon[rwyrteidx]
        aptidx = traffic.navigation.getapinear(rwylat, rwylon)
        aptname = traffic.navigation.aptname[aptidx]
        rwyname = acrte.wpname[rwyrteidx].split("/")[1]
        rwyid = rwyname.replace("RWY", "").replace("RW", "")
        rwyhdg = traffic.navigation.rwythresholds[aptname][rwyid][2]
    else:
        rwylat = traffic.lat[acidx]
        rwylon = traffic.lon[acidx]
        rwyhdg = traffic.trk[acidx]

    lat, lon = geo.qdrpos(rwylat, rwylon, rwyhdg, 2.0)
    afterwp = ""
    if rwyrteidx > 0:
        afterwp = acrte.wpname[rwyrteidx]
    elif acrte.wptype and acrte.wptype[0] == Route.orig:
        afterwp = acrte.wpname[0]

    name = f"T/O-{callsign}"
    wpidx = acrte.add_waypoint(acidx, name, Route.wplatlon, lat, lon, None, None, afterwp, "")
    acrte.calcfp()
    if wpidx < 0:
        return Err(f"Waypoint {name} not added.")

    norig = int(traffic.ap.orig[acidx] != "")
    ndest = int(traffic.ap.dest[acidx] != "")
    if len(acrte.wpname) - norig - ndest == 1:
        direct(traffic, acidx, acrte.wpname[norig])
        traffic.swlnav[acidx] = True
    if afterwp and acrte.wpname.count(afterwp) == 0:
        return Ok(f"Waypoint {afterwp} not found\nwaypoint added at end of route")
    return Ok("")


def _add_route_waypoint(
    traffic: Traffic,
    acidx: int,
    waypoint: Wpt,
    altitude: AltM | None = None,
    speed: SpeedMpsOrMach | None = None,
    insertion: WaypointInsertion | None = None,
) -> Result[str, str]:
    """Apply already-parsed waypoint insertion request."""
    callsign = traffic.callsign[acidx]
    acrte = traffic.ap.route[acidx]

    n_wpt = len(acrte.wpname)

    # Choose reference position ot look up VOR and waypoints
    # First waypoint: own position
    if n_wpt == 0:
        reflat = traffic.lat[acidx]
        reflon = traffic.lon[acidx]

    # Or last waypoint before destination
    else:
        if acrte.wptype[-1] != Route.dest or n_wpt == 1:
            reflat = acrte.wplat[-1]
            reflon = acrte.wplon[-1]
        else:
            reflat = acrte.wplat[-2]
            reflon = acrte.wplon[-2]

    alt = altitude
    spd = speed
    afterwp = ""
    beforewp = ""

    match waypoint:
        case CoordinateWaypoint(coordinates):
            name = callsign
            lat = coordinates.lat
            lon = coordinates.lon
            wptype = Route.wplatlon
        case NamedWaypoint(name):
            takeoffwpt = name.replace("-", "") == "TAKEOFF"
            if takeoffwpt:
                if altitude is not None or speed is not None or insertion is not None:
                    return Err("TAKEOFF does not accept waypoint constraints")
                return _add_takeoff_waypoint(traffic, acidx)

            match txt2pos(name, reflat, reflon, traffic.navigation, traffic):
                case Ok(posobj):
                    lat = posobj.lat
                    lon = posobj.lon
                    if posobj.type in {"nav", "apt"}:
                        wptype = Route.wpnav
                    elif posobj.type == "rwy":
                        wptype = Route.runway
                    else:
                        name = callsign
                        wptype = Route.wplatlon
                case Err():
                    return Err("Waypoint " + name + " not found.")

    match insertion:
        case InsertAfter(anchor):
            afterwp = anchor
        case InsertBefore(anchor):
            beforewp = anchor
        case None:
            pass

    # Add waypoint
    wpidx = acrte.add_waypoint(acidx, name, wptype, lat, lon, alt, spd, afterwp, beforewp)

    # Recalculate flight plan
    acrte.calcfp()

    # Check for success by checking inserted location in flight plan >= 0
    if wpidx < 0:
        return Err(f"Waypoint {name} not added.")

    # check for presence of orig/dest
    norig = int(traffic.ap.orig[acidx] != "")  # 1 if orig is present in route
    ndest = int(traffic.ap.dest[acidx] != "")  # 1 if dest is present in route

    # Check whether this is first 'real' waypoint (not orig & dest),
    # And if so, make active
    if len(acrte.wpname) - norig - ndest == 1:  # first waypoint: make active
        direct(traffic, acidx, acrte.wpname[norig])  # 0 if no orig
        # print("direct ",self.wpname[norig])
        traffic.swlnav[acidx] = True

    if afterwp and acrte.wpname.count(afterwp) == 0:
        return Ok("Waypoint " + afterwp + " not found\n" + "waypoint added at end of route")
    else:
        return Ok("")


class AtQuery(Enum):
    """Constraint subset shown by AT query forms."""

    ALL = auto()
    ALTITUDE = auto()
    SPEED = auto()
    STACK = auto()


@dataclass(frozen=True, slots=True)
class AtConstraints:
    """A complete altitude/speed pair; None means an explicit clear."""

    altitude: float | None
    speed: float | None


def _parse_at_constraints(_context: CommandParseContext, text: str) -> ParseResult[AtConstraints]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    if token.value.count("/") != 1:
        return Err(ArgumentIssue.expected("altitude/speed", token.value, token.span))
    altitude_text, speed_text = token.value.split("/", maxsplit=1)

    def cleared(value: str) -> bool:
        return len(value) > 1 and set(value) == {"-"}

    altitude: float | None
    if cleared(altitude_text):
        altitude = None
    else:
        if isinstance(parsed_altitude := parse_altitude_value(altitude_text), Err):
            return Err(parsed_altitude.err().with_span(token.span))
        altitude = parsed_altitude.ok()

    speed: float | None
    if cleared(speed_text):
        speed = None
    else:
        if isinstance(parsed_speed := parse_speed_value(speed_text), Err):
            return Err(parsed_speed.err().with_span(token.span))
        speed = parsed_speed.ok()

    return Ok(Parsed(AtConstraints(altitude, speed), token.remainder, token.span))


AtConstraintsArg = Annotated[
    AtConstraints, CmdParser.fields(_parse_at_constraints, ("altitude/speed",))
]


@dataclass(frozen=True, slots=True)
class _AtWaypoint:
    acid: str
    route: Route
    index: int


def _route_waypoint_index(traffic: Traffic, acidx: int, wpname: str) -> Result[int, str]:
    route = traffic.ap.route[acidx]
    try:
        return Ok(route.wpname.index(wpname.upper()))
    except ValueError:
        return Err(f"Waypoint {wpname} not found in the route of {traffic.callsign[acidx]}")


def _at_waypoint(traffic: Traffic, acidx: int, atwp: str) -> Result[_AtWaypoint, str]:
    if isinstance(index := _route_waypoint_index(traffic, acidx, atwp), Err):
        return index
    return Ok(_AtWaypoint(traffic.callsign[acidx], traffic.ap.route[acidx], index.ok()))


def _finish_at_mutation(traffic: Traffic, acidx: int, target: _AtWaypoint) -> Result[str, str]:
    target.route.calcfp()
    direct(traffic, acidx, target.route.wpname[target.route.iactwp])
    return Ok("")


def _format_at_query(acrte: Route, wpidx: int, atwp: str, query: AtQuery) -> str:
    show_altitude = query in {AtQuery.ALL, AtQuery.ALTITUDE}
    show_speed = query in {AtQuery.ALL, AtQuery.SPEED}
    show_stack = query in {AtQuery.ALL, AtQuery.STACK}
    text = f"{atwp} : "

    if show_altitude:
        altitude = acrte.wpalt[wpidx]
        if altitude is None:
            text += "-----"
        elif altitude > 4500 * ft:
            text += f"FL{round(altitude / (100.0 * ft))}"
        else:
            text += str(round(altitude / ft))
        if show_speed:
            text += "/"

    if show_speed:
        speed = acrte.wpspd[wpidx]
        text += "---" if speed is None else str(round(speed / kts))

    if show_altitude and show_speed:
        if acrte.wptype[wpidx] == Route.orig:
            text += "[orig]"
        elif acrte.wptype[wpidx] == Route.dest:
            text += "[dest]"

    if show_stack and acrte.wpstack[wpidx]:
        stacked = "\n".join(acrte.wpstack[wpidx])
        text += f"\nStack:\n{stacked}\n"
    return text


def direct(traffic: Traffic, acidx: int, wpname: str) -> bool:
    """Go direct to a specified waypoint in the route.

    Implements the DIRECT stack command: `DIRECT acid wpname`. Makes the
    given waypoint the active waypoint, copies its data (position, fly-by/
    fly-turn settings, next-turn data) into the active-waypoint arrays,
    recalculates the flight plan and the VNAV profile, sets the next-leg
    speed from any speed constraint, computes the turn distance for the
    new leg, and engages LNAV.

    Args:
        acidx: Aircraft index.
        wpname: Name of the waypoint in the route to fly direct to.

    Returns:
        bool: True on success.
    """
    traffic.callsign[acidx]
    acrte = traffic.ap.route[acidx]
    wpidx = acrte.wpname.index(wpname)

    acrte.iactwp = wpidx
    traffic.actwp.lat[acidx] = acrte.wplat[wpidx]
    traffic.actwp.lon[acidx] = acrte.wplon[wpidx]
    traffic.actwp.flyby[acidx] = acrte.wpflyby[wpidx]
    turn = acrte.wpturn[wpidx]
    traffic.actwp.flyturn[acidx] = turn is not None
    geometry = None if turn is None else turn.geometry
    traffic.actwp.turnrad[acidx] = (
        geometry.radius if isinstance(geometry, TurnRadius) else np.ma.masked
    )
    traffic.actwp.turnspd[acidx] = (
        np.ma.masked if turn is None or turn.speed is None else turn.speed
    )
    traffic.actwp.turnhdgr[acidx] = (
        geometry.heading_rate if isinstance(geometry, TurnHeadingRate) else np.ma.masked
    )

    next_turn = acrte.getnextturnwp()
    if next_turn is None:
        traffic.actwp.nextturnlat[acidx] = np.ma.masked
        traffic.actwp.nextturnlon[acidx] = np.ma.masked
        traffic.actwp.nextturnspd[acidx] = np.ma.masked
        traffic.actwp.nextturnrad[acidx] = np.ma.masked
        traffic.actwp.nextturnhdgr[acidx] = np.ma.masked
        traffic.actwp.nextturnidx[acidx] = np.ma.masked
    else:
        traffic.actwp.nextturnlat[acidx] = next_turn.latitude
        traffic.actwp.nextturnlon[acidx] = next_turn.longitude
        traffic.actwp.nextturnspd[acidx] = (
            np.ma.masked if next_turn.turn.speed is None else next_turn.turn.speed
        )
        next_geometry = next_turn.turn.geometry
        traffic.actwp.nextturnrad[acidx] = (
            next_geometry.radius if isinstance(next_geometry, TurnRadius) else np.ma.masked
        )
        traffic.actwp.nextturnhdgr[acidx] = (
            next_geometry.heading_rate
            if isinstance(next_geometry, TurnHeadingRate)
            else np.ma.masked
        )
        traffic.actwp.nextturnidx[acidx] = next_turn.waypoint_index

    # Determine next turn waypoint data

    # Do calculation for VNAV
    acrte.calcfp()

    profile = acrte.wpprofile[wpidx]
    if profile.altitude is None:
        traffic.actwp.nextaltco[acidx] = np.ma.masked
        traffic.actwp.xtoalt[acidx] = np.ma.masked
    else:
        traffic.actwp.nextaltco[acidx] = profile.altitude.altitude
        traffic.actwp.xtoalt[acidx] = profile.altitude.distance

    if profile.rta is None:
        traffic.actwp.torta[acidx] = np.ma.masked
        traffic.actwp.xtorta[acidx] = np.ma.masked
    else:
        traffic.actwp.torta[acidx] = profile.rta.time
        traffic.actwp.xtorta[acidx] = profile.rta.distance

    # If there is a speed specified, process it
    if (speed := acrte.wpspd[wpidx]) is not None:
        altitude = acrte.wpalt[wpidx]
        alt = traffic.alt[acidx] if altitude is None else altitude

        # Check for valid Mach or CAS
        cas = mach2cas(speed, alt) if speed < 2.0 else speed

        # Save it for next leg
        traffic.actwp.nextspd[acidx] = cas

    # No speed specified for next leg
    else:
        traffic.actwp.nextspd[acidx] = np.ma.masked

    qdr_result, dist_result = geo.qdrdist(
        traffic.lat[acidx],
        traffic.lon[acidx],
        traffic.actwp.lat[acidx],
        traffic.actwp.lon[acidx],
    )
    qdr_ = float(np.asarray(qdr_result).item())
    leg_distance = float(np.asarray(dist_result).item()) * nm

    # Save leg length & direction in actwp data
    traffic.actwp.curlegdir[acidx] = qdr_  # [deg]
    traffic.actwp.curleglen[acidx] = leg_distance  # [m]
    traffic.ap.qdr2wp[acidx] = qdr_ % 360.0
    traffic.ap.dist2wp[acidx] = leg_distance

    # FIXME(abraham): BlueSky a918582 (2022-12-30) introduced a bug: an explicit radius
    # is ignored when no heading rate is set because this path falls back to bank-derived radius
    if isinstance(geometry, TurnHeadingRate):
        turnrad = traffic.tas[acidx] * 360.0 / (2 * math.pi * geometry.heading_rate)
    else:
        turnrad = (
            traffic.tas[acidx] * traffic.tas[acidx] / math.tan(math.radians(acrte.bank)) / g0 / nm
        )

    # TODO(abraham): direct() still mixes nautical-mile turn geometry with the SI
    # ActiveWaypoint turn-distance boundary; migrate this calculation to SI.
    traffic.actwp.turndist[acidx] = (
        np.logical_or(isinstance(geometry, TurnHeadingRate), traffic.actwp.flyby[acidx] > 0.5)
        * turnrad
        * abs(
            math.tan(
                0.5 * math.radians(max(5.0, abs(degto180(qdr_ - acrte.wpdirfrom[acrte.iactwp]))))
            )
        )
    )  # [nm]

    # NOTE: in bluesky cca80df (2016-11-05), ComputeVNAV() was inserted before
    # the direct-to leg geometry had been calculated
    # (which could be the previous leg's distance)
    # the later direct-to turn-distance calculation from bluesky 08194fa
    # (2023-06-22) was also still after ComputeVNAV()
    # we current-leg distance, bearing, and turn distance first
    traffic.ap.ComputeVNAV(acidx, profile, leg_distance)

    traffic.swlnav[acidx] = True
    return True


def set_rta(
    traffic: Traffic, acidx: int, wpname: str, time: TimeS
) -> bool:  # all arguments of setRTA
    """Set a required time of arrival (RTA) at a route waypoint.

    Implements the RTA stack command: `RTA acid, wpname, time`. The RTA
    is stored with the waypoint and the guidance to the active waypoint is
    recomputed so the autopilot can adjust its speed schedule.

    Args:
        acidx: Aircraft index.
        wpname: Name of the waypoint in the route.
        time: Required time of arrival as simulation time [s].

    Returns:
        bool: True on success.
    """
    traffic.callsign[acidx]
    acrte = traffic.ap.route[acidx]
    wpidx = acrte.wpname.index(wpname)
    acrte.wprta[wpidx] = time

    # Recompute route and update actwp because of RTA addition
    direct(traffic, acidx, acrte.wpname[acrte.iactwp])

    return True


def listrte(traffic: Traffic, acidx: int, ipagetxt: str = "0") -> Result[None, str]:
    """Show the route of an aircraft in the console, page by page.

    Implements the LISTRTE stack command: `LISTRTE acid, [pagenr]`.
    Each line shows the waypoint name (active waypoint marked with `*`),
    its altitude constraint (ft or FL), speed constraint (kts or Mach) and
    type ([orig], [dest], [C] fly-by, [|] fly-over, [U] fly-turn). Seven
    waypoints are shown per page.

    Args:
        acidx: Aircraft index.
        ipagetxt: Page number as text (default "0").

    Returns:
        Result: `Ok(None)` after listing the route, or `Err` when no route exists.
    """
    # First get the appropriate ac route
    ipage = int(ipagetxt)
    acrte = traffic.ap.route[acidx]

    n_wpt = len(acrte.wpname)

    if n_wpt <= 0:
        return Err("Aircraft has no route.")

    for i in range(ipage * 7, ipage * 7 + 7):
        if 0 <= i < n_wpt:
            # Name
            if i == acrte.iactwp:
                txt = "*" + acrte.wpname[i] + " : "
            else:
                txt = " " + acrte.wpname[i] + " : "

            # Altitude
            altitude = acrte.wpalt[i]
            if altitude is None:
                txt += "-----/"
            elif altitude > 4500 * ft:
                fl = round(altitude / (100.0 * ft))
                txt += "FL" + str(fl) + "/"
            else:
                txt += str(round(altitude / ft)) + "/"

            # Speed
            speed = acrte.wpspd[i]
            if speed is None:
                txt += "---"
            elif speed > 2.0:
                txt += str(round(speed / kts))
            else:
                txt += "M" + str(speed)

            # Type: orig, dest, C = flyby, | = flyover, U = flyturn
            if acrte.wptype[i] == Route.orig:
                txt += "[orig]"
            elif acrte.wptype[i] == Route.dest:
                txt += "[dest]"
            elif acrte.wpturn[i] is not None:
                txt += "[U]"
            elif acrte.wpflyby[i]:
                txt += "[C]"
            else:  # FLYOVER
                txt += "[|]"

            # Display message
            traffic.console.echo(txt)

    return Ok(None)


def delrte(traffic: Traffic, acidx: int) -> Result[str, str]:
    """Delete the complete route (including origin/destination) of an
    aircraft.

    Implements the DELRTE stack command: `DELRTE acid`. The route is
    re-initialized empty and LNAV/VNAV are disengaged. When no callsign is
    given and an aircraft exists, that aircraft is used.

    Args:
        acidx: Aircraft index; may be None when a single aircraft exists.
    """
    # Simple re-initialize this route as empty
    acid = traffic.callsign[acidx]
    acrte = traffic.ap.route[acidx]
    acrte.__init__(traffic, acid)

    # Also disable LNAV,VNAV if route is deleted
    traffic.swlnav[acidx] = False
    traffic.swvnav[acidx] = False
    traffic.swvnavspd[acidx] = False
    traffic.actwp.torta[acidx] = np.ma.masked
    traffic.actwp.xtorta[acidx] = np.ma.masked

    return Ok("")


def delwpt(traffic: Traffic, acidx: int, wpname: str) -> Result[str, str]:
    """Delete a single waypoint from the route of an aircraft.

    Implements the DELWPT stack command: `DELWPT acid, wpname`. When the
    deleted waypoint is the active one (and not the last), guidance is
    redirected to the following waypoint. LNAV/VNAV are disengaged when
    the route becomes empty.

    Args:
        acidx: Aircraft index.
        wpname: Name of the waypoint to delete.
    """

    # Look up waypoint
    acrte = traffic.ap.route[acidx]
    n_wpt = len(acrte.wpname)

    try:
        wpidx = acrte.wpname.index(wpname.upper())
    except ValueError:
        return Err("Waypoint " + wpname + " not found")

    # check if active way point is the one being deleted and that it is not the last wpt.
    # If active wpt is deleted then change path of aircraft
    if acrte.iactwp == wpidx and wpidx != n_wpt - 1:
        direct(traffic, acidx, acrte.wpname[wpidx + 1])

    n_wpt = n_wpt - 1

    del acrte.wpname[wpidx]
    del acrte.wplat[wpidx]
    del acrte.wplon[wpidx]
    del acrte.wpalt[wpidx]
    del acrte.wpspd[wpidx]
    del acrte.wprta[wpidx]
    del acrte.wptype[wpidx]
    del acrte.wpflyby[wpidx]
    del acrte.wpturn[wpidx]
    del acrte.wpstack[wpidx]

    if acrte.iactwp > wpidx:
        acrte.iactwp = max(0, acrte.iactwp - 1)

    acrte.iactwp = min(acrte.iactwp, n_wpt - 1)

    # If no waypoints left, make sure to disable LNAV/VNAV
    if n_wpt == 0 and (acidx or acidx == 0):
        traffic.swlnav[acidx] = False
        traffic.swvnav[acidx] = False
        traffic.swvnavspd[acidx] = False

    return Ok("")


class RouteCommands:
    """Runtime-bound stack commands for editing aircraft routes."""

    def __init__(self, traffic: Traffic) -> None:
        self.traffic = traffic

    @command(name="ADDWPTMODE")
    def report_waypoint_mode(self, acidx: AcId) -> Result[str, str]:
        """Show the current waypoint insertion/fly-turn mode."""
        return _waypoint_mode_status(self.traffic, acidx)

    @command(name="ADDWPTMODE")
    def select_waypoint_mode(self, acidx: AcId, mode: WaypointModeArg) -> Result[str, str]:
        """Select FLYBY, FLYOVER, or FLYTURN waypoint mode."""
        return _set_waypoint_mode(self.traffic, acidx, mode)

    @command(name="ADDWPTMODE")
    def set_turn_parameter(
        self, acidx: AcId, parameter: TurnParameterArg, value: PositiveFiniteFloat
    ) -> Result[str, str]:
        """Set a route turn radius, speed, or heading-rate default."""
        return _set_turn_parameter(self.traffic, acidx, parameter, value)

    @command(name="ADDWPTMODE")
    def clear_turn_parameter(
        self, acidx: AcId, parameter: TurnParameterArg, _off: Literal["OFF"]
    ) -> Result[str, str]:
        """Clear a route turn radius, speed, or heading-rate default."""
        return _set_turn_parameter(self.traffic, acidx, parameter, None)

    @command(name="ADDWPT")
    def add_waypoint_mode(self, acidx: AcId, mode: WaypointModeArg) -> Result[str, str]:
        """Select FLYBY, FLYOVER, or FLYTURN through the ADDWPT mode form."""
        return _set_waypoint_mode(self.traffic, acidx, mode)

    @command(name="ADDWPT")
    def clear_waypoint_turn_parameter(
        self, acidx: AcId, parameter: TurnParameterArg, _off: Literal["OFF"]
    ) -> Result[str, str]:
        """Clear a turn parameter through the ADDWPT mode form."""
        return _set_turn_parameter(self.traffic, acidx, parameter, None)

    @command(name="ADDWPT")
    def add_waypoint_turn_parameter(
        self, acidx: AcId, parameter: TurnParameterArg, value: PositiveFiniteFloat
    ) -> Result[str, str]:
        """Set a turn parameter through the ADDWPT mode form."""
        return _set_turn_parameter(self.traffic, acidx, parameter, value)

    @command(name="ADDWPT")
    def add_takeoff_waypoint_from_runway(
        self, acidx: AcId, _takeoff: Literal["TAKEOFF"], runway: RunwayArg
    ) -> Result[str, str]:
        """Add a takeoff waypoint from an explicit runway."""
        return _add_takeoff_waypoint(self.traffic, acidx, runway)

    @command(name="ADDWPT")
    def add_takeoff_waypoint(self, acidx: AcId, _takeoff: Literal["TAKEOFF"]) -> Result[str, str]:
        """Add a takeoff waypoint using a runway already in the route or current position."""
        return _add_takeoff_waypoint(self.traffic, acidx, None)

    @command(name="ADDWPT")
    def insert_waypoint_before(
        self,
        acidx: AcId,
        waypoint: Wpt,
        altitude: AltM | None,
        speed: SpeedMpsOrMach | None,
        _after: Omitted,
        before: Keyword,
    ) -> Result[str, str]:
        """Insert a route waypoint before an existing waypoint."""
        return _add_route_waypoint(
            self.traffic,
            acidx,
            waypoint,
            altitude,
            speed,
            InsertBefore(before),
        )

    @command(name="ADDWPT")
    def insert_waypoint_after(
        self,
        acidx: AcId,
        waypoint: Wpt,
        altitude: AltM | None,
        speed: SpeedMpsOrMach | None,
        after: Keyword,
    ) -> Result[str, str]:
        """Insert a route waypoint after an existing waypoint."""
        return _add_route_waypoint(
            self.traffic,
            acidx,
            waypoint,
            altitude,
            speed,
            InsertAfter(after),
        )

    @command(name="ADDWPT", aliases=("WPTYPE",))
    def append_waypoint(
        self,
        acidx: AcId,
        waypoint: Wpt,
        altitude: AltM | None = None,
        speed: SpeedMpsOrMach | None = None,
    ) -> Result[str, str]:
        """Append a route waypoint with optional altitude and speed constraints."""
        return _add_route_waypoint(self.traffic, acidx, waypoint, altitude, speed)

    @command(name="BEFORE")
    def insert_before(
        self,
        acidx: AcId,
        beforewp: Keyword,
        _keyword: Literal["ADDWPT"],
        waypoint: Wpt,
        alt: AltM | None = None,
        spd: SpeedMpsOrMach | None = None,
    ) -> Result[str, str]:
        """Insert a waypoint before an existing route waypoint.

        Implements `acid BEFORE waypoint ADDWPT new-waypoint [altitude speed]`.
        The ADDWPT keyword is part of the command grammar; insertion
        uses the same typed mutation path as ADDWPT.
        """
        if isinstance(found := _route_waypoint_index(self.traffic, acidx, beforewp), Err):
            return found
        return _add_route_waypoint(self.traffic, acidx, waypoint, alt, spd, InsertBefore(beforewp))

    @command(name="AFTER")
    def insert_after(
        self,
        acidx: AcId,
        afterwp: Keyword,
        _keyword: Literal["ADDWPT"],
        waypoint: Wpt,
        alt: AltM | None = None,
        spd: SpeedMpsOrMach | None = None,
    ) -> Result[str, str]:
        """Insert a waypoint after an existing route waypoint.

        Implements `acid AFTER waypoint ADDWPT new-waypoint [altitude speed]`.
        The ADDWPT keyword is part of the command grammar; insertion
        uses the same typed mutation path as ADDWPT.
        """
        if isinstance(found := _route_waypoint_index(self.traffic, acidx, afterwp), Err):
            return found
        return _add_route_waypoint(self.traffic, acidx, waypoint, alt, spd, InsertAfter(afterwp))

    @command(name="AT")
    def at_all(self, acidx: AcId, atwp: Keyword) -> Result[str, str]:
        """Show all constraints and stacked commands at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        return Ok(_format_at_query(target.route, target.index, atwp, AtQuery.ALL))

    @command(name="AT")
    def at_altitude(self, acidx: AcId, atwp: Keyword, _action: Literal["ALT"]) -> Result[str, str]:
        """Show the altitude constraint at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        return Ok(_format_at_query(target.route, target.index, atwp, AtQuery.ALTITUDE))

    @command(name="AT")
    def at_speed(
        self, acidx: AcId, atwp: Keyword, _action: Literal["SPD", "SPEED"]
    ) -> Result[str, str]:
        """Show the speed constraint at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        return Ok(_format_at_query(target.route, target.index, atwp, AtQuery.SPEED))

    @command(name="AT")
    def at_stack(
        self, acidx: AcId, atwp: Keyword, _action: Literal["DO", "STACK"]
    ) -> Result[str, str]:
        """Show commands stacked at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        return Ok(_format_at_query(target.route, target.index, atwp, AtQuery.STACK))

    @command(name="AT")
    def set_at_altitude(
        self, acidx: AcId, atwp: Keyword, _action: Literal["ALT"], value: AltM
    ) -> Result[str, str]:
        """Set the altitude constraint at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpalt[target.index] = value
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def set_at_speed(
        self,
        acidx: AcId,
        atwp: Keyword,
        _action: Literal["SPD", "SPEED"],
        value: SpeedMpsOrMach,
    ) -> Result[str, str]:
        """Set the speed constraint at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpspd[target.index] = None if value <= 0.0 else value
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def set_at_constraints(
        self, acidx: AcId, atwp: Keyword, constraints: AtConstraintsArg
    ) -> Result[str, str]:
        """Set or clear the altitude/speed constraint pair at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        altitude = constraints.altitude
        speed = constraints.speed
        target.route.wpalt[target.index] = altitude
        target.route.wpspd[target.index] = None if speed is None or speed <= 0.0 else speed
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def clear_at_altitude(
        self,
        acidx: AcId,
        atwp: Keyword,
        _action: Literal["DEL", "DELETE", "CLR", "CLEAR"],
        _target: Literal["ALT"],
    ) -> Result[str, str]:
        """Clear a waypoint altitude constraint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpalt[target.index] = None
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def clear_at_speed(
        self,
        acidx: AcId,
        atwp: Keyword,
        _action: Literal["DEL", "DELETE", "CLR", "CLEAR"],
        _target: Literal["SPD", "SPEED"],
    ) -> Result[str, str]:
        """Clear a waypoint speed constraint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpspd[target.index] = None
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def clear_at_constraints(
        self,
        acidx: AcId,
        atwp: Keyword,
        _action: Literal["DEL", "DELETE", "CLR", "CLEAR"],
        _target: Literal["BOTH"],
    ) -> Result[str, str]:
        """Clear waypoint altitude and speed constraints."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpalt[target.index] = None
        target.route.wpspd[target.index] = None
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def clear_at_all(
        self,
        acidx: AcId,
        atwp: Keyword,
        _action: Literal["DEL", "DELETE", "CLR", "CLEAR"],
        _target: Literal["ALL"],
    ) -> Result[str, str]:
        """Clear all constraints and stacked commands at a waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpalt[target.index] = None
        target.route.wpspd[target.index] = None
        target.route.wpstack[target.index] = []
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def store_at_command(
        self, acidx: AcId, atwp: Keyword, _action: Literal["DO", "STACK"], command_text: Text
    ) -> Result[str, str]:
        """Store a command to execute when a route waypoint is passed.

        !!! important

            Note that we deliberately differ from bluesky in a subtle way.
            BlueSky supports an implicit-ownship shorthand.
            For example, BlueSky interprets `HX2 AT WPT10 STACK ALT 95`
            as though the deferred command were `HX2 ALT 95` (the HX2 target
            here is injected *implicitly* by bluesky)

            Minisky intentionally does **not** infer or inject that aircraft
            target. Write the complete deferred command instead:
            `HX2 AT WPT10 STACK HX2 ALT 95` (or the equivalent command-first
            form `HX2 AT WPT10 STACK ALT HX2 95`).

            The same rule applies to nested conditional commands. A BlueSky
            scenario line such as
            `HX2 AT WPT9D STACK ATSPD 0 HX2 ALT 100` should be
            `HX2 AT WPT9D STACK HX2 ATSPD 0 HX2 ALT 100` in minisky:
            the deferred `ATSPD` command must itself be complete.

        See also:

        - [BlueSky commit that introduced `AT ... DO/STACK`](https://github.com/TUDelft-CNS-ATM/bluesky/commit/90dc5c4f1dc04a86a617079a38b85aa3b42fc796)
        - [Metropolis 2](https://github.com/TUDelft-CNS-ATM/bluesky/discussions/266)
        - `scenario/M2AP3.scn` and `scenario/LNAV_VNAV/LNAV-VNAV-TwoRoutesFlights.scn` upstream
        """
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        if not command_text.strip():
            return Err("AT DO/STACK requires a command")
        target.route.wpstack[target.index].append(command_text)
        return Ok("")

    @command(name="DIRECT", aliases=("DIRECTTO", "DIRTO", "DCT"))
    def direct(self, acidx: AcId, wpname: Keyword) -> Result[str, str]:
        """Fly an aircraft directly to a waypoint in its route."""
        if isinstance(found := _route_waypoint_index(self.traffic, acidx, wpname), Err):
            return found
        direct(self.traffic, acidx, wpname)
        return Ok("")

    @command(name="RTA")
    def set_rta(self, acidx: AcId, wpname: Keyword, time: TimeS) -> Result[str, str]:
        """Set a required time of arrival at a route waypoint."""
        if isinstance(found := _route_waypoint_index(self.traffic, acidx, wpname), Err):
            return found
        set_rta(self.traffic, acidx, wpname, time)
        return Ok("")

    @command(name="LISTRTE")
    def listrte(self, acidx: AcId, ipagetxt: Keyword = "0") -> Result[None, str]:
        """Show an aircraft route in the console."""
        return listrte(self.traffic, acidx, ipagetxt)

    @command(name="DELRTE", aliases=("DELROUTE",))
    def delete_only_route(self) -> Result[str, str]:
        """Delete the route when a single aircraft exists."""
        if self.traffic.ntraf == 0:
            return Err("No aircraft in simulation")
        if self.traffic.ntraf > 1:
            return Err("Specify callsign of aircraft to delete route of")
        return delrte(self.traffic, 0)

    @command(name="DELRTE")
    def delete_route(self, acidx: AcId) -> Result[str, str]:
        """Delete an aircraft route."""
        return delrte(self.traffic, acidx)

    @command(name="DELWPT", aliases=("DELWP",))
    def delete_route_via_waypoint_wildcard(
        self, acidx: AcId, _all: Literal["*"]
    ) -> Result[str, str]:
        """Delete the complete route using BlueSky's DELWPT callsign,* form."""
        return delrte(self.traffic, acidx)

    @command(name="DELWPT")
    def delwpt(self, acidx: AcId, wpname: Keyword) -> Result[str, str]:
        """Delete a waypoint from an aircraft route."""
        return delwpt(self.traffic, acidx, wpname)
