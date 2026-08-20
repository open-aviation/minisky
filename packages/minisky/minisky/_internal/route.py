"""Route storage, flight-plan calculations, and route commands.

Contains the per-aircraft [`Route`][.Route] class (the flight plan: an ordered
list of waypoints with optional altitude, airspeed, RTA and turn constraints)
plus the route-editing functions used by [`RouteCommands`][.RouteCommands],
the runtime-owned command component for ADDWPT, ADDWPTMODE, AFTER, BEFORE,
AT, DIRECT, RTA, LISTRTE, DELRTE and DELWPT.

The route itself is passive data with flight-plan pre-calculations
([`calcfp`][.Route.calcfp]); the actual guidance along the route is performed by
[`Autopilot`][minisky.Autopilot], which pulls waypoint data
into the vectorized [`ActiveWaypoint`][minisky.ActiveWaypoint]
arrays via [`getnextwp`][.Route.getnextwp] and [`getnextturnwp`][.Route.getnextturnwp].
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, IntEnum, auto
from typing import TYPE_CHECKING, Annotated, Literal, NamedTuple, TypeAlias

import numpy as np
from annotated_types import IsFinite

import minisky.geo as geo  # noqa: PLR0402
from minisky import quantities as q
from minisky._internal.command import (
    AcId,
    ArgumentIssue,
    CmdParser,
    CommandCursor,
    CommandField,
    CommandParseContext,
    CoordinateWaypoint,
    DistanceM,
    Keyword,
    NamedWaypoint,
    Omitted,
    ParseResult,
    RunwayPosition,
    SimTimeS,
    Spanned,
    Text,
    Wpt,
    command,
    parse_pressure_altitude_value,
    parse_resolved_position,
    parse_selected_airspeed_value,
)
from minisky._internal.convert import degto180
from minisky._internal.position import (
    AirportPosition,
    NavaidPosition,
    ResolvedRunwayPosition,
    txt2pos,
)
from minisky._internal.result import Err, Ok, Result
from minisky._internal.traffic_arrays import OptionalArray
from minisky.aero import cas2tas, g0, mach2tas, vcas2tas
from minisky.types import (
    AircraftCallsign,
    AircraftIndex,
    CasMps,
    Ge0,
    Gt0,
    LatLonDegrees,
    Mach,
    OptionalAirspeedKind,
    RouteWaypointIndex,
    RunwayIdentifier,
    StdPressureAltM,
)

if TYPE_CHECKING:
    from minisky._internal.traffic import Traffic


# TODO(abraham): bluesky-era WaypointType mixes source (LATLON/NAV), route role (ORIGIN/DESTINATION),
# generated status (CALCULATED), and physical semantics (RUNWAY). split it
class WaypointType(IntEnum):
    LATLON = 0
    """Latitude/Longitude Waypoint"""
    NAV = 1
    """VOR/NAV database waypoint"""
    ORIGIN = 2
    """Origin airport"""
    DESTINATION = 3
    """Destination airport"""
    CALCULATED = 4
    """Calculated waypoint (T/C, T/D, A/C)"""
    RUNWAY = 5
    """Runway: Copy name and positions"""


class TurnRadius(NamedTuple):
    radius: q.TurnRadiusM[float]


class TurnHeadingRate(NamedTuple):
    heading_rate: q.TurnRateDegPerS[float]


TurnGeometry: TypeAlias = TurnRadius | TurnHeadingRate
RouteWaypointName: TypeAlias = str
"""Name assigned to a waypoint within one aircraft route."""


class TurnParameters(NamedTuple):
    geometry: TurnGeometry | None = None
    cas: q.CalibratedAirspeedMps[float] | None = None


class NextTurn(NamedTuple):
    latitude: q.LatitudeDeg[float]
    longitude: q.LongitudeDeg[float]
    turn: TurnParameters
    waypoint_index: RouteWaypointIndex


class AltitudeTarget(NamedTuple):
    altitude: q.PressureAltitudeM[float]
    distance: q.DistanceM[float]
    """Distance from the active waypoint to the constraint."""


class RtaTarget(NamedTuple):
    time: q.SimulationTimeS[float]
    """Required arrival time in simulation time."""
    distance: q.DistanceM[float]
    """Distance from the active waypoint to the RTA waypoint."""


class RouteProfile(NamedTuple):
    altitude: AltitudeTarget | None = None
    rta: RtaTarget | None = None


class Route:
    """Flight plan (route) of a single aircraft: basic FMS functionality.

    A Route is an ordered list of waypoints, each with an optional altitude
    constraint, airspeed constraint, required time of arrival (RTA), turn
    specification (fly-by/fly-over/fly-turn with radius, [`CAS` in m/s][minisky.types.CasMps]
    or heading rate) and stack commands to execute when the waypoint is passed.

    Waypoints from the navigation database are resolved to the entry
    closest to the given lat/lon. For plain lat/lon waypoints the aircraft
    callsign is used as waypoint name, with a number appended.

    Created by: Jacco M. Hoekstra
    """

    def __init__(self, traffic: Traffic, callsign: AircraftCallsign) -> None:
        # NOTE(abraham): does Route really need the entire Traffic object?
        self.traffic = traffic
        self.navigation = traffic.navigation
        self.callsign = callsign

        # TODO(abraham): instead of forcefully adopting SoA maybe we should just use Waypoint record
        self.wpname: list[RouteWaypointName] = []
        self.wptype: list[WaypointType] = []
        self.wplat: list[q.LatitudeDeg[float]] = []  # pyright: ignore[reportGeneralTypeIssues]
        self.wplon: list[q.LongitudeDeg[float]] = []  # pyright: ignore[reportGeneralTypeIssues]
        self.wpalt: list[q.PressureAltitudeM[float] | None] = []  # pyright: ignore[reportGeneralTypeIssues]
        self.wpairspeed: list[CasMps | Mach | None] = []
        self.wprta: list[q.SimulationTimeS[float] | None] = []  # pyright: ignore[reportGeneralTypeIssues]
        """Optional required time of arrival at each waypoint, in simulation time."""
        self.wpflyby: list[bool] = []
        """Whether each waypoint is fly-by rather than fly-over."""
        self.wpstack: list[list[str]] = []
        """Stack commands executed when each waypoint is passed."""

        self.wpturn: list[TurnParameters | None] = []
        """Optional explicit fly-turn parameters for each waypoint."""

        self.iactwp: RouteWaypointIndex | None = None
        """Index of the active waypoint, or `None` when no waypoint is active."""

        # TODO(abraham): replace swflyby/swflyturn plus per-waypoint flags with one
        # tagged transition mode; the booleans permit contradictory combinations.
        self.swflyby = True
        """Default fly-by mode for newly added waypoints."""
        self.swflyturn = False
        """Default explicit fly-turn mode for newly added waypoints."""

        self.bank: q.BankAngleDeg[float] = 25.0  # pyright: ignore[reportGeneralTypeIssues]
        self.turn = TurnParameters()
        """Fly-turn parameters applied to newly added waypoints while fly-turn mode is active."""

        self.flag_landed_runway = False
        """Whether the aircraft has touched down and should keep runway heading."""

        self.wpdirfrom: list[q.BearingDeg[float]] = []  # pyright: ignore[reportGeneralTypeIssues]
        """Outbound bearing from each waypoint."""
        self.wpdirto: list[q.BearingDeg[float]] = []  # pyright: ignore[reportGeneralTypeIssues]
        """Inbound bearing to each waypoint."""
        self.wpdistto: list[q.DistanceM[float]] = []  # pyright: ignore[reportGeneralTypeIssues]
        """Distance of the route leg ending at each waypoint."""
        self.wpprofile: list[RouteProfile] = []
        """Precomputed altitude and RTA guidance targets for each waypoint."""

    def insert_wpt_data(
        self,
        wpidx: RouteWaypointIndex,
        wpname: RouteWaypointName,
        wplat: q.LatitudeDeg[float],
        wplon: q.LongitudeDeg[float],
        wptype: WaypointType,
        wpalt: q.PressureAltitudeM[float] | None,
        wpairspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None,
    ) -> None:
        """Insert a new waypoint record at a given index in the route.

        All per-waypoint lists are updated consistently; the current default
        fly-by/fly-turn mode and turn parameters of the route are applied to
        the new waypoint, and no RTA is set.
        """

        self.wpname.insert(wpidx, wpname)
        self.wplat.insert(wpidx, wplat)
        self.wplon.insert(wpidx, wplon)
        self.wpalt.insert(wpidx, wpalt)
        self.wpairspeed.insert(wpidx, wpairspeed)
        self.wptype.insert(wpidx, wptype)
        self.wpflyby.insert(wpidx, self.swflyby)
        self.wpturn.insert(wpidx, self.turn if self.swflyturn else None)
        self.wprta.insert(wpidx, None)
        self.wpstack.insert(wpidx, [])

    def add_waypoint(
        self,
        iac: AircraftIndex,
        name: str,
        wptype: WaypointType,
        lat: q.LatitudeDeg[float],
        lon: q.LongitudeDeg[float],
        alt: q.PressureAltitudeM[float] | None = None,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None = None,
        afterwp: RouteWaypointName = "",
        beforewp: RouteWaypointName = "",
    ) -> RouteWaypointIndex | None:
        """Add a waypoint to the route and update the flight plan.

        Handles all waypoint types: origin/destination airports (placed at
        the start/end of the route, overwriting an existing orig/dest),
        navigation-database waypoints (resolved closest to the given
        position), runways and plain lat/lon waypoints. The insertion point
        can be steered with afterwp/beforewp; by default waypoints are
        appended just before the destination. Afterwards the flight-plan
        tables are recalculated ([`calcfp`][..calcfp]) and, when a waypoint is active,
        the guidance towards it is refreshed. Returns `None` when the waypoint
        cannot be resolved or inserted.
        """

        n_wpt = len(self.wplat)

        name = name.upper().strip()
        wplat = (lat + 90.0) % 180.0 - 90.0
        wplon = (lon + 180.0) % 360.0 - 180.0

        wpok = True  # switch for waypoint check

        # Check if name already exists, if so add integer 01, 02, 03 etc.
        wprtename = get_available_name(self.wpname, name, self.traffic.callsign)
        if wptype in {WaypointType.ORIGIN, WaypointType.DESTINATION}:
            orig = wptype == WaypointType.ORIGIN
            existing_idx = 0 if orig else n_wpt - 1
            suffix = "ORIG" if orig else "DEST"

            if (
                name != self.traffic.callsign[iac] + suffix
                and (i := self.navigation.getaptidx(name)) is not None
            ):  # published identifier
                wplat = self.navigation.aptlat[i]
                wplon = self.navigation.aptlon[i]

            if not orig and alt is None:
                # TODO(abraham): #22 replace this zero pressure-altitude endpoint
                # with airport MSL elevation through an explicit reference conversion.
                alt = 0.0

            # Overwrite existing origin/dest
            if n_wpt > 0 and self.wptype[existing_idx] == wptype:
                wpidx = existing_idx
                self.wpname[wpidx] = wprtename
                self.wplat[wpidx] = wplat
                self.wplon[wpidx] = wplon
                self.wpalt[wpidx] = alt
                self.wpairspeed[wpidx] = airspeed
                self.wptype[wpidx] = wptype
                self.wpflyby[wpidx] = self.swflyby
                self.wpturn[wpidx] = self.turn if self.swflyturn else None
                self.wprta[wpidx] = None
                self.wpstack[wpidx] = []

            # Or add before first waypoint/append to end
            else:
                wpidx = 0 if orig else len(self.wplat)
                self.insert_wpt_data(wpidx, wprtename, wplat, wplon, wptype, alt, airspeed)

                n_wpt += 1
                if orig and (active_idx := self.iactwp) is not None:
                    self.iactwp = active_idx + 1
                elif not orig and self.iactwp is None and n_wpt == 1:
                    # When only waypoint: adjust pointer to point to destination
                    self.iactwp = 0

            idx = 0 if orig else n_wpt - 1

        else:
            if wptype == WaypointType.LATLON:
                newname = get_available_name(self.wpname, name, self.traffic.callsign, 3)

            else:  # so wptypewpnav
                newname = wprtename

                if wptype != WaypointType.RUNWAY:
                    if (i := self.navigation.getwpidx(name, LatLonDegrees(lat, lon))) is not None:
                        wplat = self.navigation.wplat[i]
                        wplon = self.navigation.wplon[i]
                    elif (i := self.navigation.getaptidx(name)) is not None:
                        wplat = self.navigation.aptlat[i]
                        wplon = self.navigation.aptlon[i]
                    else:
                        wpok = False

            aftwp = afterwp.upper().strip()
            bfwp = beforewp.upper().strip()

            if wpok:
                if (afterwp and self.wpname.count(aftwp) > 0) or (
                    beforewp and self.wpname.count(bfwp) > 0
                ):
                    wpidx = self.wpname.index(aftwp) + 1 if afterwp else self.wpname.index(bfwp)
                else:
                    # Append, just before dest if there is a dest
                    wpidx = (
                        n_wpt - 1
                        if n_wpt > 0 and self.wptype[-1] == WaypointType.DESTINATION
                        else n_wpt
                    )

                self.insert_wpt_data(wpidx, newname, wplat, wplon, wptype, alt, airspeed)
                if (active_idx := self.iactwp) is not None and active_idx >= wpidx:
                    self.iactwp = active_idx + 1

                idx = wpidx
                n_wpt += 1

            else:
                idx = None
                if len(self.wplat) == 1:
                    self.iactwp = 0

        if idx is not None:
            next_qdr = self.getnextqdr()
            if next_qdr is None:
                self.traffic.actwp.next_qdr.clear(iac)
            else:
                self.traffic.actwp.next_qdr.set(iac, next_qdr)
            self.traffic.actwp.swlastwp[iac] = self.iactwp == n_wpt - 1

        if wptype != WaypointType.CALCULATED:
            self.calcfp()

        if wpok and (active_idx := self.iactwp) is not None and active_idx < n_wpt:
            direct(self.traffic, iac, self.wpname[active_idx])

        return idx

    def getnextturnwp(self) -> NextTurn | None:
        """Return the next fly-turn waypoint at or after the active waypoint."""
        start_idx = 0 if self.iactwp is None else self.iactwp
        for waypoint_index in range(start_idx, len(self.wpturn)):
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
        """Active waypoint position."""
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None
        """Optional [`CAS` in m/s][minisky.types.CasMps] or [`Mach`][minisky.types.Mach] constraint."""
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
        active_idx = self.iactwp
        assert active_idx is not None

        # TODO(abraham): Route really should not know how to perform a fixed-wing landing
        # don't synthesise user command strings. return a typed landing / route-complete transition?
        # let vehicle policy + schedule handle it?
        if self.flag_landed_runway:
            # when landing, LNAV is switched off
            lnavon = False

            # no further waypoint; the aircraft only needs a fixed runway heading
            name = self.wpname[active_idx]

            # Change RW06,RWY18C,RWY24001 to resp. 06,18C,24
            rwykey: RunwayIdentifier
            if "RWY" in name:
                rwykey = name[8:10]
                if len(name) > 10 and not name[10].isdigit():
                    rwykey = name[8:11]
            else:
                rwykey = name[7:9]
                if len(name) > 9 and not name[9].isdigit():
                    rwykey = name[7:10]

            wphdg = self.navigation.rwythresholds[name[:4]][rwykey][2]

            self.traffic.stack_command("HDG " + str(self.callsign) + " " + str(wphdg))

            self.traffic.stack_command(
                "DELAY " + "10 " + "SPD " + str(self.callsign) + " " + "10KT[CAS]"
            )

            self.traffic.stack_command("DELAY " + "42 " + "DEL " + str(self.callsign))

            return self.WaypointTransition(
                LatLonDegrees(self.wplat[active_idx], self.wplon[active_idx]),
                self.wpairspeed[active_idx],
                self.wpprofile[active_idx],
                lnavon,
                self.wpflyby[active_idx],
                self.wpturn[active_idx],
                None,
                active_idx == n_wpt - 1,
            )

        # Switch LNAV off when last waypoint has been passed
        lnavon = active_idx < n_wpt - 1

        if lnavon:
            active_idx += 1
            self.iactwp = active_idx

        # Activate switch to indicate that this is the last waypoint (for lenient passing logic in actwp.Reached function)
        swlastwp = active_idx == n_wpt - 1

        # Endpoint of the leg after the new active waypoint; the autopilot
        # computes the next-leg bearings for all switching aircraft in one
        # vectorised qdrdist call (see wppassingcheck).
        next_leg = self.getnextleg()

        # in case that there is a runway, the aircraft should remain on it
        # instead of deviating to the airport centre
        # When there is a destination: current = runway, next  = Dest
        # Else: current = runway and this is also the last waypoint
        if (
            self.wptype[active_idx] == WaypointType.RUNWAY
            and self.wpname[active_idx] == self.wpname[-1]
        ) or (
            self.wptype[active_idx] == WaypointType.RUNWAY
            and active_idx + 1 < n_wpt
            and self.wptype[active_idx + 1] == WaypointType.DESTINATION
        ):
            self.flag_landed_runway = True

        return self.WaypointTransition(
            LatLonDegrees(self.wplat[active_idx], self.wplon[active_idx]),
            self.wpairspeed[active_idx],
            self.wpprofile[active_idx],
            lnavon,
            self.wpflyby[active_idx],
            self.wpturn[active_idx],
            next_leg,
            swlastwp,
        )

    def runactwpstack(self) -> None:
        """Execute the stack commands stored for the active waypoint.

        Commands are attached to waypoints with the AT ... DO/STACK command
        and are issued when the aircraft passes the waypoint.
        """
        active_idx = self.iactwp
        assert active_idx is not None
        for cmdline in self.wpstack[active_idx]:
            self.traffic.stack_command(cmdline)

    def insertcalcwp(self, i: RouteWaypointIndex, name: RouteWaypointName) -> None:
        """Insert an empty calculated waypoint (T/C, T/D) at location i."""

        self.wpname.insert(i, name)
        self.wplat.insert(i, 0.0)
        self.wplon.insert(i, 0.0)
        self.wpalt.insert(i, None)
        self.wpairspeed.insert(i, None)
        self.wptype.insert(i, WaypointType.CALCULATED)

    def calcfp(self) -> None:
        """Current Flight Plan calculations, which actualize based on flight condition

        This routine prepares data for this by adding a "ruler" along the flight
        plan in the form of distance at wp to next altitude constraint (xtoalt),
        its index ial and the value (toalt). Same logic is used for time constraint.

        Note: No Top of Descent or Top of Climb can inserted here as this depends on
        the airspeed, which might be undefined (often is). Guidance in autpilot.py takes
        care of ToD and ToC logic while flying using current airspeed.

        Recomputes, per waypoint: leg directions [deg] and lengths [m]
        (wpdirfrom, wpdirto, wpdistto), plus typed altitude and RTA guidance
        targets in wpprofile.
        """

        # Direction to waypoint
        n_wpt = len(self.wpname)

        # Create cleared flight plan calculation table
        self.wpdirfrom = n_wpt * [0.0]

        self.wpdirto = n_wpt * [0.0]

        self.wpdistto = n_wpt * [0.0]

        self.wpprofile = [RouteProfile() for _ in range(n_wpt)]

        if n_wpt == 0:
            return

        # LNAV: Calculate leg distances and directions

        for i in range(n_wpt - 1):
            qdr, dist = geo.qdrdist(
                self.wplat[i], self.wplon[i], self.wplat[i + 1], self.wplon[i + 1]
            )
            self.wpdirfrom[i] = float(qdr)
            self.wpdistto[i + 1] = float(dist)

        # Also add "from direction" as to directions so no need to shift for actwpdata
        # direction to will be overwritten in actwpdata in case of a direct to
        # Add current pos to first waypoint as default value for direction to 1st waypoint
        iac = self.traffic.callsign.index(self.callsign)
        qdr, _dist = geo.qdrdist(
            self.traffic.lat[iac], self.traffic.lon[iac], self.wplat[0], self.wplon[0]
        )
        self.wpdirto = [qdr, *self.wpdirfrom[0:-1]]

        if n_wpt > 1:
            # Keep the final outbound bearing aligned with the previous leg so guidance continues straight.
            self.wpdirfrom[-1] = self.wpdirfrom[-2]

        # VNAV: calc next altitude constraint and distance to it
        altitude_target: AltitudeTarget | None = None
        for i in range(n_wpt - 1, -1, -1):
            # waypoint with altitude constraint (dest of all specified)
            if self.wptype[i] == WaypointType.DESTINATION:
                altitude_target = AltitudeTarget(0.0, 0.0)
            elif (altitude := self.wpalt[i]) is not None:
                altitude_target = AltitudeTarget(altitude, 0.0)
            # waypoint with no altitude constraint: keep counting
            elif altitude_target is not None and i != n_wpt - 1:
                altitude_target = AltitudeTarget(
                    altitude_target.altitude,
                    altitude_target.distance + self.wpdistto[i + 1],
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
                # No airspeed or RTA constraint: add to distance
                if (airspeed := self.wpairspeed[i]) is None:
                    rta_target = RtaTarget(
                        rta_target.time,
                        rta_target.distance + self.wpdistto[i + 1],
                    )
                else:
                    # airspeed constraint on this leg: shift RTA time to account for this
                    # altitude unknown; use the next altitude constraint for this waypoint
                    # or default to 10000 ft when no altitude constraint is present
                    altitude_target = self.wpprofile[i].altitude
                    altitude = (
                        altitude_target.altitude
                        if altitude_target is not None and altitude_target.altitude > 0.0
                        else q.ft_to_m(10000.0)
                    )
                    legtas = (
                        cas2tas(airspeed.value, altitude)
                        if isinstance(airspeed, CasMps)
                        else mach2tas(airspeed.value, altitude)
                    )
                    # TODO(abraham): account for wind at this position by adding wind vectors to waypoints?

                    # This fixed-airspeed leg is excluded from RTA distance, so subtract its
                    # travel time from the target time instead.
                    legtime = self.wpdistto[i + 1] / legtas
                    rta_target = RtaTarget(rta_target.time - legtime, rta_target.distance)
            else:
                rta_target = None

            self.wpprofile[i] = self.wpprofile[i]._replace(rta=rta_target)

    def findact(self, i: AircraftIndex) -> RouteWaypointIndex | None:
        """Find the best default active waypoint for an aircraft.

        Called when LNAV is (re-)engaged. Selects the waypoint closest to
        the aircraft, without walking back to earlier waypoints, and skips
        to the next waypoint when the closest one cannot be reached with
        the required heading change (turn time exceeds straight flight
        time).
        """

        n_wpt = len(self.wpname)

        if n_wpt <= 0:
            return None

        elif n_wpt == 1:
            return 0

        wplat = np.array(self.wplat)
        wplon = np.array(self.wplon)
        dy = wplat - self.traffic.lat[i]
        dx = (wplon - self.traffic.lon[i]) * self.traffic.coslat[i]
        dist2 = dx * dx + dy * dy
        # Note: the max() prevents walking back, even in cases when this might be apropriate,
        # such as when previous waypoints have been deleted

        iwpnear = max(0 if self.iactwp is None else self.iactwp, np.argmin(dist2))

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
            time_straight = (
                math.sqrt(dist2[iwpnear]) * q.nmi_to_m(60.0) / max(0.01, self.traffic.tas[i])
            )

            if time_turn > time_straight:
                iwpnear += 1

        return int(iwpnear)

    def getnextleg(self) -> LatLonDegrees | None:
        """Return the endpoint of the leg after the active waypoint."""
        if (active_idx := self.iactwp) is not None and active_idx < len(self.wpname) - 1:
            return LatLonDegrees(self.wplat[active_idx + 1], self.wplon[active_idx + 1])
        return None

    def getnextqdr(self) -> q.BearingDeg[float] | None:
        """Return the bearing of the leg after the active waypoint."""
        if (active_idx := self.iactwp) is None:
            return None
        if (next_leg := self.getnextleg()) is None:
            return None
        nextqdr, _dist = geo.qdrdist(
            self.wplat[active_idx],
            self.wplon[active_idx],
            next_leg.lat,
            next_leg.lon,
        )
        return float(nextqdr)


def get_available_name(
    data: list[RouteWaypointName],
    name_: str,
    callsigns: list[AircraftCallsign],
    len_: int = 2,
) -> RouteWaypointName:
    """Make a waypoint name unique by appending a zero-padded number.

    Checks if the name already exists in the given list (or matches an
    aircraft callsign); if so, appends/increments an integer suffix
    (01, 02, 03, ...) until the name is unique.

    Args:
        data: Names that are already in use.
        name_: Requested base name.
        callsigns: Aircraft callsigns that the generated name must also avoid.
        len_: Width of the zero-padded numeric suffix.
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
    CAS = auto()
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
    "TURNSPD": TurnParameter.CAS,
    "TURNSPEED": TurnParameter.CAS,
    "TURNHDG": TurnParameter.HEADING_RATE,
    "TURNHDGR": TurnParameter.HEADING_RATE,
    "TURNHDGRATE": TurnParameter.HEADING_RATE,
}
# bluesky also accepts TURNBANK/TURNPHI. minisky intentionally rejects them:
# its route model has no per-waypoint bank state, so accepting the syntax would
# silently discard behavior. add them only when that state is represented.


WaypointModeArg = Annotated[
    WaypointMode,
    CmdParser.choices(_WAYPOINT_MODES),
]
TurnParameterArg = Annotated[
    TurnParameter,
    CmdParser.choices(_TURN_PARAMETERS),
]
TurnRadiusMArg = Gt0[DistanceM]
TurnHeadingRateArg = q.TurnRateDegPerS[IsFinite[Gt0[float]]]


def _parse_runway(
    context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[RunwayPosition]:
    if isinstance(result := parse_resolved_position(context, cursor), Err):
        return result
    parsed = result.ok()
    if not isinstance(parsed.value, RunwayPosition):
        actual = cursor.text[parsed.span.start : parsed.span.end]
        return Err(ArgumentIssue.expected("a runway", actual, parsed.span))
    return Ok(Spanned(parsed.value, parsed.span))


RunwayArg = Annotated[
    RunwayPosition,
    CmdParser(
        _parse_runway,
        CommandField(name="runway", examples=("EHAM/RW06", "LFPG/RWY23")),
    ),
]


def _waypoint_mode_status(traffic: Traffic, acidx: AircraftIndex) -> Result[str, str]:
    acrte = traffic.ap.route[acidx]
    if acrte.swflyturn:
        mode = "FLYTURN"
    elif acrte.swflyby:
        mode = "FLYBY"
    else:
        mode = "FLYOVER"
    return Ok(f"Current ADDWPT mode is {mode}.")


def _set_waypoint_mode(
    traffic: Traffic, acidx: AircraftIndex, mode: WaypointMode
) -> Result[str, str]:
    """Set fly-by/fly-over/fly-turn behavior for newly added route waypoints."""
    acrte = traffic.ap.route[acidx]
    acrte.swflyby = mode is WaypointMode.FLYBY
    acrte.swflyturn = mode is WaypointMode.FLYTURN
    return Ok("")


def _set_turn_radius(
    traffic: Traffic, acidx: AircraftIndex, value: TurnRadiusMArg | None
) -> Result[str, str]:
    acrte = traffic.ap.route[acidx]
    geometry = acrte.turn.geometry
    if value is not None:
        geometry = TurnRadius(value)
    elif isinstance(geometry, TurnRadius):
        geometry = None
    acrte.turn = acrte.turn._replace(geometry=geometry)
    acrte.swflyby = False
    acrte.swflyturn = True
    return Ok("")


def _set_turn_cas(traffic: Traffic, acidx: AircraftIndex, value: CasMps | None) -> Result[str, str]:
    acrte = traffic.ap.route[acidx]
    acrte.turn = acrte.turn._replace(cas=None if value is None else value.value)
    acrte.swflyby = False
    acrte.swflyturn = True
    return Ok("")


def _set_turn_heading_rate(
    traffic: Traffic, acidx: AircraftIndex, value: TurnHeadingRateArg | None
) -> Result[str, str]:
    acrte = traffic.ap.route[acidx]
    geometry = acrte.turn.geometry
    if value is not None:
        geometry = TurnHeadingRate(value)
    elif isinstance(geometry, TurnHeadingRate):
        geometry = None
    acrte.turn = acrte.turn._replace(geometry=geometry)
    acrte.swflyby = False
    acrte.swflyturn = True
    return Ok("")


def _clear_turn_parameter(
    traffic: Traffic, acidx: AircraftIndex, parameter: TurnParameter
) -> Result[str, str]:
    if parameter is TurnParameter.RADIUS:
        return _set_turn_radius(traffic, acidx, None)
    if parameter is TurnParameter.CAS:
        return _set_turn_cas(traffic, acidx, None)
    return _set_turn_heading_rate(traffic, acidx, None)


def _add_takeoff_waypoint(
    traffic: Traffic, acidx: AircraftIndex, runway: RunwayPosition | None = None
) -> Result[str, str]:
    callsign = traffic.callsign[acidx]
    acrte = traffic.ap.route[acidx]
    rwyrteidx = next((i for i, name in enumerate(acrte.wpname) if "/" in name), None)

    if runway is not None:
        rwylat = runway.coordinates.lat
        rwylon = runway.coordinates.lon
        rwyhdg = runway.runway_heading
    elif rwyrteidx is not None and rwyrteidx > 0:
        rwylat = acrte.wplat[rwyrteidx]
        rwylon = acrte.wplon[rwyrteidx]
        aptidx = traffic.navigation.getapinear(rwylat, rwylon)
        aptid = traffic.navigation.aptid[aptidx]
        rwyname = acrte.wpname[rwyrteidx].split("/")[1]
        rwyid = rwyname.replace("RWY", "").replace("RW", "")
        rwyhdg = traffic.navigation.rwythresholds[aptid][rwyid][2]
    else:
        rwylat = traffic.lat[acidx]
        rwylon = traffic.lon[acidx]
        rwyhdg = traffic.trk[acidx]

    # TODO(abraham): a TAKEOFF waypoint exactly 2 NM down the runway is fixed-wing policy
    # problematic for VTOL.
    lat, lon = geo.qdrpos(rwylat, rwylon, rwyhdg, q.nmi_to_m(2.0))
    afterwp = ""
    if rwyrteidx is not None and rwyrteidx > 0:
        afterwp = acrte.wpname[rwyrteidx]
    elif acrte.wptype and acrte.wptype[0] == WaypointType.ORIGIN:
        afterwp = acrte.wpname[0]

    name = f"T/O-{callsign}"
    if (
        acrte.add_waypoint(acidx, name, WaypointType.LATLON, lat, lon, None, None, afterwp, "")
        is None
    ):
        return Err(f"Waypoint {name} not added.")
    acrte.calcfp()

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
    acidx: AircraftIndex,
    waypoint: Wpt,
    altitude: StdPressureAltM[IsFinite[float]] | None = None,
    airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None = None,
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
        if acrte.wptype[-1] != WaypointType.DESTINATION or n_wpt == 1:
            reflat = acrte.wplat[-1]
            reflon = acrte.wplon[-1]
        else:
            reflat = acrte.wplat[-2]
            reflon = acrte.wplon[-2]

    alt = None if altitude is None else altitude.value
    afterwp = ""
    beforewp = ""

    match waypoint:
        case CoordinateWaypoint(coordinates):
            name = callsign
            lat = coordinates.lat
            lon = coordinates.lon
            wptype = WaypointType.LATLON
        case NamedWaypoint(name):
            takeoffwpt = name.replace("-", "") == "TAKEOFF"
            if takeoffwpt:
                if altitude is not None or airspeed is not None or insertion is not None:
                    return Err("TAKEOFF does not accept waypoint constraints")
                return _add_takeoff_waypoint(traffic, acidx)

            match txt2pos(name, reflat, reflon, traffic.navigation, traffic):
                case Ok(posobj):
                    lat = posobj.lat
                    lon = posobj.lon
                    match posobj:
                        case NavaidPosition() | AirportPosition():
                            wptype = WaypointType.NAV
                        case ResolvedRunwayPosition():
                            wptype = WaypointType.RUNWAY
                        case _:
                            name = callsign
                            wptype = WaypointType.LATLON
                case Err():
                    return Err("Waypoint " + name + " not found.")

    match insertion:
        case InsertAfter(anchor):
            afterwp = anchor
        case InsertBefore(anchor):
            beforewp = anchor
        case None:
            pass

    if acrte.add_waypoint(acidx, name, wptype, lat, lon, alt, airspeed, afterwp, beforewp) is None:
        return Err(f"Waypoint {name} not added.")

    acrte.calcfp()

    norig = int(traffic.ap.orig[acidx] != "")  # 1 if orig is present in route
    ndest = int(traffic.ap.dest[acidx] != "")  # 1 if dest is present in route

    # Check whether this is first 'real' waypoint (not orig & dest),
    # And if so, make active
    if len(acrte.wpname) - norig - ndest == 1:  # first waypoint: make active
        direct(traffic, acidx, acrte.wpname[norig])  # 0 if no orig
        traffic.swlnav[acidx] = True

    if afterwp and acrte.wpname.count(afterwp) == 0:
        return Ok("Waypoint " + afterwp + " not found\n" + "waypoint added at end of route")
    else:
        return Ok("")


class AtQuery(Enum):
    """Constraint subset shown by AT query forms."""

    ALL = auto()
    ALTITUDE = auto()
    AIRSPEED = auto()
    STACK = auto()


@dataclass(frozen=True, slots=True)
class AtConstraints:
    """A complete altitude/airspeed pair; None means an explicit clear."""

    altitude: q.PressureAltitudeM[float] | None
    airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None


def _parse_at_constraints(
    _context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[AtConstraints]:
    result = cursor.next_value("altitude/airspeed")
    if isinstance(result, Err):
        return result
    token = result.ok()
    if token.value.count("/") != 1:
        return Err(ArgumentIssue.expected("altitude/airspeed", token.value, token.span))
    altitude_text, speed_text = token.value.split("/", maxsplit=1)

    def cleared(value: str) -> bool:
        return len(value) > 1 and set(value) == {"-"}

    altitude: q.PressureAltitudeM[float] | None
    if cleared(altitude_text):
        altitude = None
    else:
        try:
            altitude = parse_pressure_altitude_value(altitude_text).value
        except ValueError:
            return Err(ArgumentIssue.expected("a pressure altitude", altitude_text, token.span))

    airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None
    if cleared(speed_text):
        airspeed = None
    else:
        try:
            airspeed = parse_selected_airspeed_value(speed_text)
        except ValueError:
            return Err(ArgumentIssue.expected("CAS or Mach", speed_text, token.span))

    return Ok(Spanned(AtConstraints(altitude, airspeed), token.span))


AtConstraintsArg = Annotated[
    AtConstraints, CmdParser(_parse_at_constraints, CommandField(name="altitude/airspeed"))
]


@dataclass(frozen=True, slots=True)
class _AtWaypoint:
    route: Route
    index: RouteWaypointIndex


def _route_waypoint_index(
    traffic: Traffic, acidx: AircraftIndex, wpname: RouteWaypointName
) -> Result[RouteWaypointIndex, str]:
    route = traffic.ap.route[acidx]
    try:
        return Ok(route.wpname.index(wpname.upper()))
    except ValueError:
        return Err(f"Waypoint {wpname} not found in the route of {traffic.callsign[acidx]}")


def _at_waypoint(
    traffic: Traffic, acidx: AircraftIndex, atwp: RouteWaypointName
) -> Result[_AtWaypoint, str]:
    if isinstance(index := _route_waypoint_index(traffic, acidx, atwp), Err):
        return index
    return Ok(_AtWaypoint(traffic.ap.route[acidx], index.ok()))


def _finish_at_mutation(
    traffic: Traffic, acidx: AircraftIndex, target: _AtWaypoint
) -> Result[str, str]:
    target.route.calcfp()
    if (active_idx := target.route.iactwp) is not None:
        direct(traffic, acidx, target.route.wpname[active_idx])
    return Ok("")


def _format_at_query(
    acrte: Route, wpidx: RouteWaypointIndex, atwp: RouteWaypointName, query: AtQuery
) -> str:
    show_altitude = query in {AtQuery.ALL, AtQuery.ALTITUDE}
    show_airspeed = query in {AtQuery.ALL, AtQuery.AIRSPEED}
    show_stack = query in {AtQuery.ALL, AtQuery.STACK}
    text = f"{atwp} : "

    if show_altitude:
        altitude = acrte.wpalt[wpidx]
        if altitude is None:
            text += "-----"
        elif altitude > q.ft_to_m(4500.0):
            text += f"FL{round(q.m_to_ft(altitude) / 100.0)}"
        else:
            text += str(round(q.m_to_ft(altitude)))
        if show_airspeed:
            text += "/"

    if show_airspeed:
        airspeed = acrte.wpairspeed[wpidx]
        match airspeed:
            case None:
                text += "---"
            case CasMps(value):
                text += f"{q.mps_to_kt(value):.0f}KT[CAS]"
            case Mach(value):
                text += f"M{value:g}"

    if show_altitude and show_airspeed:
        if acrte.wptype[wpidx] == WaypointType.ORIGIN:
            text += "[orig]"
        elif acrte.wptype[wpidx] == WaypointType.DESTINATION:
            text += "[dest]"

    if show_stack and acrte.wpstack[wpidx]:
        stacked = "\n".join(acrte.wpstack[wpidx])
        text += f"\nStack:\n{stacked}\n"
    return text


def direct(traffic: Traffic, acidx: AircraftIndex, wpname: RouteWaypointName) -> bool:
    """Go direct to a specified waypoint in the route.

    Implements the DIRECT stack command: `DIRECT acid wpname`. Makes the
    given waypoint the active waypoint, copies its data (position, fly-by/
    fly-turn settings, next-turn data) into the active-waypoint arrays,
    recalculates the flight plan and the VNAV profile, sets the next-leg
    airspeed from any airspeed constraint, computes the turn distance for the
    new leg, and engages LNAV.
    """
    # TODO(abraham): this is a giant cross-object transaction, mutating Route,
    # ActiveWaypoint, Autopilot and Traffic. make things pure.
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
    if isinstance(geometry, TurnRadius):
        traffic.actwp.turnrad.set(acidx, geometry.radius)
    else:
        traffic.actwp.turnrad.clear(acidx)

    if turn is None:
        traffic.actwp.turn_cas.clear(acidx)
    else:
        traffic.actwp.turn_cas.set(acidx, traffic.cas[acidx] if turn.cas is None else turn.cas)

    if isinstance(geometry, TurnHeadingRate):
        traffic.actwp.turnhdgr.set(acidx, geometry.heading_rate)
    else:
        traffic.actwp.turnhdgr.clear(acidx)

    next_turn = acrte.getnextturnwp()
    if next_turn is None:
        traffic.actwp.nextturnlat.clear(acidx)
        traffic.actwp.nextturnlon.clear(acidx)
        traffic.actwp.next_turn_cas.clear(acidx)
        traffic.actwp.nextturnrad.clear(acidx)
        traffic.actwp.nextturnhdgr.clear(acidx)
        traffic.actwp.nextturnidx.clear(acidx)
    else:
        traffic.actwp.nextturnlat.set(acidx, next_turn.latitude)
        traffic.actwp.nextturnlon.set(acidx, next_turn.longitude)
        if next_turn.turn.cas is None:
            traffic.actwp.next_turn_cas.clear(acidx)
        else:
            traffic.actwp.next_turn_cas.set(acidx, next_turn.turn.cas)
        next_geometry = next_turn.turn.geometry
        if isinstance(next_geometry, TurnRadius):
            traffic.actwp.nextturnrad.set(acidx, next_geometry.radius)
        else:
            traffic.actwp.nextturnrad.clear(acidx)
        if isinstance(next_geometry, TurnHeadingRate):
            traffic.actwp.nextturnhdgr.set(acidx, next_geometry.heading_rate)
        else:
            traffic.actwp.nextturnhdgr.clear(acidx)
        traffic.actwp.nextturnidx.set(acidx, next_turn.waypoint_index)

    acrte.calcfp()

    profile = acrte.wpprofile[wpidx]
    if profile.altitude is None:
        traffic.actwp.nextaltco.clear(acidx)
        traffic.actwp.xtoalt.clear(acidx)
    else:
        traffic.actwp.nextaltco.set(acidx, profile.altitude.altitude)
        traffic.actwp.xtoalt.set(acidx, profile.altitude.distance)

    if profile.rta is None:
        traffic.actwp.torta.clear(acidx)
        traffic.actwp.xtorta.clear(acidx)
    else:
        traffic.actwp.torta.set(acidx, profile.rta.time)
        traffic.actwp.xtorta.set(acidx, profile.rta.distance)

    if (airspeed := acrte.wpairspeed[wpidx]) is None:
        traffic.actwp.next_airspeed.kind[acidx] = OptionalAirspeedKind.NONE
    else:
        traffic.actwp.next_airspeed.values[acidx] = airspeed.value
        traffic.actwp.next_airspeed.kind[acidx] = (
            OptionalAirspeedKind.CAS if isinstance(airspeed, CasMps) else OptionalAirspeedKind.MACH
        )

    qdr_result, dist_result = geo.qdrdist(
        traffic.lat[acidx],
        traffic.lon[acidx],
        traffic.actwp.lat[acidx],
        traffic.actwp.lon[acidx],
    )
    qdr_ = float(np.asarray(qdr_result).item())
    leg_distance = float(np.asarray(dist_result).item())

    traffic.actwp.curlegdir.set(acidx, qdr_)
    traffic.actwp.curleglen.set(acidx, leg_distance)
    traffic.ap.qdr2wp.set(acidx, qdr_ % 360.0)
    traffic.ap.dist2wp.set(acidx, leg_distance)

    next_qdr = acrte.getnextqdr()
    turn_geometry = traffic.actwp.calcturn(
        traffic.tas[acidx],
        traffic.ap.bankdef[acidx],
        qdr_,
        qdr_ if next_qdr is None else next_qdr,
        OptionalArray(
            traffic.actwp.turnrad.values[acidx : acidx + 1],
            traffic.actwp.turnrad.present[acidx : acidx + 1],
        ),
        OptionalArray(
            traffic.actwp.turnhdgr.values[acidx : acidx + 1],
            traffic.actwp.turnhdgr.present[acidx : acidx + 1],
        ),
        traffic.actwp.flyturn[acidx],
    )
    turn_distance = turn_geometry.distance.item()
    if turn is not None and turn.geometry is None and turn.cas is not None:
        turn_tas = vcas2tas(turn.cas, traffic.alt[acidx])
        turn_distance *= turn_tas * turn_tas / (traffic.tas[acidx] * traffic.tas[acidx])
    traffic.actwp.turndist[acidx] = (
        np.logical_or(traffic.actwp.flyby[acidx], traffic.actwp.flyturn[acidx]) * turn_distance
    )

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
    traffic: Traffic, acidx: AircraftIndex, wpname: RouteWaypointName, time: SimTimeS
) -> bool:  # all arguments of setRTA
    """Set a required time of arrival (RTA) at a route waypoint.

    Implements the RTA stack command: `RTA acid, wpname, time`. The RTA
    is stored with the waypoint and the guidance to the active waypoint is
    recomputed so the autopilot can adjust its airspeed schedule.
    """
    traffic.callsign[acidx]
    acrte = traffic.ap.route[acidx]
    wpidx = acrte.wpname.index(wpname)
    acrte.wprta[wpidx] = time

    # Recompute route and update actwp because of RTA addition
    if (active_idx := acrte.iactwp) is not None:
        direct(traffic, acidx, acrte.wpname[active_idx])

    return True


def listrte(traffic: Traffic, acidx: AircraftIndex, ipagetxt: str = "0") -> Result[None, str]:
    """Show the route of an aircraft in the console, page by page.

    Implements the LISTRTE stack command: `LISTRTE acid, [pagenr]`.
    Each line shows the waypoint name (active waypoint marked with `*`),
    its altitude constraint (ft or FL), airspeed constraint
    ([`CAS` in m/s][minisky.types.CasMps] or [`Mach`][minisky.types.Mach]) and
    type ([orig], [dest], [C] fly-by, [|] fly-over, [U] fly-turn). Seven
    waypoints are shown per page.
    """
    ipage = int(ipagetxt)
    acrte = traffic.ap.route[acidx]

    n_wpt = len(acrte.wpname)

    if n_wpt <= 0:
        return Err("Aircraft has no route.")

    for i in range(ipage * 7, ipage * 7 + 7):
        if 0 <= i < n_wpt:
            if i == acrte.iactwp:
                txt = "*" + acrte.wpname[i] + " : "
            else:
                txt = " " + acrte.wpname[i] + " : "

            altitude = acrte.wpalt[i]
            if altitude is None:
                txt += "-----/"
            elif altitude > q.ft_to_m(4500.0):
                fl = round(q.m_to_ft(altitude) / 100.0)
                txt += "FL" + str(fl) + "/"
            else:
                txt += str(round(q.m_to_ft(altitude))) + "/"

            airspeed = acrte.wpairspeed[i]
            if airspeed is None:
                txt += "---"
            elif isinstance(airspeed, CasMps):
                txt += f"{q.mps_to_kt(airspeed.value):.0f}KT[CAS]"
            else:
                txt += f"M{airspeed.value:g}"

            # Type: orig, dest, C = flyby, | = flyover, U = flyturn
            if acrte.wptype[i] == WaypointType.ORIGIN:
                txt += "[orig]"
            elif acrte.wptype[i] == WaypointType.DESTINATION:
                txt += "[dest]"
            elif acrte.wpturn[i] is not None:
                txt += "[U]"
            elif acrte.wpflyby[i]:
                txt += "[C]"
            else:  # FLYOVER
                txt += "[|]"

            traffic.console.echo(txt)

    return Ok(None)


def delrte(traffic: Traffic, acidx: AircraftIndex) -> Result[str, str]:
    """Delete the complete route (including origin/destination) of an
    aircraft.

    Implements the DELRTE stack command: `DELRTE acid`. The route is
    re-initialized empty and LNAV/VNAV are disengaged.
    """
    callsign = traffic.callsign[acidx]
    acrte = traffic.ap.route[acidx]
    # TODO(abraham): don't reset an existing Route by calling __init__ directly;
    # give route state an explicit clear/reset operation.
    acrte.__init__(traffic, callsign)

    traffic.swlnav[acidx] = False
    traffic.swvnav[acidx] = False
    traffic.swvnavairspeed[acidx] = False
    traffic.actwp.torta.clear(acidx)
    traffic.actwp.xtorta.clear(acidx)

    return Ok("")


def delwpt(traffic: Traffic, acidx: AircraftIndex, wpname: RouteWaypointName) -> Result[str, str]:
    """Delete a single waypoint from the route of an aircraft.

    Implements the DELWPT stack command: `DELWPT acid, wpname`. When the
    deleted waypoint is the active one (and not the last), guidance is
    redirected to the following waypoint. LNAV/VNAV are disengaged when
    the route becomes empty.
    """

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
    del acrte.wpairspeed[wpidx]
    del acrte.wprta[wpidx]
    del acrte.wptype[wpidx]
    del acrte.wpflyby[wpidx]
    del acrte.wpturn[wpidx]
    del acrte.wpstack[wpidx]

    if (active_idx := acrte.iactwp) is not None:
        if active_idx > wpidx:
            active_idx -= 1
        acrte.iactwp = None if n_wpt == 0 else min(active_idx, n_wpt - 1)

    # If no waypoints left, make sure to disable LNAV/VNAV
    if n_wpt == 0 and (acidx or acidx == 0):
        traffic.swlnav[acidx] = False
        traffic.swvnav[acidx] = False
        traffic.swvnavairspeed[acidx] = False

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
    def set_turn_radius(
        self,
        acidx: AcId,
        _parameter: Literal["TURNRAD", "TURNRADIUS"],
        value: TurnRadiusMArg,
    ) -> Result[str, str]:
        """Set the default fly-turn radius using explicit units such as `1NM`."""
        return _set_turn_radius(self.traffic, acidx, value)

    @command(name="ADDWPTMODE")
    def set_turn_cas(
        self,
        acidx: AcId,
        _parameter: Literal["TURNSPD", "TURNSPEED"],
        value: CasMps[IsFinite[Gt0[float]]],
    ) -> Result[str, str]:
        """Set the default fly-turn [`CAS`][minisky.types.CasMps] using an explicit quantity such as `250KT[CAS]`."""
        return _set_turn_cas(self.traffic, acidx, value)

    @command(name="ADDWPTMODE")
    def set_turn_heading_rate(
        self,
        acidx: AcId,
        _parameter: Literal["TURNHDG", "TURNHDGR", "TURNHDGRATE"],
        value: TurnHeadingRateArg,
    ) -> Result[str, str]:
        """Set the default fly-turn heading rate in degrees per second."""
        return _set_turn_heading_rate(self.traffic, acidx, value)

    @command(name="ADDWPTMODE")
    def clear_turn_parameter(
        self, acidx: AcId, parameter: TurnParameterArg, _off: Literal["OFF"]
    ) -> Result[str, str]:
        """Clear a route turn radius, [`CAS`][minisky.types.CasMps], or heading-rate default."""
        return _clear_turn_parameter(self.traffic, acidx, parameter)

    @command(name="ADDWPT")
    def add_waypoint_mode(self, acidx: AcId, mode: WaypointModeArg) -> Result[str, str]:
        """Select FLYBY, FLYOVER, or FLYTURN through the ADDWPT mode form."""
        return _set_waypoint_mode(self.traffic, acidx, mode)

    @command(name="ADDWPT")
    def clear_waypoint_turn_parameter(
        self, acidx: AcId, parameter: TurnParameterArg, _off: Literal["OFF"]
    ) -> Result[str, str]:
        """Clear a turn parameter through the ADDWPT mode form."""
        return _clear_turn_parameter(self.traffic, acidx, parameter)

    @command(name="ADDWPT")
    def add_waypoint_turn_radius(
        self,
        acidx: AcId,
        _parameter: Literal["TURNRAD", "TURNRADIUS"],
        value: TurnRadiusMArg,
    ) -> Result[str, str]:
        """Set a fly-turn radius through ADDWPT."""
        return _set_turn_radius(self.traffic, acidx, value)

    @command(name="ADDWPT")
    def add_waypoint_turn_cas(
        self,
        acidx: AcId,
        _parameter: Literal["TURNSPD", "TURNSPEED"],
        value: CasMps[IsFinite[Gt0[float]]],
    ) -> Result[str, str]:
        """Set fly-turn CAS through ADDWPT using an explicit quantity such as `250KT[CAS]`."""
        return _set_turn_cas(self.traffic, acidx, value)

    @command(name="ADDWPT")
    def add_waypoint_turn_heading_rate(
        self,
        acidx: AcId,
        _parameter: Literal["TURNHDG", "TURNHDGR", "TURNHDGRATE"],
        value: TurnHeadingRateArg,
    ) -> Result[str, str]:
        """Set a fly-turn heading rate through ADDWPT, in degrees per second."""
        return _set_turn_heading_rate(self.traffic, acidx, value)

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
        altitude: StdPressureAltM[IsFinite[float]] | None,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None,
        _after: Omitted,
        before: Keyword,
    ) -> Result[str, str]:
        """Insert a route waypoint before an existing waypoint."""
        return _add_route_waypoint(
            self.traffic,
            acidx,
            waypoint,
            altitude,
            airspeed,
            InsertBefore(before),
        )

    @command(name="ADDWPT")
    def insert_waypoint_after(
        self,
        acidx: AcId,
        waypoint: Wpt,
        altitude: StdPressureAltM[IsFinite[float]] | None,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None,
        after: Keyword,
    ) -> Result[str, str]:
        """Insert a route waypoint after an existing waypoint."""
        return _add_route_waypoint(
            self.traffic,
            acidx,
            waypoint,
            altitude,
            airspeed,
            InsertAfter(after),
        )

    @command(name="ADDWPT", aliases=("WPTYPE",))
    def append_waypoint(
        self,
        acidx: AcId,
        waypoint: Wpt,
        altitude: StdPressureAltM[IsFinite[float]] | None = None,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None = None,
    ) -> Result[str, str]:
        """Append a route waypoint with optional altitude and airspeed constraints."""
        return _add_route_waypoint(self.traffic, acidx, waypoint, altitude, airspeed)

    @command(name="BEFORE")
    def insert_before(
        self,
        acidx: AcId,
        beforewp: Keyword,
        _keyword: Literal["ADDWPT"],
        waypoint: Wpt,
        alt: StdPressureAltM[IsFinite[float]] | None = None,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None = None,
    ) -> Result[str, str]:
        """Insert a waypoint before an existing route waypoint.

        Implements `acid BEFORE waypoint ADDWPT new-waypoint [altitude airspeed]`.
        The ADDWPT keyword is part of the command grammar; insertion
        uses the same typed mutation path as ADDWPT.
        """
        if isinstance(found := _route_waypoint_index(self.traffic, acidx, beforewp), Err):
            return found
        return _add_route_waypoint(
            self.traffic, acidx, waypoint, alt, airspeed, InsertBefore(beforewp)
        )

    @command(name="AFTER")
    def insert_after(
        self,
        acidx: AcId,
        afterwp: Keyword,
        _keyword: Literal["ADDWPT"],
        waypoint: Wpt,
        alt: StdPressureAltM[IsFinite[float]] | None = None,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None = None,
    ) -> Result[str, str]:
        """Insert a waypoint after an existing route waypoint.

        Implements `acid AFTER waypoint ADDWPT new-waypoint [altitude airspeed]`.
        The ADDWPT keyword is part of the command grammar; insertion
        uses the same typed mutation path as ADDWPT.
        """
        if isinstance(found := _route_waypoint_index(self.traffic, acidx, afterwp), Err):
            return found
        return _add_route_waypoint(
            self.traffic, acidx, waypoint, alt, airspeed, InsertAfter(afterwp)
        )

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
        """Show the airspeed constraint at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        return Ok(_format_at_query(target.route, target.index, atwp, AtQuery.AIRSPEED))

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
        self,
        acidx: AcId,
        atwp: Keyword,
        _action: Literal["ALT"],
        value: StdPressureAltM[IsFinite[float]],
    ) -> Result[str, str]:
        """Set the altitude constraint at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpalt[target.index] = value.value
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def set_at_speed(
        self,
        acidx: AcId,
        atwp: Keyword,
        _action: Literal["SPD", "SPEED"],
        value: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]],
    ) -> Result[str, str]:
        """Set the airspeed constraint at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpairspeed[target.index] = value
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def set_at_constraints(
        self, acidx: AcId, atwp: Keyword, constraints: AtConstraintsArg
    ) -> Result[str, str]:
        """Set or clear the altitude/airspeed constraint pair at a route waypoint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        altitude = constraints.altitude
        airspeed = constraints.airspeed
        target.route.wpalt[target.index] = altitude
        target.route.wpairspeed[target.index] = airspeed
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
        """Clear a waypoint airspeed constraint."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpairspeed[target.index] = None
        return _finish_at_mutation(self.traffic, acidx, target)

    @command(name="AT")
    def clear_at_constraints(
        self,
        acidx: AcId,
        atwp: Keyword,
        _action: Literal["DEL", "DELETE", "CLR", "CLEAR"],
        _target: Literal["BOTH"],
    ) -> Result[str, str]:
        """Clear waypoint altitude and airspeed constraints."""
        if isinstance(target_result := _at_waypoint(self.traffic, acidx, atwp), Err):
            return target_result
        target = target_result.ok()
        target.route.wpalt[target.index] = None
        target.route.wpairspeed[target.index] = None
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
        target.route.wpairspeed[target.index] = None
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
            For example, BlueSky interprets `HX2 AT WPT10 STACK ALT 95FT[STD]`
            as though the deferred command were `HX2 ALT 95FT[STD]` (the HX2 target
            here is injected *implicitly* by bluesky)

            Minisky intentionally does **not** infer or inject that aircraft
            target. Write the complete deferred command instead:
            `HX2 AT WPT10 STACK HX2 ALT 95FT[STD]` (or the equivalent command-first
            form `HX2 AT WPT10 STACK ALT HX2 95FT[STD]`).

            The same rule applies to nested conditional commands. A BlueSky
            scenario line such as
            `HX2 AT WPT9D STACK ATSPD 200KT[CAS] HX2 ALT 100FT[STD]` should be
            `HX2 AT WPT9D STACK HX2 ATSPD 200KT[CAS] HX2 ALT 100FT[STD]` in minisky:
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
    def set_rta(self, acidx: AcId, wpname: Keyword, time: SimTimeS) -> Result[str, str]:
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
