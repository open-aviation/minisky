"""Autopilot Implementation.

Contains the [`Autopilot`][.Autopilot] class, which combines classic autopilot
modes (selected heading, altitude, vertical speed and airspeed) with FMS
guidance along the aircraft route: LNAV (lateral navigation towards the
active waypoint, including fly-by/fly-over/fly-turn logic) and VNAV
(Top-of-Climb/Top-of-Descent logic, altitude and airspeed constraints, and
required-time-of-arrival (RTA) airspeed scheduling).

The autopilot output (commanded track, true airspeed, altitude and vertical speed)
is combined with conflict-resolution commands in
[`APorASAS`][minisky.APorASAS] before being flown by
[`Traffic`][minisky.Traffic]. Many methods implement stack
commands (ALT, VS, HDG, SPD, DEST, ORIG, LNAV, VNAV, SWTOC, SWTOD).
"""

from __future__ import annotations

from collections.abc import Callable
from math import sqrt
from typing import TYPE_CHECKING

import numpy as np
from annotated_types import IsFinite

import minisky.geo as geo  # noqa: PLR0402
from minisky import quantities as q
from minisky._internal.command import (
    AcId,
    AcIdSelection,
    CoordinateWaypoint,
    NamedWaypoint,
    OnOff,
    VspdMps,
    command,
)
from minisky._internal.convert import degto180
from minisky._internal.position import txt2pos
from minisky._internal.result import Err, Ok, Result
from minisky._internal.route import (
    Route,
    RouteProfile,
    RtaTarget,
    TurnHeadingRate,
    TurnRadius,
    WaypointType,
    direct,
)
from minisky._internal.traffic_arrays import OptionalArray, TrafficArrays, VariantArray
from minisky.aero import g0, tas2cas, vcas2tas, vcasmach2tas
from minisky.types import (
    AircraftIndex,
    AirspeedKind,
    CasMps,
    Ge0,
    Gt0,
    LatLonDegrees,
    Mach,
    MagneticHeadingDeg,
    OptionalAirspeedKind,
    StdPressureAltM,
    TrueHeadingDeg,
    WaypointReference,
)

if TYPE_CHECKING:
    from minisky._internal.simulation import Simulation
    from minisky._internal.traffic import Traffic


def _waypoint_name(waypoint: CoordinateWaypoint | NamedWaypoint) -> WaypointReference:
    # TODO(abraham): bluesky stores origin and destination as strings, don't do so
    match waypoint:
        case NamedWaypoint(name):
            return name
        case CoordinateWaypoint():
            return f"{waypoint.latitude},{waypoint.longitude}"


def _resolve_waypoint(
    traffic: Traffic,
    acidx: AircraftIndex,
    route: Route,
    waypoint: CoordinateWaypoint | NamedWaypoint,
) -> Result[LatLonDegrees, str]:
    if isinstance(waypoint, CoordinateWaypoint):
        return Ok(waypoint.coordinates)

    name = waypoint.name
    apidx = traffic.navigation.getaptidx(name)
    if apidx is not None:
        return Ok(
            LatLonDegrees(
                float(traffic.navigation.aptlat[apidx]),
                float(traffic.navigation.aptlon[apidx]),
            )
        )

    if route.wpname:
        reflat = float(route.wplat[-1])
        reflon = float(route.wplon[-1])
    else:
        reflat = float(traffic.lat[acidx])
        reflon = float(traffic.lon[acidx])

    match txt2pos(name, reflat, reflon, traffic.navigation, traffic):
        case Ok(position):
            return Ok(LatLonDegrees(float(position.lat), float(position.lon)))
        case Err():
            return Err(f"Position {name} not found.")


class Autopilot(TrafficArrays):
    """Autopilot and FMS guidance implementation.

    Computes, per aircraft, the commanded track, altitude, vertical speed
    and true airspeed from the selected pilot airspeed reference and, when LNAV/VNAV are
    engaged, from the route stored in the per-aircraft [`Route`][minisky.Route]
    objects. Waypoint switching is event driven (see [`wppassingcheck`][.wppassingcheck]),
    while the continuous guidance in [`update`][.update] is fully vectorized over all
    aircraft.
    """

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__()
        self.traffic = traffic
        self.navigation = traffic.navigation
        self._get_simulation = get_simulation

        self.steepness: float = q.ft_to_m(3000.0) / q.nmi_to_m(10.0)
        """Default climb/descent gradient used for VNAV planning."""

        # NOTE(abraham): consider replacing this boolean/presence-array soup with typed guidance modes.
        # LNAV/VNAV/turn/RTA states currently permit combinations that the rest of this class
        # has to remember are invalid
        with self.settrafarrays():
            self.trk: q.GroundTrackDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.tas: q.TrueAirspeedMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.alt: q.PressureAltitudeM[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.vs: q.VerticalRateMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.swtoc = np.array([], dtype=bool)
            """Per-aircraft top-of-climb logic flags."""

            self.swtod = np.array([], dtype=bool)
            """Per-aircraft top-of-descent logic flags."""

            self.dist2vs: OptionalArray[q.DistanceM[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Distance to the active waypoint at which delayed VNAV climb or descent starts."""

            self.swvnavvs = np.array([], dtype=bool)
            """Whether VNAV uses its computed vertical-speed target."""

            self.vnavvs: q.VerticalRateMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]

            self.qdr2wp: OptionalArray[q.BearingDeg[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Bearing cached between waypoint checks to avoid late 180-degree turn reversals."""

            self.dist2wp: OptionalArray[q.DistanceM[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )

            self.qdrturn: OptionalArray[q.BearingDeg[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Bearing from each aircraft to its next fly-turn waypoint, when one exists."""

            self.dist2turn: OptionalArray[q.DistanceM[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )

            self.inturn = np.array([], dtype=bool)
            """Whether each aircraft is currently executing a turn."""

            self.orig: list[WaypointReference] = []
            """Stored `ORIG` waypoint source; may be an airport, navaid, runway, or coordinate reference."""
            self.dest: list[WaypointReference] = []
            """Stored `DEST` waypoint source; may be an airport, navaid, runway, or coordinate reference."""

            self.bankdef: q.BankAngleRad[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Default autopilot bank-angle limit per aircraft."""
            self.vsdef: q.VerticalRateMps[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Default autopilot vertical speed per aircraft."""
            self.turnphi: q.BankAngleRad[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Bank angle currently commanded for the active turn."""
            self.route: list[Route] = []

        self.idxreached = np.array([], dtype=int)
        """Aircraft indices that reached their active waypoint in the latest update."""

    @property
    def simulation(self) -> Simulation:
        """Return the simulation that owns this autopilot."""
        return self._get_simulation()

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's dependencies."""
        return implementation(self.traffic, self._get_simulation)

    def create(self, n: int = 1) -> None:
        """Initialize autopilot state for n newly created aircraft.

        Copies the initial track, airspeed and altitude from the traffic
        arrays, enables ToC/ToD logic, sets the default vertical speed
        (1500 fpm) and bank limit (25 deg), and creates an empty Route
        object for each new aircraft.
        """
        super().create(n)

        self.trk[-n:] = self.traffic.trk[-n:]
        self.tas[-n:] = self.traffic.tas[-n:]
        self.alt[-n:] = self.traffic.alt[-n:]
        self.vs[-n:] = self.traffic.vs[-n:]

        self.swtoc[-n:] = True
        self.swtod[-n:] = True

        # dist2vs starts absent until a delayed Top-of-Descent threshold is armed.

        # Direction to waypoint from the last time passing was checked
        # Distance to go to next waypoint [m]
        # Both start absent until route guidance computes an active leg.

        # Traffic performance data (temporarily default values)

        # TODO(abraham): in the trajectory reconstruction,
        # we want users to be able to customise the initial vertical speed
        # these defaults are bluesky-era and should be removed.

        self.vsdef[-n:] = q.fpm_to_mps(1500.0)

        self.bankdef[-n:] = np.radians(25.0)

        for ridx, callsign in enumerate(self.traffic.callsign[-n:]):
            self.route[ridx - n] = Route(self.traffic, callsign)

    def wppassingcheck(self, qdr: q.BearingDeg[np.ndarray], dist: q.DistanceM[np.ndarray]) -> None:
        """
        The actwp is the interface between the list of waypoint data in the route object and the autopilot guidance
        when LNAV is on (heading) and optionally VNAV is on (airspeed & altitude)

        actwp data contains traffic arrays, to allow vectorizing the guidance logic.

        Waypoint switching (just like the adding, deletion in route) are event driven commands and
        therefore not vectorized as they occur rarely compared to the guidance.

        wppassingcheck contains the waypoint switching function:
        - Check which aircraft have reached their active waypoint
        - Reached function returns list of indices where reached logic is True
        - Get the waypoint data to the actwp (active waypoint data)
        - Shift waypoint (last, next etc.) data for aircraft where necessary
        - Shift and maintain data (see last- and next- prefix in variable name) e.g. to continue a special turn
        - Prepare some VNAV triggers along the new leg for the VNAV profile (where to start descent/climb)

        `qdr` is updated in place for aircraft that switch waypoint.
        """
        # NOTE(abraham): it is difficult very difficult to understand this state transition:
        # detect passage -> mutate Route (?) -> execute deferred commands -> rebuild turn state
        # copy constraints -> prepare VNAV
        # maybe we want to split it up.

        # Get list of indices of aircraft which have reached their active waypoint
        # This vectorized function checks the passing of the waypoint using the current turn radius
        self.idxreached = self.traffic.actwp.reached(
            qdr,
            dist,
            self.traffic.actwp.flyby,
            self.traffic.actwp.flyturn,
            self.traffic.actwp.turnrad,
            self.traffic.actwp.turnhdgr,
            self.traffic.actwp.swlastwp,
        )

        actwp = self.traffic.actwp

        # Save current waypoint airspeed for use on the next leg when we pass this waypoint
        # VNAV speeds are always FROM-speeds, so we accelerate/decelerate at the waypoint
        # where this airspeed is specified, so we need to save it for use now
        # before getting the new data for the next waypoint

        # Get airspeed for the next leg from the waypoint we pass now and set it active
        actwp.airspeed.values[self.idxreached] = actwp.next_airspeed.values[self.idxreached]
        actwp.airspeed.kind[self.idxreached] = actwp.next_airspeed.kind[self.idxreached]
        actwp.airspeed_constraint.values[self.idxreached] = actwp.next_airspeed.values[
            self.idxreached
        ]
        actwp.airspeed_constraint.kind[self.idxreached] = actwp.next_airspeed.kind[self.idxreached]

        # Event-driven part, per aircraft: stack commands attached to the passed
        # waypoint and route iteration. These mutate the Route objects and queue
        # stack commands, so they cannot be vectorized. Gather the returned
        # scalar waypoint data in rows for the vectorized leg update below.
        idxlast = []  # reached aircraft already at their last waypoint
        idxnext = []  # reached aircraft with a next waypoint to activate
        transitions = []
        next_turns = []
        for i in self.idxreached:
            # Execute stack commands for the still active waypoint, which we pass now
            self.route[i].runactwpstack()

            if actwp.swlastwp[i]:
                # Prevent trying to activate the next waypoint when it was already the last waypoint
                idxlast.append(i)
            else:
                transitions.append(self.route[i].getnextwp())
                next_turns.append(self.route[i].getnextturnwp())
                idxnext.append(i)

        # In case of end of route/no more waypoints: switch off LNAV/VNAV
        if idxlast:
            last = np.array(idxlast)
            self.traffic.swlnav[last] = False
            self.traffic.swvnav[last] = False
            self.traffic.swvnavairspeed[last] = False

        # Vectorized leg data update for guidance, over the aircraft that
        # switched to a new waypoint
        if idxnext:
            nxt = np.array(idxnext)
            lat = np.fromiter((transition.position.lat for transition in transitions), dtype=float)
            lon = np.fromiter((transition.position.lon for transition in transitions), dtype=float)
            lnavon = np.fromiter(
                (transition.lnav_enabled for transition in transitions), dtype=bool
            )
            flyby = np.fromiter((transition.fly_by for transition in transitions), dtype=bool)
            swlastwp = np.fromiter(
                (transition.last_waypoint for transition in transitions), dtype=bool
            )
            has_nextleg = np.fromiter(
                (transition.next_leg is not None for transition in transitions), dtype=bool
            )
            nextleglat = np.fromiter(
                (
                    transition.position.lat
                    if transition.next_leg is None
                    else transition.next_leg.lat
                    for transition in transitions
                ),
                dtype=float,
            )
            nextleglon = np.fromiter(
                (
                    transition.position.lon
                    if transition.next_leg is None
                    else transition.next_leg.lon
                    for transition in transitions
                ),
                dtype=float,
            )

            ntransitions = len(transitions)
            next_airspeed = VariantArray(
                np.zeros(ntransitions), np.zeros(ntransitions, dtype=np.uint8)
            )
            nextaltco = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            xtoalt = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            torta = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            xtorta = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            turnrad = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            turn_cas = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            turnhdgr = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            nextturnlat = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            nextturnlon = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            next_turn_cas = OptionalArray(
                np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool)
            )
            nextturnrad = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            nextturnhdgr = OptionalArray(np.zeros(ntransitions), np.zeros(ntransitions, dtype=bool))
            nextturnidx = OptionalArray(
                np.zeros(ntransitions, dtype=int), np.zeros(ntransitions, dtype=bool)
            )
            flyturn = np.fromiter(
                (transition.turn is not None for transition in transitions), dtype=bool
            )

            for k, (transition, next_turn) in enumerate(zip(transitions, next_turns, strict=True)):
                if transition.airspeed is not None:
                    next_airspeed.values[k] = transition.airspeed.value
                    next_airspeed.kind[k] = (
                        OptionalAirspeedKind.CAS
                        if isinstance(transition.airspeed, CasMps)
                        else OptionalAirspeedKind.MACH
                    )
                if (altitude := transition.profile.altitude) is not None:
                    nextaltco.set(k, altitude.altitude)
                    xtoalt.set(k, altitude.distance)
                if (rta := transition.profile.rta) is not None:
                    torta.set(k, rta.time)
                    xtorta.set(k, rta.distance)
                if transition.turn is not None:
                    if transition.turn.cas is not None:
                        turn_cas.set(k, transition.turn.cas)
                    geometry = transition.turn.geometry
                    if isinstance(geometry, TurnRadius):
                        turnrad.set(k, geometry.radius)
                    elif isinstance(geometry, TurnHeadingRate):
                        turnhdgr.set(k, geometry.heading_rate)
                if next_turn is not None:
                    nextturnlat.set(k, next_turn.latitude)
                    nextturnlon.set(k, next_turn.longitude)
                    nextturnidx.set(k, next_turn.waypoint_index)
                    if next_turn.turn.cas is not None:
                        next_turn_cas.set(k, next_turn.turn.cas)
                    geometry = next_turn.turn.geometry
                    if isinstance(geometry, TurnRadius):
                        nextturnrad.set(k, geometry.radius)
                    elif isinstance(geometry, TurnHeadingRate):
                        nextturnhdgr.set(k, geometry.heading_rate)

            # Bearing of the leg after the new active waypoint, batched over
            # all switching aircraft; dummy coordinates keep lanes without a
            # next leg NaN-free until the ActiveWaypoint boundary conversion.
            batched_qdr, _ = geo.qdrdist(lat, lon, nextleglat, nextleglon)
            next_qdr = OptionalArray(np.asarray(batched_qdr), has_nextleg.copy())

            actwp.next_airspeed.values[nxt] = next_airspeed.values
            actwp.next_airspeed.kind[nxt] = next_airspeed.kind

            # User-entered altitude guidance for the new waypoint/profile.
            actwp.nextaltco.values[nxt] = nextaltco.values
            actwp.nextaltco.present[nxt] = nextaltco.present
            actwp.xtoalt.values[nxt] = xtoalt.values
            actwp.xtoalt.present[nxt] = xtoalt.present
            actwp.xtorta.values[nxt] = xtorta.values
            actwp.xtorta.present[nxt] = xtorta.present
            actwp.torta.values[nxt] = torta.values
            actwp.torta.present[nxt] = torta.present
            actwp.next_qdr.values[nxt] = next_qdr.values
            actwp.next_qdr.present[nxt] = next_qdr.present
            actwp.swlastwp[nxt] = swlastwp
            actwp.nextturnlat.values[nxt] = nextturnlat.values
            actwp.nextturnlat.present[nxt] = nextturnlat.present
            actwp.nextturnlon.values[nxt] = nextturnlon.values
            actwp.nextturnlon.present[nxt] = nextturnlon.present
            actwp.next_turn_cas.values[nxt] = next_turn_cas.values
            actwp.next_turn_cas.present[nxt] = next_turn_cas.present
            actwp.nextturnrad.values[nxt] = nextturnrad.values
            actwp.nextturnrad.present[nxt] = nextturnrad.present
            actwp.nextturnhdgr.values[nxt] = nextturnhdgr.values
            actwp.nextturnhdgr.present[nxt] = nextturnhdgr.present
            actwp.nextturnidx.values[nxt] = nextturnidx.values
            actwp.nextturnidx.present[nxt] = nextturnidx.present

            tas = self.traffic.tas[nxt]

            # Special turns: specified by turn radius, heading rate, and/or CAS.
            # If no turn CAS is specified, use current airspeed for the active fly-turn.
            has_turn_speed = turn_cas.present
            active_turn_speed = np.where(has_turn_speed, turn_cas.values, self.traffic.cas[nxt])

            # Use the previous turn geometry for bank angle in the current turn
            # (old values, from the waypoint we pass now; fancy indexing copies).
            oldturnrad = OptionalArray(actwp.turnrad.values[nxt], actwp.turnrad.present[nxt])
            oldturnhdgr = OptionalArray(actwp.turnhdgr.values[nxt], actwp.turnhdgr.present[nxt])
            old_turn_cas = OptionalArray(actwp.turn_cas.values[nxt], actwp.turn_cas.present[nxt])
            has_oldturnrad = oldturnrad.present
            has_oldturnhdgr = oldturnhdgr.present
            has_old_turn_cas = old_turn_cas.present
            oldturntas = vcas2tas(
                np.where(has_old_turn_cas, old_turn_cas.values, 0.0), self.traffic.alt[nxt]
            )
            oldradius = np.where(
                has_oldturnrad,
                oldturnrad.values,
                oldturntas
                * 360.0
                / (2.0 * np.pi * np.where(has_oldturnhdgr, oldturnhdgr.values, 1.0)),
            )
            useoldturn = has_old_turn_cas & (has_oldturnrad | has_oldturnhdgr)
            self.turnphi[nxt] = np.where(
                useoldturn,
                np.arctan(oldturntas * oldturntas / (np.where(useoldturn, oldradius, 1.0) * g0)),
                0.0,
            )  # [rad]

            # Check LNAV switch returned by getnextwp
            # Switch off LNAV if it failed to get next waypoint data
            lnavoff = ~lnavon & self.traffic.swlnav[nxt]
            has_next_airspeed = next_airspeed.kind != OptionalAirspeedKind.NONE
            use_last_airspeed = lnavoff & self.traffic.swvnavairspeed[nxt] & has_next_airspeed
            self.traffic.selected_airspeed.values[nxt] = np.where(
                use_last_airspeed,
                next_airspeed.values,
                self.traffic.selected_airspeed.values[nxt],
            )
            next_selected_kind = np.where(
                next_airspeed.kind == OptionalAirspeedKind.MACH,
                AirspeedKind.MACH,
                AirspeedKind.CAS,
            )
            self.traffic.selected_airspeed.kind[nxt] = np.where(
                use_last_airspeed,
                next_selected_kind,
                self.traffic.selected_airspeed.kind[nxt],
            )
            self.traffic.swlnav[nxt] = self.traffic.swlnav[nxt] & lnavon

            # In case of no LNAV, do not allow VNAV mode to be active
            self.traffic.swvnav[nxt] = self.traffic.swvnav[nxt] & self.traffic.swlnav[nxt]

            actwp.lat[nxt] = lat
            actwp.lon[nxt] = lon
            # 1.0 in case of fly by, else fly over
            actwp.flyby[nxt] = flyby

            # Update qdr and turn distance for this new waypoint for ComputeVNAV
            qdrnxt, distance = geo.qdrdist(self.traffic.lat[nxt], self.traffic.lon[nxt], lat, lon)
            qdr[nxt] = qdrnxt
            dist[nxt] = distance

            actwp.curlegdir.set(nxt, qdrnxt)
            actwp.curleglen.set(nxt, dist[nxt])

            # VNAV airspeed mode: use this waypoint airspeed as the commanded airspeed
            # while passing waypoint and save next airspeed for passing next waypoint
            # The next waypoint airspeed is already prepared in the waypoint data.
            active_airspeed = VariantArray(actwp.airspeed.values[nxt], actwp.airspeed.kind[nxt])
            use_waypoint_airspeed = self.traffic.swvnavairspeed[nxt] & (
                active_airspeed.kind != OptionalAirspeedKind.NONE
            )
            self.traffic.selected_airspeed.values[nxt] = np.where(
                use_waypoint_airspeed,
                active_airspeed.values,
                self.traffic.selected_airspeed.values[nxt],
            )
            active_selected_kind = np.where(
                active_airspeed.kind == OptionalAirspeedKind.MACH,
                AirspeedKind.MACH,
                AirspeedKind.CAS,
            )
            self.traffic.selected_airspeed.kind[nxt] = np.where(
                use_waypoint_airspeed,
                active_selected_kind,
                self.traffic.selected_airspeed.kind[nxt],
            )

            # Update turn distance so ComputeVNAV works, is there a next leg direction or not?
            local_next_qdr = np.where(~next_qdr.present, qdrnxt, next_qdr.values)

            # Calculate turn distance (and radius which we do not use now, but later)
            actwp.turndist[nxt], _ = actwp.calcturn(
                tas, self.bankdef[nxt], qdrnxt, local_next_qdr, turnrad, turnhdgr, flyturn
            )  # update turn distance for VNAV

            # Get flyturn switches and data
            # old turn CAS, turning by this waypoint
            actwp.old_turn_cas.values[nxt] = old_turn_cas.values
            actwp.old_turn_cas.present[nxt] = old_turn_cas.present
            actwp.flyturn[nxt] = flyturn
            actwp.turnrad.values[nxt] = turnrad.values
            actwp.turnrad.present[nxt] = turnrad.present
            actwp.turnhdgr.values[nxt] = turnhdgr.values
            actwp.turnhdgr.present[nxt] = turnhdgr.present
            # Keep both turning speeds: turn to leg and turn from leg
            actwp.turn_cas.values[nxt] = active_turn_speed
            actwp.turn_cas.present[nxt] = flyturn

            # Pass on whether currently flyturn mode:
            # at beginning of leg, copy to next waypoint to last waypoint
            # set next turn False
            actwp.turnfromlastwp[nxt] = actwp.turntonextwp[nxt]
            actwp.turntonextwp[nxt] = False

            # Reduce turn distance for reduced turn CAS
            redturn = flyturn & (~turnrad.present) & (~turnhdgr.present) & has_turn_speed
            turntas = vcas2tas(np.where(redturn, turn_cas.values, 0.0), self.traffic.alt[nxt])
            actwp.turndist[nxt] = actwp.turndist[nxt] * np.where(
                redturn, turntas * turntas / (tas * tas), 1.0
            )

            # VNAV = FMS ALT/SPD mode including RTA: still scalar, per aircraft
            for i, transition in zip(idxnext, transitions, strict=True):
                self.ComputeVNAV(i, transition.profile, float(dist[i]))

        # End of the waypoint switching update

        # Continuous guidance when airspeed constraint on active leg is in update-method

        # If still an RTA in the route and currently no airspeed constraint
        has_rta = self.traffic.actwp.torta.present
        has_speed_constraint = (
            self.traffic.actwp.airspeed_constraint.kind != OptionalAirspeedKind.NONE
        )
        for iac in np.where(has_rta & ~has_speed_constraint)[0]:
            iac = int(iac)
            route = self.route[iac]
            if (iwp := route.iactwp) is not None and route.wprta[iwp] is not None:
                # For all aircraft flying to an RTA waypoint, recalculate the RTA airspeed more often
                distance_to_waypoint = float(
                    np.asarray(
                        geo.kwikdist(
                            self.traffic.lat[iac],
                            self.traffic.lon[iac],
                            self.traffic.actwp.lat[iac],
                            self.traffic.actwp.lon[iac],
                        )
                    ).item()
                )

                # Set self.traffic.actwp.airspeed to the RTA airspeed, if necessary
                self.set_airspeed_for_rta(
                    iac, self.route[iac].wpprofile[iwp].rta, distance_to_waypoint
                )

                # If VNAV airspeed guidance is on (by default coupled to VNAV), use it for airspeed guidance
                if (
                    self.traffic.swvnavairspeed[iac]
                    and self.traffic.actwp.airspeed.kind[iac] != OptionalAirspeedKind.NONE
                ):
                    self.traffic.selected_airspeed.values[iac] = self.traffic.actwp.airspeed.values[
                        iac
                    ]
                    self.traffic.selected_airspeed.kind[iac] = (
                        AirspeedKind.MACH
                        if self.traffic.actwp.airspeed.kind[iac] == OptionalAirspeedKind.MACH
                        else AirspeedKind.CAS
                    )

    def update(self) -> None:
        """Run the continuous FMS/autopilot guidance for all aircraft.

        Called every simulation step. Recomputes bearing and distance to the
        active waypoints, performs the event-driven waypoint switching via
        [`wppassingcheck`][..wppassingcheck], and then applies the vectorized guidance:

        - VNAV altitude guidance: engage climb/descent when within dist2vs
          of the active waypoint (using the vertical speed prepared by
          [`ComputeVNAV`][..ComputeVNAV]).
        - LNAV track guidance: command the bearing to the active waypoint.
        - FMS airspeed guidance: anticipate deceleration for upcoming turn
          waypoints and acceleration/deceleration for airspeed constraints on
          the next leg, and select the appropriate [`CAS` in m/s][minisky.types.CasMps]
          or [`Mach`][minisky.types.Mach] command.

        The results are stored in the commanded-state arrays (trk, alt, vs,
        tas) and in the traffic selected-state arrays where applicable.
        """
        # FMS LNAV mode:
        qdr_result, distance = geo.qdrdist(
            self.traffic.lat,
            self.traffic.lon,
            self.traffic.actwp.lat,
            self.traffic.actwp.lon,
        )
        qdr = np.asarray(qdr_result)
        distance_to_waypoint = np.asarray(distance)

        # Check possible waypoint shift. Note: qdr and distance_to_waypoint are
        # updated accordingly in case of a waypoint switch.
        self.wppassingcheck(qdr, distance_to_waypoint)

        # Update qdr2wp and dist2wp with the current leg after checking waypoint passing.
        # Keep the geometry only while lateral or vertical route guidance owns it;
        # otherwise there is no meaningful active-guidance distance.
        # TODO(abraham): check if invalid states are possible.
        has_route_guidance = self.traffic.swlnav | self.traffic.swvnav
        self.qdr2wp.values[:] = qdr % 360.0
        self.qdr2wp.present[:] = has_route_guidance
        self.dist2wp.values[:] = distance_to_waypoint
        self.dist2wp.present[:] = self.qdr2wp.present

        # ================= Continuous FMS guidance ========================

        # VNAV altitude guidance logic (using the variables prepared by ComputeVNAV when activating waypoint)

        # First question is:
        # - Can we start to descend or to climb?
        #
        # The variable dist2vs indicates the distance to the active waypoint where we should start our climb/descend
        # Only use this logic if there is a valid next altitude constraint (nextaltco).
        #
        # When Top of Descent (ToD) switch is on, descend as late as possible,
        # But when Top of Climb switch is on or off, climb as soon as possible, only difference is steepness used in ComputeVNAV
        # to calculate self.traffic.actwp.vs

        nextaltco = self.traffic.actwp.nextaltco
        has_altitude_target = nextaltco.present
        nextalt = nextaltco.values
        has_vnav_start_distance = self.dist2vs.present
        vnav_start_distance = self.dist2vs.values
        startdescorclimb = has_altitude_target & np.logical_or(
            (self.traffic.alt > nextalt)
            & np.logical_or(
                np.logical_not(self.swtod),
                has_vnav_start_distance
                & (distance_to_waypoint < vnav_start_distance + self.traffic.actwp.turndist),
            ),
            self.traffic.alt < nextalt,
        )

        # If not LNAV: Climb/descend if doing so before LNAV/VNAV was switched off
        #    (because there are no more waypoints). This is needed
        #    to continue descending when you get into a conflict
        #    while descending to the destination (the last waypoint)
        #    Use 0.1 nm (185.2 m) circle in case turn distance might be zero
        has_vnav_vertical_speed = self.traffic.actwp.vs.present
        self.swvnavvs = (
            self.traffic.swvnav
            * has_vnav_vertical_speed
            * np.where(
                self.traffic.swlnav,
                startdescorclimb,
                distance_to_waypoint <= np.maximum(q.nmi_to_m(0.1), self.traffic.actwp.turndist),
            )
        )

        # Recalculate V/S based on current altitude and distance to next altitude constraint
        # How much time do we have before we need to descend?
        # Now done in ComputeVNAV
        # See ComputeVNAV for self.traffic.actwp.vs calculation

        self.vnavvs = np.where(self.swvnavvs, self.traffic.actwp.vs.values, self.vnavvs)
        # was: self.vnavvs  = np.where(self.swvnavvs, self.steepness * self.traffic.gs, self.vnavvs)

        # self.vs = np.where(self.swvnavvs, self.vnavvs, self.vsdef * self.traffic.limvs_flag)
        # for VNAV use fixed V/S and change start of descent
        # TODO(abraham): 0.1 m/s is a sentinel for "no selected vertical speed"
        # might be an issue with multicopters. we should use OptionalArray.
        selvs = np.where(abs(self.traffic.selvs) > 0.1, self.traffic.selvs, self.vsdef)  # m/s
        self.vs = np.where(self.swvnavvs, self.vnavvs, selvs)
        self.alt = np.where(self.swvnavvs, nextalt, self.traffic.selalt)

        # When descending or climbing in VNAV also update altitude command of select/hold mode
        self.traffic.selalt = np.where(self.swvnavvs, nextalt, self.traffic.selalt)

        # LNAV commanded track angle
        self.trk = np.where(self.traffic.swlnav, qdr % 360.0, self.trk)

        # FMS airspeed guidance: anticipate accel/decel distance for next leg or turn

        # Calculate actual distance it takes to decelerate/accelerate based on two cases: turn CAS (deceleration)

        # Normally next-leg airspeed (actwp.airspeed) but in case we fly turns with a specified turn CAS
        # use the turn CAS

        # TODO(abraham): this is hardcoded for fixed wing aircraft, move it behind vehicle guidance?
        # Is turn CAS specified and are we not already slow enough? We only decelerate for turns, not accel.
        has_next_turn = self.traffic.actwp.nextturnidx.present
        next_turn_cas = self.traffic.actwp.next_turn_cas
        has_turn_speed = has_next_turn & (next_turn_cas.present)
        turncas = np.where(has_turn_speed, next_turn_cas.values, self.traffic.cas)
        turntas = np.where(
            has_turn_speed,
            vcas2tas(turncas, self.traffic.alt),
            self.traffic.tas,
        )

        # FIXME(abraham): BlueSky 55c641e (2023-06-21) used next-turn presence
        # as turn-speed presence, so a fly-turn without TURNSPD could select missing CAS.
        # FIXME(abraham): the same commit used nextturnidx > 0, rejecting route index 0;
        # BlueSky 08194fa (2023-06-22) later corrected that check to >= 0.

        # t = (v1-v0)/a ; x = v0*t+1/2*a*t*t => dx = (v1*v1-v0*v0)/ (2a)
        dxturn_caschg = distaccel(turntas, self.traffic.tas, self.traffic.perf.axmax)

        # Decelerate or accelerate for the next explicit CAS/Mach constraint.
        # Convert the explicit CAS/Mach constraint to TAS only once altitude is known.
        next_airspeed = self.traffic.actwp.next_airspeed
        has_next_airspeed = next_airspeed.kind != OptionalAirspeedKind.NONE
        next_airspeed_kind = np.where(
            next_airspeed.kind == OptionalAirspeedKind.MACH,
            AirspeedKind.MACH,
            AirspeedKind.CAS,
        )
        next_airspeed_value = np.where(
            has_next_airspeed,
            next_airspeed.values,
            self.traffic.selected_airspeed.values,
        )
        next_airspeed_kind = np.where(
            has_next_airspeed, next_airspeed_kind, self.traffic.selected_airspeed.kind
        ).astype(np.uint8)
        next_tas = vcasmach2tas(
            next_airspeed_value, next_airspeed_kind == AirspeedKind.MACH, self.traffic.alt
        )
        distance_for_airspeed_change = distaccel(
            self.traffic.tas, next_tas, self.traffic.perf.axmax
        )

        qdrturn = np.zeros_like(self.traffic.lat)
        dist2turn = np.zeros_like(self.traffic.lat)
        if np.any(has_next_turn):
            qdrturn[has_next_turn], distance = geo.qdrdist(
                self.traffic.lat[has_next_turn],
                self.traffic.lon[has_next_turn],
                self.traffic.actwp.nextturnlat.values[has_next_turn],
                self.traffic.actwp.nextturnlon.values[has_next_turn],
            )
            dist2turn[has_next_turn] = distance

        # Where we don't have a turn waypoint, there is no turn bearing or distance.
        self.qdrturn.values[:] = qdrturn
        self.qdrturn.present[:] = has_next_turn
        self.dist2turn.values[:] = dist2turn
        self.dist2turn.present[:] = self.qdrturn.present

        # Check also whether VNAVSPD is on, if not, SPD SEL has override for next leg
        # and same for turn logic
        use_next_airspeed = (
            (distance_to_waypoint < distance_for_airspeed_change)
            & has_next_airspeed
            & self.traffic.swvnavairspeed
            & self.traffic.swvnav
            & self.traffic.swlnav
        )

        useturn_cas = (
            np.logical_or(
                self.traffic.actwp.turntonextwp,
                has_next_turn & (dist2turn < (dxturn_caschg + self.traffic.actwp.turndist)),
            )
            & has_turn_speed
            & self.traffic.swvnavairspeed
            & self.traffic.swvnav
            & self.traffic.swlnav
        )

        # Hold turn mode can only be switched on here, cannot be switched off here (happeps upon passing wp)
        self.traffic.actwp.turntonextwp = self.traffic.swlnav & np.logical_or(
            self.traffic.actwp.turntonextwp, useturn_cas
        )

        # TODO(abraham): remove this 2-degree threshold sentinel
        # Do not infer state from geometry tolerance; represent the active turn/leg explicitly.
        # Which CAS/Mach do we have to keep? VNAV, last turn or next turn?
        oncurrentleg = abs(degto180(self.traffic.trk - qdr)) < 2.0  # [deg]
        old_turn_cas = self.traffic.actwp.old_turn_cas
        has_old_turn_cas = old_turn_cas.present
        inoldturn = has_old_turn_cas & np.logical_not(oncurrentleg)

        # Avoid using old turning speeds when turning of this leg to the next leg
        # by disabling (old) turningspd when on leg
        self.traffic.actwp.old_turn_cas.clear(oncurrentleg & has_old_turn_cas)

        # turnfromlastwp can only be switched off here, not on (latter happens upon passing wp)
        self.traffic.actwp.turnfromlastwp = np.logical_and(
            self.traffic.actwp.turnfromlastwp, inoldturn
        )

        # Select turn CAS, next route airspeed, active route airspeed, or keep selection.
        active_constraint = self.traffic.actwp.airspeed_constraint
        active_airspeed = self.traffic.actwp.airspeed
        has_active_airspeed = (active_constraint.kind != OptionalAirspeedKind.NONE) & (
            active_airspeed.kind != OptionalAirspeedKind.NONE
        )
        use_active_airspeed = has_active_airspeed & self.traffic.swvnavairspeed
        active_airspeed_kind = np.where(
            active_airspeed.kind == OptionalAirspeedKind.MACH,
            AirspeedKind.MACH,
            AirspeedKind.CAS,
        )
        self.traffic.selected_airspeed.values[:] = np.where(
            useturn_cas,
            next_turn_cas.values,
            np.where(
                use_next_airspeed,
                next_airspeed.values,
                np.where(
                    use_active_airspeed,
                    active_airspeed.values,
                    self.traffic.selected_airspeed.values,
                ),
            ),
        )
        self.traffic.selected_airspeed.kind[:] = np.where(
            useturn_cas,
            AirspeedKind.CAS,
            np.where(
                use_next_airspeed,
                next_airspeed_kind,
                np.where(
                    use_active_airspeed,
                    active_airspeed_kind,
                    self.traffic.selected_airspeed.kind,
                ),
            ),
        ).astype(np.uint8)

        # A fly-turn CAS is always calibrated airspeed.
        use_old_turn_airspeed = (
            inoldturn & self.traffic.swvnavairspeed & self.traffic.swvnav & self.traffic.swlnav
        )
        self.traffic.selected_airspeed.values[:] = np.where(
            use_old_turn_airspeed, old_turn_cas.values, self.traffic.selected_airspeed.values
        )
        self.traffic.selected_airspeed.kind[:] = np.where(
            use_old_turn_airspeed, AirspeedKind.CAS, self.traffic.selected_airspeed.kind
        ).astype(np.uint8)

        self.inturn = np.logical_or(useturn_cas, inoldturn)
        self.tas = vcasmach2tas(
            self.traffic.selected_airspeed.values,
            self.traffic.selected_airspeed.kind == AirspeedKind.MACH,
            self.traffic.alt,
        )

    def ComputeVNAV(
        self, idx: AircraftIndex, profile: RouteProfile, distance_to_waypoint: q.DistanceM[float]
    ) -> None:
        """
        This function performs VNAV (and RTA) calculations once per leg for an aircraft index.
        If:
         - switching to next waypoint
         - when VNAV is activated
         - when a DIRECT is given

        It prepares the profile of this leg using the current altitude and the next altitude constraint (nextaltco).
        The distance to the next altitude constraint is given by xtoalt [m] after active waypoint.

        Options are (classic VNAV logic, swtoc and swtod True):

        - no altitude constraint in the future, do nothing
        - Top of CLimb logic (swtoc=True): if next altitude constraint is above us, climb as soon as possible with default steepness
        - Top of Descent Logic (swtod =True) Use ToD logic: descend as late as possible, based on
          steepness. Prepare a ToD somewhere on the leg if necessary based on distance to next altitude constraint.
          This is done by calculating distance to next waypoint where descent should start

        Alternative logic (e.g. for UAVs or GA):

        - swtoc=False and next alt co is above us, climb with the angle/steepness needed to arrive at the altitude at
        the waypoint with the altitude constraint (xtoalt m after active waypoint)
        - swtod=False and next altco is below us, descend with the angle/steepness needed to arrive at at the altitude at
        the waypoint with the altitude constraint (xtoalt m after active waypoint)

        Output of this function:

        self.dist2vs = distance 2 next waypoint where climb/descent needs to activated
        self.traffic.actwp.vs =  V/S to be used during climb/descent part, so when dist2wp<dist2vs [m] (to next waypoint)

        Args:
            profile: Optional altitude and RTA targets ahead of the active waypoint.
            distance_to_waypoint: Current distance to the active waypoint [m].
        """
        # TODO(abraham): untangle this, it is very difficult to understand.
        # it should be pure and not reach into autopilot etc.

        # Check whether active waypoint airspeed needs to be adjusted for RTA.
        # set_airspeed_for_rta sets self.traffic.actwp.airspeed if necessary.
        self.set_airspeed_for_rta(idx, profile.rta, distance_to_waypoint)
        self.dist2vs.clear(idx)

        # Check if there is a target altitude and VNAV is on, else return doing nothing.
        altitude_target = profile.altitude
        if altitude_target is None:
            self.traffic.actwp.nextaltco.clear(idx)
            self.traffic.actwp.xtoalt.clear(idx)
            self.traffic.actwp.vs.clear(idx)
            return

        toalt = altitude_target.altitude
        xtoalt = altitude_target.distance
        self.traffic.actwp.nextaltco.set(idx, toalt)
        self.traffic.actwp.xtoalt.set(idx, xtoalt)
        self.traffic.actwp.vs.clear(idx)

        if not self.traffic.swvnav[idx]:
            return
        # So: somewhere there is an altitude constraint ahead
        # Compute proper values for self.traffic.actwp.nextaltco, self.dist2vs, self.alt, self.traffic.actwp.vs
        # Descent VNAV mode (T/D logic)
        #
        # xtoalt  =  distance to go to next altitude constraint at a waypoint in the route
        #            (could be beyond next waypoint) [m]
        #
        # toalt   = altitude at next waypoint with an altitude constraint
        #
        # dist2vs = autopilot starts climb or descent when the remaining distance to next waypoint
        #           is this distance
        #
        #
        # VNAV Guidance principle:
        #
        #
        #                          T/C------X---T/D
        #                           /    .        \
        #                          /     .         \
        #       T/C----X----.-----X      .         .\
        #       /           .            .         . \
        #      /            .            .         .  X---T/D
        #     /.            .            .         .        \
        #    / .            .            .         .         \
        #   /  .            .            .         .         .\
        # pos  x            x            x         x         x X
        #
        #
        #  X = waypoint with alt constraint  x = Wp without prescribed altitude
        #
        # - Ignore and look beyond waypoints without an altitude constraint
        # - Climb as soon as possible after previous altitude constraint
        #   and climb as fast as possible, so arriving at alt earlier is ok
        # - Descend at the latest when necessary for next altitude constraint
        #   which can be many waypoints beyond current actual waypoint
        # NOTE(abraham): lots of magic numbers in VNAV thresholds.
        # (2 ft vs 9.9 ft deadzones, 0.0001 m/s, 1.02 turn margin, 0.01 divisors,
        # 0.2 * TAS). reconsider.
        epsalt = q.ft_to_m(2.0)  # deadzone
        if self.traffic.alt[idx] > toalt + epsalt:
            # Stop potential current climb (e.g. due to not making it to previous altco)
            # then stop immediately, as in: do not make it worse.
            if self.traffic.vs[idx] > 0.0001:
                self.vnavvs[idx] = 0.0
                self.alt[idx] = self.traffic.alt[idx]
                if self.traffic.swvnav[idx]:
                    self.traffic.selalt[idx] = self.traffic.alt[idx]

            # Descent modes: VNAV (= swtod/Top of Descent logic) or aiming at next alt constraint

            # Calculate max allowed altitude at next wp (above toalt)
            self.traffic.actwp.nextaltco.set(idx, toalt)
            self.traffic.actwp.xtoalt.set(idx, xtoalt)

            # VNAV ToD logic
            if self.swtod[idx]:
                # Distance to next waypoint where we need to start descent (top of descent) [m]
                descdist = (
                    abs(self.traffic.alt[idx] - toalt) / self.steepness
                )  # [m] required length for descent, uses default steepness!
                self.dist2vs.set(idx, descdist - xtoalt)

                # Exceptions: Descend now?
                if (
                    distance_to_waypoint - 1.02 * self.traffic.actwp.turndist[idx]
                    < self.dist2vs.values[idx]
                ):  # Urgent descent, we're late![m]
                    # Descend now using whole remaining distance on leg to reach altitude
                    self.alt[idx] = self.traffic.actwp.nextaltco.values[idx]
                    # dial in altitude of next waypoint as calculated
                    t2go = distance_to_waypoint / max(0.01, self.traffic.gs[idx])
                    self.traffic.actwp.vs.set(
                        idx, (self.traffic.alt[idx] - toalt) / max(0.01, t2go)
                    )

                elif xtoalt < descdist:  # Not on this leg, no descending is needed at next waypoint
                    # Top of decent needs to be on this leg, as next wp is in descent
                    self.traffic.actwp.vs.set(
                        idx,
                        -abs(self.steepness)
                        * (
                            self.traffic.gs[idx]
                            + (self.traffic.gs[idx] < 0.2 * self.traffic.tas[idx])
                            * self.traffic.tas[idx]
                        ),
                    )

                else:
                    # else still level
                    self.traffic.actwp.vs.set(idx, 0.0)

            else:
                # We are higher but swtod = False, so there is no ToD descent logic, simply aim at next altco
                # and descend immediately rather than arming a distance threshold.
                steepness_ = (
                    self.traffic.alt[idx] - self.traffic.actwp.nextaltco.values[idx]
                ) / max(0.01, distance_to_waypoint + xtoalt)
                self.traffic.actwp.vs.set(
                    idx,
                    -abs(steepness_)
                    * (
                        self.traffic.gs[idx]
                        + (self.traffic.gs[idx] < 0.2 * self.traffic.tas[idx])
                        * self.traffic.tas[idx]
                    ),
                )

        # VNAV climb mode: climb as soon as possible (T/C logic)
        elif self.traffic.alt[idx] < toalt - q.ft_to_m(9.9):
            # Stop potential current descent (e.g. due to not making it to previous altco)
            # then stop immediately, as in: do not make it worse.
            if self.traffic.vs[idx] < -0.0001:
                self.vnavvs[idx] = 0.0
                self.alt[idx] = self.traffic.alt[idx]
                if self.traffic.swvnav[idx]:
                    self.traffic.selalt[idx] = self.traffic.alt[idx]

            # Altitude we want to climb to: next alt constraint in our route (could be further down the route)
            self.traffic.actwp.nextaltco.set(idx, toalt)
            self.traffic.actwp.xtoalt.set(idx, xtoalt)
            self.alt[idx] = self.traffic.actwp.nextaltco.values[idx]
            # Climb starts immediately; no distance threshold needs to be armed.
            t2go = max(0.1, distance_to_waypoint + xtoalt) / max(0.01, self.traffic.gs[idx])
            if self.swtoc[idx]:
                steepness_ = self.steepness  # default steepness
            else:
                steepness_ = (
                    self.traffic.alt[idx] - self.traffic.actwp.nextaltco.values[idx]
                ) / max(0.01, distance_to_waypoint + xtoalt)

            self.traffic.actwp.vs.set(
                idx,
                np.maximum(
                    steepness_ * self.traffic.gs[idx],
                    (self.traffic.actwp.nextaltco.values[idx] - self.traffic.alt[idx]) / t2go,
                ),
            )
        # Level leg: never start V/S
        else:
            self.traffic.actwp.vs.clear(idx)

        return

    def set_airspeed_for_rta(
        self, idx: AircraftIndex, target: RtaTarget | None, distance_to_waypoint: q.DistanceM[float]
    ) -> q.CalibratedAirspeedMps[float] | None:
        """Compute and set the [`CAS` in m/s][minisky.types.CasMps] required to meet an RTA constraint.

        Calculates the ground speed needed to cover the remaining distance
        to the RTA waypoint exactly at the required time (see [`calcvrta`][...calcvrta]),
        corrects for the tailwind component and converts to CAS. When no
        explicit airspeed constraint is active and VNAV airspeed guidance is on,

        Args:
            target: Next RTA target, or `None` when no RTA remains.
            distance_to_waypoint: Current distance to the active waypoint.

        Returns:
            Required [`CAS` in m/s][minisky.types.CasMps], or None when there is no feasible RTA.
        """
        if target is None:
            return None

        distance = distance_to_waypoint + target.distance
        deltime = target.time - self.simulation.simt
        if deltime > 0:  # Still possible?
            gsrta = calcvrta(self.traffic.gs[idx], distance, deltime, self.traffic.perf.axmax[idx])

            # TODO(abraham): RTA guidance assumes meaningful forward ground speed and a CAS
            # command
            tailwind = (
                self.traffic.windnorth[idx] * self.traffic.gsnorth[idx]
                + self.traffic.windeast[idx] * self.traffic.gseast[idx]
            ) / self.traffic.gs[idx]

            rtacas = tas2cas(gsrta - tailwind, self.traffic.alt[idx])

            # Performance airspeed limits will be applied in traf.update
            if (
                self.traffic.actwp.airspeed_constraint.kind[idx] == OptionalAirspeedKind.NONE
                and self.traffic.swvnavairspeed[idx]
            ):
                self.traffic.actwp.airspeed.values[idx] = rtacas
                self.traffic.actwp.airspeed.kind[idx] = OptionalAirspeedKind.CAS

            return rtacas

        # FIXME(abraham): BlueSky f352d73 (2019-07-14) left the previous RTA-derived
        # stale RTA airspeed active after the RTA became infeasible; clear it when no constraint owns it.
        if self.traffic.actwp.airspeed_constraint.kind[idx] == OptionalAirspeedKind.NONE:
            self.traffic.actwp.airspeed.kind[idx] = OptionalAirspeedKind.NONE
        return None

    @command(name="ALT")
    def selaltcmd(
        self, idx: AcIdSelection, alt: StdPressureAltM[IsFinite[float]], vspd: VspdMps | None = None
    ) -> Result[str, str]:
        """Select the autopilot altitude, optionally with a vertical speed.

        Selecting an altitude disengages VNAV for this aircraft. When no
        vertical speed is given and the currently selected vertical speed
        opposes the required climb/descent direction, it is reset so the
        default vertical speed is used.
        """
        self.traffic.selalt[idx] = alt.value
        self.traffic.swvnav[idx] = False

        if vspd:
            self.traffic.selvs[idx] = vspd
        else:
            delalt = alt.value - self.traffic.alt[idx]
            # Check for VS with opposite sign => use default vs
            # by setting autopilot vs to zero
            oppositevs = np.logical_and(
                self.traffic.selvs[idx] * delalt < 0.0,
                abs(self.traffic.selvs[idx]) > 0.01,
            )

            self.traffic.selvs[idx[oppositevs]] = 0.0
        return Ok(f"altitude set to {q.m_to_ft(alt.value)} ft")

    @command(name="VS")
    def selvspdcmd(self, idx: AcIdSelection, vspd: VspdMps) -> Result[str, str]:
        """Select the autopilot vertical speed.

        Setting a vertical speed disengages VNAV for this aircraft.
        """
        self.traffic.selvs[idx] = vspd
        self.traffic.swvnav[idx] = False
        return Ok(f"vertical speed set to {q.mps_to_fpm(vspd)} ft/min")

    @command(name="HDG", aliases=("HEADING", "TURN"))
    def selhdgcmd(
        self,
        idx: AcIdSelection,
        hdg: TrueHeadingDeg[IsFinite[float]] | MagneticHeadingDeg[IsFinite[float]],
    ) -> Result[str, str]:
        """Select the autopilot heading.

        When a wind field is defined and the aircraft is airborne (above 50 ft),
        the commanded track is computed from the given heading and the local
        wind; otherwise track equals heading. Selecting a heading disengages
        LNAV for this aircraft.
        """

        resolved_hdg: q.TrueHeadingDegrees
        if isinstance(hdg, MagneticHeadingDeg):
            resolved_hdg = np.fromiter(
                (
                    (hdg.value + geo.magdec(float(lat), float(lon))) % 360.0
                    for lat, lon in zip(self.traffic.lat[idx], self.traffic.lon[idx], strict=True)
                ),
                dtype=float,
            )
        else:
            resolved_hdg = hdg.value

        if self.traffic.wind.has_wind:
            tasnorth = self.traffic.tas[idx] * np.cos(np.radians(resolved_hdg))
            taseast = self.traffic.tas[idx] * np.sin(np.radians(resolved_hdg))
            wind_north, wind_east = self.traffic.wind.getdata(
                self.traffic.lat[idx], self.traffic.lon[idx], self.traffic.alt[idx]
            )
            wind_track = np.degrees(np.arctan2(taseast + wind_east, tasnorth + wind_north)) % 360.0
            # TODO: switch above 50 ft AGL is inherited from bluesky
            # and is scheduled for removal when we implement #22 (AGL)
            self.trk[idx] = np.where(
                self.traffic.alt[idx] > q.ft_to_m(50.0), wind_track, resolved_hdg
            )
        else:
            self.trk[idx] = resolved_hdg

        self.traffic.swlnav[idx] = False
        return Ok(f"heading set to {resolved_hdg} deg")

    @command(name="SPD", aliases=("SPEED",))
    def select_airspeed(
        self,
        idx: AcIdSelection,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]],
    ) -> Result[str, str]:
        """Select [`CAS` in m/s][minisky.types.CasMps] or [`Mach`][minisky.types.Mach] explicitly."""
        self.traffic.selected_airspeed.values[idx] = airspeed.value
        if isinstance(airspeed, CasMps):
            self.traffic.selected_airspeed.kind[idx] = AirspeedKind.CAS
            message = f"airspeed set to {q.mps_to_kt(airspeed.value):g} kt CAS"
        else:
            self.traffic.selected_airspeed.kind[idx] = AirspeedKind.MACH
            message = f"airspeed set to Mach {airspeed.value:g}"
        self.traffic.swvnavairspeed[idx] = False
        return Ok(message)

    @command(name="DEST")
    def show_destination(self, acidx: AcId) -> Result[str, str]:
        """Show the destination of an aircraft."""
        return Ok(f"DEST {self.traffic.callsign[acidx]}: {self.dest[acidx]}")

    @command(name="DEST")
    def set_destination(
        self,
        acidx: AcId,
        waypoint: CoordinateWaypoint | NamedWaypoint,
        airspeed: CasMps[IsFinite[Ge0[float]]] | Mach[IsFinite[Gt0[float]]] | None = None,
    ) -> Result[str, str]:
        """Set the destination with an optional [`CAS` in m/s][minisky.types.CasMps] or [`Mach`][minisky.types.Mach] constraint."""
        route = self.route[acidx]
        wpname = _waypoint_name(waypoint)
        if isinstance(position := _resolve_waypoint(self.traffic, acidx, route, waypoint), Err):
            return Err("DEST: " + position.err())
        coordinates = position.ok()
        self.dest[acidx] = wpname
        # TODO(abraham): use MSL elevation (see issue #22)
        if (
            iwp := route.add_waypoint(
                acidx,
                self.dest[acidx],
                WaypointType.DESTINATION,
                coordinates.lat,
                coordinates.lon,
                0.0,
                airspeed,
            )
        ) is None:
            return Err("DEST position" + self.dest[acidx] + " not found.")

        if (iwp == 0) or (self.orig[acidx] != "" and len(route.wpname) == 2):
            self.traffic.swlnav[acidx] = True
            self.traffic.swvnav[acidx] = True
            route.iactwp = iwp
            direct(self.traffic, acidx, route.wpname[iwp])

        return Ok(f"destination set to {wpname}")

    @command(name="ORIG")
    def show_origin(self, acidx: AcId) -> Result[str, str]:
        """Show the origin of an aircraft."""
        return Ok(f"ORIG {self.traffic.callsign[acidx]}: {self.orig[acidx]}")

    @command(name="ORIG")
    def set_origin(
        self, acidx: AcId, waypoint: CoordinateWaypoint | NamedWaypoint
    ) -> Result[str, str]:
        """Set the origin of an aircraft."""
        route = self.route[acidx]
        wpname = _waypoint_name(waypoint)
        if isinstance(position := _resolve_waypoint(self.traffic, acidx, route, waypoint), Err):
            return Err("ORIG: " + position.err())
        coordinates = position.ok()

        # Origin: bookkeeping only for now, store in route as origin
        self.orig[acidx] = wpname
        # TODO(abraham): use MSL elevation (see issue #22)
        if (
            route.add_waypoint(
                acidx,
                self.orig[acidx],
                WaypointType.ORIGIN,
                coordinates.lat,
                coordinates.lon,
                0.0,
                CasMps(float(self.traffic.cas[acidx])),
            )
            is None
        ):
            return Err(self.orig[acidx] + " not found.")
        return Ok(f"origin set to {wpname}")

    @command(name="VNAV")
    def vnav_status(self, idx: AcIdSelection) -> Result[str, str]:
        """Show VNAV state for an aircraft or selection."""
        # BlueSky applies these commands to every aircraft in the resolved selection.
        output: list[str] = []
        for i in idx:
            msg = f"{self.traffic.callsign[i]}: VNAV is {'ON' if self.traffic.swvnav[i] else 'OFF'}"
            if not self.traffic.swvnavairspeed[i]:
                msg += " but VNAVSPD is OFF"
            output.append(msg)
        return Ok("\n".join(output))

    @command(name="VNAV")
    def set_vnav(self, idx: AcIdSelection, flag: OnOff) -> Result[str, str]:
        """Enable or disable VNAV for an aircraft or selection."""
        # Keep the per-aircraft checks here: they are runtime state invariants, not syntax dispatch.
        for i in idx:
            if flag:
                if not self.traffic.swlnav[i]:
                    return Err(self.traffic.callsign[i] + ": VNAV ON requires LNAV to be ON")
                route = self.route[i]
                if not route.wpname:
                    return Err(
                        "VNAV "
                        + self.traffic.callsign[i]
                        + ": no waypoints or destination specified"
                    )
                self.traffic.swvnav[i] = True
                self.traffic.swvnavairspeed[i] = True
                route.calcfp()
                actwpidx = route.iactwp
                assert actwpidx is not None
                profile = route.wpprofile[actwpidx]
                distance_to_waypoint = float(
                    np.asarray(
                        geo.kwikdist(
                            self.traffic.lat[i],
                            self.traffic.lon[i],
                            self.traffic.actwp.lat[i],
                            self.traffic.actwp.lon[i],
                        )
                    ).item()
                )
                self.dist2wp.set(i, distance_to_waypoint)
                self.ComputeVNAV(i, profile, distance_to_waypoint)
                if profile.altitude is None:
                    self.traffic.actwp.nextaltco.clear(i)
                else:
                    self.traffic.actwp.nextaltco.set(i, profile.altitude.altitude)
            else:
                self.traffic.swvnav[i] = False
                self.traffic.swvnavairspeed[i] = False
        return Ok(f"VNAV {'ON' if flag else 'OFF'}")

    @command(name="LNAV")
    def lnav_status(self, idx: AcIdSelection) -> Result[str, str]:
        """Show LNAV state for an aircraft or selection."""
        return Ok(
            "\n".join(
                f"{self.traffic.callsign[i]}: LNAV is {'ON' if self.traffic.swlnav[i] else 'OFF'}"
                for i in idx
            )
        )

    @command(name="LNAV")
    def set_lnav(self, idx: AcIdSelection, flag: OnOff) -> Result[str, str]:
        """Enable or disable LNAV for an aircraft or selection."""
        for i in idx:
            if flag:
                route = self.route[i]
                if not route.wpname:
                    return Err(
                        "LNAV "
                        + self.traffic.callsign[i]
                        + ": no waypoints or destination specified"
                    )
                if not self.traffic.swlnav[i]:
                    self.traffic.swlnav[i] = True
                    active_waypoint = route.findact(i)
                    assert active_waypoint is not None
                    direct(self.traffic, i, route.wpname[active_waypoint])
            else:
                self.traffic.swlnav[i] = False
        return Ok(f"LNAV {'ON' if flag else 'OFF'}")

    @command(name="SWTOC")
    def swtoc_status(self, idx: AcIdSelection) -> Result[str, str]:
        """Show Top-of-Climb state for an aircraft or selection."""
        return Ok(
            "\n".join(
                f"{self.traffic.callsign[i]}: SWTOC is {'ON' if self.swtoc[i] else 'OFF'}"
                for i in idx
            )
        )

    @command(name="SWTOC")
    def set_swtoc(self, idx: AcIdSelection, flag: OnOff) -> Result[str, str]:
        """Enable or disable Top-of-Climb logic."""
        self.swtoc[idx] = flag
        return Ok(f"SWTOC {'ON' if flag else 'OFF'}")

    @command(name="SWTOD")
    def swtod_status(self, idx: AcIdSelection) -> Result[str, str]:
        """Show Top-of-Descent state for an aircraft or selection."""
        return Ok(
            "\n".join(
                f"{self.traffic.callsign[i]}: SWTOD is {'ON' if self.swtod[i] else 'OFF'}"
                for i in idx
            )
        )

    @command(name="SWTOD")
    def set_swtod(self, idx: AcIdSelection, flag: OnOff) -> Result[str, str]:
        """Enable or disable Top-of-Descent logic."""
        self.swtod[idx] = flag
        return Ok(f"SWTOD {'ON' if flag else 'OFF'}")


def calcvrta(
    v0: q.GroundSpeedMps[float],
    dx: q.DistanceM[float],
    deltime: q.DurationS[float],
    trafax: q.AccelerationMps2[float],
) -> q.GroundSpeedMps[float]:
    """Calculate the target ground speed needed to meet an RTA on a leg.

    Solves for the end speed of a constant-acceleration speed change
    followed by a constant-speed segment, such that the remaining leg
    distance is covered exactly in the remaining time. Falls back to the
    simple average speed dx/deltime when no physical solution exists.

    Args:
        v0: Current ground speed.
        dx: Remaining distance to the RTA waypoint.
        deltime: Remaining time until the RTA.
        trafax: Available longitudinal acceleration.
    """
    # Calculate required target ground speed v1 [m/s]
    # to meet an RTA at this leg
    #
    # Arguments are scalar
    #
    #   v0      = current ground speed [m/s]
    #   dx      = leg distance [m]
    #   deltime = time left till RTA[s]
    #   trafax  = horizontal acceleration [m/s2]

    dt = deltime

    # Do we need decelerate or accelerate
    ax = max(0.01, abs(trafax)) if v0 * dt < dx else -max(0.01, abs(trafax))

    # Solve 2nd order equation for v1 which results from:
    #
    #   dx = 0.5*(v0+v1)*dtacc + v1 * dtconst
    #   dt = trta - tnow = dtacc + dtconst
    #   dtacc = (v1-v0)/ax
    #
    # with unknown dtconst, dtacc, v1
    #
    # -.5/ax * v1**2  +(v0/ax+dt)*v1 -0.5*v0**2 / ax - dx =0

    a = -0.5 / ax
    b = v0 / ax + dt
    c = -0.5 * v0 * v0 / ax - dx

    D = b * b - 4.0 * a * c

    # Possibly two v1 solutions
    vlst = []

    if D >= 0.0:
        x1 = (-b - sqrt(D)) / (2.0 * a)
        x2 = (-b + sqrt(D)) / (2.0 * a)

        for v1 in (x1, x2):
            dtacc = (v1 - v0) / ax
            dtconst = dt - dtacc

            # Physically possible: both dtacc and dtconst >0
            if dtacc >= 0 and dtconst >= 0.0:
                vlst.append(v1)

    if len(vlst) == 0:  # Not possible? Maybe borderline, so then simple calculation
        vtarg = dx / dt

    # Just in case both would be valid, take closest to v0
    elif len(vlst) == 2:
        vtarg = vlst[int(abs(vlst[1] - v0) < abs(vlst[0] - v0))]

    # Normal case is one solution
    else:
        vtarg = vlst[0]

    return vtarg


def distaccel(
    v0: q.TrueAirspeedMps,
    v1: q.TrueAirspeedMps,
    axabs: q.AccelerationMps2,
) -> q.DistanceM:
    """Calculate the distance travelled during an acceleration/deceleration.

    Uses the uniform-acceleration relation dx = |v1^2 - v0^2| / (2 |a|),
    which follows from x = v0*t + 1/2*a*t^2 and v = v0 + a*t. Whether it is
    an acceleration or a deceleration is determined by the sign of v1 - v0.
    Works on scalars as well as numpy arrays.

    Args:
        v0: Speed at the start of the speed change.
        v1: Speed at the end of the speed change.
        axabs: Acceleration magnitude; its absolute value is used.
    """
    return 0.5 * np.abs(v1 * v1 - v0 * v0) / np.maximum(0.001, np.abs(axabs))
