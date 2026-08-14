"""Active waypoint data for FMS guidance.

Holds, as per-aircraft numpy arrays, all data of the waypoint each aircraft
is currently flying towards. The [`ActiveWaypoint`][minisky.traffic.activewpdata.ActiveWaypoint] arrays form the
interface between the per-aircraft [`Route`][minisky.traffic.route.Route]
objects (event-driven, scalar waypoint switching) and the vectorized
LNAV/VNAV guidance in [`Autopilot`][minisky.traffic.autopilot.Autopilot].
Available as [`runtime.traffic.actwp`][minisky.traffic.activewpdata.ActiveWaypoint].
"""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import numpy as np

from minisky import quantities as q
from minisky.core.trafficarrays import OptionalArray, TrafficArrays, VariantArray
from minisky.tools.aero import g0, vcas2tas
from minisky.tools.convert import degto180

if TYPE_CHECKING:
    from minisky.traffic import Traffic


class ActiveWaypoint(TrafficArrays):
    """Per-aircraft data of the active (and next) waypoint.

    The autopilot copies waypoint data from the route into these arrays
    upon waypoint switching (see Autopilot.wppassingcheck() and
    route.direct()), so the continuous guidance can be vectorized. Simple optional
    per-aircraft values use `OptionalArray`; optional [`CAS` in m/s][minisky.values.CasMps]
    or [`Mach`][minisky.values.Mach] values use `VariantArray`.

    Attributes:
        lat (ndarray): Active waypoint latitude [deg].
        lon (ndarray): Active waypoint longitude [deg].
        nextturnlat (ndarray): Next turn waypoint latitude [deg].
        nextturnlon (ndarray): Next turn waypoint longitude [deg].
        next_turn_cas (ndarray): Next turn waypoint turn [`CAS` in m/s][minisky.values.CasMps].
        nextturnrad (ndarray): Next turn waypoint turn radius [m].
        nextturnhdgr (ndarray): Next turn waypoint heading rate [deg/s].
        nextturnidx (ndarray): Route index of the next turn waypoint.
        nextaltco (ndarray): Next altitude constraint [m].
        xtoalt (ndarray): Distance to the next altitude constraint [m].
        next_airspeed (VariantArray): Optional [`CAS` in m/s][minisky.values.CasMps]
            or [`Mach`][minisky.values.Mach] value for the next leg.
        airspeed (VariantArray): Optional active [`CAS` in m/s][minisky.values.CasMps]
            or [`Mach`][minisky.values.Mach] value.
        airspeed_constraint (VariantArray): Optional active waypoint [`CAS` in m/s][minisky.values.CasMps]
            or [`Mach`][minisky.values.Mach] constraint.
        vs (ndarray): Vertical speed to use in VNAV climb/descent [m/s].
        turndist (ndarray): Distance before the waypoint at which to start
            the turn [m].
        flyby (ndarray): Fly-by switch; when False, fly-over (turndist 0).
        flyturn (ndarray): Fly-turn switch (use specified turn parameters).
        turnrad (ndarray): Turn radius at the active waypoint [m].
        turn_cas (ndarray): [`CAS` in m/s][minisky.values.CasMps] held through the active fly-turn.
        turnhdgr (ndarray): Turn heading rate at the active waypoint
            [deg/s].
        old_turn_cas (ndarray): [`CAS` in m/s][minisky.values.CasMps] held while completing the previous turn.
        turnfromlastwp (ndarray): In fly-turn mode from the last waypoint
            (old turn, beginning of leg).
        turntonextwp (ndarray): In fly-turn mode towards the next waypoint
            (new turn, end of leg).
        torta (OptionalArray): Optional next required time of arrival [s].
        xtorta (ndarray): Distance to the next RTA waypoint [m].
        next_qdr (ndarray): Track angle of the next leg [deg].
        swlastwp (ndarray): Bool switch: active waypoint is the last one.
        curlegdir (ndarray): Direction of the current leg, set when the
            waypoint was activated [deg].
        curleglen (ndarray): Length of the current leg, set when the
            waypoint was activated [m].
    """

    lat: q.LatitudeDeg[np.ndarray]
    lon: q.LongitudeDeg[np.ndarray]
    nextturnlat: OptionalArray[q.LatitudeDeg[np.ndarray]]
    nextturnlon: OptionalArray[q.LongitudeDeg[np.ndarray]]
    next_airspeed: VariantArray[np.ndarray]
    airspeed: VariantArray[np.ndarray]
    airspeed_constraint: VariantArray[np.ndarray]
    next_turn_cas: OptionalArray[q.CalibratedAirspeedMps[np.ndarray]]
    nextturnrad: OptionalArray[q.TurnRadiusM[np.ndarray]]
    nextturnhdgr: OptionalArray[q.TurnRateDegPerS[np.ndarray]]
    nextturnidx: OptionalArray[np.ndarray]
    nextaltco: OptionalArray[q.PressureAltitudeM[np.ndarray]]
    xtoalt: OptionalArray[q.DistanceM[np.ndarray]]
    vs: OptionalArray[q.VerticalRateMps[np.ndarray]]
    turndist: q.DistanceM[np.ndarray]
    turnrad: OptionalArray[q.TurnRadiusM[np.ndarray]]
    turn_cas: OptionalArray[q.CalibratedAirspeedMps[np.ndarray]]
    turnhdgr: OptionalArray[q.TurnRateDegPerS[np.ndarray]]
    old_turn_cas: OptionalArray[q.CalibratedAirspeedMps[np.ndarray]]
    torta: OptionalArray[q.SimulationTimeS[np.ndarray]]
    xtorta: OptionalArray[q.DistanceM[np.ndarray]]
    next_qdr: OptionalArray[q.BearingDeg[np.ndarray]]
    curlegdir: OptionalArray[q.BearingDeg[np.ndarray]]
    curleglen: OptionalArray[q.DistanceM[np.ndarray]]

    def __init__(self, traffic: Traffic) -> None:
        super().__init__(traffic)
        self.traffic = traffic
        with self.settrafarrays():
            self.lat = np.array([])  # [deg] Active WP latitude
            self.lon = np.array([])  # [deg] Active WP longitude
            self.nextturnlat = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.nextturnlon = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.next_turn_cas = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.nextturnrad = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.nextturnhdgr = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.nextturnidx = OptionalArray(np.array([], dtype=int), np.array([], dtype=bool))
            self.nextaltco = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.xtoalt = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.next_airspeed = VariantArray(np.array([]), np.array([], dtype=np.uint8))
            self.airspeed = VariantArray(np.array([]), np.array([], dtype=np.uint8))
            self.airspeed_constraint = VariantArray(np.array([]), np.array([], dtype=np.uint8))
            self.vs = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.turndist = np.array([])  # [m] Distance when to turn to next waypoint
            self.flyby = np.array(
                [], dtype=bool
            )  # Flyby switch, when False, flyover (turndist=0.0)
            self.flyturn = np.array(
                [], dtype=bool
            )  # Flyturn switch, customised turn parameters; when False, use flyby/flyover
            self.turnrad = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.turn_cas = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.turnhdgr = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.old_turn_cas = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.turnfromlastwp = np.array(
                [], dtype=bool
            )  # Currently in flyturn-mode from last waypoint (old turn, beginning of leg)
            self.turntonextwp = np.array(
                [], dtype=bool
            )  # Currently in flyturn-mode to next waypoint (new flyturn mode, end of leg)
            self.torta = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.xtorta = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.next_qdr = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.swlastwp = np.array([], dtype=bool)  # switch indicating this is the last waypoint
            self.curlegdir = OptionalArray(np.array([]), np.array([], dtype=bool))
            self.curleglen = OptionalArray(np.array([]), np.array([], dtype=bool))

    def create(self, n: int = 1) -> None:
        """Initialize active-waypoint data for n newly created aircraft.

        Optional values start absent; required state uses neutral defaults until a route
        waypoint is activated.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)
        # LNAV route navigation
        self.lat[-n:] = 0.0  # [deg]Active WP latitude
        self.lon[-n:] = 0.0  # [deg]Active WP longitude
        self.turndist[-n:] = 1.0  # [m] Distance to active waypoint where to turn
        self.flyby[-n:] = 1.0  # Flyby/fly-over switch
        self.flyturn[-n:] = False  # Flyturn switch; False uses flyby/flyover
        self.turnfromlastwp[-n:] = (
            False  # Currently in flyturn-mode from last waypoint (old turn, beginning of leg)
        )
        self.turntonextwp[-n:] = (
            False  # Currently in flyturn-mode to next waypoint (new flyturn mode, end of leg)
        )
        self.swlastwp[-n:] = False  # Switch indicating active waypoint is last waypoint

    def new_implementation(self, implementation: type[TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's traffic object."""
        return implementation(self.traffic)

    def reached(
        self,
        qdr: q.BearingDeg[np.ndarray],
        dist: q.DistanceM[np.ndarray],
        flyby: np.ndarray,
        flyturn: np.ndarray,
        turnrad: OptionalArray[q.TurnRadiusM[np.ndarray]],
        turnhdgr: OptionalArray[q.TurnRateDegPerS[np.ndarray]],
        swlastwp: np.ndarray,
    ) -> np.ndarray:
        """Determine which aircraft have reached their active waypoint.

        Vectorized over all aircraft. A waypoint counts as reached when the
        aircraft is within the turn distance for the upcoming heading
        change, or when it has passed the waypoint (bearing to the waypoint
        differs more than 90 deg from the current leg direction, or the
        aircraft is within 4 s flying time while heading away). Only
        aircraft with LNAV engaged are considered. Also updates turndist.

        Args:
            qdr: Bearing from each aircraft to its active waypoint [deg].
            dist: Distance to the active waypoint [m].
            flyby: Fly-by switch per aircraft.
            flyturn: Fly-turn switch per aircraft.
            turnrad: Optional specified turn radius.
            turnhdgr: Optional specified turn heading rate.
            swlastwp: Switch: active waypoint is the last waypoint.

        Returns:
            ndarray: Indices of the aircraft that reached their waypoint.
        """
        # Calculate distance before waypoint where to start the turn
        # Note: this is a vectorized function, called with numpy traffic arrays
        # It returns the indices where the Reached criterion is True
        #
        # Turn radius:      R = V2 tan phi / g
        # Distance to turn: wpturn = R * tan (1/2 delhdg) but max 4 times radius
        # using default bank angle per flight phase

        # First calculate turn distance
        next_qdr = np.where(~self.next_qdr.present, qdr, self.next_qdr.values)
        has_turn_speed = self.turn_cas.present
        specified_turn_tas = vcas2tas(
            np.where(has_turn_speed, self.turn_cas.values, 0.0), self.traffic.alt
        )
        turntas = np.where(has_turn_speed, specified_turn_tas, self.traffic.tas)
        flybyturndist, turnrad = self.calcturn(
            turntas, self.traffic.ap.bankdef, qdr, next_qdr, turnrad, turnhdgr, flyturn
        )

        # Turb dist iz ero for flyover, calculated distance for others
        self.turndist = np.logical_or(flyby, flyturn) * flybyturndist

        # Avoid circling by checking too close to waypoint based on ground speed, assumption using vicinity criterion:
        # flying away and within 4 sec distance based on ground speed (4 sec = sensitivity tuning parameter)

        close2wp = (
            dist / (np.maximum(0.0001, np.abs(self.traffic.gs))) < 4.0
        )  # Waypoint is within 4 seconds flight time
        tooclose2turn = close2wp * (np.abs(degto180(self.traffic.trk % 360.0 - qdr % 360.0)) > 90.0)

        # When too close to waypoint or we have passed the active waypoint, based on leg direction,switch active waypoint
        # was:  away  = np.logical_or(close2wp,swlastwp)*(np.abs(degto180(self.traffic.trk%360. - qdr%360.)) > 90.) # difference large than 90
        curlegdir = np.where(~self.curlegdir.present, qdr, self.curlegdir.values)
        awayorpassed = np.logical_or(tooclose2turn, np.abs(degto180(qdr - curlegdir)) > 90.0)

        # Should no longer be needed with leg direction
        # Ratio between distance close enough to switch to next wp when flying away
        # When within pro1 nm and flying away: switch also
        # proxfact = 1.02 # Turnradius scales this contant , factor => [turnrad]
        # incircle = dist<turnrad*proxfact
        # circling = away*incircle # [True/False] passed wp,used for flyover as well

        # Check whether shift based dist is required, set closer than WP turn distance
        # Detect indices
        # swreached = np.where(self.traffic.swlnav * np.logical_or(awayorpassed,np.logical_or(dist < self.turndist,circling)))[0]
        swreached = np.where(
            self.traffic.swlnav * np.logical_or(awayorpassed, dist < self.turndist)
        )[0]

        # Return indices for which condition is True/1.0 for a/c where we have reached waypoint
        return swreached

    class TurnGeometry(NamedTuple):
        distance: q.DistanceM
        """Turn-initiation distance [m]."""
        radius: q.TurnRadiusM
        """Turn radius [m]."""

    # Calculate turn distance for array or scalar
    def calcturn(
        self,
        tas: q.TrueAirspeedMps,
        bank: q.BankAngleRad,
        wpqdr: q.BearingDeg,
        next_wpqdr: q.BearingDeg,
        turnrad: OptionalArray[q.TurnRadiusM[np.ndarray]],
        turnhdgr: OptionalArray[q.TurnRateDegPerS[np.ndarray]],
        flyturn: np.ndarray,
    ) -> TurnGeometry:
        """Calculate the turn-initiation distance and turn radius.

        Works on scalars as well as numpy arrays. The turn radius follows,
        in order of priority, from a user-specified radius (fly-turn mode),
        a specified heading rate, or the bank-angle limit with the given
        speed. The turn distance is the distance before the waypoint at
        which the turn must start to roll out on the next leg:
        R * tan(delta_hdg / 2).

        `turnrad` and `turnhdgr` are absent when unspecified; `flyturn` selects whether
        explicit turn parameters are active.
        """

        # Tas is also used ti

        # Calculate turn radius in meters using current true airspeed or use specified turnradius in m
        has_radius = np.logical_and(flyturn, turnrad.present)
        has_heading_rate = np.logical_and(flyturn, turnhdgr.present)
        radius = np.where(
            has_radius,
            turnrad.values,
            np.where(
                has_heading_rate,
                tas / (2 * np.pi) * (360.0 / np.where(has_heading_rate, turnhdgr.values, 1.0)),
                # bank, tas => turn radius
                tas * tas / (np.maximum(0.01, np.tan(bank)) * g0),
            ),
        )

        # turndist is in meters
        turndist = np.abs(
            radius * np.tan(np.radians(0.5 * np.abs(degto180(wpqdr % 360.0 - next_wpqdr % 360.0))))
        )
        return self.TurnGeometry(turndist, radius)
