"""Active waypoint data for FMS guidance.

Holds, as per-aircraft numpy arrays, all data of the waypoint each aircraft
is currently flying towards. The [`ActiveWaypoint`][.ActiveWaypoint] arrays form the
interface between the per-aircraft [`Route`][minisky.traffic.route.Route]
objects (event-driven, scalar waypoint switching) and the vectorized
LNAV/VNAV guidance in [`Autopilot`][minisky.traffic.autopilot.Autopilot].
Available as [`runtime.traffic.actwp`][.ActiveWaypoint].
"""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

import numpy as np

from minisky import quantities as q
from minisky.core.trafficarrays import OptionalArray, TrafficArrays, VariantArray
from minisky.tools.aero import g0, vcas2tas
from minisky.tools.convert import degto180

if TYPE_CHECKING:
    from minisky.traffic.traffic import Traffic


class ActiveWaypoint(TrafficArrays):
    """Per-aircraft data of the active (and next) waypoint.

    The autopilot copies waypoint data from the route into these arrays upon
    waypoint switching (see [`Autopilot.wppassingcheck`][minisky.traffic.autopilot.Autopilot.wppassingcheck]
    and [`route.direct`][minisky.traffic.route.direct]), so continuous guidance
    can be vectorized. Simple optional per-aircraft values use `OptionalArray`;
    optional [`CAS` in m/s][minisky.types.CasMps]
    or [`Mach`][minisky.types.Mach] values use `VariantArray`.
    """

    def __init__(self, traffic: Traffic) -> None:
        super().__init__(traffic)
        self.traffic = traffic
        with self.settrafarrays():
            self.lat: q.LatitudeDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.lon: q.LongitudeDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.nextturnlat: OptionalArray[q.LatitudeDeg[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            self.nextturnlon: OptionalArray[q.LongitudeDeg[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            self.next_turn_cas: OptionalArray[q.CalibratedAirspeedMps[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            self.nextturnrad: OptionalArray[q.TurnRadiusM[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            self.nextturnhdgr: OptionalArray[q.TurnRateDegPerS[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            self.nextturnidx: OptionalArray[np.ndarray] = OptionalArray(
                np.array([], dtype=int), np.array([], dtype=bool)
            )
            """Route index of the next turn waypoint."""
            self.nextaltco: OptionalArray[q.PressureAltitudeM[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Optional altitude constraint ahead of the active waypoint."""
            self.xtoalt: OptionalArray[q.DistanceM[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Route distance from the active waypoint to the next altitude constraint."""
            self.next_airspeed: VariantArray[np.ndarray] = VariantArray(
                np.array([]), np.array([], dtype=np.uint8)
            )
            """Optional CAS or Mach target for the next leg."""
            self.airspeed: VariantArray[np.ndarray] = VariantArray(
                np.array([]), np.array([], dtype=np.uint8)
            )
            """Optional CAS or Mach target for the active leg."""
            self.airspeed_constraint: VariantArray[np.ndarray] = VariantArray(
                np.array([]), np.array([], dtype=np.uint8)
            )
            """Optional explicit CAS or Mach constraint at the active waypoint."""
            self.vs: OptionalArray[q.VerticalRateMps[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            self.turndist: q.DistanceM[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Distance before the active waypoint at which its turn starts."""
            self.flyby = np.array([], dtype=bool)
            """Whether the active waypoint is fly-by rather than fly-over."""
            self.flyturn = np.array([], dtype=bool)
            """Whether the active waypoint uses explicit turn parameters."""
            self.turnrad: OptionalArray[q.TurnRadiusM[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            self.turn_cas: OptionalArray[q.CalibratedAirspeedMps[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            self.turnhdgr: OptionalArray[q.TurnRateDegPerS[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            self.old_turn_cas: OptionalArray[q.CalibratedAirspeedMps[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """CAS held while completing the previous fly-turn."""
            self.turnfromlastwp = np.array([], dtype=bool)
            """Whether the aircraft is completing the previous waypoint's fly-turn."""
            self.turntonextwp = np.array([], dtype=bool)
            """Whether the aircraft is entering the next waypoint's fly-turn."""
            self.torta: OptionalArray[q.SimulationTimeS[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Optional required arrival time ahead of the active waypoint."""
            self.xtorta: OptionalArray[q.DistanceM[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Route distance from the active waypoint to the next RTA waypoint."""
            self.next_qdr: OptionalArray[q.BearingDeg[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Track angle of the leg after the active waypoint."""
            self.swlastwp = np.array([], dtype=bool)
            """Whether the active waypoint is the final route waypoint."""
            self.curlegdir: OptionalArray[q.BearingDeg[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Direction of the current leg captured when its waypoint was activated."""
            self.curleglen: OptionalArray[q.DistanceM[np.ndarray]] = OptionalArray(  # pyright: ignore[reportGeneralTypeIssues]
                np.array([]), np.array([], dtype=bool)
            )
            """Length of the current leg captured when its waypoint was activated."""

    def create(self, n: int = 1) -> None:
        """Initialize active-waypoint data for n newly created aircraft.

        Optional values start absent; required state uses neutral defaults until a route
        waypoint is activated.
        """
        super().create(n)
        self.lat[-n:] = 0.0
        self.lon[-n:] = 0.0
        self.turndist[-n:] = 1.0
        self.flyby[-n:] = 1.0
        self.flyturn[-n:] = False
        self.turnfromlastwp[-n:] = False
        self.turntonextwp[-n:] = False
        self.swlastwp[-n:] = False

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
            qdr: Bearing from each aircraft to its active waypoint.
            dist: Distance from each aircraft to its active waypoint.
            flyby: Per-aircraft fly-by mode.
            flyturn: Per-aircraft fly-turn mode.
            turnrad: Optional explicit turn radius.
            turnhdgr: Optional explicit turn heading rate.
            swlastwp: Whether the active waypoint is the final route waypoint.

        Returns:
            Integer indices of aircraft that reached or passed their active waypoint.
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

        # Check whether shift based dist is required, set closer than WP turn distance
        swreached = np.where(
            self.traffic.swlnav * np.logical_or(awayorpassed, dist < self.turndist)
        )[0]

        # Return indices for which condition is True/1.0 for a/c where we have reached waypoint
        return swreached

    class TurnGeometry(NamedTuple):
        distance: q.DistanceM
        """Turn-initiation distance."""
        radius: q.TurnRadiusM

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
