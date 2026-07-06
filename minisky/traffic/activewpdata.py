"""Active waypoint data for FMS guidance.

Holds, as per-aircraft numpy arrays, all data of the waypoint each aircraft
is currently flying towards. The :class:`ActiveWaypoint` arrays form the
interface between the per-aircraft :class:`~minisky.traffic.route.Route`
objects (event-driven, scalar waypoint switching) and the vectorized
LNAV/VNAV guidance in :class:`~minisky.traffic.autopilot.Autopilot`.
Available at runtime as ``minisky.traf.actwp``.
"""

from typing import Any

import numpy as np

import minisky
from minisky.core.trafficarrays import TrafficArrays
from minisky.tools.aero import g0
from minisky.tools.convert import degto180


class ActiveWaypoint(TrafficArrays):
    """Per-aircraft data of the active (and next) waypoint.

    The autopilot copies waypoint data from the route into these arrays
    upon waypoint switching (see Autopilot.wppassingcheck() and
    route.direct()), so the continuous guidance can be vectorized. In most
    arrays a negative sentinel value (-999) means "not specified".

    Attributes:
        lat (ndarray): Active waypoint latitude [deg].
        lon (ndarray): Active waypoint longitude [deg].
        nextturnlat (ndarray): Next turn waypoint latitude [deg].
        nextturnlon (ndarray): Next turn waypoint longitude [deg].
        nextturnspd (ndarray): Next turn waypoint turn speed (CAS) [m/s].
        nextturnrad (ndarray): Next turn waypoint turn radius [m].
        nextturnhdgr (ndarray): Next turn waypoint heading rate [deg/s].
        nextturnidx (ndarray): Route index of the next turn waypoint.
        nextaltco (ndarray): Next altitude constraint [m].
        xtoalt (ndarray): Distance to the next altitude constraint [m].
        nextspd (ndarray): Speed for the next leg, from the current
            waypoint: CAS [m/s] or Mach [-].
        spd (ndarray): Active speed command (constraint or computed):
            CAS [m/s] or Mach [-].
        spdcon (ndarray): Active waypoint speed constraint:
            CAS [m/s] or Mach [-].
        vs (ndarray): Vertical speed to use in VNAV climb/descent [m/s].
        turndist (ndarray): Distance before the waypoint at which to start
            the turn [m].
        flyby (ndarray): Fly-by switch; when False, fly-over (turndist 0).
        flyturn (ndarray): Fly-turn switch (use specified turn parameters).
        turnrad (ndarray): Turn radius at the active waypoint [m].
        turnspd (ndarray): Turn speed (CAS) at the active waypoint [m/s].
        turnhdgr (ndarray): Turn heading rate at the active waypoint
            [deg/s].
        oldturnspd (ndarray): Turn speed of the previous turn (TAS) [m/s].
        turnfromlastwp (ndarray): In fly-turn mode from the last waypoint
            (old turn, beginning of leg).
        turntonextwp (ndarray): In fly-turn mode towards the next waypoint
            (new turn, end of leg).
        torta (ndarray): Next required time of arrival [s] (-999 = none).
        xtorta (ndarray): Distance to the next RTA waypoint [m].
        next_qdr (ndarray): Track angle of the next leg [deg].
        swlastwp (ndarray): Bool switch: active waypoint is the last one.
        curlegdir (ndarray): Direction of the current leg, set when the
            waypoint was activated [deg].
        curleglen (ndarray): Length of the current leg, set when the
            waypoint was activated [m].
    """

    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            self.lat = np.array([])  # [deg] Active WP latitude
            self.lon = np.array([])  # [deg] Active WP longitude
            self.nextturnlat = np.array([])  # [deg] Next turn WP latitude
            self.nextturnlon = np.array([])  # [deg] Next turn WP longitude
            self.nextturnspd = np.array([])  # [m/s] Next turn WP speed
            self.nextturnrad = np.array([])  # [m]   Next turn WP turn radius
            self.nextturnhdgr = np.array(
                []
            )  # [deg/s] Next turn WP heading rate (<0 => not specified)
            self.nextturnidx = np.array([])  # [-]   Next turn WP index
            self.nextaltco = np.array([])  # [m] Altitude to arrive at after distance xtoalt
            self.xtoalt = np.array([])  # [m] Distance to next altitude constraint
            self.nextspd = np.array([])  # [CAS[m/s]/Mach] save speed from next wp for next leg
            self.spd = np.array([])  # [CAS[m/s]/Mach]Active WP speed (constraint or calculated)
            self.spdcon = np.array([])  # [CAS[m/s]/Mach]Active WP speed constraint
            self.vs = np.array([])  # [m/s] Active vertical speed to use
            self.turndist = np.array([])  # [m] Distance when to turn to next waypoint
            self.flyby = np.array([])  # Flyby switch, when False, flyover (turndist=0.0)
            self.flyturn = np.array(
                []
            )  # Flyturn switch, customised turn parameters; when False, use flyby/flyover
            self.turnrad = np.array([])  # Flyturn turn radius (<0 => not specified)
            self.turnspd = np.array(
                []
            )  # [m/s, CAS] Flyturn turn speed for next turn (<=0 => not specified)
            self.turnhdgr = np.array([])  # [deg/s]Flyturn turn heading rate (<0 => not specified)
            self.oldturnspd = np.array(
                []
            )  # [TAS, m/s] Flyturn turn speed for previous turn (<=0 => not specified)
            self.turnfromlastwp = np.array(
                []
            )  # Currently in flyturn-mode from last waypoint (old turn, beginning of leg)
            self.turntonextwp = np.array(
                []
            )  # Currently in flyturn-mode to next waypoint (new flyturn mode, end of leg)
            self.torta = np.array([])  # [s] Next req Time of Arrival (RTA) (-999. = None)
            self.xtorta = np.array([])  # [m] distance to next RTA
            self.next_qdr = np.array([])  # [deg] track angle of next leg
            self.swlastwp = np.array([], dtype=bool)  # switch indicating this is the last waypoint
            self.curlegdir = np.array([])  # [deg] direction to active waypoint upon activation
            self.curleglen = np.array([])  # [deg] direction to active waypoint upon activation

    def create(self, n: int = 1) -> None:
        """Initialize active-waypoint data for n newly created aircraft.

        All values are set to their "not specified" sentinels (-999) or
        neutral defaults until a route waypoint is activated.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)
        # LNAV route navigation
        self.lat[-n:] = 0.0  # [deg]Active WP latitude
        self.lon[-n:] = 0.0  # [deg]Active WP longitude
        self.nextturnlat[-n:] = 0  # [deg] Next turn WP latitude
        self.nextturnlon[-n:] = 0  # [deg] Next turn WP longitude
        self.nextturnspd[-n:] = -999.0  # [m/s] Next turn WP speed
        self.nextturnrad[-n:] = -999.0  # [m]   Next turn WP radius
        self.nextturnhdgr[-n:] = -999.0  # [deg/s] Next turn WP heading rate (<0 => not specified)
        self.nextturnidx[-n:] = -999.0  # [-] Next turn WP index
        self.nextaltco[-n:] = -999.0  # [m] Altitude to arrive at after distance xtoalt
        self.xtoalt[-n:] = 0.0  # [m] Distance to next altitude constraint
        self.nextspd[-n:] = -999.0  # [CAS[m/s]/Mach]Next leg speed from current WP
        self.spd[-n:] = -999.0  # [CAS[m/s]/Mach]Active WP speed
        self.spdcon[-n:] = -999.0  # [CAS[m/s]/Mach]Active WP speed constraint
        self.turndist[-n:] = 1.0  # [m] Distance to active waypoint where to turn
        self.flyby[-n:] = 1.0  # Flyby/fly-over switch
        self.flyturn[-n:] = False  # Flyturn switch, when False, when False, use flyby/flyover
        self.turnrad[-n:] = -999.0  # [m] Flyturn turn radius (<0 => not specified)
        self.turnspd[-n:] = -999.0  # [m/s]Flyturn turn speed (<0 => not specified)
        self.turnhdgr[-n:] = -999.0  # [deg/s]Flyturn turn heading rate (<0 => not specified)
        self.oldturnspd[
            -n:
        ] = -999.0  # [TAS, m/s] Flyturn turn speed for previous turn (<=0 => not specified)
        self.turnfromlastwp[-n:] = (
            False  # Currently in flyturn-mode from last waypoint (old turn, beginning of leg)
        )
        self.turntonextwp[-n:] = (
            False  # Currently in flyturn-mode to next waypoint (new flyturn mode, end of leg)
        )
        self.torta[-n:] = -999.0  # [s] Req Time of Arrival (RTA) for next wp (-999. = None)
        self.xtorta[-n:] = 0.0  # Distance to next RTA
        self.next_qdr[-n:] = -999.0  # [deg] bearing next leg
        self.swlastwp[-n:] = False  # Switch indicating active waypoint is last waypoint
        self.curlegdir[-n:] = -999.0  # [deg] direction to active waypoint upon activation
        self.curleglen[-n:] = -999.0  # [nm] distance to active waypoint upon activation

    def reached(
        self,
        qdr: Any,
        dist: np.ndarray,
        flyby: np.ndarray,
        flyturn: np.ndarray,
        turnrad: np.ndarray,
        turnhdgr: np.ndarray,
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
            turnrad: Specified turn radius [m] (<0 = not specified).
            turnhdgr: Specified turn heading rate [deg/s]
                (<0 = not specified).
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
        next_qdr = np.where(self.next_qdr < -900.0, qdr, self.next_qdr)
        turntas = np.where(self.turnspd < 0.0, minisky.traf.tas, self.turnspd)
        flybyturndist, turnrad = self.calcturn(
            turntas, minisky.traf.ap.bankdef, qdr, next_qdr, turnrad, turnhdgr, flyturn
        )

        # Turb dist iz ero for flyover, calculated distance for others
        self.turndist = np.logical_or(flyby, flyturn) * flybyturndist

        # Avoid circling by checking too close to waypoint based on ground speed, assumption using vicinity criterion:
        # flying away and within 4 sec distance based on ground speed (4 sec = sensitivity tuning parameter)

        close2wp = (
            dist / (np.maximum(0.0001, np.abs(minisky.traf.gs))) < 4.0
        )  # Waypoint is within 4 seconds flight time
        tooclose2turn = close2wp * (np.abs(degto180(minisky.traf.trk % 360.0 - qdr % 360.0)) > 90.0)

        # When too close to waypoint or we have passed the active waypoint, based on leg direction,switch active waypoint
        # was:  away  = np.logical_or(close2wp,swlastwp)*(np.abs(degto180(minisky.traf.trk%360. - qdr%360.)) > 90.) # difference large than 90
        awayorpassed = np.logical_or(
            tooclose2turn, np.abs(degto180(qdr - minisky.traf.actwp.curlegdir)) > 90.0
        )

        # Should no longer be needed with leg direction
        # Ratio between distance close enough to switch to next wp when flying away
        # When within pro1 nm and flying away: switch also
        # proxfact = 1.02 # Turnradius scales this contant , factor => [turnrad]
        # incircle = dist<turnrad*proxfact
        # circling = away*incircle # [True/False] passed wp,used for flyover as well

        # Check whether shift based dist is required, set closer than WP turn distance
        # Detect indices
        # swreached = np.where(minisky.traf.swlnav * np.logical_or(awayorpassed,np.logical_or(dist < self.turndist,circling)))[0]
        swreached = np.where(
            minisky.traf.swlnav * np.logical_or(awayorpassed, dist < self.turndist)
        )[0]

        # Return indices for which condition is True/1.0 for a/c where we have reached waypoint
        return swreached

    # Calculate turn distance for array or scalar
    def calcturn(
        self,
        tas: Any,
        bank: Any,
        wpqdr: Any,
        next_wpqdr: Any,
        turnrad: Any = -999.0,
        turnhdgr: Any = -999.0,
        flyturn: Any = False,
    ) -> tuple:
        """Calculate the turn-initiation distance and turn radius.

        Works on scalars as well as numpy arrays. The turn radius follows,
        in order of priority, from a user-specified radius (fly-turn mode),
        a specified heading rate, or the bank-angle limit with the given
        speed. The turn distance is the distance before the waypoint at
        which the turn must start to roll out on the next leg:
        R * tan(delta_hdg / 2).

        Args:
            tas: True airspeed [m/s].
            bank: Bank angle limit [rad].
            wpqdr: Bearing to the active waypoint [deg].
            next_wpqdr: Bearing of the next leg [deg].
            turnrad: Specified turn radius [m] (<0 = not specified).
            turnhdgr: Specified turn heading rate [deg/s]
                (<0 = not specified).
            flyturn: Fly-turn switch (use the specified turn parameters).

        Returns:
            tuple: (turn distance [m], turn radius [m]).
        """

        # Tas is also used ti

        # Calculate turn radius in meters using current speed or use specified turnradius in m
        turnrad = np.where(
            np.logical_and(
                flyturn, turnrad + 0.0 * tas > 0.0
            ),  # turn radius specified? (0.*tas for dimension)
            # user specified radius
            turnrad + 0.0 * tas,
            np.where(
                np.logical_and(flyturn, turnhdgr + 0.0 * tas > 0),
                # turn radius based on heading rate?
                tas / (2 * np.pi) * (360.0 / turnhdgr),
                # bank, tas => turn radius
                tas * tas / (np.maximum(0.01, np.tan(bank)) * g0),
            ),
        )  # else none specified, calculate

        # turndist is in meters
        turndist = np.abs(
            turnrad * np.tan(np.radians(0.5 * np.abs(degto180(wpqdr % 360.0 - next_wpqdr % 360.0))))
        )
        return turndist, turnrad
