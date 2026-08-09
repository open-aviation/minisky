"""Autopilot Implementation.

Contains the [`Autopilot`][minisky.traffic.autopilot.Autopilot] class, which combines classic autopilot
modes (selected heading, altitude, vertical speed and speed) with FMS
guidance along the aircraft route: LNAV (lateral navigation towards the
active waypoint, including fly-by/fly-over/fly-turn logic) and VNAV
(Top-of-Climb/Top-of-Descent logic, altitude and speed constraints, and
required-time-of-arrival (RTA) speed scheduling).

The autopilot output (commanded track, speed, altitude and vertical speed)
is combined with conflict-resolution commands in
[`APorASAS`][minisky.traffic.aporasas.APorASAS] before being flown by
[`Traffic`][minisky.traffic.traffic.Traffic]. Many methods implement stack
commands (ALT, VS, HDG, SPD, DEST, ORIG, LNAV, VNAV, SWTOC, SWTOD).
"""

from __future__ import annotations

from collections.abc import Callable
from math import sqrt
from typing import TYPE_CHECKING, Any

import numpy as np

from minisky.command import (
    AcId,
    AcIdSelection,
    AltM,
    CoordinateWaypoint,
    HeadingDeg,
    LatLonDegrees,
    MagneticHeadingDeg,
    NamedWaypoint,
    OnOff,
    SpeedMpsOrMach,
    VspdMps,
    WaypointSpec,
    Wpt,
    command,
)
from minisky.core.trafficarrays import TrafficArrays
from minisky.result import Err, Ok, Result
from minisky.tools import geo
from minisky.tools.aero import (
    fpm,
    ft,
    g0,
    kts,
    nm,
    tas2cas,
    vcas2tas,
    vcasormach2tas,
)
from minisky.tools.convert import degto180
from minisky.tools.position import txt2pos

from .route import Route, direct

if TYPE_CHECKING:
    from minisky.simulation import Simulation
    from minisky.traffic import Traffic


def _waypoint_name(waypoint: WaypointSpec) -> str:
    match waypoint:
        case NamedWaypoint(name):
            return name
        case CoordinateWaypoint(_, source):
            return source


def _resolve_waypoint(
    traffic: Traffic, acidx: int, route: Route, waypoint: WaypointSpec
) -> Result[LatLonDegrees, str]:
    if isinstance(waypoint, CoordinateWaypoint):
        return Ok(waypoint.coordinates)

    name = waypoint.name
    apidx = traffic.navigation.getaptidx(name)
    if apidx >= 0:
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
    """BlueSky Autopilot implementation.

    Computes, per aircraft, the commanded track, altitude, vertical speed
    and speed from the selected (pilot) values and, when LNAV/VNAV are
    engaged, from the route stored in the per-aircraft [`Route`][minisky.traffic.route.Route]
    objects. Waypoint switching is event driven (see wppassingcheck()),
    while the continuous guidance in update() is fully vectorized over all
    aircraft. Accessible as [`runtime.traffic.ap`][minisky.traffic.autopilot.Autopilot].

    Attributes:
        trk (ndarray): Commanded track angle [deg].
        spd (ndarray): Commanded speed, CAS [m/s] or Mach [-].
        tas (ndarray): Commanded true airspeed [m/s].
        alt (ndarray): Commanded altitude [m].
        vs (ndarray): Commanded vertical speed [m/s].
        swtoc (ndarray): Switch: Top-of-Climb logic (climb early) enabled.
        swtod (ndarray): Switch: Top-of-Descent logic (descend late) enabled.
        dist2vs (ndarray): Distance to the active waypoint at which the
            VNAV climb/descent should start [m].
        swvnavvs (ndarray): Switch: use the VNAV-computed vertical speed.
        vnavvs (ndarray): Vertical speed used in VNAV mode [m/s].
        qdr2wp (ndarray): Bearing to the active waypoint [deg].
        dist2wp (ndarray): Distance to the active waypoint [m].
        qdrturn (ndarray): Bearing to the next turn waypoint [deg].
        dist2turn (ndarray): Distance to the next turn waypoint [m].
        inturn (ndarray): Switch: aircraft is currently in a turn.
        orig (list): Origin airport identifier per aircraft.
        dest (list): Destination airport identifier per aircraft.
        bankdef (ndarray): Default bank angle limit [rad].
        vsdef (ndarray): Default vertical speed [m/s].
        turnphi (ndarray): Bank angle used in the current turn [rad].
        route (list): Per-aircraft [`Route`][minisky.traffic.route.Route] (flight plan) objects.
        steepness (float): Default climb/descent gradient [-]
            (3000 ft per 10 nm).
        idxreached (list): Indices of aircraft that reached their active
            waypoint during the last update.
    """

    def __init__(self, traffic: Traffic, get_simulation: Callable[[], Simulation]) -> None:
        super().__init__()
        self.traffic = traffic
        self.navigation = traffic.navigation
        self._get_simulation = get_simulation

        # Standard descent steepness
        self.steepness = 3000.0 * ft / (10.0 * nm)

        # Define object arrays
        with self.settrafarrays():
            # FMS directions
            self.trk = np.array([])
            self.spd = np.array([])
            self.tas = np.array([])
            self.alt = np.array([])
            self.vs = np.array([])

            # -- VNAV variables --
            # Switch to enable Top of Climb logic (default True)
            self.swtoc = np.array([])

            # Switch to enable Top of Descent logic (default True)
            self.swtod = np.array([])

            # Distance from current waypoint to Top of Descent
            self.dist2vs = np.array([])

            # Switch to use provided vertical speed
            self.swvnavvs = np.array([])

            # Vertical speed in VNAV mode
            self.vnavvs = np.array([])

            # -- LNAV variables --

            # Bearing to waypoint from last check point
            # used to prevent 180-degree turns when bearing updates shortly before passing waypoint
            self.qdr2wp: np.ndarray = np.array([])

            # Distance to active waypoint [m]
            self.dist2wp: np.ndarray = np.array([])

            # Bearing to next turn
            self.qdrturn = np.array([])

            # Distance to next turn [m]
            self.dist2turn = np.array([])

            # Aircraft turning status
            self.inturn = np.array([])

            # Traffic navigation information
            self.orig = []  # Origin airport code (4 letters)
            self.dest = []  # Destination airport code (4 letters)

            # Default values
            self.bankdef = np.array([])  # Default bank angle [radians]
            self.vsdef = np.array([])  # Default vertical speed [m/s]

            # Currently used bank angle [rad]
            self.turnphi = np.array([])  # Current bank angle setting

            # Route objects
            self.route = []

        self.idxreached = []  # Indices of aircraft that have reached their active waypoint

    @property
    def simulation(self) -> Simulation:
        """Return the simulation that owns this autopilot."""
        return self._get_simulation()

    def new_implementation(self, implementation: Callable[..., TrafficArrays]) -> TrafficArrays:
        """Construct a replacement with this runtime's dependencies."""
        return implementation(self.traffic, self._get_simulation)

    def create(self, n: int = 1) -> None:
        """Initialize autopilot state for n newly created aircraft.

        Copies the initial track, speed and altitude from the traffic
        arrays, enables ToC/ToD logic, sets the default vertical speed
        (1500 fpm) and bank limit (25 deg), and creates an empty Route
        object for each new aircraft.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)

        # FMS directions
        self.trk[-n:] = self.traffic.trk[-n:]
        self.tas[-n:] = self.traffic.tas[-n:]
        self.alt[-n:] = self.traffic.alt[-n:]
        self.vs[-n:] = -999

        # Default ToC/ToD logic on
        self.swtoc[-n:] = True
        self.swtod[-n:] = True

        # VNAV Variables
        self.dist2vs[-n:] = -999.0

        # LNAV variables

        # Direction to waypoint from the last time passing was checked
        self.qdr2wp[-n:] = -999.0

        # Distance to go to next waypoint [nm]
        self.dist2wp[-n:] = -999.0

        # Traffic performance data (temporarily default values)

        # default vertical speed of autopilot
        self.vsdef[-n:] = 1500.0 * fpm

        self.bankdef[-n:] = np.radians(25.0)

        # Route objects
        for ridx, acid in enumerate(self.traffic.callsign[-n:]):
            self.route[ridx - n] = Route(self.traffic, acid)

    def wppassingcheck(self, qdr: Any, dist: Any) -> None:
        """
        The actwp is the interface between the list of waypoint data in the route object and the autopilot guidance
        when LNAV is on (heading) and optionally VNAV is on (speed & altitude)

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

        Args:
            qdr: Bearing from each aircraft to its active waypoint [deg];
                updated in place for aircraft that switch waypoint.
            dist: Distance from each aircraft to its active waypoint [m].
        """

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

        # Save current waypoint speed for use on next leg when we pass this waypoint
        # VNAV speeds are always FROM-speeds, so we accelerate/decelerate at the waypoint
        # where this speed is specified, so we need to save it for use now
        # before getting the new data for the next waypoint

        # Get speed for next leg from the waypoint we pass now and set as active speed
        actwp.spd[self.idxreached] = actwp.nextspd[self.idxreached]
        actwp.spdcon[self.idxreached] = actwp.nextspd[self.idxreached]

        # Event-driven part, per aircraft: stack commands attached to the passed
        # waypoint and route iteration. These mutate the Route objects and queue
        # stack commands, so they cannot be vectorized. Gather the returned
        # scalar waypoint data in rows for the vectorized leg update below.
        idxlast = []  # reached aircraft already at their last waypoint
        idxnext = []  # reached aircraft with a next waypoint to activate
        wpdata = []  # per aircraft in idxnext: getnextwp() + getnextturnwp() data
        for i in self.idxreached:
            # Execute stack commands for the still active waypoint, which we pass now
            self.route[i].runactwpstack()

            if actwp.swlastwp[i]:
                # Prevent trying to activate the next waypoint when it was already the last waypoint
                idxlast.append(i)
            else:
                # Get next waypoint. [m] note: xtoalt,nextaltco are in meters
                wpdata.append(
                    tuple(self.route[i].getnextwp()) + tuple(self.route[i].getnextturnwp())
                )
                idxnext.append(i)

        # In case of end of route/no more waypoints: switch off LNAV/VNAV
        if idxlast:
            last = np.array(idxlast)
            self.traffic.swlnav[last] = False
            self.traffic.swvnav[last] = False
            self.traffic.swvnavspd[last] = False

        # Vectorized leg data update for guidance, over the aircraft that
        # switched to a new waypoint
        if idxnext:
            nxt = np.array(idxnext)
            (
                lat,
                lon,
                alt,
                nextspd,
                xtoalt,
                toalt,
                xtorta,
                torta,
                lnavon,
                flyby,
                flyturn,
                turnrad,
                turnspd,
                turnhdgr,
                nextleglat,
                nextleglon,
                swlastwp,
                nextturnlat,
                nextturnlon,
                nextturnspd,
                nextturnrad,
                nextturnhdgr,
                nextturnidx,
            ) = (np.array(col) for col in zip(*wpdata, strict=True))
            # NOTE(abraham): keeping legacy for ActiveWaypoint
            alt = np.fromiter((-999.0 if value is None else value for value in alt), dtype=float)
            nextspd = np.fromiter(
                (-999.0 if value is None else value for value in nextspd), dtype=float
            )
            lnavon = lnavon.astype(bool)
            flyturn = flyturn.astype(bool)

            # Bearing of the leg after the new active waypoint, batched over
            # all switching aircraft (-999.0 sentinel when there is no next
            # leg; dummy coordinates keep the masked lanes NaN-free)
            has_nextleg = nextleglat > -900.0
            batched_qdr, _ = geo.qdrdist(
                lat,
                lon,
                np.where(has_nextleg, nextleglat, lat),
                np.where(has_nextleg, nextleglon, lon),
            )
            next_qdr = np.where(has_nextleg, batched_qdr, -999.0)

            actwp.nextspd[nxt] = nextspd
            actwp.xtorta[nxt] = xtorta
            actwp.torta[nxt] = torta
            actwp.next_qdr[nxt] = next_qdr
            actwp.swlastwp[nxt] = swlastwp.astype(bool)
            actwp.nextturnlat[nxt] = nextturnlat
            actwp.nextturnlon[nxt] = nextturnlon
            actwp.nextturnspd[nxt] = nextturnspd
            actwp.nextturnrad[nxt] = nextturnrad
            actwp.nextturnhdgr[nxt] = nextturnhdgr
            actwp.nextturnidx[nxt] = nextturnidx

            tas = self.traffic.tas[nxt]

            # Special turns: specified by turn radius or bank angle
            # If no turn speed specified, use current speed
            turnspd = np.where(flyturn & (turnspd <= 0.0), tas, turnspd)
            # Heading rate overrides turn radius
            turnrad = np.where(
                flyturn & (turnhdgr > 0.0), tas * 360.0 / (2.0 * np.pi * turnhdgr), turnrad
            )

            # Use last turn radius for bank angle in current turn
            # (old values, from the waypoint we pass now; fancy indexing copies)
            oldturnrad = actwp.turnrad[nxt]
            oldturnspd = actwp.turnspd[nxt]
            useoldturn = flyturn & (oldturnrad > 0.0)
            self.turnphi[nxt] = np.where(
                useoldturn,
                np.arctan(oldturnspd * oldturnspd / (np.where(useoldturn, oldturnrad, 1.0) * g0)),
                0.0,
            )  # [rad]

            # Check LNAV switch returned by getnextwp
            # Switch off LNAV if it failed to get next waypoint data
            lnavoff = ~lnavon & self.traffic.swlnav[nxt]
            # Last waypoint: copy last waypoint values for altitude and speed in autopilot
            uselastspd = lnavoff & self.traffic.swvnavspd[nxt] & (nextspd >= 0.0)
            self.traffic.selspd[nxt] = np.where(uselastspd, nextspd, self.traffic.selspd[nxt])
            self.traffic.swlnav[nxt] = self.traffic.swlnav[nxt] & lnavon

            # In case of no LNAV, do not allow VNAV mode to be active
            self.traffic.swvnav[nxt] = self.traffic.swvnav[nxt] & self.traffic.swlnav[nxt]

            actwp.lat[nxt] = lat  # [deg]
            actwp.lon[nxt] = lon  # [deg]
            # 1.0 in case of fly by, else fly over
            actwp.flyby[nxt] = flyby

            # Update qdr and turn distance for this new waypoint for ComputeVNAV
            qdrnxt, distnmi = geo.qdrdist(self.traffic.lat[nxt], self.traffic.lon[nxt], lat, lon)
            qdr[nxt] = qdrnxt
            self.dist2wp[nxt] = distnmi * nm

            actwp.curlegdir[nxt] = qdrnxt
            actwp.curleglen[nxt] = self.dist2wp[nxt]

            # User has entered an altitude for the new waypoint:
            # positive altitude on this waypoint means altitude constraint
            altco = alt >= -0.01
            actwp.nextaltco[nxt] = np.where(altco, alt, toalt)  # [m]
            actwp.xtoalt[nxt] = np.where(altco, 0.0, xtoalt)  # [m]

            # VNAV speed mode: use speed of this waypoint as commanded speed
            # while passing waypoint and save next speed for passing next waypoint
            # Speed is now from speed! Next speed is ready in waypoint data
            usewpspd = self.traffic.swvnavspd[nxt] & (actwp.spd[nxt] >= 0.0)
            self.traffic.selspd[nxt] = np.where(usewpspd, actwp.spd[nxt], self.traffic.selspd[nxt])

            # Update turn distance so ComputeVNAV works, is there a next leg direction or not?
            local_next_qdr = np.where(next_qdr < -900.0, qdrnxt, next_qdr)

            # Calculate turn distance (and radius which we do not use now, but later)
            actwp.turndist[nxt], _ = actwp.calcturn(
                tas, self.bankdef[nxt], qdrnxt, local_next_qdr, turnrad, turnhdgr, flyturn
            )  # update turn distance for VNAV

            # Get flyturn switches and data
            # old turn speed, turning by this waypoint
            actwp.oldturnspd[nxt] = oldturnspd
            actwp.flyturn[nxt] = flyturn
            actwp.turnrad[nxt] = turnrad
            actwp.turnhdgr[nxt] = turnhdgr
            # Keep both turning speeds: turn to leg and turn from leg
            actwp.turnspd[nxt] = np.where(flyturn, turnspd, -990.0)

            # Pass on whether currently flyturn mode:
            # at beginning of leg, copy to next waypoint to last waypoint
            # set next turn False
            actwp.turnfromlastwp[nxt] = actwp.turntonextwp[nxt]
            actwp.turntonextwp[nxt] = False

            # Reduce turn distance for reduced turn speed
            redturn = flyturn & (turnrad < 0.0) & (actwp.turnspd[nxt] >= 0.0)
            turntas = vcas2tas(np.where(redturn, actwp.turnspd[nxt], 0.0), self.traffic.alt[nxt])
            actwp.turndist[nxt] = actwp.turndist[nxt] * np.where(
                redturn, turntas * turntas / (tas * tas), 1.0
            )

            # VNAV = FMS ALT/SPD mode including RTA: still scalar, per aircraft
            for k, i in enumerate(idxnext):
                self.ComputeVNAV(i, toalt[k], actwp.xtoalt[i], actwp.torta[i], actwp.xtorta[i])

        # End of the waypoint switching update

        # Update qdr2wp with up-to-date qdr, now that we have checked passing waypoint
        self.qdr2wp = qdr % 360.0

        # Continuous guidance when speed constraint on active leg is in update-method

        # If still an RTA in the route and currently no speed constraint
        for iac in np.where((self.traffic.actwp.torta > -99.0) * (self.traffic.actwp.spdcon < 0.0))[
            0
        ]:
            iac = int(iac)
            iwp = self.route[iac].iactwp
            if self.route[iac].wprta[iwp] is not None:
                # For all aircraft flying to an RTA waypoint, recalculate speed more often
                dist2go4rta = (
                    geo.kwikdist(
                        self.traffic.lat[iac],
                        self.traffic.lon[iac],
                        self.traffic.actwp.lat[iac],
                        self.traffic.actwp.lon[iac],
                    )
                    * nm
                    + self.route[iac].wpxtorta[iwp]
                )  # last term zero for active waypoint RTA

                # Set self.traffic.actwp.spd to RTA speed, if necessary
                self.setspeedforRTA(iac, self.traffic.actwp.torta[iac], dist2go4rta)

                # If VNAV speed is on (by default coupled to VNAV), use it for speed guidance
                if self.traffic.swvnavspd[iac] and self.traffic.actwp.spd[iac] >= 0.0:
                    self.traffic.selspd[iac] = self.traffic.actwp.spd[iac]

    def update(self) -> None:
        """Run the continuous FMS/autopilot guidance for all aircraft.

        Called every simulation step. Recomputes bearing and distance to the
        active waypoints, performs the event-driven waypoint switching via
        wppassingcheck(), and then applies the vectorized guidance:

        - VNAV altitude guidance: engage climb/descent when within dist2vs
          of the active waypoint (using the vertical speed prepared by
          ComputeVNAV()).
        - LNAV track guidance: command the bearing to the active waypoint.
        - FMS speed guidance: anticipate deceleration for upcoming turn
          waypoints and acceleration/deceleration for speed constraints on
          the next leg, and select the appropriate CAS/Mach command.

        The results are stored in the commanded-state arrays (trk, alt, vs,
        tas) and in the traffic selected-state arrays where applicable.
        """
        # FMS LNAV mode:
        # qdr[deg],distinnm[nm]
        qdr, distinnm = geo.qdrdist(
            self.traffic.lat,
            self.traffic.lon,
            self.traffic.actwp.lat,
            self.traffic.actwp.lon,
        )  # [deg][nm])

        self.qdr2wp = np.asarray(qdr)
        self.dist2wp = np.asarray(distinnm) * nm  # Conversion to meters

        # Check possible waypoint shift. Note: qdr, dist2wp will be updated accordingly in case of waypoint switch
        self.wppassingcheck(qdr, self.dist2wp)  # Updates self.qdr2wp when necessary

        # ================= Continuous FMS guidance ========================

        # Note that the code below is vectorized, with traffic arrays, so for all aircraft
        # ComputeVNAV and inside waypoint loop of wppassingcheck, it was scalar (per aircraft with index i)

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

        startdescorclimb = (self.traffic.actwp.nextaltco >= -0.1) * np.logical_or(
            (self.traffic.alt > self.traffic.actwp.nextaltco)
            * np.logical_or(
                (self.dist2wp < self.dist2vs + self.traffic.actwp.turndist),
                (np.logical_not(self.swtod)),
            ),
            self.traffic.alt < self.traffic.actwp.nextaltco,
        )

        # print("self.dist2vs =",self.dist2vs)

        # If not LNAV: Climb/descend if doing so before LNAV/VNAV was switched off
        #    (because there are no more waypoints). This is needed
        #    to continue descending when you get into a conflict
        #    while descending to the destination (the last waypoint)
        #    Use 0.1 nm (185.2 m) circle in case turn distance might be zero
        self.swvnavvs = self.traffic.swvnav * np.where(
            self.traffic.swlnav,
            startdescorclimb,
            self.dist2wp <= np.maximum(0.1 * nm, self.traffic.actwp.turndist),
        )

        # Recalculate V/S based on current altitude and distance to next altitude constraint
        # How much time do we have before we need to descend?
        # Now done in ComputeVNAV
        # See ComputeVNAV for self.traffic.actwp.vs calculation

        self.vnavvs = np.where(self.swvnavvs, self.traffic.actwp.vs, self.vnavvs)
        # was: self.vnavvs  = np.where(self.swvnavvs, self.steepness * self.traffic.gs, self.vnavvs)

        # self.vs = np.where(self.swvnavvs, self.vnavvs, self.vsdef * self.traffic.limvs_flag)
        # for VNAV use fixed V/S and change start of descent
        selvs = np.where(abs(self.traffic.selvs) > 0.1, self.traffic.selvs, self.vsdef)  # m/s
        self.vs = np.where(self.swvnavvs, self.vnavvs, selvs)
        self.alt = np.where(self.swvnavvs, self.traffic.actwp.nextaltco, self.traffic.selalt)

        # When descending or climbing in VNAV also update altitude command of select/hold mode
        self.traffic.selalt = np.where(
            self.swvnavvs, self.traffic.actwp.nextaltco, self.traffic.selalt
        )

        # LNAV commanded track angle
        self.trk = np.where(self.traffic.swlnav, self.qdr2wp, self.trk)

        # FMS speed guidance: anticipate accel/decel distance for next leg or turn

        # Calculate actual distance it takes to decelerate/accelerate based on two cases: turning speed (decel)

        # Normally next leg speed (actwp.spd) but in case we fly turns with a specified turn speed
        # use the turn speed

        # Is turn speed specified and are we not already slow enough? We only decelerate for turns, not accel.
        turntas = np.where(
            self.traffic.actwp.nextturnspd > 0.0,
            vcas2tas(self.traffic.actwp.nextturnspd, self.traffic.alt),
            -1.0 + 0.0 * self.traffic.tas,
        )

        # Switch is now whether the aircraft has any turn waypoints
        swturnspd = self.traffic.actwp.nextturnidx > 0
        np.maximum(0.0, (self.traffic.tas - turntas) * (turntas > 0.0))

        # t = (v1-v0)/a ; x = v0*t+1/2*a*t*t => dx = (v1*v1-v0*v0)/ (2a)
        dxturnspdchg = distaccel(turntas, self.traffic.tas, self.traffic.perf.axmax)

        # Decelerate or accelerate for next required speed because of speed constraint or RTA speed
        # Note that because nextspd comes from the stack, and can be either a mach number or
        # a calibrated airspeed, it can only be converted from Mach / CAS [kts] to TAS [m/s]
        # once the altitude is known.
        nexttas = vcasormach2tas(
            self.traffic.actwp.nextspd,
            self.traffic.alt,
            self.traffic.casmach_threshold,
        )
        dxspdconchg = distaccel(self.traffic.tas, nexttas, self.traffic.perf.axmax)

        qdrturn, dist2turn = geo.qdrdist(
            self.traffic.lat,
            self.traffic.lon,
            self.traffic.actwp.nextturnlat,
            self.traffic.actwp.nextturnlon,
        )

        self.qdrturn = qdrturn
        dist2turn = dist2turn * nm

        # Where we don't have a turn waypoint, as in turn idx is negative, then put distance
        # as Earth circumference.
        self.dist2turn = np.where(self.traffic.actwp.nextturnidx > 0, dist2turn, 40075000)

        # Check also whether VNAVSPD is on, if not, SPD SEL has override for next leg
        # and same for turn logic
        usenextspdcon = (
            (self.dist2wp < dxspdconchg)
            * (self.traffic.actwp.nextspd > -990.0)
            * self.traffic.swvnavspd
            * self.traffic.swvnav
            * self.traffic.swlnav
        )

        useturnspd = (
            np.logical_or(
                self.traffic.actwp.turntonextwp,
                (self.dist2turn < (dxturnspdchg + self.traffic.actwp.turndist)),
            )
            * swturnspd
            * self.traffic.swvnavspd
            * self.traffic.swvnav
            * self.traffic.swlnav
        )

        # Hold turn mode can only be switched on here, cannot be switched off here (happeps upon passing wp)
        self.traffic.actwp.turntonextwp = self.traffic.swlnav * np.logical_or(
            self.traffic.actwp.turntonextwp, useturnspd
        )

        # Which CAS/Mach do we have to keep? VNAV, last turn or next turn?
        oncurrentleg = abs(degto180(self.traffic.trk - qdr)) < 2.0  # [deg]
        inoldturn = (self.traffic.actwp.oldturnspd > 0.0) * np.logical_not(oncurrentleg)

        # Avoid using old turning speeds when turning of this leg to the next leg
        # by disabling (old) turningspd when on leg
        self.traffic.actwp.oldturnspd = np.where(
            oncurrentleg * (self.traffic.actwp.oldturnspd > 0.0),
            -998.0,
            self.traffic.actwp.oldturnspd,
        )

        # turnfromlastwp can only be switched off here, not on (latter happens upon passing wp)
        self.traffic.actwp.turnfromlastwp = np.logical_and(
            self.traffic.actwp.turnfromlastwp, inoldturn
        )

        # Select speed: turn sped, next speed constraint, or current speed constraint
        self.traffic.selspd = np.where(
            useturnspd,
            self.traffic.actwp.nextturnspd,
            np.where(
                usenextspdcon,
                self.traffic.actwp.nextspd,
                np.where(
                    (self.traffic.actwp.spdcon >= 0) * self.traffic.swvnavspd,
                    self.traffic.actwp.spd,
                    self.traffic.selspd,
                ),
            ),
        )

        # Temporary override when still in old turn
        self.traffic.selspd = np.where(
            inoldturn
            * (self.traffic.actwp.oldturnspd > 0.0)
            * self.traffic.swvnavspd
            * self.traffic.swvnav
            * self.traffic.swlnav,
            self.traffic.actwp.oldturnspd,
            self.traffic.selspd,
        )

        self.inturn = np.logical_or(useturnspd, inoldturn)

        # Below crossover altitude: CAS=const, above crossover altitude: Mach = const
        self.tas = vcasormach2tas(
            self.traffic.selspd,
            self.traffic.alt,
            self.traffic.casmach_threshold,
        )

    def ComputeVNAV(self, idx: int, toalt: Any, xtoalt: Any, torta: Any, xtorta: Any) -> None:
        """
        This function to do VNAV (and RTA) calculations is only called only once per leg for an aircraft index.
        If:
         - switching to next waypoint
         - when VNAV is activated
         - when a DIRECT is given

        It prepares the profile of this leg using the the current altitude and the next altitude constraint (nextaltco).
        The distance to the next altitude constraint is given by xtoalt [m] after active waypoint.

        Options are (classic VNAV logic, swtoc and swtod True):
        - no altitude constraint in the future, do nothing
        - Top of CLimb logic (swtoc=True): if next altitude constrain is baove us, climb as soon as possible with default steepness
        - Top of Descent Logic (swtod =True) Use ToD logic: descend as late aspossible, based on
          steepness. Prepare a ToD somewhere on the leg if necessary based on distance to next altitude constraint.
          This is done by calculating distance to next waypoint where descent should start

        Alternative logic (e.g. for UAVs or GA):
        - swtoc=False and next alt co is above us, climb with the angle/steepness needed to arrive at the altitude at
        the waypoint with the altitude constraint (xtoalt m after active waypoint)
        - swtod=False and next altco is below us, descend with the angle/steepness needed to arrive at at the altitude at
        the waypoint with the altitude constraint (xtoalt m after active waypoint)

        Output if this function:
        self.dist2vs = distance 2 next waypoint where climb/descent needs to activated
        self.traffic.actwp.vs =  V/S to be used during climb/descent part, so when dist2wp<dist2vs [m] (to next waypoint)

        Args:
            idx: Aircraft index (scalar).
            toalt: Next altitude constraint [m] (negative = none).
            xtoalt: Distance from the active waypoint to that altitude
                constraint [m].
            torta: Next required time of arrival (RTA) as simulation time
                [s] (-999 = none).
            xtorta: Distance from the active waypoint to the RTA waypoint [m].
        """

        # print ("ComputeVNAV for",self.traffic.id[idx],":",toalt/ft,"ft  ",xtoalt/nm,"nm")
        # print("Called by",callstack()[1].function)

        # Check  whether active waypoint speed needs to be adjusted for RTA
        # sets self.traffic.actwp.spd, if necessary
        # debug print("xtorta+legdist =",(xtorta+legdist)/nm)
        self.setspeedforRTA(idx, torta, xtorta + self.dist2wp[idx])  # all scalar

        # Check if there is a target altitude and VNAV is on, else return doing nothing
        if toalt < 0 or not self.traffic.swvnav[idx]:
            self.dist2vs[
                idx
            ] = -999999.0  # dist to next wp will never be less than this, so VNAV will do nothing
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
        epsalt = 2.0 * ft  # deadzone
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
            self.traffic.actwp.nextaltco[idx] = toalt  # [m] next alt constraint
            self.traffic.actwp.xtoalt[idx] = (
                xtoalt  # [m] distance to next alt constraint measured from next waypoint
            )

            # VNAV ToD logic
            if self.swtod[idx]:
                # Get distance to waypoint
                self.dist2wp[idx] = nm * geo.kwikdist(
                    self.traffic.lat[idx],
                    self.traffic.lon[idx],
                    self.traffic.actwp.lat[idx],
                    self.traffic.actwp.lon[idx],
                )  # was not always up to date, so update first

                # Distance to next waypoint where we need to start descent (top of descent) [m]
                descdist = (
                    abs(self.traffic.alt[idx] - toalt) / self.steepness
                )  # [m] required length for descent, uses default steepness!
                self.dist2vs[idx] = descdist - xtoalt  # [m] part of that length on this leg

                # print(self.traffic.id[idx],"traf.alt =",self.traffic.alt[idx]/ft,"ft toalt = ",toalt/ft,"ft descdist =",descdist/nm,"nm")
                # print ("d2wp = ",self.dist2wp[idx]/nm,"nm d2vs = ",self.dist2vs[idx]/nm,"nm")
                # print("xtoalt =",xtoalt/nm,"nm descdist =",descdist/nm,"nm")

                # Exceptions: Descend now?
                if (
                    self.dist2wp[idx] - 1.02 * self.traffic.actwp.turndist[idx] < self.dist2vs[idx]
                ):  # Urgent descent, we're late![m]
                    # Descend now using whole remaining distance on leg to reach altitude
                    self.alt[idx] = self.traffic.actwp.nextaltco[
                        idx
                    ]  # dial in altitude of next waypoint as calculated
                    t2go = self.dist2wp[idx] / max(0.01, self.traffic.gs[idx])
                    self.traffic.actwp.vs[idx] = (self.traffic.alt[idx] - toalt) / max(0.01, t2go)

                elif xtoalt < descdist:  # Not on this leg, no descending is needed at next waypoint
                    # Top of decent needs to be on this leg, as next wp is in descent
                    self.traffic.actwp.vs[idx] = -abs(self.steepness) * (
                        self.traffic.gs[idx]
                        + (self.traffic.gs[idx] < 0.2 * self.traffic.tas[idx])
                        * self.traffic.tas[idx]
                    )

                else:
                    # else still level
                    self.traffic.actwp.vs[idx] = 0.0

            else:
                # We are higher but swtod = False, so there is no ToD descent logic, simply aim at next altco
                steepness_ = (self.traffic.alt[idx] - self.traffic.actwp.nextaltco[idx]) / (
                    max(0.01, self.dist2wp[idx] + xtoalt)
                )
                self.traffic.actwp.vs[idx] = -abs(steepness_) * (
                    self.traffic.gs[idx]
                    + (self.traffic.gs[idx] < 0.2 * self.traffic.tas[idx]) * self.traffic.tas[idx]
                )
                self.dist2vs[idx] = (
                    99999.0  # [m] Forces immediate descent as current distance to next wp will be less
                )

                # print("in else swtod for ", self.traffic.id[idx])

        # VNAV climb mode: climb as soon as possible (T/C logic)
        elif self.traffic.alt[idx] < toalt - 9.9 * ft:
            # Stop potential current descent (e.g. due to not making it to previous altco)
            # then stop immediately, as in: do not make it worse.
            if self.traffic.vs[idx] < -0.0001:
                self.vnavvs[idx] = 0.0
                self.alt[idx] = self.traffic.alt[idx]
                if self.traffic.swvnav[idx]:
                    self.traffic.selalt[idx] = self.traffic.alt[idx]

            # Altitude we want to climb to: next alt constraint in our route (could be further down the route)
            self.traffic.actwp.nextaltco[idx] = toalt  # [m]
            self.traffic.actwp.xtoalt[idx] = (
                xtoalt  # [m] distance to next alt constraint measured from next waypoint
            )
            self.alt[idx] = self.traffic.actwp.nextaltco[
                idx
            ]  # dial in altitude of next waypoint as calculated
            self.dist2vs[idx] = (
                99999.0  # [m] Forces immediate climb as current distance to next wp will be less
            )

            t2go = max(0.1, self.dist2wp[idx] + xtoalt) / max(0.01, self.traffic.gs[idx])
            if self.swtoc[idx]:
                steepness_ = self.steepness  # default steepness
            else:
                steepness_ = (self.traffic.alt[idx] - self.traffic.actwp.nextaltco[idx]) / (
                    max(0.01, self.dist2wp[idx] + xtoalt)
                )

            self.traffic.actwp.vs[idx] = np.maximum(
                steepness_ * self.traffic.gs[idx],
                (self.traffic.actwp.nextaltco[idx] - self.traffic.alt[idx]) / t2go,
            )  # [m/s]
        # Level leg: never start V/S
        else:
            self.dist2vs[idx] = -999.0  # [m]

        return

    def setspeedforRTA(self, idx: int, torta: Any, xtorta: float) -> float | bool:
        """Compute and set the speed required to meet an RTA constraint.

        Calculates the ground speed needed to cover the remaining distance
        to the RTA waypoint exactly at the required time (see calcvrta()),
        corrects for the tailwind component and converts to CAS. When no
        explicit speed constraint is active and VNAV speed guidance is on,
        the result is stored as the active waypoint speed command.

        Args:
            idx: Aircraft index (scalar).
            torta: Required time of arrival as simulation time [s]
                (-999 = no RTA).
            xtorta: Distance to go to the RTA waypoint [m].

        Returns:
            float or bool: Required CAS [m/s], or False when there is no
            (feasible) RTA.
        """
        # debug print("setspeedforRTA called, torta,xtorta =",torta,xtorta/nm)

        # Calculate required CAS to meet RTA
        # for aircraft nr. idx (scalar)
        if torta < -90.0:  # -999 signals there is no RTA defined in remainder of route
            return False

        deltime = torta - self.simulation.simt  # Remaining time to next RTA [s] in simtime
        if deltime > 0:  # Still possible?
            gsrta = calcvrta(self.traffic.gs[idx], xtorta, deltime, self.traffic.perf.axmax[idx])

            # Subtract tail wind speed vector
            tailwind = (
                self.traffic.windnorth[idx] * self.traffic.gsnorth[idx]
                + self.traffic.windeast[idx] * self.traffic.gseast[idx]
            ) / self.traffic.gs[idx]

            # Convert to CAS
            rtacas = tas2cas(gsrta - tailwind, self.traffic.alt[idx])

            # Performance limits on speed will be applied in traf.update
            if self.traffic.actwp.spdcon[idx] < 0.0 and self.traffic.swvnavspd[idx]:
                self.traffic.actwp.spd[idx] = rtacas
                # print("setspeedforRTA: xtorta =",xtorta)

            return rtacas
        else:
            return False

    @command(name="ALT")
    def selaltcmd(
        self, idx: AcIdSelection, alt: AltM, vspd: VspdMps | None = None
    ) -> Result[str, str]:
        """Select the autopilot altitude, optionally with a vertical speed.

        Implements the ALT stack command: `ALT acid, alt, [vspd]`.
        Selecting an altitude disengages VNAV for this aircraft. When no
        vertical speed is given and the currently selected vertical speed
        opposes the required climb/descent direction, it is reset so the
        default vertical speed is used.

        Args:
            idx: Aircraft index (or collection of indices).
            alt: Selected altitude [m] (stack input in ft/FL).
            vspd: Optional vertical speed [m/s] (stack input in fpm).
        """
        self.traffic.selalt[idx] = alt
        self.traffic.swvnav[idx] = False

        # Check for optional VS argument
        if vspd:
            self.traffic.selvs[idx] = vspd
        else:
            delalt = alt - self.traffic.alt[idx]
            # Check for VS with opposite sign => use default vs
            # by setting autopilot vs to zero
            oppositevs = np.logical_and(
                self.traffic.selvs[idx] * delalt < 0.0,
                abs(self.traffic.selvs[idx]) > 0.01,
            )

            self.traffic.selvs[idx[oppositevs]] = 0.0
        return Ok(f"altitude set to {alt / ft} ft")

    @command(name="VS")
    def selvspdcmd(self, idx: AcIdSelection, vspd: VspdMps) -> Result[str, str]:
        """Select the autopilot vertical speed.

        Implements the VS stack command: `VS acid, vspd (ft/min)`.
        Setting a vertical speed disengages VNAV for this aircraft.

        Args:
            idx: Aircraft index.
            vspd: Selected vertical speed [m/s] (stack input in fpm).
        """
        self.traffic.selvs[idx] = vspd
        self.traffic.swvnav[idx] = False
        return Ok(f"vertical speed set to {vspd / fpm} ft/min")

    @command(name="HDG", aliases=("HEADING", "TURN"))
    def selhdgcmd(self, idx: AcIdSelection, hdg: HeadingDeg) -> Result[str, str]:  # HDG command
        """Select the autopilot heading.

        Implements the HDG stack command: `HDG acid, hdg (deg)`. When a
        wind field is defined and the aircraft is airborne (above 50 ft),
        the commanded track is computed from the given heading and the local
        wind; otherwise track equals heading. Selecting a heading disengages
        LNAV for this aircraft.

        Args:
            idx: Aircraft index.
            hdg: Selected heading [deg].
        """

        resolved_hdg: float | np.ndarray[Any, Any]
        if isinstance(hdg, MagneticHeadingDeg):
            resolved_hdg = np.fromiter(
                (
                    (hdg.degrees + geo.magdec(float(lat), float(lon))) % 360.0
                    for lat, lon in zip(self.traffic.lat[idx], self.traffic.lon[idx], strict=True)
                ),
                dtype=float,
            )
        else:
            resolved_hdg = hdg.degrees

        if self.traffic.wind.winddim > 0:
            tasnorth = self.traffic.tas[idx] * np.cos(np.radians(resolved_hdg))
            taseast = self.traffic.tas[idx] * np.sin(np.radians(resolved_hdg))
            wind_north, wind_east = self.traffic.wind.getdata(
                self.traffic.lat[idx], self.traffic.lon[idx], self.traffic.alt[idx]
            )
            wind_track = np.degrees(np.arctan2(taseast + wind_east, tasnorth + wind_north)) % 360.0
            # Above 50ft: compute track based on wind
            # Below 50ft: track equals heading
            self.trk[idx] = np.where(self.traffic.alt[idx] > 50.0 * ft, wind_track, resolved_hdg)
        else:
            self.trk[idx] = resolved_hdg

        self.traffic.swlnav[idx] = False
        return Ok(f"heading set to {resolved_hdg} deg")

    @command(name="SPD", aliases=("SPEED",))
    def selspdcmd(
        self, idx: AcIdSelection, casmach: SpeedMpsOrMach
    ) -> Result[str, str]:  # SPD command
        """Select the autopilot speed.

        Implements the SPD stack command: `SPD acid, casmach`. Switches
        off VNAV speed guidance, as a manually selected speed overrides the
        FMS speed. Whether CAS or Mach is held during altitude changes
        depends on the position relative to the crossover altitude.

        Args:
            idx: Aircraft index.
            casmach: Selected speed: CAS [m/s] or Mach [-] (values above 1.0
                are interpreted as CAS; stack input in kts or Mach).
        """
        # Depending on or position relative to crossover altitude,
        # we will maintain CAS or Mach when altitude changes
        # We will convert values when needed
        self.traffic.selspd[idx] = casmach

        # Used to be: Switch off VNAV: SPD command overrides
        self.traffic.swvnavspd[idx] = False

        if casmach > 1.0:
            msg = f"speed set to {casmach / kts} kts"
        else:
            msg = f"speed set to Mach {casmach}"

        return Ok(msg)

    @command(name="DEST")
    def show_destination(self, acidx: AcId) -> Result[str, str]:
        """Show the destination of an aircraft."""
        return Ok(f"DEST {self.traffic.callsign[acidx]}: {self.dest[acidx]}")

    @command(name="DEST")
    def set_destination(
        self, acidx: AcId, waypoint: Wpt, casmach: SpeedMpsOrMach | None = None
    ) -> Result[str, str]:
        """Set the destination of an aircraft, with an optional speed constraint."""
        route = self.route[acidx]
        wpname = _waypoint_name(waypoint)
        if isinstance(position := _resolve_waypoint(self.traffic, acidx, route, waypoint), Err):
            return Err("DEST: " + position.err())
        coordinates = position.ok()
        self.dest[acidx] = wpname
        iwp = route.add_waypoint(
            acidx, self.dest[acidx], route.dest, coordinates.lat, coordinates.lon, 0.0, casmach
        )
        # If only waypoint: activate
        if (iwp == 0) or (self.orig[acidx] != "" and len(route.wpname) == 2):
            self.traffic.swlnav[acidx] = True
            self.traffic.swvnav[acidx] = True
            route.iactwp = iwp
            direct(self.traffic, acidx, route.wpname[iwp])

        # If not found, say so
        elif iwp < 0:
            return Err("DEST position" + self.dest[acidx] + " not found.")
        return Ok(f"destination set to {wpname}")

    @command(name="ORIG")
    def show_origin(self, acidx: AcId) -> Result[str, str]:
        """Show the origin of an aircraft."""
        return Ok(f"ORIG {self.traffic.callsign[acidx]}: {self.orig[acidx]}")

    @command(name="ORIG")
    def set_origin(self, acidx: AcId, waypoint: Wpt) -> Result[str, str]:
        """Set the origin of an aircraft."""
        route = self.route[acidx]
        wpname = _waypoint_name(waypoint)
        if isinstance(position := _resolve_waypoint(self.traffic, acidx, route, waypoint), Err):
            return Err("ORIG: " + position.err())
        coordinates = position.ok()

        # Origin: bookkeeping only for now, store in route as origin
        self.orig[acidx] = wpname
        iwp = route.add_waypoint(
            acidx,
            self.orig[acidx],
            route.orig,
            coordinates.lat,
            coordinates.lon,
            0.0,
            self.traffic.cas[acidx],
        )
        if iwp < 0:
            return Err(self.orig[acidx] + " not found.")
        return Ok(f"origin set to {wpname}")

    @command(name="VNAV")
    def vnav_status(self, idx: AcIdSelection) -> Result[str, str]:
        """Show VNAV state for an aircraft or selection."""
        # BlueSky applies these commands to every aircraft in the resolved selection.
        output: list[str] = []
        for i in idx:
            msg = f"{self.traffic.callsign[i]}: VNAV is {'ON' if self.traffic.swvnav[i] else 'OFF'}"
            if not self.traffic.swvnavspd[i]:
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
                self.traffic.swvnavspd[i] = True
                route.calcfp()
                actwpidx = route.iactwp
                self.ComputeVNAV(
                    i,
                    route.wptoalt[actwpidx],
                    route.wpxtoalt[actwpidx],
                    route.wptorta[actwpidx],
                    route.wpxtorta[actwpidx],
                )
                self.traffic.actwp.nextaltco[i] = route.wptoalt[actwpidx]
            else:
                self.traffic.swvnav[i] = False
                self.traffic.swvnavspd[i] = False
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
                    direct(self.traffic, i, route.wpname[route.findact(i)])
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


def calcvrta(v0: float, dx: float, deltime: float, trafax: float) -> float:
    """Calculate the target ground speed needed to meet an RTA on a leg.

    Solves for the end speed of a constant-acceleration speed change
    followed by a constant-speed segment, such that the remaining leg
    distance is covered exactly in the remaining time. Falls back to the
    simple average speed dx/deltime when no physical solution exists.

    Args:
        v0: Current ground speed [m/s].
        dx: Remaining leg distance [m].
        deltime: Remaining time until the RTA [s].
        trafax: Available longitudinal acceleration [m/s2].

    Returns:
        float: Required target ground speed [m/s].
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

    # Set up variables
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

        # Check solutions for v1
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


def distaccel(v0: Any, v1: Any, axabs: Any) -> Any:
    """Calculate the distance travelled during an acceleration/deceleration.

    Uses the uniform-acceleration relation dx = |v1^2 - v0^2| / (2 |a|),
    which follows from x = v0*t + 1/2*a*t^2 and v = v0 + a*t. Whether it is
    an acceleration or a deceleration is determined by the sign of v1 - v0.
    Works on scalars as well as numpy arrays.

    Args:
        v0: Start speed [m/s].
        v1: End speed [m/s].
        axabs: Acceleration/deceleration of which the absolute value is
            used [m/s2].

    Returns:
        Distance travelled during the speed change [m].
    """
    return 0.5 * np.abs(v1 * v1 - v0 * v0) / np.maximum(0.001, np.abs(axabs))
