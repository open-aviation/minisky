"""Autopilot Implementation."""

from collections.abc import Collection

# debug
from inspect import stack as callstack
from math import atan, sqrt

import numpy as np

import clearsky as cs
from clearsky import stack
from clearsky.core import Entity, timed_function
from clearsky.tools import geo
from clearsky.tools.aero import (
    cas2tas,
    fpm,
    ft,
    g0,
    nm,
    tas2cas,
    vcas2tas,
    vcasormach2tas,
)
from clearsky.tools.misc import degto180
from clearsky.tools.position import txt2pos

from .route import Route


class Autopilot(Entity, replaceable=True):
    """BlueSky Autopilot implementation."""

    def __init__(self):
        super().__init__()

        # Standard self.steepness for descent
        self.steepness = 3000.0 * ft / (10.0 * nm)

        # From here, define object arrays
        with self.settrafarrays():
            # FMS directions
            self.trk = np.array([])
            self.spd = np.array([])
            self.tas = np.array([])
            self.alt = np.array([])
            self.vs = np.array([])

            # VNAV variables
            self.swtoc = np.array(
                []
            )  # ToC switch to switch on VNAV Top of Climb logic (default value True)
            self.swtod = np.array(
                []
            )  # ToD switch to switch on VNAV Top of Descent logic (default value True)

            self.dist2vs = np.array([])  # distance from coming waypoint to TOD
            self.dist2accel = np.array(
                []
            )  # Distance to go to acceleration(decelaration) for turn next waypoint [nm]

            self.swvnavvs = np.array([])  # whether to use given VS or not
            self.vnavvs = np.array([])  # vertical speed in VNAV

            # LNAV variables
            self.qdr2wp = np.array(
                []
            )  # Direction to waypoint from the last time passing was checked
            # to avoid 180 turns due to updated qdr shortly before passing wp
            self.dist2wp = np.array([])  # [m] Distance to active waypoint
            self.qdrturn = np.array([])  # qdr to next turn]
            self.dist2turn = np.array([])  # Distance to next turn [m]

            self.inturn = np.array([])  # If we're in a turn maneuver or not

            # Traffic navigation information
            self.orig = []  # Four letter code of origin airport
            self.dest = []  # Four letter code of destination airport

            # Default values
            self.bankdef = np.array([])  # nominal bank angle, [radians]
            self.vsdef = np.array([])  # [m/s]default vertical speed of autopilot

            # Currently used roll/bank angle [rad]
            self.turnphi = np.array([])  # [rad] bank angle setting of autopilot

            # Route objects
            self.route = []

        self.idxreached = []  # List indices of aircraft who have reached their active waypoint

    def create(self, n=1):
        super().create(n)

        # FMS directions
        self.trk[-n:] = cs.traf.trk[-n:]
        self.tas[-n:] = cs.traf.tas[-n:]
        self.alt[-n:] = cs.traf.alt[-n:]
        self.vs[-n:] = -999

        # Default ToC/ToD logic on
        self.swtoc[-n:] = True
        self.swtod[-n:] = True

        # VNAV Variables
        self.dist2vs[-n:] = -999.0
        self.dist2accel[
            -n:
        ] = (
            -999.0
        )  # Distance to go to acceleration(decelaration) for turn next waypoint [nm]

        # LNAV variables
        self.qdr2wp[
            -n:
        ] = -999.0  # Direction to waypoint from the last time passing was checked
        self.dist2wp[-n:] = -999.0  # Distance to go to next waypoint [nm]

        # Traffic performance data
        # (temporarily default values)
        self.vsdef[-n:] = 1500.0 * fpm  # default vertical speed of autopilot
        self.bankdef[-n:] = np.radians(25.0)

        # Route objects
        for ridx, acid in enumerate(cs.traf.id[-n:]):
            self.route[ridx - n] = Route(acid)

    # no longer timed @timed_function(name='fms', dt=cs.settings.fms_dt, manual=True)
    def wppassingcheck(self, qdr, dist):  # qdr [deg], dist [m[
        """
        The actwp is the interface between the list of waypoint data in the route object and the autopilot guidance
        when LNAV is on (heading) and optionally VNAV is on (spd & altitude)

        actwp data contains traffic arrays, to allow vectorizing the guidance logic.

        Waypoint switching (just like the adding, deletion in route) are event driven commands and
        therefore not vectorized as they occur rarely compared to the guidance.

        wppassingcheck contains the waypoint switching function:
        - Check which aircraft i have reached their active waypoint
        - Reached function return list of indices where reached logic is True
        - Get the waypoint data to the actwp (active waypoint data)
        - Shift waypoint (last,next etc.) data for aircraft i where necessary
        - Shift and maintain data (see last- and next- prefix in varubale name) e.g. to continue a special turn
        - Prepare some VNAV triggers along the new leg for the VNAV profile (where to start descent/climb)
        """

        # Get list of indices of aircraft which have reached their active waypoint
        # This vectorized function checks the passing of the waypoint using a.o. the current turn radius
        self.idxreached = cs.traf.actwp.reached(
            qdr,
            dist,
            cs.traf.actwp.flyby,
            cs.traf.actwp.flyturn,
            cs.traf.actwp.turnrad,
            cs.traf.actwp.turnhdgr,
            cs.traf.actwp.swlastwp,
        )

        # For the one who have reached their active waypoint, update vectorized leg data for guidance
        for i in self.idxreached:
            # debug commands to check VNAV state while passing waypoint
            # print("Passing waypoint",cs.traf.ap.route[i].wpname[cs.traf.ap.route[i].iactwp])
            # print("dist2wp,dist2vs",self.dist2wp[i]/nm,self.dist2vs[i]/nm) # distance to wp & distance to ToD/ToC

            # Save current wp speed for use on next leg when we pass this waypoint
            # VNAV speeds are always FROM-speeds, so we accelerate/decellerate at the waypoint
            # where this speed is specified, so we need to save it for use now
            # before getting the new data for the next waypoint

            # Get speed for next leg from the waypoint we pass now and set as active spd
            cs.traf.actwp.spd[i] = cs.traf.actwp.nextspd[i]
            cs.traf.actwp.spdcon[i] = cs.traf.actwp.nextspd[i]

            # Execute stack commands for the still active waypoint, which we pass now
            self.route[i].runactwpstack()

            # Get next wp, if there still is one
            if not cs.traf.actwp.swlastwp[i]:
                (
                    lat,
                    lon,
                    alt,
                    cs.traf.actwp.nextspd[i],
                    cs.traf.actwp.xtoalt[i],
                    toalt,
                    cs.traf.actwp.xtorta[i],
                    cs.traf.actwp.torta[i],
                    lnavon,
                    flyby,
                    flyturn,
                    turnrad,
                    turnspd,
                    turnhdgr,
                    cs.traf.actwp.next_qdr[i],
                    cs.traf.actwp.swlastwp[i],
                ) = self.route[
                    i
                ].getnextwp()  # [m] note: xtoalt,nextaltco are in meters

                (
                    cs.traf.actwp.nextturnlat[i],
                    cs.traf.actwp.nextturnlon[i],
                    cs.traf.actwp.nextturnspd[i],
                    cs.traf.actwp.nextturnrad[i],
                    cs.traf.actwp.nextturnhdgr[i],
                    cs.traf.actwp.nextturnidx[i],
                ) = self.route[i].getnextturnwp()

            else:
                # Prevent trying to activate the next waypoint when it was already the last waypoint
                # In case of end of route/no more waypoints: switch off LNAV using the lnavon
                cs.traf.swlnav[i] = False
                cs.traf.swvnav[i] = False
                cs.traf.swvnavspd[i] = False
                continue  # Go to next a/c which reached its active waypoint

            # Special turns: specified by turn radius or bank angle
            # If specified, use the given turn radius of passing wp for bank angle
            if flyturn:
                if turnspd <= 0.0:
                    turnspd = cs.traf.tas[i]

                # Heading rate overrides turnrad
                if turnhdgr > 0:
                    turnrad = cs.traf.tas[i] * 360.0 / (2 * np.pi * turnhdgr)

                # Use last turn radius for bank angle in current turn
                if cs.traf.actwp.turnrad[i] > 0.0:
                    self.turnphi[i] = atan(
                        cs.traf.actwp.turnspd[i]
                        * cs.traf.actwp.turnspd[i]
                        / (cs.traf.actwp.turnrad[i] * g0)
                    )  # [rad]
                else:
                    self.turnphi[i] = 0.0  # [rad] or leave untouched???

            else:
                self.turnphi[i] = 0.0  # [rad] or leave untouched???

            # Check LNAV switch returned by getnextwp
            # Switch off LNAV if it failed to get next wpdata
            if not lnavon and cs.traf.swlnav[i]:
                cs.traf.swlnav[i] = False
                # Last wp: copy last wp values for alt and speed in autopilot
                if cs.traf.swvnavspd[i] and cs.traf.actwp.nextspd[i] >= 0.0:
                    cs.traf.selspd[i] = cs.traf.actwp.nextspd[i]

            # In case of no LNAV, do not allow VNAV mode to be active
            cs.traf.swvnav[i] = cs.traf.swvnav[i] and cs.traf.swlnav[i]

            cs.traf.actwp.lat[i] = lat  # [deg]
            cs.traf.actwp.lon[i] = lon  # [deg]
            # 1.0 in case of fly by, else fly over
            cs.traf.actwp.flyby[i] = int(flyby)

            # Update qdr and turndist for this new waypoint for ComputeVNAV
            qdr[i], distnmi = geo.qdrdist(
                cs.traf.lat[i],
                cs.traf.lon[i],
                cs.traf.actwp.lat[i],
                cs.traf.actwp.lon[i],
            )

            # dist[i] = distnmi * nm
            self.dist2wp[i] = distnmi * nm

            cs.traf.actwp.curlegdir[i] = qdr[i]
            cs.traf.actwp.curleglen[i] = self.dist2wp[i]

            # User has entered an altitude for the new waypoint
            if alt >= -0.01:  # positive alt on this waypoint means altitude constraint
                cs.traf.actwp.nextaltco[i] = alt  # [m]
                cs.traf.actwp.xtoalt[i] = 0.0
            else:
                cs.traf.actwp.nextaltco[i] = toalt  # [m]

            # if not cs.traf.swlnav[i]:
            #    cs.traf.actwp.spd[i] = -997.

            # VNAV spd mode: use speed of this waypoint as commanded speed
            # while passing waypoint and save next speed for passing next wp
            # Speed is now from speed! Next speed is ready in wpdata
            if cs.traf.swvnavspd[i] and cs.traf.actwp.spd[i] >= 0.0:
                cs.traf.selspd[i] = cs.traf.actwp.spd[i]

            # Update turndist so ComputeVNAV works, is there a next leg direction or not?
            if cs.traf.actwp.next_qdr[i] < -900.0:
                local_next_qdr = qdr[i]
            else:
                local_next_qdr = cs.traf.actwp.next_qdr[i]

            # Calculate turn dist (and radius which we do not use now, but later) now for scalar variable [i]
            cs.traf.actwp.turndist[i], dummy = cs.traf.actwp.calcturn(
                cs.traf.tas[i],
                self.bankdef[i],
                qdr[i],
                local_next_qdr,
                turnrad,
                turnhdgr,
                flyturn,
            )  # update turn distance for VNAV

            # Get flyturn switches and data
            cs.traf.actwp.flyturn[i] = flyturn
            cs.traf.actwp.turnrad[i] = turnrad
            cs.traf.actwp.turnspd[i] = turnspd
            cs.traf.actwp.turnhdgr[i] = turnhdgr

            # Pass on whether currently flyturn mode:
            # at beginning of leg,c copy tonextwp to lastwp
            # set next turn False
            cs.traf.actwp.turnfromlastwp[i] = cs.traf.actwp.turntonextwp[i]
            cs.traf.actwp.turntonextwp[i] = False

            # Keep both turning speeds: turn to leg and turn from leg
            cs.traf.actwp.oldturnspd[i] = cs.traf.actwp.turnspd[
                i
            ]  # old turnspd, turning by this waypoint
            if cs.traf.actwp.flyturn[i]:
                cs.traf.actwp.turnspd[i] = (
                    turnspd  # new turnspd, turning by next waypoint
                )
            else:
                cs.traf.actwp.turnspd[i] = -990.0

            # Reduce turn dist for reduced turnspd
            if (
                cs.traf.actwp.flyturn[i]
                and cs.traf.actwp.turnrad[i] < 0.0
                and cs.traf.actwp.turnspd[i] >= 0.0
            ):
                turntas = cas2tas(cs.traf.actwp.turnspd[i], cs.traf.alt[i])
                cs.traf.actwp.turndist[i] = (
                    cs.traf.actwp.turndist[i]
                    * turntas
                    * turntas
                    / (cs.traf.tas[i] * cs.traf.tas[i])
                )

            # VNAV = FMS ALT/SPD mode incl. RTA
            self.ComputeVNAV(
                i,
                toalt,
                cs.traf.actwp.xtoalt[i],
                cs.traf.actwp.torta[i],
                cs.traf.actwp.xtorta[i],
            )

        # End of reached-loop: the per waypoint i switching loop

        # Update qdr2wp with up-to-date qdr, now that we have checked passing wp
        self.qdr2wp = qdr % 360.0

        # Continuous guidance when speed constraint on active leg is in update-method

        # If still an RTA in the route and currently no speed constraint
        for iac in np.where(
            (cs.traf.actwp.torta > -99.0) * (cs.traf.actwp.spdcon < 0.0)
        )[0]:
            iwp = cs.traf.ap.route[iac].iactwp
            if cs.traf.ap.route[iac].wprta[iwp] > -99.0:
                # For all a/c flying to an RTA waypoint, recalculate speed more often
                dist2go4rta = (
                    geo.kwikdist(
                        cs.traf.lat[iac],
                        cs.traf.lon[iac],
                        cs.traf.actwp.lat[iac],
                        cs.traf.actwp.lon[iac],
                    )
                    * nm
                    + cs.traf.ap.route[iac].wpxtorta[iwp]
                )  # last term zero for active wp rta

                # Set cs.traf.actwp.spd to rta speed, if necessary
                self.setspeedforRTA(iac, cs.traf.actwp.torta[iac], dist2go4rta)

                # If VNAV speed is on (by default coupled to VNAV), use it for speed guidance
                if cs.traf.swvnavspd[iac] and cs.traf.actwp.spd[iac] >= 0.0:
                    cs.traf.selspd[iac] = cs.traf.actwp.spd[iac]

    def update(self):
        # FMS LNAV mode:
        # qdr[deg],distinnm[nm]
        qdr, distinnm = geo.qdrdist(
            cs.traf.lat, cs.traf.lon, cs.traf.actwp.lat, cs.traf.actwp.lon
        )  # [deg][nm])

        self.qdr2wp = qdr
        self.dist2wp = distinnm * nm  # Conversion to meters

        # Check possible waypoint shift. Note: qdr, dist2wp will be updated accordingly in case of wp switch
        self.wppassingcheck(qdr, self.dist2wp)  # Updates self.qdr2wp when necessary

        # ================= Continuous FMS guidance ========================

        # Note that the code below is vectorized, with traffic arrays, so for all aircraft
        # ComputeVNAV and inside waypoint loop of wppassingcheck, it was scalar (per a/c with index i)

        # VNAV altitude guidance logic (using the variables prepared by ComputeVNAV when activating waypoint)

        # First question is:
        # - Can we please we start to descend or to climb?
        #
        # The variable dist2vs indicates the distance to the active waypoint where we should start our climb/descend
        # Only use this logic if there is a valid next altitude constraint (nextaltco).
        #
        # Well, when Top of Descent (ToD) switch is on, descend as late as possible,
        # But when Top of Climb switch is on or off, climb as soon as possible, only difference is steepness used in ComputeVNAV
        # to calculate cs.traf.actwp.vs

        startdescorclimb = (cs.traf.actwp.nextaltco >= -0.1) * np.logical_or(
            (cs.traf.alt > cs.traf.actwp.nextaltco)
            * np.logical_or(
                (self.dist2wp < self.dist2vs + cs.traf.actwp.turndist),
                (np.logical_not(self.swtod)),
            ),
            cs.traf.alt < cs.traf.actwp.nextaltco,
        )

        # print("self.dist2vs =",self.dist2vs)

        # If not lnav:Climb/descend if doing so before lnav/vnav was switched off
        #    (because there are no more waypoints). This is needed
        #    to continue descending when you get into a conflict
        #    while descending to the destination (the last waypoint)
        #    Use 0.1 nm (185.2 m) circle in case turndist might be zero
        self.swvnavvs = cs.traf.swvnav * np.where(
            cs.traf.swlnav,
            startdescorclimb,
            self.dist2wp <= np.maximum(0.1 * nm, cs.traf.actwp.turndist),
        )

        # Recalculate V/S based on current altitude and distance to next alt constraint
        # How much time do we have before we need to descend?
        # Now done in ComputeVNAV
        # See ComputeVNAV for cs.traf.actwp.vs calculation

        self.vnavvs = np.where(self.swvnavvs, cs.traf.actwp.vs, self.vnavvs)
        # was: self.vnavvs  = np.where(self.swvnavvs, self.steepness * cs.traf.gs, self.vnavvs)

        # self.vs = np.where(self.swvnavvs, self.vnavvs, self.vsdef * cs.traf.limvs_flag)
        # for VNAV use fixed V/S and change start of descent
        selvs = np.where(abs(cs.traf.selvs) > 0.1, cs.traf.selvs, self.vsdef)  # m/s
        self.vs = np.where(self.swvnavvs, self.vnavvs, selvs)
        self.alt = np.where(self.swvnavvs, cs.traf.actwp.nextaltco, cs.traf.selalt)

        # When descending or climbing in VNAV also update altitude command of select/hold mode
        cs.traf.selalt = np.where(
            self.swvnavvs, cs.traf.actwp.nextaltco, cs.traf.selalt
        )

        # LNAV commanded track angle
        self.trk = np.where(cs.traf.swlnav, self.qdr2wp, self.trk)

        # FMS speed guidance: anticipate accel/decel distance for next leg or turn

        # Calculate actual distance it takes to decelerate/accelerate based on two cases: turning speed (decel)

        # Normally next leg speed (actwp.spd) but in case we fly turns with a specified turn speed
        # use the turn speed

        # Is turn speed specified and are we not already slow enough? We only decelerate for turns, not accel.
        turntas = np.where(
            cs.traf.actwp.nextturnspd > 0.0,
            vcas2tas(cs.traf.actwp.nextturnspd, cs.traf.alt),
            -1.0 + 0.0 * cs.traf.tas,
        )
        # Switch is now whether the aircraft has any turn waypoints
        swturnspd = cs.traf.actwp.nextturnidx > 0
        turntasdiff = np.maximum(0.0, (cs.traf.tas - turntas) * (turntas > 0.0))

        # t = (v1-v0)/a ; x = v0*t+1/2*a*t*t => dx = (v1*v1-v0*v0)/ (2a)
        dxturnspdchg = distaccel(turntas, cs.traf.tas, cs.traf.perf.axmax)

        # Decelerate or accelerate for next required speed because of speed constraint or RTA speed
        # Note that because nextspd comes from the stack, and can be either a mach number or
        # a calibrated airspeed, it can only be converted from Mach / CAS [kts] to TAS [m/s]
        # once the altitude is known.
        nexttas = vcasormach2tas(cs.traf.actwp.nextspd, cs.traf.alt)
        #
        dxspdconchg = distaccel(cs.traf.tas, nexttas, cs.traf.perf.axmax)

        qdrturn, dist2turn = geo.qdrdist(
            cs.traf.lat,
            cs.traf.lon,
            cs.traf.actwp.nextturnlat,
            cs.traf.actwp.nextturnlon,
        )

        self.qdrturn = qdrturn
        dist2turn = dist2turn * nm

        # Where we don't have a turn waypoint, as in turn idx is negative, then put distance
        # as Earth circumference.
        self.dist2turn = np.where(cs.traf.actwp.nextturnidx > 0, dist2turn, 40075000)

        # Check also whether VNAVSPD is on, if not, SPD SEL has override for next leg
        # and same for turn logic
        usenextspdcon = (
            (self.dist2wp < dxspdconchg)
            * (cs.traf.actwp.nextspd > -990.0)
            * cs.traf.swvnavspd
            * cs.traf.swvnav
            * cs.traf.swlnav
        )

        useturnspd = (
            np.logical_or(
                cs.traf.actwp.turntonextwp,
                (self.dist2turn < (dxturnspdchg + cs.traf.actwp.turndist)),
            )
            * swturnspd
            * cs.traf.swvnavspd
            * cs.traf.swvnav
            * cs.traf.swlnav
        )

        # Hold turn mode can only be switched on here, cannot be switched off here (happeps upon passing wp)
        cs.traf.actwp.turntonextwp = cs.traf.swlnav * np.logical_or(
            cs.traf.actwp.turntonextwp, useturnspd
        )

        # Which CAS/Mach do we have to keep? VNAV, last turn or next turn?
        oncurrentleg = abs(degto180(cs.traf.trk - qdr)) < 2.0  # [deg]
        inoldturn = (cs.traf.actwp.oldturnspd > 0.0) * np.logical_not(oncurrentleg)

        # Avoid using old turning speeds when turning of this leg to the next leg
        # by disabling (old) turningspd when on leg
        cs.traf.actwp.oldturnspd = np.where(
            oncurrentleg * (cs.traf.actwp.oldturnspd > 0.0),
            -998.0,
            cs.traf.actwp.oldturnspd,
        )

        # turnfromlastwp can only be switched off here, not on (latter happens upon passing wp)
        cs.traf.actwp.turnfromlastwp = np.logical_and(
            cs.traf.actwp.turnfromlastwp, inoldturn
        )

        # Select speed: turn sped, next speed constraint, or current speed constraint
        cs.traf.selspd = np.where(
            useturnspd,
            cs.traf.actwp.nextturnspd,
            np.where(
                usenextspdcon,
                cs.traf.actwp.nextspd,
                np.where(
                    (cs.traf.actwp.spdcon >= 0) * cs.traf.swvnavspd,
                    cs.traf.actwp.spd,
                    cs.traf.selspd,
                ),
            ),
        )

        # Temporary override when still in old turn
        cs.traf.selspd = np.where(
            inoldturn
            * (cs.traf.actwp.oldturnspd > 0.0)
            * cs.traf.swvnavspd
            * cs.traf.swvnav
            * cs.traf.swlnav,
            cs.traf.actwp.oldturnspd,
            cs.traf.selspd,
        )

        self.inturn = np.logical_or(useturnspd, inoldturn)

        # Below crossover altitude: CAS=const, above crossover altitude: Mach = const
        self.tas = vcasormach2tas(cs.traf.selspd, cs.traf.alt)

    def ComputeVNAV(self, idx, toalt, xtoalt, torta, xtorta):
        """
        This function to do VNAV (and RTA) calculations is only called only once per leg for one aircraft idx.
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
        cs.traf.actwp.vs =  V/S to be used during climb/descent part, so when dist2wp<dist2vs [m] (to next waypoint)
        """

        # print ("ComputeVNAV for",cs.traf.id[idx],":",toalt/ft,"ft  ",xtoalt/nm,"nm")
        # print("Called by",callstack()[1].function)

        # Check  whether active waypoint speed needs to be adjusted for RTA
        # sets cs.traf.actwp.spd, if necessary
        # debug print("xtorta+legdist =",(xtorta+legdist)/nm)
        self.setspeedforRTA(idx, torta, xtorta + self.dist2wp[idx])  # all scalar

        # Check if there is a target altitude and VNAV is on, else return doing nothing
        if toalt < 0 or not cs.traf.swvnav[idx]:
            self.dist2vs[
                idx
            ] = (
                -999999.0
            )  # dist to next wp will never be less than this, so VNAV will do nothing
            return

        # So: somewhere there is an altitude constraint ahead
        # Compute proper values for cs.traf.actwp.nextaltco, self.dist2vs, self.alt, cs.traf.actwp.vs
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
        #
        if cs.traf.alt[idx] > toalt + epsalt:
            # Stop potential current climb (e.g. due to not making it to previous altco)
            # then stop immediately, as in: do not make it worse.
            if cs.traf.vs[idx] > 0.0001:
                self.vnavvs[idx] = 0.0
                self.alt[idx] = cs.traf.alt[idx]
                if cs.traf.swvnav[idx]:
                    cs.traf.selalt[idx] = cs.traf.alt[idx]

            # Descent modes: VNAV (= swtod/Top of Descent logic) or aiming at next alt constraint

            # Calculate max allowed altitude at next wp (above toalt)
            cs.traf.actwp.nextaltco[idx] = toalt  # [m] next alt constraint
            cs.traf.actwp.xtoalt[idx] = (
                xtoalt  # [m] distance to next alt constraint measured from next waypoint
            )

            # VNAV ToD logic
            if self.swtod[idx]:
                # Get distance to waypoint
                self.dist2wp[idx] = nm * geo.kwikdist(
                    cs.traf.lat[idx],
                    cs.traf.lon[idx],
                    cs.traf.actwp.lat[idx],
                    cs.traf.actwp.lon[idx],
                )  # was not always up to date, so update first

                # Distance to next waypoint where we need to start descent (top of descent) [m]
                descdist = (
                    abs(cs.traf.alt[idx] - toalt) / self.steepness
                )  # [m] required length for descent, uses default steepness!
                self.dist2vs[idx] = (
                    descdist - xtoalt
                )  # [m] part of that length on this leg

                # print(cs.traf.id[idx],"traf.alt =",cs.traf.alt[idx]/ft,"ft toalt = ",toalt/ft,"ft descdist =",descdist/nm,"nm")
                # print ("d2wp = ",self.dist2wp[idx]/nm,"nm d2vs = ",self.dist2vs[idx]/nm,"nm")
                # print("xtoalt =",xtoalt/nm,"nm descdist =",descdist/nm,"nm")

                # Exceptions: Descend now?
                # print("Active WP:",cs.traf.ap.route[idx].wpname[cs.traf.ap.route[idx].iactwp])
                # print("dist2wp,turndist, dist2vs= ",self.dist2wp[idx],cs.traf.actwp.turndist[idx],self.dist2vs[idx])
                if (
                    self.dist2wp[idx] - 1.02 * cs.traf.actwp.turndist[idx]
                    < self.dist2vs[idx]
                ):  # Urgent descent, we're late![m]
                    # Descend now using whole remaining distance on leg to reach altitude
                    self.alt[idx] = cs.traf.actwp.nextaltco[
                        idx
                    ]  # dial in altitude of next waypoint as calculated
                    t2go = self.dist2wp[idx] / max(0.01, cs.traf.gs[idx])
                    cs.traf.actwp.vs[idx] = (cs.traf.alt[idx] - toalt) / max(0.01, t2go)

                elif (
                    xtoalt < descdist
                ):  # Not on this leg, no descending is needed at next waypoint
                    # Top of decent needs to be on this leg, as next wp is in descent
                    cs.traf.actwp.vs[idx] = -abs(self.steepness) * (
                        cs.traf.gs[idx]
                        + (cs.traf.gs[idx] < 0.2 * cs.traf.tas[idx]) * cs.traf.tas[idx]
                    )

                else:
                    # else still level
                    cs.traf.actwp.vs[idx] = 0.0

            else:
                # We are higher but swtod = False, so there is no ToD descent logic, simply aim at next altco
                steepness_ = (cs.traf.alt[idx] - cs.traf.actwp.nextaltco[idx]) / (
                    max(0.01, self.dist2wp[idx] + xtoalt)
                )
                cs.traf.actwp.vs[idx] = -abs(steepness_) * (
                    cs.traf.gs[idx]
                    + (cs.traf.gs[idx] < 0.2 * cs.traf.tas[idx]) * cs.traf.tas[idx]
                )
                self.dist2vs[idx] = (
                    99999.0  # [m] Forces immediate descent as current distance to next wp will be less
                )

                # print("in else swtod for ", cs.traf.id[idx])

        # VNAV climb mode: climb as soon as possible (T/C logic)
        elif cs.traf.alt[idx] < toalt - 9.9 * ft:
            # Stop potential current descent (e.g. due to not making it to previous altco)
            # then stop immediately, as in: do not make it worse.
            if cs.traf.vs[idx] < -0.0001:
                self.vnavvs[idx] = 0.0
                self.alt[idx] = cs.traf.alt[idx]
                if cs.traf.swvnav[idx]:
                    cs.traf.selalt[idx] = cs.traf.alt[idx]

            # Altitude we want to climb to: next alt constraint in our route (could be further down the route)
            cs.traf.actwp.nextaltco[idx] = toalt  # [m]
            cs.traf.actwp.xtoalt[idx] = (
                xtoalt  # [m] distance to next alt constraint measured from next waypoint
            )
            self.alt[idx] = cs.traf.actwp.nextaltco[
                idx
            ]  # dial in altitude of next waypoint as calculated
            self.dist2vs[idx] = (
                99999.0  # [m] Forces immediate climb as current distance to next wp will be less
            )

            t2go = max(0.1, self.dist2wp[idx] + xtoalt) / max(0.01, cs.traf.gs[idx])
            if self.swtoc[idx]:
                steepness_ = self.steepness  # default steepness
            else:
                steepness_ = (cs.traf.alt[idx] - cs.traf.actwp.nextaltco[idx]) / (
                    max(0.01, self.dist2wp[idx] + xtoalt)
                )

            cs.traf.actwp.vs[idx] = np.maximum(
                steepness_ * cs.traf.gs[idx],
                (cs.traf.actwp.nextaltco[idx] - cs.traf.alt[idx]) / t2go,
            )  # [m/s]
        # Level leg: never start V/S
        else:
            self.dist2vs[idx] = -999.0  # [m]

        return

    def setspeedforRTA(self, idx, torta, xtorta):
        # debug print("setspeedforRTA called, torta,xtorta =",torta,xtorta/nm)

        # Calculate required CAS to meet RTA
        # for aircraft nr. idx (scalar)
        if torta < -90.0:  # -999 signals there is no RTA defined in remainder of route
            return False

        deltime = torta - cs.sim.simt  # Remaining time to next RTA [s] in simtime
        if deltime > 0:  # Still possible?
            gsrta = calcvrta(cs.traf.gs[idx], xtorta, deltime, cs.traf.perf.axmax[idx])

            # Subtract tail wind speed vector
            tailwind = (
                cs.traf.windnorth[idx] * cs.traf.gsnorth[idx]
                + cs.traf.windeast[idx] * cs.traf.gseast[idx]
            ) / cs.traf.gs[idx]

            # Convert to CAS
            rtacas = tas2cas(gsrta - tailwind, cs.traf.alt[idx])

            # Performance limits on speed will be applied in traf.update
            if cs.traf.actwp.spdcon[idx] < 0.0 and cs.traf.swvnavspd[idx]:
                cs.traf.actwp.spd[idx] = rtacas
                # print("setspeedforRTA: xtorta =",xtorta)

            return rtacas
        else:
            return False

    @stack.command(name="ALT")
    def selaltcmd(self, idx: "acid", alt: "alt", vspd: "vspd" = None):
        """ALT acid, alt, [vspd]

        Select autopilot altitude command."""
        cs.traf.selalt[idx] = alt
        cs.traf.swvnav[idx] = False

        # Check for optional VS argument
        if vspd:
            cs.traf.selvs[idx] = vspd
        else:
            if not isinstance(idx, Collection):
                idx = np.array([idx])
            delalt = alt - cs.traf.alt[idx]
            # Check for VS with opposite sign => use default vs
            # by setting autopilot vs to zero
            oppositevs = np.logical_and(
                cs.traf.selvs[idx] * delalt < 0.0, abs(cs.traf.selvs[idx]) > 0.01
            )

            cs.traf.selvs[idx[oppositevs]] = 0.0
        return True, f"altitude set to {alt} ft"

    @stack.command(name="VS")
    def selvspdcmd(self, idx: "acid", vspd: "vspd"):
        """VS acid,vspd (ft/min)

        Vertical speed command (autopilot)"""
        cs.traf.selvs[idx] = vspd  # [fpm]
        # cs.traf.vs[idx] = vspd
        cs.traf.swvnav[idx] = False
        return True, f"vertical speed set to {vspd} ft/min"

    @stack.command(name="HDG", aliases=("HEADING", "TURN"))
    def selhdgcmd(self, idx: "acid", hdg: "hdg"):  # HDG command
        """HDG acid,hdg (deg,True or Magnetic)

        Autopilot select heading command."""
        if not isinstance(idx, Collection):
            idx = np.array([idx])
        if not isinstance(hdg, Collection):
            hdg = np.array([hdg])
        # If there is wind, compute the corresponding track angle
        if cs.traf.wind.winddim > 0:
            ab50 = cs.traf.alt[idx] > 50.0 * ft
            bel50 = np.logical_not(ab50)
            iab = idx[ab50]
            ibel = idx[bel50]

            tasnorth = cs.traf.tas[iab] * np.cos(np.radians(hdg[ab50]))
            taseast = cs.traf.tas[iab] * np.sin(np.radians(hdg[ab50]))
            vnwnd, vewnd = cs.traf.wind.getdata(
                cs.traf.lat[iab], cs.traf.lon[iab], cs.traf.alt[iab]
            )
            gsnorth = tasnorth + vnwnd
            gseast = taseast + vewnd
            self.trk[iab] = np.degrees(np.arctan2(gseast, gsnorth)) % 360.0
            self.trk[ibel] = hdg
        else:
            self.trk[idx] = hdg

        cs.traf.swlnav[idx] = False
        return True, f"heading set to {hdg} deg"

    @stack.command(name="SPD", aliases=("SPEED",))
    def selspdcmd(self, idx: "acid", casmach: "spd"):  # SPD command
        """SPD acid, casmach (= CASkts/Mach)

        Select autopilot speed."""
        # Depending on or position relative to crossover altitude,
        # we will maintain CAS or Mach when altitude changes
        # We will convert values when needed
        cs.traf.selspd[idx] = casmach

        # Used to be: Switch off VNAV: SPD command overrides
        cs.traf.swvnavspd[idx] = False
        return True, f"speed set to {casmach}"

    @stack.command(name="DEST")
    def setdest(self, acidx: "acid", wpname: "wpt" = None):
        """DEST acid, latlon/airport

        Set destination of aircraft, aircraft wil fly to this airport."""
        if wpname is None:
            return True, "DEST " + cs.traf.id[acidx] + ": " + self.dest[acidx]
        route = self.route[acidx]
        apidx = cs.navdb.getaptidx(wpname)
        if apidx < 0:
            if cs.traf.ap.route[acidx].nwp > 0:
                reflat = cs.traf.ap.route[acidx].wplat[-1]
                reflon = cs.traf.ap.route[acidx].wplon[-1]
            else:
                reflat = cs.traf.lat[acidx]
                reflon = cs.traf.lon[acidx]

            success, posobj = txt2pos(wpname, reflat, reflon)
            if success:
                lat = posobj.lat
                lon = posobj.lon
            else:
                return False, "DEST: Position " + wpname + " not found."

        else:
            lat = cs.navdb.aptlat[apidx]
            lon = cs.navdb.aptlon[apidx]

        self.dest[acidx] = wpname
        iwp = route.addwpt(
            acidx, self.dest[acidx], route.dest, lat, lon, 0.0, cs.traf.cas[acidx]
        )
        # If only waypoint: activate
        if (iwp == 0) or (self.orig[acidx] != "" and route.nwp == 2):
            cs.traf.actwp.lat[acidx] = route.wplat[iwp]
            cs.traf.actwp.lon[acidx] = route.wplon[iwp]
            cs.traf.actwp.nextaltco[acidx] = route.wpalt[iwp]
            cs.traf.actwp.spd[acidx] = route.wpspd[iwp]

            cs.traf.swlnav[acidx] = True
            cs.traf.swvnav[acidx] = True
            route.iactwp = iwp
            route.direct(acidx, route.wpname[iwp])

        # If not found, say so
        elif iwp < 0:
            return False, ("DEST position" + self.dest[acidx] + " not found.")

        return True, f"destination set to {wpname}"

    @stack.command(name="ORIG")
    def setorig(self, acidx: "acid", wpname: "wpt" = None):
        """ORIG acid, latlon/airport

        Set origin of aircraft."""
        if wpname is None:
            return True, "ORIG " + cs.traf.id[acidx] + ": " + self.orig[acidx]
        route = self.route[acidx]
        apidx = cs.navdb.getaptidx(wpname)
        if apidx < 0:
            if cs.traf.ap.route[acidx].nwp > 0:
                reflat = cs.traf.ap.route[acidx].wplat[-1]
                reflon = cs.traf.ap.route[acidx].wplon[-1]
            else:
                reflat = cs.traf.lat[acidx]
                reflon = cs.traf.lon[acidx]

            success, posobj = txt2pos(wpname, reflat, reflon)
            if success:
                lat = posobj.lat
                lon = posobj.lon
            else:
                return False, ("ORIG: Position " + wpname + " not found.")

        else:
            lat = cs.navdb.aptlat[apidx]
            lon = cs.navdb.aptlon[apidx]

        # Origin: bookkeeping only for now, store in route as origin
        self.orig[acidx] = wpname
        iwp = route.addwpt(
            acidx, self.orig[acidx], route.orig, lat, lon, 0.0, cs.traf.cas[acidx]
        )
        if iwp < 0:
            return False, (self.orig[acidx] + " not found.")

        return True, f"origin set to {wpname}"

    @stack.command(name="VNAV")
    def setVNAV(self, idx: "acid", flag: "bool" = None):
        """VNAV acid,[ON/OFF]

        Switch on/off VNAV mode, the vertical FMS mode (autopilot)"""
        if not isinstance(idx, Collection):
            if idx is None:
                # All aircraft are targeted
                cs.traf.swvnav = np.array(cs.traf.ntraf * [flag])
                cs.traf.swvnavspd = np.array(cs.traf.ntraf * [flag])
            else:
                # Prepare for the loop
                idx = np.array([idx])

        # Set VNAV for all aircraft in idx array
        output = []
        for i in idx:
            if flag is None:
                msg = (
                    cs.traf.id[i] + ": VNAV is " + "ON" if cs.traf.swvnav[i] else "OFF"
                )
                if not cs.traf.swvnavspd[i]:
                    msg += " but VNAVSPD is OFF"
                output.append(
                    cs.traf.id[i] + ": VNAV is " + "ON" if cs.traf.swvnav[i] else "OFF"
                )

            elif flag:
                if not cs.traf.swlnav[i]:
                    return False, (cs.traf.id[i] + ": VNAV ON requires LNAV to be ON")

                route = self.route[i]
                if route.nwp > 0:
                    cs.traf.swvnav[i] = True
                    cs.traf.swvnavspd[i] = True
                    self.route[i].calcfp()
                    actwpidx = self.route[i].iactwp
                    self.ComputeVNAV(
                        i,
                        self.route[i].wptoalt[actwpidx],
                        self.route[i].wpxtoalt[actwpidx],
                        self.route[i].wptorta[actwpidx],
                        self.route[i].wpxtorta[actwpidx],
                    )
                    cs.traf.actwp.nextaltco[i] = self.route[i].wptoalt[actwpidx]

                else:
                    return False, (
                        "VNAV "
                        + cs.traf.id[i]
                        + ": no waypoints or destination specified"
                    )
            else:
                cs.traf.swvnav[i] = False
                cs.traf.swvnavspd[i] = False
        if flag == None:
            return True, "\n".join(output)

        return True, f"VNAV {'ON' if flag else 'OFF'}"

    @stack.command(name="LNAV")
    def setLNAV(self, idx: "acid", flag: "bool" = None):
        """LNAV acid,[ON/OFF]

        LNAV (lateral FMS mode) switch for autopilot"""
        if not isinstance(idx, Collection):
            if idx is None:
                # All aircraft are targeted
                cs.traf.swlnav = np.array(cs.traf.ntraf * [flag])
            else:
                # Prepare for the loop
                idx = np.array([idx])

        # Set LNAV for all aircraft in idx array
        output = []
        for i in idx:
            if flag is None:
                output.append(
                    cs.traf.id[i]
                    + ": LNAV is "
                    + ("ON" if cs.traf.swlnav[i] else "OFF")
                )

            elif flag:
                route = self.route[i]
                if route.nwp <= 0:
                    return False, (
                        "LNAV "
                        + cs.traf.id[i]
                        + ": no waypoints or destination specified"
                    )
                elif not cs.traf.swlnav[i]:
                    cs.traf.swlnav[i] = True
                    route.direct(i, route.wpname[route.findact(i)])
            else:
                cs.traf.swlnav[i] = False
        if flag is None:
            return True, "\n".join(output)

        return True, f"LNAV {'ON' if flag else 'OFF'}"

    @stack.command(name="SWTOC")
    def setswtoc(self, idx: "acid", flag: "bool" = None):
        """SWTOC acid,[ON/OFF]

        Switch ToC logic (=climb early) on/off"""

        if not isinstance(idx, Collection):
            if idx is None:
                # All aircraft are targeted
                self.swtoc = np.array(cs.traf.ntraf * [flag])
            else:
                # Prepare for the loop
                idx = np.array([idx])

        # Set SWTOC for all aircraft in idx array
        output = []
        for i in idx:
            if flag is None:
                output.append(
                    cs.traf.id[i] + ": SWTOC is " + ("ON" if self.swtoc[i] else "OFF")
                )

            elif flag:
                self.swtoc[i] = True
            else:
                self.swtoc[i] = False
        if flag is None:
            return True, "\n".join(output)

        return True, f"SWTOC {'ON' if flag else 'OFF'}"

    @stack.command(name="SWTOD")
    def setswtod(self, idx: "acid", flag: "bool" = None):
        """SWTOD acid,[ON/OFF]

        Switch ToD logic (=climb early) on/off"""
        if not isinstance(idx, Collection):
            if idx is None:
                # All aircraft are targeted
                self.swtod = np.array(cs.traf.ntraf * [flag])
            else:
                # Prepare for the loop
                idx = np.array([idx])

        # Set SWTOD for all aircraft in idx array
        output = []
        for i in idx:
            if flag is None:
                output.append(
                    cs.traf.id[i] + ": SWTOD is " + ("ON" if self.swtoc[i] else "OFF")
                )

            elif flag:
                self.swtod[i] = True
            else:
                self.swtod[i] = False
        if flag is None:
            return True, "\n".join(output)

        return True, f"SWTOD {'ON' if flag else 'OFF'}"


def calcvrta(v0, dx, deltime, trafax):
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
    if v0 * dt < dx:
        ax = max(0.01, abs(trafax))
    else:
        ax = -max(0.01, abs(trafax))

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


def distaccel(v0, v1, axabs):
    """Calculate distance travelled during acceleration/deceleration
    v0 = start speed, v1 = endspeed, axabs = magnitude of accel/decel
    accel/decel is detemremind by sign of v1-v0
    axabs is acceleration/deceleration of which absolute value will be used
    solve for x: x = vo*t + 1/2*a*t*t    v = v0 + a*t"""
    return 0.5 * np.abs(v1 * v1 - v0 * v0) / np.maximum(0.001, np.abs(axabs))
