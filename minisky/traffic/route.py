"""Route implementation for the BlueSky FMS.

Contains the per-aircraft :class:`Route` class (the flight plan: an ordered
list of waypoints with optional altitude, speed, RTA and turn constraints)
plus the module-level functions that implement the route-editing stack
commands: ADDWPT, ADDWPTMODE, AFTER, BEFORE, AT, DIRECT, RTA, LISTRTE,
DELRTE and DELWPT.

The route itself is passive data with flight-plan pre-calculations
(calcfp()); the actual guidance along the route is performed by
:class:`~minisky.traffic.autopilot.Autopilot`, which pulls waypoint data
into the vectorized :class:`~minisky.traffic.activewpdata.ActiveWaypoint`
arrays via getnextwp()/getnextturnwp().
"""

import math

import numpy as np

import minisky
from minisky import stack

# from minisky.core import Replaceable
from minisky.stack import Command
from minisky.stack.argparser import Alt, Spd, Time, Wpt
from minisky.tools import geo
from minisky.tools.aero import casormach2tas, ft, g0, kts, mach2cas, nm
from minisky.tools.convert import degto180, txt2alt, txt2spd
from minisky.tools.position import Position, txt2pos


class Route:
    """Flight plan (route) of a single aircraft: basic FMS functionality.

    A Route is an ordered list of waypoints, each with an optional altitude
    constraint, speed constraint, required time of arrival (RTA), turn
    specification (fly-by/fly-over/fly-turn with radius, speed or heading
    rate) and stack commands to execute when the waypoint is passed. One
    Route object is kept per aircraft in ``minisky.traf.ap.route``.

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
        wpalt (list): Altitude constraints [m] (negative = not specified).
        wpspd (list): Speed constraints, CAS [m/s] or Mach [-]
            (negative = not specified).
        wprta (list): Required times of arrival [s] (negative = none).
        wpflyby (list): Fly-by (True) / fly-over (False) switch.
        wpflyturn (list): Fly-turn switch (use specified turn parameters).
        wpturnrad (list): Turn radius per waypoint (<0 = not specified).
        wpturnspd (list): Turn speed (CAS) per waypoint (<0 = not specified).
        wpturnhdgr (list): Turn heading rate per waypoint [deg/s]
            (<0 = not specified).
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
        wpialt (list): Index of the next waypoint with an altitude
            constraint.
        wptoalt (list): Next altitude constraint [m].
        wpxtoalt (list): Distance to the next altitude constraint [m].
        wptorta (list): Next time constraint [s].
        wpxtorta (list): Distance to the next time constraint [m].

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

    def __init__(self, acid: str) -> None:
        self.acid = acid

        # Waypoint data
        self.wpname = []  # List of waypoint names for this flight plan
        self.wptype = []  # List of waypoint types
        self.wplat = []  # List of waypoint latitudes
        self.wplon = []  # List of waypoint longitudes
        self.wpalt = []  # [m] negative value means not specified
        self.wpspd = []  # [m/s] negative value means not specified
        self.wprta = []  # [m/s] negative value means not specified
        self.wpflyby = []  # Flyby (True)/flyover(False) switch
        self.wpstack = []  # Stack with command execured when passing this waypoint

        # Made for drones: fly turn mode, means use specified turn radius and optionally turn speed
        self.wpflyturn = []  # Flyturn (True) or flyover/flyby (False) switch
        self.wpturnrad = []  # [nm] Turn radius per waypoint (<0 = not specified)
        self.wpturnspd = []  # [kts] Turn speed (IAS/CAS) per waypoint (<0 = not specified)
        self.wpturnhdgr = []  # [deg/s] Heading rate, uses actual speed to calculate bank & radius (<0 = not specified)

        # Current actual waypoint
        self.iactwp = -1

        # Set to default addwpt wpmode
        # Note that neither flyby nor flyturn means: flyover)
        self.swflyby = True  # Default waypoints are flyby waypoint
        self.swflyturn = False  # Default waypoints are waypoints w/o specified turn

        # Default turn values to be used in flyturn mode
        self.bank = 25.0  # [deg] Default bank angle
        self.turnrad = -999.0  # [m] Negative value indicating no value has been set
        self.turnspd = (
            -999.0
        )  # [kts] Dito, in this case bank angle of vehicle will be used with current speed
        self.turnhdgr = (
            -999.0
        )  # [deg/s] Dito, in this case bank angle of vehicle will be used with current speed

        # if the aircraft lands on a runway, the aircraft should keep the
        # runway heading
        # default: False
        self.flag_landed_runway = False

        self.wpdirfrom = []  # [deg] direction leg to wp
        self.wpdirto = []  # [deg] direction leg from wp
        self.wpdistto = []  # [nm] leg length to wp
        self.wpialt = []
        self.wptoalt = []  # [m] next alt contraint
        self.wpxtoalt = []  # [m] distance ot next alt constraint
        self.wptorta = []  # [s] next time constraint
        self.wpxtorta = []  # [m] distance to next time constaint

    def insert_wpt_data(
        self,
        wpidx: int,
        wpname: str,
        wplat: float,
        wplon: float,
        wptype: int,
        wpalt: float,
        wpspd: float,
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
            wpalt: Altitude constraint [m] (negative = not specified).
            wpspd: Speed constraint, CAS [m/s] or Mach [-]
                (negative = not specified).
        """

        self.wpname.insert(wpidx, wpname)
        self.wplat.insert(wpidx, wplat)
        self.wplon.insert(wpidx, wplon)
        self.wpalt.insert(wpidx, wpalt)
        self.wpspd.insert(wpidx, wpspd)
        self.wptype.insert(wpidx, wptype)
        self.wpflyby.insert(wpidx, self.swflyby)
        self.wpflyturn.insert(wpidx, self.swflyturn)
        self.wpturnrad.insert(wpidx, self.turnrad)
        self.wpturnspd.insert(wpidx, self.turnspd)
        self.wpturnhdgr.insert(wpidx, self.turnhdgr)
        self.wprta.insert(wpidx, -999.0)  # initially no RTA
        self.wpstack.insert(wpidx, [])

    def add_waypoint(
        self,
        iac: int,
        name: str,
        wptype: int,
        lat: float,
        lon: float,
        alt: float = -999.0,
        spd: float = -999.0,
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
            alt: Altitude constraint [m] (negative = not specified).
            spd: Speed constraint, CAS [m/s] or Mach [-]
                (negative = not specified).
            afterwp: Optional name of the waypoint after which to insert.
            beforewp: Optional name of the waypoint before which to insert.

        Returns:
            int: Index of the added waypoint in the route, or -1 on failure.
        """

        # For safety
        n_wpt = len(self.wplat)

        name = name.upper().strip()

        wplat = (lat + 90.0) % 180.0 - 90.0
        wplon = (lon + 180.0) % 360.0 - 180.0

        wpok = True  # switch for waypoint check

        # Check if name already exists, if so add integer 01, 02, 03 etc.
        wprtename = get_available_name(self.wpname, name)
        # Select on wptype
        # ORIGIN: Wptype is origin/destination?
        if wptype == Route.orig or wptype == Route.dest:
            orig = wptype == Route.orig
            wpidx = 0 if orig else -1
            suffix = "ORIG" if orig else "DEST"

            if name != minisky.traf.callsign[iac] + suffix:  # published identifier
                i = minisky.navdb.getaptidx(name)
                if i >= 0:
                    wplat = minisky.navdb.aptlat[i]
                    wplon = minisky.navdb.aptlon[i]

            if not orig and alt < 0:
                alt = 0

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
                self.wpflyturn[wpidx] = self.swflyturn
                self.wpturnrad[wpidx] = self.turnrad
                self.wpturnspd[wpidx] = self.turnspd
                self.wpturnhdgr[wpidx] = self.turnhdgr
                self.wprta[wpidx] = -999.0  # initially no RTA
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
                newname = get_available_name(self.wpname, name, 3)

            # Else make data complete with nav database and closest to given lat,lon
            else:  # so wptypewpnav
                newname = wprtename

                if wptype != Route.runway:
                    i = minisky.navdb.getwpidx(name, lat, lon)
                    wpok = i >= 0

                    if wpok:
                        wplat = minisky.navdb.wplat[i]
                        wplon = minisky.navdb.wplon[i]
                    else:
                        i = minisky.navdb.getaptidx(name)
                        wpok = i >= 0
                        if wpok:
                            wplat = minisky.navdb.aptlat[i]
                            wplon = minisky.navdb.aptlon[i]

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
            minisky.traf.actwp.next_qdr[iac] = self.getnextqdr()
            minisky.traf.actwp.swlastwp[iac] = self.iactwp == n_wpt - 1

        # Update waypoints
        if wptype != Route.calcwp:
            self.calcfp()

        # Update autopilot settings
        if wpok and 0 <= self.iactwp < n_wpt:
            direct(iac, self.wpname[self.iactwp])

        return idx

    def getnextturnwp(self) -> list:
        """Give the data of the next fly-turn waypoint at or after the
        active waypoint.

        Returns:
            list: [lat [deg], lon [deg], turn speed (CAS, <0 = not
            specified), turn radius (<0 = not specified), turn heading rate
            [deg/s] (<0 = not specified), waypoint index]. Default values
            (zeros / -999) are returned when the route has no upcoming turn
            waypoint.
        """
        # Scan forward from the active waypoint; called for every switching
        # aircraft, so avoid converting the whole route to a numpy array
        trnidx = next(
            (j for j in range(max(self.iactwp, 0), len(self.wpflyturn)) if self.wpflyturn[j]),
            None,
        )
        if trnidx is None:
            # No turn waypoints, return default values
            return [0.0, 0.0, -999.0, -999.0, -999, -999.0]

        # Return the next turn waypoint info
        return [
            self.wplat[trnidx],
            self.wplon[trnidx],
            self.wpturnspd[trnidx],
            self.wpturnrad[trnidx],
            self.wpturnhdgr[trnidx],
            trnidx,
        ]

    def getnextwp(self) -> tuple:
        """Activate the next waypoint in the route and return its data.

        Called by the autopilot when the active waypoint has been passed.
        Advances iactwp (unless the last waypoint was reached, in which case
        the returned LNAV switch is False). When the new active waypoint is
        a runway used for landing, a fixed runway heading is commanded and
        deceleration plus deletion of the aircraft are scheduled via the
        stack.

        Returns:
            tuple: (lat [deg], lon [deg], altitude constraint [m], speed
            constraint (CAS [m/s] or Mach), distance to next altitude
            constraint [m], next altitude constraint [m], distance to next
            RTA [m], next RTA [s], lnavon switch, fly-by switch, fly-turn
            switch, turn radius, turn speed (CAS), turn heading rate
            [deg/s], next-leg endpoint lat [deg], next-leg endpoint lon
            [deg] (-999.0 pair when there is no next leg), last-waypoint
            switch).
        """

        n_wpt = len(self.wpname)

        if self.flag_landed_runway:
            # when landing, LNAV is switched off
            lnavon = False

            # no further waypoint
            nextleglat = -999.0
            nextleglon = -999.0

            # and the aircraft just needs a fixed heading to
            # remain on the runway
            # syntax: HDG acid,hdg (deg,True)
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
            wphdg = minisky.navdb.rwythresholds[name[:4]][rwykey][2]

            # keep constant runway heading
            stack.stack("HDG " + str(self.acid) + " " + str(wphdg))

            # start decelerating
            stack.stack("DELAY " + "10 " + "SPD " + str(self.acid) + " " + "10")

            # delete aircraft
            stack.stack("DELAY " + "42 " + "DEL " + str(self.acid))

            swlastwp = self.iactwp == n_wpt - 1

            return (
                self.wplat[self.iactwp],
                self.wplon[self.iactwp],
                self.wpalt[self.iactwp],
                self.wpspd[self.iactwp],
                self.wpxtoalt[self.iactwp],
                self.wptoalt[self.iactwp],
                self.wpxtorta[self.iactwp],
                self.wptorta[self.iactwp],
                lnavon,
                self.wpflyby[self.iactwp],
                self.wpflyturn[self.iactwp],
                self.wpturnrad[self.iactwp],
                self.wpturnspd[self.iactwp],
                self.wpturnhdgr[self.iactwp],
                nextleglat,
                nextleglon,
                swlastwp,
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
        if -1 < self.iactwp < n_wpt - 1:
            nextleglat = self.wplat[self.iactwp + 1]
            nextleglon = self.wplon[self.iactwp + 1]
        else:
            nextleglat = -999.0
            nextleglon = -999.0

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

        # print ("getnextwp:",self.wpname[self.iactwp],"   torta = ",self.wptorta[self.iactwp])

        return (
            self.wplat[self.iactwp],
            self.wplon[self.iactwp],
            self.wpalt[self.iactwp],
            self.wpspd[self.iactwp],
            self.wpxtoalt[self.iactwp],
            self.wptoalt[self.iactwp],
            self.wpxtorta[self.iactwp],
            self.wptorta[self.iactwp],
            lnavon,
            self.wpflyby[self.iactwp],
            self.wpflyturn[self.iactwp],
            self.wpturnrad[self.iactwp],
            self.wpturnspd[self.iactwp],
            self.wpturnhdgr[self.iactwp],
            nextleglat,
            nextleglon,
            swlastwp,
        )

    def runactwpstack(self) -> None:
        """Execute the stack commands stored for the active waypoint.

        Commands are attached to waypoints with the AT ... DO/STACK command
        and are issued when the aircraft passes the waypoint.
        """
        for cmdline in self.wpstack[self.iactwp]:
            stack.stack(cmdline)
            # debug
            # stack.stack("ECHO "+self.acid+" AT "+self.wpname[self.iactwp]+" command issued:"+cmdline)
        return

    def insertcalcwp(self, i: int, name: str) -> None:
        """Insert an empty calculated waypoint (T/C, T/D) at location i."""

        self.wpname.insert(i, name)
        self.wplat.insert(i, 0.0)
        self.wplon.insert(i, 0.0)
        self.wpalt.insert(i, -999.0)
        self.wpspd.insert(i, -999.0)
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
        (wpdirfrom, wpdirto, wpdistto), the next altitude constraint and
        distance to it (wptoalt [m], wpxtoalt [m]), and the next time
        constraint and distance to it (wptorta [s], wpxtorta [m]).
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

        # wp index of next alttud constraint
        self.wpialt = n_wpt * [-1]

        # [m] next alt contraint
        self.wptoalt = n_wpt * [-999.0]

        # [m] dist to next alt constraint, default 1.0 to avoid division by zero
        self.wpxtoalt = n_wpt * [1.0]

        # wp index of next time constraint
        self.wpirta = n_wpt * [-1]

        # [s] next time constraint
        self.wptorta = n_wpt * [-999.0]

        # [m] dist to next time constraint, default 1.0 to avoid division by zero
        self.wpxtorta = n_wpt * [1.0]

        # No waypoints: make empty variables to be safe and return: nothing to do
        if n_wpt == 0:
            return

        # Calculate lateral leg data
        # LNAV: Calculate leg distances and directions

        for i in range(0, n_wpt - 1):
            qdr, dist = geo.qdrdist(
                self.wplat[i], self.wplon[i], self.wplat[i + 1], self.wplon[i + 1]
            )
            self.wpdirfrom[i] = float(qdr)  # [deg]
            self.wpdistto[i + 1] = float(dist)  # [nm]  distto is in nautical miles

        # Also add "from direction" as to directions so no need to shift for actwpdata
        # direction to will be overwritten in actwpdata in case of a direct to
        # Add current pos to first waypoint as default value for direction to 1st waypoint
        iac = minisky.traf.idx(self.acid)
        qdr, dist = geo.qdrdist(
            minisky.traf.lat[iac], minisky.traf.lon[iac], self.wplat[0], self.wplon[0]
        )
        self.wpdirto = [qdr] + self.wpdirfrom[0:-1]  # [deg] Direction to waypoints

        # Continue flying in the saem direction
        if n_wpt > 1:
            self.wpdirfrom[-1] = self.wpdirfrom[-2]

        # Calculate longitudinal leg data
        # VNAV: calc next altitude constraint: index, altitude and distance to it
        ialt = -1  # index to waypoint with next altitude constraint
        toalt = -999.0  # value of next altitude constraint
        xtoalt = 0.0  # distance to next altitude constraint from this wp
        for i in range(n_wpt - 1, -1, -1):
            # waypoint with altitude constraint (dest of al specified)
            if self.wptype[i] == Route.dest:
                ialt = i
                toalt = 0.0
                xtoalt = 0.0  # [m]

            elif self.wpalt[i] >= 0:
                ialt = i
                toalt = self.wpalt[i]
                xtoalt = 0.0  # [m]

            # waypoint with no altitude constraint:keep counting
            else:
                # [m] xtoalt is in meters!
                xtoalt = xtoalt + self.wpdistto[i + 1] * nm if i != n_wpt - 1 else 0.0

            self.wpialt[i] = ialt
            self.wptoalt[i] = toalt  # [m]
            self.wpxtoalt[i] = xtoalt  # [m]

        # RTA: calc next rta constraint: index, altitude and distance to it
        # If any RTA.
        if any(np.array(self.wprta) >= 0.0):
            # print("Yes, I found RTAs")
            irta = -1  # index of wp
            torta = -999.0  # next rta value
            xtorta = 0.0  # distance to next rta
            for i in range(n_wpt - 1, -1, -1):
                # waypoint with rta: reset counter, update rts
                if self.wprta[i] >= 0:
                    irta = i
                    torta = self.wprta[i]
                    xtorta = 0.0  # [m]

                # waypoint with no altitude constraint:keep counting
                else:
                    if i != n_wpt - 1:
                        # No speed or rta constraint: add to xtorta
                        if self.wpspd[i] <= 0.0:
                            xtorta = xtorta + self.wpdistto[i + 1] * nm  # [m] xtoalt is in meters!
                        else:
                            # speed constraint on this leg: shift torta to account for this
                            # altitude unknown
                            # TODO: current a/c altitude would be better guess, but not accessible here
                            # as we do not know aircraft index for this route.
                            # Default to 10000 ft to minimize errors, when no alt constraints
                            # are present
                            alt = toalt if self.wptoalt[i] > 0.0 else 10000.0 * ft
                            legtas = casormach2tas(self.wpspd[i], alt)
                            # TODO: account for wind at this position vy adding wind vectors to waypoints?

                            # xtorta stays the same! This leg will not be available for RTA scheduling, so distance
                            # is not in xtorta. Therefore we need to subtract legtime to ignore this leg for the RTA
                            # scheduling
                            legtime = self.wpdistto[i + 1] / legtas
                            torta = torta - legtime
                    else:
                        xtorta = 0.0
                        torta = -999.0

                self.wpirta[i] = irta
                self.wptorta[i] = torta  # [s]
                self.wpxtorta[i] = xtorta  # [m]
            # print("wpxtorta=",self.wpxtorta)
            # print("wptorta=", self.wptorta)

    def findact(self, i: int) -> int:
        """Find the best default active waypoint for an aircraft.

        Called when LNAV is (re-)engaged. Selects the waypoint closest to
        the aircraft, without walking back to earlier waypoints, and skips
        to the next waypoint when the closest one cannot be reached with
        the required heading change (turn time exceeds straight flight
        time).

        Args:
            i: Aircraft index.

        Returns:
            int: Index of the suggested active waypoint in this route,
            or -1 for an empty route.
        """

        n_wpt = len(self.wpname)

        # Check for easy answers first
        if n_wpt <= 0:
            return -1

        elif n_wpt == 1:
            return 0

        # Find closest
        wplat = np.array(self.wplat)
        wplon = np.array(self.wplon)
        dy = wplat - minisky.traf.lat[i]
        dx = (wplon - minisky.traf.lon[i]) * minisky.traf.coslat[i]
        dist2 = dx * dx + dy * dy
        # Note: the max() prevents walking back, even in cases when this might be apropriate,
        # such as when previous waypoints have been deleted

        iwpnear = max(self.iactwp, np.argmin(dist2))

        # Unless behind us, next waypoint?
        if iwpnear + 1 < n_wpt:
            qdr = math.degrees(math.atan2(dx[iwpnear], dy[iwpnear]))
            delhdg = abs(degto180(minisky.traf.trk[i] - qdr))

            # we only turn to the first waypoint if we can reach the required
            # heading before reaching the waypoint
            time_turn = (
                max(0.01, minisky.traf.tas[i])
                * math.radians(delhdg)
                / (g0 * math.tan(minisky.traf.ap.bankdef[i]))
            )
            time_straight = math.sqrt(dist2[iwpnear]) * 60.0 * nm / max(0.01, minisky.traf.tas[i])

            if time_turn > time_straight:
                iwpnear += 1

        return int(iwpnear)

    def getnextqdr(self):
        """Return the bearing of the leg after the active waypoint [deg].

        Returns -999.0 when there is no next leg (no active waypoint or the
        active waypoint is the last one).
        """
        # get qdr for next leg
        if -1 < self.iactwp < len(self.wpname) - 1:
            nextqdr, dist = geo.qdrdist(
                self.wplat[self.iactwp],
                self.wplon[self.iactwp],
                self.wplat[self.iactwp + 1],
                self.wplon[self.iactwp + 1],
            )
        else:
            nextqdr = -999.0
        return nextqdr


# ---- following are functions managing the routes ----


def get_available_name(data: list, name_: str, len_: int = 2) -> str:
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
    if minisky.traf.callsign.count(name_) > 0:
        appi = 1
        name_ = name_ + fmt_.format(appi)

    while data.count(name_) > 0:
        appi += 1
        name_ = name_[:-len_] + fmt_.format(appi)
    return name_


def change_wpt_mode(acidx: int, mode=None, value=None) -> bool | None:
    """Change the mode with which ADDWPT adds new waypoints.

    Implements the ADDWPTMODE stack command. Available modes: FLYBY,
    FLYOVER, FLYTURN. Also used to specify the TURNSPEED, TURNRADIUS or
    TURNHDGRATE used for fly-turn waypoints. Without arguments, the current
    ADDWPT mode is echoed.

    Args:
        acidx: Aircraft index.
        mode: Mode keyword (FLYBY/FLYOVER/FLYTURN) or turn-parameter keyword
            (TURNSPEED/TURNRADIUS/TURNHDGRATE and synonyms); None to show
            the current mode.
        value: Value for the selected turn parameter (parsed via the alt
            argument parser; see addwpt() for the unit conversions).

    Returns:
        bool: True on success.
    """
    # Get aircraft route
    minisky.traf.callsign[acidx]
    acrte = minisky.traf.ap.route[acidx]
    # First, we want to check what 'mode' is, and then call addwpt_stack
    # accordingly.
    if mode in ["FLYBY", "FLYOVER", "FLYTURN"]:
        # We're just changing addwpt mode, call the appropriate function.
        addwpt(acidx, mode)
        return True

    elif mode in [
        "TURNSPEED",
        "TURNSPD",
        "TURNRADIUS",
        "TURNRAD",
        "TURNHDGRATE",
        "TURNHDG",
        "TURNHDGR",
    ]:
        # We're changing the turn speed or radius
        addwpt(acidx, mode, value)
        return True

    elif mode == None:
        # Just echo the current wptmode
        if acrte.swflyby == True and acrte.swflyturn == False:
            minisky.scr.echo("Current ADDWPT mode is FLYBY.")
            return True

        elif acrte.swflyby == False and acrte.swflyturn == False:
            minisky.scr.echo("Current ADDWPT mode is FLYOVER.")
            return True

        else:
            minisky.scr.echo("Current ADDWPT mode is FLYTURN.")
            return True


def addwpt(ac: str | int, *args) -> bool | tuple:  # args: all arguments of addwpt
    """Add a waypoint to the route of an aircraft.

    Implements the ADDWPT stack command:
    ``ADDWPT acid, (wpname/lat,lon), [alt], [spd], [afterwp], [beforewp]``.

    Besides adding a regular waypoint (navdb waypoint, airport, runway or
    lat/lon position, with optional altitude constraint [m] and speed
    constraint (CAS [m/s] or Mach)), this function also handles:

    - Mode keywords FLYBY/FLYOVER/FLYTURN, which change the default mode
      for waypoints added afterwards.
    - Turn-parameter keywords TURNRAD(IUS)/TURNSPD/TURNSPEED/TURNHDG(RATE),
      which set the default turn radius/speed/heading rate and switch on
      fly-turn mode ("OFF" removes the setting).
    - The special TAKEOFF waypoint, placed 2 nm beyond the runway threshold
      in the runway direction.

    The first real waypoint added to a route is made active, engaging LNAV.

    Args:
        ac: Aircraft callsign or index.
        *args: Remaining ADDWPT arguments as described above.

    Returns:
        bool or tuple: True on success, or (success flag, message).
    """

    # First get the appropriate ac route
    if isinstance(ac, str):
        acidx = minisky.traf.idx(ac)
        callsign = ac
    else:
        acidx = ac
        callsign = minisky.traf.callsign[acidx]

    acrte = minisky.traf.ap.route[acidx]

    # Check FLYBY or FLYOVER switch, instead of adding a waypoint

    if len(args) == 1:
        swwpmode = args[0].replace("-", "")

        if swwpmode == "FLYBY":
            acrte.swflyby = True
            acrte.swflyturn = False
            return True

        elif swwpmode == "FLYOVER":
            acrte.swflyby = False
            acrte.swflyturn = False
            return True

        elif swwpmode == "FLYTURN":
            acrte.swflyby = False
            acrte.swflyturn = True
            return True

    elif len(args) == 2:
        swwpmode = args[0].replace("-", "")

        if swwpmode == "TURNRAD" or swwpmode == "TURNRADIUS":
            try:
                if args[1] == "OFF":
                    acrte.turnrad = -999
                else:
                    acrte.turnrad = float(args[1] / ft * nm)  # arg was originally parsed as wpalt
            except Exception:
                return False, "Error in processing value of turn radius"

            # Switch flyturn automatically when this is set
            acrte.swflyby = False
            acrte.swflyturn = True

            return True

        elif swwpmode == "TURNSPD" or swwpmode == "TURNSPEED":
            try:
                if args[1] == "OFF":
                    acrte.turnspd = -999
                else:
                    acrte.turnspd = (
                        args[1] * kts / ft
                    )  # [m/s] Arg was wpalt Keep it as IAS/CAS orig in kts, now in m/s
            except Exception:
                return False, "Error in processing value of turn speed"

            # Switch flyturn automatically when this is set
            acrte.swflyby = False
            acrte.swflyturn = True

        elif swwpmode == "TURNHDGRATE" or swwpmode == "TURNHDG" or swwpmode == "TURNHDGR":
            try:
                if args[1] == "OFF":
                    acrte.turnhdgr = -999
                else:
                    acrte.turnhdgr = args[1] / ft  # [deg/s] turn rate
            except Exception:
                return False, "Error in processing value of turn heading rate"

            # Switch flyturn automatically when this is set
            acrte.swflyby = False
            acrte.swflyturn = True

            return True

    # Convert to positions
    name = args[0].upper().strip()

    n_wpt = len(acrte.wpname)

    # Choose reference position ot look up VOR and waypoints
    # First waypoint: own position
    if n_wpt == 0:
        reflat = minisky.traf.lat[acidx]
        reflon = minisky.traf.lon[acidx]

    # Or last waypoint before destination
    else:
        if acrte.wptype[-1] != Route.dest or n_wpt == 1:
            reflat = acrte.wplat[-1]
            reflon = acrte.wplon[-1]
        else:
            reflat = acrte.wplat[-2]
            reflon = acrte.wplon[-2]

    # Default altitude, speed and afterwp
    alt = -999.0
    spd = -999.0
    afterwp = ""
    beforewp = ""

    # Is it aspecial take-off waypoint?
    takeoffwpt = name.replace("-", "") == "TAKEOFF"

    # Normal waypoint (no take-off waypoint => see else)
    if not takeoffwpt:
        # Get waypoint position
        success, posobj = txt2pos(name, reflat, reflon)
        if success:
            assert isinstance(posobj, Position)
            lat = posobj.lat
            lon = posobj.lon

            if posobj.type == "nav" or posobj.type == "apt":
                wptype = Route.wpnav

            elif posobj.type == "rwy":
                wptype = Route.runway

            else:  # treat as lat/lon
                name = callsign
                wptype = Route.wplatlon

            if len(args) > 1 and args[1]:
                alt = args[1]

            if len(args) > 2 and args[2]:
                spd = args[2]

            if len(args) > 3 and args[3]:
                afterwp = args[3]

            if len(args) > 4 and args[4]:
                beforewp = args[4]

        else:
            return False, "Waypoint " + name + " not found."

    # Take off waypoint: positioned 20% of the runway length after the runway
    else:
        # Look up runway in route
        rwyrteidx = -1
        i = 0
        while i < n_wpt and rwyrteidx < 0:
            if acrte.wpname[i].count("/") > 0:
                rwyrteidx = i
            i += 1

        # Only TAKEOFF is specified wihtou a waypoint/runway
        if len(args) == 1 or not args[1]:
            # No runway given: use first in route or current position

            # print ("rwyrteidx =",rwyrteidx)
            # We find a runway in the route, so use it
            if rwyrteidx > 0:
                rwylat = acrte.wplat[rwyrteidx]
                rwylon = acrte.wplon[rwyrteidx]
                aptidx = minisky.navdb.getapinear(rwylat, rwylon)
                aptname = minisky.navdb.aptname[aptidx]

                rwyname = acrte.wpname[rwyrteidx].split("/")[1]
                rwyid = rwyname.replace("RWY", "").replace("RW", "")
                rwyhdg = minisky.navdb.rwythresholds[aptname][rwyid][2]

            else:
                rwylat = minisky.traf.lat[acidx]
                rwylon = minisky.traf.lon[acidx]
                rwyhdg = minisky.traf.trk[acidx]

        elif args[1].count("/") > 0 or len(args) > 2 and args[2]:  # we need apt,rwy
            # Take care of both EHAM/RW06 as well as EHAM,RWY18L (so /&, and RW/RWY)
            if args[1].count("/") > 0:
                aptid, rwyname = args[1].split("/")
            else:
                # Runway specified
                aptid = args[1]
                rwyname = args[2]  # type: ignore[misc]

            rwyid = rwyname.replace("RWY", "").replace("RW", "")  # take away RW or RWY
            #                    print ("apt,rwy=",aptid,rwyid)
            # TODO: Add finding the runway heading with rwyrteidx>0 and navdb!!!
            # Try to get it from the database
            try:
                rwyhdg = minisky.navdb.rwythresholds[aptid][rwyid][2]
            except Exception:
                rwydir = rwyid.replace("L", "").replace("R", "").replace("C", "")
                try:
                    rwyhdg = float(rwydir) * 10.0
                except ValueError:
                    return False, name + " not found."

            success, posobj = txt2pos(aptid + "/RW" + rwyid, reflat, reflon)
            if success:
                assert isinstance(posobj, Position)
                rwylat, rwylon = posobj.lat, posobj.lon
            else:
                rwylat = minisky.traf.lat[acidx]
                rwylon = minisky.traf.lon[acidx]

        else:
            return False, "Use ADDWPT TAKEOFF,AIRPORTID,RWYNAME"

        # Create a waypoint 2 nm away from current point
        rwydist = 2.0  # [nm] use default distance away from threshold
        lat, lon = geo.qdrpos(rwylat, rwylon, rwyhdg, rwydist)  # [deg,deg
        wptype = Route.wplatlon

        # Add after the runwy in the route
        if rwyrteidx > 0:
            afterwp = acrte.wpname[rwyrteidx]

        elif acrte.wptype and acrte.wptype[0] == Route.orig:
            afterwp = acrte.wpname[0]

        else:
            # Assume we're called before other waypoints are added
            afterwp = ""

        name = "T/O-" + callsign  # Use lat/lon naming convention

    # Add waypoint
    wpidx = acrte.add_waypoint(acidx, name, wptype, lat, lon, alt, spd, afterwp, beforewp)

    # Recalculate flight plan
    acrte.calcfp()

    # Check for success by checking inserted location in flight plan >= 0
    if wpidx < 0:
        return False, "Waypoint " + name + " not added."

    # check for presence of orig/dest
    norig = int(minisky.traf.ap.orig[acidx] != "")  # 1 if orig is present in route
    ndest = int(minisky.traf.ap.dest[acidx] != "")  # 1 if dest is present in route

    # Check whether this is first 'real' waypoint (not orig & dest),
    # And if so, make active
    if n_wpt - norig - ndest == 1:  # first waypoint: make active
        direct(acidx, acrte.wpname[norig])  # 0 if no orig
        # print("direct ",self.wpname[norig])
        minisky.traf.swlnav[acidx] = True

    if afterwp and acrte.wpname.count(afterwp) == 0:
        return (
            True,
            "Waypoint " + afterwp + " not found\n" + "waypoint added at end of route",
        )
    else:
        return True


def addwpt_before(
    acidx: int,
    beforewp: Wpt,
    addwptkey,
    waypoint,
    alt: Alt | None = None,
    spd: Spd | None = None,
) -> bool | tuple:
    """Add a waypoint to a route before an existing waypoint.

    Implements the BEFORE stack command:
    ``acid BEFORE wpt ADDWPT (wpname/lat,lon), [alt], [spd]``.
    Thin wrapper around addwpt() with the insertion point set.

    Args:
        acidx: Aircraft index.
        beforewp: Name of the existing waypoint to insert before.
        addwptkey: The literal ADDWPT keyword (ignored).
        waypoint: Waypoint name or lat/lon text of the new waypoint.
        alt: Optional altitude constraint [m].
        spd: Optional speed constraint, CAS [m/s] or Mach [-].

    Returns:
        bool or tuple: Result of addwpt().
    """
    return addwpt(acidx, waypoint, alt, spd, None, beforewp)


def addwpt_after(
    acidx: int,
    afterwp: Wpt,
    addwptkey,
    waypoint,
    alt: Alt | None = None,
    spd: Spd | None = None,
) -> bool | tuple:
    """Add a waypoint to a route after an existing waypoint.

    Implements the AFTER stack command:
    ``acid AFTER wpt ADDWPT (wpname/lat,lon), [alt], [spd]``.
    Thin wrapper around addwpt() with the insertion point set.

    Args:
        acidx: Aircraft index.
        afterwp: Name of the existing waypoint to insert after.
        addwptkey: The literal ADDWPT keyword (ignored).
        waypoint: Waypoint name or lat/lon text of the new waypoint.
        alt: Optional altitude constraint [m].
        spd: Optional speed constraint, CAS [m/s] or Mach [-].

    Returns:
        bool or tuple: Result of addwpt().
    """
    return addwpt(acidx, waypoint, alt, spd, afterwp)


def at_wpt(acidx: int, atwp: Wpt, *args) -> bool | tuple:
    """Show, set or delete constraints and commands at a route waypoint.

    Implements the AT stack command:
    ``AT acid, wpt [DEL] ALT/SPD/DO alt/spd/stack command``.

    Usage examples:

    - ``KL204 AT LOPIK``: show altitude/speed constraints at the waypoint.
    - ``KL204 AT LOPIK FL090/250``: set both altitude and speed constraint.
    - ``KL204 AT LOPIK ALT FL090``: set the altitude constraint.
    - ``KL204 AT LOPIK SPD 250``: set the speed constraint.
    - ``KL204 AT LOPIK DO SPD 250``: stack a command when passing the
      waypoint (own callsign is prepended when the command needs one).
    - ``KL204 AT LOPIK DEL ALT/SPD/BOTH/ALL``: delete constraint(s).

    After editing, the flight plan and active-waypoint guidance are
    recalculated.

    Args:
        acidx: Aircraft index.
        atwp: Name of the waypoint in the route.
        *args: Remaining AT arguments as described above.

    Returns:
        bool or tuple: True on success, or (success flag, message).
    """
    acid = minisky.traf.callsign[acidx]
    acrte = minisky.traf.ap.route[acidx]
    if atwp in acrte.wpname:
        wpidx = acrte.wpname.index(atwp)

        if not args or (len(args) == 1 and args[0].count("/") != 1):
            # Only show Altitude and/or speed set in route at this waypoint:
            #    KL204 AT LOPIK => acid AT wpt: show alt & spd constraints at this waypoint
            #    KL204 AT LOPIK SPD => acid AT wpt SPD: show spd constraint at this waypoint
            #    KL204 AT LOPIK ALT => acid AT wpt ALT: show alt constraint at this waypoint
            txt = atwp + " : "

            # Select what to show
            if len(args) == 0:
                swalt = True
                swspd = True
                swat = True
            else:
                swalt = args[0].upper() == "ALT"
                swspd = args[0].upper() in ("SPD", "SPEED")
                swat = args[0].upper() in ("DO", "STACK")

                # To be safe show both when we do not know what
                if not (swalt or swspd or swat):
                    swalt = True
                    swspd = True
                    swat = True

            # Show altitude
            if swalt:
                if acrte.wpalt[wpidx] < 0:
                    txt += "-----"

                elif acrte.wpalt[wpidx] > 4500 * ft:
                    fl = int(round(acrte.wpalt[wpidx] / (100.0 * ft)))
                    txt += "FL" + str(fl)

                else:
                    txt += str(int(round(acrte.wpalt[wpidx] / ft)))

                if swspd:
                    txt += "/"

            # Show speed
            if swspd:
                if acrte.wpspd[wpidx] < 0:
                    txt += "---"
                else:
                    txt += str(int(round(acrte.wpspd[wpidx] / kts)))

            # Type
            if swalt and swspd:
                if acrte.wptype[wpidx] == Route.orig:
                    txt += "[orig]"
                elif acrte.wptype[wpidx] == Route.dest:
                    txt += "[dest]"

            # Show also stacked commands for when passing this waypoint
            if swat:
                if len(acrte.wpstack[wpidx]) > 0:
                    txt = txt + "\nStack:\n"
                    for stackedtxt in acrte.wpstack[wpidx]:
                        txt = txt + stackedtxt + "\n"

                return True, txt

        elif args[0].count("/") == 1:
            # Set both alt & speed at this waypoint
            #     KL204 AT LOPIK FL090/250  => acid AT wpt alt/spd
            success = True

            # Use parse from stack.py to interpret alt & speed
            alttxt, spdtxt = args[0].split("/")

            # Edit waypoint altitude constraint
            if alttxt.count("-") > 1:  # "----" = delete
                acrte.wpalt[wpidx] = -999.0
            else:
                try:
                    acrte.wpalt[wpidx] = txt2alt(alttxt)
                    acrte.calcfp()  # Recalculate VNAV axes
                except ValueError:
                    success = False

            # Edit waypoint speed constraint
            if spdtxt.count("-") > 1:  # "----" = delete
                acrte.wpspd[wpidx] = -999.0
            else:
                try:
                    acrte.wpspd[wpidx] = txt2spd(spdtxt)
                except ValueError:
                    success = False

            if not success:
                return False, "Could not parse " + args[0] + " as alt / spd"

            # If success: update flight plan and guidance
            acrte.calcfp()
            direct(acidx, acrte.wpname[acrte.iactwp])

        # acid AT wpt ALT/SPD alt/spd
        elif len(args) >= 2:
            # KL204 AT LOPIK ALT FL090 => set altitude to be reached at this waypoint in route
            # KL204 AT LOPIK SPD 250 => Set speed at twhich is set at this waypoint
            # KL204 AT LOPIK DO PAN LOPIK => When passing stack command after DO
            # KL204 AT LOPIK STACK PAN LOPIK => AT...STACK synonym for AT...DO
            # KL204 AT LOPIK DO ALT FL240 => => stack "KL204 ALT FL240" => use acid from beginning if omitted as first argument

            swalt = args[0].upper() == "ALT"
            swspd = args[0].upper() in ("SPD", "SPEED")
            swat = args[0].upper() in ("DO", "STACK")

            # Use parse from stack.py to interpret alt & speed

            # Edit waypoint altitude constraint
            if swalt:
                try:
                    acrte.wpalt[wpidx] = txt2alt(args[1])
                except ValueError as e:
                    return False, e.args[0]

            # Edit waypoint speed constraint
            elif swspd:
                try:
                    acrte.wpspd[wpidx] = txt2spd(args[1])
                except ValueError as e:
                    return False, e.args[0]

            # add stack command: args[1] is DO or STACK, args[2:] contains a command
            elif swat:
                # Check if first argument is missing aircraft id, if so, use this acid

                # IF command starts with aircraft id, it is not missing
                cmd = args[1].upper()
                if cmd not in minisky.traf.callsign:
                    # Look up arg types
                    try:
                        cmdobj = Command.cmddict[cmd]

                        # Command found, check arguments
                        argtypes = cmdobj.annotations  # type: ignore[attr-defined]

                        if (
                            len(argtypes) > 0
                            and argtypes[0] == int
                            and not (len(args) > 2 and args[2].upper() in minisky.traf.callsign)
                        ):
                            # missing acid, so add ownship acid
                            acrte.wpstack[wpidx].append(acid + " " + " ".join(args[1:]))
                        else:
                            # This command does not need an acid or it is already first argument
                            acrte.wpstack[wpidx].append(" ".join(args[1:]))
                    except Exception:
                        return (
                            False,
                            "Stacked command " + cmd + " unknown or syntax error",
                        )
                else:
                    # Command line starts with an aircraft id at the beginning of the command line, stack it
                    acrte.wpstack[wpidx].append(" ".join(args[1:]))

            # Delete a constraint (or both) at this waypoint
            elif args[0] == "DEL" or args[0] == "DELETE" or args[0] == "CLR" or args[0] == "CLEAR":
                swalt = args[1].upper() == "ALT"
                swspd = args[1].upper() in ("SPD", "SPEED")
                swboth = args[1].upper() == "BOTH"
                swall = args[1].upper() == "ALL"

                if swspd or swboth or swall:
                    acrte.wpspd[wpidx] = -999.0

                if swalt or swboth or swall:
                    acrte.wpalt[wpidx] = -999.0

                if swall:
                    acrte.wpstack[wpidx] = []

            else:
                return False, "No " + args[0] + " at ", atwp

            # If success: update flight plan and guidance
            acrte.calcfp()
            direct(acidx, acrte.wpname[acrte.iactwp])

    # Waypoint not found in route
    else:
        return False, atwp + " not found in route " + acid

    return True


def direct(acidx: int, wpname: Wpt) -> bool:
    """Go direct to a specified waypoint in the route.

    Implements the DIRECT stack command: ``DIRECT acid wpname``. Makes the
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
    minisky.traf.callsign[acidx]
    acrte = minisky.traf.ap.route[acidx]
    wpidx = acrte.wpname.index(wpname)

    acrte.iactwp = wpidx
    minisky.traf.actwp.lat[acidx] = acrte.wplat[wpidx]
    minisky.traf.actwp.lon[acidx] = acrte.wplon[wpidx]
    minisky.traf.actwp.flyby[acidx] = acrte.wpflyby[wpidx]
    minisky.traf.actwp.flyturn[acidx] = acrte.wpflyturn[wpidx]
    minisky.traf.actwp.turnrad[acidx] = acrte.wpturnrad[wpidx]
    minisky.traf.actwp.turnspd[acidx] = acrte.wpturnspd[wpidx]
    minisky.traf.actwp.turnhdgr[acidx] = acrte.wpturnhdgr[wpidx]

    (
        minisky.traf.actwp.nextturnlat[acidx],
        minisky.traf.actwp.nextturnlon[acidx],
        minisky.traf.actwp.nextturnspd[acidx],
        minisky.traf.actwp.nextturnrad[acidx],
        minisky.traf.actwp.nextturnhdgr[acidx],
        minisky.traf.actwp.nextturnidx[acidx],
    ) = acrte.getnextturnwp()

    # Determine next turn waypoint data

    # Do calculation for VNAV
    acrte.calcfp()

    minisky.traf.actwp.xtoalt[acidx] = acrte.wpxtoalt[wpidx]
    minisky.traf.actwp.nextaltco[acidx] = acrte.wptoalt[wpidx]

    minisky.traf.actwp.torta[acidx] = acrte.wptorta[wpidx]  # available for active RTA-guidance
    minisky.traf.actwp.xtorta[acidx] = acrte.wpxtorta[wpidx]  # available for active RTA-guidance

    # VNAV calculations like V/S and speed for RTA
    minisky.traf.ap.ComputeVNAV(
        acidx,
        acrte.wptoalt[wpidx],
        acrte.wpxtoalt[wpidx],
        acrte.wptorta[wpidx],
        acrte.wpxtorta[wpidx],
    )

    # If there is a speed specified, process it
    if acrte.wpspd[wpidx] > 0.0:
        # Set target speed for autopilot

        alt = minisky.traf.alt[acidx] if acrte.wpalt[wpidx] < 0.0 else acrte.wpalt[wpidx]

        # Check for valid Mach or CAS
        cas = mach2cas(acrte.wpspd[wpidx], alt) if acrte.wpspd[wpidx] < 2.0 else acrte.wpspd[wpidx]

        # Save it for next leg
        minisky.traf.actwp.nextspd[acidx] = cas

    # No speed specified for next leg
    else:
        minisky.traf.actwp.nextspd[acidx] = -999.0

    qdr_, dist_ = geo.qdrdist(
        minisky.traf.lat[acidx],
        minisky.traf.lon[acidx],
        minisky.traf.actwp.lat[acidx],
        minisky.traf.actwp.lon[acidx],
    )

    # Save leg length & direction in actwp data
    minisky.traf.actwp.curlegdir[acidx] = qdr_  # [deg]
    minisky.traf.actwp.curleglen[acidx] = dist_ * nm  # [m]

    if acrte.wpflyturn[wpidx] and acrte.wpturnrad[wpidx] > 0.0:  # turn radius specified
        turnrad = acrte.wpturnrad[wpidx]
    # Overwrite is hdgrate  defined
    if acrte.wpflyturn[wpidx] and acrte.wpturnhdgr[wpidx] > 0.0:  # heading rate specified
        turnrad = minisky.traf.tas[acidx] * 360.0 / (2 * math.pi * acrte.wpturnhdgr[wpidx])
    else:  # nothing specified, use default bank ang;e
        turnrad = (
            minisky.traf.tas[acidx]
            * minisky.traf.tas[acidx]
            / math.tan(math.radians(acrte.bank))
            / g0
            / nm
        )  # [nm]default bank angle e.g. 25 deg

    minisky.traf.actwp.turndist[acidx] = (
        np.logical_or(acrte.wpturnhdgr[wpidx] > 0.0, minisky.traf.actwp.flyby[acidx] > 0.5)
        * turnrad
        * abs(
            math.tan(
                0.5 * math.radians(max(5.0, abs(degto180(qdr_ - acrte.wpdirfrom[acrte.iactwp]))))
            )
        )
    )  # [nm]

    minisky.traf.swlnav[acidx] = True
    return True


def set_rta(acidx: int, wpname: Wpt, time: Time) -> bool:  # all arguments of setRTA
    """Set a required time of arrival (RTA) at a route waypoint.

    Implements the RTA stack command: ``RTA acid, wpname, time``. The RTA
    is stored with the waypoint and the guidance to the active waypoint is
    recomputed so the autopilot can adjust its speed schedule.

    Args:
        acidx: Aircraft index.
        wpname: Name of the waypoint in the route.
        time: Required time of arrival as simulation time [s].

    Returns:
        bool: True on success.
    """
    minisky.traf.callsign[acidx]
    acrte = minisky.traf.ap.route[acidx]
    wpidx = acrte.wpname.index(wpname)
    acrte.wprta[wpidx] = time

    # Recompute route and update actwp because of RTA addition
    direct(acidx, acrte.wpname[acrte.iactwp])

    return True


def listrte(acidx: int, ipagetxt: str = "0") -> tuple | None:
    """Show the route of an aircraft in the console, page by page.

    Implements the LISTRTE stack command: ``LISTRTE acid, [pagenr]``.
    Each line shows the waypoint name (active waypoint marked with ``*``),
    its altitude constraint (ft or FL), speed constraint (kts or Mach) and
    type ([orig], [dest], [C] fly-by, [|] fly-over, [U] fly-turn). Seven
    waypoints are shown per page.

    Args:
        acidx: Aircraft index.
        ipagetxt: Page number as text (default "0").

    Returns:
        tuple or None: (False, message) when the aircraft has no route.
    """
    # First get the appropriate ac route
    ipage = int(ipagetxt)
    acrte = minisky.traf.ap.route[acidx]

    n_wpt = len(acrte.wpname)

    if n_wpt <= 0:
        return False, "Aircraft has no route."

    for i in range(ipage * 7, ipage * 7 + 7):
        if 0 <= i < n_wpt:
            # Name
            if i == acrte.iactwp:
                txt = "*" + acrte.wpname[i] + " : "
            else:
                txt = " " + acrte.wpname[i] + " : "

            # Altitude
            if acrte.wpalt[i] < 0:
                txt += "-----/"

            elif acrte.wpalt[i] > 4500 * ft:
                fl = int(round(acrte.wpalt[i] / (100.0 * ft)))
                txt += "FL" + str(fl) + "/"

            else:
                txt += str(int(round(acrte.wpalt[i] / ft))) + "/"

            # Speed
            if acrte.wpspd[i] < 0.0:
                txt += "---"
            elif acrte.wpspd[i] > 2.0:
                txt += str(int(round(acrte.wpspd[i] / kts)))
            else:
                txt += "M" + str(acrte.wpspd[i])

            # Type: orig, dest, C = flyby, | = flyover, U = flyturn
            if acrte.wptype[i] == Route.orig:
                txt += "[orig]"
            elif acrte.wptype[i] == Route.dest:
                txt += "[dest]"
            elif acrte.wpflyturn[i]:
                txt += "[U]"
            elif acrte.wpflyby[i]:
                txt += "[C]"
            else:  # FLYOVER
                txt += "[|]"

            # Display message
            minisky.scr.echo(txt)


def delrte(acidx: int | None = None) -> bool | tuple:
    """Delete the complete route (including origin/destination) of an
    aircraft.

    Implements the DELRTE stack command: ``DELRTE acid``. The route is
    re-initialized empty and LNAV/VNAV are disengaged. When no callsign is
    given and exactly one aircraft exists, that aircraft is used.

    Args:
        acidx: Aircraft index; may be None when only one aircraft exists.

    Returns:
        bool or tuple: True on success, or (False, error message).
    """
    if acidx is None:
        if minisky.traf.ntraf == 0:
            return False, "No aircraft in simulation"
        if minisky.traf.ntraf > 1:
            return False, "Specify callsign of aircraft to delete route of"
        acidx = 0
    # Simple re-initialize this route as empty
    acid = minisky.traf.callsign[acidx]
    acrte = minisky.traf.ap.route[acidx]
    acrte.__init__(acid)

    # Also disable LNAV,VNAV if route is deleted
    minisky.traf.swlnav[acidx] = False
    minisky.traf.swvnav[acidx] = False
    minisky.traf.swvnavspd[acidx] = False

    return True


def delwpt(acidx: int, wpname: Wpt) -> bool | tuple:
    """Delete a single waypoint from the route of an aircraft.

    Implements the DELWPT stack command: ``DELWPT acid, wpname``. When the
    deleted waypoint is the active one (and not the last), guidance is
    redirected to the following waypoint. LNAV/VNAV are disengaged when
    the route becomes empty.

    Args:
        acidx: Aircraft index.
        wpname: Name of the waypoint to delete.

    Returns:
        bool or tuple: True on success, or (False, error message).
    """

    # Look up waypoint
    acrte = minisky.traf.ap.route[acidx]
    n_wpt = len(acrte.wpname)

    try:
        wpidx = acrte.wpname.index(wpname.upper())
    except ValueError:
        return False, "Waypoint " + wpname + " not found"

    # check if active way point is the one being deleted and that it is not the last wpt.
    # If active wpt is deleted then change path of aircraft
    if acrte.iactwp == wpidx and wpidx != n_wpt - 1:
        direct(acidx, acrte.wpname[wpidx + 1])

    n_wpt = n_wpt - 1

    del acrte.wpname[wpidx]
    del acrte.wplat[wpidx]
    del acrte.wplon[wpidx]
    del acrte.wpalt[wpidx]
    del acrte.wpspd[wpidx]
    del acrte.wprta[wpidx]
    del acrte.wptype[wpidx]
    del acrte.wpflyby[wpidx]
    del acrte.wpflyturn[wpidx]
    del acrte.wpturnrad[wpidx]
    del acrte.wpturnspd[wpidx]
    del acrte.wpturnhdgr[wpidx]
    del acrte.wpstack[wpidx]

    if acrte.iactwp > wpidx:
        acrte.iactwp = max(0, acrte.iactwp - 1)

    acrte.iactwp = min(acrte.iactwp, n_wpt - 1)

    # If no waypoints left, make sure to disable LNAV/VNAV
    if n_wpt == 0 and (acidx or acidx == 0):
        minisky.traf.swlnav[acidx] = False
        minisky.traf.swvnav[acidx] = False
        minisky.traf.swvnavspd[acidx] = False

    return True
