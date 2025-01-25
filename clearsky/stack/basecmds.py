"""BlueSky base stack commands."""

import clearsky as cs
from clearsky.core import select_implementation, simtime
from clearsky.core import varexplorer as ve
from clearsky.stack.cmdparser import append_commands
from clearsky.tools import aero, areafilter, geo


def initbasecmds():
    """Initialise BlueSky base stack commands."""
    # Command dictionary with command as key, gives a list with:
    #
    #         command: [ helptext ,
    #                    arglist ,
    #                    function to call,
    #                    description in one line ]
    #
    # Regarding the arglist:
    #    - Separate aruments with a comma ","
    #    - Enclose optional arguments with "[" and "]"
    #    - Separate different argument type variants in one argument with "/"
    #    - Repeat last one using "..." ,    (see e.g. WIND or POLY)
    #
    # Argtypes = syntax parsing (see below in this module for parsing):
    #
    #   acid      = aircraft id (text => index)
    #   alt       = altitude (FL250, 25000  ft+. meters)
    #   spd       = CAS or Mach (when <1)   => m/s
    #   hdg       = heading in degrees, True or Magnetic
    #
    #   float     = plain float
    #   int       = integer
    #   txt       = text will be converted to upper case
    #               (for keywords, navaids, flags, waypoints,acid etc)
    #   word      = single, case sensitive word
    #   string    = case sensitive string
    #   on/off    = text => boolean
    #
    #   latlon    = converts acid, wpt, airport etc => lat,lon (deg) so 2 args!
    #   wpt       = converts postext or lat,lon into a text string,
    #               to be used as named waypoint
    #   wpinroute = text string with name of waypoint in route
    #   pandir    = text with LEFT, RIGHT, UP/ABOVE or DOWN
    #
    # Below this dictionary also a dictionary of synonym commands is given
    #
    # --------------------------------------------------------------------
    cmddict = {
        "ATALT": [
            "acid ATALT alt cmd ",
            "acid,alt,string",
            cs.traf.cond.ataltcmd,
            "When a/c at given altitude , execute a command cmd",
        ],
        "ATDIST": [
            "acid ATDIST pos dist cmd ",
            "acid,latlon,float,string",
            cs.traf.cond.atdistcmd,
            "When a/c passing this distance[nm] to position, execute the command cmd",
        ],
        "ATSPD": [
            "acid ATSPD spd cmd ",
            "acid,spd,string",
            cs.traf.cond.atspdcmd,
            "When a/c reaches given speed, execute a command cmd",
        ],
        "BANK": [
            "BANK acid bankangle[deg]",
            "acid,[float]",
            cs.traf.setbanklim,
            "Set or show bank limit for this vehicle",
        ],
        "BENCHMARK": [
            "BENCHMARK [scenfile,time]",
            "[string,time]",
            cs.sim.benchmark,
            "Run benchmark",
        ],
        "BOX": [
            "BOX name,lat,lon,lat,lon,[top,bottom]",
            "txt,latlon,latlon,[alt,alt]",
            lambda name, *coords: areafilter.defineArea(
                name, "BOX", coords[:4], *coords[4:]
            ),
            "Define a box-shaped area",
        ],
        "CASMACHTHR": [
            "CASMACHTHR threshold",
            "float",
            aero.casmachthr,
            """Set a threshold below which speeds should be considered as Mach numbers
               in CRE(ATE), ADDWPT, and SPD commands. Set to zero if speeds should
               never be considered as Mach number(e.g., when simulating drones).""",
        ],
        "CIRCLE": [
            "CIRCLE name,lat,lon,radius,[top,bottom]",
            "txt,latlon,float,[alt,alt]",
            lambda name, *coords: areafilter.defineArea(
                name, "CIRCLE", coords[:3], *coords[3:]
            ),
            "Define a circle-shaped area",
        ],
        "CLRCRECMD": [
            "CLRCRECMD",
            "",
            cs.traf.clrcrecmd,
            "CLRCRECMD will clear CRECMD list of commands a/c creation",
        ],
        "CRE": [
            "CRE acid,type,lat,lon,hdg,alt,spd",
            "txt,txt,float,float,[hdg,alt,spd]",
            cs.traf.cre,
            "Create an aircraft",
        ],
        "CRECMD": [
            "CRECMD cmdline (to be added after a/c id )",
            "string",
            cs.traf.crecmd,
            "Add a command for each aircraft to be issued after creation of aircraft",
        ],
        "CRECONFS": [
            "CRECONFS id, type, targetid, dpsi, cpa, tlos_hor, dH, tlos_ver, spd",
            "txt,txt,acid,hdg,float,time,[alt,time,spd]",
            cs.traf.creconfs,
            "Create an aircraft that is in conflict with 'targetid'",
        ],
        "DATE": [
            "DATE [day,month,year,HH:MM:SS.hh]",
            "[int,int,int,txt]",
            cs.sim.setutc,
            "Set simulation date",
        ],
        "DEFWPT": [
            "DEFWPT wpname,lat,lon,[DELETE/FIX/VOR/DME/NDB/DEL]",
            "txt,latlon,[txt]",
            cs.navdb.defwpt,
            "Define (or delete) a waypoint only for this scenario/run",
        ],
        "DEL": [
            "DEL acid/ALL/WIND/shape",
            "acid/txt,...",
            lambda *a: (
                cs.traf.wind.clear()
                if isinstance(a[0], str) and a[0] == "WIND"
                else (
                    areafilter.deleteArea(a[0])
                    if isinstance(a[0], str)
                    else (
                        cs.traf.groups.delgroup(a[0])
                        if hasattr(a[0], "groupname")
                        else cs.traf.delete(a)
                    )
                )
            ),
            "Delete command (aircraft, wind, area)",
        ],
        "DIST": [
            "DIST lat0, lon0, lat1, lon1",
            "latlon,latlon",
            distcalc,
            "Distance and direction calculation between two positions",
        ],
        "DT": [
            "DT [dt] OR [target,dt]",
            "[float/txt,float]",
            lambda *args: simtime.setdt(*reversed(args)),
            "Set simulation time step",
        ],
        "DTMULT": [
            "DTMULT multiplier",
            "float",
            cs.sim.set_dtmult,
            "Sel multiplication factor for fast-time simulation",
        ],
        "ECHO": [
            "ECHO txt",
            "string",
            cs.scr.echo,
            "Show a text in command window for user to read",
        ],
        "FF": [
            "FF [timeinsec]",
            "[time]",
            cs.sim.fastforward,
            "Fast forward the simulation",
        ],
        "FIXDT": [
            "FIXDT ON/OFF [tend]",
            "onoff,[time]",
            lambda flag, *args: cs.sim.ff(*args) if flag else cs.op(),
            "Legacy function for TMX compatibility",
        ],
        "GROUP": [
            "GROUP [grname, (areaname OR acid,...) ]",
            "[txt,acid/txt,...]",
            cs.traf.groups.group,
            "Add aircraft to a group. OR all aircraft in given area.\n"
            + "Returns list of groups when no argument is passed.\n"
            + "Returns list of aircraft in group when only a groupname is passed.\n"
            + "A group is created when a group with the given name doesn't exist yet.",
        ],
        "HOLD": ["HOLD", "", cs.sim.hold, "Pause(hold) simulation"],
        "IMPLEMENTATION": [
            "IMPLEMENTATION [base, implementation]",
            "[txt,txt]",
            select_implementation,
            "Select an alternate implementation for a Bluesky base class",
        ],
        "LINE": [
            "LINE name,lat,lon,lat,lon",
            "txt,latlon,latlon",
            lambda name, *coords: areafilter.defineArea(name, "LINE", coords),
            "Draw a line on the radar screen",
        ],
        "LSVAR": [
            "LSVAR path.to.variable",
            "[word]",
            ve.lsvar,
            "Inspect any variable in a simulation",
        ],
        "MAGVAR": [
            "MAGVAR lat,lon",
            "lat,lon",
            cs.tools.geo.magdeccmd,
            "Show magnetic variation/declination at position",
        ],
        "MCRE": [
            "MCRE n,[lat,lon,lat,lon,type,alt,spd]",
            "int,[float,float,float,float,txt,alt,spd]",
            cs.traf.mcre,
            "Multiple random create of n aircraft in current view",
        ],
        "MOVE": [
            "MOVE acid,lat,lon,[alt,hdg,spd,vspd]",
            "acid,latlon,[alt,hdg,spd,vspd]",
            cs.traf.move,
            "Move an aircraft to a new position",
        ],
        "NOISE": [
            "NOISE [ON/OFF]",
            "[onoff]",
            cs.traf.setnoise,
            "Turbulence/noise switch",
        ],
        "OP": ["OP", "", cs.sim.op, "Start/Run simulation or continue after hold"],
        "POLY": [
            "POLY name,[lat,lon,lat,lon, ...]",
            "txt,[latlon,...]",
            lambda name, *coords: areafilter.defineArea(name, "POLY", coords),
            "Define a polygon-shaped area",
        ],
        "POLYALT": [
            "POLYALT name,top,bottom,lat,lon,lat,lon, ...",
            "txt,alt,alt,latlon,...",
            lambda name, top, bottom, *coords: areafilter.defineArea(
                name, "POLYALT", coords, top, bottom
            ),
            "Define a polygon-shaped area in 3D: between two altitudes",
        ],
        "POLYLINE": [
            "POLYLINE name,lat,lon,lat,lon,...",
            "txt,latlon,...",
            lambda name, *coords: areafilter.defineArea(name, "LINE", coords),
            "Draw a multi-segment line on the radar screen",
        ],
        "POS": [
            "POS acid/waypoint",
            "acid/wpt",
            cs.traf.poscommand,
            "Get info on aircraft, airport or waypoint",
        ],
        "QUIT": ["QUIT", "", cs.sim.stop, "Quit program/Stop simulation"],
        "REALTIME": [
            "REALTIME [ON/OFF]",
            "[bool]",
            cs.sim.realtime,
            "En-/disable realtime running allowing a variable timestep.",
        ],
        "RESET": ["RESET", "", cs.sim.reset, "Reset simulation"],
        "SEED": [
            "SEED value",
            "int",
            cs.sim.setseed,
            "Set seed for all functions using a randomizer (e.g.mcre,noise)",
        ],
        "THR": [
            "THR acid, IDLE/0.0/throttlesetting/1.0/AUTO(default)",
            "acid[,txt]",
            cs.traf.setthrottle,
            "Set throttle or autotothrottle(default)",
        ],
        "TIME": [
            "TIME RUN(default) / HH:MM:SS.hh / REAL / UTC ",
            "[txt]",
            cs.sim.setutc,
            "Set simulated clock time",
        ],
        "TRAIL": [
            "TRAIL ON/OFF, [dt] OR TRAIL acid colour",
            "[acid/bool],[float/txt]",
            cs.traf.trails.setTrails,
            "Toggle aircraft trails on/off",
        ],
        "UNGROUP": [
            "UNGROUP grname, acid",
            "txt,acid,...",
            cs.traf.groups.ungroup,
            "Remove aircraft from a group",
        ],
    }

    #
    # Command synonym dictionary definea equivalent commands globally in stack
    #
    # Actual command definitions: see dictionary in def init(...) below
    #
    synonyms = {
        "ADDAIRWAY": "ADDAWY",
        "AWY": "POS",
        "AIRPORT": "POS",
        "AIRWAYS": "AIRWAY",
        "BANKLIM": "BANK",
        "CHDIR": "CD",
        "COL": "COLOUR",
        "COLOR": "COLOUR",
        "CONTINUE": "OP",
        "CREATE": "CRE",
        "CLOSE": "QUIT",
        "DEBUG": "CALC",
        "DELETE": "DEL",
        "DISP": "SWRAD",
        "END": "QUIT",
        "EXIT": "QUIT",
        "FWD": "FF",
        "IMPL": "IMPLEMENTATION",
        "IMPLEMENT": "IMPLEMENTATION",
        "LINES": "POLYLINE",
        "MAGDEC": "MAGVAR",
        "MAGDECL": "MAGVAR",
        "PAUSE": "HOLD",
        "POLYGON": "POLY",
        "POLYLINES": "POLYLINE",
        "PRINT": "ECHO",
        "Q": "QUIT",
        "RT": "REALTIME",
        "RTF": "DTMULT",
        "STOP": "QUIT",
        "RUN": "OP",
        "RUNWAYS": "POS",
        "SAVE": "SAVEIC",
        "START": "OP",
        "TRAILS": "TRAIL",
        "VAR": "MAGVAR",
    }

    append_commands(cmddict, synonyms)


def distcalc(lat0, lon0, lat1, lon1):
    try:
        qdr, dist = geo.qdrdist(lat0, lon0, lat1, lon1)
        return True, "QDR = %.2f deg, Dist = %.3f nm" % (qdr % 360.0, dist)
    except:
        return False, "Error in dist calculation."
