# --------------------------------------------------------------------
#
# Command dictionary with command as key, gives a list with:
#
# command: [
#     function,
#     arglist,
#     helptext,
#     description in one line,
# ]
#
# Regarding the arglist:
#    - Separate aruments with a comma ","
#    - Enclose optional arguments with "[" and "]"
#    - Separate different argument type variants in one argument with "/"
#    - Repeat last one using "..." ,    (see e.g. WIND or POLY)
#
# Argtypes = syntax parsing (see below in this module for parsing):
#
#   callsign  = callsign (text will be converted to index)
#   alt       = altitude (FL250, 25000  ft+. meters)
#   spd       = CAS or Mach (when <1)   => m/s
#   hdg       = heading in degrees, True or Magnetic
#
#   float     = plain float
#   int       = integer
#   txt       = text will be converted to upper case
#               (for keywords, navaids, flags, waypoints, callsign etc)
#   word      = single, case sensitive word
#   string    = case sensitive string
#   on/off    = text => boolean
#
#   latlon    = converts callsign, wpt, airport etc => lat,lon (deg) so 2 args!
#   wpt       = converts postext or lat,lon into a text string,
#               to be used as named waypoint
#   pandir    = text with LEFT, RIGHT, UP/ABOVE or DOWN
#
# Below this dictionary also a dictionary of synonym commands is given
#
# --------------------------------------------------------------------

"""Definition of the base stack commands of the simulator.

This module contains the command dictionary that couples every base text
command of the simulator (e.g., CRE, ALT, HDG) to the Python function that
implements it, its argument type specification, and its usage and help
texts, plus a dictionary of command synonyms. Both dictionaries are
registered with the command interpreter in minisky.stack.init().

The strings in the command dictionary are the in-simulator help texts
shown by the HELP command.
"""


def get_commands() -> tuple:
    """Assemble the base command and synonym dictionaries of the simulator.

    Imports minisky at call time so that command callbacks can be bound to
    the fully initialised simulation objects (traf, sim, navdb, scr, ...).

    Returns:
        tuple: (cmddict, synonyms). cmddict maps a command name to a list
        of [function, argument type string, brief usage text, help text];
        synonyms maps a command name to a list of alias names.
    """
    import minisky
    from minisky.traffic.asas import resolution as asasresolution

    cmddict = {
        "ADDWPT": [
            minisky.traffic.route.addwpt,
            "callsign,wpt,[alt,spd,wpt,wpt]",
            "ADDWPT callsign, wpt, [alt, spd, wpt, wpt]",
            "Add a waypoint to the route.",
        ],
        "ADDWPTMODE": [
            minisky.traffic.route.change_wpt_mode,
            "callsign, [wpt,alt]",
            "ADDWPTMODE callsign, [wpt,alt]",
            "Changes the mode of the ADDWPT command to add waypoints of type 'mode'.",
        ],
        "AFTER": [
            minisky.traffic.route.addwpt_after,
            "callsign,wpt,txt,wpt,[alt,spd]",
            "AFTER callsign, wpt, addwpt, waypoint, [alt, spd]",
            "Add a waypoint after another waypoint in the route.",
        ],
        "ALT": [
            minisky.traf.ap.selaltcmd,
            "callsign,alt,[vspd]",
            "ALT callsign, alt, [vspd]",
            "Select autopilot altitude command.",
        ],
        "ASAS": [
            minisky.traf.cd.switch,
            "[txt]",
            "ASAS [ON/OFF]",
            "Select a Conflict Detection method.",
        ],
        "AT": [
            minisky.traffic.route.at_wpt,
            "callsign,wpt,[txt,...]",
            "AT callsign, wpt, [DEL] ALT/SPD/DO alt/spd/stack command",
            "Set or show altitude and/or speed constraints at a waypoint.",
        ],
        "ATALT": [
            minisky.traf.cond.ataltcmd,
            "callsign,alt,string",
            "callsign ATALT alt cmd ",
            "When aircraft at given altitude , execute the command",
        ],
        "ATDIST": [
            minisky.traf.cond.atdistcmd,
            "callsign,latlon,float,string",
            "callsign ATDIST pos dist cmd ",
            "When aircraft passing this distance (in nm) to position, execute the command",
        ],
        "ATSPD": [
            minisky.traf.cond.atspdcmd,
            "callsign,spd,string",
            "callsign ATSPD spd cmd ",
            "When aircraft reaches given speed, execute the command",
        ],
        "BANK": [
            minisky.traf.setbanklim,
            "callsign,[float]",
            "BANK callsign bankangle[deg]",
            "Set or show bank limit for this vehicle",
        ],
        "BEFORE": [
            minisky.traffic.route.addwpt_before,
            "callsign,wpt,txt,wpt,[alt,spd]",
            "BEFORE callsign, wpt, addwpt, waypoint, [alt, spd]",
            "Add a waypoint before another waypoint in the route.",
        ],
        "BOX": [
            minisky.tools.areafilter.define_box_area,
            "txt,latlon,latlon,[alt,alt]",
            "BOX name,lat,lon,lat,lon,[top,bottom]",
            "Define a box-shaped area",
        ],
        "CASMACHTHR": [
            minisky.tools.aero.casmachthr,
            "float",
            "CASMACHTHR threshold",
            """Set a threshold below which speeds should be considered as Mach numbers
                in CRE(ATE), ADDWPT, and SPD commands. Set to zero if speeds should
                never be considered as Mach number(e.g., when simulating drones).""",
        ],
        "CIRCLE": [
            minisky.tools.areafilter.define_circle_area,
            "txt,latlon,float,[alt,alt]",
            "CIRCLE name,lat,lon,radius,[top,bottom]",
            "Define a circle-shaped area",
        ],
        "CLRCRECMD": [
            minisky.traf.clrcrecmd,
            "",
            "CLRCRECMD",
            "CLRCRECMD will clear CRECMD list of commands aircraft creation",
        ],
        "CRE": [
            minisky.traf.cre,
            "txt,txt,float,float,[hdg,alt,spd]",
            "CRE callsign,type,lat,lon,hdg,alt,spd",
            "Create an aircraft",
        ],
        "CRECMD": [
            minisky.traf.crecmd,
            "string",
            "CRECMD cmdline (to be added after aircraft id )",
            "Add a command for each aircraft to be issued after creation of aircraft",
        ],
        "CRECONFS": [
            minisky.traf.creconfs,
            "txt,txt,callsign,hdg,float,time,[alt,time,spd]",
            "CRECONFS id, type, targetid, dpsi, cpa, tlos_hor, dH, tlos_ver, spd",
            "Create an aircraft that is in conflict with 'targetid'",
        ],
        "DATE": [
            minisky.sim.setutc,
            "[int,int,int,txt]",
            "DATE [day,month,year,HH:MM:SS.hh]",
            "Set simulation date",
        ],
        "DEFWPT": [
            minisky.navdb.defwpt,
            "txt,latlon,[txt]",
            "DEFWPT wpname,lat,lon,[DELETE/FIX/VOR/DME/NDB/DEL]",
            "Define (or delete) a waypoint only for this scenario/run",
        ],
        "DEL": [
            minisky.stack.delete_element,
            "callsign/txt,...",
            "DEL callsign/ALL/WIND/shape",
            "Delete command (aircraft, wind, area)",
        ],
        "DELAY": [
            minisky.stack.delay,
            "time, string",
            "DELAY time, cmdline",
            "Delay a stack command until a specific simulation time.",
        ],
        "DELRTE": [
            minisky.traffic.route.delrte,
            "callsign",
            "DELRTE callsign",
            "Delete the complete route for an aircraft.",
        ],
        "DELWPT": [
            minisky.traffic.route.delwpt,
            "callsign,wpt",
            "DELWPT callsign,wpt",
            "Delete a waypoint from a route.",
        ],
        "DEST": [
            minisky.traf.ap.setdest,
            "callsign,wpt,[spd]",
            "DEST callsign, latlon/airport, casmach (= CASkts/Mach)",
            "Set destination of aircraft, aircraft will fly to this airport.",
        ],
        "DIRECT": [
            minisky.traffic.route.direct,
            "callsign, wpt",
            "DIRECT callsign, wpt",
            "Go direct to a specified waypoint in the route.",
        ],
        "DTLOOK": [
            minisky.traf.cd.setdtlook,
            "[time,callsign,...]",
            "DTLOOK [time, callsign...]",
            "Set the lookahead time (in [hh:mm:]sec) for conflict detection.",
        ],
        "DTNOLOOK": [
            minisky.traf.cd.setdtnolook,
            "[time,callsign,...]",
            "DTNOLOOK [time, callsign...]",
            "Set the interval (in [hh:mm:]sec) in which conflict detection is skipped after a conflict resolution.",
        ],
        "ECHO": [
            minisky.scr.echo,
            "string",
            "ECHO txt",
            "Show a text in command window for user to read",
        ],
        "GETWIND": [
            minisky.traf.wind.get,
            "lat, lon, [alt]",
            "GETWIND lat, lon, [alt]",
            "Get wind at a specified position (and optionally at altitude).",
        ],
        "GROUP": [
            minisky.traf.groups.group,
            "[txt,callsign/txt,...]",
            "GROUP [grname, (areaname OR callsign,...) ]",
            "Add aircraft to a group. OR all aircraft in given area.\n"
            + "Returns list of groups when no argument is passed.\n"
            + "Returns list of aircraft in group when only a groupname is passed.\n"
            + "A group is created when a group with the given name doesn't exist yet.",
        ],
        "HDG": [
            minisky.traf.ap.selhdgcmd,
            "callsign,hdg",
            "HDG callsign,hdg (deg,True or Magnetic)",
            "Autopilot select heading command.",
        ],
        "HELP": [
            minisky.stack.showhelp,
            "[txt,txt]",
            "HELP [cmd, subcmd]",
            "Display general help text or help text for a specific command.",
        ],
        "HOLD": [
            minisky.sim.hold,
            "",
            "HOLD",
            "Pause(hold) simulation",
        ],
        "IC": [
            minisky.stack.ic,
            "string",
            "IC scenario_filename",
            "Load a scenario filename.",
        ],
        "LINE": [
            minisky.tools.areafilter.define_line_area,
            "txt,latlon,latlon",
            "LINE name,lat,lon,lat,lon",
            "Draw a line on the radar screen",
        ],
        "LISTRTE": [
            minisky.traffic.route.listrte,
            "callsign,[txt]",
            "LISTRTE callsign, [pagenr]",
            "Show list of route in window per page of 5 waypoints.",
        ],
        "LNAV": [
            minisky.traf.ap.setLNAV,
            "callsign,[bool]",
            "LNAV callsign,[ON/OFF]",
            "LNAV (lateral FMS mode) switch for autopilot.",
        ],
        "LSVAR": [
            minisky.core.varexplorer.lsvar,
            "[word]",
            "LSVAR path.to.variable",
            "Inspect any variable in a simulation",
        ],
        "MAGVAR": [
            minisky.tools.geo.magdeccmd,
            "lat,lon",
            "MAGVAR lat,lon",
            "Show magnetic variation/declination at position",
        ],
        "MCRE": [
            minisky.traf.mcre,
            "int,[float,float,float,float,txt,alt,spd]",
            "MCRE n,[lat,lon,lat,lon,type,alt,spd]",
            "Multiple random create of n aircraft in current view",
        ],
        "MOVE": [
            minisky.traf.move,
            "callsign,latlon,[alt,hdg,spd,vspd]",
            "MOVE callsign,lat,lon,[alt,hdg,spd,vspd]",
            "Move an aircraft to a new position",
        ],
        "NOISE": [
            minisky.traf.setnoise,
            "[onoff]",
            "NOISE [ON/OFF]",
            "Turbulence/noise switch",
        ],
        "NORESO": [
            asasresolution.setnoreso,
            "[callsign,...]",
            "NORESO callsign...",
            "ADD or Remove aircraft that nobody will avoid.",
        ],
        "OP": [
            minisky.sim.op,
            "",
            "OP",
            "Start/Run simulation or continue after hold",
        ],
        "PERFSTATS": [
            minisky.traf.perf.show_performance,
            "callsign",
            "PERFSTATS callsign",
            "Show the performace information of an aircraft.",
        ],
        "ORIG": [
            minisky.traf.ap.setorig,
            "callsign,wpt",
            "ORIG callsign, latlon/airport",
            "Set origin of aircraft.",
        ],
        "PLUGINS": [
            minisky.plugin.manage_plugins,
            "[txt,txt]",
            "PLUGINS [LIST/LOAD, plugin_name]",
            "List available plugins or load a plugin",
        ],
        "POLY": [
            minisky.tools.areafilter.define_poly_area,
            "txt,[latlon,...]",
            "POLY name,[lat,lon,lat,lon, ...]",
            "Define a polygon-shaped area",
        ],
        "POLYALT": [
            minisky.tools.areafilter.define_polyalt_area,
            "txt,alt,alt,latlon,...",
            "POLYALT name,top,bottom,lat,lon,lat,lon, ...",
            "Define a polygon-shaped area in 3D: between two altitudes",
        ],
        "POLYLINE": [
            minisky.tools.areafilter.define_polyline_area,
            "txt,latlon,...",
            "POLYLINE name,lat,lon,lat,lon,...",
            "Draw a multi-segment line on the radar screen",
        ],
        "POS": [
            minisky.traf.position,
            "callsign/wpt",
            "POS callsign/waypoint",
            "Get info on aircraft, airport or waypoint",
        ],
        "PRIORULES": [
            asasresolution.setprio,
            "[bool, txt]",
            "PRIORULES [flag, priocode]",
            "Define priority rules (right of way) for conflict resolution.",
        ],
        "QUIT": [
            minisky.sim.stop,
            "",
            "QUIT",
            "Quit program/Stop simulation",
        ],
        "REALTIME": [
            minisky.sim.realtime,
            "[bool]",
            "REALTIME [ON/OFF]",
            "En-/disable realtime running allowing a variable timestep.",
        ],
        "RESET": [
            minisky.sim.reset,
            "",
            "RESET",
            "Reset simulation",
        ],
        "RESO": [
            minisky.traf.cr.setmethod,
            "[txt]",
            "RESO [name]",
            "Select a Conflict Resolution method.",
        ],
        "RESOOFF": [
            asasresolution.setresooff,
            "[callsign,...]",
            "RESOOFF callsign...",
            "ADD or Remove aircraft that will not avoid anybody else.",
        ],
        "RMETHH": [
            asasresolution.setresometh,
            "[txt]",
            "RMETHH [ON / BOTH / OFF / NONE / SPD / HDG]",
            "Select the horizontal resolution method for MVP conflict resolution.",
        ],
        "RMETHV": [
            asasresolution.setresometv,
            "[txt]",
            "RMETHV [ON / V/S / OFF / NONE]",
            "Select the vertical resolution method for MVP conflict resolution.",
        ],
        "RFACH": [
            asasresolution.setresofach,
            "[float]",
            "RFACH [factor]",
            "Set resolution factor horizontal.",
        ],
        "RFACV": [
            asasresolution.setresofacv,
            "[float]",
            "RFACV [factor]",
            "Set resolution factor vertical.",
        ],
        "RTA": [
            minisky.traffic.route.set_rta,
            "callsign, wpt, time",
            "RTA callsign, wpt, time",
            "Add RTA to waypoint record.",
        ],
        "RSZONEDH": [
            asasresolution.setresozonedh,
            "[float]",
            "RSZONEDH [zonedh]",
            "Set resolution factor vertical, but then with absolute value.",
        ],
        "RSZONER": [
            asasresolution.setresozoner,
            "[float]",
            "RSZONER [zoner]",
            "Set resolution factor horizontal, but then with absolute value.",
        ],
        "SCHEDULE": [
            minisky.stack.schedule,
            "time,string",
            "SCHEDULE a stack command at a specific simulation time.",
            "Schedule a stack command at a specific simulation time.",
        ],
        "SCENARIO": [
            minisky.stack.scenario,
            "string",
            "SCENARIO name",
            "Sets the scenario name for the current simulation.",
        ],
        "SEED": [
            minisky.sim.setseed,
            "int",
            "SEED value",
            "Set seed for all functions using a randomizer (e.g.mcre,noise)",
        ],
        "SELECTIMPL": [
            minisky.core.trafficarrays.select_implementation,
            "[txt,txt]",
            "SELECTIMPL [classname, implname]",
            "Select implementation for a replaceable class (e.g., SELECTIMPL AUTOPILOT MYAUTOPILOT)",
        ],
        "SPD": [
            minisky.traf.ap.selspdcmd,
            "callsign,spd",
            "SPD callsign,casmach (= CASkts/Mach)",
            "Select autopilot speed.",
        ],
        "SWTOC": [
            minisky.traf.ap.setswtoc,
            "callsign,[bool]",
            "SWTOC callsign,[ON/OFF]",
            "Switch ToC logic (=climb early) on/off.",
        ],
        "SWTOD": [
            minisky.traf.ap.setswtod,
            "callsign,[bool]",
            "SWTOD callsign,[ON/OFF]",
            "Switch ToD logic (=climb early) on/off.",
        ],
        "THR": [
            minisky.traf.setthrottle,
            "callsign[,txt]",
            "THR callsign, IDLE/0.0/throttlesetting/1.0/AUTO(default)",
            "Set throttle or autotothrottle(default)",
        ],
        "TIME": [
            minisky.sim.setutc,
            "[txt]",
            "TIME RUN(default) / HH:MM:SS.hh / REAL / UTC ",
            "Set simulated clock time",
        ],
        "TRAIL": [
            minisky.traf.trails.setTrails,
            "[callsign/bool],[float/txt]",
            "TRAIL ON/OFF, [dt] OR TRAIL callsign colour",
            "Toggle aircraft trails on/off",
        ],
        "UNGROUP": [
            minisky.traf.groups.ungroup,
            "txt,callsign,...",
            "UNGROUP grname, callsign",
            "Remove aircraft from a group",
        ],
        "VNAV": [
            minisky.traf.ap.setVNAV,
            "callsign,[bool]",
            "VNAV callsign,[ON/OFF]",
            "Switch on/off VNAV mode, the vertical FMS mode (autopilot).",
        ],
        "VS": [
            minisky.traf.ap.selvspdcmd,
            "callsign,vspd",
            "VS callsign,vspd (ft/min)",
            "Vertical speed command (autopilot).",
        ],
        "WIND": [
            minisky.traf.wind.add,
            "latlon,[float/txt,float,float]...",
            "WIND lat,lon,[alt],dir,spd[,alt,dir,spd,...] or WIND lat,lon,DEL",
            "Define a wind vector as part of the 2D or 3D wind field.",
        ],
        "ZONEDH": [
            minisky.traf.cd.sethpz,
            "[float,callsign,...]",
            "ZONEDH [height, callsign...]",
            "Set the vertical separation distance (i.e., half of the protected zone height) in feet.",
        ],
        "ZONER": [
            minisky.traf.cd.setrpz,
            "[float,callsign,...]",
            "ZONER [radius, callsign...]",
            "Set the horizontal separation distance (i.e., the radius of the protected zone) in nautical miles.",
        ],
    }

    # Command synonym dictionary
    synonyms = {
        "ASAS": ["CD", "CDMETHOD"],
        "POS": ["AWY", "AIRPORT", "RUNWAYS", "AIRWAY", "AIRWAYS"],
        "BANK": ["BANKLIM"],
        "OP": ["CONTINUE", "RUN", "START"],
        "CRE": ["CREATE"],
        "QUIT": ["CLOSE", "END", "EXIT", "STOP"],
        "DEL": ["DELETE"],
        "SELECTIMPL": ["IMPL", "IMPLEMENTATION", "IMPLEMENT"],
        "POLYLINE": ["LINES", "POLYLINES"],
        "MAGVAR": ["MAGDEC", "MAGDECL", "VAR"],
        "HOLD": ["PAUSE"],
        "POLY": ["POLYGON"],
        "ECHO": ["PRINT"],
        "REALTIME": ["RT"],
        "TRAIL": ["TRAILS"],
        "PERFSTATS": ["PERFINFO", "PERFDATA"],
        "PLUGINS": ["PLUGIN"],
    }

    return cmddict, synonyms
