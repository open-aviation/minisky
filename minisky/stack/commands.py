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


def get_commands():
    import minisky

    cmddict = {
        "ALT": [
            minisky.traf.ap.selaltcmd,
            "acid,alt,[vspd]",
            "ALT acid, alt, [vspd]",
            "Select autopilot altitude command.",
        ],
        "ASAS": [
            minisky.traf.cd.switch,
            "[txt]",
            "ASAS [ON/OFF]",
            "Select a Conflict Detection method.",
        ],
        "ATALT": [
            minisky.traf.cond.ataltcmd,
            "acid,alt,string",
            "acid ATALT alt cmd ",
            "When aircraft at given altitude , execute the command",
        ],
        "ATDIST": [
            minisky.traf.cond.atdistcmd,
            "acid,latlon,float,string",
            "acid ATDIST pos dist cmd ",
            "When aircraft passing this distance (in nm) to position, execute the command",
        ],
        "ATSPD": [
            minisky.traf.cond.atspdcmd,
            "acid,spd,string",
            "acid ATSPD spd cmd ",
            "When aircraft reaches given speed, execute the command",
        ],
        "BANK": [
            minisky.traf.setbanklim,
            "acid,[float]",
            "BANK acid bankangle[deg]",
            "Set or show bank limit for this vehicle",
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
            "CRE acid,type,lat,lon,hdg,alt,spd",
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
            "txt,txt,acid,hdg,float,time,[alt,time,spd]",
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
            "acid/txt,...",
            "DEL acid/ALL/WIND/shape",
            "Delete command (aircraft, wind, area)",
        ],
        "DEST": [
            minisky.traf.ap.setdest,
            "acid,wpt",
            "DEST acid, latlon/airport",
            "Set destination of aircraft, aircraft will fly to this airport.",
        ],
        "DTLOOK": [
            minisky.traf.cd.setdtlook,
            "[time, acid...]",
            "DTLOOK [time, acid...]",
            "Set the lookahead time (in [hh:mm:]sec) for conflict detection.",
        ],
        "DTNOLOOK": [
            minisky.traf.cd.setdtnolook,
            "[time, acid...]",
            "DTNOLOOK [time, acid...]",
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
            "[txt,acid/txt,...]",
            "GROUP [grname, (areaname OR acid,...) ]",
            "Add aircraft to a group. OR all aircraft in given area.\n"
            + "Returns list of groups when no argument is passed.\n"
            + "Returns list of aircraft in group when only a groupname is passed.\n"
            + "A group is created when a group with the given name doesn't exist yet.",
        ],
        "HDG": [
            minisky.traf.ap.selhdgcmd,
            "acid,hdg",
            "HDG acid,hdg (deg,True or Magnetic)",
            "Autopilot select heading command.",
        ],
        "HOLD": [
            minisky.sim.hold,
            "",
            "HOLD",
            "Pause(hold) simulation",
        ],
        "LINE": [
            minisky.tools.areafilter.define_line_area,
            "txt,latlon,latlon",
            "LINE name,lat,lon,lat,lon",
            "Draw a line on the radar screen",
        ],
        "LNAV": [
            minisky.traf.ap.setLNAV,
            "acid,[bool]",
            "LNAV acid,[ON/OFF]",
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
            "acid,latlon,[alt,hdg,spd,vspd]",
            "MOVE acid,lat,lon,[alt,hdg,spd,vspd]",
            "Move an aircraft to a new position",
        ],
        "NOISE": [
            minisky.traf.setnoise,
            "[onoff]",
            "NOISE [ON/OFF]",
            "Turbulence/noise switch",
        ],
        "NORESO": [
            minisky.traf.cr.setnoreso,
            "acid...",
            "NORESO acid...",
            "ADD or Remove aircraft that nobody will avoid.",
        ],
        "OP": [
            minisky.sim.op,
            "",
            "OP",
            "Start/Run simulation or continue after hold",
        ],
        "ORIG": [
            minisky.traf.ap.setorig,
            "acid,wpt",
            "ORIG acid, latlon/airport",
            "Set origin of aircraft.",
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
            minisky.traf.poscommand,
            "acid/wpt",
            "POS acid/waypoint",
            "Get info on aircraft, airport or waypoint",
        ],
        "PRIORULES": [
            minisky.traf.cr.setprio,
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
            minisky.traf.cr.setresooff,
            "acid...",
            "RESOOFF acid...",
            "ADD or Remove aircraft that will not avoid anybody else.",
        ],
        "RFACH": [
            minisky.traf.cr.setresofach,
            "[float]",
            "RFACH [factor]",
            "Set resolution factor horizontal.",
        ],
        "RFACV": [
            minisky.traf.cr.setresofacv,
            "[float]",
            "RFACV [factor]",
            "Set resolution factor vertical.",
        ],
        "RSZONEDH": [
            minisky.traf.cr.setresozonedh,
            "[float]",
            "RSZONEDH [zonedh]",
            "Set resolution factor vertical, but then with absolute value.",
        ],
        "RSZONER": [
            minisky.traf.cr.setresozoner,
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
        "SEED": [
            minisky.sim.setseed,
            "int",
            "SEED value",
            "Set seed for all functions using a randomizer (e.g.mcre,noise)",
        ],
        "SPD": [
            minisky.traf.ap.selspdcmd,
            "acid,spd",
            "SPD acid,casmach (= CASkts/Mach)",
            "Select autopilot speed.",
        ],
        "SWTOC": [
            minisky.traf.ap.setswtoc,
            "acid,[bool]",
            "SWTOC acid,[ON/OFF]",
            "Switch ToC logic (=climb early) on/off.",
        ],
        "SWTOD": [
            minisky.traf.ap.setswtod,
            "acid,[bool]",
            "SWTOD acid,[ON/OFF]",
            "Switch ToD logic (=climb early) on/off.",
        ],
        "THR": [
            minisky.traf.setthrottle,
            "acid[,txt]",
            "THR acid, IDLE/0.0/throttlesetting/1.0/AUTO(default)",
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
            "[acid/bool],[float/txt]",
            "TRAIL ON/OFF, [dt] OR TRAIL acid colour",
            "Toggle aircraft trails on/off",
        ],
        "UNGROUP": [
            minisky.traf.groups.ungroup,
            "txt,acid,...",
            "UNGROUP grname, acid",
            "Remove aircraft from a group",
        ],
        "VNAV": [
            minisky.traf.ap.setVNAV,
            "acid,[bool]",
            "VNAV acid,[ON/OFF]",
            "Switch on/off VNAV mode, the vertical FMS mode (autopilot).",
        ],
        "VS": [
            minisky.traf.ap.selvspdcmd,
            "acid,vspd",
            "VS acid,vspd (ft/min)",
            "Vertical speed command (autopilot).",
        ],
        "WIND": [
            minisky.traf.wind.add,
            "latlon,[alt,float,float]...",
            "WIND lat,lon,alt,dir,spd,[alt,dir,spd,alt,...]",
            "Define a wind vector as part of the 2D or 3D wind field.",
        ],
        "ZONEDH": [
            minisky.traf.cd.sethpz,
            "[float, acid...]",
            "ZONEDH [height, acid...]",
            "Set the vertical separation distance (i.e., half of the protected zone height) in feet.",
        ],
        "ZONER": [
            minisky.traf.cd.setrpz,
            "[float, acid...]",
            "ZONER [radius, acid...]",
            "Set the horizontal separation distance (i.e., the radius of the protected zone) in nautical miles.",
        ],
        "IC": [
            minisky.stack.ic,
            "string",
            "IC scenario_filename",
            "Load a scenario filename.",
        ],
        "SCENARIO": [
            minisky.stack.scenario,
            "name",
            "SCENARIO name",
            "Sets the scenario name for the current simulation.",
        ],
        "DELAY": [
            minisky.stack.delay,
            "time, string",
            "DELAY time, cmdline",
            "Delay a stack command until a specific simulation time.",
        ],
        "HELP": [
            minisky.stack.showhelp,
            "[cmd, subcmd]",
            "HELP [cmd, subcmd]",
            "Display general help text or help text for a specific command.",
        ],
        # "RMETHH": [
        #     minisky.traffic.asas.mvp.setresmethh,
        #     "[txt]",
        #     "RMETHH [method]",
        #     "Processes the RMETHH command. Sets swresovert = False",
        # ],
        # "RMETHV": [
        #     minisky.traffic.asas.mvp.setresmethv,
        #     "[txt]",
        #     "RMETHV [method]",
        #     "Processes the RMETHV command. Sets swresohoriz = False",
        # ],
        "ADDWPTMODE": [
            minisky.traffic.route.Route.addwptMode,
            "acid, [wpt,alt]",
            "ADDWPTMODE acid, [wpt,alt]",
            "Changes the mode of the ADDWPT command to add waypoints of type 'mode'.",
        ],
        "ADDWPT": [
            minisky.traffic.route.Route.addwptStack,
            "acid,wpt,[alt,spd,wpinroute,wpinroute]",
            "ADDWPT acid, wpt, [alt, spd, wpinroute, wpinroute]",
            "Add a waypoint to the route.",
        ],
        "BEFORE": [
            minisky.traffic.route.Route.before,
            "acid, wpinroute, addwpt, waypoint, [alt, spd]",
            "BEFORE acid, wpinroute, addwpt, waypoint, [alt, spd]",
            "Add a waypoint before another waypoint in the route.",
        ],
        "AFTER": [
            minisky.traffic.route.Route.after,
            "acid, wpinroute, addwpt, waypoint, [alt, spd]",
            "AFTER acid, wpinroute, addwpt, waypoint, [alt, spd]",
            "Add a waypoint after another waypoint in the route.",
        ],
        "AT": [
            minisky.traffic.route.Route.at,
            "acid, wpinroute, [DEL] ALT/SPD/DO alt/spd/stack command",
            "AT acid, wpinroute, [DEL] ALT/SPD/DO alt/spd/stack command",
            "Set or show altitude and/or speed constraints at a waypoint.",
        ],
        "DIRECT": [
            minisky.traffic.route.Route.direct,
            "acid, wpinroute",
            "DIRECT acid, wpinroute",
            "Go direct to a specified waypoint in the route.",
        ],
        "RTA": [
            minisky.traffic.route.Route.SetRTA,
            "acid, wpinroute, time",
            "RTA acid, wpinroute, time",
            "Add RTA to waypoint record.",
        ],
        "LISTRTE": [
            minisky.traffic.route.Route.listrte,
            "acid, [pagenr]",
            "LISTRTE acid, [pagenr]",
            "Show list of route in window per page of 5 waypoints.",
        ],
        "DELRTE": [
            minisky.traffic.route.Route.delrte,
            "acid",
            "DELRTE acid",
            "Delete the complete route for an aircraft.",
        ],
        "DELWP": [
            minisky.traffic.route.Route.delwpt,
            "acid, wpinroute",
            "DELWP acid, wpinroute",
            "Delete a waypoint from a route.",
        ],
        "DUMPRTE": [
            minisky.traffic.route.Route.dumprte,
            "acid",
            "DUMPRTE acid",
            "Write route to output/routelog.txt.",
        ],
    }

    # Command synonym dictionary
    synonyms = {
        "ADDAWY": ["ADDAIRWAY"],
        "ASAS": ["CD", "CDMETHOD"],
        "POS": ["AWY", "AIRPORT", "RUNWAYS"],
        "AIRWAY": ["AIRWAYS"],
        "BANK": ["BANKLIM"],
        "COLOUR": ["COL", "COLOR"],
        "OP": ["CONTINUE", "RUN", "START"],
        "CRE": ["CREATE"],
        "QUIT": ["CLOSE", "END", "EXIT", "STOP"],
        "CALC": ["DEBUG"],
        "DEL": ["DELETE"],
        "SWRAD": ["DISP"],
        "IMPLEMENTATION": ["IMPL", "IMPLEMENT"],
        "POLYLINE": ["LINES", "POLYLINES"],
        "MAGVAR": ["MAGDEC", "MAGDECL", "VAR"],
        "HOLD": ["PAUSE"],
        "POLY": ["POLYGON"],
        "ECHO": ["PRINT"],
        "REALTIME": ["RT"],
        "DTMULT": ["RTF"],
        "TRAIL": ["TRAILS"],
    }

    return cmddict, synonyms
