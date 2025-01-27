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
    import clearsky as cs

    cmddict = {
        "ALT": [
            cs.traf.ap.selaltcmd,
            "acid,alt,[vspd]",
            "ALT acid, alt, [vspd]",
            "Select autopilot altitude command.",
        ],
        "ATALT": [
            cs.traf.cond.ataltcmd,
            "acid,alt,string",
            "acid ATALT alt cmd ",
            "When aircraft at given altitude , execute the command",
        ],
        "ATDIST": [
            cs.traf.cond.atdistcmd,
            "acid,latlon,float,string",
            "acid ATDIST pos dist cmd ",
            "When aircraft passing this distance (in nm) to position, execute the command",
        ],
        "ATSPD": [
            cs.traf.cond.atspdcmd,
            "acid,spd,string",
            "acid ATSPD spd cmd ",
            "When aircraft reaches given speed, execute the command",
        ],
        "BANK": [
            cs.traf.setbanklim,
            "acid,[float]",
            "BANK acid bankangle[deg]",
            "Set or show bank limit for this vehicle",
        ],
        "BENCHMARK": [
            cs.sim.benchmark,
            "[string,time]",
            "BENCHMARK [scenfile,time]",
            "Run benchmark",
        ],
        "BOX": [
            cs.tools.areafilter.define_box_area,
            "txt,latlon,latlon,[alt,alt]",
            "BOX name,lat,lon,lat,lon,[top,bottom]",
            "Define a box-shaped area",
        ],
        "CASMACHTHR": [
            cs.tools.aero.casmachthr,
            "float",
            "CASMACHTHR threshold",
            """Set a threshold below which speeds should be considered as Mach numbers
                in CRE(ATE), ADDWPT, and SPD commands. Set to zero if speeds should
                never be considered as Mach number(e.g., when simulating drones).""",
        ],
        "CIRCLE": [
            cs.tools.areafilter.define_circle_area,
            "txt,latlon,float,[alt,alt]",
            "CIRCLE name,lat,lon,radius,[top,bottom]",
            "Define a circle-shaped area",
        ],
        "CLRCRECMD": [
            cs.traf.clrcrecmd,
            "",
            "CLRCRECMD",
            "CLRCRECMD will clear CRECMD list of commands aircraft creation",
        ],
        "CRE": [
            cs.traf.cre,
            "txt,txt,float,float,[hdg,alt,spd]",
            "CRE acid,type,lat,lon,hdg,alt,spd",
            "Create an aircraft",
        ],
        "CRECMD": [
            cs.traf.crecmd,
            "string",
            "CRECMD cmdline (to be added after aircraft id )",
            "Add a command for each aircraft to be issued after creation of aircraft",
        ],
        "CRECONFS": [
            cs.traf.creconfs,
            "txt,txt,acid,hdg,float,time,[alt,time,spd]",
            "CRECONFS id, type, targetid, dpsi, cpa, tlos_hor, dH, tlos_ver, spd",
            "Create an aircraft that is in conflict with 'targetid'",
        ],
        "DATE": [
            cs.sim.setutc,
            "[int,int,int,txt]",
            "DATE [day,month,year,HH:MM:SS.hh]",
            "Set simulation date",
        ],
        "DEFWPT": [
            cs.navdb.defwpt,
            "txt,latlon,[txt]",
            "DEFWPT wpname,lat,lon,[DELETE/FIX/VOR/DME/NDB/DEL]",
            "Define (or delete) a waypoint only for this scenario/run",
        ],
        "DEL": [
            cs.stack.delete_element,
            "acid/txt,...",
            "DEL acid/ALL/WIND/shape",
            "Delete command (aircraft, wind, area)",
        ],
        "DEST": [
            cs.traf.ap.setdest,
            "acid,wpt",
            "DEST acid, latlon/airport",
            "Set destination of aircraft, aircraft will fly to this airport.",
        ],
        "DT": [
            cs.core.simtime.setdt_ui,
            "[float/txt,float]",
            "DT [dt] OR [target,dt]",
            "Set simulation time step",
        ],
        "DTLOOK": [
            cs.traf.cd.setdtlook,
            "[time, acid...]",
            "DTLOOK [time, acid...]",
            "Set the lookahead time (in [hh:mm:]sec) for conflict detection.",
        ],
        "DTMULT": [
            cs.sim.set_dtmult,
            "float",
            "DTMULT multiplier",
            "Sel multiplication factor for fast-time simulation",
        ],
        "DTNOLOOK": [
            cs.traf.cd.setdtnolook,
            "[time, acid...]",
            "DTNOLOOK [time, acid...]",
            "Set the interval (in [hh:mm:]sec) in which conflict detection is skipped after a conflict resolution.",
        ],
        "ECHO": [
            cs.scr.echo,
            "string",
            "ECHO txt",
            "Show a text in command window for user to read",
        ],
        "FF": [
            cs.sim.fastforward,
            "[time]",
            "FF [timeinsec]",
            "Fast forward the simulation",
        ],
        "GETWIND": [
            cs.traf.wind.get,
            "lat, lon, [alt]",
            "GETWIND lat, lon, [alt]",
            "Get wind at a specified position (and optionally at altitude).",
        ],
        "GROUP": [
            cs.traf.groups.group,
            "[txt,acid/txt,...]",
            "GROUP [grname, (areaname OR acid,...) ]",
            "Add aircraft to a group. OR all aircraft in given area.\n"
            + "Returns list of groups when no argument is passed.\n"
            + "Returns list of aircraft in group when only a groupname is passed.\n"
            + "A group is created when a group with the given name doesn't exist yet.",
        ],
        "HDG": [
            cs.traf.ap.selhdgcmd,
            "acid,hdg",
            "HDG acid,hdg (deg,True or Magnetic)",
            "Autopilot select heading command.",
        ],
        "HOLD": [
            cs.sim.hold,
            "",
            "HOLD",
            "Pause(hold) simulation",
        ],
        "LINE": [
            cs.tools.areafilter.define_line_area,
            "txt,latlon,latlon",
            "LINE name,lat,lon,lat,lon",
            "Draw a line on the radar screen",
        ],
        "LNAV": [
            cs.traf.ap.setLNAV,
            "acid,[bool]",
            "LNAV acid,[ON/OFF]",
            "LNAV (lateral FMS mode) switch for autopilot.",
        ],
        "LSVAR": [
            cs.core.varexplorer.lsvar,
            "[word]",
            "LSVAR path.to.variable",
            "Inspect any variable in a simulation",
        ],
        "MAGVAR": [
            cs.tools.geo.magdeccmd,
            "lat,lon",
            "MAGVAR lat,lon",
            "Show magnetic variation/declination at position",
        ],
        "MCRE": [
            cs.traf.mcre,
            "int,[float,float,float,float,txt,alt,spd]",
            "MCRE n,[lat,lon,lat,lon,type,alt,spd]",
            "Multiple random create of n aircraft in current view",
        ],
        "MOVE": [
            cs.traf.move,
            "acid,latlon,[alt,hdg,spd,vspd]",
            "MOVE acid,lat,lon,[alt,hdg,spd,vspd]",
            "Move an aircraft to a new position",
        ],
        "NOISE": [
            cs.traf.setnoise,
            "[onoff]",
            "NOISE [ON/OFF]",
            "Turbulence/noise switch",
        ],
        "NORESO": [
            cs.traf.cr.setnoreso,
            "acid...",
            "NORESO acid...",
            "ADD or Remove aircraft that nobody will avoid.",
        ],
        "OP": [
            cs.sim.op,
            "",
            "OP",
            "Start/Run simulation or continue after hold",
        ],
        "ORIG": [
            cs.traf.ap.setorig,
            "acid,wpt",
            "ORIG acid, latlon/airport",
            "Set origin of aircraft.",
        ],
        "POLY": [
            cs.tools.areafilter.define_poly_area,
            "txt,[latlon,...]",
            "POLY name,[lat,lon,lat,lon, ...]",
            "Define a polygon-shaped area",
        ],
        "POLYALT": [
            cs.tools.areafilter.define_polyalt_area,
            "txt,alt,alt,latlon,...",
            "POLYALT name,top,bottom,lat,lon,lat,lon, ...",
            "Define a polygon-shaped area in 3D: between two altitudes",
        ],
        "POLYLINE": [
            cs.tools.areafilter.define_polyline_area,
            "txt,latlon,...",
            "POLYLINE name,lat,lon,lat,lon,...",
            "Draw a multi-segment line on the radar screen",
        ],
        "POS": [
            cs.traf.poscommand,
            "acid/wpt",
            "POS acid/waypoint",
            "Get info on aircraft, airport or waypoint",
        ],
        "PRIORULES": [
            cs.traf.cr.setprio,
            "[bool, txt]",
            "PRIORULES [flag, priocode]",
            "Define priority rules (right of way) for conflict resolution.",
        ],
        "QUIT": [
            cs.sim.stop,
            "",
            "QUIT",
            "Quit program/Stop simulation",
        ],
        "REALTIME": [
            cs.sim.realtime,
            "[bool]",
            "REALTIME [ON/OFF]",
            "En-/disable realtime running allowing a variable timestep.",
        ],
        "RESET": [
            cs.sim.reset,
            "",
            "RESET",
            "Reset simulation",
        ],
        "RESO": [
            cs.traf.cr.setmethod,
            "[txt]",
            "RESO [name]",
            "Select a Conflict Resolution method.",
        ],
        "RESOOFF": [
            cs.traf.cr.setresooff,
            "acid...",
            "RESOOFF acid...",
            "ADD or Remove aircraft that will not avoid anybody else.",
        ],
        "RFACH": [
            cs.traf.cr.setresofach,
            "[float]",
            "RFACH [factor]",
            "Set resolution factor horizontal.",
        ],
        "RFACV": [
            cs.traf.cr.setresofacv,
            "[float]",
            "RFACV [factor]",
            "Set resolution factor vertical.",
        ],
        "RSZONEDH": [
            cs.traf.cr.setresozonedh,
            "[float]",
            "RSZONEDH [zonedh]",
            "Set resolution factor vertical, but then with absolute value.",
        ],
        "RSZONER": [
            cs.traf.cr.setresozoner,
            "[float]",
            "RSZONER [zoner]",
            "Set resolution factor horizontal, but then with absolute value.",
        ],
        "SCHEDULE": [
            cs.stack.schedule,
            "time,string",
            "SCHEDULE a stack command at a specific simulation time.",
            "Schedule a stack command at a specific simulation time.",
        ],
        "SEED": [
            cs.sim.setseed,
            "int",
            "SEED value",
            "Set seed for all functions using a randomizer (e.g.mcre,noise)",
        ],
        "SPD": [
            cs.traf.ap.selspdcmd,
            "acid,spd",
            "SPD acid,casmach (= CASkts/Mach)",
            "Select autopilot speed.",
        ],
        "SWTOC": [
            cs.traf.ap.setswtoc,
            "acid,[bool]",
            "SWTOC acid,[ON/OFF]",
            "Switch ToC logic (=climb early) on/off.",
        ],
        "SWTOD": [
            cs.traf.ap.setswtod,
            "acid,[bool]",
            "SWTOD acid,[ON/OFF]",
            "Switch ToD logic (=climb early) on/off.",
        ],
        "THR": [
            cs.traf.setthrottle,
            "acid[,txt]",
            "THR acid, IDLE/0.0/throttlesetting/1.0/AUTO(default)",
            "Set throttle or autotothrottle(default)",
        ],
        "TIME": [
            cs.sim.setutc,
            "[txt]",
            "TIME RUN(default) / HH:MM:SS.hh / REAL / UTC ",
            "Set simulated clock time",
        ],
        "TRAIL": [
            cs.traf.trails.setTrails,
            "[acid/bool],[float/txt]",
            "TRAIL ON/OFF, [dt] OR TRAIL acid colour",
            "Toggle aircraft trails on/off",
        ],
        "UNGROUP": [
            cs.traf.groups.ungroup,
            "txt,acid,...",
            "UNGROUP grname, acid",
            "Remove aircraft from a group",
        ],
        "VNAV": [
            cs.traf.ap.setVNAV,
            "acid,[bool]",
            "VNAV acid,[ON/OFF]",
            "Switch on/off VNAV mode, the vertical FMS mode (autopilot).",
        ],
        "VS": [
            cs.traf.ap.selvspdcmd,
            "acid,vspd",
            "VS acid,vspd (ft/min)",
            "Vertical speed command (autopilot).",
        ],
        "WIND": [
            cs.traf.wind.add,
            "lat, lon, float...",
            "WIND lat, lon, winddata...",
            "Define a wind vector as part of the 2D or 3D wind field.",
        ],
        "ZONEDH": [
            cs.traf.cd.sethpz,
            "[float, acid...]",
            "ZONEDH [height, acid...]",
            "Set the vertical separation distance (i.e., half of the protected zone height) in feet.",
        ],
        "ZONER": [
            cs.traf.cd.setrpz,
            "[float, acid...]",
            "ZONER [radius, acid...]",
            "Set the horizontal separation distance (i.e., the radius of the protected zone) in nautical miles.",
        ],
        "IC": [
            cs.stack.ic,
            "string",
            "IC scenario_filename",
            "Load a scenario filename.",
        ],
        "SCENARIO": [
            cs.stack.scenario,
            "name",
            "SCENARIO name",
            "Sets the scenario name for the current simulation.",
        ],
        "DELAY": [
            cs.stack.delay,
            "time, string",
            "DELAY time, cmdline",
            "Delay a stack command until a specific simulation time.",
        ],
        "HELP": [
            cs.stack.showhelp,
            "[cmd, subcmd]",
            "HELP [cmd, subcmd]",
            "Display general help text or help text for a specific command.",
        ],
        # "RMETHH": [
        #     cs.traffic.asas.mvp.setresmethh,
        #     "[txt]",
        #     "RMETHH [method]",
        #     "Processes the RMETHH command. Sets swresovert = False",
        # ],
        # "RMETHV": [
        #     cs.traffic.asas.mvp.setresmethv,
        #     "[txt]",
        #     "RMETHV [method]",
        #     "Processes the RMETHV command. Sets swresohoriz = False",
        # ],
        "ADDWPTMODE": [
            cs.traffic.route.Route.addwptMode,
            "acid, [wpt,alt]",
            "ADDWPTMODE acid, [wpt,alt]",
            "Changes the mode of the ADDWPT command to add waypoints of type 'mode'.",
        ],
        "ADDWPT": [
            cs.traffic.route.Route.addwptStack,
            "acid,wpt,[alt,spd,wpinroute,wpinroute]",
            "ADDWPT acid, wpt, [alt, spd, wpinroute, wpinroute]",
            "Add a waypoint to the route.",
        ],
        "BEFORE": [
            cs.traffic.route.Route.before,
            "acid, wpinroute, addwpt, waypoint, [alt, spd]",
            "BEFORE acid, wpinroute, addwpt, waypoint, [alt, spd]",
            "Add a waypoint before another waypoint in the route.",
        ],
        "AFTER": [
            cs.traffic.route.Route.after,
            "acid, wpinroute, addwpt, waypoint, [alt, spd]",
            "AFTER acid, wpinroute, addwpt, waypoint, [alt, spd]",
            "Add a waypoint after another waypoint in the route.",
        ],
        "AT": [
            cs.traffic.route.Route.at,
            "acid, wpinroute, [DEL] ALT/SPD/DO alt/spd/stack command",
            "AT acid, wpinroute, [DEL] ALT/SPD/DO alt/spd/stack command",
            "Set or show altitude and/or speed constraints at a waypoint.",
        ],
        "DIRECT": [
            cs.traffic.route.Route.direct,
            "acid, wpinroute",
            "DIRECT acid, wpinroute",
            "Go direct to a specified waypoint in the route.",
        ],
        "RTA": [
            cs.traffic.route.Route.SetRTA,
            "acid, wpinroute, time",
            "RTA acid, wpinroute, time",
            "Add RTA to waypoint record.",
        ],
        "LISTRTE": [
            cs.traffic.route.Route.listrte,
            "acid, [pagenr]",
            "LISTRTE acid, [pagenr]",
            "Show list of route in window per page of 5 waypoints.",
        ],
        "DELRTE": [
            cs.traffic.route.Route.delrte,
            "acid",
            "DELRTE acid",
            "Delete the complete route for an aircraft.",
        ],
        "DELWP": [
            cs.traffic.route.Route.delwpt,
            "acid, wpinroute",
            "DELWP acid, wpinroute",
            "Delete a waypoint from a route.",
        ],
        "DUMPRTE": [
            cs.traffic.route.Route.dumprte,
            "acid",
            "DUMPRTE acid",
            "Write route to output/routelog.txt.",
        ],
    }

    # Command synonym dictionary
    synonyms = {
        "ADDAWY": ["ADDAIRWAY"],
        "POS": ["AWY", "AIRPORT", "RUNWAYS"],
        "AIRWAY": ["AIRWAYS"],
        "BANK": ["BANKLIM"],
        "CD": ["CHDIR"],
        "COLOUR": ["COL", "COLOR"],
        "OP": ["CONTINUE", "RUN", "START"],
        "CRE": ["CREATE"],
        "QUIT": ["CLOSE", "END", "EXIT", "STOP"],
        "CALC": ["DEBUG"],
        "DEL": ["DELETE"],
        "SWRAD": ["DISP"],
        "FF": ["FWD"],
        "IMPLEMENTATION": ["IMPL", "IMPLEMENT"],
        "POLYLINE": ["LINES", "POLYLINES"],
        "MAGVAR": ["MAGDEC", "MAGDECL", "VAR"],
        "HOLD": ["PAUSE"],
        "POLY": ["POLYGON"],
        "ECHO": ["PRINT"],
        "REALTIME": ["RT"],
        "DTMULT": ["RTF"],
        "SAVEIC": ["SAVE"],
        "TRAIL": ["TRAILS"],
    }

    return cmddict, synonyms
