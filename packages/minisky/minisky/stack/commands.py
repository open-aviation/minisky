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
registered with the command interpreter in `CommandStack.init()`.

The strings in the command dictionary are the in-simulator help texts
shown by the HELP command.
"""

from __future__ import annotations

from collections.abc import Callable
from functools import partial
from typing import TYPE_CHECKING, Any, NamedTuple, TypeAlias

if TYPE_CHECKING:
    from minisky.stack import CommandStack


class CommandDefinition(NamedTuple):
    callback: Callable[..., Any]
    arguments: str
    brief: str
    help: str


CommandDefinitions: TypeAlias = dict[str, CommandDefinition]
CommandAliases: TypeAlias = dict[str, tuple[str, ...]]


class CommandCatalog(NamedTuple):
    definitions: CommandDefinitions
    aliases: CommandAliases


def get_commands(command_stack: CommandStack) -> CommandCatalog:
    """Assemble the base command and synonym dictionaries of the simulator.

    Binds callbacks to the objects owned by the provided runtime command stack.
    """
    from minisky import tools
    from minisky.traffic import route

    # TODO(abraham): migrate core commands from this legacy table to typed declarations.
    cmddict: dict[str, list[Any]] = {
        "ADDWPT": [
            partial(route.addwpt, command_stack.traffic),
            "callsign,wpt,[alt,spd,wpt,wpt]",
            "ADDWPT callsign, wpt, [alt, spd, wpt, wpt]",
            "Add a waypoint to the route.",
        ],
        "ADDWPTMODE": [
            partial(route.change_wpt_mode, command_stack.traffic),
            "callsign, [wpt,alt]",
            "ADDWPTMODE callsign, [wpt,alt]",
            "Changes the mode of the ADDWPT command to add waypoints of type 'mode'.",
        ],
        "AFTER": [
            partial(route.addwpt_after, command_stack.traffic),
            "callsign,wpt,txt,wpt,[alt,spd]",
            "AFTER callsign, wpt, addwpt, waypoint, [alt, spd]",
            "Add a waypoint after another waypoint in the route.",
        ],
        "ASAS": [
            command_stack.traffic.cd.switch,
            "[txt]",
            "ASAS [ON/OFF]",
            "Select a Conflict Detection method.",
        ],
        "AT": [
            partial(route.at_wpt, command_stack.traffic),
            "callsign,wpt,[txt,...]",
            "AT callsign, wpt, [DEL] ALT/SPD/DO alt/spd/stack command",
            "Set or show altitude and/or speed constraints at a waypoint.",
        ],
        "BANK": [
            command_stack.traffic.setbanklim,
            "callsign,[float]",
            "BANK callsign bankangle[deg]",
            "Set or show bank limit for this vehicle",
        ],
        "BEFORE": [
            partial(route.addwpt_before, command_stack.traffic),
            "callsign,wpt,txt,wpt,[alt,spd]",
            "BEFORE callsign, wpt, addwpt, waypoint, [alt, spd]",
            "Add a waypoint before another waypoint in the route.",
        ],
        "BOX": [
            command_stack.areas.define_box_area,
            "txt,latlon,latlon,[alt,alt]",
            "BOX name,lat,lon,lat,lon,[top,bottom]",
            "Define a box-shaped area",
        ],
        "CASMACHTHR": [
            command_stack.traffic.casmachthr,
            "float",
            "CASMACHTHR threshold",
            """Set a threshold below which speeds should be considered as Mach numbers
                in CRE(ATE), ADDWPT, and SPD commands. Set to zero if speeds should
                never be considered as Mach number(e.g., when simulating drones).""",
        ],
        "CIRCLE": [
            command_stack.areas.define_circle_area,
            "txt,latlon,float,[alt,alt]",
            "CIRCLE name,lat,lon,radius,[top,bottom]",
            "Define a circle-shaped area",
        ],
        "CRECONFS": [
            command_stack.traffic.creconfs,
            "txt,txt,callsign,hdg,float,time,[alt,time,spd]",
            "CRECONFS id, type, targetid, dpsi, cpa, tlos_hor, dH, tlos_ver, spd",
            "Create an aircraft that is in conflict with 'targetid'",
        ],
        "DEL": [
            command_stack.delete_element,
            "callsign/txt,...",
            "DEL callsign/ALL/WIND/shape",
            "Delete command (aircraft, wind, area)",
        ],
        "DELRTE": [
            partial(route.delrte, command_stack.traffic),
            "callsign",
            "DELRTE callsign",
            "Delete the complete route for an aircraft.",
        ],
        "DELWPT": [
            partial(route.delwpt, command_stack.traffic),
            "callsign,wpt",
            "DELWPT callsign,wpt",
            "Delete a waypoint from a route.",
        ],
        "DEST": [
            command_stack.traffic.ap.setdest,
            "callsign,wpt,[spd]",
            "DEST callsign, latlon/airport, casmach (= CASkts/Mach)",
            "Set destination of aircraft, aircraft will fly to this airport.",
        ],
        "DIRECT": [
            partial(route.direct, command_stack.traffic),
            "callsign, wpt",
            "DIRECT callsign, wpt",
            "Go direct to a specified waypoint in the route.",
        ],
        "DTLOOK": [
            command_stack.traffic.cd.setdtlook,
            "[time,callsign,...]",
            "DTLOOK [time, callsign...]",
            "Set the lookahead time (in [hh:mm:]sec) for conflict detection.",
        ],
        "DTNOLOOK": [
            command_stack.traffic.cd.setdtnolook,
            "[time,callsign,...]",
            "DTNOLOOK [time, callsign...]",
            "Set the interval (in [hh:mm:]sec) in which conflict detection is skipped after a conflict resolution.",
        ],
        "GETWIND": [
            command_stack.traffic.wind.get,
            "lat, lon, [alt]",
            "GETWIND lat, lon, [alt]",
            "Get wind at a specified position (and optionally at altitude).",
        ],
        "GROUP": [
            command_stack.traffic.groups.group,
            "[txt,callsign/txt,...]",
            "GROUP [grname, (areaname OR callsign,...) ]",
            "Add aircraft to a group. OR all aircraft in given area.\n"
            + "Returns list of groups when no argument is passed.\n"
            + "Returns list of aircraft in group when only a groupname is passed.\n"
            + "A group is created when a group with the given name doesn't exist yet.",
        ],
        "HELP": [
            command_stack.showhelp,
            "[txt,txt]",
            "HELP [cmd, subcmd]",
            "Display general help text or help text for a specific command.",
        ],
        "HOLD": [
            command_stack.simulation.hold,
            "",
            "HOLD",
            "Pause(hold) simulation",
        ],
        "LINE": [
            command_stack.areas.define_line_area,
            "txt,latlon,latlon",
            "LINE name,lat,lon,lat,lon",
            "Draw a line on the radar screen",
        ],
        "LISTRTE": [
            partial(route.listrte, command_stack.traffic),
            "callsign,[txt]",
            "LISTRTE callsign, [pagenr]",
            "Show list of route in window per page of 5 waypoints.",
        ],
        "LNAV": [
            command_stack.traffic.ap.setLNAV,
            "callsign,[bool]",
            "LNAV callsign,[ON/OFF]",
            "LNAV (lateral FMS mode) switch for autopilot.",
        ],
        "MAGVAR": [
            tools.geo.magdeccmd,
            "lat,lon",
            "MAGVAR lat,lon",
            "Show magnetic variation/declination at position",
        ],
        "MCRE": [
            command_stack.traffic.mcre,
            "int,[float,float,float,float,txt,alt,spd]",
            "MCRE n,[lat,lon,lat,lon,type,alt,spd]",
            "Multiple random create of n aircraft in current view",
        ],
        "MOVE": [
            command_stack.traffic.move,
            "callsign,latlon,[alt,hdg,spd,vspd]",
            "MOVE callsign,lat,lon,[alt,hdg,spd,vspd]",
            "Move an aircraft to a new position",
        ],
        "NORESO": [
            command_stack.traffic.cr.setnoreso,
            "[callsign,...]",
            "NORESO callsign...",
            "ADD or Remove aircraft that nobody will avoid.",
        ],
        "OP": [
            command_stack.simulation.op,
            "",
            "OP",
            "Start/Run simulation or continue after hold",
        ],
        "PERFSTATS": [
            command_stack.traffic.perf.show_performance,
            "callsign",
            "PERFSTATS callsign",
            "Show the performace information of an aircraft.",
        ],
        "ORIG": [
            command_stack.traffic.ap.setorig,
            "callsign,wpt",
            "ORIG callsign, latlon/airport",
            "Set origin of aircraft.",
        ],
        "PLUGINS": [
            command_stack.plugins.manage,
            "[txt,txt]",
            "PLUGINS [LIST/LOAD, plugin_name]",
            "List available plugins or load a plugin",
        ],
        "POLY": [
            command_stack.areas.define_poly_area,
            "txt,[latlon,...]",
            "POLY name,[lat,lon,lat,lon, ...]",
            "Define a polygon-shaped area",
        ],
        "POLYALT": [
            command_stack.areas.define_polyalt_area,
            "txt,alt,alt,latlon,...",
            "POLYALT name,top,bottom,lat,lon,lat,lon, ...",
            "Define a polygon-shaped area in 3D: between two altitudes",
        ],
        "POLYLINE": [
            command_stack.areas.define_polyline_area,
            "txt,latlon,...",
            "POLYLINE name,lat,lon,lat,lon,...",
            "Draw a multi-segment line on the radar screen",
        ],
        "POS": [
            command_stack.traffic.position,
            "callsign/wpt",
            "POS callsign/waypoint",
            "Get info on aircraft, airport or waypoint",
        ],
        "PRIORULES": [
            command_stack.traffic.cr.setprio,
            "[bool, txt]",
            "PRIORULES [flag, priocode]",
            "Define priority rules (right of way) for conflict resolution.",
        ],
        "QUIT": [
            command_stack.simulation.stop,
            "",
            "QUIT",
            "Quit program/Stop simulation",
        ],
        "RESET": [
            command_stack.simulation.reset,
            "",
            "RESET",
            "Reset simulation",
        ],
        "RESO": [
            command_stack.traffic.cr.setmethod,
            "[txt]",
            "RESO [name]",
            "Select a Conflict Resolution method.",
        ],
        "RESOOFF": [
            command_stack.traffic.cr.setresooff,
            "[callsign,...]",
            "RESOOFF callsign...",
            "ADD or Remove aircraft that will not avoid anybody else.",
        ],
        "RMETHH": [
            command_stack.traffic.cr.setresometh,
            "[txt]",
            "RMETHH [ON / BOTH / OFF / NONE / SPD / HDG]",
            "Select the horizontal resolution method for MVP conflict resolution.",
        ],
        "RMETHV": [
            command_stack.traffic.cr.setresometv,
            "[txt]",
            "RMETHV [ON / V/S / OFF / NONE]",
            "Select the vertical resolution method for MVP conflict resolution.",
        ],
        "RFACH": [
            command_stack.traffic.cr.setresofach,
            "[float]",
            "RFACH [factor]",
            "Set resolution factor horizontal.",
        ],
        "RFACV": [
            command_stack.traffic.cr.setresofacv,
            "[float]",
            "RFACV [factor]",
            "Set resolution factor vertical.",
        ],
        "RTA": [
            partial(route.set_rta, command_stack.traffic),
            "callsign, wpt, time",
            "RTA callsign, wpt, time",
            "Add RTA to waypoint record.",
        ],
        "RSZONEDH": [
            command_stack.traffic.cr.setresozonedh,
            "[float]",
            "RSZONEDH [zonedh]",
            "Set resolution factor vertical, but then with absolute value.",
        ],
        "RSZONER": [
            command_stack.traffic.cr.setresozoner,
            "[float]",
            "RSZONER [zoner]",
            "Set resolution factor horizontal, but then with absolute value.",
        ],
        "SELECTIMPL": [
            command_stack.replaceables.select,
            "[txt,txt]",
            "SELECTIMPL [classname, implname]",
            "Select implementation for a replaceable class (e.g., SELECTIMPL AUTOPILOT MYAUTOPILOT)",
        ],
        "SWTOC": [
            command_stack.traffic.ap.setswtoc,
            "callsign,[bool]",
            "SWTOC callsign,[ON/OFF]",
            "Switch ToC logic (=climb early) on/off.",
        ],
        "SWTOD": [
            command_stack.traffic.ap.setswtod,
            "callsign,[bool]",
            "SWTOD callsign,[ON/OFF]",
            "Switch ToD logic (=climb early) on/off.",
        ],
        "THR": [
            command_stack.traffic.setthrottle,
            "callsign[,txt]",
            "THR callsign, IDLE/0.0/throttlesetting/1.0/AUTO(default)",
            "Set throttle or autotothrottle(default)",
        ],
        "TRAIL": [
            command_stack.traffic.trails.setTrails,
            "[callsign/bool],[float/txt]",
            "TRAIL ON/OFF, [dt] OR TRAIL callsign colour",
            "Toggle aircraft trails on/off",
        ],
        "UNGROUP": [
            command_stack.traffic.groups.ungroup,
            "txt,callsign,...",
            "UNGROUP grname, callsign",
            "Remove aircraft from a group",
        ],
        "VNAV": [
            command_stack.traffic.ap.setVNAV,
            "callsign,[bool]",
            "VNAV callsign,[ON/OFF]",
            "Switch on/off VNAV mode, the vertical FMS mode (autopilot).",
        ],
        "WIND": [
            command_stack.traffic.wind.add,
            "latlon,[float/txt,float,float]...",
            "WIND lat,lon,[alt],dir,spd[,alt,dir,spd,...] or WIND lat,lon,DEL",
            "Define a wind vector as part of the 2D or 3D wind field.",
        ],
        "ZONEDH": [
            command_stack.traffic.cd.sethpz,
            "[float,callsign,...]",
            "ZONEDH [height, callsign...]",
            "Set the vertical separation distance (i.e., half of the protected zone height) in feet.",
        ],
        "ZONER": [
            command_stack.traffic.cd.setrpz,
            "[float,callsign,...]",
            "ZONER [radius, callsign...]",
            "Set the horizontal separation distance (i.e., the radius of the protected zone) in nautical miles.",
        ],
    }

    # Command synonym dictionary
    synonyms: dict[str, list[str]] = {
        "ASAS": ["CD", "CDMETHOD"],
        "POS": ["AWY", "AIRPORT", "RUNWAYS", "AIRWAY", "AIRWAYS"],
        "BANK": ["BANKLIM"],
        "OP": ["CONTINUE", "RUN", "START"],
        "QUIT": ["CLOSE", "END", "EXIT", "STOP"],
        "DEL": ["DELETE"],
        "SELECTIMPL": ["IMPL", "IMPLEMENTATION", "IMPLEMENT"],
        "POLYLINE": ["LINES", "POLYLINES"],
        "MAGVAR": ["MAGDEC", "MAGDECL", "VAR"],
        "HOLD": ["PAUSE"],
        "POLY": ["POLYGON"],
        "TRAIL": ["TRAILS"],
        "PERFSTATS": ["PERFINFO", "PERFDATA"],
        "PLUGINS": ["PLUGIN"],
    }

    definitions = {name: CommandDefinition(*values) for name, values in cmddict.items()}
    aliases = {name: tuple(names) for name, names in synonyms.items()}
    return CommandCatalog(definitions, aliases)
