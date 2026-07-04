"""Stack argument parsers.

Converts the text arguments of stack commands into typed Python values.
Every argument type that can appear in a command's argument specification
(e.g., "alt", "spd", "latlon", "callsign") maps to a Parser object in the
module-level ``argparsers`` dictionary. For each function parameter of a
stack command a Parameter object is created, which selects the applicable
parsers based on the command's annotation string; when multiple types are
allowed (separated by "/"), each parser is tried in turn.

The module-level ``refdata`` namespace stores reference data (position,
aircraft index, heading, speed) taken from previously parsed arguments, so
that context-dependent arguments - such as a bare waypoint name resolved to
the closest occurrence, or a magnetic heading - can be interpreted relative
to the last parsed position or aircraft.
"""

import inspect
import re
from collections.abc import Callable
from types import SimpleNamespace
from typing import Any

import minisky
from minisky.tools.convert import (
    txt2alt,
    txt2bool,
    txt2hdg,
    txt2lat,
    txt2lon,
    txt2spd,
    txt2tim,
    txt2vs,
)
from minisky.tools.position import Position, islat

# Regular expression for argument parser
# Reading the regular expression:
# [\'"]?             : skip potential opening quote
# (?<=[\'"])[^\'"]+  : look behind for a leading quote, and if so, parse everything until closing quote
# (?<![\'"])[^\s,]+  : look behind for not a leading quote, then parse until first whitespace or comma
# [\'"]?\s*,?\s*     : skip potential closing quote, whitespace, and a potential single comma
re_getarg = re.compile(r'\s*[\'"]?((?<=[\'"])[^\'"]*|(?<![\'"])[^\s,]*)[\'"]?\s*,?\s*(.*)')


def _match_groups(argstring: str) -> tuple[str, str]:
    """Match argstring against re_getarg (which always matches) and return groups."""
    m = re_getarg.match(argstring)
    assert m is not None
    return m.groups()  # type: ignore[return-value]


# Stack reference data namespace
refdata = SimpleNamespace(lat=None, lon=None, alt=None, acidx=-1, hdg=None, cas=None)


def getnextarg(cmdstring: str) -> tuple:
    """Return first argument and remainder of command string from cmdstring.

    Arguments are separated by whitespace and/or a comma; quoted arguments
    may contain separators and are returned without the quotes.

    Args:
        cmdstring: (Partial) command-line text.

    Returns:
        tuple: (first argument (str), remaining command string (str)).
    """
    return _match_groups(cmdstring)


def reset() -> None:
    """Reset reference data.

    Clears the stored reference position, aircraft index, heading, and
    speed used to resolve context-dependent arguments.
    """
    refdata.lat = None
    refdata.lon = None
    refdata.alt = None
    refdata.acidx = -1
    refdata.hdg = None
    refdata.cas = None


class Parameter:
    """Wrapper class for stack function parameters.

    Combines a parameter from the function signature of a stack command
    with the argument type annotation from the command definition, and
    builds the list of Parser objects that convert argument text into
    values. Calling a Parameter with an argument string returns the parsed
    value(s) followed by the remaining argument string.

    Attributes:
        name: Parameter name in the function signature.
        default: Default value, or inspect._empty when there is none.
        optional: True when the argument may be omitted.
        gobble: True when this parameter consumes all remaining arguments
            (variable-length argument without annotation).
        annotation: Argument type annotation (e.g., "alt", "latlon").
        parsers: Parser objects tried in order when parsing this parameter.
        valid: False for keyword-only or unparsable parameters.
    """

    def __init__(
        self, param: inspect.Parameter, annotation: str = "", isopt: "bool | None" = None
    ) -> None:
        self.name = param.name
        self.default = param.default
        self.optional = (
            (self.hasdefault() or param.kind == param.VAR_POSITIONAL) if isopt is None else isopt
        )
        self.gobble = param.kind == param.VAR_POSITIONAL and not annotation
        self.annotation = annotation or param.annotation

        # Make list of parsers
        if self.annotation is inspect._empty:
            # Without annotation the argument is passed on unchanged as string
            # (i.e., the 'word' argument type)
            self.parsers = [Parser(str)]
            self.annotation = "word"
        elif isinstance(self.annotation, str):
            # If the annotation is a string we get our parsers from the argparsers dict
            pfuns = [argparsers.get(a) for a in self.annotation.split("/")]
            self.parsers = [p for p in pfuns if p is not None]
        elif isinstance(param.annotation, type) and issubclass(param.annotation, Parser):
            # If the paramter annotation is a class derived from Parser
            self.parsers = [self.annotation()]
        else:
            # All other annotation types are expected to have default behaviour
            # and are wrapped in Parser
            self.parsers = [Parser(self.annotation)]

        # This parameter is not valid if it has no parsers, or is keyword-only.
        # In those cases it can be skipped from the list of parameters when
        # processing a stack command line.
        self.valid = bool(self.parsers) and self.canwrap(param)

    def __call__(self, argstring: str):
        """Parse the next argument(s) for this parameter from argstring.

        Args:
            argstring: Remaining command-line text.

        Returns:
            tuple: Parsed value(s) followed by the remaining argument
            string. When the argument is omitted the default value (or
            None for optional arguments) is returned instead.

        Raises:
            ArgumentError: When a required argument is missing, or when
                all available parsers fail.
        """
        # First check if argument is omitted and default value is needed
        if not argstring or argstring[0] == ",":
            _, argstring = _match_groups(argstring)
            if self.hasdefault():
                return self.default, argstring
            if self.optional:
                return (None, argstring) if argstring else ("",)
            raise ArgumentError(f"Missing argument {self.name}")
        # Try available parsers
        error = ""
        for parser in self.parsers:
            try:
                return parser.parse(argstring)
            except (ValueError, ArgumentError) as e:
                error += "\n" + e.args[0]

        # If all fail, raise error
        raise ArgumentError(error)

    def __str__(self) -> str:
        return f"{self.name}:{self.annotation}"

    def __bool__(self) -> bool:
        return self.valid

    def size(self) -> int:
        """Returns the (maximum) number of return variables when parsing this
        parameter."""
        return max(p.size for p in self.parsers)

    def hasdefault(self) -> bool:
        """Returns True if this parameter has a default value."""
        return self.default is not inspect._empty

    @staticmethod
    def canwrap(param: inspect.Parameter) -> bool:
        """Returns True if Parameter can be used to wrap given function parameter.
        Returns False if param is keyword-only."""
        return param.kind not in (param.VAR_KEYWORD, param.KEYWORD_ONLY)


class ArgumentError(Exception):
    """This error is raised when stack argument parsing fails."""

    pass


class Parser:
    """Base implementation of argument parsers
    that are used to parse arguments to stack commands.

    The base implementation extracts one argument from the argument string
    and passes it to a conversion function (e.g., float, txt2alt). Derived
    classes implement more complex parsing, such as positions that consume
    one or two arguments.

    Attributes:
        size: Class attribute; the (maximum) number of values this parser
            returns (e.g., 2 for a lat/lon position).
        parsefun: Function that converts one argument string to a value.
    """

    # Output size of this parser
    size = 1

    def __init__(self, parsefun: "Callable[..., Any] | None" = None) -> None:
        self.parsefun = parsefun

    def parse(self, argstring: str) -> tuple:
        """Parse the next argument from argstring.

        Args:
            argstring: Remaining command-line text.

        Returns:
            tuple: (parsed value, remaining argument string).
        """
        curarg, argstring = _match_groups(argstring)
        assert self.parsefun is not None
        return self.parsefun(curarg), argstring


class StringArg(Parser):
    """Argument parser that simply consumes the entire remaining text string."""

    def parse(self, argstring: str) -> tuple:
        """Return the complete remaining text as a single string argument."""
        return argstring, ""


class CallsignArg(Parser):
    """Argument parser for aircraft callsigns and group ids."""

    def parse(self, argstring: str) -> tuple:
        """Parse a callsign or group name into traffic index/indices.

        For an aircraft callsign the traffic index is returned and the
        parser reference position is updated to the aircraft position;
        for a group name the list of member indices is returned.

        Raises:
            ArgumentError: When no aircraft with the given callsign exists.
        """
        arg, argstring = _match_groups(argstring)
        callsign = arg.upper()
        if callsign in minisky.traf.groups:
            idx = minisky.traf.groups.listgroup(callsign)
        else:
            idx = minisky.traf.idx(callsign)
            if idx < 0:
                raise ArgumentError(f"Aircraft with callsign {callsign} not found")

            # Update ref position for navdb lookup
            refdata.lat = minisky.traf.lat[idx]
            refdata.lon = minisky.traf.lon[idx]
            refdata.acidx = idx
        return idx, argstring


class WptArg(Parser):
    """Argument parser for waypoints.
    Makes 1 or 2 argument(s) into 1 position text to be used as waypoint

    Examples valid position texts:
    lat/lon : "N52.12,E004.23","N52'14'12',E004'23'10"
    navaid/fix: "SPY","OA","SUGOL"
    airport:   "EHAM"
    runway:    "EHAM/RW06" "LFPG/RWY23"
    Default values
    """

    def parse(self, argstring: str) -> tuple:
        """Combine one or two arguments into a single waypoint position text.

        Aircraft ids are translated to a "lat,lon" text; lat/lon pairs and
        airport/runway combinations are joined into one string.
        """
        arg, argstring = _match_groups(argstring)
        name = arg.upper()

        # Try aircraft first: translate a/c id into a valid position text with a lat,lon
        idx = minisky.traf.idx(name)
        if idx >= 0:
            name = f"{minisky.traf.lat[idx]},{minisky.traf.lon[idx]}"

        # Check if lat/lon combination
        elif islat(name):
            # lat,lon ? Combine into one string with a comma
            arg, argstring = _match_groups(argstring)
            name = name + "," + arg

        # apt,runway ? Combine into one string with a slash as separator
        elif argstring[:2].upper() == "RW" and name in minisky.navdb.aptid:
            arg, argstring = _match_groups(argstring)
            name = name + "/" + arg.upper()

        return name, argstring


class PosArg(Parser):
    """Argument parser for lat/lon positions.
    Makes 1 or 2 argument(s) into a lat/lon coordinate

    Examples valid position texts:
    lat/lon : "N52.12,E004.23","N52'14'12',E004'23'10"
    navaid/fix: "SPY","OA","SUGOL"
    airport:   "EHAM"
    runway:    "EHAM/RW06" "LFPG/RWY23"
    Default values
    """

    # This parser's output size is 2 (lat, lon)
    size = 2

    def parse(self, argstring: str) -> tuple:
        """Parse one or two arguments into a lat/lon position.

        Also updates the parser reference position to the parsed location.

        Returns:
            tuple: (lat [deg], lon [deg], remaining argument string).

        Raises:
            ArgumentError: When the text is not a valid waypoint, airport,
                runway, or aircraft id.
        """
        arg, argstring = _match_groups(argstring)
        argu = arg.upper()

        # Try aircraft first: translate a/c id into a valid position text with a lat,lon
        idx = minisky.traf.idx(argu)
        if idx >= 0:
            return minisky.traf.lat[idx], minisky.traf.lon[idx], argstring

        # Check if lat/lon combination
        if islat(argu):
            nextarg, argstring = _match_groups(argstring)
            refdata.lat = txt2lat(argu)
            refdata.lon = txt2lon(nextarg)
            return txt2lat(argu), txt2lon(nextarg), argstring

        # apt,runway ? Combine into one string with a slash as separator
        if argstring[:2].upper() == "RW" and argu in minisky.navdb.aptid:
            arg, argstring = _match_groups(argstring)
            argu = argu + "/" + arg.upper()

        if refdata.lat is None:
            refdata.lat, refdata.lon = minisky.scr.getviewctr()

        posobj = Position(argu, refdata.lat, refdata.lon)
        if posobj.error:
            raise ArgumentError(f"{argu} is not a valid waypoint, airport, runway, or aircraft id.")

        # Update reference lat/lon
        refdata.lat = posobj.lat
        refdata.lon = posobj.lon
        refdata.hdg = posobj.refhdg

        return posobj.lat, posobj.lon, argstring


class PandirArg(Parser):
    """Parse pan direction commands."""

    def parse(self, argstring: str) -> tuple:
        """Parse a screen pan direction (LEFT, RIGHT, UP/ABOVE, or DOWN).

        Raises:
            ArgumentError: When the text is not a valid pan direction.
        """
        arg, argstring = _match_groups(argstring)
        pandir = arg.upper()
        if pandir not in ("LEFT", "RIGHT", "UP", "ABOVE", "DOWN"):
            raise ArgumentError(f"{arg} is not a valid pan direction")
        return pandir, argstring


argparsers = {
    "*": None,
    "txt": Parser(str.upper),
    "word": Parser(str),
    "string": StringArg(),
    "float": Parser(float),
    "int": Parser(int),
    "onoff": Parser(txt2bool),
    "bool": Parser(txt2bool),
    "callsign": CallsignArg(),
    "wpt": WptArg(),
    "latlon": PosArg(),
    "lat": PosArg(),
    "lon": None,
    "pandir": PandirArg(),
    "spd": Parser(txt2spd),
    "vspd": Parser(txt2vs),
    "alt": Parser(txt2alt),
    "hdg": Parser(lambda txt: txt2hdg(txt, refdata.lat, refdata.lon)),
    "time": Parser(txt2tim),
}
