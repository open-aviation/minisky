"""Typed stack-command declarations and parsers.

BlueSky scenario file grammar:

```
batch          = command, { optional-space, ";", optional-space, command } ;
command        = argument, { separator, argument } ;
separator      = space, { space }
                | optional-space, ",", optional-space ;
argument       = bare | single-quoted | double-quoted ;
bare           = bare-char, { bare-char } ;
single-quoted  = "'", { any-char-except-single-quote }, "'" ;
double-quoted  = '"', { any-char-except-double-quote }, '"' ;
```
"""

from __future__ import annotations

import inspect
from collections.abc import Callable, Iterable, Iterator
from dataclasses import dataclass, replace
from functools import cache
from types import UnionType
from typing import (
    TYPE_CHECKING,
    Annotated,
    Any,
    Generic,
    Literal,
    TypeAlias,
    TypeVar,
    Union,
    get_args,
    get_origin,
    overload,
)

import numpy as np
from annotated_types import (
    BaseMetadata,
    Ge,
    GroupedMetadata,
    Gt,
    IsFinite,
    Le,
    Lt,
    MaxLen,
    MinLen,
    Predicate,
)

from minisky.identifiers import normalize_command_name
from minisky.result import Err, Ok, Result
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
from minisky.tools.position import islat

if TYPE_CHECKING:
    from minisky.tools.navdata import Navdatabase
    from minisky.traffic import Traffic

CommandCallback = Callable[..., Any]
CommandTarget = TypeVar("CommandTarget", bound=CommandCallback)
ParsedT_co = TypeVar("ParsedT_co", covariant=True)
_COMMAND = "__minisky_command__"


#
# lexical grammar
#


@dataclass(frozen=True, slots=True)
class SourceSpan:
    """Half-open source range for a parsed value."""

    start: int
    end: int

    def offset_by(self, offset: int) -> SourceSpan:
        return SourceSpan(self.start + offset, self.end + offset)


@dataclass(frozen=True, slots=True)
class Parsed(Generic[ParsedT_co]):
    """A successfully parsed prefix together with the unconsumed input.

    Command parsers compose left-to-right. A parser consumes only the prefix
    that belongs to its value and returns the untouched `remainder` for the
    next parameter.
    """

    value: ParsedT_co
    remainder: str
    span: SourceSpan
    """The source characters that produced `value`."""


@dataclass(frozen=True, slots=True)
class ArgumentIssue:
    """An argument error with optional source location context."""

    message: str
    span: SourceSpan | None = None
    source_text: str | None = None

    @classmethod
    def expected(
        cls, expected: str, actual: object, span: SourceSpan | None = None
    ) -> ArgumentIssue:
        # NOTE: while we would like to eventually move towards thiserror-like structured error messages
        # not implementing it for simplicity
        return cls(f"expected {expected}, but got {actual}", span)

    def offset_by(self, offset: int) -> ArgumentIssue:
        span = self.span.offset_by(offset) if self.span is not None else None
        return replace(self, span=span)

    def with_span(self, span: SourceSpan | None) -> ArgumentIssue:
        return replace(self, span=span or self.span)

    def at_argument(
        self, name: str, source_text: str, offset: int, fallback: SourceSpan
    ) -> ArgumentIssue:
        span = (self.span or fallback).offset_by(offset)
        return replace(
            self,
            message=f"argument `{name}`: {self.message}",
            span=span,
            source_text=source_text,
        )

    def __str__(self) -> str:
        if self.source_text is None or self.span is None:
            return self.message
        width = max(1, self.span.end - self.span.start)
        marker = " " * self.span.start + "^" * width
        return f"{self.message}\n{self.source_text}\n{marker}"


ParseResult: TypeAlias = Result[Parsed[ParsedT_co], ArgumentIssue]


def next_argument(text: str) -> ParseResult[str]:
    """Parse a legacy command token without raising for invalid input."""
    index = 0
    length = len(text)
    while index < length and text[index].isspace():
        index += 1

    if index == length:
        return Ok(Parsed("", "", SourceSpan(length, length)))
    if text[index] == ",":
        start = index
        index += 1
        while index < length and text[index].isspace():
            index += 1
        return Ok(Parsed("", text[index:], SourceSpan(start, start + 1)))

    token_start = index
    quote = text[index] if text[index] in ("'", '"') else None
    if quote is not None:
        index += 1
        end = text.find(quote, index)
        if end < 0:
            return Err(
                ArgumentIssue.expected(
                    f"a closing {quote} quote", "end of input", SourceSpan(token_start, length)
                )
            )
        value = text[index:end]
        index = end + 1
        token_end = index
        if index < length and not text[index].isspace() and text[index] != ",":
            return Err(
                ArgumentIssue.expected(
                    "a separator after the quoted argument",
                    text[index],
                    SourceSpan(token_start, index + 1),
                )
            )
    else:
        start = index
        while index < length and not text[index].isspace() and text[index] != ",":
            # Apostrophes inside a bare token are part of legacy DMS positions,
            # for example N52'14'12'. Quotes only delimit an argument when they
            # appear at its start.
            index += 1
        value = text[start:index]
        token_end = index

    while index < length and text[index].isspace():
        index += 1
    if index < length and text[index] == ",":
        index += 1
    while index < length and text[index].isspace():
        index += 1
    return Ok(Parsed(value, text[index:], SourceSpan(token_start, token_end)))


def split_commands(text: str) -> Result[tuple[str, ...], ArgumentIssue]:
    """Split a semicolon-delimited batch without splitting quoted arguments."""
    commands: list[str] = []
    start = 0
    quote: str | None = None
    quote_start: int | None = None
    argument_start = True
    for index, character in enumerate(text):
        if quote is not None:
            if character == quote:
                quote = None
            continue
        if argument_start and character in ("'", '"'):
            quote = character
            quote_start = index
            argument_start = False
        elif character == ";":
            command_text = text[start:index].strip()
            if command_text:
                commands.append(command_text)
            start = index + 1
            argument_start = True
        elif character.isspace() or character == ",":
            argument_start = True
        else:
            argument_start = False
    if quote is not None:
        start = quote_start if quote_start is not None else len(text)
        return Err(
            ArgumentIssue(
                f"expected a closing {quote} quote, but got end of input",
                SourceSpan(start, len(text)),
                text,
            )
        )
    command_text = text[start:].strip()
    if command_text:
        commands.append(command_text)
    return Ok(tuple(commands))


#
# parser schema
#


@dataclass(frozen=True, slots=True)
class CommandParseContext:
    """Read-only runtime services available while parsing a command value.

    BlueSky uses mutable `bs.ref` fields as cross-argument registers. We
    intentionally expose only runtime lookup services here.
    """

    traffic: Traffic
    navigation: Navdatabase


@dataclass(frozen=True, slots=True)
class NamedFields:
    """Semantic field names used when rendering command usage.

    A parser may consume several BlueSky fields. For example, a wind level
    consumes altitude, direction, and speed.
    """

    names: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.names:
            raise ValueError("command parser fields cannot be empty")


@dataclass(frozen=True, slots=True)
class LiteralSyntax:
    """Exact keywords used when rendering command usage.

    Values are stored in their case-insensitive command form.
    """

    values: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class OmittedSyntax:
    """A field that must be present syntactically but contain no value.

    BlueSky command text uses adjacent commas to skip positional fields. In
    `CMD A,,B`, the middle field is omitted.
    """


@dataclass(frozen=True, slots=True)
class ChoiceSyntax:
    """Alternative syntax used when rendering a union annotation."""

    alternatives: tuple[ParserSyntax | None, ...]


ParserSyntax: TypeAlias = NamedFields | LiteralSyntax | OmittedSyntax | ChoiceSyntax
"""Structured parser syntax available to help and documentation renderers."""


ParseFunction: TypeAlias = Callable[[CommandParseContext, str], ParseResult[ParsedT_co]]


@dataclass(frozen=True, slots=True)
class CmdParser(Generic[ParsedT_co]):
    """Connect a Python value type to command parsing and usage syntax.

    This is the metadata inside `Annotated`, which describes what a callback
    receives. Use `fields()` for a multi-field value's usage names and the
    syntax helpers when generated usage needs more detail than a parameter name.

    Custom parsers should be side-effect free.
    """

    func: ParseFunction[ParsedT_co]
    """A text-to-value conversion function that returns a [`Parsed`][minisky.command.Parsed]."""
    syntax: ParserSyntax | None = None
    # NOTE: not collapsing it to a string here
    # TODO: make zensical macros consume this to build a rich display
    # and serialise it to tangram

    @classmethod
    def fields(
        cls, func: ParseFunction[ParsedT_co], names: tuple[str, ...]
    ) -> CmdParser[ParsedT_co]:
        return cls(func, NamedFields(names))

    @classmethod
    def literals(
        cls, func: ParseFunction[ParsedT_co], values: tuple[str, ...]
    ) -> CmdParser[ParsedT_co]:
        normalized = tuple(value.upper() for value in values)
        if not normalized:
            raise ValueError("command parser literals cannot be empty")
        if len(normalized) != len(set(normalized)):
            raise ValueError("command parser literals must be unique ignoring case")
        return cls(func, LiteralSyntax(normalized))

    @classmethod
    def omitted(cls, func: ParseFunction[ParsedT_co]) -> CmdParser[ParsedT_co]:
        return cls(func, OmittedSyntax())

    def __call__(self, context: CommandParseContext, text: str) -> ParseResult[ParsedT_co]:
        return self.func(context, text)


#
# primitives
#


FiniteFloat: TypeAlias = IsFinite[float]
NonNegativeFiniteFloat: TypeAlias = Annotated[FiniteFloat, Ge(0)]
PositiveFiniteFloat: TypeAlias = Annotated[FiniteFloat, Gt(0)]


def _convert_value(
    value: str,
    converter: Callable[[str], ParsedT_co],
    expected: str,
) -> Result[ParsedT_co, ArgumentIssue]:
    try:
        return Ok(converter(value))
    except ValueError:
        return Err(ArgumentIssue.expected(expected, value))


def _convert(
    text: str,
    converter: Callable[[str], ParsedT_co],
    expected: str,
) -> ParseResult[ParsedT_co]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()

    if isinstance(converted := _convert_value(token.value, converter, expected), Err):
        return Err(converted.err().with_span(token.span))
    return Ok(Parsed(converted.ok(), token.remainder, token.span))


def parse_token(_context: CommandParseContext, text: str) -> ParseResult[str]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    if not token.value:
        return Err(ArgumentIssue.expected("a non-empty argument", "empty input", token.span))
    return result


Token = Annotated[str, CmdParser(parse_token)]
"""A non-empty BlueSky command field parsed as text.

Normal tokenization applies: spaces or commas terminate bare fields and surrounding
quotes are removed. Unlike [`Text`][minisky.command.Text], `Token` never consumes
the rest of the command line.
"""


def parse_keyword(context: CommandParseContext, text: str) -> ParseResult[str]:
    if isinstance(result := parse_token(context, text), Err):
        return result
    token = result.ok()
    return Ok(Parsed(token.value.upper(), token.remainder, token.span))


Keyword = Annotated[str, CmdParser(parse_keyword)]
"""A [`Token`][minisky.command.Token] normalized to upper case.

Use this for case-insensitive command keywords whose value remains data.
Python `Literal[...]` annotations use
[`LiteralSyntax`][minisky.command.LiteralSyntax] when the keyword is grammar.
"""


@dataclass(frozen=True, slots=True)
class OmittedField:
    """Sentinel for indicating a required empty comma field was present."""


_OMITTED_FIELD = OmittedField()


def parse_omitted_field(_context: CommandParseContext, text: str) -> ParseResult[OmittedField]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    if token.value or token.span.start == token.span.end:
        return Err(
            ArgumentIssue.expected("an omitted comma field", token.value or "input", token.span)
        )
    return Ok(Parsed(_OMITTED_FIELD, token.remainder, token.span))


Omitted = Annotated[
    OmittedField,
    CmdParser.omitted(parse_omitted_field),
]
"""A required empty positional field, such as the middle field in `CMD A,,B`."""


def parse_text(_context: CommandParseContext, text: str) -> ParseResult[str]:
    # BlueSky's `string` parser consumed the remainder verbatim.
    return Ok(Parsed(text, "", SourceSpan(0, len(text))))


Text = Annotated[str, CmdParser(parse_text)]
"""The complete remaining command text, consumed verbatim.

This intentionally preserves BlueSky's `string` terminal for nested commands such
as ECHO and DELAY. Unlike [`Token`][minisky.command.Token], quotes are not stripped
because no further tokenization occurs.
"""


def parse_int(_context: CommandParseContext, text: str) -> ParseResult[int]:
    return _convert(text, int, "an integer")


def parse_float(_context: CommandParseContext, text: str) -> ParseResult[float]:
    return _convert(text, float, "a number")


def parse_on_off(_context: CommandParseContext, text: str) -> ParseResult[bool]:
    return _convert(text, txt2bool, "ON or OFF")


OnOff = Annotated[bool, CmdParser(parse_on_off)]


def parse_altitude_value(value: str) -> Result[float, ArgumentIssue]:
    return _convert_value(value, txt2alt, "an altitude")


def parse_altitude(_context: CommandParseContext, text: str) -> ParseResult[float]:
    return _convert(text, txt2alt, "an altitude")


AltM = Annotated[float, CmdParser(parse_altitude)]


def parse_speed_value(value: str) -> Result[float, ArgumentIssue]:
    return _convert_value(value, txt2spd, "a speed")


def parse_speed(_context: CommandParseContext, text: str) -> ParseResult[float]:
    return _convert(text, txt2spd, "a speed")


SpeedMpsOrMach = Annotated[float, CmdParser(parse_speed)]


def parse_vertical_speed(_context: CommandParseContext, text: str) -> ParseResult[float]:
    return _convert(text, txt2vs, "a vertical speed")


VspdMps = Annotated[float, CmdParser(parse_vertical_speed)]


def parse_time(_context: CommandParseContext, text: str) -> ParseResult[float]:
    return _convert(text, txt2tim, "a time")


TimeS = Annotated[float, CmdParser(parse_time)]


#
# domain-specific
#


# NOTE: TrueHeadingDeg and MagneticHeadingDeg is scheduled for removal when #40 is implemented.


@dataclass(frozen=True, slots=True)
class TrueHeadingDeg:
    """A true heading in degrees."""

    degrees: float


@dataclass(frozen=True, slots=True)
class MagneticHeadingDeg:
    """A magnetic heading in degrees, not yet resolved to true north."""

    degrees: float


def _parse_heading_token(
    token: Parsed[str],
) -> Result[TrueHeadingDeg | MagneticHeadingDeg, ArgumentIssue]:
    value = token.value.upper()
    if value.endswith("M"):
        try:
            return Ok(MagneticHeadingDeg(float(value[:-1])))
        except ValueError:
            return Err(ArgumentIssue.expected("a heading", token.value, token.span))
    if "M" in value:
        return Err(ArgumentIssue.expected("a heading", token.value, token.span))
    try:
        return Ok(TrueHeadingDeg(txt2hdg(value)))
    except ValueError:
        return Err(ArgumentIssue.expected("a heading", token.value, token.span))


def parse_heading(
    _context: CommandParseContext, text: str
) -> ParseResult[TrueHeadingDeg | MagneticHeadingDeg]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    if isinstance(value := _parse_heading_token(token), Err):
        return value
    return Ok(Parsed(value.ok(), token.remainder, token.span))


HeadingDeg = Annotated[TrueHeadingDeg | MagneticHeadingDeg, CmdParser(parse_heading)]
"""Heading syntax preserving whether the input refers to true or magnetic north."""


def parse_true_heading(_context: CommandParseContext, text: str) -> ParseResult[TrueHeadingDeg]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    if isinstance(value := _parse_heading_token(token), Err):
        return value
    heading = value.ok()
    if not isinstance(heading, TrueHeadingDeg):
        return Err(ArgumentIssue.expected("a true heading", token.value, token.span))
    return Ok(Parsed(heading, token.remainder, token.span))


TrueHdgDeg = Annotated[TrueHeadingDeg, CmdParser(parse_true_heading)]
"""Numeric or explicitly true heading syntax, parsed as [`TrueHeadingDeg`][minisky.command.TrueHeadingDeg]."""


def parse_magnetic_heading(
    _context: CommandParseContext, text: str
) -> ParseResult[MagneticHeadingDeg]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    if isinstance(value := _parse_heading_token(token), Err):
        return value
    heading = value.ok()
    if not isinstance(heading, MagneticHeadingDeg):
        return Err(ArgumentIssue.expected("a magnetic heading", token.value, token.span))
    return Ok(Parsed(heading, token.remainder, token.span))


MagneticHdgDeg = Annotated[MagneticHeadingDeg, CmdParser(parse_magnetic_heading)]
"""Magnetic heading syntax such as `090M`, parsed as [`MagneticHeadingDeg`][minisky.command.MagneticHeadingDeg]."""


@dataclass(frozen=True, slots=True)
class RunwayHeadingRequest:
    """Preserve a source-level `*` request until the command can resolve it.

    For example, in `CRE KLM1,A320,EHAM,RWY18L,*,0,250`, `*` means "use the
    heading of the runway parsed in the previous argument". Unlike bluesky
    (which internally stores the RWY18L in its parser), minisky directly returns
    the sentinel to avoid a parser-global cross-argument state.
    """


_RUNWAY_HEADING = RunwayHeadingRequest()


def parse_runway_heading(
    _context: CommandParseContext, text: str
) -> ParseResult[RunwayHeadingRequest]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    if token.value != "*":
        return Err(ArgumentIssue.expected("*", token.value, token.span))
    return Ok(Parsed(_RUNWAY_HEADING, token.remainder, token.span))


UseRunwayHeading = Annotated[
    RunwayHeadingRequest,
    CmdParser.literals(parse_runway_heading, ("*",)),
]
"""The `*` heading form, preserved as an explicit callback value.

Only commands that can derive a heading from another argument should include this
in their annotation, for example `HeadingDeg | UseRunwayHeading`.
"""


def _aircraft_index(context: CommandParseContext, callsign: str) -> Result[int, ArgumentIssue]:
    index = context.traffic.idx(callsign)
    if index < 0:
        return Err(ArgumentIssue.expected("an existing aircraft", callsign))
    return Ok(index)


def parse_aircraft(context: CommandParseContext, text: str) -> ParseResult[int]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    callsign = token.value.upper()

    if context.traffic.idx(callsign) < 0 and callsign in context.traffic.groups:
        return Err(ArgumentIssue.expected("an aircraft", f"group {callsign}", token.span))

    if isinstance(index := _aircraft_index(context, callsign), Err):
        return Err(index.err().with_span(token.span))
    return Ok(Parsed(index.ok(), token.remainder, token.span))


AcId = Annotated[int, CmdParser(parse_aircraft)]
"""An existing aircraft callsign resolved to its traffic-array index."""


def parse_aircraft_selection(
    context: CommandParseContext, text: str
) -> ParseResult[np.ndarray[Any, Any]]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    name = token.value.upper()

    if name in {"*", "ALL"} or name in context.traffic.groups:
        if isinstance(selection := context.traffic.groups.listgroup(name), Err):
            return Err(
                ArgumentIssue.expected("an existing aircraft or group", selection.err(), token.span)
            )
        return Ok(Parsed(selection.ok(), token.remainder, token.span))

    if isinstance(index := _aircraft_index(context, name), Err):
        return Err(ArgumentIssue.expected("an existing aircraft or group", name, token.span))
    selection = np.asarray([index.ok()], dtype=int)
    return Ok(Parsed(selection, token.remainder, token.span))


AcIdSelection = Annotated[np.ndarray[Any, Any], CmdParser(parse_aircraft_selection)]
"""An aircraft, a traffic group, or `*`/`ALL`, resolved to traffic indices."""


def aircraft_indices(
    selections: Iterable[np.ndarray[Any, Any]],
) -> np.ndarray[Any, Any]:
    """Flatten aircraft and group selections into an index array."""
    indices = (int(index) for selection in selections for index in selection)
    return np.fromiter(indices, dtype=int)


@dataclass(frozen=True, slots=True)
class LatLonDegrees:
    """Resolved latitude and longitude in degrees."""

    lat: float
    lon: float


@dataclass(frozen=True, slots=True)
class NamedWaypoint:
    """A waypoint expression that should be resolved by name."""

    name: str


@dataclass(frozen=True, slots=True)
class CoordinateWaypoint:
    """A waypoint expressed directly as latitude and longitude."""

    coordinates: LatLonDegrees
    source: str


WaypointSpec: TypeAlias = NamedWaypoint | CoordinateWaypoint
"""Structured waypoint syntax consumed by route and origin/destination commands."""


def parse_waypoint(_context: CommandParseContext, text: str) -> ParseResult[WaypointSpec]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    name = token.value.upper()
    remainder = token.remainder

    if islat(name):
        offset = len(text) - len(remainder)
        if isinstance(longitude_result := next_argument(remainder), Err):
            return Err(longitude_result.err().offset_by(offset))
        longitude = longitude_result.ok()
        if not longitude.value:
            return Err(
                ArgumentIssue.expected(
                    "a longitude after the latitude", "end of input", longitude.span
                ).offset_by(offset)
            )
        end = offset + longitude.span.end
        span = SourceSpan(token.span.start, end)
        try:
            coordinates = LatLonDegrees(txt2lat(name), txt2lon(longitude.value))
        except ValueError:
            return Err(
                ArgumentIssue.expected(
                    "a latitude and longitude", f"{name},{longitude.value}", span
                )
            )
        return Ok(
            Parsed(
                CoordinateWaypoint(coordinates, f"{name},{longitude.value}"),
                longitude.remainder,
                span,
            )
        )

    if remainder[:2].upper() == "RW":
        offset = len(text) - len(remainder)
        if isinstance(runway_result := next_argument(remainder), Err):
            return Err(runway_result.err().offset_by(offset))
        runway = runway_result.ok()
        span = SourceSpan(token.span.start, offset + runway.span.end)
        return Ok(
            Parsed(
                NamedWaypoint(f"{name}/{runway.value.upper()}"),
                runway.remainder,
                span,
            )
        )

    return Ok(Parsed(NamedWaypoint(name), remainder, token.span))


Wpt = Annotated[WaypointSpec, CmdParser(parse_waypoint)]
"""A named or coordinate waypoint expression, preserved as structured syntax."""


@dataclass(frozen=True, slots=True)
class RunwayPosition:
    """Resolved runway coordinates and their runway heading."""

    coordinates: LatLonDegrees
    runway_heading: float


ResolvedPosition: TypeAlias = LatLonDegrees | RunwayPosition


def _resolve_named_position(
    context: CommandParseContext, name: str, span: SourceSpan | None
) -> Result[ResolvedPosition, ArgumentIssue]:
    navigation = context.navigation

    if "/RW" in name:
        airport, runway_text = name.split("/RW", maxsplit=1)
        runway = runway_text.lstrip("Y")
        try:
            lat, lon, heading = navigation.rwythresholds[airport][runway]
        except KeyError:
            return Err(
                ArgumentIssue.expected("a waypoint, airport, runway, or aircraft id", name, span)
            )
        return Ok(RunwayPosition(LatLonDegrees(float(lat), float(lon)), float(heading)))

    if name in navigation.aptid:
        index = navigation.aptid.index(name)
        return Ok(LatLonDegrees(float(navigation.aptlat[index]), float(navigation.aptlon[index])))

    occurrences = navigation.wpid.count(name)
    if occurrences > 1:
        return Err(
            ArgumentIssue.expected("an unambiguous waypoint id or explicit coordinates", name, span)
        )
    if occurrences == 1:
        index = navigation.wpid.index(name)
        return Ok(LatLonDegrees(float(navigation.wplat[index]), float(navigation.wplon[index])))

    return Err(ArgumentIssue.expected("a waypoint, airport, runway, or aircraft id", name, span))


def parse_resolved_position(
    context: CommandParseContext, text: str
) -> ParseResult[ResolvedPosition]:
    if isinstance(result := parse_waypoint(context, text), Err):
        return result
    parsed = result.ok()
    waypoint = parsed.value

    if isinstance(waypoint, CoordinateWaypoint):
        return Ok(Parsed(waypoint.coordinates, parsed.remainder, parsed.span))

    index = context.traffic.idx(waypoint.name)
    if index >= 0:
        coordinates = LatLonDegrees(
            float(context.traffic.lat[index]), float(context.traffic.lon[index])
        )
        return Ok(Parsed(coordinates, parsed.remainder, parsed.span))

    if isinstance(resolved := _resolve_named_position(context, waypoint.name, parsed.span), Err):
        return resolved
    return Ok(Parsed(resolved.ok(), parsed.remainder, parsed.span))


ResolvedPositionArg = Annotated[ResolvedPosition, CmdParser(parse_resolved_position)]
"""A position expression resolved against navigation data and traffic.

Runways retain their heading in [`RunwayPosition`][minisky.command.RunwayPosition];
other positions become [`LatLonDegrees`][minisky.command.LatLonDegrees]. Ambiguous
waypoint identifiers are rejected because this grammar has no geographic reference.
"""


def parse_lat_lon(context: CommandParseContext, text: str) -> ParseResult[LatLonDegrees]:
    if isinstance(result := parse_resolved_position(context, text), Err):
        return result
    parsed = result.ok()
    coordinates = (
        parsed.value.coordinates if isinstance(parsed.value, RunwayPosition) else parsed.value
    )
    return Ok(Parsed(coordinates, parsed.remainder, parsed.span))


LatLonDeg = Annotated[LatLonDegrees, CmdParser(parse_lat_lon)]
"""A resolved position reduced to latitude and longitude degrees.

It accepts the same BlueSky position expressions as
[`ResolvedPositionArg`][minisky.command.ResolvedPositionArg] but intentionally
discards runway-specific heading after resolution.
"""


#
# parameter contracts
#


_Constraint: TypeAlias = Gt | Ge | Lt | Le | MinLen | MaxLen | Predicate


@dataclass(frozen=True, slots=True)
class _ArgumentType:
    """Compiled conversion and validation rules for an annotation.

    Annotations compile to reusable runtime contracts. Parsing succeeds only
    after the produced value satisfies every supported annotation constraint.
    """

    parser: CmdParser[Any]
    constraints: tuple[_Constraint, ...]
    nullable: bool = False

    def parse(self, context: CommandParseContext, text: str) -> ParseResult[Any]:
        if isinstance(parsed := self.parser(context, text), Err):
            return parsed

        value = parsed.ok()
        if isinstance(validation := _validate_constraints(value.value, self.constraints), Err):
            return Err(validation.err().with_span(value.span))
        return parsed


@dataclass(frozen=True, slots=True)
class ParsedArguments:
    """Values produced for a callback parameter plus remaining command text.

    Most parameters contribute a value. A repeated `*args` parameter may contribute
    many, so this is distinct from [`Parsed`][minisky.command.Parsed]
    """

    values: tuple[Any, ...]
    remainder: str


@dataclass(frozen=True, slots=True)
class Parameter:
    """Executable command contract compiled from a callback parameter.

    A default permits the field to be absent;
    `T | None` permits an explicitly empty comma field to become `None`;
    `*args` repeats the parser until input is exhausted.
    Note that in bluesky: `CMD A` (no field) and `CMD A,` (an empty positional
    field) are not always the same syntax.

    The [`CmdParser`][minisky.command.CmdParser] and annotation constraints are
    compiled when a command is mounted. Runtime parsing therefore applies an
    already-validated contract.
    """

    name: str
    argument_type: _ArgumentType
    default: object = inspect.Parameter.empty
    repeat: bool = False

    def parse(
        self,
        context: CommandParseContext,
        text: str,
        *,
        source_text: str,
        offset: int,
    ) -> Result[ParsedArguments, ArgumentIssue]:
        if self.repeat:
            return self._parse_repeated(context, text, source_text=source_text, offset=offset)
        if not text:
            return self._missing_or_default(
                source_text=source_text,
                offset=offset,
                span=SourceSpan(0, 0),
                actual="end of input",
                remainder=text,
            )

        if (omitted := self._omitted_field(text)) is not None:
            if self.argument_type.nullable:
                return Ok(ParsedArguments((None,), omitted.remainder))
            if self.default is not inspect.Parameter.empty:
                return Ok(ParsedArguments((self.default,), omitted.remainder))
        return self._parse_one(context, text, source_text=source_text, offset=offset)

    def _parse_repeated(
        self,
        context: CommandParseContext,
        text: str,
        *,
        source_text: str,
        offset: int,
    ) -> Result[ParsedArguments, ArgumentIssue]:
        values: list[Any] = []
        remainder = text
        while remainder:
            current_offset = offset + len(text) - len(remainder)
            if (omitted := self._omitted_field(remainder)) is not None:
                return Err(
                    self._missing(
                        source_text=source_text,
                        offset=current_offset,
                        span=omitted.span,
                        actual="an omitted field",
                    )
                )

            if isinstance(
                parsed_result := self._parse_one(
                    context, remainder, source_text=source_text, offset=current_offset
                ),
                Err,
            ):
                return parsed_result
            parsed = parsed_result.ok()
            if len(parsed.remainder) >= len(remainder):
                raise TypeError(f"parser {self.argument_type.parser!r} consumed no input")
            values.extend(parsed.values)
            remainder = parsed.remainder
        return Ok(ParsedArguments(tuple(values), remainder))

    def _missing_or_default(
        self,
        *,
        source_text: str,
        offset: int,
        span: SourceSpan,
        actual: str,
        remainder: str,
    ) -> Result[ParsedArguments, ArgumentIssue]:
        if self.default is not inspect.Parameter.empty:
            return Ok(ParsedArguments((self.default,), remainder))
        return Err(
            self._missing(
                source_text=source_text,
                offset=offset,
                span=span,
                actual=actual,
            )
        )

    @staticmethod
    def _omitted_field(text: str) -> Parsed[str] | None:
        index = len(text) - len(text.lstrip())
        if text[index : index + 1] != ",":
            return None
        result = next_argument(text)
        assert isinstance(result, Ok)
        return result.ok()

    def _missing(
        self,
        *,
        source_text: str,
        offset: int,
        span: SourceSpan,
        actual: str,
    ) -> ArgumentIssue:
        return ArgumentIssue.expected("a value", actual, span).at_argument(
            self.name, source_text, offset, span
        )

    def _parse_one(
        self,
        context: CommandParseContext,
        text: str,
        *,
        source_text: str,
        offset: int,
    ) -> Result[ParsedArguments, ArgumentIssue]:
        if isinstance(parsed_result := self.argument_type.parse(context, text), Err):
            issue = parsed_result.err()
            fallback = issue.span or SourceSpan(0, max(1, len(text)))
            return Err(issue.at_argument(self.name, source_text, offset, fallback))
        parsed = parsed_result.ok()
        return Ok(ParsedArguments((parsed.value,), parsed.remainder))

    @property
    def optional(self) -> bool:
        return self.repeat or self.default is not inspect.Parameter.empty

    @property
    def parser(self) -> CmdParser[Any]:
        """Parser metadata compiled from this parameter's annotation."""
        return self.argument_type.parser

    @property
    def nullable(self) -> bool:
        """Whether an explicit empty field may be parsed as `None`."""
        return self.argument_type.nullable


#
# annotation compilation
#


def compile_parameter(parameter: inspect.Parameter) -> Parameter:
    """Compile a callback parameter into an executable command contract."""
    argument_type = _argument_type(parameter.annotation)
    if argument_type is None:
        raise TypeError(
            f"unsupported stack annotation for {parameter.name}: {parameter.annotation!r}"
        )
    if parameter.default is None and not argument_type.nullable:
        raise TypeError(
            f"stack parameter {parameter.name} defaults to None but is not annotated T | None"
        )
    if (
        argument_type.nullable
        and parameter.default is not inspect.Parameter.empty
        and parameter.default is not None
    ):
        raise TypeError(
            f"nullable stack parameter {parameter.name} must default to None when optional"
        )
    return Parameter(
        name=parameter.name,
        argument_type=argument_type,
        default=parameter.default,
        repeat=parameter.kind is inspect.Parameter.VAR_POSITIONAL,
    )


def _argument_type(annotation: Any) -> _ArgumentType | None:
    nullable = _is_nullable(annotation)
    members = _union_members(annotation)
    if len(members) > 1:
        alternatives = tuple(_argument_type(member) for member in members)
        if any(alternative is None for alternative in alternatives):
            return None
        typed_alternatives = tuple(
            alternative for alternative in alternatives if alternative is not None
        )
        return _ArgumentType(_choice_parser(typed_alternatives), (), nullable=nullable)

    value_annotation = members[0] if members else annotation
    parser = _parser_for(value_annotation)
    if parser is None:
        return None
    return _ArgumentType(
        parser,
        _annotation_constraints(value_annotation),
        nullable=nullable,
    )


def _parser_for(annotation: Any) -> CmdParser[Any] | None:
    annotation = _without_none(annotation)
    if get_origin(annotation) is Literal:
        return _literal_parser(annotation)
    if annotation is inspect._empty or annotation is str or annotation is Any:
        return CmdParser(parse_token)
    parser = _annotated_parser(annotation)
    if parser is not None:
        return parser
    if get_origin(annotation) is Annotated:
        return _parser_for(get_args(annotation)[0])
    if annotation is bool:
        return CmdParser(parse_on_off)
    if annotation is int:
        return CmdParser(parse_int)
    if annotation is float:
        return CmdParser(parse_float)
    return None


@dataclass(frozen=True, slots=True)
class _ChoiceParse:
    alternatives: tuple[_ArgumentType, ...]

    def __call__(self, context: CommandParseContext, text: str) -> ParseResult[Any]:
        failure: ArgumentIssue | None = None
        for alternative in self.alternatives:
            result = alternative.parse(context, text)
            if isinstance(result, Ok):
                return result
            failure = result.err()
        assert failure is not None
        return Err(failure)


def _choice_parser(alternatives: tuple[_ArgumentType, ...]) -> CmdParser[Any]:
    syntax = ChoiceSyntax(tuple(alternative.parser.syntax for alternative in alternatives))
    return CmdParser(_ChoiceParse(alternatives), syntax)


def _literal_parser(annotation: Any) -> CmdParser[str]:
    values = get_args(annotation)
    if not values or any(not isinstance(value, str) for value in values):
        raise TypeError("stack command Literal values must be strings")
    literals = tuple(value.upper() for value in values)
    if len(literals) != len(set(literals)):
        raise TypeError("stack command Literal repeats a case-insensitive value")
    return _cached_literal_parser(literals)


@cache
def _cached_literal_parser(literals: tuple[str, ...]) -> CmdParser[str]:
    def parse_literal(context: CommandParseContext, text: str) -> ParseResult[str]:
        if isinstance(result := parse_keyword(context, text), Err):
            return result
        token = result.ok()
        if token.value not in literals:
            return Err(ArgumentIssue.expected(" or ".join(literals), token.value, token.span))
        return Ok(token)

    return CmdParser.literals(parse_literal, literals)


def _is_nullable(annotation: Any) -> bool:
    """Return whether an annotation explicitly allows None."""
    if get_origin(annotation) is Annotated:
        return _is_nullable(get_args(annotation)[0])
    if get_origin(annotation) not in (Union, UnionType):
        return False
    return type(None) in get_args(annotation)


def _union_members(annotation: Any) -> tuple[Any, ...]:
    if get_origin(annotation) not in (Union, UnionType):
        return ()
    return tuple(member for member in get_args(annotation) if member is not type(None))


def _without_none(annotation: Any) -> Any:
    members = _union_members(annotation)
    return members[0] if len(members) == 1 else annotation


def _annotated_parser(annotation: Any) -> CmdParser[Any] | None:
    if get_origin(annotation) is not Annotated:
        return None
    parsers = tuple(item for item in get_args(annotation)[1:] if isinstance(item, CmdParser))
    if len(parsers) > 1:
        raise TypeError("stack annotation contains multiple CmdParser markers")
    return parsers[0] if parsers else None


def _annotation_constraints(annotation: Any) -> tuple[_Constraint, ...]:
    annotation = _without_none(annotation)
    if get_origin(annotation) is not Annotated:
        return ()
    return tuple(_constraints(get_args(annotation)[1:]))


def _constraints(metadata: Iterable[object]) -> Iterator[_Constraint]:
    for item in metadata:
        if isinstance(item, GroupedMetadata):
            yield from _constraints(item)
        elif isinstance(item, (Gt, Ge, Lt, Le, MinLen, MaxLen, Predicate)):
            yield item
        elif isinstance(item, BaseMetadata):
            raise TypeError(f"unsupported annotated-types constraint: {item!r}")


def _validate_constraints(
    value: Any, constraints: Iterable[_Constraint]
) -> Result[None, ArgumentIssue]:
    for constraint in constraints:
        expected: str | None = None
        if isinstance(constraint, Gt) and not value > constraint.gt:
            expected = f"a value greater than {constraint.gt}"
        elif isinstance(constraint, Ge) and not value >= constraint.ge:
            expected = f"a value greater than or equal to {constraint.ge}"
        elif isinstance(constraint, Lt) and not value < constraint.lt:
            expected = f"a value less than {constraint.lt}"
        elif isinstance(constraint, Le) and not value <= constraint.le:
            expected = f"a value less than or equal to {constraint.le}"
        elif isinstance(constraint, MinLen) and len(value) < constraint.min_length:
            expected = f"a value with length at least {constraint.min_length}"
        elif isinstance(constraint, MaxLen) and len(value) > constraint.max_length:
            expected = f"a value with length at most {constraint.max_length}"
        elif isinstance(constraint, Predicate) and not constraint.func(value):
            name = getattr(constraint.func, "__name__", "predicate")
            expected = f"a value satisfying {name}"
        if expected is not None:
            return Err(ArgumentIssue.expected(expected, value))
    return Ok(None)


#
# command declarations
#


@dataclass(frozen=True, slots=True)
class CommandDeclaration:
    """Static command metadata stored on a decorated callable."""

    name: str = ""
    aliases: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", normalize_command_name(self.name) if self.name else "")
        object.__setattr__(
            self,
            "aliases",
            tuple(normalize_command_name(alias) for alias in self.aliases),
        )


@dataclass(frozen=True, slots=True)
class BoundCommand:
    """A declaration paired with its runtime-bound callback."""

    callback: CommandCallback
    source: CommandCallback
    declaration: CommandDeclaration

    @property
    def name(self) -> str:
        return self.declaration.name or normalize_command_name(self.source.__name__)

    @property
    def aliases(self) -> tuple[str, ...]:
        return self.declaration.aliases

    @property
    def help(self) -> str:
        return inspect.cleandoc(inspect.getdoc(self.source) or "")


@overload
def command(func: CommandTarget, /) -> CommandTarget: ...


@overload
def command(
    *,
    name: str = "",
    aliases: tuple[str, ...] = (),
) -> Callable[[CommandTarget], CommandTarget]: ...


def command(
    func: CommandTarget | None = None,
    /,
    *,
    name: str = "",
    aliases: tuple[str, ...] = (),
) -> CommandTarget | Callable[[CommandTarget], CommandTarget]:
    """Declare a callable as a stack command."""

    def decorate(target: CommandTarget) -> CommandTarget:
        actual = _underlying_function(target)
        if _COMMAND in vars(actual):
            raise TypeError("a stack command may be declared only once")
        setattr(actual, _COMMAND, CommandDeclaration(name, aliases))
        return target

    return decorate(func) if func is not None else decorate


def declared_commands(component: object, /) -> Iterator[BoundCommand]:
    """Yield command declarations bound to a component instance."""
    for name, value in _declared_attributes(component):
        source = _underlying_function(value)
        declaration = getattr(source, _COMMAND, None) if callable(source) else None
        if declaration is None:
            continue
        if not isinstance(declaration, CommandDeclaration):
            raise TypeError(f"invalid command declaration on {name!r}")
        yield BoundCommand(_bound_method(component, name), source, declaration)


def _bound_method(component: object, name: str, kind: str = "command") -> CommandCallback:
    callback = getattr(component, name)
    if not inspect.ismethod(callback) or callback.__self__ is not component:
        raise TypeError(f"decorated {kind} {name!r} must be an instance method")
    return callback


def _declared_attributes(component: object) -> Iterator[tuple[str, object]]:
    seen: set[str] = set()
    for cls in type(component).__mro__:
        for name, value in vars(cls).items():
            if name in seen:
                continue
            seen.add(name)
            yield name, value


def _underlying_function(value: Any) -> Any:
    if isinstance(value, (staticmethod, classmethod)):
        value = value.__func__
    return inspect.unwrap(value) if callable(value) else value


#
# fmt
#


def _format_parser_syntax(syntax: ParserSyntax | None, parameter_name: str) -> str:
    """Render structured parser syntax as legacy command text."""
    match syntax:
        case NamedFields(names):
            return ",".join(names)
        case LiteralSyntax(values):
            return "|".join(values)
        case OmittedSyntax():
            return ""
        case ChoiceSyntax(alternatives):
            return "|".join(
                _format_parser_syntax(alternative, parameter_name) for alternative in alternatives
            )
        case None:
            return parameter_name


def _format_parameter(parameter: Parameter) -> str:
    """Render a compiled command parameter as legacy command text."""
    text = _format_parser_syntax(parameter.parser.syntax, parameter.name)
    if parameter.repeat:
        text += "..."
    if parameter.optional or parameter.nullable:
        text = f"[{text}]"
    return text


def format_command_form(name: str, parameters: Iterable[Parameter]) -> str:
    """Render a command form as legacy command text."""
    rendered = ",".join(_format_parameter(parameter) for parameter in parameters)
    return f"{name} {rendered}" if rendered else name
