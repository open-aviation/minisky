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
import re
from collections.abc import Callable, Iterable, Iterator, Mapping
from dataclasses import dataclass, replace
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

from minisky import quantities as q
from minisky import values as value_types
from minisky.identifiers import normalize_command_name
from minisky.result import Err, Ok, Result
from minisky.tools.convert import (
    txt2bool,
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
ValueT_co = TypeVar("ValueT_co", covariant=True)
MappedT = TypeVar("MappedT")
_COMMAND = "__minisky_command__"


#
# lexical grammar
#


@dataclass(frozen=True, slots=True)
class SourceSpan:
    """Half-open character range within a parser input string."""

    start: int
    end: int


@dataclass(frozen=True, slots=True)
class Spanned(Generic[ValueT_co]):
    """A parsed value and the source characters that produced it."""

    value: ValueT_co
    span: SourceSpan

    def map(self, value: MappedT) -> Spanned[MappedT]:
        return Spanned(value, self.span)


@dataclass(frozen=True, slots=True)
class ArgumentIssue:
    """A parse error with optional source location context."""

    message: str
    span: SourceSpan | None = None

    @classmethod
    def expected(
        cls, expected: str, actual: object, span: SourceSpan | None = None
    ) -> ArgumentIssue:
        return cls(f"expected {expected}, but got {actual}", span)

    def with_span(self, span: SourceSpan) -> ArgumentIssue:
        return replace(self, span=span)

    def at_argument(self, name: str, fallback: SourceSpan) -> ArgumentIssue:
        return replace(
            self,
            message=f"argument `{name}`: {self.message}",
            span=self.span or fallback,
        )

    def __str__(self) -> str:
        return self.message


ParseResult: TypeAlias = Result[Spanned[ValueT_co], ArgumentIssue]


@dataclass(frozen=True, slots=True)
class CommandField:
    """One lexical field; `None` is the grammar's omitted-field variant."""

    value: str | None
    span: SourceSpan


_FieldResult: TypeAlias = Result[CommandField | None, ArgumentIssue]


@dataclass(slots=True)
class CommandCursor:
    """Transactional cursor over immutable command text.

    Only `pos` mutates. Parser failures restore it to the parser's entry point.
    `end` bounds a cursor to a framed command without copying text.
    """

    text: str
    pos: int = 0
    end: int | None = None

    def __post_init__(self) -> None:
        if self.end is None:
            self.end = len(self.text)
        if not 0 <= self.pos <= self.end <= len(self.text):
            raise ValueError("invalid command cursor bounds")

    @property
    def remaining(self) -> str:
        assert self.end is not None
        return self.text[self.pos : self.end]

    @property
    def at_end(self) -> bool:
        assert self.end is not None
        index = self._skip_spaces(self.pos)
        return index >= self.end

    def checkpoint(self) -> int:
        return self.pos

    def restore(self, checkpoint: int) -> None:
        assert self.end is not None
        if not 0 <= checkpoint <= self.end:
            raise ValueError("checkpoint outside cursor bounds")
        self.pos = checkpoint

    def peek_field(self) -> _FieldResult:
        checkpoint = self.pos
        result = self.next_field()
        self.pos = checkpoint
        return result

    def next_field(self) -> _FieldResult:
        """Consume a field, or return `None` at the cursor's framed end."""
        return self._next_field(stop_at_semicolon=False)

    def _next_field(self, *, stop_at_semicolon: bool) -> _FieldResult:
        assert self.end is not None
        text = self.text
        index = self._skip_spaces(self.pos)

        if index >= self.end:
            self.pos = index
            return Ok(None)
        if text[index] == ";":
            if stop_at_semicolon:
                self.pos = index
                return Ok(None)
            return Err(
                ArgumentIssue.expected("a command field", text[index], SourceSpan(index, index + 1))
            )

        if text[index] == ",":
            start = index
            self.pos = self._skip_spaces(index + 1)
            return Ok(CommandField(None, SourceSpan(start, start + 1)))

        token_start = index
        quote = text[index] if text[index] in ("'", '"') else None
        if quote is not None:
            value_start = index + 1
            close = text.find(quote, value_start, self.end)
            if close < 0:
                return Err(
                    ArgumentIssue.expected(
                        f"a closing {quote} quote",
                        "end of input",
                        SourceSpan(token_start, self.end),
                    )
                )
            value = text[value_start:close]
            index = close + 1
            token_end = index
            if (
                index < self.end
                and not text[index].isspace()
                and text[index] != ","
                and not (stop_at_semicolon and text[index] == ";")
            ):
                return Err(
                    ArgumentIssue.expected(
                        "a separator after the quoted argument",
                        text[index],
                        SourceSpan(token_start, index + 1),
                    )
                )
        else:
            value_start = index
            while index < self.end and not text[index].isspace() and text[index] not in ",;":
                # Apostrophes inside a bare token are part of legacy DMS positions,
                # for example N52'14'12'. Quotes only delimit an argument when they
                # appear at its start.
                index += 1
            value = text[value_start:index]
            token_end = index

        index = self._skip_spaces(index)
        if index < self.end and text[index] == ",":
            index = self._skip_spaces(index + 1)
        self.pos = index
        return Ok(CommandField(value, SourceSpan(token_start, token_end)))

    def next_value(self, expected: str) -> Result[Spanned[str], ArgumentIssue]:
        """Consume a non-omitted field value with a semantic expectation."""
        result = self.next_field()
        if isinstance(result, Err):
            return result
        field = result.ok()
        if field is None:
            span = SourceSpan(self.pos, self.pos)
            return Err(ArgumentIssue.expected(expected, "end of input", span))
        if field.value is None:
            return Err(ArgumentIssue.expected(expected, "an omitted field", field.span))
        return Ok(Spanned(field.value, field.span))

    def take_text(self) -> ParseResult[str]:
        """Consume verbatim text through this command's batch boundary."""
        start = self.pos
        while True:
            result = self._next_field(stop_at_semicolon=True)
            if isinstance(result, Err):
                return result
            if result.ok() is None:
                end = self.pos
                return Ok(Spanned(self.text[start:end], SourceSpan(start, end)))

    def next_command(self) -> Result[Spanned[str] | None, ArgumentIssue]:
        """Consume the next non-empty semicolon-delimited command."""
        assert self.end is not None
        initial = self.pos
        text = self.text

        while True:
            index = self._skip_spaces(self.pos)
            while index < self.end and text[index] == ";":
                index = self._skip_spaces(index + 1)
            self.pos = index
            if index >= self.end:
                return Ok(None)

            command_start = index
            while True:
                result = self._next_field(stop_at_semicolon=True)
                if isinstance(result, Err):
                    self.pos = initial
                    return result
                if result.ok() is None:
                    break

            command_end = self.pos
            while command_end > command_start and text[command_end - 1].isspace():
                command_end -= 1
            if self.pos < self.end and text[self.pos] == ";":
                self.pos += 1
            if command_end > command_start:
                span = SourceSpan(command_start, command_end)
                return Ok(Spanned(text[command_start:command_end], span))

    def _skip_spaces(self, index: int) -> int:
        assert self.end is not None
        text = self.text
        while index < self.end and text[index].isspace():
            index += 1
        return index


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


ParseFunction: TypeAlias = Callable[[CommandParseContext, CommandCursor], ParseResult[ValueT_co]]


@dataclass(frozen=True, slots=True)
class CmdParser(Generic[ValueT_co]):
    """Connect a Python value type to command parsing and usage syntax.

    This is the metadata inside `Annotated`, which describes what a callback
    receives. Use `fields()` for a multi-field value's usage names and the
    syntax helpers when generated usage needs more detail than a parameter name.

    Custom parsers may advance only the supplied cursor. `CmdParser` restores the
    cursor automatically when they return `Err`.
    """

    func: ParseFunction[ValueT_co]
    """A transactional cursor-to-value parser."""
    syntax: ParserSyntax | None = None
    # NOTE: not collapsing it to a string here
    # TODO: make zensical macros consume this to build a rich display
    # and serialise it to tangram

    @classmethod
    def fields(cls, func: ParseFunction[ValueT_co], names: tuple[str, ...]) -> CmdParser[ValueT_co]:
        return cls(func, NamedFields(names))

    @classmethod
    def omitted(cls, func: ParseFunction[ValueT_co]) -> CmdParser[ValueT_co]:
        return cls(func, OmittedSyntax())

    @staticmethod
    def value(
        converter: Callable[[str], MappedT], expected: str, *, field: str | None = None
    ) -> CmdParser[MappedT]:
        def parse(_context: CommandParseContext, cursor: CommandCursor) -> ParseResult[MappedT]:
            return parse_field(cursor, converter, expected)

        syntax = NamedFields((field,)) if field is not None else None
        return CmdParser(parse, syntax)

    @staticmethod
    def keywords(mapping: Mapping[str, MappedT], expected: str) -> CmdParser[MappedT]:
        normalized = {key.upper(): value for key, value in mapping.items()}
        if len(normalized) != len(mapping):
            raise ValueError("command parser keywords must be unique ignoring case")

        def parse(context: CommandParseContext, cursor: CommandCursor) -> ParseResult[MappedT]:
            result = _KEYWORD_PARSER(context, cursor)
            if isinstance(result, Err):
                return result
            token = result.ok()
            if token.value not in normalized:
                return Err(ArgumentIssue.expected(expected, token.value, token.span))
            return Ok(token.map(normalized[token.value]))

        return CmdParser(parse, LiteralSyntax(tuple(normalized)))

    def __call__(
        self, context: CommandParseContext, cursor: CommandCursor
    ) -> ParseResult[ValueT_co]:
        checkpoint = cursor.checkpoint()
        result = self.func(context, cursor)
        if isinstance(result, Err):
            cursor.restore(checkpoint)
        return result


#
# primitives
#


FiniteFloat: TypeAlias = IsFinite[float]
NonNegativeFiniteFloat: TypeAlias = Annotated[FiniteFloat, Ge(0)]
PositiveFiniteFloat: TypeAlias = Annotated[FiniteFloat, Gt(0)]


def parse_field(
    cursor: CommandCursor, converter: Callable[[str], MappedT], expected: str
) -> ParseResult[MappedT]:
    """Consume a field and convert its text into a semantic value."""
    result = cursor.next_value(expected)
    if isinstance(result, Err):
        return result
    token = result.ok()
    try:
        value = converter(token.value)
    except ValueError:
        return Err(ArgumentIssue.expected(expected, token.value, token.span))
    return Ok(token.map(value))


def _convert_value(
    value: str, converter: Callable[[str], MappedT], expected: str
) -> Result[MappedT, ArgumentIssue]:
    try:
        return Ok(converter(value))
    except ValueError:
        return Err(ArgumentIssue.expected(expected, value))


def parse_token(_context: CommandParseContext, cursor: CommandCursor) -> ParseResult[str]:
    result = cursor.next_value("a non-empty argument")
    if isinstance(result, Err):
        return result
    token = result.ok()
    if not token.value:
        return Err(ArgumentIssue.expected("a non-empty argument", "empty input", token.span))
    return Ok(token)


_TOKEN_PARSER = CmdParser(parse_token)
Token = Annotated[str, _TOKEN_PARSER]
"""A non-empty BlueSky command field parsed as text.

Normal tokenization applies: spaces or commas terminate bare fields and surrounding
quotes are removed. Unlike [`Text`][minisky.command.Text], `Token` never consumes
the rest of the command.
"""


def parse_keyword(context: CommandParseContext, cursor: CommandCursor) -> ParseResult[str]:
    if isinstance(result := _TOKEN_PARSER(context, cursor), Err):
        return result
    token = result.ok()
    return Ok(token.map(token.value.upper()))


_KEYWORD_PARSER = CmdParser(parse_keyword)
Keyword = Annotated[str, _KEYWORD_PARSER]
"""A [`Token`][minisky.command.Token] normalized to upper case.

Use this for case-insensitive command keywords whose value remains data.
Python `Literal[...]` annotations use
[`LiteralSyntax`][minisky.command.LiteralSyntax] when the keyword is grammar.
"""


@dataclass(frozen=True, slots=True)
class OmittedField:
    """Sentinel indicating a required empty comma field was present."""


_OMITTED_FIELD = OmittedField()


def parse_omitted_field(
    _context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[OmittedField]:
    result = cursor.next_field()
    if isinstance(result, Err):
        return result
    token = result.ok()
    if token is None or token.value is not None:
        span = token.span if token is not None else SourceSpan(cursor.pos, cursor.pos)
        actual = token.value if token is not None else "end of input"
        return Err(ArgumentIssue.expected("an omitted comma field", actual, span))
    return Ok(Spanned(_OMITTED_FIELD, token.span))


Omitted = Annotated[OmittedField, CmdParser.omitted(parse_omitted_field)]
"""A required empty positional field, such as the middle field in `CMD A,,B`."""


def parse_text(_context: CommandParseContext, cursor: CommandCursor) -> ParseResult[str]:
    return cursor.take_text()


Text = Annotated[str, CmdParser(parse_text)]
"""The complete remaining framed-command text, consumed verbatim.

This intentionally preserves BlueSky's `string` terminal for nested commands such
as ECHO and DELAY. Unlike [`Token`][minisky.command.Token], quotes are not stripped
because no further tokenization occurs.
"""


_INT_PARSER = CmdParser.value(int, "an integer")
_FLOAT_PARSER = CmdParser.value(float, "a number")
_ON_OFF_PARSER = CmdParser.value(txt2bool, "ON or OFF")
OnOff = Annotated[bool, _ON_OFF_PARSER]


_UNSIGNED_NUMBER_RE = r"(?:\d+(?:\.\d*)?|\.\d+)"
_NUMBER_RE = rf"[+-]?{_UNSIGNED_NUMBER_RE}"
_ALTITUDE_VALUE = re.compile(rf"(?P<value>{_NUMBER_RE})(?P<unit>FT|M)?", re.IGNORECASE)
_MSL_ALTITUDE = re.compile(rf"(?P<value>{_NUMBER_RE})(?P<unit>FT|M)?\[MSL\]", re.IGNORECASE)
_FLIGHT_LEVEL = re.compile(r"FL(?P<level>\d+)", re.IGNORECASE)


def _vertical_metres(value: str, unit: str | None) -> float:
    return float(value) if (unit or "FT").upper() == "M" else q.ft_to_m(float(value))


def parse_pressure_altitude_value(value: str) -> value_types.StdPressureAltM:
    if match := _FLIGHT_LEVEL.fullmatch(value):
        return value_types.StdPressureAltM(q.ft_to_m(100.0 * int(match.group("level"))))
    if (match := _ALTITUDE_VALUE.fullmatch(value)) is None:
        raise ValueError
    return value_types.StdPressureAltM(_vertical_metres(match.group("value"), match.group("unit")))


def _parse_msl_altitude(value: str) -> value_types.MslAltM:
    if (match := _MSL_ALTITUDE.fullmatch(value)) is None:
        raise ValueError
    return value_types.MslAltM(_vertical_metres(match.group("value"), match.group("unit")))


# Unqualified altitude syntax intentionally means standard-pressure altitude.
# AGL stays out until ground elevation exists; relabeling pressure altitude as
# height would manufacture a datum the simulation does not have.


def _parse_vertical_distance(value: str) -> q.VerticalDistanceM[float]:
    if (match := _ALTITUDE_VALUE.fullmatch(value)) is None:
        raise ValueError
    return _vertical_metres(match.group("value"), match.group("unit"))


VerticalDistanceM = Annotated[
    q.VerticalDistanceM[float],
    CmdParser.value(
        _parse_vertical_distance,
        "a vertical distance such as 1000, 1000FT, or 304.8M",
    ),
]

def parse_speed_value(
    value: str,
) -> Result[q.MachNumber[float] | q.CalibratedAirspeedMps[float], ArgumentIssue]:
    return _convert_value(value, txt2spd, "a speed")


# TODO(abraham): #40 must split calibrated airspeed and Mach at runtime.
SpeedMpsOrMach = Annotated[
    q.MachNumber[float] | q.CalibratedAirspeedMps[float],
    CmdParser.value(txt2spd, "a speed"),
]
VspdMps = Annotated[q.VerticalRateMps[float], CmdParser.value(txt2vs, "a vertical speed")]
TimeS = Annotated[q.DurationS[float], CmdParser.value(txt2tim, "a time")]
SimTimeS = Annotated[q.SimulationTimeS[float], CmdParser.value(txt2tim, "a time")]


#
# domain-specific
#


_TRUE_HEADING = re.compile(rf"(?P<value>{_NUMBER_RE})T?", re.IGNORECASE)
_MAGNETIC_HEADING = re.compile(rf"(?P<value>{_NUMBER_RE})M", re.IGNORECASE)
_GROUND_TRACK = re.compile(rf"(?P<value>{_NUMBER_RE})TRK", re.IGNORECASE)


def _matched_number(pattern: re.Pattern[str], value: str) -> float:
    if (match := pattern.fullmatch(value)) is None:
        raise ValueError
    return float(match.group("value"))


def _parse_true_heading(value: str) -> value_types.TrueHeadingDeg:
    return value_types.TrueHeadingDeg(_matched_number(_TRUE_HEADING, value))


def _parse_magnetic_heading(value: str) -> value_types.MagneticHeadingDeg:
    return value_types.MagneticHeadingDeg(_matched_number(_MAGNETIC_HEADING, value))


def _parse_heading(value: str) -> value_types.TrueHeadingDeg | value_types.MagneticHeadingDeg:
    return _parse_magnetic_heading(value) if value.upper().endswith("M") else _parse_true_heading(value)


HeadingDeg = Annotated[
    value_types.TrueHeadingDeg | value_types.MagneticHeadingDeg,
    CmdParser.value(_parse_heading, "a heading"),
]
"""Heading syntax preserving whether the input refers to true or magnetic north."""


def _parse_ground_track(value: str) -> value_types.GroundTrackDeg:
    return value_types.GroundTrackDeg(_matched_number(_GROUND_TRACK, value))


@dataclass(frozen=True, slots=True)
class RunwayHeadingRequest:
    """Preserve a source-level `*` request until the command can resolve it.

    For example, in `CRE KLM1,A320,EHAM,RWY18L,*,0,250`, `*` means "use the
    heading of the runway parsed in the previous argument". Unlike bluesky
    (which internally stores the RWY18L in its parser), minisky directly returns
    the sentinel to avoid a parser-global cross-argument state.
    """


_RUNWAY_HEADING = RunwayHeadingRequest()
UseRunwayHeading = Annotated[
    RunwayHeadingRequest,
    CmdParser.keywords({"*": _RUNWAY_HEADING}, "*"),
]
"""The `*` heading form, preserved as an explicit callback value.

Only commands that can derive a heading from another argument should include this
in their annotation, for example `HeadingDeg | UseRunwayHeading`.
"""


def _aircraft_index(context: CommandParseContext, callsign: str) -> Result[int, ArgumentIssue]:
    index = context.traffic.idx(callsign)
    if index is None:
        return Err(ArgumentIssue.expected("an existing aircraft", callsign))
    return Ok(index)


def parse_aircraft(context: CommandParseContext, cursor: CommandCursor) -> ParseResult[int]:
    result = cursor.next_value("an aircraft")
    if isinstance(result, Err):
        return result
    token = result.ok()
    callsign = token.value.upper()

    if context.traffic.idx(callsign) is None and callsign in context.traffic.groups:
        return Err(ArgumentIssue.expected("an aircraft", f"group {callsign}", token.span))

    if isinstance(index := _aircraft_index(context, callsign), Err):
        return Err(index.err().with_span(token.span))
    return Ok(token.map(index.ok()))


AcId = Annotated[int, CmdParser(parse_aircraft)]
"""An existing aircraft callsign resolved to its traffic-array index."""


def parse_aircraft_selection(
    context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[np.ndarray[Any, Any]]:
    result = cursor.next_value("an existing aircraft or group")
    if isinstance(result, Err):
        return result
    token = result.ok()
    name = token.value.upper()

    if name in {"*", "ALL"} or name in context.traffic.groups:
        if isinstance(selection := context.traffic.groups.listgroup(name), Err):
            return Err(
                ArgumentIssue.expected("an existing aircraft or group", selection.err(), token.span)
            )
        return Ok(token.map(selection.ok()))

    if isinstance(index := _aircraft_index(context, name), Err):
        return Err(ArgumentIssue.expected("an existing aircraft or group", name, token.span))
    return Ok(token.map(np.asarray([index.ok()], dtype=int)))


AcIdSelection = Annotated[np.ndarray[Any, Any], CmdParser(parse_aircraft_selection)]
"""An aircraft, a traffic group, or `*`/`ALL`, resolved to traffic indices."""


def aircraft_indices(
    selections: Iterable[np.ndarray[Any, Any]],
) -> np.ndarray[Any, Any]:
    """Flatten aircraft and group selections into an index array."""
    indices = (int(index) for selection in selections for index in selection)
    return np.fromiter(indices, dtype=int)


@dataclass(frozen=True, slots=True)
class NamedWaypoint:
    """A waypoint expression that should be resolved by name."""

    name: str


@dataclass(frozen=True, slots=True)
class CoordinateWaypoint:
    """A waypoint expressed directly as latitude and longitude."""

    coordinates: value_types.LatLonDegrees
    source: str


WaypointSpec: TypeAlias = NamedWaypoint | CoordinateWaypoint
"""Structured waypoint syntax consumed by route and origin/destination commands."""


def parse_waypoint(
    _context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[WaypointSpec]:
    first = cursor.next_value("a waypoint")
    if isinstance(first, Err):
        return first
    token = first.ok()
    if not token.value:
        return Err(ArgumentIssue.expected("a waypoint", "empty input", token.span))

    name = token.value.upper()
    if islat(name):
        longitude_result = cursor.next_value("a longitude after the latitude")
        if isinstance(longitude_result, Err):
            return longitude_result
        longitude = longitude_result.ok()
        if not longitude.value:
            return Err(
                ArgumentIssue.expected(
                    "a longitude after the latitude", "empty input", longitude.span
                )
            )
        span = SourceSpan(token.span.start, longitude.span.end)
        try:
            coordinates = value_types.LatLonDegrees(txt2lat(name), txt2lon(longitude.value))
        except ValueError:
            return Err(
                ArgumentIssue.expected(
                    "a latitude and longitude", f"{name},{longitude.value}", span
                )
            )
        return Ok(Spanned(CoordinateWaypoint(coordinates, f"{name},{longitude.value}"), span))

    checkpoint = cursor.checkpoint()
    runway_result = cursor.next_field()
    if isinstance(runway_result, Err):
        return runway_result
    runway = runway_result.ok()
    if runway is not None and runway.value is not None and runway.value.upper().startswith("RW"):
        span = SourceSpan(token.span.start, runway.span.end)
        return Ok(Spanned(NamedWaypoint(f"{name}/{runway.value.upper()}"), span))
    cursor.restore(checkpoint)
    return Ok(Spanned(NamedWaypoint(name), token.span))


_WAYPOINT_PARSER = CmdParser(parse_waypoint)
Wpt = Annotated[WaypointSpec, _WAYPOINT_PARSER]
"""A named or coordinate waypoint expression, preserved as structured syntax."""


@dataclass(frozen=True, slots=True)
class RunwayPosition:
    """Resolved runway coordinates and their runway heading."""

    coordinates: value_types.LatLonDegrees
    runway_heading: q.TrueHeadingDegrees[float]


ResolvedPosition: TypeAlias = value_types.LatLonDegrees | RunwayPosition


def _resolve_named_position(
    context: CommandParseContext, name: str, span: SourceSpan
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
        return Ok(RunwayPosition(value_types.LatLonDegrees(float(lat), float(lon)), float(heading)))

    if name in navigation.aptid:
        index = navigation.aptid.index(name)
        return Ok(value_types.LatLonDegrees(float(navigation.aptlat[index]), float(navigation.aptlon[index])))

    occurrences = navigation.wpid.count(name)
    if occurrences > 1:
        return Err(
            ArgumentIssue.expected("an unambiguous waypoint id or explicit coordinates", name, span)
        )
    if occurrences == 1:
        index = navigation.wpid.index(name)
        return Ok(value_types.LatLonDegrees(float(navigation.wplat[index]), float(navigation.wplon[index])))

    return Err(ArgumentIssue.expected("a waypoint, airport, runway, or aircraft id", name, span))


def parse_resolved_position(
    context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[ResolvedPosition]:
    if isinstance(result := _WAYPOINT_PARSER(context, cursor), Err):
        return result
    parsed = result.ok()
    waypoint = parsed.value

    if isinstance(waypoint, CoordinateWaypoint):
        return Ok(parsed.map(waypoint.coordinates))

    index = context.traffic.idx(waypoint.name)
    if index is not None:
        coordinates = value_types.LatLonDegrees(
            float(context.traffic.lat[index]), float(context.traffic.lon[index])
        )
        return Ok(parsed.map(coordinates))

    if isinstance(resolved := _resolve_named_position(context, waypoint.name, parsed.span), Err):
        return resolved
    return Ok(parsed.map(resolved.ok()))


_RESOLVED_POSITION_PARSER = CmdParser(parse_resolved_position)
ResolvedPositionArg = Annotated[ResolvedPosition, _RESOLVED_POSITION_PARSER]
"""A position expression resolved against navigation data and traffic.

Runways retain their heading in [`RunwayPosition`][minisky.command.RunwayPosition];
other positions become [`LatLonDegrees`][minisky.values.LatLonDegrees]. Ambiguous
waypoint identifiers are rejected because this grammar has no geographic reference.
"""


def parse_lat_lon(
    context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[value_types.LatLonDegrees]:
    if isinstance(result := _RESOLVED_POSITION_PARSER(context, cursor), Err):
        return result
    parsed = result.ok()
    coordinates = (
        parsed.value.coordinates if isinstance(parsed.value, RunwayPosition) else parsed.value
    )
    return Ok(parsed.map(coordinates))


LatLonDeg = Annotated[value_types.LatLonDegrees, CmdParser(parse_lat_lon)]
"""A resolved position reduced to latitude and longitude degrees.

It accepts the same BlueSky position expressions as
[`ResolvedPositionArg`][minisky.command.ResolvedPositionArg] but intentionally
discards runway-specific heading after resolution.
"""


_VALUE_PARSERS: dict[Any, CmdParser[Any]] = {
    bool: _ON_OFF_PARSER,
    int: _INT_PARSER,
    float: _FLOAT_PARSER,
    value_types.StdPressureAltM: CmdParser.value(
        parse_pressure_altitude_value,
        "pressure altitude such as FL100, 10000, 10000FT, or 3048M",
        field="pressure altitude",
    ),
    value_types.MslAltM: CmdParser.value(
        _parse_msl_altitude,
        "MSL altitude such as 10000FT[MSL] or 3048M[MSL]",
        field="MSL altitude",
    ),
    value_types.TrueHeadingDeg: CmdParser.value(
        _parse_true_heading, "a true heading", field="true heading"
    ),
    value_types.MagneticHeadingDeg: CmdParser.value(
        _parse_magnetic_heading, "a magnetic heading", field="magnetic heading"
    ),
    value_types.GroundTrackDeg: CmdParser.value(
        _parse_ground_track, "a ground track such as 090TRK", field="ground track"
    ),
}


#
# parameter contracts
#


_Constraint: TypeAlias = Gt | Ge | Lt | Le | MinLen | MaxLen | Predicate


@dataclass(frozen=True, slots=True)
class _ArgumentType:
    """Compiled conversion and validation rules for an annotation."""

    parser: CmdParser[Any]
    constraints: tuple[_Constraint, ...]
    nullable: bool = False

    def parse(self, context: CommandParseContext, cursor: CommandCursor) -> ParseResult[Any]:
        checkpoint = cursor.checkpoint()
        if isinstance(parsed := self.parser(context, cursor), Err):
            return parsed
        value = parsed.ok()
        if isinstance(validation := _validate_constraints(value.value, self.constraints), Err):
            cursor.restore(checkpoint)
            return Err(validation.err().with_span(value.span))
        return parsed


@dataclass(frozen=True, slots=True)
class _ArgumentValues:
    """Values produced for a callback parameter."""

    values: tuple[Any, ...]


@dataclass(frozen=True, slots=True)
class Parameter:
    """Executable command contract compiled from a callback parameter.

    Parsing mutates only `CommandCursor.pos`.
    """

    name: str
    argument_type: _ArgumentType
    default: object = inspect.Parameter.empty
    repeat: bool = False

    def parse(
        self, context: CommandParseContext, cursor: CommandCursor
    ) -> Result[_ArgumentValues, ArgumentIssue]:
        if self.repeat:
            return self._parse_repeated(context, cursor)
        if cursor.at_end:
            if self.default is not inspect.Parameter.empty:
                return Ok(_ArgumentValues((self.default,)))
            return Err(self._missing(SourceSpan(cursor.pos, cursor.pos), "end of input"))

        if self.argument_type.nullable or self.default is not inspect.Parameter.empty:
            peeked = cursor.peek_field()
            if isinstance(peeked, Err):
                return peeked
            field = peeked.ok()
            if field is not None and field.value is None:
                cursor.next_field()
                value = None if self.argument_type.nullable else self.default
                return Ok(_ArgumentValues((value,)))
        return self._parse_one(context, cursor)

    def _parse_repeated(
        self, context: CommandParseContext, cursor: CommandCursor
    ) -> Result[_ArgumentValues, ArgumentIssue]:
        values: list[Any] = []
        while not cursor.at_end:
            result = self._parse_one(context, cursor)
            if isinstance(result, Err):
                return result
            values.extend(result.ok().values)
        return Ok(_ArgumentValues(tuple(values)))

    def _missing(self, span: SourceSpan, actual: str) -> ArgumentIssue:
        return ArgumentIssue.expected("a value", actual, span).at_argument(self.name, span)

    def _parse_one(
        self, context: CommandParseContext, cursor: CommandCursor
    ) -> Result[_ArgumentValues, ArgumentIssue]:
        start = cursor.checkpoint()
        if isinstance(parsed_result := self.argument_type.parse(context, cursor), Err):
            issue = parsed_result.err()
            fallback = issue.span or SourceSpan(start, start + 1)
            return Err(issue.at_argument(self.name, fallback))
        if cursor.pos <= start:
            raise TypeError(f"parser {self.argument_type.parser!r} consumed no input")
        return Ok(_ArgumentValues((parsed_result.ok().value,)))

    @property
    def optional(self) -> bool:
        return self.repeat or self.default is not inspect.Parameter.empty

    @property
    def parser(self) -> CmdParser[Any]:
        return self.argument_type.parser

    @property
    def nullable(self) -> bool:
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
        return _TOKEN_PARSER
    parser = _annotated_parser(annotation)
    if parser is not None:
        return parser
    if get_origin(annotation) is Annotated:
        return _parser_for(get_args(annotation)[0])
    return _VALUE_PARSERS.get(annotation)


def _choice_parser(alternatives: tuple[_ArgumentType, ...]) -> CmdParser[Any]:
    def parse(context: CommandParseContext, cursor: CommandCursor) -> ParseResult[Any]:
        failure: ArgumentIssue | None = None
        for alternative in alternatives:
            result = alternative.parse(context, cursor)
            if isinstance(result, Ok):
                return result
            failure = result.err()
        assert failure is not None
        return Err(failure)

    syntax = ChoiceSyntax(tuple(alternative.parser.syntax for alternative in alternatives))
    return CmdParser(parse, syntax)


def _literal_parser(annotation: Any) -> CmdParser[str]:
    values = get_args(annotation)
    if not values or any(not isinstance(value, str) for value in values):
        raise TypeError("stack command Literal values must be strings")
    literals = tuple(value.upper() for value in values)
    try:
        return CmdParser.keywords(dict(zip(values, literals, strict=True)), " or ".join(literals))
    except ValueError:
        raise TypeError("stack command Literal repeats a case-insensitive value") from None


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
