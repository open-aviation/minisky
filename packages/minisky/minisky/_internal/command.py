"""Typed stack-command declarations and parsers.

Command grammar. Alphabetic terminals are case-insensitive; semantic names such
as `aircraft` are resolved against runtime state after syntax parsing.

```text
batch              = [ command ], { optional-space, ";", optional-space, [ command ] } ;
command            = field, { separator, field } ;
field              = argument | omitted-field ;
separator          = spaces | optional-space, ",", optional-space ;
omitted-field      = /* empty field created by a comma, e.g. A,,B */ ;
argument           = bare | single-quoted | double-quoted ;
bare               = bare-char, { bare-char } ;
single-quoted      = "'", { any-char-except-single-quote }, "'" ;
double-quoted      = '"', { any-char-except-double-quote }, '"' ;
bare-char          = any-char-except-space-comma-semicolon ;
spaces             = space, { space } ;
optional-space     = { space } ;

digit              = "0" | "1" | "2" | "3" | "4" | "5" | "6" | "7" | "8" | "9" ;
digits             = digit, { digit } ;
sign               = "+" | "-" ;
unsigned-number    = digits, [ ".", { digit } ] | ".", digits ;
number             = [ sign ], unsigned-number ;

boolean            = "TRUE" | "YES" | "Y" | "1" | "ON"
                   | "FALSE" | "NO" | "N" | "0" | "OFF" ;
time               = number | [ digits ], ":", number
                   | [ digits ], ":", [ digits ], ":", number ;
altitude           = pressure-altitude | msl-altitude ;
pressure-altitude  = flight-level | standard-altitude ;
standard-altitude  = altitude-value, "[STD]" ;
msl-altitude       = altitude-value, "[MSL]" ;
flight-level       = "FL", digits ;                  # hundreds of feet, standard pressure
altitude-value     = number, altitude-unit ;
altitude-unit      = "FT" | "M" ;
distance           = number, distance-unit ;
distance-unit      = "M" | "KM" | "NM" | "FT" ;

speed              = mach | speed-value ;
selected-airspeed  = mach | cas ;
unit-speed         = number, speed-unit ;
cas                = unsigned-number, speed-unit, "[CAS]" ;
tas                = unsigned-number, speed-unit, "[TAS]" ;
gs                 = unsigned-number, speed-unit, "[GS]" ;
speed-value        = cas | tas | gs ;
speed-unit         = "MPS" | "FT/MIN" | "KT" | "KMH" ;
mach               = "M", (digits, ".", digits | ".", digits) ;

heading            = true-heading | magnetic-heading ;
true-heading       = number, [ "T" ] ;
magnetic-heading   = number, "M" ;
ground-track       = number, "TRK" ;
runway-heading     = "*" ;

coordinate         = latitude, separator, longitude ;
latitude           = number | [ "N" | "S" ], dms-angle ;
longitude          = number | [ "E" | "W" ], dms-angle ;
dms-angle          = unsigned-number, { dms-mark, unsigned-number }, [ dms-mark ] ;
dms-mark           = "'" | '"' | "°" ;
waypoint           = coordinate | name | name, separator, runway ;
runway             = "RW", [ "Y" ], name ;
aircraft           = name ;                 # must resolve to an aircraft
aircraft-selection = "*" | "ALL" | name ;   # name may resolve to aircraft/group
name               = bare ;
```
"""

from __future__ import annotations

import inspect
import re
from collections.abc import Callable, Iterable, Iterator, Mapping
from dataclasses import dataclass, field, replace
from types import UnionType
from typing import (
    TYPE_CHECKING,
    Annotated,
    Any,
    Generic,
    Literal,
    NamedTuple,
    TypeAlias,
    TypeVar,
    Union,
    get_args,
    get_origin,
    get_type_hints,
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
from minisky import types as t
from minisky._internal.convert import (
    txt2lat,
    txt2lon,
    txt2tim,
)
from minisky._internal.identifiers import normalize_command_name
from minisky._internal.position import islat
from minisky._internal.result import Err, Ok, Result

if TYPE_CHECKING:
    from minisky._internal.navigation import Navdatabase
    from minisky._internal.stack import Command
    from minisky._internal.traffic import Traffic

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
class _LexicalField:
    """One lexical field; `None` is the grammar's omitted-field variant."""

    value: str | None
    span: SourceSpan


_FieldResult: TypeAlias = Result[_LexicalField | None, ArgumentIssue]


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
            return Ok(_LexicalField(None, SourceSpan(start, start + 1)))

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
        return Ok(_LexicalField(value, SourceSpan(token_start, token_end)))

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
# parser inputs
#


@dataclass(frozen=True, slots=True)
class CommandParseContext:
    """Read-only runtime services available while parsing a command value."""

    traffic: Traffic
    navigation: Navdatabase


@dataclass(frozen=True, slots=True)
class CommandField:
    """Metadata and compiled syntax for a lexical command field."""

    kind: Literal["field"] = field(init=False, default="field", repr=False)
    name: str | None = None
    examples: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class _RecordInput:
    kind: Literal["record"] = field(init=False, default="record", repr=False)
    name: str
    fields: tuple[CommandField, ...]


@dataclass(frozen=True, slots=True)
class _LiteralInput:
    kind: Literal["literal"] = field(init=False, default="literal", repr=False)
    values: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class _BooleanInput:
    kind: Literal["boolean"] = field(init=False, default="boolean", repr=False)
    true: tuple[str, ...]
    false: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class _OmittedInput:
    kind: Literal["omitted"] = field(init=False, default="omitted", repr=False)


@dataclass(frozen=True, slots=True)
class _TextInput:
    kind: Literal["text"] = field(init=False, default="text", repr=False)
    examples: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class _ChoiceInput:
    kind: Literal["choice"] = field(init=False, default="choice", repr=False)
    alternatives: tuple[CommandInput, ...]


CommandInput: TypeAlias = (
    CommandField
    | _RecordInput
    | _LiteralInput
    | _BooleanInput
    | _OmittedInput
    | _TextInput
    | _ChoiceInput
)
"""Inspectable command grammar shared by runtime parsing and static schemas."""


def _join_examples(values: tuple[str, ...]) -> str:
    if len(values) == 1:
        return values[0]
    if len(values) == 2:
        return f"{values[0]} or {values[1]}"
    return f"{', '.join(values[:-1])}, or {values[-1]}"


def _input_expectation(name: str | None, examples: tuple[str, ...]) -> str:
    if not name:
        return f"a value such as {_join_examples(examples)}" if examples else "a value"
    if examples:
        return f"{name} such as {_join_examples(examples)}"
    lowered = name.lower()
    if lowered.startswith(("a ", "an ", "the ")) or not name[0].islower():
        return name
    article = "an" if name[0] in "aeiou" else "a"
    return f"{article} {name}"


def _input_expected(value: CommandInput) -> str:
    match value:
        case CommandField(name=name, examples=examples):
            return _input_expectation(name, examples)
        case _RecordInput(name, fields):
            return f"{name} ({', '.join(field.name or 'value' for field in fields)})"
        case _LiteralInput(values):
            return _join_examples(values)
        case _BooleanInput(true, false):
            return f"a boolean ({_join_examples((*true, *false))})"
        case _OmittedInput():
            return "an omitted comma field"
        case _TextInput(examples):
            return _input_expectation("text", examples)
        case _ChoiceInput(alternatives):
            return " or ".join(_input_expected(alternative) for alternative in alternatives)


ParseFunction: TypeAlias = Callable[[CommandParseContext, CommandCursor], ParseResult[ValueT_co]]


@dataclass(frozen=True, slots=True)
class CmdParser(Generic[ValueT_co]):
    """Executable parser paired with its inspectable command-input contract."""

    func: ParseFunction[ValueT_co]
    input: CommandInput

    @classmethod
    def fields(cls, func: ParseFunction[ValueT_co], names: tuple[str, ...]) -> CmdParser[ValueT_co]:
        """Temporary compatibility for bespoke multi-field parsers."""
        return cls(func, _RecordInput("record", tuple(CommandField(name) for name in names)))

    @staticmethod
    def choices(mapping: Mapping[str, MappedT]) -> CmdParser[MappedT]:
        normalized = {key.upper(): value for key, value in mapping.items()}
        if len(normalized) != len(mapping):
            raise ValueError("command parser choices must be unique ignoring case")
        input_spec = _LiteralInput(tuple(normalized))

        def parse(context: CommandParseContext, cursor: CommandCursor) -> ParseResult[MappedT]:
            result = _KEYWORD_PARSER(context, cursor)
            if isinstance(result, Err):
                return result
            token = result.ok()
            if token.value not in normalized:
                return Err(
                    ArgumentIssue.expected(_input_expected(input_spec), token.value, token.span)
                )
            return Ok(token.map(normalized[token.value]))

        return CmdParser(parse, input_spec)

    def __call__(
        self, context: CommandParseContext, cursor: CommandCursor
    ) -> ParseResult[ValueT_co]:
        checkpoint = cursor.checkpoint()
        result = self.func(context, cursor)
        if isinstance(result, Err):
            cursor.restore(checkpoint)
        return result


@dataclass(frozen=True, slots=True)
class Converter(Generic[MappedT]):
    """Convert a lexical command field before semantic validation."""

    func: Callable[[str], MappedT]


def _converter_parser(
    converter: Converter[MappedT], input_spec: CommandField
) -> CmdParser[MappedT]:
    def parse(_context: CommandParseContext, cursor: CommandCursor) -> ParseResult[MappedT]:
        return _parse_field(cursor, converter.func, input_spec)

    return CmdParser(parse, input_spec)


#
# primitives
#


def _parse_field(
    cursor: CommandCursor, converter: Callable[[str], MappedT], input_spec: CommandField
) -> ParseResult[MappedT]:
    expected = _input_expected(input_spec)
    result = cursor.next_value(expected)
    if isinstance(result, Err):
        return result
    token = result.ok()
    try:
        value = converter(token.value)
    except ValueError:
        return Err(ArgumentIssue.expected(expected, token.value, token.span))
    return Ok(token.map(value))


def parse_field(
    cursor: CommandCursor, converter: Callable[[str], MappedT], expected: str
) -> ParseResult[MappedT]:
    """Compatibility helper for remaining bespoke field parsers."""
    return _parse_field(cursor, converter, CommandField(expected))


def _token_parser(input_spec: CommandField) -> CmdParser[str]:
    def parse(_context: CommandParseContext, cursor: CommandCursor) -> ParseResult[str]:
        result = cursor.next_value(_input_expected(input_spec))
        if isinstance(result, Err):
            return result
        token = result.ok()
        if not token.value:
            return Err(ArgumentIssue.expected("a non-empty argument", "empty input", token.span))
        return Ok(token)

    return CmdParser(parse, input_spec)


_TOKEN_PARSER = _token_parser(CommandField())
Token = Annotated[str, _TOKEN_PARSER]
"""A non-empty BlueSky command field parsed as text."""


def parse_keyword(context: CommandParseContext, cursor: CommandCursor) -> ParseResult[str]:
    if isinstance(result := _TOKEN_PARSER(context, cursor), Err):
        return result
    token = result.ok()
    return Ok(token.map(token.value.upper()))


_KEYWORD_PARSER = CmdParser(parse_keyword, CommandField())
Keyword = Annotated[str, _KEYWORD_PARSER]
"""A command token normalized to upper case."""


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


_OMITTED_INPUT = _OmittedInput()
Omitted = Annotated[OmittedField, CmdParser(parse_omitted_field, _OMITTED_INPUT)]


def parse_text(_context: CommandParseContext, cursor: CommandCursor) -> ParseResult[str]:
    return cursor.take_text()


_TEXT_INPUT = _TextInput()
Text = Annotated[str, CmdParser(parse_text, _TEXT_INPUT)]


_BOOLEAN_INPUT = _BooleanInput(
    true=("TRUE", "YES", "Y", "1", "ON"),
    false=("FALSE", "NO", "N", "0", "OFF"),
)


def _parse_boolean(_context: CommandParseContext, cursor: CommandCursor) -> ParseResult[bool]:
    result = cursor.next_value(_input_expected(_BOOLEAN_INPUT))
    if isinstance(result, Err):
        return result
    token = result.ok()
    value = token.value.upper()
    if value in _BOOLEAN_INPUT.true:
        return Ok(token.map(True))
    if value in _BOOLEAN_INPUT.false:
        return Ok(token.map(False))
    return Err(ArgumentIssue.expected(_input_expected(_BOOLEAN_INPUT), token.value, token.span))


_INT_PARSER = _converter_parser(Converter(int), CommandField())
_FLOAT_PARSER = _converter_parser(Converter(float), CommandField())
_ON_OFF_PARSER = CmdParser(_parse_boolean, _BOOLEAN_INPUT)
OnOff = Annotated[bool, _ON_OFF_PARSER]


_UNSIGNED_NUMBER_RE = r"(?:\d+(?:\.\d*)?|\.\d+)"
_NUMBER_RE = rf"[+-]?{_UNSIGNED_NUMBER_RE}"
_STD_ALTITUDE = re.compile(rf"(?P<value>{_NUMBER_RE})(?P<unit>FT|M)\[STD\]", re.IGNORECASE)
_MSL_ALTITUDE = re.compile(rf"(?P<value>{_NUMBER_RE})(?P<unit>FT|M)\[MSL\]", re.IGNORECASE)
_FLIGHT_LEVEL = re.compile(r"FL(?P<level>\d+)", re.IGNORECASE)
_DISTANCE = re.compile(rf"(?P<value>{_NUMBER_RE})(?P<unit>NM|KM|FT|M)", re.IGNORECASE)
_SPEED_UNIT_RE = r"(?:MPS|FT/MIN|KT|KMH)"
_UNIT_SPEED = re.compile(rf"(?P<value>{_NUMBER_RE})(?P<unit>{_SPEED_UNIT_RE})", re.IGNORECASE)
_CAS = re.compile(
    rf"(?P<value>{_UNSIGNED_NUMBER_RE})(?P<unit>{_SPEED_UNIT_RE})\[CAS\]",
    re.IGNORECASE,
)
_MACH = re.compile(r"M(?P<value>(?:\d+\.\d+|\.\d+))", re.IGNORECASE)


def _vertical_metres(value: str, unit: str) -> float:
    return float(value) if unit.upper() == "M" else q.ft_to_m(float(value))


def _parse_pressure_altitude(value: str) -> q.PressureAltitudeM[float]:
    if match := _FLIGHT_LEVEL.fullmatch(value):
        return q.ft_to_m(100.0 * int(match.group("level")))
    if (match := _STD_ALTITUDE.fullmatch(value)) is None:
        raise ValueError
    return _vertical_metres(match.group("value"), match.group("unit"))


def parse_pressure_altitude_value(value: str) -> t.StdPressureAltM[float]:
    """Compatibility wrapper for bespoke parsers not yet migrated."""
    return t.StdPressureAltM(_parse_pressure_altitude(value))


def _parse_msl_altitude(value: str) -> q.MslAltitudeM[float]:
    if (match := _MSL_ALTITUDE.fullmatch(value)) is None:
        raise ValueError
    return _vertical_metres(match.group("value"), match.group("unit"))


# Numeric altitudes require an explicit unit. AGL stays out until ground
# elevation exists; relabeling pressure altitude as height would manufacture a
# datum the simulation does not have.


def parse_distance_value(value: str) -> q.DistanceM[float]:
    """Parse an explicit length unit and normalize to metres."""
    if (match := _DISTANCE.fullmatch(value)) is None:
        raise ValueError
    magnitude = float(match.group("value"))
    match match.group("unit").upper():
        case "M":
            return magnitude
        case "KM":
            return q.km_to_m(magnitude)
        case "NM":
            return q.nmi_to_m(magnitude)
        case "FT":
            return q.ft_to_m(magnitude)
        case _:
            raise AssertionError("unreachable distance unit")


DistanceM = Annotated[
    q.DistanceM[IsFinite[float]],
    CommandField(name="distance", examples=("5NM", "9.26KM", "9260M", "3000FT")),
    Converter(parse_distance_value),
]
"""Quantity-kind-free distance normalized to metres.

An explicit unit is required. Use forms such as `5NM`, `9.26KM`, `9260M`,
or `3000FT`.
"""


def parse_speed_value(value: str) -> q.SpeedMps[float]:
    """Parse an explicit speed unit and normalize to metres per second."""
    if (match := _UNIT_SPEED.fullmatch(value)) is None:
        raise ValueError
    magnitude = float(match.group("value"))
    match match.group("unit").upper():
        case "MPS":
            return magnitude
        case "FT/MIN":
            return q.fpm_to_mps(magnitude)
        case "KT":
            return q.kt_to_mps(magnitude)
        case "KMH":
            return q.kmh_to_mps(magnitude)
        case _:
            raise AssertionError("unreachable speed unit")


SpeedMps = Annotated[
    q.SpeedMps[IsFinite[float]],
    CommandField(name="speed", examples=("10MPS", "600FT/MIN", "20KT", "90KMH")),
    Converter(parse_speed_value),
]
"""Quantity-kind-free speed normalized to metres per second.

An explicit unit is required. Use forms such as `10MPS`, `600FT/MIN`, `20KT`,
or `90KMH`.
"""


def _parse_cas(value: str) -> q.CalibratedAirspeedMps[float]:
    if (match := _CAS.fullmatch(value)) is None:
        raise ValueError
    unit_value = f"{match.group('value')}{match.group('unit')}"
    return parse_speed_value(unit_value)


def parse_cas_value(value: str) -> t.CasMps[float]:
    """Compatibility wrapper for bespoke callers not yet migrated."""
    return t.CasMps(_parse_cas(value))


def _parse_mach(value: str) -> q.MachNumber[float]:
    if (match := _MACH.fullmatch(value)) is None:
        raise ValueError
    return float(match.group("value"))


def parse_selected_airspeed_value(
    value: str,
) -> t.CasMps[IsFinite[t.Ge0[float]]] | t.Mach[IsFinite[t.Gt0[float]]]:
    if value.upper().startswith("M"):
        mach = _parse_mach(value)
        if mach <= 0.0:
            raise ValueError
        return t.Mach(mach)
    return t.CasMps(_parse_cas(value))


VspdMps = Annotated[
    q.VerticalRateMps[IsFinite[float]],
    CommandField(name="vertical speed", examples=("1500FT/MIN", "7.62MPS")),
    Converter(parse_speed_value),
]
TimeS = Annotated[
    q.DurationS[IsFinite[t.Ge0[float]]],
    CommandField(name="time", examples=("90", "01:30", "01:02:03")),
    Converter(txt2tim),
]
SimTimeS = Annotated[
    q.SimulationTimeS[IsFinite[t.Ge0[float]]],
    CommandField(name="time", examples=("90", "01:30", "01:02:03")),
    Converter(txt2tim),
]


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


def _parse_true_heading(value: str) -> q.TrueHeadingDegrees[float]:
    return _matched_number(_TRUE_HEADING, value)


def _parse_magnetic_heading(value: str) -> q.MagneticHeadingDegrees[float]:
    return _matched_number(_MAGNETIC_HEADING, value)


def _parse_ground_track(value: str) -> q.GroundTrackDeg[float]:
    return _matched_number(_GROUND_TRACK, value)


@dataclass(frozen=True, slots=True)
class RunwayHeadingRequest:
    """Preserve a source-level `*` request until the command can resolve it.

    For example, in `CRE KLM1,A320,EHAM,RWY18L,*,0FT[STD],250KT[CAS]`, `*` means "use the
    heading of the runway parsed in the previous argument". Unlike bluesky
    (which internally stores the RWY18L in its parser), minisky directly returns
    the sentinel to avoid a parser-global cross-argument state.
    """


_RUNWAY_HEADING = RunwayHeadingRequest()
UseRunwayHeading = Annotated[
    RunwayHeadingRequest,
    CmdParser.choices({"*": _RUNWAY_HEADING}),
]
"""The `*` heading form, preserved as an explicit callback value.

Only commands that can derive a heading from another argument should include this
in their annotation, for example `HeadingDeg | UseRunwayHeading`.
"""


def _aircraft_index(
    context: CommandParseContext, callsign: t.AircraftCallsign
) -> Result[t.AircraftIndex, ArgumentIssue]:
    index = context.traffic.idx(callsign)
    if index is None:
        return Err(ArgumentIssue.expected("an existing aircraft", callsign))
    return Ok(index)


_AIRCRAFT_INPUT = CommandField(name="aircraft")


def parse_aircraft(
    context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[t.AircraftIndex]:
    result = cursor.next_value(_input_expected(_AIRCRAFT_INPUT))
    if isinstance(result, Err):
        return result
    token = result.ok()
    callsign = token.value.upper()

    if context.traffic.idx(callsign) is None and callsign in context.traffic.groups:
        return Err(ArgumentIssue.expected("an aircraft", f"group {callsign}", token.span))

    if isinstance(index := _aircraft_index(context, callsign), Err):
        return Err(index.err().with_span(token.span))
    return Ok(token.map(index.ok()))


AcId = Annotated[t.AircraftIndex, CmdParser(parse_aircraft, _AIRCRAFT_INPUT)]
"""An existing aircraft callsign resolved to its traffic-array index."""


_AIRCRAFT_SELECTION_INPUT = CommandField(name="aircraft or group", examples=("*", "ALL"))


def parse_aircraft_selection(
    context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[np.ndarray[Any, Any]]:
    result = cursor.next_value(_input_expected(_AIRCRAFT_SELECTION_INPUT))
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


AcIdSelection = Annotated[
    np.ndarray[Any, Any], CmdParser(parse_aircraft_selection, _AIRCRAFT_SELECTION_INPUT)
]
"""An aircraft, a traffic group, or `*`/`ALL`, resolved to traffic indices."""


def aircraft_indices(
    selections: Iterable[np.ndarray[Any, Any]],
) -> np.ndarray[Any, Any]:
    """Flatten aircraft and group selections into an index array."""
    indices = (int(index) for selection in selections for index in selection)
    return np.fromiter(indices, dtype=int)


def _parse_waypoint_reference(value: str) -> t.WaypointReference:
    name = value.upper()
    if not name or islat(name):
        raise ValueError
    return name


_LatitudeArg = Annotated[
    q.LatitudeDeg[IsFinite[float]],
    CommandField(name="latitude", examples=("52.5", "N52'30'")),
    Converter(txt2lat),
    Ge(-90),
    Le(90),
]
_LongitudeArg = Annotated[
    q.LongitudeDeg[IsFinite[float]],
    CommandField(name="longitude", examples=("4.5", "E004'30'")),
    Converter(txt2lon),
    Ge(-180),
    Le(180),
]
_WaypointReferenceArg = Annotated[
    t.WaypointReference,
    CommandField(name="waypoint", examples=("SUGOL", "EHAM", "EHAM/RW06")),
    Converter(_parse_waypoint_reference),
]


class CoordinateWaypoint(NamedTuple):
    """A waypoint expressed directly as latitude and longitude."""

    latitude: _LatitudeArg
    longitude: _LongitudeArg

    @property
    def coordinates(self) -> t.LatLonDegrees:
        return t.LatLonDegrees(self.latitude, self.longitude)


class NamedWaypoint(NamedTuple):
    """A waypoint expression that should be resolved by name."""

    name: _WaypointReferenceArg


@dataclass(frozen=True, slots=True)
class RunwayPosition:
    """Resolved runway coordinates and their runway heading."""

    coordinates: t.LatLonDegrees
    runway_heading: q.TrueHeadingDegrees[float]


def _resolve_named_position(
    context: CommandParseContext, name: str, span: SourceSpan
) -> Result[t.LatLonDegrees | RunwayPosition, ArgumentIssue]:
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
        return Ok(RunwayPosition(t.LatLonDegrees(float(lat), float(lon)), float(heading)))

    if name in navigation.aptid:
        index = navigation.aptid.index(name)
        return Ok(t.LatLonDegrees(float(navigation.aptlat[index]), float(navigation.aptlon[index])))

    occurrences = navigation.wpid.count(name)
    if occurrences > 1:
        return Err(
            ArgumentIssue.expected("an unambiguous waypoint id or explicit coordinates", name, span)
        )
    if occurrences == 1:
        index = navigation.wpid.index(name)
        return Ok(t.LatLonDegrees(float(navigation.wplat[index]), float(navigation.wplon[index])))

    return Err(ArgumentIssue.expected("a waypoint, airport, runway, or aircraft id", name, span))


def _resolve_position(
    context: CommandParseContext, waypoint: CoordinateWaypoint | NamedWaypoint, span: SourceSpan
) -> Result[t.LatLonDegrees | RunwayPosition, ArgumentIssue]:
    if isinstance(waypoint, CoordinateWaypoint):
        return Ok(waypoint.coordinates)

    index = context.traffic.idx(waypoint.name)
    if index is not None:
        return Ok(
            t.LatLonDegrees(float(context.traffic.lat[index]), float(context.traffic.lon[index]))
        )

    return _resolve_named_position(context, waypoint.name, span)


def parse_resolved_position(
    context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[t.LatLonDegrees | RunwayPosition]:
    if isinstance(result := _WAYPOINT_CONTRACT.parser(context, cursor), Err):
        return result
    parsed = result.ok()
    if isinstance(resolved := _resolve_position(context, parsed.value, parsed.span), Err):
        return resolved
    return Ok(parsed.map(resolved.ok()))


def parse_lat_lon(
    context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[t.LatLonDegrees]:
    if isinstance(result := _RESOLVED_POSITION_PARSER(context, cursor), Err):
        return result
    parsed = result.ok()
    coordinates = (
        parsed.value.coordinates if isinstance(parsed.value, RunwayPosition) else parsed.value
    )
    return Ok(parsed.map(coordinates))


_RUNTIME_NEWTYPE_METADATA: dict[
    type[t.RuntimeNewType[Any]], tuple[CommandField, Converter[Any]]
] = {
    t.CasMps: (
        CommandField(name="CAS", examples=("250KT[CAS]", "128MPS[CAS]")),
        Converter(_parse_cas),
    ),
    t.Mach: (CommandField(name="Mach", examples=("M0.78", "M.78")), Converter(_parse_mach)),
    t.StdPressureAltM: (
        CommandField(name="pressure altitude", examples=("FL100", "10000FT[STD]", "3048M[STD]")),
        Converter(_parse_pressure_altitude),
    ),
    t.MslAltM: (
        CommandField(name="MSL altitude", examples=("10000FT[MSL]", "3048M[MSL]")),
        Converter(_parse_msl_altitude),
    ),
    t.TrueHeadingDeg: (
        CommandField(name="true heading", examples=("090", "090T")),
        Converter(_parse_true_heading),
    ),
    t.MagneticHeadingDeg: (
        CommandField(name="magnetic heading", examples=("090M",)),
        Converter(_parse_magnetic_heading),
    ),
    t.GroundTrackDeg: (
        CommandField(name="ground track", examples=("090TRK",)),
        Converter(_parse_ground_track),
    ),
}


#
# parameter contracts
#


_Constraint: TypeAlias = Gt | Ge | Lt | Le | MinLen | MaxLen | Predicate


@dataclass(frozen=True, slots=True)
class _ArgumentVariant:
    input: CommandInput
    annotation: Any
    constraints: tuple[_Constraint, ...] = ()


@dataclass(frozen=True, slots=True)
class _ArgumentContract:
    """One executable command parser and its semantic branch descriptions."""

    parser: CmdParser[Any]
    variants: tuple[_ArgumentVariant, ...]
    nullable: bool = False


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


def _contract(
    parser: CmdParser[Any],
    annotation: Any,
    constraints: tuple[_Constraint, ...] = (),
    *,
    nullable: bool = False,
    finalize: Callable[[Any], Any] | None = None,
) -> _ArgumentContract:
    input_spec = parser.input
    if constraints or finalize is not None:
        inner = parser

        def parse(context: CommandParseContext, cursor: CommandCursor) -> ParseResult[Any]:
            result = inner(context, cursor)
            if isinstance(result, Err):
                return result
            value = result.ok()
            if isinstance(validation := _validate_constraints(value.value, constraints), Err):
                return Err(validation.err().with_span(value.span))
            return Ok(value.map(finalize(value.value) if finalize is not None else value.value))

        parser = CmdParser(parse, input_spec)

    return _ArgumentContract(
        parser=parser,
        variants=(_ArgumentVariant(input_spec, annotation, constraints),),
        nullable=nullable,
    )


@dataclass(frozen=True, slots=True)
class Parameter:
    """Executable command contract compiled from a callback parameter."""

    name: str
    contract: _ArgumentContract
    default: object = inspect.Parameter.empty
    repeat: bool = False

    def parse(
        self, context: CommandParseContext, cursor: CommandCursor
    ) -> Result[tuple[Any, ...], ArgumentIssue]:
        if self.repeat:
            return self._parse_repeated(context, cursor)
        if cursor.at_end:
            if self.default is not inspect.Parameter.empty:
                return Ok((self.default,))
            return Err(self._missing(SourceSpan(cursor.pos, cursor.pos), "end of input"))

        if self.contract.nullable or self.default is not inspect.Parameter.empty:
            peeked = cursor.peek_field()
            if isinstance(peeked, Err):
                return peeked
            field_value = peeked.ok()
            if field_value is not None and field_value.value is None:
                cursor.next_field()
                value = None if self.contract.nullable else self.default
                return Ok((value,))
        return self._parse_one(context, cursor)

    def _parse_repeated(
        self, context: CommandParseContext, cursor: CommandCursor
    ) -> Result[tuple[Any, ...], ArgumentIssue]:
        values: list[Any] = []
        while not cursor.at_end:
            result = self._parse_one(context, cursor)
            if isinstance(result, Err):
                return result
            values.extend(result.ok())
        return Ok(tuple(values))

    def _missing(self, span: SourceSpan, actual: str) -> ArgumentIssue:
        return ArgumentIssue.expected(
            _input_expected(self.contract.parser.input), actual, span
        ).at_argument(self.name, span)

    def _parse_one(
        self, context: CommandParseContext, cursor: CommandCursor
    ) -> Result[tuple[Any, ...], ArgumentIssue]:
        start = cursor.checkpoint()
        if isinstance(parsed_result := self.contract.parser(context, cursor), Err):
            issue = parsed_result.err()
            fallback = issue.span or SourceSpan(start, start + 1)
            return Err(issue.at_argument(self.name, fallback))
        if cursor.pos <= start:
            raise TypeError(f"parser {self.contract.parser!r} consumed no input")
        return Ok((parsed_result.ok().value,))

    @property
    def optional(self) -> bool:
        return self.repeat or self.default is not inspect.Parameter.empty

    @property
    def parser(self) -> CmdParser[Any]:
        return self.contract.parser

    @property
    def variants(self) -> tuple[_ArgumentVariant, ...]:
        return self.contract.variants

    @property
    def nullable(self) -> bool:
        return self.contract.nullable


#
# annotation compilation
#


@dataclass(frozen=True, slots=True)
class _Annotation:
    """One normalized view of a Python command annotation."""

    annotation: Any
    base: Any
    metadata: tuple[object, ...]
    members: tuple[Any, ...]
    nullable: bool
    parser: CmdParser[Any] | None
    converter: Converter[Any] | None
    field: CommandField | None
    constraints: tuple[_Constraint, ...]


def _constraints(metadata: Iterable[object]) -> Iterator[_Constraint]:
    for item in metadata:
        if isinstance(item, GroupedMetadata):
            yield from _constraints(item)
        elif isinstance(item, (Gt, Ge, Lt, Le, MinLen, MaxLen, Predicate)):
            yield item
        elif isinstance(item, BaseMetadata):
            raise TypeError(f"unsupported annotated-types constraint: {item!r}")


def _annotation(annotation: Any) -> _Annotation:
    if get_origin(annotation) is Annotated:
        base, *metadata = get_args(annotation)
    else:
        base, metadata = annotation, []

    parsers = tuple(item for item in metadata if isinstance(item, CmdParser))
    converters = tuple(item for item in metadata if isinstance(item, Converter))
    if len(parsers) > 1:
        raise TypeError("stack annotation contains multiple CmdParser markers")
    if len(converters) > 1:
        raise TypeError("stack annotation contains multiple Converter markers")
    parser = parsers[0] if parsers else None
    converter = converters[0] if converters else None
    if parser is not None and converter is not None:
        raise TypeError("stack annotation cannot contain both CmdParser and Converter")

    field_markers = tuple(item for item in metadata if isinstance(item, CommandField))
    field_marker: CommandField | None = None
    if field_markers:
        name: str | None = None
        examples: tuple[str, ...] = ()
        for marker in field_markers:
            if marker.name is not None:
                name = marker.name
            if marker.examples:
                examples = marker.examples
        field_marker = CommandField(name, examples)

    origin = get_origin(base)
    union = origin in (Union, UnionType)
    members = (
        tuple(member for member in get_args(base) if member is not type(None)) if union else ()
    )
    nullable = union and type(None) in get_args(base)
    metadata_tuple = tuple(metadata)
    return _Annotation(
        annotation=annotation,
        base=base,
        metadata=metadata_tuple,
        members=members,
        nullable=nullable,
        parser=parser,
        converter=converter,
        field=field_marker,
        constraints=tuple(_constraints(metadata_tuple)),
    )


def _merge_field(input_spec: CommandField, marker: CommandField | None) -> CommandField:
    if marker is None:
        return input_spec
    return CommandField(
        marker.name if marker.name is not None else input_spec.name,
        marker.examples or input_spec.examples,
    )


def _argument_contract(annotation: Any) -> _ArgumentContract:
    info = _annotation(annotation)

    # Explicit parser/converter metadata owns the whole semantic annotation.
    # Otherwise a Python union is a command-level left-to-right choice.
    if info.members and info.parser is None and info.converter is None:
        if info.field is not None:
            raise TypeError("CommandField cannot annotate a structural union")
        if info.constraints:
            raise TypeError("constraints on a command union must annotate its individual branches")
        alternatives = tuple(_argument_contract(member) for member in info.members)
        if len(alternatives) == 1:
            return replace(alternatives[0], nullable=info.nullable)

        def parse_choice(context: CommandParseContext, cursor: CommandCursor) -> ParseResult[Any]:
            failure: ArgumentIssue | None = None
            for alternative in alternatives:
                result = alternative.parser(context, cursor)
                if isinstance(result, Ok):
                    return result
                failure = result.err()
            assert failure is not None
            return Err(failure)

        return _ArgumentContract(
            parser=CmdParser(
                parse_choice,
                _ChoiceInput(tuple(alternative.parser.input for alternative in alternatives)),
            ),
            variants=tuple(
                variant for alternative in alternatives for variant in alternative.variants
            ),
            nullable=info.nullable,
        )

    origin = get_origin(info.base)
    runtime_type = origin if origin is not None else info.base
    runtime_newtype = (
        runtime_type
        if isinstance(runtime_type, type) and issubclass(runtime_type, t.RuntimeNewType)
        else None
    )
    if runtime_newtype is not None and info.constraints:
        raise TypeError(
            "RuntimeNewType constraints belong on the inner type parameter, "
            f"for example {runtime_newtype.__name__}[IsFinite[t.Gt0[float]]]"
        )

    if info.parser is not None:
        if info.field is not None:
            raise TypeError("CommandField cannot override an explicit CmdParser input")
        return _contract(info.parser, info.annotation, info.constraints, nullable=info.nullable)

    if runtime_newtype is not None:
        arguments = get_args(info.base)
        inner = arguments[0] if arguments else Any
        if info.converter is None:
            metadata = _RUNTIME_NEWTYPE_METADATA.get(runtime_newtype)
            if metadata is None:
                raise TypeError(f"no command converter registered for {runtime_newtype.__name__}")
            input_spec, converter = metadata
        else:
            input_spec, converter = CommandField(), info.converter
        parser = _converter_parser(converter, _merge_field(input_spec, info.field))
        return _contract(
            parser,
            info.annotation,
            _annotation(inner).constraints,
            nullable=info.nullable,
            finalize=runtime_newtype,
        )

    if info.converter is not None:
        parser = _converter_parser(info.converter, _merge_field(CommandField(), info.field))
        return _contract(parser, info.annotation, info.constraints, nullable=info.nullable)

    if (
        isinstance(info.base, type)
        and issubclass(info.base, tuple)
        and isinstance(getattr(info.base, "_fields", None), tuple)
        and bool(getattr(info.base, "__annotations__", None))
    ):
        if info.field is not None:
            raise TypeError("CommandField metadata can only annotate a single command field")
        return _namedtuple_contract(info)

    base = info.base
    if get_origin(base) is Literal:
        if info.field is not None:
            raise TypeError("CommandField metadata can only annotate a single command field")
        values = get_args(base)
        if not values or any(not isinstance(value, str) for value in values):
            raise TypeError("stack command Literal values must be strings")
        literals = tuple(value.upper() for value in values)
        try:
            parser = CmdParser.choices(dict(zip(values, literals, strict=True)))
        except ValueError:
            raise TypeError("stack command Literal repeats a case-insensitive value") from None
    elif base is bool:
        if info.field is not None:
            raise TypeError("CommandField metadata can only annotate a single command field")
        parser = _ON_OFF_PARSER
    elif base is inspect._empty or base is str or base is Any:
        parser = _token_parser(_merge_field(CommandField(), info.field))
    elif base is int:
        parser = _converter_parser(Converter(int), _merge_field(CommandField(), info.field))
    elif base is float:
        parser = _converter_parser(Converter(float), _merge_field(CommandField(), info.field))
    else:
        raise TypeError(f"unsupported stack annotation: {info.annotation!r}")

    return _contract(parser, info.annotation, info.constraints, nullable=info.nullable)


def _namedtuple_contract(info: _Annotation) -> _ArgumentContract:
    """Compile a NamedTuple so consecutive command fields produce one record."""
    namedtuple_type: type[tuple[Any, ...]] = info.base
    if getattr(namedtuple_type, "_field_defaults", {}):
        raise TypeError(
            f"command NamedTuple {namedtuple_type.__name__} cannot define field defaults"
        )

    annotations = get_type_hints(namedtuple_type, include_extras=True)
    field_names: tuple[str, ...] = namedtuple_type._fields  # type: ignore[attr-defined]
    fields: list[_ArgumentContract] = []
    inputs: list[CommandField] = []
    for name in field_names:
        field_annotation = annotations[name]
        field_info = _annotation(field_annotation)
        contract = _argument_contract(field_annotation)
        if contract.nullable:
            raise TypeError(
                f"command NamedTuple field {namedtuple_type.__name__}.{name} cannot be nullable"
            )
        if not isinstance(contract.parser.input, CommandField):
            raise TypeError(
                f"command NamedTuple field {namedtuple_type.__name__}.{name} must consume one field"
            )
        input_spec = contract.parser.input
        if field_info.field is None or field_info.field.name is None:
            input_spec = replace(input_spec, name=name)
        fields.append(contract)
        inputs.append(input_spec)

    def parse(context: CommandParseContext, cursor: CommandCursor) -> ParseResult[Any]:
        start = cursor.checkpoint()
        values: list[Any] = []
        end = start
        for name, contract in zip(field_names, fields, strict=True):
            field_start = cursor.checkpoint()
            result = contract.parser(context, cursor)
            if isinstance(result, Err):
                issue = result.err()
                return Err(issue.at_argument(name, SourceSpan(field_start, field_start + 1)))
            parsed = result.ok()
            values.append(parsed.value)
            end = parsed.span.end
        return Ok(Spanned(namedtuple_type(*values), SourceSpan(start, end)))

    parser = CmdParser(parse, _RecordInput(namedtuple_type.__name__, tuple(inputs)))
    return _contract(parser, info.annotation, info.constraints, nullable=info.nullable)


def compile_parameter(parameter: inspect.Parameter) -> Parameter:
    """Compile a callback parameter into an executable command contract."""
    contract = _argument_contract(parameter.annotation)
    if parameter.default is None and not contract.nullable:
        raise TypeError(
            f"stack parameter {parameter.name} defaults to None but is not annotated T | None"
        )
    if (
        contract.nullable
        and parameter.default is not inspect.Parameter.empty
        and parameter.default is not None
    ):
        raise TypeError(
            f"nullable stack parameter {parameter.name} must default to None when optional"
        )
    return Parameter(
        name=parameter.name,
        contract=contract,
        default=parameter.default,
        repeat=parameter.kind is inspect.Parameter.VAR_POSITIONAL,
    )


_WAYPOINT_CONTRACT = _argument_contract(CoordinateWaypoint | NamedWaypoint)
_RESOLVED_POSITION_PARSER = CmdParser(parse_resolved_position, _WAYPOINT_CONTRACT.parser.input)
ResolvedPositionArg = Annotated[
    t.LatLonDegrees | RunwayPosition,
    _RESOLVED_POSITION_PARSER,
]
LatLonDeg = Annotated[
    t.LatLonDegrees,
    CmdParser(parse_lat_lon, _WAYPOINT_CONTRACT.parser.input),
]


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


def _format_input(input_spec: CommandInput, parameter_name: str) -> str:
    match input_spec:
        case CommandField(name=name):
            return name or parameter_name
        case _RecordInput(_, fields):
            return ",".join(field.name or parameter_name for field in fields)
        case _LiteralInput(values):
            return "|".join(values)
        case _BooleanInput():
            return parameter_name
        case _OmittedInput():
            return ""
        case _TextInput():
            return parameter_name
        case _ChoiceInput(alternatives):
            return "|".join(
                _format_input(alternative, parameter_name) for alternative in alternatives
            )


def _format_parameter(parameter: Parameter) -> str:
    text = _format_input(parameter.parser.input, parameter.name)
    if parameter.repeat:
        text += "..."
    if parameter.optional or parameter.nullable:
        text = f"[{text}]"
    return text


def format_command_form(name: str, parameters: Iterable[Parameter]) -> str:
    rendered = ",".join(_format_parameter(parameter) for parameter in parameters)
    return f"{name} {rendered}" if rendered else name


#
# schema
#


@dataclass(frozen=True, slots=True)
class CommandVariant:
    input: CommandInput


@dataclass(frozen=True, slots=True)
class ParameterSchema:
    name: str
    variants: tuple[CommandVariant, ...]
    optional: bool = False
    nullable: bool = False
    repeat: bool = False

    @property
    def input(self) -> CommandInput:
        if len(self.variants) == 1:
            return self.variants[0].input
        return _ChoiceInput(tuple(variant.input for variant in self.variants))


@dataclass(frozen=True, slots=True)
class CommandFormSchema:
    parameters: tuple[ParameterSchema, ...]
    doc: str = ""


@dataclass(frozen=True, slots=True)
class CommandEntry:
    forms: tuple[CommandFormSchema, ...]
    aliases: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class CommandSchema:
    commands: dict[str, CommandEntry]


def build_command_schema(commands: Iterable[Command]) -> CommandSchema:
    entries = {
        command.name: CommandEntry(
            forms=tuple(
                CommandFormSchema(
                    parameters=tuple(
                        ParameterSchema(
                            name=parameter.name,
                            variants=tuple(
                                CommandVariant(variant.input) for variant in parameter.variants
                            ),
                            optional=parameter.optional,
                            nullable=parameter.nullable,
                            repeat=parameter.repeat,
                        )
                        for parameter in form.parameters
                    ),
                    doc=form.help.strip(),
                )
                for form in command.forms
            ),
            aliases=tuple(sorted(command.aliases)),
        )
        for command in sorted(commands, key=lambda item: item.name)
    }
    return CommandSchema(entries)


def load_command_schema(data: str | bytes) -> CommandSchema:
    from pydantic import TypeAdapter

    return TypeAdapter(CommandSchema).validate_json(data)
