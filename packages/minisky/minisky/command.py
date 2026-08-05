"""Typed stack-command declarations and parsers."""

from __future__ import annotations

import inspect
from collections.abc import Callable, Iterator
from dataclasses import dataclass, replace
from typing import (
    TYPE_CHECKING,
    Annotated,
    Any,
    Generic,
    Literal,
    TypeAlias,
    TypeVar,
    get_args,
    get_origin,
    overload,
)

import numpy as np
from annotated_types import BaseMetadata, Ge, Gt, IsFinite, Le, Predicate

from minisky.identifiers import normalize_public_name
from minisky.result import Err, Ok, Result
from minisky.tools.convert import txt2alt, txt2bool, txt2lat, txt2lon, txt2spd, txt2tim, txt2vs
from minisky.tools.position import islat

if TYPE_CHECKING:
    from minisky.tools.navdata import Navdatabase
    from minisky.traffic import Traffic

CommandCallback = Callable[..., Any]
CommandTarget = TypeVar("CommandTarget", bound=CommandCallback)
ParsedT_co = TypeVar("ParsedT_co", covariant=True)
_COMMAND = "__minisky_command__"


@dataclass(frozen=True, slots=True)
class SourceSpan:
    """Half-open source range for a parsed value."""

    start: int
    end: int

    def offset_by(self, offset: int) -> SourceSpan:
        return SourceSpan(self.start + offset, self.end + offset)


@dataclass(frozen=True, slots=True)
class Parsed(Generic[ParsedT_co]):
    """A parsed value together with unconsumed command text."""

    value: ParsedT_co
    remainder: str
    span: SourceSpan


@dataclass(frozen=True, slots=True)
class ArgumentIssue:
    """A command argument parsing error."""

    message: str
    span: SourceSpan | None = None
    source_text: str | None = None

    @classmethod
    def expected(
        cls, expected: str, actual: object, span: SourceSpan | None = None
    ) -> ArgumentIssue:
        return cls(f"expected {expected}, but got {actual}", span)

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


@dataclass(frozen=True, slots=True)
class CommandParseContext:
    """Read-only runtime services available while parsing a command value."""

    traffic: Traffic
    navigation: Navdatabase


ParseFunction: TypeAlias = Callable[[CommandParseContext, str], ParseResult[ParsedT_co]]


def next_argument(text: str) -> ParseResult[str]:
    """Parse a command field and leave the remaining fields untouched."""
    index = 0
    length = len(text)
    while index < length and text[index].isspace():
        index += 1

    if index == length:
        return Ok(Parsed("", "", SourceSpan(length, length)))

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
            index += 1
        value = text[start:index]
        token_end = index

    # TODO(abraham): spans eventually include separators, or stay value-only?
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


@dataclass(frozen=True, slots=True)
class LiteralSyntax:
    """Exact case-insensitive keywords used when rendering command usage."""

    values: tuple[str, ...]


ParserSyntax: TypeAlias = LiteralSyntax


@dataclass(frozen=True, slots=True)
class CmdParser(Generic[ParsedT_co]):
    """Connect an annotated Python value to command-text parsing."""

    func: ParseFunction[ParsedT_co]
    syntax: ParserSyntax | None = None

    @classmethod
    def literals(
        cls, func: ParseFunction[ParsedT_co], values: tuple[str, ...]
    ) -> CmdParser[ParsedT_co]:
        normalized = tuple(value.upper() for value in values)
        if not normalized or len(normalized) != len(set(normalized)):
            raise ValueError("command parser literals must be non-empty and unique")
        return cls(func, LiteralSyntax(normalized))

    def __call__(self, context: CommandParseContext, text: str) -> ParseResult[ParsedT_co]:
        return self.func(context, text)


def parse_text(_context: CommandParseContext, text: str) -> ParseResult[str]:
    """Consume the complete remaining command text verbatim."""
    return Ok(Parsed(text, "", SourceSpan(0, len(text))))


Text = Annotated[str, CmdParser(parse_text)]
"""The complete remaining command text, consumed verbatim."""


def parse_token(_context: CommandParseContext, text: str) -> ParseResult[str]:
    """Parse a non-empty command field as case-sensitive text."""
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    if not token.value:
        return Err(ArgumentIssue.expected("a non-empty argument", "empty input", token.span))
    return result


Token = Annotated[str, CmdParser(parse_token)]
"""A non-empty command field parsed as case-sensitive text."""


FiniteFloat: TypeAlias = IsFinite[float]
PositiveFiniteFloat: TypeAlias = Annotated[FiniteFloat, Gt(0)]


def _convert(
    text: str, converter: Callable[[str], ParsedT_co], expected: str
) -> ParseResult[ParsedT_co]:
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    try:
        value = converter(token.value)
    except ValueError:
        return Err(ArgumentIssue.expected(expected, token.value or "empty input", token.span))
    return Ok(Parsed(value, token.remainder, token.span))


def parse_int(_context: CommandParseContext, text: str) -> ParseResult[int]:
    """Parse a integer command field."""
    return _convert(text, int, "an integer")


def parse_float(_context: CommandParseContext, text: str) -> ParseResult[float]:
    """Parse a floating-point command field."""
    return _convert(text, float, "a number")


def parse_on_off(_context: CommandParseContext, text: str) -> ParseResult[bool]:
    """Parse a on/off command field."""
    return _convert(text, txt2bool, "ON or OFF")


OnOff = Annotated[bool, CmdParser(parse_on_off)]


def parse_altitude(_context: CommandParseContext, text: str) -> ParseResult[float]:
    """Parse a BlueSky altitude field as meters."""
    return _convert(text, txt2alt, "an altitude")


AltM = Annotated[float, CmdParser(parse_altitude)]


def parse_speed(_context: CommandParseContext, text: str) -> ParseResult[float]:
    """Parse a BlueSky speed field as CAS m/s or Mach."""
    return _convert(text, txt2spd, "a speed")


SpeedMpsOrMach = Annotated[float, CmdParser(parse_speed)]


def parse_vertical_speed(_context: CommandParseContext, text: str) -> ParseResult[float]:
    """Parse a BlueSky vertical-speed field as meters per second."""
    return _convert(text, txt2vs, "a vertical speed")


VspdMps = Annotated[float, CmdParser(parse_vertical_speed)]


def parse_time(_context: CommandParseContext, text: str) -> ParseResult[float]:
    """Parse a BlueSky time field as seconds."""
    return _convert(text, txt2tim, "a time")


TimeS = Annotated[float, CmdParser(parse_time)]


def parse_aircraft(context: CommandParseContext, text: str) -> ParseResult[int]:
    """Resolve an existing aircraft callsign to its traffic-array index."""
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    callsign = token.value.upper()
    index = context.traffic.idx(callsign)
    if index < 0 and callsign in context.traffic.groups:
        return Err(ArgumentIssue.expected("an aircraft", f"group {callsign}", token.span))
    if index < 0:
        return Err(ArgumentIssue.expected("an existing aircraft", callsign, token.span))
    return Ok(Parsed(index, token.remainder, token.span))


AcId = Annotated[int, CmdParser(parse_aircraft)]


def parse_aircraft_selection(
    context: CommandParseContext, text: str
) -> ParseResult[np.ndarray[Any, Any]]:
    """Resolve an aircraft, group, or `*` to traffic-array indices."""
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    name = token.value.upper()

    if name == "*" or name in context.traffic.groups:
        if isinstance(selection := context.traffic.groups.listgroup(name), Err):
            return Err(
                ArgumentIssue.expected("an existing aircraft or group", selection.err(), token.span)
            )
        return Ok(Parsed(np.asarray(selection.ok(), dtype=int), token.remainder, token.span))

    index = context.traffic.idx(name)
    if index < 0:
        return Err(ArgumentIssue.expected("an existing aircraft or group", name, token.span))
    return Ok(Parsed(np.asarray([index], dtype=int), token.remainder, token.span))


AcIdSelection = Annotated[np.ndarray[Any, Any], CmdParser(parse_aircraft_selection)]


@dataclass(frozen=True, slots=True)
class LatLonDegrees:
    """Resolved latitude and longitude in degrees."""

    lat: float
    lon: float


def _resolve_named_lat_lon(
    context: CommandParseContext, name: str, span: SourceSpan
) -> Result[LatLonDegrees, ArgumentIssue]:
    if "/RW" in name:
        airport, runway_text = name.split("/RW", maxsplit=1)
        runway = runway_text.lstrip("Y")
        try:
            lat, lon, _heading = context.navigation.rwythresholds[airport][runway]
        except KeyError:
            return Err(
                ArgumentIssue.expected("a waypoint, airport, runway, or aircraft id", name, span)
            )
        return Ok(LatLonDegrees(float(lat), float(lon)))

    if name in context.navigation.aptid:
        index = context.navigation.aptid.index(name)
        return Ok(
            LatLonDegrees(
                float(context.navigation.aptlat[index]),
                float(context.navigation.aptlon[index]),
            )
        )

    occurrences = context.navigation.wpid.count(name)
    if occurrences > 1:
        return Err(
            ArgumentIssue.expected("an unambiguous waypoint id or explicit coordinates", name, span)
        )
    if occurrences == 1:
        index = context.navigation.wpid.index(name)
        return Ok(
            LatLonDegrees(
                float(context.navigation.wplat[index]),
                float(context.navigation.wplon[index]),
            )
        )
    return Err(ArgumentIssue.expected("a waypoint, airport, runway, or aircraft id", name, span))


def parse_lat_lon(context: CommandParseContext, text: str) -> ParseResult[LatLonDegrees]:
    """Resolve coordinates, aircraft, airports, runways, or waypoints to lat/lon."""
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    name = token.value.upper()
    remainder = token.remainder

    index = context.traffic.idx(name)
    if index >= 0:
        coordinates = LatLonDegrees(
            float(context.traffic.lat[index]), float(context.traffic.lon[index])
        )
        return Ok(Parsed(coordinates, remainder, token.span))

    if islat(name):
        offset = len(text) - len(remainder)
        if isinstance(longitude_result := next_argument(remainder), Err):
            issue = longitude_result.err()
            span = issue.span.offset_by(offset) if issue.span is not None else None
            return Err(ArgumentIssue(issue.message, span))
        longitude = longitude_result.ok()
        span = SourceSpan(token.span.start, offset + longitude.span.end)
        if not longitude.value:
            return Err(
                ArgumentIssue.expected("a longitude after the latitude", "end of input", span)
            )
        try:
            coordinates = LatLonDegrees(txt2lat(name), txt2lon(longitude.value))
        except ValueError:
            return Err(
                ArgumentIssue.expected(
                    "a latitude and longitude", f"{name},{longitude.value}", span
                )
            )
        return Ok(Parsed(coordinates, longitude.remainder, span))

    span = token.span
    if remainder[:2].upper() == "RW" and name in context.navigation.aptid:
        offset = len(text) - len(remainder)
        if isinstance(runway_result := next_argument(remainder), Err):
            issue = runway_result.err()
            issue_span = issue.span.offset_by(offset) if issue.span is not None else None
            return Err(ArgumentIssue(issue.message, issue_span))
        runway = runway_result.ok()
        name = f"{name}/{runway.value.upper()}"
        span = SourceSpan(token.span.start, offset + runway.span.end)
        remainder = runway.remainder

    if isinstance(resolved := _resolve_named_lat_lon(context, name, span), Err):
        return resolved
    return Ok(Parsed(resolved.ok(), remainder, span))


LatLonDeg = Annotated[LatLonDegrees, CmdParser(parse_lat_lon)]


_Constraint: TypeAlias = Gt | Ge | Le | Predicate


@dataclass(frozen=True, slots=True)
class Parameter:
    """A callback parameter compiled to a typed command parser."""

    name: str
    parser: CmdParser[Any]
    constraints: tuple[_Constraint, ...] = ()
    nullable: bool = False
    default: object = inspect.Parameter.empty

    def parse(
        self,
        context: CommandParseContext,
        text: str,
        *,
        source_text: str,
        offset: int,
    ) -> ParseResult[Any]:
        if not text and self.default is not inspect.Parameter.empty:
            return Ok(Parsed(self.default, text, SourceSpan(0, 0)))

        index = len(text) - len(text.lstrip())
        if text[index : index + 1] == ",":
            omitted = next_argument(text)
            assert isinstance(omitted, Ok)
            field = omitted.ok()
            if self.nullable:
                return Ok(Parsed(None, field.remainder, field.span))
            if self.default is not inspect.Parameter.empty:
                return Ok(Parsed(self.default, field.remainder, field.span))

        if isinstance(result := self.parser(context, text), Err):
            issue = result.err()
            fallback = SourceSpan(0, max(1, len(text)))
            return Err(issue.at_argument(self.name, source_text, offset, fallback))

        parsed = result.ok()
        if isinstance(validation := _validate_constraints(parsed.value, self.constraints), Err):
            return Err(validation.err().at_argument(self.name, source_text, offset, parsed.span))
        return result

    def __str__(self) -> str:
        if isinstance(self.parser.syntax, LiteralSyntax):
            text = "|".join(self.parser.syntax.values)
        else:
            text = self.name
        if self.default is not inspect.Parameter.empty or self.nullable:
            return f"[{text}]"
        return text


def compile_parameter(parameter: inspect.Parameter) -> Parameter:
    """Compile the parser and constraints carried by an annotation."""
    annotation = parameter.annotation
    nullable = False
    union_members = get_args(annotation)
    if type(None) in union_members:
        non_null = tuple(member for member in union_members if member is not type(None))
        if len(non_null) != 1:
            raise TypeError(f"command parameter {parameter.name!r} has unsupported nullable union")
        annotation = non_null[0]
        nullable = True

    metadata: tuple[object, ...] = ()
    if get_origin(annotation) is Annotated:
        annotation, *raw_metadata = get_args(annotation)
        metadata = tuple(raw_metadata)

    parsers = tuple(item for item in metadata if isinstance(item, CmdParser))
    if len(parsers) > 1:
        raise TypeError(f"command parameter {parameter.name!r} declares multiple parsers")
    parser = parsers[0] if parsers else _parser_for(annotation)
    if parser is None:
        raise TypeError(f"command parameter {parameter.name!r} has no typed parser")

    constraints: list[_Constraint] = []
    for item in metadata:
        if isinstance(item, (Gt, Ge, Le, Predicate)):
            constraints.append(item)
        elif isinstance(item, BaseMetadata):
            raise TypeError(f"unsupported command constraint: {item!r}")

    if parameter.default is None and not nullable:
        raise TypeError(
            f"command parameter {parameter.name!r} defaults to None but is not nullable"
        )
    if (
        nullable
        and parameter.default is not inspect.Parameter.empty
        and parameter.default is not None
    ):
        raise TypeError(f"nullable command parameter {parameter.name!r} must default to None")

    return Parameter(
        parameter.name,
        parser,
        tuple(constraints),
        nullable=nullable,
        default=parameter.default,
    )


# TODO(abraham): add structured union-choice parsing
def _parser_for(annotation: object) -> CmdParser[Any] | None:
    if get_origin(annotation) is Literal:
        return _literal_parser(annotation)
    if annotation is int:
        return CmdParser(parse_int)
    if annotation is float:
        return CmdParser(parse_float)
    return None


def _literal_parser(annotation: object) -> CmdParser[str]:
    values = get_args(annotation)
    if not values or any(not isinstance(value, str) for value in values):
        raise TypeError("command Literal values must be strings")
    literals = tuple(value.upper() for value in values)
    if len(literals) != len(set(literals)):
        raise TypeError("command Literal repeats a case-insensitive value")

    def parse_literal(_context: CommandParseContext, text: str) -> ParseResult[str]:
        if isinstance(result := next_argument(text), Err):
            return result
        token = result.ok()
        value = token.value.upper()
        if value not in literals:
            return Err(
                ArgumentIssue.expected(" or ".join(literals), value or "empty input", token.span)
            )
        return Ok(Parsed(value, token.remainder, token.span))

    return CmdParser.literals(parse_literal, literals)


def _validate_constraints(
    value: Any, constraints: tuple[_Constraint, ...]
) -> Result[None, ArgumentIssue]:
    # TODO(abraham): add more annotated-types constraints
    for constraint in constraints:
        expected: str | None = None
        if isinstance(constraint, Gt) and not value > constraint.gt:
            expected = f"a value greater than {constraint.gt}"
        elif isinstance(constraint, Ge) and not value >= constraint.ge:
            expected = f"a value greater than or equal to {constraint.ge}"
        elif isinstance(constraint, Le) and not value <= constraint.le:
            expected = f"a value less than or equal to {constraint.le}"
        elif isinstance(constraint, Predicate) and not constraint.func(value):
            name = getattr(constraint.func, "__name__", "predicate")
            expected = f"a value satisfying {name}"
        if expected is not None:
            return Err(ArgumentIssue.expected(expected, value))
    return Ok(None)


@dataclass(frozen=True, slots=True)
class CommandDeclaration:
    """Static command metadata stored on a decorated callable."""

    name: str = ""
    aliases: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", normalize_public_name(self.name) if self.name else "")
        object.__setattr__(
            self,
            "aliases",
            tuple(normalize_public_name(alias) for alias in self.aliases),
        )


@dataclass(frozen=True, slots=True)
class BoundCommand:
    """A command declaration paired with its runtime-bound callback."""

    callback: CommandCallback
    source: CommandCallback
    declaration: CommandDeclaration

    @property
    def name(self) -> str:
        return self.declaration.name or normalize_public_name(self.source.__name__)

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
    """Declare a callable as a typed stack command."""

    def decorate(target: CommandTarget) -> CommandTarget:
        source = _underlying_function(target)
        if _COMMAND in vars(source):
            raise TypeError("a typed stack command may be declared only once")
        setattr(source, _COMMAND, CommandDeclaration(name, aliases))
        return target

    return decorate(func) if func is not None else decorate


def declared_commands(component: object) -> Iterator[BoundCommand]:
    """Yield typed command declarations bound to a component instance."""
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
