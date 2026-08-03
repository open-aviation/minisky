"""Typed stack-command declarations and parsers."""

from __future__ import annotations

import inspect
from collections.abc import Callable, Iterator
from dataclasses import dataclass, replace
from typing import (
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

from annotated_types import BaseMetadata, Ge, Gt, IsFinite, Le, Predicate

from minisky.identifiers import normalize_public_name
from minisky.result import Err, Ok, Result
from minisky.tools.convert import txt2bool, txt2tim

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
ParseFunction: TypeAlias = Callable[[str], ParseResult[ParsedT_co]]


def next_argument(text: str) -> ParseResult[str]:
    """Parse one command field and leave the remaining fields untouched."""
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

    def __call__(self, text: str) -> ParseResult[ParsedT_co]:
        return self.func(text)


def parse_text(text: str) -> ParseResult[str]:
    """Consume the complete remaining command text verbatim."""
    return Ok(Parsed(text, "", SourceSpan(0, len(text))))


Text = Annotated[str, CmdParser(parse_text)]
"""The complete remaining command text, consumed verbatim."""


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


def parse_int(text: str) -> ParseResult[int]:
    """Parse one integer command field."""
    return _convert(text, int, "an integer")


def parse_float(text: str) -> ParseResult[float]:
    """Parse one floating-point command field."""
    return _convert(text, float, "a number")


def parse_on_off(text: str) -> ParseResult[bool]:
    """Parse one on/off command field."""
    return _convert(text, txt2bool, "ON or OFF")


OnOff = Annotated[bool, CmdParser(parse_on_off)]


def parse_time(text: str) -> ParseResult[float]:
    """Parse one BlueSky time field as seconds."""
    return _convert(text, txt2tim, "a time")


TimeS = Annotated[float, CmdParser(parse_time)]


_Constraint: TypeAlias = Gt | Ge | Le | Predicate


@dataclass(frozen=True, slots=True)
class Parameter:
    """A callback parameter compiled to a typed command parser."""

    name: str
    parser: CmdParser[Any]
    constraints: tuple[_Constraint, ...] = ()

    def parse(self, text: str, *, source_text: str, offset: int) -> ParseResult[Any]:
        if isinstance(result := self.parser(text), Err):
            issue = result.err()
            # TODO(abraham): move missing/default semantics here
            fallback = SourceSpan(0, max(1, len(text)))
            return Err(issue.at_argument(self.name, source_text, offset, fallback))

        parsed = result.ok()
        if isinstance(validation := _validate_constraints(parsed.value, self.constraints), Err):
            return Err(validation.err().at_argument(self.name, source_text, offset, parsed.span))
        return result

    def __str__(self) -> str:
        if isinstance(self.parser.syntax, LiteralSyntax):
            return "|".join(self.parser.syntax.values)
        return self.name


def compile_parameter(parameter: inspect.Parameter) -> Parameter:
    """Compile the parser and constraints carried by one annotation."""
    annotation = parameter.annotation
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

    return Parameter(parameter.name, parser, tuple(constraints))


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

    def parse_literal(text: str) -> ParseResult[str]:
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
    seen: set[str] = set()
    for cls in type(component).__mro__:
        for name, value in vars(cls).items():
            if name in seen:
                continue
            seen.add(name)
            source = _underlying_function(value)
            declaration = getattr(source, _COMMAND, None) if callable(source) else None
            if declaration is None:
                continue
            if not isinstance(declaration, CommandDeclaration):
                raise TypeError(f"invalid typed command declaration on {name!r}")
            callback = getattr(component, name)
            if not inspect.ismethod(callback) or callback.__self__ is not component:
                raise TypeError(f"decorated command {name!r} must be an instance method")
            yield BoundCommand(callback, source, declaration)


def _underlying_function(value: Any) -> Any:
    if isinstance(value, (staticmethod, classmethod)):
        value = value.__func__
    return inspect.unwrap(value) if callable(value) else value
