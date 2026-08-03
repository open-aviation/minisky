"""Typed stack-command declarations and parsers."""

from __future__ import annotations

import inspect
from collections.abc import Callable, Iterator
from dataclasses import dataclass, replace
from typing import Annotated, Any, Generic, TypeAlias, TypeVar, get_args, get_origin, overload

from minisky.identifiers import normalize_public_name
from minisky.result import Err, Ok, Result

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
class CmdParser(Generic[ParsedT_co]):
    """Connect an annotated Python value to command-text parsing."""

    func: ParseFunction[ParsedT_co]

    def __call__(self, text: str) -> ParseResult[ParsedT_co]:
        return self.func(text)


def parse_text(text: str) -> ParseResult[str]:
    """Consume the complete remaining command text verbatim."""
    return Ok(Parsed(text, "", SourceSpan(0, len(text))))


Text = Annotated[str, CmdParser(parse_text)]
"""The complete remaining command text, consumed verbatim."""


def parse_int(text: str) -> ParseResult[int]:
    """Parse one integer command field."""
    if isinstance(result := next_argument(text), Err):
        return result
    token = result.ok()
    try:
        value = int(token.value)
    except ValueError:
        return Err(ArgumentIssue.expected("an integer", token.value or "empty input", token.span))
    return Ok(Parsed(value, token.remainder, token.span))


@dataclass(frozen=True, slots=True)
class Parameter:
    """A callback parameter compiled to a typed command parser."""

    name: str
    parser: CmdParser[Any]

    def parse(self, text: str, *, source_text: str, offset: int) -> ParseResult[Any]:
        if isinstance(result := self.parser(text), Err):
            issue = result.err()
            # TODO(abraham): missing/default handling live here once optional fields exist?
            fallback = SourceSpan(0, max(1, len(text)))
            return Err(issue.at_argument(self.name, source_text, offset, fallback))
        return result

    def __str__(self) -> str:
        return self.name


def compile_parameter(parameter: inspect.Parameter) -> Parameter:
    """Compile the parser metadata carried by one annotated parameter."""
    annotation = parameter.annotation
    if annotation is int:
        return Parameter(parameter.name, CmdParser(parse_int))
    if get_origin(annotation) is not Annotated:
        raise TypeError(f"command parameter {parameter.name!r} has no typed parser")

    parsers = tuple(
        metadata for metadata in get_args(annotation)[1:] if isinstance(metadata, CmdParser)
    )
    if len(parsers) != 1:
        raise TypeError(f"command parameter {parameter.name!r} must declare exactly one parser")
    return Parameter(parameter.name, parsers[0])


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
