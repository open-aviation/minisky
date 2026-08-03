"""Typed stack-command declarations and parsers."""

from __future__ import annotations

import inspect
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from typing import Annotated, Any, Generic, TypeAlias, TypeVar, get_args, get_origin, overload

from minisky.identifiers import normalize_public_name
from minisky.result import Ok, Result

CommandCallback = Callable[..., Any]
CommandTarget = TypeVar("CommandTarget", bound=CommandCallback)
ParsedT_co = TypeVar("ParsedT_co", covariant=True)
_COMMAND = "__minisky_command__"


@dataclass(frozen=True, slots=True)
class SourceSpan:
    """Half-open source range for a parsed value."""

    start: int
    end: int


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

    def __str__(self) -> str:
        return self.message


ParseResult: TypeAlias = Result[Parsed[ParsedT_co], ArgumentIssue]
ParseFunction: TypeAlias = Callable[[str], ParseResult[ParsedT_co]]


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


@dataclass(frozen=True, slots=True)
class Parameter:
    """A callback parameter compiled to a typed command parser."""

    name: str
    parser: CmdParser[Any]

    def parse(self, text: str) -> ParseResult[Any]:
        return self.parser(text)

    def __str__(self) -> str:
        return self.name


def compile_parameter(parameter: inspect.Parameter) -> Parameter:
    """Compile the parser metadata carried by one annotated parameter."""
    annotation = parameter.annotation
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
