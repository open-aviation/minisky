"""Decorators for plugin commands and simulation hooks.

The decorators store metadata only. Typed plugins mount an instance with
[PluginContext][minisky.plugin.plugin.PluginContext] before MiniSky binds its
declarations to that runtime.
"""
# TODO(abraham): delete the legacy module scanner along with `init_plugin(runtime)`

from __future__ import annotations

import inspect
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from types import ModuleType
from typing import TYPE_CHECKING, Any, Literal, TypeVar, overload

from minisky.identifiers import normalize_public_name

if TYPE_CHECKING:
    from minisky.core.trafficarrays import TrafficArrays
    from minisky.stack import CommandStack, PreparedCommand

#
# commands
#

CommandCallback = Callable[..., Any]
CommandTarget = TypeVar("CommandTarget", bound=CommandCallback)
_COMMAND = "__minisky_command__"


@dataclass(frozen=True, slots=True)
class CommandDeclaration:
    """Command metadata stored on a decorated method."""

    arguments: str = ""
    name: str = ""
    aliases: tuple[str, ...] = ()
    brief: str = ""
    help: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "name", normalize_public_name(self.name) if self.name else "")
        object.__setattr__(
            self,
            "aliases",
            tuple(normalize_public_name(alias) for alias in self.aliases),
        )


@dataclass(frozen=True, slots=True)
class BoundCommand:
    """A command declaration bound to a component instance."""

    callback: CommandCallback
    declaration: CommandDeclaration

    @property
    def name(self) -> str:
        return self.declaration.name or normalize_public_name(self.callback.__name__)

    @property
    def aliases(self) -> tuple[str, ...]:
        return self.declaration.aliases

    @property
    def brief(self) -> str:
        if self.declaration.brief:
            return self.declaration.brief
        parameters: list[str] = []
        for parameter in inspect.signature(self.callback).parameters.values():
            name = parameter.name
            if parameter.kind is inspect.Parameter.VAR_POSITIONAL:
                name = f"{name},..."
            if parameter.default is not inspect.Parameter.empty:
                name = f"[{name}]"
            parameters.append(name)
        suffix = f" {','.join(parameters)}" if parameters else ""
        return f"{self.name}{suffix}"

    @property
    def help(self) -> str:
        return self.declaration.help or inspect.cleandoc(inspect.getdoc(self.callback) or "")


@overload
def command(func: CommandTarget, /) -> CommandTarget: ...


@overload
def command(
    *,
    arguments: str = "",
    name: str = "",
    aliases: tuple[str, ...] = (),
    brief: str = "",
    help: str = "",
) -> Callable[[CommandTarget], CommandTarget]: ...


def command(
    func: CommandTarget | None = None,
    /,
    *,
    arguments: str = "",
    name: str = "",
    aliases: tuple[str, ...] = (),
    brief: str = "",
    help: str = "",
) -> CommandTarget | Callable[[CommandTarget], CommandTarget]:
    """Declare an instance method as a stack command.

    The method name becomes the command name unless `name` is provided. Its
    docstring becomes command help and its signature becomes the brief usage
    text unless those values are passed explicitly for compatibility.
    """

    def decorate(target: CommandTarget) -> CommandTarget:
        actual = _underlying_function(target)
        if _COMMAND in vars(actual):
            raise TypeError("a plugin command may be declared only once")
        setattr(actual, _COMMAND, CommandDeclaration(arguments, name, aliases, brief, help))
        return target

    return decorate(func) if func is not None else decorate


def declared_commands(component: object) -> Iterator[BoundCommand]:
    """Bind command declarations to this exact component instance."""
    for attribute_name, value in _declaration_namespace(component).items():
        declaration = getattr(_underlying_function(value), _COMMAND, None)
        if isinstance(declaration, CommandDeclaration):
            yield BoundCommand(_bound_method(component, attribute_name, "command"), declaration)


#
# hooks
#

HookCallback = Callable[..., Any]
HookTarget = TypeVar("HookTarget", bound=HookCallback)
HookName = Literal["preupdate", "update", "reset", "hold"]
_HOOKS = "__minisky_hooks__"
_HOOK_NAMES = frozenset({"preupdate", "update", "reset", "hold"})


@dataclass(frozen=True, slots=True)
class HookDeclaration:
    """Simulation-hook metadata stored on a decorated method."""

    hook: HookName | None = None
    interval: float = 0.0
    name: str = ""


@dataclass(frozen=True, slots=True)
class BoundHook:
    """A hook declaration bound to a component instance."""

    callback: HookCallback
    declaration: HookDeclaration

    @property
    def hook(self) -> HookName:
        value = self.declaration.hook or self.callback.__name__.lower()
        if value not in _HOOK_NAMES:
            raise ValueError(
                f"cannot infer plugin hook from {self.callback.__name__!r}; "
                "specify preupdate, update, reset, or hold"
            )
        return value  # type: ignore[return-value]

    @property
    def name(self) -> str:
        return self.declaration.name or self.callback.__name__


@overload
def hook(func: HookTarget, /) -> HookTarget: ...


@overload
def hook(
    hook_name: HookName | None = None,
    /,
    *,
    interval: float = 0.0,
    name: str = "",
) -> Callable[[HookTarget], HookTarget]: ...


def hook(
    func_or_name: HookTarget | HookName | None = None,
    /,
    *,
    interval: float = 0.0,
    name: str = "",
) -> HookTarget | Callable[[HookTarget], HookTarget]:
    """Declare a synchronous simulation hook on an instance method."""

    def decorate(target: HookTarget, hook_name: HookName | None) -> HookTarget:
        actual = _underlying_function(target)
        declarations = tuple(getattr(actual, _HOOKS, ()))
        setattr(actual, _HOOKS, (*declarations, HookDeclaration(hook_name, interval, name)))
        return target

    if callable(func_or_name):
        return decorate(func_or_name, None)
    return lambda target: decorate(target, func_or_name)


def declared_hooks(component: object) -> Iterator[BoundHook]:
    """Bind hook declarations to this exact component instance."""
    for attribute_name, value in _declaration_namespace(component).items():
        declarations = getattr(_underlying_function(value), _HOOKS, ())
        if not declarations:
            continue
        callback = _bound_method(component, attribute_name, "hook")
        for declaration in declarations:
            if not isinstance(declaration, HookDeclaration):
                raise TypeError(f"invalid hook declaration on {attribute_name!r}")
            yield BoundHook(callback, declaration)


#
# replacements
#

ReplacementTarget = TypeVar("ReplacementTarget", bound=type[Any])
_REPLACEMENT = "__minisky_replacement__"


@dataclass(frozen=True, slots=True)
class ReplacementDeclaration:
    """Replacement metadata stored on a decorated class."""

    base: type[TrafficArrays] | None = None
    name: str = ""


@overload
def replacement(target: ReplacementTarget, /) -> ReplacementTarget: ...


@overload
def replacement(
    *,
    base: type[TrafficArrays] | None = None,
    name: str = "",
) -> Callable[[ReplacementTarget], ReplacementTarget]: ...


def replacement(
    target: ReplacementTarget | None = None,
    /,
    *,
    base: type[TrafficArrays] | None = None,
    name: str = "",
) -> ReplacementTarget | Callable[[ReplacementTarget], ReplacementTarget]:
    """Declare a runtime-local traffic implementation."""

    def decorate(implementation: ReplacementTarget) -> ReplacementTarget:
        if _REPLACEMENT in vars(implementation):
            raise TypeError(f"replacement already declared: {implementation.__name__}")
        setattr(implementation, _REPLACEMENT, ReplacementDeclaration(base, name))
        return implementation

    return decorate(target) if target is not None else decorate


def declared_replacement(implementation: type[Any]) -> ReplacementDeclaration:
    """Return replacement metadata for an explicitly declared class."""
    declaration = vars(implementation).get(_REPLACEMENT)
    if not isinstance(declaration, ReplacementDeclaration):
        raise TypeError(f"replacement {implementation.__name__!r} must use @plugin.replacement")
    return declaration


#
# legacy commands
#


def prepare_declared_commands(
    command_stack: CommandStack, module: ModuleType
) -> tuple[PreparedCommand, ...]:
    """Construct declarations from a legacy plugin module."""
    commands: list[PreparedCommand] = []
    for value in vars(module).values():
        callback = _underlying_function(value)
        declaration = getattr(callback, _COMMAND, None)
        if not isinstance(declaration, CommandDeclaration):
            continue
        bound = BoundCommand(callback, declaration)
        commands.append(
            command_stack.prepare_command(
                callback,
                name=bound.name,
                aliases=bound.aliases,
                arguments=declaration.arguments,
                brief=bound.brief,
                help=bound.help,
            )
        )
    return tuple(commands)


def register_declared_commands(command_stack: CommandStack, module: ModuleType) -> None:
    """Register command declarations from a legacy plugin module."""
    commands = prepare_declared_commands(command_stack, module)
    command_stack.validate_commands(commands)
    command_stack.install_commands(commands)


def prepare_commands(
    command_stack: CommandStack,
    newcommands: dict[str, list[Any] | tuple[Any, ...]],
    syndict: dict[str, list[str]] | None = None,
) -> tuple[PreparedCommand, ...]:
    """Construct commands from the original plugin return format."""
    synonyms = syndict or {}
    commands: list[PreparedCommand] = []
    for name, values in newcommands.items():
        function = values[0]
        arguments = values[1] if len(values) > 1 else ""
        brief = values[2] if len(values) > 2 else ""
        help_text = values[3] if len(values) > 3 else ""
        commands.append(
            command_stack.prepare_command(
                function,
                name=name,
                arguments=arguments,
                brief=brief,
                help=help_text,
                aliases=tuple(synonyms.get(name, ())),
            )
        )
    return tuple(commands)


def append_commands(
    command_stack: CommandStack,
    newcommands: dict[str, list[Any] | tuple[Any, ...]],
    syndict: dict[str, list[str]] | None = None,
) -> None:
    """Append commands from the original plugin return format."""
    commands = prepare_commands(command_stack, newcommands, syndict)
    command_stack.validate_commands(commands)
    command_stack.install_commands(commands)


def _bound_method(component: object, name: str, kind: str) -> Callable[..., Any]:
    callback = getattr(component, name)
    if not inspect.ismethod(callback) or callback.__self__ is not component:
        raise TypeError(f"decorated {kind} {name!r} must be an instance method")
    return callback


def _declaration_namespace(component: object) -> dict[str, Any]:
    namespace: dict[str, Any] = {}
    for cls in reversed(type(component).__mro__):
        namespace.update(vars(cls))
    return namespace


def _underlying_function(value: Any) -> Any:
    if isinstance(value, (staticmethod, classmethod)):
        value = value.__func__
    return inspect.unwrap(value) if callable(value) else value
