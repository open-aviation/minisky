"""Decorators for plugin commands, hooks, and replacements.

The decorators store metadata only. Typed plugins mount an instance with
[PluginContext][minisky.plugin.plugin.PluginContext] before MiniSky binds its
declarations to that runtime.
"""

from __future__ import annotations

import inspect
from collections.abc import Callable, Iterator
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, TypeVar, overload

from minisky.command import (
    _bound_method,
    _declared_attributes,
    _underlying_function,
)
from minisky.command import (
    command as typed_command,
)
from minisky.identifiers import normalize_public_name

if TYPE_CHECKING:
    from minisky.core.trafficarrays import TrafficArrays

#
# commands
#

CommandCallback = Callable[..., Any]
CommandTarget = TypeVar("CommandTarget", bound=CommandCallback)
_LEGACY_COMMAND = "__minisky_legacy_plugin_command__"
_ARGUMENTS_UNSET = object()


@dataclass(frozen=True, slots=True)
class LegacyCommandDeclaration:
    """Legacy plugin command metadata for the arguments DSL."""

    arguments: str
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
class BoundLegacyCommand:
    """A legacy command declaration bound to a component instance."""

    callback: CommandCallback
    declaration: LegacyCommandDeclaration

    @property
    def name(self) -> str:
        return self.declaration.name or normalize_public_name(self.callback.__name__)

    @property
    def aliases(self) -> tuple[str, ...]:
        return self.declaration.aliases

    @property
    def brief(self) -> str:
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
        return inspect.cleandoc(inspect.getdoc(self.callback) or "")


@overload
def command(func: CommandTarget, /) -> CommandTarget: ...


@overload
def command(
    *,
    name: str = "",
    aliases: tuple[str, ...] = (),
) -> Callable[[CommandTarget], CommandTarget]: ...


@overload
def command(
    *,
    arguments: str,
    name: str = "",
    aliases: tuple[str, ...] = (),
) -> Callable[[CommandTarget], CommandTarget]: ...


def command(
    func: CommandTarget | None = None,
    /,
    *,
    arguments: str | object = _ARGUMENTS_UNSET,
    name: str = "",
    aliases: tuple[str, ...] = (),
) -> CommandTarget | Callable[[CommandTarget], CommandTarget]:
    """Declare a typed plugin command, with temporary arguments-DSL compatibility."""
    if arguments is _ARGUMENTS_UNSET:
        typed_decorator = typed_command(name=name, aliases=aliases)
        return typed_decorator(func) if func is not None else typed_decorator
    if not isinstance(arguments, str):
        raise TypeError("plugin command arguments must be a string")

    def decorate_legacy(target: CommandTarget) -> CommandTarget:
        source = _underlying_function(target)
        if _LEGACY_COMMAND in vars(source):
            raise TypeError("a legacy plugin command may be declared only once")
        setattr(source, _LEGACY_COMMAND, LegacyCommandDeclaration(arguments, name, aliases))
        return target

    return decorate_legacy(func) if func is not None else decorate_legacy


def declared_legacy_commands(component: object) -> Iterator[BoundLegacyCommand]:
    """Bind explicit arguments-DSL declarations to this exact component instance."""
    for attribute_name, value in _declared_attributes(component):
        declaration = getattr(_underlying_function(value), _LEGACY_COMMAND, None)
        if isinstance(declaration, LegacyCommandDeclaration):
            yield BoundLegacyCommand(
                _bound_method(component, attribute_name, "legacy command"), declaration
            )


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
    for attribute_name, value in _declared_attributes(component):
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
