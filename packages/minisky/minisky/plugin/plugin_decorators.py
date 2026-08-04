"""Decorators for plugin hooks and replacements.

The decorators store metadata only. Typed plugins mount an instance with
[PluginContext][minisky.plugin.plugin.PluginContext] before MiniSky binds its
declarations to that runtime.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Literal, TypeVar, overload

from minisky.command import _bound_method, _declared_attributes, _underlying_function

if TYPE_CHECKING:
    from minisky.core.trafficarrays import TrafficArrays

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
