"""Griffe extension that labels `@command`-decorated methods in the API docs.

mkdocstrings does not render decorators, so nothing on an API page reveals that
a method is also reachable as a stack command. This extension reads the
`@command(...)` decorator that griffe already collects and prepends a short
admonition to the method's docstring, keeping the decorator the single source
of truth for the command name and its aliases.
"""

from __future__ import annotations

import ast
from typing import TYPE_CHECKING, Any

from griffe import Docstring, Extension
from minisky.identifiers import normalize_command_name

if TYPE_CHECKING:
    from griffe import Function, Inspector, Visitor

_DECORATOR = "command"
# Relative to the `docs/api/*.md` pages that render these docstrings.
_REFERENCE = "../reference/commands.md"


def _declaration(func: Function) -> tuple[str, tuple[str, ...]] | None:
    """Return the (name, aliases) declared by a `@command` decorator, if any."""
    for decorator in func.decorators:
        expression = str(decorator.value)
        # A bare `@command` takes the command name from the function name.
        if expression == _DECORATOR:
            return normalize_command_name(func.name), ()
        try:
            call = ast.parse(expression, mode="eval").body
        except SyntaxError:
            continue
        if not isinstance(call, ast.Call) or getattr(call.func, "id", None) != _DECORATOR:
            continue
        keywords: dict[str, Any] = {}
        for keyword in call.keywords:
            if keyword.arg is None:
                continue
            try:
                keywords[keyword.arg] = ast.literal_eval(keyword.value)
            except ValueError:
                return None
        name = keywords.get("name") or func.name
        aliases = tuple(keywords.get("aliases", ()))
        return normalize_command_name(name), tuple(map(normalize_command_name, aliases))
    return None


def _banner(name: str, aliases: tuple[str, ...]) -> str:
    label = f"`{name}`"
    if aliases:
        label += " (aliases: " + ", ".join(f"`{alias}`" for alias in aliases) + ")"
    return (
        f'!!! abstract "Stack command"\n\n'
        f"    Also callable from the stack as {label} — see the\n"
        f"    [stack command reference]({_REFERENCE}).\n"
    )


class StackCommands(Extension):
    """Prepend a stack-command admonition to every `@command` method."""

    def on_function_instance(
        self,
        *,
        node: ast.AST | Any,
        func: Function,
        agent: Visitor | Inspector,
        **kwargs: Any,
    ) -> None:
        """Inject the admonition when the function declares a stack command."""
        declaration = _declaration(func)
        if declaration is None:
            return
        banner = _banner(*declaration)
        if func.docstring is None:
            func.docstring = Docstring(banner, parent=func)
        else:
            func.docstring.value = f"{banner}\n{func.docstring.value}"
