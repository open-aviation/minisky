"""Stack command declarations for MiniSky plugins.

The `@command` decorator stores command metadata on a function. Importing a
plugin module does not register that command globally. Instead, the
[`PluginManager`][minisky.plugin.plugin.PluginManager] that loads the module
registers its declarations with the owning runtime's
[`CommandStack`][minisky.stack.CommandStack].
"""

from __future__ import annotations

import inspect
from collections.abc import Callable
from types import ModuleType
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from minisky.stack import CommandStack, PreparedCommand


def command(
    func: Callable[..., Any] | None = None,
    name: str = "",
    aliases: tuple[str, ...] = (),
    brief: str = "",
    help: str = "",
    arguments: str = "",
) -> Any:
    """Declare a function as a stack command.

    The declaration is stored on the function and registered only when the
    runtime-owned plugin manager loads the containing module. This keeps module
    import free of command-registry side effects while preserving the familiar
    decorator syntax.

    Args:
        func: Function to decorate. It may be omitted when using
            `@command(...)` syntax.
        name: Command name. Defaults to the function name.
        aliases: Alternative command names.
        brief: Brief usage string.
        help: Detailed help text. When omitted, the function docstring is used.
        arguments: Argument specification string, for example
            `callsign,alt,[spd]`.

    Example:
        from minisky import stack
        from minisky.stack.argparser import Txt

        @stack.command
        def mycommand(arg1: Txt, arg2: int = 5):
            '''Help text for mycommand.'''
            return True, "Success"

        @stack.command(name="MYCMD", aliases=("MC",))
        def my_command(arg: str):
            '''Help text.'''
            return True, "Done"

    Returns:
        The original function or descriptor, unmodified apart from the stored
        declaration metadata.
    """

    def deco(declared: Callable[..., Any]) -> Any:
        # Static and class methods store their declaration on the underlying
        # function so the plugin loader can inspect them uniformly.
        actual_func = (
            declared.__func__ if isinstance(declared, (staticmethod, classmethod)) else declared
        )
        declaration = {
            "name": name or actual_func.__name__,
            "aliases": aliases,
            "brief": brief,
            "help": help or inspect.cleandoc(inspect.getdoc(actual_func) or ""),
            "arguments": arguments,
        }
        actual_func.__stack_command__ = declaration  # type: ignore[reportFunctionMemberAccess]
        return declared

    # Allow both `@command` and `@command(...)` forms.
    return deco(func) if func else deco


def prepare_declared_commands(
    command_stack: CommandStack, module: ModuleType
) -> tuple[PreparedCommand, ...]:
    """Construct command declarations from a plugin module."""
    commands: list[PreparedCommand] = []
    for value in vars(module).values():
        actual_func = value.__func__ if isinstance(value, (staticmethod, classmethod)) else value
        declaration = getattr(actual_func, "__stack_command__", None)
        if declaration is not None:
            commands.append(command_stack.prepare_command(actual_func, **declaration))
    return tuple(commands)


def register_declared_commands(command_stack: CommandStack, module: ModuleType) -> None:
    """Register command declarations from one plugin module.

    Only the supplied module is inspected. This is intentionally narrower than
    scanning `sys.modules`: loading a plugin into one runtime must not register
    commands imported for another runtime.

    Args:
        command_stack: Runtime-owned command registry that receives the
            declarations.
        module: Imported plugin module whose decorated functions are inspected.
    """
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
    """Append a plugin command dictionary to one runtime's command registry.

    This supports the original plugin return format in which `init_plugin`
    returns a second dictionary mapping command names to a callback, argument
    specification, brief usage text, and full help text.

    Args:
        command_stack: Runtime-owned command registry that receives the
            commands.
        newcommands: Mapping of command name to
            `[function, arguments, brief, help]`. Missing trailing values are
            treated as empty strings.
        syndict: Optional mapping of command name to aliases.
    """
    commands = prepare_commands(command_stack, newcommands, syndict)
    command_stack.validate_commands(commands)
    command_stack.install_commands(commands)
