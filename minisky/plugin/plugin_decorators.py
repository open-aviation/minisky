"""Stack command declarations for MiniSky plugins.

The `@command` decorator stores command metadata on a function. It registers
immediately when a runtime is active; otherwise the declaration is collected
by `CommandStack.init()` when the runtime is constructed.
"""

import inspect
import sys
from collections.abc import Callable
from typing import Any


def command(
    func: Callable[..., Any] | None = None,
    name: str = "",
    aliases: tuple[str, ...] = (),
    brief: str = "",
    help: str = "",
    arguments: str = "",
) -> Any:
    """Decorator to register a function as a stack command.

    Args:
        func: The function to decorate (can be omitted for @command() style)
        name: Command name (defaults to function name in uppercase)
        aliases: Tuple of command aliases
        brief: Brief usage string
        help: Detailed help text
        arguments: Argument specification string (e.g., "callsign,alt,[spd]")

    Example:
        from minisky.stack.argparser import Txt

        @command
        def mycommand(arg1: Txt, arg2: int = 5):
            '''Help text for mycommand.'''
            return True, "Success"

        @command(name='MYCMD', aliases=('MC',))
        def my_command(arg: str):
            '''Help text.'''
            return True, "Done"

    Returns:
        The original function (unmodified)
    """

    def deco(func):
        actual_func = func.__func__ if isinstance(func, (staticmethod, classmethod)) else func
        declaration = {
            "name": name or actual_func.__name__,
            "aliases": aliases,
            "brief": brief,
            "help": help or inspect.cleandoc(inspect.getdoc(actual_func) or ""),
            "arguments": arguments,
        }
        actual_func.__stack_command__ = declaration  # type: ignore[reportFunctionMemberAccess]

        try:
            from minisky.stack import Command, current

            current()
        except (ImportError, RuntimeError):
            return func

        Command.addcommand(actual_func, **declaration)
        return func

    # Allow both @command and @command(args)
    return deco(func) if func else deco


def register_declared_commands() -> None:
    """Register command declarations from modules imported before runtime startup."""
    from minisky.stack import Command

    for module in tuple(sys.modules.values()):
        if module is None:
            continue
        for value in vars(module).values():
            actual_func = (
                value.__func__ if isinstance(value, (staticmethod, classmethod)) else value
            )
            declaration = getattr(actual_func, "__stack_command__", None)
            if declaration is not None:
                Command.addcommand(actual_func, **declaration)


def append_commands(newcommands: dict, syndict: dict | None = None) -> None:
    """Append additional functions to the stack command dictionary.

    Used by plugin loader to register plugin commands.

    Args:
        newcommands: Dict of command name -> [function, arguments, brief, help]
        syndict: Optional dict of command name -> list of synonyms
    """
    # Import here to avoid circular import
    from minisky.stack import Command

    syndict = syndict or {}

    for name, values in newcommands.items():
        if len(values) >= 4:
            function, arguments, brief, help_text = values[:4]
        else:
            function = values[0]
            arguments = values[1] if len(values) > 1 else ""
            brief = values[2] if len(values) > 2 else ""
            help_text = values[3] if len(values) > 3 else ""

        Command.addcommand(
            function,
            name=name,
            arguments=arguments,
            brief=brief,
            help=help_text,
            aliases=syndict.get(name, []),
        )
