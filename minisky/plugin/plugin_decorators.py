"""Stack command decorators for MiniSky plugins.

Provides the @command decorator for registering stack commands.
"""
import inspect


def command(func=None, name='', aliases=(), brief='', help='', arguments=''):
    """Decorator to register a function as a stack command.

    Args:
        func: The function to decorate (can be omitted for @command() style)
        name: Command name (defaults to function name in uppercase)
        aliases: Tuple of command aliases
        brief: Brief usage string
        help: Detailed help text
        arguments: Argument specification string (e.g., "callsign,alt,[spd]")

    Example:
        @command
        def mycommand(arg1: 'txt', arg2: int = 5):
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
        # Import here to avoid circular import
        from minisky.stack import Command

        # Get the underlying function if decorated with staticmethod/classmethod
        actual_func = func.__func__ if isinstance(func, (staticmethod, classmethod)) else func

        # Determine command name
        cmd_name = name or actual_func.__name__

        # Use function docstring as help if not provided
        cmd_help = help or inspect.cleandoc(inspect.getdoc(actual_func) or '')

        # Register the command
        Command.addcommand(
            actual_func,
            name=cmd_name,
            aliases=aliases,
            brief=brief,
            help=cmd_help,
            arguments=arguments
        )

        return func

    # Allow both @command and @command(args)
    return deco(func) if func else deco


def append_commands(newcommands, syndict=None):
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
            arguments = values[1] if len(values) > 1 else ''
            brief = values[2] if len(values) > 2 else ''
            help_text = values[3] if len(values) > 3 else ''

        Command.addcommand(
            function,
            name=name,
            arguments=arguments,
            brief=brief,
            help=help_text,
            aliases=syndict.get(name, [])
        )
