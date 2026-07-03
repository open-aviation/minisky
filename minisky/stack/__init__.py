"""The stack parses all text-based commands in the simulation.

The stack is MiniSky's text-command interpreter. Every instruction to the
simulator - typed by a user, read from a scenario (.scn) file, or issued by
a plugin - enters as a line of text such as ``CRE KL204 B744 52.0 4.0 90
FL300 250``. Command lines are queued with :func:`stack` and executed once
per simulation step by :func:`process`.

Each available command is represented by a :class:`Command` object, which
couples the command name to the Python function that implements it and to
the argument parsers that convert argument text into typed values (see
:mod:`minisky.stack.argparser`). The base command set is defined in
:mod:`minisky.stack.commands` and registered in :func:`init`.

This module also implements scenario handling: :func:`ic` loads a scenario
file, whose timestamped command lines are buffered and moved onto the stack
by :func:`checkscen` when the simulation time passes their timestamps.
"""

import inspect
import os
import traceback
from io import StringIO
from pathlib import Path
from typing import Dict

import minisky
from minisky.stack import argparser, commands
from minisky.stack.argparser import ArgumentError, Parameter, getnextarg
from minisky.plugin.plugin_decorators import command, append_commands


def init():
    """Initialise BlueSky base stack commands."""

    cmddict, synonyms = commands.get_commands()

    # register command
    for name, values in cmddict.items():
        function, arguments, brief, help_text = values

        Command.addcommand(
            function,
            name=name,
            arguments=arguments,
            brief=brief,
            help=help_text,
            aliases=synonyms.get(name, []),
        )


class Command:
    """Stack command object.

    A Command wraps a Python callback function and makes it available as a
    text command in the simulator. It stores the command name, help texts,
    and aliases, and builds a list of Parameter objects that convert the
    raw argument text of a command line into typed Python arguments for
    the callback. Calling a Command instance with an argument string parses
    the arguments and executes the callback.

    All commands are stored in the class-level ``cmddict`` dictionary,
    which maps command names (and aliases) to Command instances.

    Attributes:
        name: Command name in upper case (e.g., "CRE").
        help: Full help text shown by the HELP command.
        brief: Brief usage text (command name plus argument list).
        aliases: Tuple of alternative names for this command.
        callback: The function that implements this command.
        params: List of Parameter objects used to parse arguments.
        valid: False when the callback is an unbound class/instance method.
    """

    # Dictionary with all command objects
    cmddict: Dict[str, "Command"] = dict()

    @classmethod
    def addcommand(cls, func, parent=None, name="", **kwargs):
        """Add 'func' as a stack command.

        Creates a Command object for the given function and registers it
        (and its aliases) in Command.cmddict. When a command with the same
        name already exists, the existing Command object is kept.

        Args:
            func: Function (or static/class method) implementing the command.
            parent: Optional parent command when this is a subcommand.
            name: Command name. Defaults to the function name, upper-cased.
            **kwargs: Command options: ``arguments`` (argument type
                specification string, e.g. "callsign,alt,[vspd]"),
                ``brief``, ``help``, and ``aliases``.
        """
        # Get function object if it's decorated as static or classmethod
        func = func.__func__ if isinstance(func, (staticmethod, classmethod)) else func
        # Stack command name
        name = (name or func.__name__).upper()

        # When a parent is passed this function is a subcommand
        target = Command.cmddict

        # Check if this command already exists
        cmdobj = target.get(name)
        if not cmdobj:
            cmdobj = cls(func, parent, name, **kwargs)
            target[name] = cmdobj
            for alias in cmdobj.aliases:
                target[alias] = cmdobj
        else:
            # for subclasses reimplementing stack functions we keep only one
            # Command object
            print(f"Attempt to reimplement {name} from {cmdobj.callback} to {func}")
            if not isinstance(cmdobj, cls):
                raise TypeError(
                    f"Error reimplementing {name}: "
                    f"A {type(cmdobj).__name__} cannot be "
                    f"reimplemented as a {cls.__name__}"
                )
        # Store reference to command object for function
        if not inspect.ismethod(func):
            func.__stack_cmd__ = cmdobj

    def __init__(self, func, parent=None, name="", **kwargs):
        self.name = name
        self.help = inspect.cleandoc(kwargs.get("help", ""))
        self.brief = kwargs.get("brief", "")
        self.aliases = kwargs.get("aliases", tuple())
        self.impl = ""
        self.valid = True
        self.arguments = self._get_arguments(kwargs.get("arguments", ""))
        self.params = list()
        self.parent = parent
        self.callback = func

    def __call__(self, argstring):
        """Parse an argument string and execute this command.

        The command's Parameter objects convert the argument text into
        typed values, which are passed to the callback function.

        Args:
            argstring: The command-line text following the command name.

        Returns:
            tuple: (success (bool), echotext (str)) describing the result.

        Raises:
            ArgumentError: When argument parsing fails, or when more
                arguments are given than the command accepts.
        """
        args = []
        param = None
        # Use callback-specified parameter parsers to generate param list from strings
        for param in self.params:
            result = param(argstring)
            argstring = result[-1]
            args.extend(result[:-1])

        # Parse repeating final args
        while argstring:
            if param is None or not param.gobble:
                msg = f"{self.name} takes {len(self.params)} argument"
                if len(self.params) > 1:
                    msg += "s"
                count = len(self.params)
                while argstring:
                    _, argstring = getnextarg(argstring)
                    count += 1
                msg += f", but {count} were given"
                raise ArgumentError(msg)
            result = param(argstring)
            argstring = result[-1]
            args.extend(result[:-1])

        # Call callback function with parsed parameters
        ret = self.callback(*args)
        # Always return a tuple with a success value and a message string
        if ret is None:
            return True, ""
        if isinstance(ret, (tuple, list)) and ret:
            if len(ret) > 1:
                # Assume that (success, echotext) is returned
                return ret[:2]
            ret = ret[0]
        return ret, ""

    def __repr__(self):
        if self.valid:
            return f"<Stack Command {self.name}, callback={self.callback}>"
        return f"<Stack Command {self.name} (invalid), callback=unbound method {self.callback}"

    def notimplemented(self, *args, **kwargs):
        """Placeholder callback for commands without an implementation."""
        pass

    @property
    def callback(self):
        """Callback pointing to the actual function that implements this
        stack command.
        """
        return self._callback

    @callback.setter
    def callback(self, function):
        self._callback = function
        spec = inspect.signature(function)
        # Check if this is an unbound class/instance method
        self.valid = (
            spec.parameters.get("self") is None and spec.parameters.get("cls") is None
        )

        if self.valid:
            # Store implementation origin
            if not self.impl:
                # Check if this is a bound (class or object) method
                if inspect.ismethod(function):
                    if inspect.isclass(function.__self__):
                        self.impl = function.__self__.__name__
                    else:
                        self.impl = function.__self__.__class__.__name__

            self.brief = self.brief or (self.name + " " + ",".join(spec.parameters))
            self.help = self.help or inspect.cleandoc(inspect.getdoc(function) or "")
            paramspecs = list(filter(Parameter.canwrap, spec.parameters.values()))
            if self.arguments:
                self.params = list()
                pos = 0
                for annot, isopt in self.arguments:
                    if annot == "...":
                        if paramspecs[-1].kind != paramspecs[-1].VAR_POSITIONAL:
                            raise IndexError(
                                "Repeating arguments (...) given for function"
                                " not ending in starred (variable-length) argument"
                            )
                        self.params[-1].gobble = True
                        break

                    param = Parameter(paramspecs[pos], annot, isopt)
                    if param:
                        pos = min(pos + param.size(), len(paramspecs) - 1)
                        self.params.append(param)
                if (
                    len(self.params) > len(paramspecs)
                    and paramspecs[-1].kind != paramspecs[-1].VAR_POSITIONAL
                ):
                    raise IndexError(
                        f"More arguments given than function "
                        f"{self.callback.__name__} has arguments."
                    )
            else:
                self.params = [p for p in map(Parameter, paramspecs) if p]

    def helptext(self, subcmd=""):
        """Return complete help text."""
        msg = f"{self.help}\nUsage:\n{self.brief}"
        if self.aliases:
            msg += "\nCommand aliases: " + ",".join(self.aliases)
        if self._callback.__name__ == "<lambda>":
            msg += "\nAnonymous (lambda) function, implemented in "
        else:
            msg += f"\nFunction {self._callback.__name__}(), implemented in "
        if hasattr(self._callback, "__code__"):
            fname = self._callback.__code__.co_filename
            fname_stripped = fname.replace(os.getcwd(), "").lstrip("/")
            firstline = self._callback.__code__.co_firstlineno
            msg += f"{fname_stripped} on line {firstline}"
        else:
            msg += f"module {self._callback.__module__}"

        return msg

    def brieftext(self):
        """Return the brief usage text."""
        return self.brief

    def _get_arguments(self, arguments):
        """Get arguments from string, or tuple/list."""
        if isinstance(arguments, (tuple, list)):
            return tuple(arguments)
        # Assume it is a comma-separated string
        argtypes = []

        # Process and reduce annotation string from left to right
        # First cut at square brackets, then take separate argument types
        while arguments:
            opt = arguments[0] == "["
            cut = (
                arguments.find("]")
                if opt
                else arguments.find("[")
                if "[" in arguments
                else len(arguments)
            )

            types = arguments[:cut].strip("[,]").split(",")
            # Returned argtypes are tuples of type and optional status
            argtypes += zip(types, [opt or t == "..." for t in types])
            arguments = arguments[cut:].lstrip(",]")

        return tuple(argtypes)


class Stack:
    """Stack static-only namespace.

    Holds the queue of pending command lines, as well as the commands and
    timestamps loaded from a scenario file. This class is never
    instantiated; all state is kept in class attributes.

    Attributes:
        current: Command line currently being processed.
        cmdstack: List of (cmdline, sender route) tuples awaiting processing.
        scenname: Name of the currently loaded scenario.
        scentime: Execution times [s] of the buffered scenario commands.
        scencmd: Buffered scenario command lines.
        sender_rte: Network route to the sender of the current command.
    """

    # Stack data
    current = ""
    cmdstack = []  # The actual stack: Current commands to be processed

    # Scenario details
    scenname = ""  # Currently used scenario name (for reading)
    scentime = []  # Times of the commands from the read scenario file
    scencmd = []  # Commands from the scenario file

    # Current command details
    sender_rte = None  # bs net route to sender

    @classmethod
    def reset(cls):
        """Reset stack variables."""
        cls.cmdstack = []
        cls.scenname = ""
        cls.scentime = []
        cls.scencmd = []
        cls.sender_rte = None

    @classmethod
    def commands(cls):
        """Generator function to iterate over stack commands."""
        # Return commands from PCALL if passed, otherwise own command stack
        for cls.current, cls.sender_rte in cls.cmdstack:
            yield cls.current

    @classmethod
    def clear(cls):
        """Remove all commands from the command stack."""
        cls.cmdstack.clear()


def delete_element(*arg):
    """DEL: Delete an element (aircraft, wind field, area shape, or group).

    Dispatches based on the first argument: the string "WIND" clears the
    wind field, any other string deletes the area with that name, a traffic
    group object deletes that group, and anything else is treated as
    aircraft indices to delete.

    Args:
        *arg: Element(s) to delete: "WIND", an area name, a traffic group,
            or one or more aircraft indices.

    Returns:
        The result of the dispatched delete function.
    """
    if isinstance(arg[0], str) and arg[0] == "WIND":
        return minisky.traf.wind.clear()
    elif isinstance(arg[0], str):
        return minisky.tools.areafilter.deleteArea(arg[0])
    elif hasattr(arg[0], "groupname"):
        return minisky.traf.groups.delgroup(arg[0])
    else:
        return minisky.traf.delete(arg)


def reset():
    """Reset the stack.

    Clears the command queue and buffered scenario data, and resets the
    argument-parser reference data (position, heading, speed).
    """
    Stack.reset()
    argparser.reset()


def process():
    """Sim-side stack processing; called once per simulation step.

    First moves due scenario commands onto the stack (see checkscen), then
    parses and executes every queued command line: the first word is looked
    up in Command.cmddict (an aircraft callsign may also be used as prefix,
    in which case the second word is the command, defaulting to POS), the
    remaining text is passed to the Command object for argument parsing and
    execution, and any resulting message is echoed to the screen. The
    command stack is cleared afterwards.
    """
    # First check for commands in scenario file
    checkscen()

    # Process stack of commands
    for cmdline in Stack.commands():
        success = True
        echotext = ""
        echoflags = minisky.BS_OK

        # Get first argument from command line and check if it's a command
        cmd, argstring = argparser.getnextarg(cmdline)
        cmdu = cmd.upper()
        cmdobj = Command.cmddict.get(cmdu)

        # If no function is found for 'cmd', check if cmd is actually an aircraft id
        if not cmdobj and cmdu in minisky.traf.callsign:
            cmd, argstring = argparser.getnextarg(argstring)
            argstring = cmdu + " " + argstring
            # When no other args are parsed, command is POS
            cmdu = cmd.upper() if cmd else "POS"
            cmdobj = Command.cmddict.get(cmdu)

        # Proceed if a command object was found
        if cmdobj:
            try:
                # Call the command, passing the argument string
                success, echotext = cmdobj(argstring)
                if not success:
                    if not argstring:
                        echotext = echotext or cmdobj.brieftext()
                    else:
                        echoflags = minisky.BS_FUNERR
                        echotext = f"Error: {echotext or cmdobj.brieftext()}"

            except argparser.ArgumentError as e:
                success = False
                echoflags = minisky.BS_ARGERR
                header = (
                    "" if not argstring else e.args[0] if e.args else "Argument error."
                )
                echotext = f"{header}\nUsage:\n{cmdobj.brieftext()}"
            except Exception as e:
                echoflags = minisky.BS_FUNERR
                header = (
                    "" if not argstring else e.args[0] if e.args else "Function error."
                )
                echotext = (
                    f"Error calling function implementation of {cmdu}: {header}\n"
                    + "Traceback printed to terminal."
                )
                traceback.print_exc()

        # Command not found
        else:
            success = False
            echoflags = minisky.BS_CMDERR
            if not argstring:
                echotext = f"error: unknown command or aircraft: {cmd}"
            else:
                echotext = f"error: unknown command: {cmd}"

        if echotext:
            minisky.scr.echo(echotext)

    # Clear the processed commands
    Stack.clear()


def readscn(scn):
    """Read a scenario file and yield its timestamped commands.

    Parses lines of the form ``HH:MM:SS.hh>CMDLINE``, skipping comments
    (lines starting with "#") and short lines, and supporting line
    continuation with a trailing backslash.

    Args:
        scn: Scenario source: path to a .scn file (str or Path; the .scn
            suffix is added when missing), or a StringIO object.

    Yields:
        tuple: (command time [s] (float), command line (str)).

    Raises:
        TypeError: When scn is neither a path nor a StringIO object.
    """
    if isinstance(scn, str) or isinstance(scn, Path):
        # ensure .scn suffix if necessary
        scn_path = Path(scn).with_suffix(".scn")

        with open(scn_path, "r") as fscen:
            scn_input = StringIO(fscen.read())
    elif isinstance(scn, StringIO):
        scn_input = scn
    else:
        raise TypeError("scn must be a string or StringIO")

    prevline = ""
    for line in scn_input:
        line = line.strip()
        # Skip emtpy lines and comments
        if len(line) < 12 or line[0] == "#":
            continue
        line = prevline + line

        # Check for line continuation
        if line[-1] == "\\":
            prevline = f"{line[:-1].strip()} "
            continue
        prevline = ""

        # Try reading timestamp and command
        try:
            icmdline = line.index(">")
            tstamp = line[:icmdline]
            ttxt = tstamp.strip().split(":")
            ihr = int(ttxt[0]) * 3600.0
            imin = int(ttxt[1]) * 60.0
            xsec = float(ttxt[2])
            cmdtime = ihr + imin + xsec

            yield (cmdtime, line[icmdline + 1 :].strip("\n"))
        except (ValueError, IndexError):
            # nice try, we will just ignore this syntax error
            if not (len(line.strip()) > 0 and line.strip()[0] == "#"):
                print("except this:" + line)


def ic(scn: str):
    """IC: Load a scenario file.

    Resets the simulation, reads the scenario file, and buffers its
    timestamped commands for execution when the simulation time passes
    their timestamps (see checkscen).

    Args:
        scn: The filename of the scenario, relative to the project root.

    Returns:
        tuple: (success (bool), message (str)).
    """

    minisky.sim.reset()

    scn = Path(__file__).parent.parent.parent / scn
    if not Path(scn).exists():
        return False, f"IC: File not found: {scn}"

    lines = readscn(scn)

    for cmdtime, cmd in lines:
        Stack.scentime.append(cmdtime)
        Stack.scencmd.append(cmd)
    Stack.scenname = scn.stem

    return True, f"scenario {scn} loaded."


def ic_StringIO(scn: StringIO, scn_name: str = None):
    """IC: Load a scenario from a StringIO object.

    Resets the simulation, reads scenario lines from the StringIO object,
    and buffers the timestamped commands for execution (see checkscen).

    Args:
        scn: StringIO object containing scenario lines.
        scn_name: The name of the scenario (optional).

    Returns:
        tuple: (success (bool), message (str)).
    """

    # reset sim always
    minisky.sim.reset()

    lines = readscn(scn)

    for cmdtime, cmd in lines:
        Stack.scentime.append(cmdtime)
        Stack.scencmd.append(cmd)
    Stack.scenname = scn_name

    return True, f"scenario {scn_name} loaded."


def scenario(name: "string"):
    """SCENARIO: Set the scenario name for the current simulation.

    Args:
        name: The name to give the scenario.

    Returns:
        tuple: (True, confirmation message).
    """
    Stack.scenname = name
    return True, "Starting scenario " + name


def schedule(time: "time", cmdline: "string"):
    """SCHEDULE: Schedule a stack command at a specific simulation time.

    The command is inserted into the scenario buffer, keeping the buffer
    sorted by execution time.

    Args:
        time: Absolute simulation time [s] at which the command should
            be executed.
        cmdline: The command line to be executed.

    Returns:
        bool: True (the command is always scheduled).
    """
    # Get index of first scentime greater than 'time' as insert position
    idx = next(
        (i for i, t in enumerate(Stack.scentime) if t > time), len(Stack.scentime)
    )
    Stack.scentime.insert(idx, time)
    Stack.scencmd.insert(idx, cmdline)
    return True


def delay(time: "time", cmdline: "string"):
    """DELAY: Delay a stack command by a time interval.

    Like schedule(), but the given time is relative to the current
    simulation time.

    Args:
        time: Time interval [s] by which the command should be delayed.
        cmdline: The command line to be executed after the delay.

    Returns:
        bool: True (the command is always scheduled).
    """
    # Get index of first scentime greater than 'time' as insert position
    time += minisky.sim.simt
    idx = next(
        (i for i, t in enumerate(Stack.scentime) if t > time), len(Stack.scentime)
    )
    Stack.scentime.insert(idx, time)
    Stack.scencmd.insert(idx, cmdline)
    return True


def showhelp(cmd: "txt" = "", subcmd: "txt" = ""):
    """HELP: Display general help text or help text for a specific command,
    or dump command reference in file when command is >filename.

    Args:
        cmd: Command name to display help for, or ">filename" to write a
            tab-delimited command reference for all commands to a file
            in the docs directory.
        subcmd: Optional subcommand to display help for.

    Returns:
        tuple: (success (bool), help text or status message (str)).
    """

    # Check if help is asked for a specific command
    cmdobj = Command.cmddict.get(cmd or "HELP")
    if cmdobj:
        return True, cmdobj.helptext(subcmd)

    # Write command reference to tab-delimited text file
    if cmd[0] == ">":
        # Get filename
        if len(cmd) > 1:
            fname = "./docs/" + cmd[1:]
        else:
            fname = "./docs/minisky-commands.txt"

        # Get unique set of commands
        cmdobjs = set(Command.cmddict.values())
        table = []  # for alphabetical sort use a table

        # Get info for all commands
        for obj in cmdobjs:
            fname = obj.callback.__name__.replace("<", "").replace(">", "")
            args = ",".join((str(p) for p in obj.parsers))
            syn = ",".join(obj.aliases)
            line = f"{obj.name}\t{obj.help}\t{obj.brief}\t{args}\t{fname}\t{syn}"
            table.append(line)

        # Sort & write table
        table.sort()
        with open(fname, "w") as f:
            # Header of first table
            f.write("Command\tDescription\tUsage\tArgument types\tFunction\tSynonyms\n")
            f.write("\n".join(table))
        return True, "Writing command reference in " + fname

    return False, "HELP: Unknown command: " + cmd


def checkscen():
    """Check if commands from the scenario buffer need to be stacked.

    All buffered scenario commands with a timestamp at or before the
    current simulation time are moved onto the command stack and removed
    from the scenario buffer.
    """
    if Stack.scencmd:
        # Find index of first timestamp exceeding minisky.sim.simt
        idx = next(
            (i for i, t in enumerate(Stack.scentime) if t > minisky.sim.simt), None
        )
        # Stack all commands before that time, and remove from scenario
        stack(*Stack.scencmd[:idx])
        del Stack.scencmd[:idx]
        del Stack.scentime[:idx]


def stack(*cmdlines, sender_id=None):
    """Stack one or more commands separated by ";".

    The queued commands are executed on the next call to process().

    Args:
        *cmdlines: Command line strings; each may contain multiple
            commands separated by ";".
        sender_id: Optional network route/id of the command sender.
    """
    for cmdline in cmdlines:
        cmdline = cmdline.strip()
        if cmdline:
            for line in cmdline.split(";"):
                Stack.cmdstack.append((line, sender_id))


def sender():
    """Return the sender of the currently executed stack command.
    If there is no sender id (e.g., when the command originates
    from a scenario file), None is returned."""
    return Stack.sender_rte[-1] if Stack.sender_rte else None


def routetosender():
    """Return the route to the sender of the currently executed stack command.
    If there is no sender id (e.g., when the command originates
    from a scenario file), None is returned."""
    return Stack.sender_rte


def get_scenname():
    """Return the name of the current scenario.
    This is either the name defined by the SCEN command,
    or otherwise the filename of the scenario."""
    return Stack.scenname


def get_scendata():
    """Return the scenario data that was loaded from a scenario file.

    Returns:
        tuple: (scentime, scencmd), the lists of command times [s] and
        command lines still buffered for execution.
    """
    return Stack.scentime, Stack.scencmd


def set_scendata(newtime, newcmd):
    """Set the scenario data. This is used by the batch logic."""
    Stack.scentime = newtime
    Stack.scencmd = newcmd
