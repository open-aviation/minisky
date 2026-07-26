"""The stack parses all text-based commands in the simulation.

The stack is MiniSky's text-command interpreter. Every instruction to the
simulator—typed by a user, read from a scenario (`.scn`) file, or issued by
a plugin—enters as a line of text such as
`CRE KL204 B744 52.0 4.0 90 FL300 250`. Command lines are queued with
[stack][minisky.stack.stack] and executed once per simulation step by [process][minisky.stack.process].

Each available command is represented by a [Command][minisky.stack.Command] object, which
couples the command name to the Python function that implements it and to
the argument parsers that convert argument text into typed values. The base
command set is defined in `minisky.stack.commands` and registered by
`init()`.

Each `CommandStack` owns one runtime's command registry, pending command
queue, scenario buffer, and sender state. The module-level functions and
`Command.cmddict` remain compatibility aliases for the active runtime.

This module also implements scenario handling: [ic][minisky.stack.ic] loads a scenario
file, whose timestamped command lines are buffered and moved onto the stack
by `checkscen()` when the simulation time passes their timestamps.
"""

from __future__ import annotations

import inspect
import os
import traceback
from collections.abc import Callable, Iterator
from io import StringIO
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from minisky.core import trafficarrays
from minisky.plugin.plugin_decorators import append_commands, command, register_declared_commands
from minisky.stack import argparser, commands
from minisky.stack.argparser import ArgumentError, Parameter, String, Time, Txt, getnextarg

if TYPE_CHECKING:
    from minisky.core.varexplorer import VariableExplorer
    from minisky.simulation import ConsoleIO, Runner, Simulation
    from minisky.tools.areafilter import AreaFilter
    from minisky.tools.navdata import Navdatabase
    from minisky.traffic import Traffic


class Command:
    """Stack command object.

    A Command wraps a Python callback function and makes it available as a
    text command in the simulator. It stores the command name, help texts,
    and aliases, and builds a list of Parameter objects that convert the
    raw argument text of a command line into typed Python arguments for
    the callback. Calling a Command instance with an argument string parses
    the arguments and executes the callback.

    `cmddict` is a compatibility alias for the active runtime registry and
    maps command names and aliases to Command instances.

    Attributes:
        name: Command name in upper case (e.g., "CRE").
        help: Full help text shown by the HELP command.
        brief: Brief usage text (command name plus argument list).
        aliases: Tuple of alternative names for this command.
        callback: The function that implements this command.
        argument_parser: Runtime-owned argument parser used by the command.
        params: List of Parameter objects used to parse arguments.
        valid: False when the callback is an unbound class/instance method.
    """

    # Dictionary with all command objects
    cmddict: dict[str, Command] = {}

    @classmethod
    def addcommand(
        cls, func: Callable, parent: Command | None = None, name: str = "", **kwargs: Any
    ) -> None:
        """Add `func` as a stack command.

        Delegates registration to the active runtime's `CommandStack`,
        which creates a [Command][minisky.stack.Command] object for the function and registers
        its name and aliases. When a command with the same name already
        exists, the existing command object is kept.

        Args:
            func: Function, static method, or class method implementing the
                command.
            parent: Optional parent command when this is a subcommand.
            name: Command name. Defaults to the function name in upper case.
            **kwargs: Command options: `arguments` (an argument type
                specification such as `callsign,alt,[vspd]`), `brief`, `help`,
                and `aliases`.
        """
        current().addcommand(func, parent=parent, name=name, command_type=cls, **kwargs)

    def __init__(
        self,
        func,
        parent: Command | None = None,
        name: str = "",
        *,
        argument_parser: argparser.ArgumentParser,
        **kwargs,
    ) -> None:
        self.argument_parser = argument_parser
        self.name = name
        self.help = inspect.cleandoc(kwargs.get("help", ""))
        self.brief = kwargs.get("brief", "")
        self.aliases = kwargs.get("aliases", ())
        self.impl = ""
        self.valid = True
        self.arguments = self._get_arguments(kwargs.get("arguments", ""))
        self.params = []
        self.parent = parent
        self.callback = func

    def __call__(self, argstring: str):
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
            ret = ret[0]  # type: ignore[misc]
        return ret, ""

    def __repr__(self) -> str:
        if self.valid:
            return f"<Stack Command {self.name}, callback={self.callback}>"
        return f"<Stack Command {self.name} (invalid), callback=unbound method {self.callback}"

    def notimplemented(self, *args, **kwargs) -> None:
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
        try:
            # eval_str resolves stringified hints (from __future__ import annotations)
            # to the actual objects, so Annotated aliases are recognised either way
            spec = inspect.signature(function, eval_str=True)
        except NameError:
            # legacy style: parser-DSL strings ("alt", "wpt") as annotations
            spec = inspect.signature(function)
        # Check if this is an unbound class/instance method
        self.valid = spec.parameters.get("self") is None and spec.parameters.get("cls") is None

        if self.valid:
            # Store implementation origin if this is a bound (class or object) method
            if not self.impl and inspect.ismethod(function):
                if inspect.isclass(function.__self__):
                    self.impl = function.__self__.__name__
                else:
                    self.impl = function.__self__.__class__.__name__

            self.brief = self.brief or (self.name + " " + ",".join(spec.parameters))
            self.help = self.help or inspect.cleandoc(inspect.getdoc(function) or "")
            paramspecs = list(filter(Parameter.canwrap, spec.parameters.values()))
            if self.arguments:
                self.params = []
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

                    param = self.argument_parser.parameter(paramspecs[pos], annot, isopt)
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
                self.params = [
                    parameter
                    for spec in paramspecs
                    if (parameter := self.argument_parser.parameter(spec))
                ]

    def helptext(self, subcmd: str = "") -> str:
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

    def brieftext(self) -> str:
        """Return the brief usage text."""
        return self.brief

    def _get_arguments(self, arguments) -> tuple:
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

            types = [t.strip() for t in arguments[:cut].strip("[,] ").split(",")]
            # Returned argtypes are tuples of type and optional status
            argtypes += [(t, opt or t == "...") for t in types if t]
            arguments = arguments[cut:].lstrip(",]")

        return tuple(argtypes)


class CommandStack:
    """Command registry, queue, and scenario state for one runtime.

    Holds the available command objects, the queue of pending command lines,
    and the commands and timestamps loaded from a scenario file. Each
    `MiniSky` runtime owns one instance, so command and
    scenario state is not shared between runtimes.

    Attributes:
        cmddict: Mapping of command names and aliases to [Command][minisky.stack.Command] objects.
        current: Command line currently being processed.
        cmdstack: List of `(cmdline, sender route)` tuples awaiting processing.
        scenname: Name of the currently loaded scenario.
        scentime: Execution times [s] of the buffered scenario commands.
        scencmd: Buffered scenario command lines.
        sender_rte: Network route to the sender of the current command.
        argument_parser: Runtime-owned parser registry and reference data.
    """

    def __init__(
        self,
        traffic: Traffic,
        navigation: Navdatabase,
        console: ConsoleIO,
        areas: AreaFilter,
        variables: VariableExplorer,
        get_simulation: Callable[[], Simulation],
        get_runner: Callable[[], Runner],
        scenario_root: Path | None = None,
    ) -> None:
        self.traffic = traffic
        self.navigation = navigation
        self.console = console
        self.areas = areas
        self.variables = variables
        self.argument_parser = argparser.ArgumentParser(traffic, navigation, console)
        self._get_simulation = get_simulation
        self._get_runner = get_runner
        self.scenario_root = scenario_root or Path(__file__).parent.parent.parent
        self.cmddict: dict[str, Command] = {}
        self._reset_state()

    @property
    def simulation(self) -> Simulation:
        return self._get_simulation()

    @property
    def runner(self) -> Runner:
        return self._get_runner()

    def addcommand(
        self,
        func: Callable,
        parent: Command | None = None,
        name: str = "",
        command_type: type[Command] = Command,
        **kwargs: Any,
    ) -> None:
        """Add `func` as a stack command in this runtime.

        Creates a command object for the given function and registers it and
        its aliases in this command stack's `cmddict`. When a command with the
        same name already exists, the existing command object is kept.

        Args:
            func: Function, static method, or class method implementing the
                command.
            parent: Optional parent command when this is a subcommand.
            name: Command name. Defaults to the function name in upper case.
            command_type: Command class used to wrap the callback.
            **kwargs: Command options: `arguments` (an argument type
                specification such as `callsign,alt,[vspd]`), `brief`, `help`,
                and `aliases`.
        """
        func = func.__func__ if isinstance(func, (staticmethod, classmethod)) else func
        name = (name or func.__name__).upper()

        cmdobj = self.cmddict.get(name)
        if not cmdobj:
            cmdobj = command_type(
                func,
                parent,
                name,
                argument_parser=self.argument_parser,
                **kwargs,
            )
            self.cmddict[name] = cmdobj
            for alias in cmdobj.aliases:
                self.cmddict[alias] = cmdobj
        else:
            if cmdobj.callback is func:
                return
            print(f"Attempt to reimplement {name} from {cmdobj.callback} to {func}")
            if not isinstance(cmdobj, command_type):
                raise TypeError(
                    f"Error reimplementing {name}: "
                    f"A {type(cmdobj).__name__} cannot be "
                    f"reimplemented as a {command_type.__name__}"
                )

        if not inspect.ismethod(func):
            func.__stack_cmd__ = cmdobj  # type: ignore[reportFunctionMemberAccess]

    def _reset_state(self) -> None:
        """Reset the runtime-owned command queue and scenario state."""
        # Stack data
        self.current = ""
        self.cmdstack: list[tuple[str, bytes | None]] = []

        # Scenario details
        self.scenname = ""
        self.scentime: list[float] = []
        self.scencmd: list[str] = []

        # Current command details
        self.sender_rte: bytes | None = None

    def commands(self) -> Iterator[str]:
        """Iterate over the command lines pending for this simulation step.

        Detaches the pending command list before iterating so that a
        [stack][minisky.stack.stack] call from another thread, such as a plugin I/O thread,
        cannot race with processing: a command lands either on the detached
        list processed in this step or on the fresh list processed next step.
        """
        pending, self.cmdstack = self.cmdstack, []
        # Assign to instance attributes so current and sender_rte track the loop.
        for self.current, self.sender_rte in pending:
            yield self.current

    def select_implementation(self, basename: str = "", implname: str = "") -> tuple[bool, str]:
        """Select a replaceable implementation on this runtime's traffic tree."""
        return trafficarrays.select_implementation(basename, implname, self.traffic, self.cmddict)

    def init(self) -> None:
        """Initialise BlueSky base stack commands."""

        cmddict, synonyms = commands.get_commands(self)

        # register command
        for name, values in cmddict.items():
            function, arguments, brief, help_text = values

            self.addcommand(
                function,
                name=name,
                arguments=arguments,
                brief=brief,
                help=help_text,
                aliases=synonyms.get(name, []),
            )

        register_declared_commands()

    def delete_element(self, *arg):
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
            return self.traffic.wind.clear()
        elif isinstance(arg[0], str):
            return self.areas.deleteArea(arg[0])
        elif hasattr(arg[0], "groupname"):
            return self.traffic.groups.delgroup(arg[0])
        else:
            return self.traffic.delete(np.array(arg))

    def reset(self) -> None:
        """Reset the stack.

        Clears the command queue and buffered scenario data, and resets the
        argument-parser reference data (position, heading, speed).
        """
        self._reset_state()
        self.argument_parser.reset()

    def process(self) -> None:
        """Sim-side stack processing; called once per simulation step.

        First moves due scenario commands onto the stack (see checkscen), then
        parses and executes every queued command line: the first word is looked
        up in self.cmddict (an aircraft callsign may also be used as prefix,
        in which case the second word is the command, defaulting to POS), the
        remaining text is passed to the Command object for argument parsing and
        execution, and any resulting message is echoed to the screen. The
        pending commands are detached from the stack up front (see
        Stack.commands), so commands stacked while processing runs — including
        from other threads — are kept for the next step instead of being lost.
        """
        # First check for commands in scenario file
        self.checkscen()

        # Process stack of commands
        for cmdline in self.commands():
            success = True
            echotext = ""

            # Get first argument from command line and check if it's a command
            cmd, argstring = argparser.getnextarg(cmdline)
            cmdu = cmd.upper()
            cmdobj = self.cmddict.get(cmdu)

            # If no function is found for 'cmd', check if cmd is actually an aircraft id
            if not cmdobj and cmdu in self.traffic.callsign:
                cmd, argstring = argparser.getnextarg(argstring)
                argstring = cmdu + " " + argstring
                # When no other args are parsed, command is POS
                cmdu = cmd.upper() if cmd else "POS"
                cmdobj = self.cmddict.get(cmdu)

            # Proceed if a command object was found
            if cmdobj:
                try:
                    # Call the command, passing the argument string
                    success, echotext = cmdobj(argstring)
                    if not success:
                        if not argstring:
                            echotext = echotext or cmdobj.brieftext()
                        else:
                            echotext = f"Error: {echotext or cmdobj.brieftext()}"

                except argparser.ArgumentError as e:
                    success = False
                    header = "" if not argstring else e.args[0] if e.args else "Argument error."
                    echotext = f"{header}\nUsage:\n{cmdobj.brieftext()}"
                except Exception as e:
                    header = "" if not argstring else e.args[0] if e.args else "Function error."
                    echotext = (
                        f"Error calling function implementation of {cmdu}: {header}\n"
                        + "Traceback printed to terminal."
                    )
                    traceback.print_exc()

            # Command not found
            else:
                success = False
                if not argstring:
                    echotext = f"error: unknown command or aircraft: {cmd}"
                else:
                    echotext = f"error: unknown command: {cmd}"

            if echotext:
                self.console.echo(echotext)

    def readscn(self, scn: str | Path | StringIO) -> Iterator[tuple[float, str]]:
        """Read a scenario file and yield its timestamped commands.

        Parses lines of the form `HH:MM:SS.hh>CMDLINE`, skipping comments
        (lines starting with "#") and empty lines, and supporting line
        continuation with a trailing backslash.

        Args:
            scn: Scenario source: path to a .scn file (str or Path; the .scn
                suffix is added when missing), or a StringIO object.

        Yields:
            tuple: (command time [s] (float), command line (str)).

        Raises:
            TypeError: When scn is neither a path nor a StringIO object.
        """
        if isinstance(scn, (str, Path)):
            # ensure .scn suffix if necessary
            scn_path = Path(scn).with_suffix(".scn")

            with open(scn_path) as fscen:
                scn_input = StringIO(fscen.read())
        elif isinstance(scn, StringIO):
            scn_input = scn
        else:
            raise TypeError("scn must be a string or StringIO")

        prevline = ""
        for line in scn_input:
            line = line.strip()
            # Skip emtpy lines and comments
            if not line or line[0] == "#":
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
                    self.console.echo(f"Skipping invalid scenario line: {line.strip()}")

    def ic(self, scn: str) -> tuple[bool, str]:
        """IC: Load a scenario file.

        Resets the simulation, reads the scenario file, and buffers its
        timestamped commands for execution when the simulation time passes
        their timestamps (see checkscen).

        Args:
            scn: The filename of the scenario, relative to the project root.

        Returns:
            tuple: (success (bool), message (str)).
        """

        self.simulation.reset()

        scn_path = self.scenario_root / scn
        if not scn_path.exists():
            return False, f"IC: File not found: {scn_path}"

        lines = self.readscn(scn_path)

        for cmdtime, cmd in lines:
            self.scentime.append(cmdtime)
            self.scencmd.append(cmd)
        self.scenname = scn_path.stem

        return True, f"scenario {scn_path} loaded."

    def ic_StringIO(self, scn: StringIO, scn_name: str | None = None) -> tuple[bool, str]:
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
        self.simulation.reset()

        lines = self.readscn(scn)

        for cmdtime, cmd in lines:
            self.scentime.append(cmdtime)
            self.scencmd.append(cmd)
        self.scenname = scn_name or ""

        return True, f"scenario {scn_name} loaded."

    def scenario(self, name: String) -> tuple[bool, str]:
        """SCENARIO: Set the scenario name for the current simulation.

        Args:
            name: The name to give the scenario.

        Returns:
            tuple: (True, confirmation message).
        """
        self.scenname = name
        return True, "Starting scenario " + name

    def schedule(self, time: Time, cmdline: String) -> bool:
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
        idx = next((i for i, t in enumerate(self.scentime) if t > time), len(self.scentime))
        self.scentime.insert(idx, time)
        self.scencmd.insert(idx, cmdline)
        return True

    def delay(self, time: Time, cmdline: String) -> bool:
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
        time += self.simulation.simt
        idx = next((i for i, t in enumerate(self.scentime) if t > time), len(self.scentime))
        self.scentime.insert(idx, time)
        self.scencmd.insert(idx, cmdline)
        return True

    def showhelp(self, cmd: Txt = "", subcmd: Txt = "") -> tuple[bool, str]:
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
        cmdobj = self.cmddict.get(cmd or "HELP")
        if cmdobj:
            return True, cmdobj.helptext(subcmd)

        # Write command reference to tab-delimited text file
        if cmd[0] == ">":
            # Get filename
            fname = "./docs/" + cmd[1:] if len(cmd) > 1 else "./docs/minisky-commands.txt"

            # Get unique set of commands
            cmdobjs = set(self.cmddict.values())
            table = []  # for alphabetical sort use a table

            # Get info for all commands
            for obj in cmdobjs:
                funcname = obj.callback.__name__.replace("<", "").replace(">", "")
                args = ",".join(str(p) for p in obj.params)
                syn = ",".join(obj.aliases)
                line = f"{obj.name}\t{obj.help}\t{obj.brief}\t{args}\t{funcname}\t{syn}"
                table.append(line)

            # Sort & write table
            table.sort()
            with open(fname, "w") as f:
                # Header of first table
                f.write("Command\tDescription\tUsage\tArgument types\tFunction\tSynonyms\n")
                f.write("\n".join(table))
            return True, "Writing command reference in " + fname

        return False, "HELP: Unknown command: " + cmd

    def checkscen(self) -> None:
        """Check if commands from the scenario buffer need to be stacked.

        All buffered scenario commands with a timestamp at or before the
        current simulation time are moved onto the command stack and removed
        from the scenario buffer.
        """
        if self.scencmd:
            # Find index of first timestamp exceeding self.simulation.simt
            idx = next((i for i, t in enumerate(self.scentime) if t > self.simulation.simt), None)
            # Stack all commands before that time, and remove from scenario
            self.stack(*self.scencmd[:idx])
            del self.scencmd[:idx]
            del self.scentime[:idx]

    def stack(self, *cmdlines: str, sender_id: bytes | None = None) -> None:
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
                    self.cmdstack.append((line, sender_id))

    def sender(self):
        """Return the sender of the currently executed stack command.
        If there is no sender id (e.g., when the command originates
        from a scenario file), None is returned."""
        return self.sender_rte[-1] if self.sender_rte else None

    def routetosender(self):
        """Return the route to the sender of the currently executed stack command.
        If there is no sender id (e.g., when the command originates
        from a scenario file), None is returned."""
        return self.sender_rte

    def get_scenname(self) -> str:
        """Return the name of the current scenario.
        This is either the name defined by the SCEN command,
        or otherwise the filename of the scenario."""
        return self.scenname

    def get_scendata(self) -> tuple:
        """Return the scenario data that was loaded from a scenario file.

        Returns:
            tuple: (scentime, scencmd), the lists of command times [s] and
            command lines still buffered for execution.
        """
        return self.scentime, self.scencmd

    def set_scendata(self, newtime, newcmd) -> None:
        """Set the scenario data. This is used by the batch logic."""
        self.scentime = newtime
        self.scencmd = newcmd


_active_stack: CommandStack | None = None


def _activate(command_stack: CommandStack) -> None:
    """Activate a runtime command stack for compatibility APIs."""
    global _active_stack
    _active_stack = command_stack
    Command.cmddict = command_stack.cmddict


def current() -> CommandStack:
    """Return the active runtime command stack."""
    if _active_stack is None:
        raise RuntimeError("MiniSky command stack is not initialized")
    return _active_stack


class Stack:
    """Compatibility namespace for the former static stack class.

    The command queue and scenario state now belong to the active runtime's
    `CommandStack`. This class preserves the former `Stack.reset()` and
    `Stack.commands()` entry points by delegating to that active instance.
    """

    @classmethod
    def reset(cls) -> None:
        """Reset the active runtime's stack variables."""
        current()._reset_state()

    @classmethod
    def commands(cls) -> Iterator[str]:
        """Iterate over the active runtime's pending command lines.

        The pending list is detached before iteration so commands added while
        processing are retained for the next simulation step.
        """
        return current().commands()


def init() -> None:
    """Initialise the base stack commands for the active runtime."""
    current().init()


def delete_element(*arg):
    """DEL: Delete an element (aircraft, wind field, area shape, or group).

    Dispatches based on the first argument: the string `WIND` clears the wind
    field, any other string deletes the area with that name, a traffic group
    object deletes that group, and anything else is treated as aircraft
    indices to delete.

    Args:
        *arg: Element or elements to delete: `WIND`, an area name, a traffic
            group, or one or more aircraft indices.

    Returns:
        The result of the dispatched delete function.
    """
    return current().delete_element(*arg)


def reset() -> None:
    """Reset the stack.

    Clears the command queue and buffered scenario data, and resets the
    argument-parser reference data for position, heading, and speed.
    """
    current().reset()


def process() -> None:
    """Process the active runtime's command stack once.

    First moves due scenario commands onto the stack, then parses and executes
    every queued command line. The first word is looked up in
    `Command.cmddict`; an aircraft callsign may also be used as a prefix, in
    which case the second word is the command and defaults to `POS`. Remaining
    text is parsed into typed arguments and passed to the command callback.

    The pending commands are detached before processing, so commands stacked
    during processing, including from other threads, are retained for the next
    simulation step.
    """
    current().process()


def readscn(scn: str | Path | StringIO) -> Iterator[tuple[float, str]]:
    """Read a scenario file and yield its timestamped commands.

    Parses lines of the form `HH:MM:SS.hh>CMDLINE`, skipping comments and empty
    lines and supporting line continuation with a trailing backslash.

    Args:
        scn: Scenario source: a path to a `.scn` file, or a `StringIO` object.
            The `.scn` suffix is added to paths when missing.

    Yields:
        A `(command time [s], command line)` tuple for each valid line.

    Raises:
        TypeError: When `scn` is neither a path nor a `StringIO` object.
    """
    return current().readscn(scn)


def ic(scn: str) -> tuple[bool, str]:
    """IC: Load a scenario file.

    Resets the simulation, reads the scenario file, and buffers its timestamped
    commands for execution when simulation time passes their timestamps.

    Args:
        scn: Scenario filename relative to the project root.

    Returns:
        A `(success, message)` tuple.
    """
    return current().ic(scn)


def ic_StringIO(scn: StringIO, scn_name: str | None = None) -> tuple[bool, str]:
    """IC: Load a scenario from a `StringIO` object.

    Resets the simulation, reads scenario lines from the object, and buffers
    the timestamped commands for execution.

    Args:
        scn: Object containing scenario lines.
        scn_name: Optional scenario name.

    Returns:
        A `(success, message)` tuple.
    """
    return current().ic_StringIO(scn, scn_name)


def scenario(name: String) -> tuple[bool, str]:
    """SCENARIO: Set the scenario name for the current simulation.

    Args:
        name: Name to give the scenario.

    Returns:
        A `(True, confirmation message)` tuple.
    """
    return current().scenario(name)


def schedule(time: Time, cmdline: String) -> bool:
    """SCHEDULE: Schedule a command at a specific simulation time.

    The command is inserted into the scenario buffer while preserving its
    execution-time ordering.

    Args:
        time: Absolute simulation time [s] at which to execute the command.
        cmdline: Command line to execute.

    Returns:
        `True`; the command is always scheduled.
    """
    return current().schedule(time, cmdline)


def delay(time: Time, cmdline: String) -> bool:
    """DELAY: Delay a command by a time interval.

    Like [schedule][minisky.stack.schedule], but `time` is relative to the current simulation time.

    Args:
        time: Time interval [s] by which to delay the command.
        cmdline: Command line to execute after the delay.

    Returns:
        `True`; the command is always scheduled.
    """
    return current().delay(time, cmdline)


def showhelp(cmd: Txt = "", subcmd: Txt = "") -> tuple[bool, str]:
    """HELP: Display command help or write a command reference file.

    Args:
        cmd: Command name to display, or `>filename` to write a tab-delimited
            command reference in the documentation directory.
        subcmd: Optional subcommand to display.

    Returns:
        A `(success, help text or status message)` tuple.
    """
    return current().showhelp(cmd, subcmd)


def checkscen() -> None:
    """Move due scenario commands onto the active runtime's command queue.

    All buffered scenario commands with a timestamp at or before the current
    simulation time are removed from the scenario buffer and queued for
    execution.
    """
    current().checkscen()


def stack(*cmdlines: str, sender_id: bytes | None = None) -> None:
    """Stack one or more commands separated by semicolons.

    Queued commands are executed on the next call to [process][minisky.stack.process].

    Args:
        *cmdlines: Command line strings. Each may contain multiple commands
            separated by semicolons.
        sender_id: Optional network route or identifier of the sender.
    """
    current().stack(*cmdlines, sender_id=sender_id)


def sender():
    """Return the sender of the command currently being executed.

    Returns `None` when the command has no sender identifier, such as a command
    originating from a scenario file.
    """
    return current().sender()


def routetosender():
    """Return the route to the sender of the current command.

    Returns `None` when the command has no sender identifier, such as a command
    originating from a scenario file.
    """
    return current().routetosender()


def get_scenname() -> str:
    """Return the current scenario name.

    This is the name defined by the `SCENARIO` command or, when no explicit
    name was set, the scenario filename.
    """
    return current().get_scenname()


def get_scendata() -> tuple[list[float], list[str]]:
    """Return the buffered scenario data.

    Returns:
        A `(scentime, scencmd)` tuple containing command times [s] and command
        lines still buffered for execution.
    """
    return current().get_scendata()


def set_scendata(newtime, newcmd) -> None:
    """Replace the buffered scenario data used by batch execution."""
    current().set_scendata(newtime, newcmd)


for _name in (
    "init",
    "delete_element",
    "reset",
    "process",
    "readscn",
    "ic",
    "ic_StringIO",
    "scenario",
    "schedule",
    "delay",
    "showhelp",
    "checkscen",
    "stack",
    "sender",
    "routetosender",
    "get_scenname",
    "get_scendata",
    "set_scendata",
):
    globals()[_name].__doc__ = getattr(CommandStack, _name).__doc__
