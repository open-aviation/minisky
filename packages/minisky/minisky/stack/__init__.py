"""The stack parses all text-based commands in the simulation.

The stack is MiniSky's text-command interpreter. Every instruction to the
simulator—typed by a user, read from a scenario (`.scn`) file, or issued by
a plugin—enters as a line of text such as
`CRE KL204 B744 52.0 4.0 90 FL300 250`. Command lines are queued with
[`CommandStack.stack`][minisky.stack.CommandStack.stack] and executed once per simulation step by [`CommandStack.process`][minisky.stack.CommandStack.process].

Each available command is represented by a [Command][minisky.stack.Command] object, which
couples the command name to the Python function that implements it and to
the argument parsers that convert argument text into typed values. The base
command set is defined in `minisky.stack.commands` and registered by
[`CommandStack.init`][minisky.stack.CommandStack.init].

Each `CommandStack` owns a runtime's command registry, pending command
queue, scenario buffer, and sender state.

This module also implements scenario handling: [`CommandStack.ic`][minisky.stack.CommandStack.ic] loads a scenario file,
whose timestamped command lines are buffered and moved onto the stack by
[`CommandStack.checkscen`][minisky.stack.CommandStack.checkscen] when the
simulation time passes their timestamps.
"""

from __future__ import annotations

import asyncio
import inspect
import traceback
from collections.abc import Awaitable, Callable, Iterator
from contextlib import suppress
from dataclasses import dataclass
from functools import partial
from io import StringIO
from pathlib import Path
from threading import Lock
from typing import TYPE_CHECKING, Any, NamedTuple, TypeAlias

import numpy as np

from minisky.command import (
    ArgumentIssue,
    BoundCommand,
    SourceSpan,
    Text,
    TimeS,
    command,
    compile_parameter,
    declared_commands,
    next_argument,
)
from minisky.command import Parameter as TypedParameter
from minisky.result import Err, Ok, Result
from minisky.stack import argparser, commands
from minisky.stack.argparser import ArgumentError, Parameter, String, Txt, getnextarg

if TYPE_CHECKING:
    from minisky.core.trafficarrays import ReplaceableManager
    from minisky.core.varexplorer import VariableExplorer
    from minisky.plugin import PluginManager
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

    def __init__(
        self,
        func: Callable[..., Any],
        name: str = "",
        *,
        argument_parser: argparser.ArgumentParser,
        **kwargs: Any,
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
        self.callback = func

    def __call__(self, argstring: str) -> Result[str, str] | Awaitable[Result[str, str]]:
        """Parse arguments and execute the callback."""
        args: list[Any] = []
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
                raise ArgumentError(f"{msg}, but {count} were given")
            result = param(argstring)
            argstring = result[-1]
            args.extend(result[:-1])

        result = self.callback(*args)
        if inspect.isawaitable(result):
            return self._await_result(result)
        return self._result(result)

    @staticmethod
    async def _await_result(result: Awaitable[Any]) -> Result[str, str]:
        return Command._result(await result)

    @staticmethod
    def _result(result: Any) -> Result[str, str]:
        if isinstance(result, (Ok, Err)):
            return result
        if result is None:
            return Ok("")
        if isinstance(result, (tuple, list)):
            if len(result) > 1:
                text = str(result[1])
                return Ok(text) if bool(result[0]) else Err(text)
            if len(result) == 1:
                result = result[0]
        return Ok("") if bool(result) else Err("")

    def __repr__(self) -> str:
        if self.valid:
            return f"<Stack Command {self.name}, callback={self.callback}>"
        return f"<Stack Command {self.name} (invalid), callback=unbound method {self.callback}"

    @property
    def callback(self):
        """Callback pointing to the actual function that implements this
        stack command.
        """
        return self._callback

    @callback.setter
    def callback(self, function):
        self._callback = function
        source = function.func if isinstance(function, partial) else function
        self._callback_source = inspect.unwrap(source)
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
            if not self.impl and inspect.ismethod(self._callback_source):
                if inspect.isclass(self._callback_source.__self__):
                    self.impl = self._callback_source.__self__.__name__
                else:
                    self.impl = self._callback_source.__self__.__class__.__name__

            self.brief = self.brief or (self.name + " " + ",".join(spec.parameters))
            self.help = self.help or inspect.cleandoc(inspect.getdoc(self._callback_source) or "")
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
                        f"{self._callback_source.__name__} has arguments."
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
        if self._callback_source.__name__ == "<lambda>":
            msg += "\nAnonymous (lambda) function, implemented in "
        else:
            msg += f"\nFunction {self._callback_source.__name__}(), implemented in "
        if hasattr(self._callback_source, "__code__"):
            fname = self._callback_source.__code__.co_filename
            fname_stripped = fname.replace(str(Path.cwd()), "").lstrip("/")
            firstline = self._callback_source.__code__.co_firstlineno
            msg += f"{fname_stripped} on line {firstline}"
        else:
            msg += f"module {self._callback_source.__module__}"

        return msg

    def brieftext(self) -> str:
        """Return the brief usage text."""
        return self.brief

    class ArgumentSpec(NamedTuple):
        annotation: str
        """Parser annotation."""
        optional: bool
        """Whether the argument may be omitted."""

    def _get_arguments(
        self, arguments: str | list[ArgumentSpec] | tuple[ArgumentSpec, ...]
    ) -> tuple[ArgumentSpec, ...]:
        """Get arguments from string, or typed argument specifications."""
        if isinstance(arguments, (tuple, list)):
            return tuple(arguments)
        # Assume it is a comma-separated string
        argtypes: list[Command.ArgumentSpec] = []

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
            argtypes.extend(self.ArgumentSpec(t, opt or t == "...") for t in types if t)
            arguments = arguments[cut:].lstrip(",]")

        return tuple(argtypes)


@dataclass(frozen=True, slots=True)
class TypedCommandForm:
    callback: Callable[..., Any]
    source: Callable[..., Any]
    parameters: tuple[TypedParameter, ...]
    help: str


@dataclass(frozen=True, slots=True)
class _TypedCommandCall:
    form: TypedCommandForm
    arguments: tuple[object, ...]


@dataclass(frozen=True, slots=True)
class TypedCommand:
    name: str
    aliases: tuple[str, ...]
    forms: tuple[TypedCommandForm, ...]

    def __post_init__(self) -> None:
        if not self.forms:
            raise ValueError(f"typed command {self.name} has no forms")
        if len(self.names) != len(set(self.names)):
            raise ValueError(f"command {self.name} repeats an alias")

    def __call__(self, argstring: str) -> Result[str, str | ArgumentIssue]:
        if isinstance(resolved := self._resolve(argstring), Err):
            return resolved
        call = resolved.ok()
        result = call.form.callback(*call.arguments)
        if isinstance(result, (Ok, Err)):
            return result
        if result is None:
            return Ok("")
        if isinstance(result, bool):
            return Ok("") if result else Err("")
        # TODO(abraham): async callbacks still use the legacy command path.
        raise TypeError(f"typed command {self.name} returned unsupported {type(result).__name__}")

    def _resolve(self, argstring: str) -> Result[_TypedCommandCall, ArgumentIssue]:
        source_text = self.name + (f" {argstring}" if argstring else "")
        argument_offset = len(self.name) + (1 if argstring else 0)
        failure: ArgumentIssue | None = None
        for form in self.forms:
            parsed = _parse_typed_form(form, argstring, source_text, argument_offset)
            if isinstance(parsed, Ok):
                return Ok(_TypedCommandCall(form, parsed.ok()))
            failure = parsed.err()

        # TODO(abraham): keep the last form failure for now; richer error aggregation can wait.
        assert failure is not None
        return Err(failure)

    def merge(self, other: TypedCommand) -> TypedCommand:
        if self.name != other.name:
            raise ValueError(f"cannot merge commands {self.name} and {other.name}")
        aliases = tuple(dict.fromkeys((*self.aliases, *other.aliases)))
        return TypedCommand(self.name, aliases, (*self.forms, *other.forms))

    @property
    def names(self) -> tuple[str, ...]:
        return (self.name, *self.aliases)

    @property
    def callback(self) -> Callable[..., Any]:
        return self.forms[0].callback

    @property
    def params(self) -> tuple[TypedParameter, ...]:
        return self.forms[0].parameters

    @property
    def help(self) -> str:
        return "\n\n".join(form.help for form in self.forms if form.help)

    @property
    def brief(self) -> str:
        return "\n".join(self._form_brief(form) for form in self.forms)

    @property
    def _callback_source(self) -> Callable[..., Any]:
        # temp legacy help-table adapter
        return self.forms[0].source

    def brieftext(self) -> str:
        return self.brief

    def helptext(self, _subcmd: str = "") -> str:
        sections = []
        for form in self.forms:
            usage = f"Usage:\n{self._form_brief(form)}"
            sections.append(f"{form.help}\n{usage}" if form.help else usage)
        message = "\n\n".join(sections)
        if self.aliases:
            message += "\nCommand aliases: " + ",".join(self.aliases)
        return message

    def _form_brief(self, form: TypedCommandForm) -> str:
        suffix = ",".join(str(parameter) for parameter in form.parameters)
        return f"{self.name} {suffix}" if suffix else self.name


def _parse_typed_form(
    form: TypedCommandForm,
    argstring: str,
    source_text: str,
    argument_offset: int,
) -> Result[tuple[object, ...], ArgumentIssue]:
    arguments: list[object] = []
    remainder = argstring
    for parameter in form.parameters:
        offset = argument_offset + len(argstring) - len(remainder)
        parsed = parameter.parse(remainder, source_text=source_text, offset=offset)
        if isinstance(parsed, Err):
            return parsed
        value = parsed.ok()
        arguments.append(value.value)
        remainder = value.remainder

    if remainder:
        offset = argument_offset + len(argstring) - len(remainder)
        token_result = next_argument(remainder)
        if isinstance(token_result, Err):
            issue = token_result.err()
        else:
            token = token_result.ok()
            issue = ArgumentIssue.expected("the end of the command", token.value, token.span)
        return Err(
            issue.at_argument("extra", source_text, offset, SourceSpan(0, max(1, len(remainder))))
        )
    return Ok(tuple(arguments))


CommandEntry: TypeAlias = Command | TypedCommand


@dataclass(frozen=True, slots=True)
class PreparedCommand:
    """A parsed command ready for registry installation."""

    command: CommandEntry
    names: tuple[str, ...]


@dataclass(slots=True)
class _PendingCommand:
    task: asyncio.Future[Result[str, str]]
    name: str
    argstring: str
    command: CommandEntry


class CommandStack:
    """Command registry, queue, and scenario state for a runtime.

    Holds the available command objects, the queue of pending command lines,
    and the commands and timestamps loaded from a scenario file. Each
    `MiniSky` runtime owns an instance, so command and
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
        plugins: PluginManager,
        replaceables: ReplaceableManager,
        get_simulation: Callable[[], Simulation],
        get_runner: Callable[[], Runner],
        scenario_root: Path | None = None,
    ) -> None:
        self.traffic = traffic
        self.navigation = navigation
        self.console = console
        self.areas = areas
        self.variables = variables
        self.plugins = plugins
        self.replaceables = replaceables
        self.argument_parser = argparser.ArgumentParser(traffic, navigation, console)
        self._get_simulation = get_simulation
        self._get_runner = get_runner
        # TODO(abraham): package bundled scenarios inside minisky and resolve
        # them with importlib.resources for wheel installs.
        self.scenario_root = scenario_root or Path(__file__).parent.parent.parent
        self.cmddict: dict[str, CommandEntry] = {}
        self._queue_lock = Lock()
        self._pending_command: _PendingCommand | None = None
        self._reset_state()

    @property
    def simulation(self) -> Simulation:
        return self._get_simulation()

    @property
    def runner(self) -> Runner:
        return self._get_runner()

    # TODO(abraham): derive stack parsers from Annotated[...] metadata and remove
    # the arguments DSL.
    def prepare_command(
        self,
        func: Callable[..., Any],
        *,
        name: str = "",
        aliases: tuple[str, ...] = (),
        arguments: str = "",
        brief: str = "",
        help_text: str = "",
    ) -> PreparedCommand:
        """Construct and parse a command without registering it."""
        callback = func.__func__ if isinstance(func, (staticmethod, classmethod)) else func
        callback = self.replaceables.bind_callback(callback)
        command_name = (name or callback.__name__).upper()
        alias_names = tuple(alias.upper() for alias in aliases)
        names = (command_name, *alias_names)
        if len(names) != len(set(names)):
            raise ValueError(f"command {command_name} repeats an alias")
        command_obj = Command(
            callback,
            name=command_name,
            argument_parser=self.argument_parser,
            aliases=alias_names,
            arguments=arguments,
            brief=brief,
            help=help_text,
        )
        return PreparedCommand(command_obj, names)

    def prepare_component(self, component: object) -> tuple[PreparedCommand, ...]:
        """Compile and merge typed declarations from one runtime-owned component."""
        merged: dict[str, TypedCommand] = {}
        order: list[str] = []
        for bound in declared_commands(component):
            command_obj = self._prepare_declared_command(bound)
            previous = merged.get(bound.name)
            if previous is None:
                merged[bound.name] = command_obj
                order.append(bound.name)
                continue
            merged[bound.name] = previous.merge(command_obj)

        # TODO(abraham): keep overload merging provider-local until plugin mounting shares this path.
        return tuple(PreparedCommand(merged[name], merged[name].names) for name in order)

    def _prepare_declared_command(self, bound: BoundCommand) -> TypedCommand:
        callback = self.replaceables.bind_callback(bound.callback)
        annotations = inspect.get_annotations(bound.source, eval_str=True)
        signature = inspect.signature(callback, eval_str=False)
        signature = signature.replace(
            parameters=[
                parameter.replace(annotation=annotations.get(parameter.name, parameter.annotation))
                for parameter in signature.parameters.values()
            ]
        )
        if "self" in signature.parameters or "cls" in signature.parameters:
            raise TypeError(f"typed command {bound.name} is not bound")

        parameters: list[TypedParameter] = []
        for parameter in signature.parameters.values():
            if parameter.kind is inspect.Parameter.VAR_KEYWORD:
                raise TypeError(f"typed command {bound.name} cannot accept **{parameter.name}")
            if parameter.kind is inspect.Parameter.KEYWORD_ONLY:
                if parameter.default is inspect.Parameter.empty:
                    raise TypeError(
                        f"typed command {bound.name} has required keyword-only argument {parameter.name}"
                    )
                continue
            parameters.append(compile_parameter(parameter))

        form = TypedCommandForm(
            callback=callback,
            source=bound.source,
            parameters=tuple(parameters),
            help=bound.help,
        )
        return TypedCommand(bound.name, bound.aliases, (form,))

    def validate_commands(self, commands: tuple[PreparedCommand, ...]) -> None:
        """Reject command names already used by this stack or the same batch."""
        seen: set[str] = set()
        for prepared in commands:
            for name in prepared.names:
                if name in seen:
                    raise ValueError(f"command name repeated in batch: {name}")
                if name in self.cmddict:
                    raise ValueError(f"command already registered: {name}")
                seen.add(name)

    def install_commands(self, commands: tuple[PreparedCommand, ...]) -> None:
        """Install commands that were already constructed and validated."""
        for prepared in commands:
            for name in prepared.names:
                self.cmddict[name] = prepared.command

    def remove_commands(self, commands: tuple[PreparedCommand, ...]) -> None:
        """Remove command names only while they still refer to the same object."""
        for prepared in commands:
            for name in prepared.names:
                if self.cmddict.get(name) is prepared.command:
                    del self.cmddict[name]

    def _reset_state(self) -> None:
        """Reset the runtime-owned command queue and scenario state."""
        # Stack data
        self.current = ""
        with self._queue_lock:
            self.cmdstack: list[tuple[str, bytes | None]] = []
        pending, self._pending_command = self._pending_command, None
        if pending is not None and not pending.task.done():
            pending.task.cancel()
            pending.task.add_done_callback(_consume_task_result)

        # Scenario details
        self.scenname = ""
        self.scentime: list[float] = []
        self.scencmd: list[str] = []

        # Current command details
        self.sender_rte: bytes | None = None

    def _take_commands(self) -> list[tuple[str, bytes | None]]:
        """Detach the current queue while preserving each command's sender."""
        with self._queue_lock:
            pending, self.cmdstack = self.cmdstack, []
        return pending

    def commands(self) -> Iterator[str]:
        """Iterate over the command lines pending for this simulation step."""
        for current, sender in self._take_commands():
            self.current = current
            self.sender_rte = sender
            yield current

    def init(self) -> None:
        """Prepare, validate, and install the base stack commands."""
        catalog = commands.get_commands(self)
        legacy = tuple(
            self.prepare_command(
                definition.callback,
                name=name,
                aliases=catalog.aliases.get(name, ()),
                arguments=definition.arguments,
                brief=definition.brief,
                help_text=definition.help,
            )
            for name, definition in catalog.definitions.items()
        )
        prepared = (
            *legacy,
            *self.prepare_component(self.console),
            *self.prepare_component(self),
            *self.prepare_component(self.simulation),
            *self.prepare_component(self.runner),
        )
        self.validate_commands(prepared)
        self.install_commands(prepared)

    def delete_element(self, *arg: Any) -> Any:
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

    def process(self) -> bool:
        """Process commands until an awaitable callback owns the stack."""
        if not self._finish_pending_command():
            return False

        # First check for commands in scenario file
        self.checkscen()

        # Process stack of commands
        pending = self._take_commands()
        for index, (cmdline, sender_id) in enumerate(pending):
            self.current = cmdline
            self.sender_rte = sender_id
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

            if cmdobj is None:
                message = (
                    f"error: unknown command or aircraft: {cmd}"
                    if not argstring
                    else f"error: unknown command: {cmd}"
                )
                self.console.echo(message)
                continue

            try:
                result = cmdobj(argstring)
            except argparser.ArgumentError as exc:
                header = "" if not argstring else exc.args[0] if exc.args else "Argument error."
                self.console.echo(f"{header}\nUsage:\n{cmdobj.brieftext()}")
                continue
            except Exception as exc:  # ruff: ignore[BLE001] commands are arbitrary callbacks
                self._echo_command_exception(cmdu, argstring, exc)
                continue

            if inspect.isawaitable(result):
                try:
                    asyncio.get_running_loop()
                except RuntimeError:
                    if inspect.iscoroutine(result):
                        result.close()
                    self.console.echo("asynchronous stack commands require a running event loop")
                    continue
                # NOTE(abraham): one awaitable owns the stack. later commands stay
                # at the same simulation timestamp until it finishes.
                # TODO(abraham): add per-caller completion handles if callers need
                # responses independent of console output.
                task = asyncio.ensure_future(result)
                self._pending_command = _PendingCommand(task, cmdu, argstring, cmdobj)
                self._prepend_commands(pending[index + 1 :])
                return False

            self._echo_command_result(cmdobj, argstring, result)
        return True

    def _finish_pending_command(self) -> bool:
        pending = self._pending_command
        if pending is None:
            return True
        if not pending.task.done():
            return False
        self._pending_command = None
        try:
            result = pending.task.result()
        except asyncio.CancelledError:
            return True
        except Exception as exc:  # ruff: ignore[BLE001] commands are arbitrary callbacks
            self._echo_command_exception(pending.name, pending.argstring, exc)
        else:
            self._echo_command_result(pending.command, pending.argstring, result)
        return True

    def _prepend_commands(self, commands: list[tuple[str, bytes | None]]) -> None:
        if not commands:
            return
        with self._queue_lock:
            self.cmdstack[0:0] = commands

    def _echo_command_result(
        self, command_obj: CommandEntry, argstring: str, result: Result[str, str | ArgumentIssue]
    ) -> None:
        match result:
            case Ok(text):
                pass
            case Err(error):
                text = str(error)
                if not argstring:
                    text = text or command_obj.brieftext()
                else:
                    text = f"Error: {text or command_obj.brieftext()}"
        if text:
            self.console.echo(text)

    def _echo_command_exception(self, name: str, argstring: str, error: Exception) -> None:
        header = "" if not argstring else error.args[0] if error.args else "Function error."
        self.console.echo(
            f"Error calling function implementation of {name}: {header}\n"
            "Traceback printed to terminal."
        )
        traceback.print_exception(error)

    @property
    def command_pending(self) -> bool:
        return self._pending_command is not None

    async def wait_for_pending(self) -> None:
        pending = self._pending_command
        if pending is not None and not pending.task.done():
            await asyncio.wait((pending.task,))

    async def aclose(self) -> None:
        """Cancel and await the stack-owned asynchronous command."""
        pending, self._pending_command = self._pending_command, None
        if pending is None:
            return
        if not pending.task.done():
            pending.task.cancel()
        with suppress(asyncio.CancelledError, Exception):
            await pending.task

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

            with scn_path.open() as fscen:
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

    def ic(self, scn: str) -> Result[str, str]:
        """IC: Load a scenario file.

        Resets the simulation, reads the scenario file, and buffers its
        timestamped commands for execution when the simulation time passes
        their timestamps (see checkscen).

        Args:
            scn: The filename of the scenario, relative to the project root.
        """

        self.simulation.reset()

        scn_path = self.scenario_root / scn
        if not scn_path.exists():
            return Err(f"IC: File not found: {scn_path}")

        lines = self.readscn(scn_path)

        for cmdtime, cmd in lines:
            self.scentime.append(cmdtime)
            self.scencmd.append(cmd)
        self.scenname = scn_path.stem

        return Ok(f"scenario {scn_path} loaded.")

    def ic_StringIO(self, scn: StringIO, scn_name: str | None = None) -> Result[str, str]:
        """IC: Load a scenario from a StringIO object.

        Resets the simulation, reads scenario lines from the StringIO object,
        and buffers the timestamped commands for execution (see checkscen).

        Args:
            scn: StringIO object containing scenario lines.
            scn_name: The name of the scenario (optional).
        """

        # reset sim always
        self.simulation.reset()

        lines = self.readscn(scn)

        for cmdtime, cmd in lines:
            self.scentime.append(cmdtime)
            self.scencmd.append(cmd)
        self.scenname = scn_name or ""

        return Ok(f"scenario {scn_name} loaded.")

    def scenario(self, name: String) -> Result[str, str]:
        """SCENARIO: Set the scenario name for the current simulation.

        Args:
            name: The name to give the scenario.
        """
        self.scenname = name
        return Ok("Starting scenario " + name)

    @command(name="SCHEDULE")
    def schedule(self, time: TimeS, cmdline: Text) -> bool:
        """Schedule a stack command at a specific simulation time.

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

    @command(name="DELAY")
    def delay(self, time: TimeS, cmdline: Text) -> bool:
        """Delay a stack command by a time interval.

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

    def showhelp(self, cmd: Txt = "", subcmd: Txt = "") -> Result[str, str]:
        """HELP: Display general help text or help text for a specific command,
        or dump command reference in file when command is >filename.

        Args:
            cmd: Command name to display help for, or ">filename" to write a
                tab-delimited command reference for all commands to a file
                in the docs directory.
            subcmd: Optional subcommand to display help for.
        """

        # Check if help is asked for a specific command
        cmdobj = self.cmddict.get(cmd or "HELP")
        if cmdobj:
            return Ok(cmdobj.helptext(subcmd))

        # Write command reference to tab-delimited text file
        if cmd[0] == ">":
            # Get filename
            fname = "./docs/" + cmd[1:] if len(cmd) > 1 else "./docs/minisky-commands.txt"

            # Get unique set of commands
            cmdobjs = set(self.cmddict.values())
            table = []  # for alphabetical sort use a table

            # Get info for all commands
            for obj in cmdobjs:
                funcname = obj._callback_source.__name__.replace("<", "").replace(">", "")
                args = ",".join(str(p) for p in obj.params)
                syn = ",".join(obj.aliases)
                line = f"{obj.name}\t{obj.help}\t{obj.brief}\t{args}\t{funcname}\t{syn}"
                table.append(line)

            # Sort & write table
            table.sort()
            with Path(fname).open("w") as f:
                # Header of first table
                f.write("Command\tDescription\tUsage\tArgument types\tFunction\tSynonyms\n")
                f.write("\n".join(table))
            return Ok("Writing command reference in " + fname)

        return Err("HELP: Unknown command: " + cmd)

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
        queued: list[tuple[str, bytes | None]] = []
        for cmdline in cmdlines:
            text = cmdline.strip()
            if text:
                queued.extend((line, sender_id) for line in text.split(";") if line)
        with self._queue_lock:
            self.cmdstack.extend(queued)

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

    class ScenarioData(NamedTuple):
        times: list[float]
        """Buffered command execution times [s]."""
        commands: list[str]
        """Buffered scenario command lines."""

    def get_scendata(self) -> ScenarioData:
        """Return the scenario data that was loaded from a scenario file."""
        return self.ScenarioData(self.scentime, self.scencmd)

    def set_scendata(self, newtime, newcmd) -> None:
        """Set the scenario data. This is used by the batch logic."""
        self.scentime = newtime
        self.scencmd = newcmd


def _consume_task_result(task: asyncio.Future[Any]) -> None:
    with suppress(asyncio.CancelledError, Exception):
        task.result()
