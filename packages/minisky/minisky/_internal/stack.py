"""The stack parses all text-based commands in the simulation.

The stack is MiniSky's text-command interpreter. Every instruction to the
simulator (typed by a user, read from a scenario (`.scn`) file, or issued by
a plugin) enters as a line of text such as
`CRE KL204 B744 52.0 4.0 90 FL300 250KT[CAS]`. Command lines are queued with
[`CommandStack.stack`][.CommandStack.stack] and executed once per simulation
step by [`CommandStack.process`][.CommandStack.process].

Each available command is represented by a [`Command`][.Command] object, which
couples the command name to the Python function that implements it and to
the argument parsers that convert argument text into typed values.
Core and plugin callbacks declare commands directly with
[`@command`][minisky.command]. The runtime composition root mounts
declarations from its explicitly owned core components; plugins use the same
declaration and preparation path with a separate load and unload lifecycle.

Each `CommandStack` owns a runtime's command registry, pending command
queue, scenario buffer, and sender state.

This module also implements scenario handling: [`CommandStack.ic`][.CommandStack.ic] loads a scenario file,
whose timestamped command lines are buffered and moved onto the stack by
[`CommandStack.checkscen`][.CommandStack.checkscen] when the
simulation time passes their timestamps.
"""

from __future__ import annotations

import asyncio
import inspect
import traceback
from collections.abc import Awaitable, Callable, Iterable, Iterator
from contextlib import suppress
from dataclasses import dataclass
from functools import partial
from io import StringIO
from pathlib import Path
from threading import Lock
from typing import TYPE_CHECKING, Any

import numpy as np

from minisky import quantities as q
from minisky._internal.command import (
    ArgumentIssue,
    CommandBoundConstraint,
    CommandCursor,
    CommandDefinition,
    CommandFiniteConstraint,
    CommandInput,
    CommandLengthConstraint,
    CommandParseContext,
    CommandPredicateConstraint,
    Keyword,
    Parameter,
    ParameterSchema,
    SimTimeS,
    SourceSpan,
    Text,
    TimeS,
    build_command_schema,
    command,
    compile_parameter,
    declared_commands,
)
from minisky._internal.identifiers import normalize_command_name
from minisky._internal.result import Err, Ok, Result

if TYPE_CHECKING:
    from minisky._internal.console import ConsoleIO
    from minisky._internal.navigation import Navdatabase
    from minisky._internal.plugin import PluginManager
    from minisky._internal.runner import Runner
    from minisky._internal.shapes import Shapes
    from minisky._internal.simulation import Simulation
    from minisky._internal.traffic import Traffic
    from minisky._internal.traffic_arrays import ReplaceableManager
    from minisky._internal.variables import VariableExplorer


@dataclass(frozen=True, slots=True)
class CommandForm:
    """An independently typed callback form of a public stack command."""

    callback: Callable[..., object]
    source: Callable[..., object]
    parameters: tuple[Parameter, ...]
    help: str


@dataclass(frozen=True, slots=True)
class _CommandCall:
    form: CommandForm
    arguments: tuple[object, ...]


@dataclass(frozen=True, slots=True)
class Command:
    """A public stack command containing typed syntax forms.

    Same-name forms use strict left-to-right choice, matching Pydantic's
    `union_mode="left_to_right"` selection rule: the first form whose entire
    argument list parses wins. We do not implement Pydantic's smart-union
    scoring or its multi-error tree.
    """

    name: str
    aliases: tuple[str, ...]
    forms: tuple[CommandForm, ...]
    parse_context: CommandParseContext

    def __post_init__(self) -> None:
        if not self.forms:
            raise ValueError(f"command {self.name} has no forms")

    def __call__(
        self, argstring: str
    ) -> Result[str, str | ArgumentIssue] | Awaitable[Result[str, str]]:
        if isinstance(resolved := self._resolve(argstring), Err):
            return resolved
        call = resolved.ok()
        result = call.form.callback(*call.arguments)
        if inspect.isawaitable(result):
            return self._await_result(result)
        return self._result(result)

    def parse_arguments(self, argstring: str) -> Result[tuple[object, ...], ArgumentIssue]:
        """Parse arguments using the first command form that accepts them."""
        if isinstance(resolved := self._resolve(argstring), Err):
            return resolved
        return Ok(resolved.ok().arguments)

    def _resolve(self, argstring: str) -> Result[_CommandCall, ArgumentIssue]:
        text = self.name + (f" {argstring}" if argstring else "")
        argument_start = len(self.name) + (1 if argstring else 0)
        failure: ArgumentIssue | None = None

        # match Pydantic's left-to-right union rule: first complete parse wins.
        # we dont use its "smart" algorithm for simplicity
        for form in self.forms:
            cursor = CommandCursor(text, argument_start)
            parsed = _parse_form(form, cursor, self.parse_context)
            if isinstance(parsed, Ok):
                return Ok(_CommandCall(form, parsed.ok()))
            failure = parsed.err()

        assert failure is not None
        return Err(failure)

    @staticmethod
    async def _await_result(result: Awaitable[object]) -> Result[str, str]:
        return Command._result(await result)

    @staticmethod
    def _result(result: object) -> Result[str, str]:
        if isinstance(result, Err):
            return result
        if isinstance(result, Ok):
            value = result.ok()
            return Ok("") if value is None else result
        if result is None:
            return Ok("")
        if isinstance(result, bool):
            return Ok("") if result else Err("")
        raise TypeError(
            f"invalid command return type: {type(result).__name__}\n"
            "expected: Result[str, str], bool, or None\n"
            "help: replace legacy (bool, text) returns with Ok(text) or Err(text)"
        )

    @property
    def names(self) -> tuple[str, ...]:
        return (self.name, *self.aliases)


def _format_constraint(constraint: object) -> str:
    match constraint:
        case CommandBoundConstraint(kind=kind, value=value):
            operator = {"gt": ">", "ge": ">=", "lt": "<", "le": "<="}[kind]
            return f"{operator} {value}"
        case CommandLengthConstraint(kind="min_length", value=value):
            return f"length >= {value}"
        case CommandLengthConstraint(kind="max_length", value=value):
            return f"length <= {value}"
        case CommandFiniteConstraint():
            return "finite"
        case CommandPredicateConstraint(name=name):
            return name
        case _:
            raise AssertionError(f"unknown command constraint: {constraint!r}")


def _format_parameter_type(
    parameter: ParameterSchema, definitions: dict[str, CommandDefinition]
) -> str:
    variants = []
    for variant in parameter.variants:
        values = []
        for value in variant.values:
            definition = definitions[value.ref]
            suffixes = ([value.unit] if value.unit is not None else []) + [
                _format_constraint(item) for item in value.constraints
            ]
            values.append(
                f"{definition.name}[{', '.join(suffixes)}]" if suffixes else definition.name
            )
        variants.append(" | ".join(values) or "value")
    if parameter.nullable:
        variants.append("None")
    return " | ".join(variants)


def _format_input(input_spec: CommandInput, *, named: bool = False) -> str:
    if input_spec.kind == "field":
        return (
            f"(e.g. {', '.join(input_spec.examples)})"
            if input_spec.examples
            else input_spec.name or ""
        )
    if input_spec.kind == "literal":
        return ", ".join(input_spec.values)
    if input_spec.kind == "boolean":
        return f"True: {', '.join(input_spec.true)}; False: {', '.join(input_spec.false)}"
    if input_spec.kind == "omitted":
        return "Requires an omitted comma field."
    if input_spec.kind == "text":
        examples = f" Examples: {', '.join(input_spec.examples)}" if input_spec.examples else ""
        return f"Consumes the remaining command text.{examples}"
    if input_spec.kind == "record":
        fields = ", ".join(
            (field.name or "value")
            + (f" (e.g. {', '.join(field.examples)})" if field.examples else "")
            for field in input_spec.fields
        )
        detail = f"All of: {fields}"
        return f"{input_spec.name}: {detail}" if named else detail
    return "One of: " + "; ".join(
        _format_input(alternative, named=True) for alternative in input_spec.alternatives
    )


def _format_usage_parameter(parameter: ParameterSchema) -> str:
    input_spec = parameter.input
    if input_spec.kind == "literal" and parameter.name.startswith("_"):
        value = "|".join(input_spec.values)
    elif input_spec.kind == "omitted":
        value = "<empty-comma-field>"
    else:
        value = f"<{parameter.name}>"
    if parameter.repeat:
        value += "..."
    return f"[{value}]" if parameter.optional else value


def _format_help_parameter(
    parameter: ParameterSchema, definitions: dict[str, CommandDefinition]
) -> list[str]:
    line = f"    {parameter.name}({_format_parameter_type(parameter, definitions)})"
    if parameter_docs := " ".join(reversed(parameter.docs)):
        line += f": {parameter_docs}"

    variants = []
    for variant in parameter.variants:
        name = " | ".join(definitions[value.ref].name for value in variant.values) or "value"
        docs = list(reversed(variant.docs))
        for value in variant.values:
            docs.extend(reversed(value.docs))
            if definition_doc := definitions[value.ref].doc:
                docs.append(definition_doc)
        detail = _format_input(variant.input)
        variants.append((name, " ".join((*docs, detail) if detail else docs)))

    if (len(variants) > 1 or parameter.nullable) and any(detail for _, detail in variants):
        lines = [line, "        One of:"]
        lines.extend(
            f"            {name}" + (f": {detail}" if detail else "") for name, detail in variants
        )
        if parameter.nullable:
            lines.append("            None")
        return lines

    if len(variants) == 1 and (detail := variants[0][1]):
        line += (" " if parameter_docs or detail.startswith("(") else ": ") + detail
    return [line]


def _parse_form(
    form: CommandForm,
    cursor: CommandCursor,
    context: CommandParseContext,
) -> Result[tuple[object, ...], ArgumentIssue]:
    arguments: list[object] = []
    for parameter in form.parameters:
        result = parameter.parse(context, cursor)
        if isinstance(result, Err):
            return result
        arguments.extend(result.ok())

    if not cursor.at_end:
        extra_start = cursor.pos
        token_result = cursor.next_field()
        if isinstance(token_result, Err):
            issue = token_result.err()
        else:
            token = token_result.ok()
            actual = token.value if token is not None else "end of input"
            span = token.span if token is not None else SourceSpan(extra_start, extra_start)
            issue = ArgumentIssue.expected("the end of the command", actual, span)
        fallback = SourceSpan(extra_start, max(extra_start + 1, extra_start))
        return Err(issue.at_argument("extra", fallback))
    return Ok(tuple(arguments))


@dataclass(frozen=True, slots=True)
class QueuedCommand:
    """A queued command line paired with its optional sender route."""

    text: str
    sender_id: bytes | None


@dataclass(frozen=True, slots=True)
class ScheduledCommand:
    """A scenario command with the simulation time at which it becomes due."""

    time: q.SimulationTimeS[float]
    text: str


@dataclass(frozen=True, slots=True)
class ScenarioData:
    """An immutable scenario snapshot; time and command text cannot diverge."""

    commands: tuple[ScheduledCommand, ...]


@dataclass(slots=True)
class _PendingCommand:
    task: asyncio.Future[Result[str, str]]
    name: str
    argstring: str
    command: Command


class CommandStack:
    """Command registry, queue, and scenario state for a runtime.

    Holds the available command objects, the queue of pending command lines,
    and the commands and timestamps loaded from a scenario file. Each
    `MiniSky` runtime owns an instance, so command and
    scenario state is not shared between runtimes.
    """

    def __init__(
        self,
        traffic: Traffic,
        navigation: Navdatabase,
        console: ConsoleIO,
        shapes: Shapes,
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
        self.shapes = shapes
        self.variables = variables
        self.plugins = plugins
        self.replaceables = replaceables
        self.parse_context = CommandParseContext(traffic, navigation)
        self._get_simulation = get_simulation
        self._get_runner = get_runner
        # TODO(abraham): package bundled scenarios inside minisky and resolve
        # them with importlib.resources for wheel installs.
        self.scenario_root = scenario_root or Path(__file__).parent.parent.parent
        self.cmddict: dict[str, Command] = {}
        """Canonical command names and aliases mapped to compiled commands."""
        self._queue_lock = Lock()
        self._pending_command: _PendingCommand | None = None
        self._reset_state()

    @property
    def simulation(self) -> Simulation:
        return self._get_simulation()

    @property
    def runner(self) -> Runner:
        return self._get_runner()

    def prepare_command(
        self,
        func: Callable[..., Any],
        *,
        name: str = "",
        aliases: tuple[str, ...] = (),
        help_text: str = "",
        source: Callable[..., Any] | None = None,
    ) -> Command:
        """Compile a command from its callback signature without registering it."""
        raw_callback = func.__func__ if isinstance(func, (staticmethod, classmethod)) else func
        source_func = (
            source
            if source is not None
            else raw_callback.func
            if isinstance(raw_callback, partial)
            else inspect.unwrap(raw_callback)
        )
        callback = self.replaceables.bind_callback(raw_callback)
        command_name = normalize_command_name(name or source_func.__name__)
        alias_names = tuple(normalize_command_name(alias) for alias in aliases)
        try:
            signature = inspect.signature(callback, eval_str=False)
        except TypeError as exc:
            raise TypeError(f"cannot inspect command {command_name}") from exc
        annotations = inspect.get_annotations(source_func, eval_str=True)
        signature = signature.replace(
            parameters=[
                parameter.replace(annotation=annotations.get(parameter.name, parameter.annotation))
                for parameter in signature.parameters.values()
            ]
        )
        if "self" in signature.parameters or "cls" in signature.parameters:
            raise TypeError(f"command {command_name} is not bound")
        for parameter in signature.parameters.values():
            if parameter.kind is inspect.Parameter.VAR_KEYWORD:
                raise TypeError(f"command {command_name} cannot accept **{parameter.name}")
            if (
                parameter.kind is inspect.Parameter.KEYWORD_ONLY
                and parameter.default is inspect.Parameter.empty
            ):
                raise TypeError(
                    f"command {command_name} has required keyword-only argument {parameter.name}"
                )
        # Optional keyword-only arguments are implementation controls, such as
        # ConsoleIO.echo(flag=...). They keep their Python defaults and are not
        # part of the positional BlueSky command grammar.
        command_parameters = tuple(
            parameter
            for parameter in signature.parameters.values()
            if parameter.kind is not inspect.Parameter.KEYWORD_ONLY
        )
        parameters = tuple(compile_parameter(parameter) for parameter in command_parameters)
        form = CommandForm(
            callback=callback,
            source=source_func,
            parameters=parameters,
            help=inspect.cleandoc(help_text or inspect.getdoc(source_func) or ""),
        )
        command = Command(
            name=command_name,
            aliases=alias_names,
            forms=(form,),
            parse_context=self.parse_context,
        )
        if len(command.names) != len(set(command.names)):
            raise ValueError(f"command {command_name} repeats an alias")
        return command

    def prepare_component(self, component: object) -> tuple[Command, ...]:
        """Compile and coalesce every command declared by a component instance."""
        prepared = tuple(
            self.prepare_command(
                bound.callback,
                name=bound.name,
                aliases=bound.aliases,
                help_text=bound.help,
                source=bound.source,
            )
            for bound in declared_commands(component)
        )
        return self.merge_commands(prepared)

    def merge_commands(self, commands: tuple[Command, ...]) -> tuple[Command, ...]:
        """Coalesce same-name declarations in declaration order."""
        merged: dict[str, Command] = {}
        order: list[str] = []
        for command_obj in commands:
            previous = merged.get(command_obj.name)
            if previous is None:
                merged[command_obj.name] = command_obj
                order.append(command_obj.name)
                continue
            aliases = tuple(dict.fromkeys((*previous.aliases, *command_obj.aliases)))
            merged[command_obj.name] = Command(
                name=command_obj.name,
                aliases=aliases,
                forms=(*previous.forms, *command_obj.forms),
                parse_context=self.parse_context,
            )
        return tuple(merged[name] for name in order)

    def prepare_components(self, components: Iterable[object]) -> tuple[Command, ...]:
        """Compile and validate commands from a set of provider components."""
        prepared = tuple(
            command for component in components for command in self.prepare_component(component)
        )
        # same-name overloads belong to one component; duplicate providers are errors.
        self.validate_commands(prepared)
        return prepared

    def mount_components(self, components: Iterable[object]) -> tuple[Command, ...]:
        """Prepare and install commands from provider components."""
        prepared = self.prepare_components(components)
        self.install_commands(prepared)
        return prepared

    def mount_component(self, component: object) -> tuple[Command, ...]:
        """Install commands from a component after runtime initialization."""
        return self.mount_components((component,))

    def validate_commands(self, commands: tuple[Command, ...]) -> None:
        """Reject command names already used by this stack or the same batch."""
        seen: set[str] = set()
        for command_obj in commands:
            for name in command_obj.names:
                if name in seen:
                    raise ValueError(f"command name repeated in batch: {name}")
                if name in self.cmddict:
                    raise ValueError(f"command already registered: {name}")
                seen.add(name)

    def install_commands(self, commands: tuple[Command, ...]) -> None:
        """Install commands that were already constructed and validated."""
        for command_obj in commands:
            for name in command_obj.names:
                self.cmddict[name] = command_obj

    def remove_commands(self, commands: tuple[Command, ...]) -> None:
        """Remove command names only while they still refer to the same object."""
        for command_obj in commands:
            for name in command_obj.names:
                if self.cmddict.get(name) is command_obj:
                    del self.cmddict[name]

    def _reset_state(self) -> None:
        """Reset the runtime-owned command queue and scenario state."""
        self.current = ""
        """Command line currently being processed."""
        with self._queue_lock:
            self.cmdstack = []
            """Queued commands awaiting processing."""
        pending, self._pending_command = self._pending_command, None
        if pending is not None and not pending.task.done():
            pending.task.cancel()
            pending.task.add_done_callback(_consume_task_result)

        self.scenname = ""
        """Name of the currently loaded scenario."""
        self.scenario_commands = []

        self.sender_rte = None
        """Sender route associated with the command currently being processed."""

    def _take_commands(self) -> list[QueuedCommand]:
        """Detach the current queue while preserving each command's sender.

        Commands stacked while the detached batch is executing remain in the
        live queue and run on the next simulation step.
        """
        with self._queue_lock:
            pending, self.cmdstack = self.cmdstack, []
        return pending

    def commands(self) -> Iterator[str]:
        """Iterate over the command lines pending for this simulation step."""
        for queued in self._take_commands():
            self.current = queued.text
            self.sender_rte = queued.sender_id
            yield queued.text

    @command(name="DEL", aliases=("DELETE",))
    def delete_element(self, target: Keyword, *additional_targets: Keyword) -> Result[str, str]:
        """Delete an element (aircraft, wind field, area shape, or group).

            DEL = "DEL" ( "WIND" | area-name | group-name
                          | selection { selection } ) ;
            selection = aircraft-id | group-name | "*" | "ALL" ;

        A sole stored group deletes its member aircraft and releases the group,
        matching BlueSky. In a multi-selection form a group expands to aircraft
        but the group definition remains.
        """
        targets = (target, *additional_targets)
        first = target
        stored_group = first in self.traffic.groups.groups
        exact_aircraft = first not in {"*", "ALL"} and self.traffic.idx(first) is not None

        # bluesky's `acid/txt` parser resolves stored groups before aircraft and
        # only falls back to text targets such as WIND or area names afterwards.
        if len(targets) == 1 and stored_group:
            match self.traffic.groups.delete_group(first):
                case Ok():
                    return Ok("")
                case Err(error):
                    return Err(error)

        if first == "WIND" and not stored_group and not exact_aircraft:
            if len(targets) != 1:
                return Err("DEL WIND does not accept additional targets")
            self.traffic.wind.clear()
            return Ok("")

        if not stored_group and not exact_aircraft and first in self.shapes.areas:
            if len(targets) != 1:
                return Err("An area cannot be combined with other DEL targets")
            return self.shapes.delete(first)

        if not stored_group and not exact_aircraft and first in self.shapes.lines:
            if len(targets) != 1:
                return Err("A line cannot be combined with other DEL targets")
            return self.shapes.delete(first)

        indices: list[int] = []
        for target_name in targets:
            if target_name in {"*", "ALL"}:
                match self.traffic.groups.listgroup(target_name):
                    case Ok(group):
                        indices.extend(int(value) for value in group)
                    case Err(error):
                        return Err(error)
                continue

            if target_name in self.traffic.groups:
                match self.traffic.groups.listgroup(target_name):
                    case Ok(group):
                        indices.extend(int(value) for value in group)
                    case Err(error):
                        return Err(error)
                continue
            index = self.traffic.idx(target_name)
            if index is not None:
                indices.append(index)
                continue
            return Err(f"Unknown aircraft, group, area, or DEL target: {target_name}")

        if indices:
            self.traffic.delete(np.unique(np.asarray(indices, dtype=int)))
        return Ok("")

    def reset(self) -> None:
        """Reset the stack.

        Clears the command queue and buffered scenario data.
        """
        self._reset_state()

    def process(self) -> bool:
        """Process commands until an awaitable callback owns the stack."""
        if not self._finish_pending_command():
            return False

        self.checkscen()

        pending = self._take_commands()
        for index, queued in enumerate(pending):
            cmdline = queued.text
            self.current = cmdline
            self.sender_rte = queued.sender_id
            cursor = CommandCursor(cmdline)
            parsed_result = cursor.next_value("a command")
            if isinstance(parsed_result, Err):
                self.console.echo(f"error: {parsed_result.err()}")
                continue
            cmd = parsed_result.ok().value
            argstring = cursor.remaining
            cmdu = cmd.upper()
            cmdobj = self.cmddict.get(cmdu)

            # bluesky shorthand permits `CALLSIGN COMMAND ...`
            # a bare callsign is the POS query form.
            if not cmdobj and cmdu in self.traffic.callsign:
                acid = cmdu
                if cursor.at_end:
                    cmd = ""
                    argstring = acid
                    cmdu = "POS"
                else:
                    parsed_result = cursor.next_value("a command")
                    if isinstance(parsed_result, Err):
                        self.console.echo(f"error: {parsed_result.err()}")
                        continue
                    cmd = parsed_result.ok().value
                    argstring = f"{acid} {cursor.remaining}"
                    cmdu = cmd.upper()
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
                # NOTE(abraham): an awaitable owns the stack. later commands stay
                # at the same simulation timestamp until it finishes.
                # TODO(abraham): add per-caller completion handles if callers need
                # responses independent of console output.
                task = asyncio.ensure_future(result)
                self._pending_command = _PendingCommand(task, cmdu, argstring, cmdobj)
                self._prepend_commands(pending[index + 1 :])
                return False

            self._echo_command_result(result)
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
            self._echo_command_result(result)
        return True

    def _prepend_commands(self, commands: list[QueuedCommand]) -> None:
        if not commands:
            return
        with self._queue_lock:
            self.cmdstack[0:0] = commands

    def _echo_command_result(self, result: Result[str, str | ArgumentIssue]) -> None:
        if isinstance(result, Ok):
            text = result.ok()
        else:
            error = result.err()
            text = f"error: {error}" if error else ""
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

    def readscn(self, scn: str | Path | StringIO) -> Iterator[ScheduledCommand]:
        """Read a scenario file and yield its timestamped commands.

        Parses lines of the form `HH:MM:SS.hh>CMDLINE`, skips full-line comments,
        supports continuation with a trailing backslash, and yields commands
        in stable timestamp order. Path-like sources are normalized to a `.scn`
        suffix; `StringIO` sources are read directly.

        Yields:
            A scheduled command containing its time and command text.

        Raises:
            TypeError: When scn is neither a path nor a StringIO object.
        """
        if isinstance(scn, (str, Path)):
            scn_path = Path(scn).with_suffix(".scn")

            with scn_path.open() as fscen:
                scn_input = StringIO(fscen.read())
        elif isinstance(scn, StringIO):
            scn_input = scn
        else:
            raise TypeError("scn must be a string or StringIO")

        commands: list[ScheduledCommand] = []
        prevline = ""
        for line in scn_input:
            line = line.strip()
            if not line or line[0] == "#":
                continue
            line = prevline + line

            if line[-1] == "\\":
                prevline = f"{line[:-1].strip()} "
                continue
            prevline = ""
            line = line.split("#", maxsplit=1)[0].strip()
            if not line:
                continue

            try:
                icmdline = line.index(">")
                tstamp = line[:icmdline]
                ttxt = tstamp.strip().split(":")
                ihr = q.hour_to_s(int(ttxt[0]))
                imin = q.min_to_s(int(ttxt[1]))
                xsec = float(ttxt[2])
                cmdtime = ihr + imin + xsec

                commands.append(ScheduledCommand(cmdtime, line[icmdline + 1 :]))
            except (ValueError, IndexError):
                self.console.echo(f"Skipping invalid scenario line: {line}")

        yield from sorted(commands, key=lambda command: command.time)

    @command(name="IC", aliases=("LOAD", "OPEN"))
    def ic(self, scn: Text) -> Result[str, str]:
        """Load a scenario file.

        Resets the simulation, reads the scenario file relative to the project
        root, and buffers its timestamped commands for execution when the
        simulation time passes their timestamps.
        """

        self.simulation.reset()

        scn_path = self.scenario_root / scn
        if not scn_path.exists():
            return Err(f"IC: File not found: {scn_path}")

        lines = self.readscn(scn_path)

        self.scenario_commands.extend(lines)
        self.scenname = scn_path.stem

        return Ok(f"scenario {scn_path} loaded.")

    def ic_StringIO(self, scn: StringIO, scn_name: str | None = None) -> Result[str, str]:
        """Load a scenario from a StringIO object.

        Resets the simulation, reads scenario lines from the `StringIO` object,
        and buffers the timestamped commands for execution (see [`checkscen`][..checkscen]). An
        optional `scn_name` becomes the current scenario name.
        """

        self.simulation.reset()

        lines = self.readscn(scn)

        self.scenario_commands.extend(lines)
        self.scenname = scn_name or ""

        return Ok(f"scenario {scn_name} loaded.")

    @command(name="SCENARIO", aliases=("SCEN",))
    def scenario(self, name: Text) -> Result[str, str]:
        """Set the scenario name for the current simulation."""
        self.scenname = name
        return Ok("Starting scenario " + name)

    @command(name="SCHEDULE")
    def schedule(self, time: SimTimeS, cmdline: Text) -> bool:
        """Schedule a stack command at a specific simulation time.

        The command is inserted into the scenario buffer, keeping the buffer
        sorted by execution time.
        """
        command = ScheduledCommand(time, cmdline)
        index = next(
            (i for i, scheduled in enumerate(self.scenario_commands) if scheduled.time > time),
            len(self.scenario_commands),
        )
        self.scenario_commands.insert(index, command)
        return True

    @command(name="DELAY")
    def delay(self, time: TimeS, cmdline: Text) -> bool:
        """Delay a stack command by a time interval.

        Like [`schedule`][..schedule], but the given time is relative to the current
        simulation time.
        """
        return self.schedule(self.simulation.simt + time, cmdline)

    @command(name="HELP", aliases=("?",))
    def show_help(self, cmd: Keyword) -> Result[str, str]:
        """Show help for a command."""
        command = self.cmddict.get(cmd)
        if command is None:
            return Err(f"HELP: Unknown command: {cmd}")

        schema = build_command_schema((command,))
        entry = schema.commands[command.name]
        sections = []
        for form in entry.forms:
            usage = " ".join(
                (
                    command.name,
                    *(_format_usage_parameter(parameter) for parameter in form.parameters),
                )
            )
            lines = [form.doc, "", usage] if form.doc else [usage]
            visible = [
                parameter for parameter in form.parameters if not parameter.name.startswith("_")
            ]
            if visible:
                lines.extend(("", "Args:"))
                for parameter in visible:
                    lines.extend(_format_help_parameter(parameter, schema.definitions))
            sections.append("\n".join(lines))

        if len(sections) > 1:
            sections = [
                "\n".join((f"{index}. {first}", *(f"   {line}" if line else "" for line in rest)))
                for index, section in enumerate(sections, start=1)
                for first, *rest in (section.splitlines(),)
            ]

        message = "\n\n".join(sections)
        if command.aliases:
            message += f"\n\nAliases: {', '.join(command.aliases)}"
        return Ok(message)

    def checkscen(self) -> None:
        """Check if commands from the scenario buffer need to be stacked.

        All buffered scenario commands with a timestamp at or before the
        current simulation time are moved onto the command stack and removed
        from the scenario buffer.
        """
        if not self.scenario_commands:
            return
        index = next(
            (
                i
                for i, scheduled in enumerate(self.scenario_commands)
                if scheduled.time > self.simulation.simt
            ),
            len(self.scenario_commands),
        )
        due = self.scenario_commands[:index]
        self.stack(*(scheduled.text for scheduled in due))
        del self.scenario_commands[:index]

    def stack(self, *cmdlines: str, sender_id: bytes | None = None) -> None:
        """Stack commands separated by ";".

        The queued commands are executed on the next call to [`process`][..process].

        Args:
            *cmdlines: Command line strings; each may contain multiple
                commands separated by ";".
            sender_id: Optional network route/id of the command sender.
        """
        queued: list[QueuedCommand] = []
        for cmdline in cmdlines:
            cursor = CommandCursor(cmdline)
            while True:
                result = cursor.next_command()
                if isinstance(result, Err):
                    self.console.echo(f"error: {result.err()}")
                    break
                line = result.ok()
                if line is None:
                    break
                queued.append(QueuedCommand(line.value, sender_id))
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

    def get_scendata(self) -> ScenarioData:
        """Return an immutable snapshot of buffered scenario commands."""
        return ScenarioData(tuple(self.scenario_commands))

    def set_scendata(self, data: ScenarioData) -> None:
        """Replace the scenario buffer with commands ordered by execution time."""
        self.scenario_commands = sorted(data.commands, key=lambda command: command.time)


def _consume_task_result(task: asyncio.Future[Any]) -> None:
    with suppress(asyncio.CancelledError, Exception):
        task.result()
