"""Build stack-command documentation from live Python declarations."""

from __future__ import annotations

import ast
import tomllib
from pathlib import Path
from typing import Any, NamedTuple

from griffe import ExprCall, ExprKeyword, ExprName, Extension, Function
from markdown import Markdown
from minisky import MiniSky, MiniSkyConfig
from minisky._internal.command import (
    CommandDefinition,
    CommandEntry,
    CommandSchema,
    ParameterSchema,
    build_command_schema,
)
from minisky._internal.identifiers import normalize_command_name
from zensical.extensions.autorefs import AutorefsExtension

_ROOT = Path(__file__).parents[1]


class PluginReference(NamedTuple):
    name: str
    commands: dict[str, CommandEntry]


class StackCommandReference(NamedTuple):
    name: str
    aliases: tuple[str, ...]
    anchor: str


_COMMAND_DECORATORS = {"minisky.command", "minisky._internal.command.command"}


def _plugin_id(package: str) -> str:
    return "minisky" if package == "minisky" else package.removeprefix("minisky_").replace("_", "-")


def _anchor(plugin: str, command: str) -> str:
    return f"command-{plugin.lower()}-{command.lower()}"


class _DecoratedCommand(NamedTuple):
    name: str
    aliases: tuple[str, ...]


def _decorated_command(function: Function) -> _DecoratedCommand | None:
    for decorator in function.decorators:
        if decorator.callable_path not in _COMMAND_DECORATORS:
            continue
        expression = decorator.value
        if isinstance(expression, ExprName):
            return _DecoratedCommand(normalize_command_name(function.name), ())
        if not isinstance(expression, ExprCall):
            continue
        options: dict[str, object] = {}
        for argument in expression.arguments:
            if not isinstance(argument, ExprKeyword):
                continue
            try:
                options[argument.name] = ast.literal_eval(str(argument.value))
            except (SyntaxError, ValueError):
                return None
        name = normalize_command_name(str(options.get("name") or function.name))
        alias_values = options.get("aliases", ())
        if not isinstance(alias_values, (list, tuple)):
            return None
        aliases = tuple(normalize_command_name(str(alias)) for alias in alias_values)
        return _DecoratedCommand(name, aliases)
    return None


def _stack_command_reference(function: Function) -> StackCommandReference | None:
    declaration = _decorated_command(function)
    if declaration is None:
        return None
    name = declaration.name
    aliases = list(declaration.aliases)
    parent = function.parent
    if parent is not None:
        for member in parent.members.values():
            if not isinstance(member, Function) or member is function:
                continue
            sibling = _decorated_command(member)
            if sibling is None or sibling.name != name:
                continue
            aliases.extend(alias for alias in sibling.aliases if alias not in aliases)
    package = function.module.path.partition(".")[0]
    return StackCommandReference(name, tuple(aliases), _anchor(_plugin_id(package), name))


class StackCommands(Extension):
    """Expose stack-command metadata on decorated functions to mkdocstrings."""

    def on_function(self, *, func: Function, **_kwargs: Any) -> None:
        if reference := _stack_command_reference(func):
            func.extra["stack_command"] = reference


def _repository_plugin_ids() -> tuple[str, ...]:
    plugin_ids: set[str] = set()
    for path in (_ROOT / "packages").glob("*/pyproject.toml"):
        project = tomllib.loads(path.read_text()).get("project", {})
        entry_points = project.get("entry-points", {})
        plugin_ids.update(entry_points.get("minisky.plugins", {}))
    return tuple(sorted(plugin_ids))


def _plugin_schema(runtime: MiniSky, plugin_name: str) -> CommandSchema:
    record = runtime.plugins.plugins[plugin_name.upper()]
    spec = runtime.plugins._build(plugin_name, record.entry_point.load())
    commands = runtime.commands.prepare_components(spec.components)
    return build_command_schema(commands)


class _CommandReference(NamedTuple):
    plugins: tuple[PluginReference, ...]
    definitions: dict[str, CommandDefinition]


def _load_reference() -> _CommandReference:
    plugins: list[PluginReference] = []
    definitions: dict[str, CommandDefinition] = {}
    with MiniSky(MiniSkyConfig()) as runtime:
        schemas = runtime.commands.command_schemas()
        for plugin_name in _repository_plugin_ids():
            schemas[plugin_name] = _plugin_schema(runtime, plugin_name)

    for plugin_name, schema in schemas.items():
        definitions.update(schema.definitions)
        if schema.commands:
            plugins.append(PluginReference(plugin_name, schema.commands))

    plugins.sort(key=lambda plugin: (plugin.name != "minisky", plugin.name))
    return _CommandReference(tuple(plugins), definitions)


def _visible_parameters(parameters: tuple[ParameterSchema, ...]) -> tuple[ParameterSchema, ...]:
    return tuple(parameter for parameter in parameters if not parameter.name.startswith("_"))


def _markdown(text: str) -> str:
    renderer = Markdown(
        extensions=[
            AutorefsExtension(),
            "admonition",
            "pymdownx.details",
            "pymdownx.superfences",
            "pymdownx.inlinehilite",
        ]
    )
    return renderer.convert(text)


def define_env(env: Any) -> None:
    reference = _load_reference()
    env.variables["command_reference"] = reference.plugins
    env.variables["command_definitions"] = reference.definitions
    env.filter(_visible_parameters, "visible_parameters")
    env.filter(_markdown, "markdown_html")
    env.filter(_anchor, "command_anchor")
