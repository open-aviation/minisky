"""Load tracked command schemas for the documentation build."""

from __future__ import annotations

from pathlib import Path
from typing import Any, NamedTuple

from markdown import Markdown
from minisky._internal.command import (
    CommandDefinition,
    CommandEntry,
    ParameterSchema,
    load_command_schema,
)
from zensical.extensions.autorefs import AutorefsExtension

_ROOT = Path(__file__).parents[1]


class PluginReference(NamedTuple):
    name: str
    commands: dict[str, CommandEntry]


class _CommandReference(NamedTuple):
    plugins: tuple[PluginReference, ...]
    definitions: dict[str, CommandDefinition]


def _plugin_id(package: str) -> str:
    return "minisky" if package == "minisky" else package.removeprefix("minisky_").replace("_", "-")


def _load_reference() -> _CommandReference:
    plugins: list[PluginReference] = []
    definitions: dict[str, CommandDefinition] = {}
    for path in sorted((_ROOT / "packages").rglob("static/commands.json")):
        schema = load_command_schema(path.read_bytes())
        definitions.update(schema.definitions)
        if schema.commands:
            package = path.parent.parent.name
            plugins.append(PluginReference(_plugin_id(package), schema.commands))
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
