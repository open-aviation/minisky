"""Load tracked command schemas for the documentation build."""

from __future__ import annotations

from pathlib import Path
from typing import Any, NamedTuple

from minisky._internal.command import CommandDefinition, CommandEntry, load_command_schema

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


def define_env(env: Any) -> None:
    reference = _load_reference()
    env.variables["command_reference"] = reference.plugins
    env.variables["command_definitions"] = reference.definitions
