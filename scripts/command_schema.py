"""Generate and verify tracked stack-command schemas."""

from __future__ import annotations

import json
from importlib import import_module
from pathlib import Path
from types import MappingProxyType
from typing import NamedTuple, cast

import typer
from minisky import MiniSky, MiniSkyConfig, Plugin, PluginContext
from minisky._internal.command import CommandSchema, build_command_schema
from pydantic import TypeAdapter

app = typer.Typer(no_args_is_help=True)


class PackageSchema(NamedTuple):
    path: Path
    schema: CommandSchema


def _package_schema(package: str) -> PackageSchema:
    module = import_module(package)
    module_path = module.__file__
    if module_path is None:
        raise RuntimeError(f"package {package} has no filesystem location")

    with MiniSky(MiniSkyConfig()) as runtime:
        if package == "minisky":
            commands = tuple(dict.fromkeys(runtime.commands.cmddict.values()))
        else:
            declaration = cast(Plugin, module.plugin)
            config: object = (
                MappingProxyType({})
                if declaration.config_class is None
                else TypeAdapter(declaration.config_class).validate_python({})
            )
            spec = declaration.build(PluginContext(config, runtime.python_random))
            commands = runtime.commands.prepare_components(spec.components)

    path = Path(module_path).parent / "static" / "commands.json"
    return PackageSchema(path, build_command_schema(commands))


_EMPTY_DEFAULTS = frozenset({"aliases", "constraints", "docs", "examples"})
_FALSE_DEFAULTS = frozenset({"nullable", "optional", "repeat"})


def _compact_schema(value: object) -> object:
    if isinstance(value, dict):
        compact: dict[str, object] = {}
        for key, item in value.items():
            item = _compact_schema(item)
            if key in _EMPTY_DEFAULTS and item == ():
                continue
            if key in _FALSE_DEFAULTS and item is False:
                continue
            if key == "doc" and item == "":
                continue
            compact[key] = item
        return compact
    if isinstance(value, list):
        return tuple(_compact_schema(item) for item in value)
    return value


def _dump(schema: CommandSchema) -> str:
    payload = TypeAdapter(CommandSchema).dump_python(schema, mode="json", exclude_none=True)
    return json.dumps(_compact_schema(payload), separators=(",", ":")) + "\n"


@app.command()
def export(packages: list[str]) -> None:
    """Regenerate tracked command schemas for explicit packages."""
    for package in packages:
        path, schema = _package_schema(package)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(_dump(schema))
        typer.echo(path)


@app.command()
def check(packages: list[str]) -> None:
    """Fail when a tracked command schema differs from generated output."""
    stale: list[Path] = []
    for package in packages:
        path, schema = _package_schema(package)
        if not path.is_file() or path.read_text() != _dump(schema):
            stale.append(path)
    if stale:
        for path in stale:
            typer.echo(f"stale command schema: {path}", err=True)
        raise typer.Exit(1)


if __name__ == "__main__":
    app()
