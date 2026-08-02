"""Command-line interface for MiniSky."""

from __future__ import annotations

import asyncio
import json
import os
import tomllib
from pathlib import Path
from pprint import pprint
from typing import TYPE_CHECKING, Annotated, TypeAlias

import requests
import typer
import websockets
from colorama import Fore, Style
from prompt_toolkit import prompt
from prompt_toolkit.completion import NestedCompleter, PathCompleter
from prompt_toolkit.history import FileHistory
from pydantic import ValidationError

from minisky.core.config import MiniSkyConfig, default_user_config_toml_path

if TYPE_CHECKING:
    from minisky.runtime import MiniSky

app = typer.Typer(help="MiniSky command-line tools.", no_args_is_help=True)

_ConfigOption: TypeAlias = Annotated[
    Path | None,
    typer.Option(help="Config TOML file. Overrides the default user config path."),
]

history_file = os.path.expanduser("/tmp/hacksky_console_history")
path_completer = PathCompleter()
completer = NestedCompleter.from_nested_dict({"load": path_completer, "/load": path_completer})


def _load_config(path: Path | None) -> MiniSkyConfig:
    selected = path.expanduser() if path is not None else default_user_config_toml_path()
    try:
        return MiniSkyConfig.from_path(selected)
    except FileNotFoundError as exc:
        if path is None:
            return MiniSkyConfig()
        raise typer.BadParameter(
            f"config file not found: {selected}",
            param_hint="--config",
        ) from exc
    except (tomllib.TOMLDecodeError, ValidationError) as exc:
        raise typer.BadParameter(
            f"invalid config file {selected}: {exc}",
            param_hint="--config",
        ) from exc


def _new_runtime(config: MiniSkyConfig, scenario: str | None = None) -> MiniSky:
    """Construct a runtime from validated configuration."""
    from minisky import MiniSky

    return MiniSky(config=config, scenario=scenario)


async def _run_scenario(scenario: str, speed: int, config_path: Path | None) -> None:
    """Initialise the simulator with a scenario and run it to completion."""
    async with _new_runtime(_load_config(config_path), scenario) as runtime:
        await runtime.plugins.load_configured()
        runtime.runner.speed = speed
        await runtime.run()


@app.command("run")
def run_cmd(
    scenario: Annotated[str, typer.Option(help="Scenario (.scn) file to run.")],
    speed: Annotated[int, typer.Option(help="Simulation speed multiplier.")] = 1,
    config: _ConfigOption = None,
) -> None:
    """Run a scenario file without interaction."""
    asyncio.run(_run_scenario(scenario, speed, config))


@app.command("server")
def server_cmd(
    host: Annotated[str | None, typer.Option(help="Host address to bind.")] = None,
    port: Annotated[int | None, typer.Option(help="TCP port to bind.")] = None,
    reload: Annotated[bool, typer.Option(help="Enable uvicorn auto-reload.")] = False,
    config: _ConfigOption = None,
) -> None:
    """Start the REST and WebSocket API server."""
    import uvicorn

    # NOTE(abraham): we want config to be explicit.
    if reload and config is not None:
        raise typer.BadParameter(
            "--config cannot be combined with --reload yet",
            param_hint="--config",
        )

    loaded_config = _load_config(config)
    host = host if host is not None else loaded_config.server.host
    port = port if port is not None else loaded_config.server.port

    if reload:
        uvicorn.run(
            "minisky.server:create_app",
            factory=True,
            host=host,
            port=port,
            reload=True,
        )
        return

    from minisky.server import create_app

    uvicorn.run(
        create_app(_new_runtime(loaded_config)),
        host=host,
        port=port,
    )


@app.command("console")
def console_cmd(
    server: Annotated[str, typer.Option(help="API server base URL.")] = "http://localhost",
    port: Annotated[int, typer.Option(help="API server port.")] = 8000,
) -> None:
    """Run the interactive console client for a MiniSky API server."""
    typer.echo(f"MiniSky Console, connect to {server}:{port}, use /exit to quit")

    root_url = f"{server}:{port}"

    while True:
        print(Fore.LIGHTGREEN_EX + Style.BRIGHT, end="")
        cmd = prompt("> ", completer=completer, history=FileHistory(history_file))
        print(Style.RESET_ALL, end="")

        if cmd == "":
            continue

        if cmd in ("/exit", "exit"):
            break

        if cmd in ("/clear", "clear"):
            os.system("clear")
            continue

        if cmd.startswith(("/load ", "load ")):
            file_path = cmd.split(" ", maxsplit=1)[1]

            if os.path.isfile(file_path):
                with open(file_path, "rb") as f:
                    files = {"file": (os.path.basename(file_path), f)}
                    response = requests.post(f"{root_url}/scn", files=files, timeout=30)
                    typer.echo(response.json())
            else:
                typer.echo("File does not exist\n")
            continue

        if not cmd.startswith("/"):
            response = requests.get(f"{root_url}/stack/{cmd.strip('/')}", timeout=30).json()
            pprint(response)
        else:
            response = requests.get(f"{root_url}/{cmd}", timeout=30).json()
            pprint(response)


async def _stream_snapshots(url: str, raw: bool) -> None:
    async with websockets.connect(url) as ws:
        typer.echo(f"connected to {url}")
        while True:
            snap = json.loads(await ws.recv())
            if raw:
                typer.echo(json.dumps(snap))
                continue
            info = snap["siminfo"]
            ac = snap["acdata"]
            typer.echo(
                f"t={info['simt']:8.1f}s  state={info['state']}  "
                f"ntraf={info['ntraf']}  speed={info['speed']}x  "
                f"callsigns={ac['callsign']}"
            )


@app.command("stream")
def stream_cmd(
    url: Annotated[
        str,
        typer.Option(help="WebSocket URL of the MiniSky stream."),
    ] = "ws://localhost:8000/stream",
    raw: Annotated[bool, typer.Option(help="Print raw JSON snapshots.")] = False,
) -> None:
    """Connect to the streaming API and print snapshots."""
    try:
        asyncio.run(_stream_snapshots(url, raw))
    except KeyboardInterrupt:
        typer.echo("\ndisconnected")
