"""Command-line interface for MiniSky."""

from __future__ import annotations

import asyncio
import json
import tomllib
from pathlib import Path
from typing import TYPE_CHECKING, Annotated, TypeAlias

import httpx2
import typer
import websockets
from prompt_toolkit import PromptSession
from prompt_toolkit.completion import NestedCompleter, PathCompleter
from prompt_toolkit.history import FileHistory
from prompt_toolkit.styles import Style
from pydantic import ValidationError
from rich.console import Console

from minisky._internal.config import MiniSkyConfig, default_user_config_toml_path

if TYPE_CHECKING:
    from minisky._internal.runtime import MiniSky

app = typer.Typer(help="MiniSky command-line tools.", no_args_is_help=True)
console = Console()

_ConfigOption: TypeAlias = Annotated[
    Path | None,
    typer.Option(help="Config TOML file. Overrides the default user config path."),
]

history_file = Path("/tmp/hacksky_console_history").expanduser()
path_completer = PathCompleter()
completer = NestedCompleter.from_nested_dict({"load": path_completer, "/load": path_completer})
prompt_style = Style.from_dict({"prompt": "bold ansibrightgreen"})


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
            "minisky._internal.server:create_app",
            factory=True,
            host=host,
            port=port,
            reload=True,
        )
        return

    from minisky._internal.server import create_app

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
    asyncio.run(_console(server, port))


async def _console(server: str, port: int) -> None:
    console.print(f"MiniSky Console, connect to {server}:{port}, use /exit to quit")
    root_url = f"{server}:{port}"
    session = PromptSession[str](
        completer=completer,
        history=FileHistory(str(history_file)),
        style=prompt_style,
    )

    async with httpx2.AsyncClient(timeout=30.0) as client:
        while True:
            cmd = await session.prompt_async([("class:prompt", "> ")])

            if cmd == "":
                continue

            if cmd in ("/exit", "exit"):
                break

            if cmd in ("/clear", "clear"):
                process = await asyncio.create_subprocess_exec("clear")
                await process.wait()
                continue

            if cmd.startswith(("/load ", "load ")):
                file_path = Path(cmd.split(" ", maxsplit=1)[1])

                if file_path.is_file():
                    contents = await asyncio.to_thread(file_path.read_bytes)
                    files = {"file": (file_path.name, contents)}
                    response = await client.post(f"{root_url}/scn", files=files)
                    console.print(response.json())
                else:
                    console.print("File does not exist\n")
                continue

            if not cmd.startswith("/"):
                response = await client.get(f"{root_url}/stack/{cmd}")
            else:
                response = await client.get(f"{root_url}/{cmd.lstrip('/')}")
            console.print(response.json())


async def _stream_snapshots(url: str, raw: bool) -> None:
    async with websockets.connect(url) as ws:
        console.print(f"connected to {url}")
        while True:
            snap = json.loads(await ws.recv())
            if raw:
                console.print(json.dumps(snap))
                continue
            info = snap["siminfo"]
            ac = snap["acdata"]
            console.print(
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
        console.print("\ndisconnected")
