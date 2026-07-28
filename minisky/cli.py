"""Command-line interface for MiniSky."""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
from pprint import pprint
from typing import TYPE_CHECKING, Annotated

import requests
import typer
import websockets
from colorama import Fore, Style
from prompt_toolkit import prompt
from prompt_toolkit.completion import NestedCompleter, PathCompleter
from prompt_toolkit.history import FileHistory

if TYPE_CHECKING:
    from minisky.runtime import MiniSky

app = typer.Typer(help="MiniSky command-line tools.", no_args_is_help=True)
commands_app = typer.Typer(help="Inspect or regenerate stack-command documentation.")
app.add_typer(commands_app, name="commands")

history_file = os.path.expanduser("/tmp/hacksky_console_history")
path_completer = PathCompleter()
completer = NestedCompleter.from_nested_dict({"load": path_completer, "/load": path_completer})

COMMAND_DOCS_OUTFILE = Path(__file__).parent.parent / "docs" / "reference" / "commands.md"
COMMAND_DOCS_START = "<!-- MINISKY_COMMAND_DOCS_START: do not edit this section by hand! -->"
COMMAND_DOCS_END = "<!-- MINISKY_COMMAND_DOCS_END -->"
COMMAND_DOCS_TABLE_HEADER = """\
| Command | Usage | Description | Synonyms |
| --- | --- | --- | --- |
"""
COMMAND_DOCS_HEADER = f"{COMMAND_DOCS_START}\n\n{COMMAND_DOCS_TABLE_HEADER}"


def _new_runtime(scenario: str | None = None) -> MiniSky:
    """Construct a runtime from the default settings."""
    from minisky import DEFAULT_SETTINGS_FILE, MiniSky, MiniSkySettings

    settings = MiniSkySettings.from_file(DEFAULT_SETTINGS_FILE)
    runtime = MiniSky(settings, scenario)
    return runtime


async def _run_scenario(scenario: str, speed: int) -> None:
    """Initialise the simulator with a scenario and run it to completion."""
    async with _new_runtime(scenario) as runtime:
        runtime.load_plugins()
        runtime.runner.speed = speed
        await runtime.run()


@app.command("run")
def run_cmd(
    scenario: Annotated[str, typer.Option(help="Scenario (.scn) file to run.")],
    speed: Annotated[int, typer.Option(help="Simulation speed multiplier.")] = 1,
) -> None:
    """Run a scenario file without interaction."""
    asyncio.run(_run_scenario(scenario, speed))


@app.command("server")
def server_cmd(
    host: Annotated[str, typer.Option(help="Host address to bind.")] = os.environ.get(
        "MINISKY_HOST", "0.0.0.0"
    ),
    port: Annotated[int, typer.Option(help="TCP port to bind.")] = int(
        os.environ.get("MINISKY_PORT", "8000")
    ),
    reload: Annotated[bool, typer.Option(help="Enable uvicorn auto-reload.")] = False,
) -> None:
    """Start the REST and WebSocket API server."""
    import uvicorn

    uvicorn.run(
        "minisky.server:create_app",
        factory=True,
        host=host,
        port=port,
        reload=reload,
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

        if cmd.startswith("/load ") or cmd.startswith("load "):
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


def _build_command_rows() -> list[str]:
    from minisky.stack import Command

    with _new_runtime() as runtime:
        primary: dict[str, Command] = {}
        synonyms: dict[str, list[str]] = {}
        for name, cmdobj in sorted(runtime.commands.cmddict.items()):
            if cmdobj.name == name:
                primary[name] = cmdobj
            else:
                synonyms.setdefault(cmdobj.name, []).append(name)

        lines: list[str] = []
        for name, cmdobj in sorted(primary.items()):
            usage = (cmdobj.brief or "").replace("|", "\\|").replace("\n", " ")
            help_text = (cmdobj.help or "").replace("|", "\\|")
            help_text = help_text.strip().splitlines()[0] if help_text.strip() else ""
            syns = ", ".join(f"`{s}`" for s in sorted(synonyms.get(name, [])))
            lines.append(f"| `{name}` | `{usage}` | {help_text} | {syns} |\n")
        return lines


def _render_command_docs(rows: list[str]) -> str:
    return f"{COMMAND_DOCS_HEADER}{''.join(rows)}{COMMAND_DOCS_END}"


def _replace_command_docs_section(document: str, generated: str) -> str:
    start_count = document.count(COMMAND_DOCS_START)
    end_count = document.count(COMMAND_DOCS_END)
    if start_count != 1 or end_count != 1:
        raise ValueError(
            "expected exactly one MINISKY_COMMAND_DOCS_START and one "
            "MINISKY_COMMAND_DOCS_END sentinel"
        )

    start = document.index(COMMAND_DOCS_START)
    end = document.index(COMMAND_DOCS_END)
    if end < start:
        raise ValueError("MINISKY_COMMAND_DOCS_END appears before MINISKY_COMMAND_DOCS_START")

    end += len(COMMAND_DOCS_END)
    return document[:start] + generated + document[end:]


@commands_app.command("list")
def commands_list() -> None:
    """Print the stack command table as Markdown."""
    typer.echo(COMMAND_DOCS_TABLE_HEADER + "".join(_build_command_rows()))


@commands_app.command("docs")
def commands_docs(
    output: Annotated[
        Path,
        typer.Option("--output", "-o", help="Markdown file to update."),
    ] = COMMAND_DOCS_OUTFILE,
) -> None:
    """Regenerate the sentinel-delimited stack-command reference section."""
    rows = _build_command_rows()
    try:
        document = output.read_text()
        updated = _replace_command_docs_section(document, _render_command_docs(rows))
        output.write_text(updated)
    except (OSError, ValueError) as exc:
        typer.echo(f"Could not update {output}: {exc}", err=True)
        raise typer.Exit(1) from exc

    typer.echo(f"Wrote {len(rows)} commands to {output}")
