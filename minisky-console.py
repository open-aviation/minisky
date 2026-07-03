"""Interactive console client for a running MiniSky REST API server.

Provides a prompt (with history and path completion) that forwards input to the server:
lines without a leading ``/`` are sent as stack commands via the ``stack/`` endpoint;
lines starting with ``/`` are sent as raw API request paths (e.g. ``/all``,
``/speed/10``). ``/load path.scn`` uploads a local scenario file, ``/exit`` quits.

    python minisky-console.py [--server http://host] [--port 8000]
"""
import os
from pprint import pprint

import click
import pandas as pd
import requests
from colorama import Fore, Style
from prompt_toolkit import prompt
from prompt_toolkit.completion import NestedCompleter, PathCompleter
from prompt_toolkit.history import FileHistory

history_file = os.path.expanduser("/tmp/hacksky_console_history")
path_completer = PathCompleter()

completer = NestedCompleter.from_nested_dict(
    {"load": path_completer, "/load": path_completer}
)


@click.command()
@click.option("--server", default="http://localhost", help="API server")
@click.option("--port", default=8000, help="API Port")
def main(server, port):
    """Run the interactive prompt loop against the given API server.

    Args:
        server: Base URL of the MiniSky API server (default ``http://localhost``).
        port: TCP port the server listens on (default 8000).
    """
    print(f"MiniSky Console, connect to {server}:{port}, use /exit to quit")

    rool_url = f"{server}:{port}"

    while True:
        print(Fore.LIGHTGREEN_EX + Style.BRIGHT, end="")
        cmd = prompt("> ", completer=completer, history=FileHistory(history_file))
        print(Style.RESET_ALL, end="")

        if cmd == "":
            continue

        elif cmd == "/exit" or cmd == "exit":
            break

        elif cmd == "/clear" or cmd == "clear":
            os.system("clear")

        elif cmd.startswith("/load ") or cmd.startswith("load "):
            file_path = cmd.split(" ")[1]

            if os.path.isfile(file_path):
                with open(file_path, "rb") as f:
                    files = {"file": (os.path.basename(file_path), f)}
                    response = requests.post(f"{rool_url}/scn", files=files)
                    print(response.json())
            else:
                print("File does not exist\n")

        elif not cmd.startswith("/"):
            response = requests.get(f"{rool_url}/stack/{cmd.strip('/')}").json()
            pprint(response)
        else:
            response = requests.get(f"{rool_url}/{cmd}").json()
            pprint(response)


if __name__ == "__main__":
    main()
