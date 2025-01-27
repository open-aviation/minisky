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

completer = NestedCompleter.from_nested_dict({"ic": path_completer})


@click.command()
@click.option("--server", default="http://localhost", help="API server")
@click.option("--port", default=8000, help="API Port")
def main(server, port):
    print(f"ClearSky Console, connect to {server}:{port}, use /exit to quit")

    rool_url = f"{server}:{port}"

    while True:
        print(Fore.LIGHTGREEN_EX + Style.BRIGHT, end="")
        cmd = prompt("> ", completer=completer, history=FileHistory(history_file))
        print(Style.RESET_ALL, end="")

        if cmd == "":
            continue

        elif cmd == "exit":
            break

        elif cmd == "clear":
            os.system("clear")

        elif cmd.startswith("ic "):
            file_path = cmd.split(" ")[1]

            if os.path.isfile(file_path):
                with open(file_path, "rb") as f:
                    files = {"file": (os.path.basename(file_path), f)}
                    response = requests.post(f"{rool_url}/scn", files=files)
                    print(response.json())
            else:
                print("File does not exist\n")

        else:
            response = requests.get(f"{rool_url}/{cmd}").json()
            pprint(response)


if __name__ == "__main__":
    main()
