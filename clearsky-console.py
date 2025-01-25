import os

import click
import requests
from colorama import Fore, Style
from prompt_toolkit import prompt
from prompt_toolkit.completion import NestedCompleter, PathCompleter
from prompt_toolkit.history import FileHistory

import pandas as pd

history_file = os.path.expanduser("/tmp/hacksky_console_history")
path_completer = PathCompleter()

completer = NestedCompleter.from_nested_dict(
    {
        "/ic": path_completer,
        "ic": path_completer,
        "/all": None,
        "/exit": None,
        "/clear": None,
    }
)


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

        elif cmd in ["/exit", "exit"]:
            break

        elif cmd in ["/clear", "clear"]:
            os.system("clear")

        elif cmd in ["/all", "all"]:
            base_url = rool_url
            response = requests.get(f"{base_url}/all").json()
            if response:
                print(pd.DataFrame(response))
            print()

        elif cmd.startswith("/ic ") or cmd.startswith("ic "):
            base_url = rool_url
            file_path = cmd.split(" ")[1]

            if os.path.isfile(file_path):
                with open(file_path, "rb") as f:
                    files = {"file": (os.path.basename(file_path), f)}
                    response = requests.post(f"{base_url}/scn", files=files)
                    print(response.json())
            else:
                print("File does not exist\n")

        else:
            base_url = rool_url + "/stack"
            response = requests.get(f"{base_url}/{cmd}").json()
            if "msg" in response:
                msg = response["msg"]
                print(f"{msg}\n")
            else:
                print("no response message\n")


if __name__ == "__main__":
    main()
