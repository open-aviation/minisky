# Control console

`minisky-console.py` is an interactive command-line client for a running
[REST API server](rest-api.md). It gives you a prompt with history and path completion,
and forwards what you type to the server.

```bash
uv run python minisky-console.py                       # connect to localhost:8000
uv run python minisky-console.py --server http://host --port 9000
```

## Command syntax

Two kinds of input, distinguished by the leading `/`:

- **Without `/`** — the line is sent as a [stack command](../reference/commands.md) to the
  `stack/` endpoint. This is the same command language used in scenario files.
- **With `/`** — the line is a console/API command, sent directly as an HTTP request path.

```text
> POS EHAM                  # stack: show aircraft near EHAM
> MCRE 3                    # stack: create 3 random aircraft
> CRE KL001 B738 52 4 90 FL100 250

> /all                      # API: all aircraft states
> /conflicts                # API: current conflicts
> /simtime                  # API: simulation time
> /speed/10                 # API: set simulation speed to 10x
> /forward/30               # API: fast-forward 30 seconds
> /plugins                  # API: list plugins

> /load scenarios/kl204.scn # upload a local scenario file to the server
> /clear                    # clear the terminal
> /exit                     # quit the console
```

Commands are case-insensitive.

!!! note
    `/load` reads the file from *your* machine and POSTs it to the server's `/scn`
    endpoint — the file does not need to exist on the server.
