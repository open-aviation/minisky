# Command-line interface

MiniSky installs a top-level command, `minisky`, with subcommands for running
scenarios, serving the API, using the console, and streaming snapshots.

```bash
uv run minisky --help
```

## User commands

| Command | Purpose |
| --- | --- |
| `minisky run --scenario FILE [--speed N] [--config FILE]` | Run a scenario file without interaction. |
| `minisky server [--host HOST] [--port PORT] [--reload] [--config FILE]` | Start the REST and WebSocket API server; CLI bind options override `[server]` config. |
| `minisky console [--server URL] [--port PORT]` | Open an interactive console against a running server. |
| `minisky stream [--url URL] [--raw]` | Print snapshots from the `/stream` WebSocket. |

## Config file

`run` and `server` use built-in defaults unless the platform-specific [default config file](configuration.md) exists. Pass `--config FILE` to choose another file explicitly.
