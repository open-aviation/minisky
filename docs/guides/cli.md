# Command-line interface

MiniSky installs a top-level command, `minisky`, with subcommands for running
scenarios, serving the API, using the console, and streaming snapshots.

```bash
uv run minisky --help
```

## User commands

| Command | Purpose |
| --- | --- |
| `minisky run --scenario FILE [--speed N]` | Run a scenario file without interaction. |
| `minisky server [--host HOST] [--port PORT] [--reload]` | Start the REST and WebSocket API server. |
| `minisky console [--server URL] [--port PORT]` | Open an interactive console against a running server. |
| `minisky stream [--url URL] [--raw]` | Print snapshots from the `/stream` WebSocket. |

## Developer commands

| Command | Purpose |
| --- | --- |
| `just check` | Run repository linting and type checks. |
| `just test` | Run the default test suite. |
| `just test-unit` | Run fast unit tests. |
| `just test-api` | Run opt-in REST API tests. |
| `just docs-serve` | Serve this documentation site locally. |
| `just docs-build` | Build the documentation site into `site/`. |
