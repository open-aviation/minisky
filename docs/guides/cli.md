# Command-line interface

MiniSky installs one top-level command, `minisky`, with subcommands for running
scenarios, serving the API, using the console, and developer maintenance tasks.

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
| `minisky commands list` | Print the stack command table as Markdown. |
| `minisky commands docs` | Regenerate `docs/reference/commands.md`. |

<!-- TODO(abraham): we should develop a mkdocstrings extension (maybe zensical?) for generating commands.md -->

## Developer commands

| Command | Purpose |
| --- | --- |
| `just check` | Run repository linting and type checks. |
| `just test` | Run the default test suite. |
| `just test-unit` | Run fast unit tests. |
| `just test-api` | Run opt-in REST API tests. |
| `just docs-serve` | Serve this documentation site locally. |
| `just docs-build` | Build the documentation site into `site/`. |
