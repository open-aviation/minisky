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

## Developer commands

| Command | Purpose |
| --- | --- |
| `minisky check` | Run `ruff check .` and `pyright`. |
| `minisky test all` | Run the default test suite. |
| `minisky test unit` | Run fast unit tests. |
| `minisky test api` | Run opt-in REST API tests. |
| `minisky docs serve` | Serve this documentation site locally. |
| `minisky docs build` | Build the documentation site into `site/`. |

The `minisky ...` command is the supported command-line surface; older wrapper scripts
have been removed to keep the repository small and unambiguous.
