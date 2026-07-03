# REST API server

`minisky-api.py` wraps the simulator in a [FastAPI](https://fastapi.tiangolo.com/)
application. The simulation runs continuously in the server's event loop (via
[`Runner.run()`][minisky.simulation.runner.Runner.run]) and the endpoints read from and
command the live simulation.

## Starting the server

```bash
uv run fastapi dev minisky-api.py          # development mode with auto-reload
uv run fastapi run minisky-api.py          # production mode
```

FastAPI serves interactive OpenAPI docs at `http://localhost:8000/docs`.

## Endpoints

| Method | Endpoint | Description |
| --- | --- | --- |
| GET | `/` | Health check |
| GET | `/all` | State of all aircraft (position, altitude, speeds, headings) |
| GET | `/conflicts` | Detected conflict pairs with distance, time to loss of separation, and closest point of approach |
| GET | `/simtime` | Current simulation time in seconds |
| GET | `/speed/{speed}` | Set the simulation speed multiplier |
| GET | `/forward/{seconds}` | Fast-forward the simulation by a number of seconds |
| GET | `/stack/{cmd}` | Execute any [stack command](../reference/commands.md) and return its output |
| GET/POST | `/scn` | Upload and load a scenario file (GET serves a small upload form) |
| GET | `/map` | Browser-based aircraft map viewer (served from `static/`) |
| GET | `/plugins` | List available and loaded plugins |
| GET | `/plugins/load/{name}` | Load a plugin by name |

## Examples

```bash
# Create 3 random aircraft
httpx "http://localhost:8000/stack/MCRE 3"

# Create a specific aircraft
httpx "http://localhost:8000/stack/CRE KL001 B738 52.0 4.0 90 FL100 250"

# Show aircraft near Amsterdam
httpx "http://localhost:8000/stack/POS EHAM"

# All aircraft states as JSON
httpx "http://localhost:8000/all"

# Run 10x faster than wall time
httpx "http://localhost:8000/speed/10"

# Jump ahead 5 minutes
httpx "http://localhost:8000/forward/300"

# Current conflicts
httpx "http://localhost:8000/conflicts"
```

Stack commands are case-insensitive. URL-encode spaces if your client requires it
(`httpx` and browsers handle this for you).

## How `stack/{cmd}` returns output

Stack commands don't run immediately — they are queued and executed on the next
simulation step. The endpoint queues the command, waits on the
[`ConsoleIO`][minisky.simulation.console.ConsoleIO] event that fires when the stack has
produced output, then returns the buffered echo text:

```json
{
  "command to minisky": "POS KL001",
  "message": "Info on KL001 B738 ..."
}
```

## Uploading scenarios

The `/scn` POST endpoint accepts a multipart file upload and feeds it to the stack as if
it were loaded with `IC`:

```bash
curl -F "file=@scenarios/kl204.scn" http://localhost:8000/scn
```

This lets you run the server remotely and push local scenario files to it — the
[console](console.md) `/load` command uses exactly this endpoint.
