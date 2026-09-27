# REST API server

`minisky server` wraps the simulator in a [FastAPI](https://fastapi.tiangolo.com/)
application. The simulation runs continuously in the server's event loop (via
[`Runner.run()`][minisky.simulation.runner.Runner.run]) and the endpoints read from and
command the live simulation.

## Starting the server

```bash
uv run minisky server                      # production-style local server
uv run minisky server --reload             # development mode with auto-reload
```

FastAPI serves interactive OpenAPI docs at `http://localhost:8000/docs`. The server uses built-in defaults unless the [default config file](configuration.md) exists; pass `--config FILE` to choose another file explicitly.

## Endpoints

| Method | Endpoint | Description |
| --- | --- | --- |
| GET | `/` | Health check |
| GET | `/all` | State of all aircraft (position, altitude, speeds, headings) |
| GET | `/conflicts` | Detected conflict pairs with distance, time to loss of separation, and closest point of approach |
| GET | `/simtime` | Current simulation time in seconds |
| GET | `/speed/{speed}` | Set the simulation speed multiplier |
| GET | `/forward/{seconds}` | Fast-forward the simulation by a number of seconds |
| GET | `/traffic/arrays` | List every registered traffic array name (core or plugin-added) |
| GET | `/traffic/arrays/{name}` | Full values of one registered traffic array |
| GET | `/traffic/arrays/{name}/{callsign}` | One aircraft's value from a registered traffic array |
| GET | `/traffic/route/{callsign}` | One aircraft's route (waypoints and active-waypoint index) |
| GET | `/stack/{cmd}` | Execute any [stack command](../reference/commands.md) and return its output |
| GET | `/commands` | List canonical stack commands and usage strings |
| WebSocket | `/stream` | Receive rate-capped full simulation snapshots, with optional per-connection extras (see below) |
| GET/POST | `/scn` | Upload and load a scenario file (GET serves a small upload form) |
| GET | `/plugins` | List available and loaded plugins |
| GET | `/plugins/load/{name}` | Load a plugin by name |

## Examples

```bash
# Create 3 random aircraft
httpx "http://localhost:8000/stack/MCRE 3"

# Create a specific aircraft
httpx "http://localhost:8000/stack/CRE KL001 B738 52.0 4.0 90 FL100 250KT[CAS]"

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

# List every registered traffic array (core or plugin-added)
httpx "http://localhost:8000/traffic/arrays"

# Full "lat" array, and one aircraft's value from it
httpx "http://localhost:8000/traffic/arrays/lat"
httpx "http://localhost:8000/traffic/arrays/lat/KL001"

# KL001's route (waypoints, active-waypoint index)
httpx "http://localhost:8000/traffic/route/KL001"
```

Stack commands are case-insensitive. URL-encode spaces if your client requires it
(`httpx` and browsers handle this for you).

An unknown array name or callsign returns `200` with `{"msg": "..."}` rather than a
4xx status — there's no error-status convention elsewhere in this API either.

## Stream extras: per-connection subscriptions on `/stream`

`/stream` always pushes the base per-tick `Snapshot` (`siminfo` + `acdata`) to every
connection, unchanged. A connected client can additionally send small JSON control
frames on the *same* socket to layer per-connection extras onto its own outgoing
ticks, under an `extras` key, until it sends a matching `unsubscribe` (or an optional
`ttl` elapses):

```json
{"action": "subscribe", "topic": {"kind": "route", "callsign": "KL001"}}
{"action": "subscribe", "topic": {"kind": "array", "name": "tcpamax", "callsign": "KL001"}, "ttl": 30}
{"action": "unsubscribe", "topic": {"kind": "route", "callsign": "KL001"}}
```

- `topic.kind: "route"` streams that aircraft's route (same shape as `/traffic/route/{callsign}`).
- `topic.kind: "array"` streams a registered traffic array, either in full (omit `callsign`)
  or just that aircraft's value (include `callsign`) — same lookup as `/traffic/arrays/{name}[/{callsign}]`.
- `ttl` (seconds) is optional; omitted means the subscription lives until an explicit `unsubscribe`
  or disconnect.

A tick with active subscriptions gains an `extras` key, keyed by a stable string per topic
(`f"route:{callsign}"` / `f"array:{name}:{callsign or ''}"`):

```json
{"siminfo": {...}, "acdata": {...}, "extras": {"route:KL001": {...}, "array:tcpamax:KL001": 12.4}}
```

Clients that never send a control frame never see an `extras` key — the tick payload is
unchanged. Malformed control frames are ignored rather than closing the connection; topics
that no longer resolve (deleted aircraft, unregistered array name) are silently omitted from
`extras` rather than erroring.

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
curl -F "file=@packages/minisky/scenarios/kl204.scn" http://localhost:8000/scn
```

This lets you run the server remotely and push local scenario files to it — the
[console](console.md) `/load` command uses exactly this endpoint.
