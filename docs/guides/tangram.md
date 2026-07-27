# Streaming to a tangram map

MiniSky can act as an *external simulator* for
[tangram](https://github.com/open-aviation/tangram), the open aviation data
visualisation framework: live simulated traffic appears on tangram's map, and
the simulator can be controlled (run/hold/reset, speed, arbitrary stack
commands) from a tangram sidebar widget.

Nothing is added to the tangram source tree. Tangram discovers plugins
through Python entry points, so its side of the integration is a package you
`pip install` into whatever environment runs `tangram serve`, plus one line
of configuration. Both halves of the integration live in this repository:

```
minisky process (TANGRAM plugin)          tangram process
  publishes  to:minisky:new-data  ─▶ Redis ─▶ Channel service ─▶ browser
  publishes  to:minisky:console   ─▶            (WebSocket)        │
  listens on from:minisky:command ◀─ Redis ◀────────────────────── ┘
```

- **`example_plugins/tangram.py`** (the `TANGRAM` MiniSky plugin) owns all the
  logic: it converts each simulation snapshot to aviation units, publishes it
  to Redis, relays console output, and executes stack commands pushed from the
  browser. MiniSky talks to tangram *only* through Redis pub/sub — tangram's
  transport convention (`to:<topic>:<event>` / `from:<topic>:<event>`) that
  has stayed stable across its plugin API changes.
- **`example_plugins/tangram/tangram_minisky/`** is a separately packaged, thin
  tangram frontend plugin: it registers a `minisky_aircraft` entity type, a
  deck.gl layer, trail rendering via tangram's shared trajectory store, and
  the control widget. No business logic lives there, so it is cheap to rewrite
  when tangram's frontend plugin API changes.

## Step-by-step setup

The steps below assume Redis in a container, MiniSky and tangram on the host
— the common development setup. (Tangram's own container image only bundles
its in-tree plugins, so a dockerised `tangram serve` cannot load
`tangram_minisky` without image changes; run it on the host instead.)

### 1. Redis

Any Redis 5+ reachable by both processes. With docker/podman, publish the
port to the host:

```bash
docker run -d --rm -p 6379:6379 --name redis redis:8-alpine
```

If you already run Redis for a tangram deployment, reuse it — the two sides
meet on the channel name, not on any shared configuration.

### 2. MiniSky side (the producer)

```bash
just sync
```


In `settings.toml`:

```toml
enabled_plugins = ["TANGRAM"]

# [tangram] is optional; uncomment to override the defaults shown here.
# [tangram]
# redis_url = "redis://127.0.0.1:6379"
# channel = "minisky"
# max_hz = 5
```

Start MiniSky (any front — the bridge works the same in all of them):

```bash
uv run minisky server
# or: uv run minisky run --scenario scenarios/kl204.scn
```

Startup should print
`Tangram bridge publishing to to:minisky:* at redis://127.0.0.1:6379`.

**Verify the transport now, before touching any frontend:**

```bash
# watch everything MiniSky publishes (prefix with `docker exec -it <container>`
# if redis-cli is not installed on the host)
redis-cli psubscribe "to:*"

# drive the simulator from outside
redis-cli publish "from:minisky:command" '{"command": "CRE KL204 B744 52 4 90 FL300 250"}'
redis-cli publish "from:minisky:command" '{"command": "HOLD"}'
redis-cli publish "from:minisky:command" '{"command": "OP"}'
```

Expect `to:minisky:new-data` snapshots (~`[tangram].max_hz`/s while running,
1/s heartbeat otherwise) reacting to the commands, plus `to:minisky:console`
lines. If this works, the simulator side is done; everything after this point
is tangram-side only.

### 3. Run tangram

There are two options:

#### uv tool

```bash
uv tool install tangram_core \
  --with ./example_plugins/tangram/tangram_minisky \
  --force
tangram serve --config /path/to/tangram.toml
```

Rerun the install after rebuilding the frontend so the tool receives the new
`dist-frontend` bundle.

#### execution project (recommended)

Expected layout:

```text
./
├── minisky/
├── tangram/
└── tangram_minisky_exe/
```

`tangram_minisky_exe/pyproject.toml` pins the published core and keeps only the
plugin editable:

```toml
[project]
name = "tangram-minisky-exe"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = ["tangram-core", "tangram-minisky"]

[tool.uv.sources]
tangram-minisky = { path = "../minisky/example_plugins/tangram/tangram_minisky", editable = true }
```

Run it with:

```bash
cd ../tangram_minisky_exe
uv sync
uv run tangram serve --config tangram.toml
```

Open <http://localhost:2346>. Note that this runs on the published tangram package.

## Local tangram checkout

To test an unpublished local checkout instead, make these temporary development-only changes.

In `tangram_minisky_exe/pyproject.toml`, unpin core and add its editable source:

```toml
[project]
dependencies = ["tangram-core", "tangram-minisky"]

[tool.uv.sources]
tangram-core = { path = "../tangram/packages/tangram_core", editable = true }
tangram-minisky = { path = "../minisky/example_plugins/tangram/tangram_minisky", editable = true }
```

In MiniSky's root `pnpm-workspace.yaml`, temporarily add:

```yaml
overrides:
  "@open-aviation/tangram-core": "link:../tangram/packages/tangram_core"
```

Then refresh both environments:

```bash
pnpm install
pnpm build
cd ../tangram_minisky_exe
uv sync
```

Do not commit the temporary overrides or their lockfile changes. Comment them and
run `pnpm install` again.

## Troubleshooting

Work upstream-to-downstream:

1. **No `to:minisky:new-data` in `redis-cli psubscribe "to:*"`** — MiniSky
   side. Check the plugin loaded at startup, and run the `TANGRAM` stack
   command in the MiniSky console: it reports connection state, message count
   and the last Redis error. The bridge heartbeats once per second even when
   the simulation is idle, so *silence means it is not connected*.
2. **Snapshots flow, but tangram logs `fail to decode JWT` /
   `InvalidSignature` on joins** — the browser holds tokens signed under a
   different `jwt_secret` than the running channel service. Almost always a
   stale tab auto-reconnecting after a restart with a changed secret:
   hard-reload the page. Also check nothing else (an old `tangram serve`, a
   tangram container) is squatting ports 2346/2347: `lsof -i :2346 -i :2347`.
3. **Channel joins succeed but the widget says "Simulator offline"** — no
   snapshot or heartbeat arrived for 5 seconds. Almost always a Redis URL
   mismatch: `tangram.toml`'s `redis_url` and `settings.toml`'s
   `[tangram].redis_url` must point at the *same* Redis instance (mind
   host-vs-container addressing: a dockerised tangram reaches a compose
   Redis at `redis://redis:6379`, a host process at `redis://127.0.0.1:6379`).
   A channel-name mismatch between the two sides has the same symptom.

## Wire contract

All payloads are JSON. Aircraft fields use aviation units (altitude in ft,
speeds in kt, vertical rate in fpm) under jet1090-style names, so tangram-side
consumers see familiar shapes; the conversion from MiniSky's internal SI state
happens in the MiniSky plugin, keeping `minisky.streaming` consumer-agnostic.

- `to:<channel>:new-data`:
  `{"aircraft": [{id, callsign, typecode, latitude, longitude, altitude,
  groundspeed, tas, ias, vertical_rate, track, inconf, timestamp}],
  "count": n, "siminfo": {simt, simdt, simutc, speed, ntraf, state,
  state_name, scenname, nconf_cur, nlos_cur}}`.
  Published on every simulation step (wall-clock capped at `[tangram].max_hz`).
  Whenever the simulation is not advancing — including a freshly started
  simulator with no scenario — a heartbeat with refreshed `siminfo` (and the
  last aircraft list) is republished every second, so the frontend always
  sees the simulator and its state changes.
- `to:<channel>:console`: `{"lines": [...]}` — everything echoed to the
  MiniSky console (the bridge tees the console, it does not consume it).
- `from:<channel>:command`: `{"command": "..."}` — one stack command,
  executed on the next simulation step (works while paused, so `OP` can
  un-pause). Bare strings are also accepted for redis-cli convenience.

## Known limitations

- tangram's playback timeline (`api.time`) is client-side only; scrubbing or
  pausing it does not drive the simulator clock. Use the control widget (or
  stack commands) instead.
- Console output relayed to tangram is a tee of everything echoed by the
  simulator, not a per-command response stream.
- Commands published before the bridge's Redis subscription is live are lost
  (pub/sub has no replay); the bridge logs its status via the `TANGRAM`
  stack command.
