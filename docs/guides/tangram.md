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
uv sync --extra tangram
```

This installs the `redis` client the `TANGRAM` bridge needs into the repo-root
`.venv`. The MiniSky producer is deliberately decoupled from the tangram
frontend: `tangram_core` and the plugin live in their own self-contained
workspace at `example_plugins/tangram/` (a separate `.venv`, so its heavy,
platform-specific `tangram_core` wheel never touches the MiniSky env). That
half is set up in steps 3–4.

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

### 3. Build the frontend plugin

The frontend lives in its own uv + pnpm workspace at `example_plugins/tangram/`
(mirroring [tangram's own `tangram/` layout](https://github.com/open-aviation/tangram)).
Set it up and build:

```bash
cd example_plugins/tangram
just sync                      # uv sync --all-packages + pnpm install
pnpm build                     # bundles each plugin into dist-frontend/
```

Sanity-check the packaging:

```bash
uv run tangram check-plugin tangram_minisky
```

### 4. Install tangram + the plugin on the host

**Workspace route (recommended in this repo).** Nothing to install: step 3's
`just sync` already put `tangram_core` and the (editable) plugin into
`example_plugins/tangram/.venv`, and tangram resolves an editable install's
`dist-frontend` straight from the source tree. `pnpm build` output is picked
up on the next `tangram serve` restart — no reinstall needed. Run tangram from
that workspace (`cd example_plugins/tangram && uv run tangram serve ...`, see
step 5).

**Separate-environment route.** To run tangram outside this workspace's venv
(e.g. a standalone deployment):

```bash
uv tool install tangram_core --with ./example_plugins/tangram/tangram_minisky --force
```

(Or with a plain venv: `uv pip install tangram_core ./example_plugins/tangram/tangram_minisky`.
The `--force`/reinstall is also the update path: rerun it after every
`pnpm build`.)

### 5. Configure and run tangram

Create a `tangram.toml` anywhere (the `tangram` CLI does not care about the
working directory, only about the `--config` path):

```toml
[core]
redis_url = "redis://127.0.0.1:6379"
plugins = ["tangram_minisky"]

[server]
host = "127.0.0.1"
port = 2346

[channel]
host = "127.0.0.1"
port = 2347
jwt_secret = "any-random-string-you-like"
jwt_expiration_secs = 315360000

# Optional — only needed if you change the channel name; it must then match
# [tangram].channel in MiniSky's settings.toml.
# [plugins.tangram_minisky]
# channel = "minisky"
```

`jwt_secret` is used by tangram's channel service to sign and verify its own
WebSocket tokens; it does not need to match anything on the MiniSky side.
Every name in `plugins = [...]` must be installed in the environment running
`tangram serve`.

```bash
cd example_plugins/tangram
uv run tangram serve --config /path/to/tangram.toml   # workspace route
# or, with the uv tool route (from anywhere): tangram serve --config /path/to/tangram.toml
```

Open <http://localhost:2346>. With MiniSky running you should see, within a
couple of seconds: the MiniSky chip in the top bar (state, aircraft count,
sim time), the "MiniSky Simulator" sidebar widget, and any aircraft on the
map (yellow; orange when in conflict). Click an aircraft to select it and a
trail grows behind it. The widget's Run/Hold/speed buttons and command box go
through the same `from:minisky:command` path verified in step 2.

## Developing the plugin

The layout follows the
[tangram out-of-tree plugin guide](https://mode-s.org/tangram/plugins/frontend/):
`tsconfig.json` extends tangram-core's exported `tsconfig.plugin.json`
(browser plugin code), `tsconfig.vite.json` extends `tsconfig.node.json`
(the vite config itself). One known wart, straight from that guide:
tangram-core v0.5.0 publishes no type declaration for its `vite-plugin`
subpath, so `vite.config.ts` carries a `// @ts-expect-error` on the import —
drop it once upstream ships declarations.

All checks run from the workspace root, `example_plugins/tangram/`. The
`justfile` bundles the Python (ruff/pyright) and frontend (eslint/vue-tsc/tsc)
checks so the pre-commit loop is just two commands:

```bash
just fmt      # ruff --fix + ruff format + pnpm lint:fix
just check    # ruff, ruff format --check, pyright, and pnpm check
```

The underlying frontend scripts (run them directly for a tighter loop):

```bash
pnpm --filter ./tangram_minisky lint             # eslint (typescript-eslint + eslint-plugin-vue)
pnpm --filter ./tangram_minisky typecheck        # vue-tsc over src/ (.ts + .vue)
pnpm --filter ./tangram_minisky typecheck:vite   # tsc over vite.config.ts
pnpm check                                        # all frontend checks, every workspace package
pnpm build                                        # bundle into dist-frontend/
```

Python type-checking lives in this workspace (not the repo root) precisely so
`pyright` can resolve `tangram_core`; the repo-root pyright excludes
`example_plugins/tangram`.

The edit loop with the workspace route: edit → `just check` →
`pnpm build` → restart `uv run tangram serve`. Only the uv tool route needs
the `uv tool install ... --force` reinstall after a rebuild.

### Running against a local tangram checkout (`tangram_exe`)

To develop against a tangram *checkout* instead of the published
`tangram_core`, use the
[`tangram_exe` pattern from the tangram backend guide](https://mode-s.org/tangram/plugins/backend/):
a small execution-environment project outside both repositories whose
`uv.sources` point at each local tree, e.g.

```
workspace/
├── tangram/          # tangram checkout
├── minisky/          # this repository
└── tangram_exe/
    ├── pyproject.toml
    └── tangram.toml
```

```toml
# tangram_exe/pyproject.toml
[project]
name = "tangram-exe"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = ["tangram-core", "tangram-minisky"]

[tool.uv.sources]
tangram-core = { path = "../tangram/packages/tangram_core", editable = true }
tangram-minisky = { path = "../minisky/example_plugins/tangram/tangram_minisky", editable = true }
```

```bash
cd tangram_exe
uv sync
uv run tangram serve --config tangram.toml
```

Both installs are editable, so rebuilding either frontend (tangram-core's
own, or this plugin's `pnpm build`) only needs a serve restart. Note that
`tangram check-plugin` compares the plugin's npm dependency ranges against
the *installed* tangram-core's expectations, so its verdict can differ
between a checkout and the published release; the ranges in this repo track
the published `tangram_core`.

The `tangram_exe` project above wires up the *Python* side against a local
tangram. The `@open-aviation/tangram-core` **npm** dependency is independent —
to build this plugin's bundle against a local tangram-core JavaScript checkout,
uncomment the `overrides` block in `example_plugins/tangram/pnpm-workspace.yaml`
(pointing it at your local `packages/tangram_core`) and run `pnpm i`. It is a
deliberately ugly manual toggle: **re-comment it and rerun `pnpm i` before
committing**, so the lockfile never pins a machine-local `link:` path.

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
