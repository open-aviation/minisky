# tangram_minisky

A thin [tangram](https://github.com/open-aviation/tangram) frontend plugin that
renders live traffic from a MiniSky simulator.

This package is **frontend-only** and deliberately disposable: it contains no
business logic and no simulation state. Everything simulator-side (unit
conversion, snapshot publishing, command handling) lives in MiniSky's own
`TANGRAM` plugin (`packages/minisky-tangram/src/minisky_tangram/__init__.py`), which talks to tangram
exclusively over Redis pub/sub — tangram's one transport layer that has been
stable across its recent plugin API churn. If tangram's frontend API breaks
again, only this package needs touching.

## What it registers

- a `minisky_aircraft` entity type (own type — it does not impersonate
  `jet1090_aircraft`)
- a deck.gl aircraft layer (`entities` slot) and a trail layer for selected
  aircraft (`live_trails` slot)
- a top-bar aircraft/state counter and a sidebar simulator control widget
  (run/hold/reset, speed, a stack command console)
- a trajectory authority on `api.bus` / `api.trajectory`: it appends points
  for selected aircraft and answers `TrajectoryApi.TOPIC_GET` requests, so
  other plugins can consume the simulator feed without depending on this
  package

## Wire contract (Redis, via tangram's Channel service)

- `to:<channel>:new-data` — `{aircraft, count, siminfo}` full snapshots,
  aviation units (ft/kt/fpm), jet1090-style field names
- `to:<channel>:console` — `{lines: [...]}` echoed simulator output
- `from:<channel>:command` — `{command: "OP"}` stack commands from the browser

`<channel>` defaults to `minisky` and is configurable on both sides
(`[tangram].channel` in MiniSky's `settings.toml`, `channel` in tangram's
`tangram.toml` under `[plugins.tangram_minisky]`).

## Build and run

From the MiniSky repository root:

```bash
just sync
just check
```

Run with published tangram using either route:

```bash
uv tool install tangram_core \
  --with ./packages/tangram-minisky \
  --force
tangram serve --config ../tangram_minisky_exe/tangram.toml
```

or:

```bash
cd ../tangram_minisky_exe
uv sync
uv run tangram serve --config tangram.toml
```

The complete setup and temporary local-checkout overrides are documented in
[Streaming to a tangram map](../../docs/guides/tangram.md).

## Run the simulator side

In the MiniSky repo:

```bash
just sync
# settings.toml: enabled_plugins = ["TANGRAM"], and (optionally) a [tangram] table
# with redis_url pointing at the same Redis instance tangram uses
minisky server        # or: minisky run --scenario scenarios/kl204.scn
```

Debug the transport without any frontend:

```bash
redis-cli psubscribe "to:*"
redis-cli publish "from:minisky:command" '{"command": "CRE KL204 B744 52 4 90 FL300 250"}'
```
