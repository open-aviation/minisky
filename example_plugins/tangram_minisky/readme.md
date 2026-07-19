# tangram_minisky

A thin [tangram](https://github.com/open-aviation/tangram) frontend plugin that
renders live traffic from a MiniSky simulator.

This package is **frontend-only** and deliberately disposable: it contains no
business logic and no simulation state. Everything simulator-side (unit
conversion, snapshot publishing, command handling) lives in MiniSky's own
`TANGRAM` plugin (`example_plugins/tangram.py`), which talks to tangram
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
(`tangram_channel` in MiniSky's `settings.yml`, `channel` in tangram's
`tangram.toml` under `[plugins.tangram_minisky]`).

## Build, check & install

This package is a **uv workspace member** of the MiniSky repository and its
layout follows the
[tangram out-of-tree plugin guide](https://mode-s.org/tangram/plugins/frontend/)
(`tsconfig.json` extends tangram-core's `tsconfig.plugin.json`,
`tsconfig.vite.json` extends `tsconfig.node.json`; `vite.config.ts` carries a
`// @ts-expect-error` because tangram-core v0.5.0 ships no declarations for
the vite-plugin subpath).

```bash
npm install
npm run check          # eslint + vue-tsc + tsc (vite config)
npm run build          # produces dist-frontend/ (bundle + plugin.json)
```

Python side (from the MiniSky repo root): `uv sync --all-packages` installs
`tangram_core` and this package editable into the repo `.venv`; ruff and
pyright cover it via the normal repo-root commands, and
`uv run tangram check-plugin example_plugins/tangram_minisky` validates the
packaging.

With that workspace install, `uv run tangram serve` from the repo root loads
the plugin directly — a rebuild only needs `npm run build` plus a serve
restart. To install into a *separate* environment instead:

```bash
uv tool install tangram_core --with ./ --force
```

(or `uv pip install ./` into a plain venv). Rerun the install after every
`npm run build`. Either way, enable it in `tangram.toml`:

```toml
[core]
plugins = ["tangram_minisky"]
```

The full end-to-end walkthrough (Redis, MiniSky side, tangram.toml,
troubleshooting) is in the MiniSky docs:
[Streaming to a tangram map](../../docs/guides/tangram.md).

## Run the simulator side

In the MiniSky repo:

```bash
uv sync --all-packages --extra tangram
# settings.yml: enabled_plugins: ['TANGRAM'], tangram_redis_url pointing at
# the same Redis instance tangram uses
minisky server        # or: minisky run --scenario scenarios/kl204.scn
```

Debug the transport without any frontend:

```bash
redis-cli psubscribe "to:*"
redis-cli publish "from:minisky:command" '{"command": "CRE KL204 B744 52 4 90 FL300 250"}'
```
