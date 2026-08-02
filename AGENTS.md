# MiniSky agent guide

Guidance for coding agents (Claude Code, etc.) working in this repository.

## What this is

MiniSky is a minimal, hackable command-line air traffic simulator — a fork of [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) that strips out the GUI, networking, and multi-node architecture. It targets three uses: running scenario files, driving the simulator over a REST API, and embedding the simulation in your own Python code. The ongoing refactor direction (see `readme.md` task list) is to *remove* features toward a bare minimum, not add them.

Before evaluating or porting an upstream BlueSky PR/feature, check `docs/upstream.md` — it logs upstream changes that were already considered and deliberately rejected (with rationale).

## Commands

Read `docs/guides/cli.md`.

The `api` marker is excluded via `addopts = -m 'not api'`; API tests start a real FastAPI process and must be run explicitly.

### Running the simulator

```bash
minisky run --scenario scenarios/kl204.scn [--speed 10]  # headless scenario run
minisky server [--reload]                                  # REST API server
minisky console                                            # interactive console against the API
```

The FastAPI app lives in `packages/minisky/minisky/server.py`; `minisky server` is the CLI entry point
(default `0.0.0.0:8000`; configure `[server]` in TOML or override with `--host` and `--port`).

## Architecture

Full details in `docs/architecture.md` — read it before making structural changes. The essentials:

**Runtime ownership.** [`MiniSky`][minisky.runtime.MiniSky] owns one simulator object graph: config, simulation, traffic, runner, console, navigation, command stack, plugins, replaceables, areas, variable explorer, random generators, and streaming hub. Unlike `bluesky`, there is no package-level `traf`, `sim`, `scr`, `runner`, or `navdb`.

**Lifecycle.** Use `with MiniSky()` for manually stepped synchronous work with the default user config, or pass `config=` explicitly. Use `async with MiniSky()` and `await runtime.run()` when running the async loop. The FastAPI lifespan owns its background runner task and awaits asynchronous cleanup.

**Simulation loop.** `sim.step()` runs, in order: stack processing → time advance (only in `OP` state) → plugin `preupdate` → `traf.update()` (autopilot/FMS, conflict detection+resolution, performance limits, wind, position integration) → plugin `update`. States: `INIT`, `OP`, `HOLD`, `END`. Drive it either by calling `sim.step()` manually (embedding) or via `runner.run()` (wall-clock paced; `runner.speed` and `runner.forward()`).

**Per-aircraft arrays (`TrafficArrays`).** Aircraft state lives in parallel NumPy arrays/lists spread across many objects (`traf.lat`, `traf.perf.mass`, `traf.ap.route`, …), one element per aircraft. Classes holding per-aircraft data derive from `Entity`/`TrafficArrays` and register arrays inside `with self.settrafarrays():`. The instances form a tree rooted at `traf`; create/delete walks the tree so index `i` is the same aircraft everywhere. When adding per-aircraft state, register it this way or it will desync on create/delete.

**Units.** Internal state is SI (m, m/s, deg). Aviation units (FL/ft, knots, Mach) exist only in stack commands / scenario files and are converted at the argument-parser boundary.

**I/O.** Simulation code never prints directly — it echoes into `minisky.scr` (`ConsoleIO`), a buffer. The REST `stack/` endpoint sends a command, waits for the stack to process it, then reads the buffer back to the HTTP client.

**Streaming.** Besides the poll-style REST endpoints, `minisky/streaming.py` provides a per-tick push feed. `build_snapshot()` reads the singletons and returns a JSON-serialisable, **SI-unit** `{siminfo, acdata}` dict; a `StreamHub` fans it out, published from the `update` plugin hook (rate-capped, default 10 Hz, and skipped when no client is connected). It surfaces over the `GET /stream` WebSocket. It is deliberately consumer-agnostic — raw SI on the wire, no client-specific unit conversion or field mapping. The server also exposes `GET /commands` (`{name: brief}` from `Command.cmddict`) for client autocomplete, and the `DTMULT` stack command sets the runner speed multiplier (`Runner.setspeed`) for clients that drive speed through the stack rather than the REST `/speed/{n}` endpoint.

## The command stack (critical convention)

Every text command — scenario file, REST `stack/` endpoint, or console — goes through `minisky.stack`. The built-in command table is `packages/minisky/minisky/stack/commands.py`; plugins add commands with the `@command` decorator.

**Stack command arguments are parsed at runtime from the parameter annotations.** `packages/minisky/minisky/stack/argparser.py` inspects `param.annotation` when a command is registered:
- **`Annotated` aliases (preferred):** `packages/minisky/minisky/stack/argparser.py` exports `Acid`, `Wpt`, `Alt`, `Spd`, `Vspd`, `Hdg`, `Time`, `Txt`, `String`, `OnOff`, `Lat`, `Lon` — e.g. `def selaltcmd(self, idx: int, alt: Alt, vspd: Vspd | None = None)`. These are real type hints (lint- and pyright-clean) carrying the parser key as `Annotated` metadata; unions with `None` are unwrapped.
- **Argument-spec strings in the command table** (`packages/minisky/minisky/stack/commands.py`, e.g. `"callsign,alt,[vspd]"`) are plain data looked up in the `argparsers` dict and *override* function annotations.
- **Legacy DSL strings** as annotations (`alt: "alt"`) still parse, but don't write new ones — use the `Annotated` aliases so linting works.
- A real `type` annotation gets wrapped in `Parser(type)` (called on the argument text — fine for `int`/`float`/`str`, wrong for `bool`; use `OnOff`).

`Command.callback` resolves signatures with `inspect.signature(func, eval_str=True)` (falling back to raw strings for legacy DSL annotations), so `from __future__ import annotations` is safe in command modules.

`E711`/`E712`/`E721` are ignored in `pyproject.toml` because numpy overrides `==`/`is` elementwise, so `arr == None` is intentional and *not* equivalent to `arr is None`.

`packages/minisky/minisky/traffic/asas/__init__.py` has a deliberately non-alphabetical import block wrapped in `# isort: off/on` (resolution before mvp, since MVP subclasses ConflictResolution) — don't "fix" it.

## Conventions

- Package/dependency management is **uv**. Prefix Python invocations with `uv run`.
- After adding or changing a stack command, regenerate `docs/reference/commands.md` with the gen script.
- MiniSky config defaults live only on `MiniSkyConfig`. The CLI optionally reads `default_user_config_toml_path()`; `--config` selects another file explicitly. `[plugins.<id>]` tables enable and configure plugins.
