# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

MiniSky is a minimal, hackable command-line air traffic simulator — a fork of [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) that strips out the GUI, networking, and multi-node architecture. It targets three uses: running scenario files, driving the simulator over a REST API, and embedding the simulation in your own Python code. The ongoing refactor direction (see `readme.md` task list) is to *remove* features toward a bare minimum, not add them.

Before evaluating or porting an upstream BlueSky PR/feature, check `docs/upstream.md` — it logs upstream changes that were already considered and deliberately rejected (with rationale).

## Commands

```bash
uv run pytest                              # unit + integration (excludes api by default)
uv run pytest tests/unit                   # fast pure-function tests only
uv run pytest tests/integration/test_stack.py::test_name   # single test
uv run pytest -m api tests/test_api.py     # REST API tests — spawn a separate process, opt-in

uv run ruff check .                        # lint
uv run ruff format .                       # format (line-length 100)
uv run pyright                             # type check (standard mode)

uv run minisky commands docs               # regenerate docs/reference/commands.md after changing commands
uv run minisky docs serve                  # docs live preview
```

The `api` marker is excluded via `addopts = -m 'not api'`; API tests start a real FastAPI process and must be run explicitly.

### Running the simulator

```bash
minisky run --scenario scenarios/kl204.scn [--speed 10]  # headless scenario run
minisky server [--reload]                                  # REST API server
minisky console                                            # interactive console against the API
```

The FastAPI app lives in `minisky/server.py`; `minisky server` is the CLI entry point
(`MINISKY_HOST`/`MINISKY_PORT` env vars, default `0.0.0.0:8000`).

## Architecture

Full details in `docs/architecture.md` — read it before making structural changes. The essentials:

**Singletons.** `minisky.init()` constructs module-level singletons everything else references: `sim` (clock/state machine), `traf` (all aircraft state + flight-dynamics update), `runner` (async loop stepping at a controllable rate), `scr` (`ConsoleIO` output buffer), `navdb` (waypoints/airports/airways from parquet). They are `None` until `init()` runs. Call `load_plugins()` after `init()` to activate plugins from `settings.yml`.

**Import order in `minisky/__init__.py` is load-bearing.** `traffic` is imported last and separately because the performance model runs module-level code touching `minisky.data` (set up by the settings import). Reordering causes a circular import.

**Simulation loop.** `sim.step()` runs, in order: stack processing → time advance (only in `OP` state) → plugin `preupdate` → `traf.update()` (autopilot/FMS, conflict detection+resolution, performance limits, wind, position integration) → plugin `update`. States: `INIT`, `OP`, `HOLD`, `END`. Drive it either by calling `sim.step()` manually (embedding) or via `runner.run()` (wall-clock paced; `runner.speed` and `runner.forward()`).

**Per-aircraft arrays (`TrafficArrays`).** Aircraft state lives in parallel NumPy arrays/lists spread across many objects (`traf.lat`, `traf.perf.mass`, `traf.ap.route`, …), one element per aircraft. Classes holding per-aircraft data derive from `Entity`/`TrafficArrays` and register arrays inside `with self.settrafarrays():`. The instances form a tree rooted at `traf`; create/delete walks the tree so index `i` is the same aircraft everywhere. When adding per-aircraft state, register it this way or it will desync on create/delete.

**Units.** Internal state is SI (m, m/s, deg). Aviation units (FL/ft, knots, Mach) exist only in stack commands / scenario files and are converted at the argument-parser boundary.

**I/O.** Simulation code never prints directly — it echoes into `minisky.scr` (`ConsoleIO`), a buffer. The REST `stack/` endpoint sends a command, waits for the stack to process it, then reads the buffer back to the HTTP client.

**Streaming.** Besides the poll-style REST endpoints, `minisky/streaming.py` provides a per-tick push feed. `build_snapshot()` reads the singletons and returns a JSON-serialisable, **SI-unit** `{siminfo, acdata}` dict; a `StreamHub` fans it out, published from the `update` plugin hook (rate-capped, default 10 Hz, and skipped when no client is connected). It surfaces over the `GET /stream` WebSocket. It is deliberately consumer-agnostic — raw SI on the wire, no client-specific unit conversion or field mapping. The server also exposes `GET /commands` (`{name: brief}` from `Command.cmddict`) for client autocomplete, and the `DTMULT` stack command sets the runner speed multiplier (`Runner.setspeed`) for clients that drive speed through the stack rather than the REST `/speed/{n}` endpoint.

## The command stack (critical convention)

Every text command — scenario file, REST `stack/` endpoint, or console — goes through `minisky.stack`. The built-in command table is `minisky/stack/commands.py`; plugins add commands with the `@command` decorator.

**Stack command arguments are parsed at runtime from the parameter annotations.** `minisky/stack/argparser.py` inspects `param.annotation` when a command is registered:
- **`Annotated` aliases (preferred):** `minisky/stack/argparser.py` exports `Acid`, `Wpt`, `Alt`, `Spd`, `Vspd`, `Hdg`, `Time`, `Txt`, `String`, `OnOff`, `Lat`, `Lon` — e.g. `def selaltcmd(self, idx: int, alt: Alt, vspd: Vspd | None = None)`. These are real type hints (lint- and pyright-clean) carrying the parser key as `Annotated` metadata; unions with `None` are unwrapped.
- **Argument-spec strings in the command table** (`minisky/stack/commands.py`, e.g. `"callsign,alt,[vspd]"`) are plain data looked up in the `argparsers` dict and *override* function annotations.
- **Legacy DSL strings** as annotations (`alt: "alt"`) still parse, but don't write new ones — use the `Annotated` aliases so linting works.
- A real `type` annotation gets wrapped in `Parser(type)` (called on the argument text — fine for `int`/`float`/`str`, wrong for `bool`; use `OnOff`).

`Command.callback` resolves signatures with `inspect.signature(func, eval_str=True)` (falling back to raw strings for legacy DSL annotations), so `from __future__ import annotations` is safe in command modules.

`E711`/`E712`/`E721` are ignored in `pyproject.toml` because numpy overrides `==`/`is` elementwise, so `arr == None` is intentional and *not* equivalent to `arr is None`.

`minisky/traffic/asas/__init__.py` has a deliberately non-alphabetical import block wrapped in `# isort: off/on` (resolution before mvp, since MVP subclasses ConflictResolution) — don't "fix" it.

## Conventions

- Package/dependency management is **uv**. Prefix Python invocations with `uv run`.
- After adding or changing a stack command, regenerate `docs/reference/commands.md` with the gen script.
- `settings.yml` holds runtime config (ASAS protected-zone margins, plugin path, `enabled_plugins`).
