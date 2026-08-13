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

**Runtime ownership.** [`MiniSky`][minisky.runtime.MiniSky] owns one simulator object graph: config, simulation, traffic, runner, console, navigation, command stack, plugins, replaceables, shapes, variable explorer, random generators, and streaming hub. Unlike `bluesky`, there is no package-level `traf`, `sim`, `scr`, `runner`, or `navdb`.

**Lifecycle.** Use `with MiniSky()` for manually stepped synchronous work with the default user config, or pass `config=` explicitly. Use `async with MiniSky()` and `await runtime.run()` when running the async loop. The FastAPI lifespan owns its background runner task and awaits asynchronous cleanup.

**Simulation loop.** `sim.step()` runs, in order: stack processing → time advance (only in `OP` state) → plugin `preupdate` → `traf.update()` (autopilot/FMS, conflict detection+resolution, performance limits, wind, position integration) → plugin `update`. States: `INIT`, `OP`, `HOLD`, `END`. Drive it either by calling `sim.step()` manually (embedding) or via `runner.run()` (wall-clock paced; `runner.speed` and `runner.forward()`).

**Per-aircraft arrays (`TrafficArrays`).** Aircraft state lives in parallel NumPy arrays/lists spread across many objects (`traf.lat`, `traf.perf.mass`, `traf.ap.route`, …), one element per aircraft. Classes holding per-aircraft data derive from `Entity`/`TrafficArrays` and register arrays inside `with self.settrafarrays():`. The instances form a tree rooted at `traf`; create/delete walks the tree so index `i` is the same aircraft everywhere. When adding per-aircraft state, register it this way or it will desync on create/delete.

**Units and quantities.** Internal state is SI (m, m/s, deg) — including geographic/route distances, which are metres, not NM. Aviation units (FL/ft, knots, Mach) exist only in stack commands / scenario files and are converted at the argument-parser boundary. Physical values are annotated with the isqx-backed `Annotated` aliases in `minisky/quantities.py` (`q.DistanceM`, `q.TrueAirspeedMps`, …) — annotate new physical state the same way, leave the carrier type unparameterised where possible (not `q.DistanceM[np.ndarray | float]`), and convert units with the `q.ft_to_m`-style helpers instead of multiplying by unit constants.

**No sentinels.** Missing per-aircraft values use `np.ma` masked arrays — `TrafficArrays` create/delete preserves masks — or explicit state, never magic numbers (`-999`, `0.0`-means-unset). Discrete state uses enums (`WaypointType`, `FlightPhase`, `LiftType`, `WindFieldKind`, `PriorityCode`, …) rather than bare ints/strings.

**Shapes.** Area/line geometry lives in `minisky/tools/shapes.py` (`Shapes`; formerly `areafilter`). Shape constructors take typed `LatLonDegrees`, membership tests are `contains(...)` (formerly `checkInside`), and polygon containment uses shapely — matplotlib is no longer a dependency.

**I/O.** Simulation code never prints directly — it echoes into `minisky.scr` (`ConsoleIO`), a buffer. The REST `stack/` endpoint sends a command, waits for the stack to process it, then reads the buffer back to the HTTP client.

**Streaming.** Besides the poll-style REST endpoints, `minisky/streaming.py` provides a per-tick push feed. `build_snapshot()` reads the singletons and returns a JSON-serialisable, **SI-unit** `{siminfo, acdata}` dict; a `StreamHub` fans it out, published once per `Simulation.step` through the `publish_tick` callback the runtime injects (rate-capped, default 10 Hz, and skipped when no client is connected; it fires in every simulation state, so snapshots keep flowing while holding). It surfaces over the `GET /stream` WebSocket. It is deliberately consumer-agnostic — raw SI on the wire, no client-specific unit conversion or field mapping. The server also exposes `GET /commands` (`{name: brief}` from `Command.cmddict`) for client autocomplete, and the `DTMULT` stack command sets the runner speed multiplier (`Runner.setspeed`) for clients that drive speed through the stack rather than the REST `/speed/{n}` endpoint.

`packages/minisky/minisky/traffic/asas/__init__.py` has a deliberately non-alphabetical import block wrapped in `# isort: off/on` (resolution before mvp, since MVP subclasses ConflictResolution) — don't "fix" it.

## Conventions

- Package/dependency management is **uv**. Prefix Python invocations with `uv run`.
- After adding or changing a stack command, regenerate `docs/reference/commands.md` with the gen script.
- Refer to stack commands in docstrings and docs by their canonical upper-case name in single backticks — `` `QUIT` ``, `` `OP` ``, `` `RESET` `` — never lower-cased or bare. Name the canonical command only; `docs/reference/commands.md` carries the alias list, so don't write `QUIT`/`STOP` for a command and its alias. Reserve the `` `X`/`Y` `` slash form for genuinely distinct commands (`` `OP` ``/`` `HOLD` ``/`` `RESET` ``).
- MiniSky config defaults live only on `MiniSkyConfig`. The CLI optionally reads `default_user_config_toml_path()`; `--config` selects another file explicitly. `[plugins.<id>]` tables enable and configure plugins — `packages/minisky-multicopter` is the reference for a pydantic-validated plugin config backed by a TOML data table.
- Class docstrings document attributes in one `Attributes:` table; don't repeat them as an `Args:` section on `__init__`, which `merge_init_into_class` would render a second time, and don't hand-write the `Attributes: <title>` split either. `docs/griffe_attribute_tables.py` derives it: attributes assigned from an `__init__` parameter go to a `Constructor attributes` table, typed from the signature, and the rest to `Internally managed attributes`. A constructor attribute you never documented still gets a typed row, so write a row only when it deserves prose — `Simulation` and `Runner` are the reference. Because the split follows `__init__`, a parameter that also changes at runtime (`Runner.speed`, set by `DTMULT`) lands in the constructor table; note the mutation in its description. The extension also hides the member blocks `show_if_no_docstring` would render below the table, so an attribute is documented once: in the table, or in its own block when it carries a docstring of its own.
- Pyright runs in `strict` mode repo-wide. The `reportUnknown*`/`reportMissingTypeArgument` relaxations in the root `pyproject.toml` exist so unparameterised `q.*` quantity annotations stay ergonomic — don't remove them, and don't "fix" code by adding `[float]`/`[np.ndarray]` carrier parameters everywhere. The suppressions marked temporary there are scheduled for removal, not conventions to rely on.
