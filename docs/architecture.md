# Architecture

MiniSky keeps BlueSky's core simulation model but removes the GUI, networking,
and node management. The mutabl state is owned by a [`MiniSky`][minisky.MiniSky] runtime.

## Runtime ownership

Constructing `MiniSky` creates an independent object graph:

| Runtime attribute | Class | Role |
| --- | --- | --- |
| [`runtime.simulation`][minisky.simulation.simulation.Simulation] | [`Simulation`][minisky.simulation.simulation.Simulation] | Clock, timestep, and state machine |
| [`runtime.traffic`][minisky.traffic.traffic.Traffic] | [`Traffic`][minisky.traffic.traffic.Traffic] | Per-aircraft state and flight-dynamics update |
| [`runtime.runner`][minisky.simulation.runner.Runner] | [`Runner`][minisky.simulation.runner.Runner] | Async loop that steps the simulation |
| [`runtime.console`][minisky.simulation.console.ConsoleIO] | [`ConsoleIO`][minisky.simulation.console.ConsoleIO] | Buffered text output |
| [`runtime.navigation`][minisky.tools.navdata.Navdatabase] | [`Navdatabase`][minisky.tools.navdata.Navdatabase] | Waypoints, airports, and airways |
| [`runtime.commands`][minisky.stack.CommandStack] | [`CommandStack`][minisky.stack.CommandStack] | Command registry, queue, and scenario state |
| [`runtime.plugins`][minisky.plugin.plugin.PluginManager] | [`PluginManager`][minisky.plugin.plugin.PluginManager] | Plugin declarations, hooks, and lifespans |
| `runtime.areas` | [`AreaFilter`][minisky.tools.areafilter.AreaFilter] | Named geographic areas |
| `runtime.variables` | [`VariableExplorer`][minisky.core.varexplorer.VariableExplorer] | Runtime data inspection |
| `runtime.streaming` | `StreamHub` | Rate-capped snapshot fan-out |

```python
import asyncio

from minisky import DEFAULT_SETTINGS_FILE, MiniSky, MiniSkySettings


async def main() -> None:
    settings = MiniSkySettings.from_file(DEFAULT_SETTINGS_FILE)
    async with MiniSky(settings) as runtime:
        await runtime.plugins.load_configured()
        runtime.simulation.step()


asyncio.run(main())
```

## The simulation loop

The simulation advances in discrete timesteps of [`runtime.simulation.simdt`][minisky.simulation.simulation.Simulation] seconds (default 1 s). One
call to [`Simulation.step`][minisky.simulation.simulation.Simulation.step] does, in order:

1. `INIT` switches to `OP` when traffic or scenario work exists.
2. pending text commands are parsed and executed
   ([`CommandStack.process`][minisky.stack.CommandStack.process]).
3. [`runtime.simulation.simt`][minisky.simulation.simulation.Simulation] and the simulated UTC clock move forward by `simdt`
   (only in the `OP` state).
4. Plugin callbacks registered for the `preupdate` phase.
5. [`Traffic.update`][minisky.traffic.traffic.Traffic.update]
   integrates aircraft state: autopilot/FMS logic, conflict detection and resolution,
   aircraft performance limits, wind, and finally position integration.
6. Plugin callbacks registered for the `update` phase.
7. The runtime-owned hub publishes when subscribers are present.

The simulation state machine uses [`SimulationState`][minisky.simulation.simulation.SimulationState]:
`SimulationState.INIT` waits for traffic, `SimulationState.OP` runs,
`SimulationState.HOLD` pauses, and `SimulationState.END` stops. The simulation
switches from `SimulationState.INIT` to `SimulationState.OP` automatically as soon as there is traffic
or pending scenario commands.

### Real time vs. fast time

There are two ways to drive the loop:

- **Manual stepping** — call `runtime.simulation.step()` yourself in a plain loop. Each call advances the
  simulation by [`runtime.simulation.simdt`][minisky.simulation.simulation.Simulation] simulated seconds, as fast as your CPU allows. This is what you
  want when embedding MiniSky in your own code or experiments.
- **The runner** — `await runtime.run()` steps the simulation once per wall-clock
  interval. `runtime.runner.speed = 10` makes simulated time pass 10× faster than wall time, and
  `runtime.runner.forward(seconds)` fast-forwards by stepping at the maximum rate until the target
  simulation time is reached. The REST API server and `minisky run` both use the runner.

## Per-aircraft arrays: `TrafficArrays`

Aircraft state is stored as NumPy arrays (and lists for strings), one element per
aircraft, spread across many objects: [`runtime.traffic.lat`][minisky.traffic.traffic.Traffic], [`runtime.traffic.alt`][minisky.traffic.traffic.Traffic], [`runtime.traffic.ap.route`][minisky.traffic.route.Route],
[`runtime.traffic.perf.mass`][minisky.traffic.performance.perfoap.OpenAP], and so on. Keeping all of these in sync when aircraft are created and
deleted is the job of [`TrafficArrays`][minisky.core.trafficarrays.TrafficArrays].

Classes that hold per-aircraft data derive from it and register their arrays:

```python
class Example(Entity):
    def __init__(self, traffic):
        super().__init__(traffic)
        with self.settrafarrays():
            self.npassengers = np.array([])
```

`TrafficArrays` instances form a tree rooted at [`runtime.traffic`][minisky.traffic.traffic.Traffic]. When an aircraft is created or
deleted, the whole tree is walked and every registered array grows or shrinks in lockstep,
so index `i` refers to the same aircraft everywhere.

## The command stack

Every text command — whether it comes from a scenario file, the REST `stack/` endpoint,
or the console — goes through the same interpreter: [`minisky.stack`](api/stack.md).

- Commands are queued with `runtime.commands.stack("CRE KL001 B738 52 4 90 FL100 250")`
  and executed on the next `runtime.simulation.step()`.
- Each command is a [`Command`][minisky.stack.Command] object with typed
  parameters. Argument strings like `"callsign,wpt,[alt,spd]"` are parsed by
  [`minisky.stack.argparser`](api/stack.md#argument-parsing), which knows aviation types
  (`alt` accepts `FL100`, ft, or m; `spd` accepts CAS knots or Mach; `latlon` resolves
  navaid names to coordinates).
- The built-in command table lives in `packages/minisky/minisky/stack/commands.py`; plugins add commands
  with the [`@command`][minisky.plugin.plugin_decorators.command] decorator.
- Scenario files (`.scn`) are simply time-stamped stack commands; `IC filename` loads one.

See the [stack command reference](reference/commands.md) for every available command.

## Traffic internals

[`Traffic`][minisky.traffic.traffic.Traffic] owns the aircraft state and composes the
subsystems that act on it each timestep:

- **Autopilot / FMS** ([`autopilot.py`](api/traffic.md)) — selected altitude/speed/heading,
  LNAV/VNAV logic following a [`Route`][minisky.traffic.route.Route] of waypoints.
- **Conflict detection** (`traffic/asas/detection.py`) — pairwise state-based detection
  within a lookahead time against a protected zone (default 5 NM / 1000 ft, configurable
  in `settings.toml`). Candidate pairs are pre-selected with a KD-tree on projected
  positions plus a vertical reachability filter, so cost scales with local traffic
  density rather than N².
- **Conflict resolution** (`traffic/asas/mvp.py`) — Modified Voltage Potential resolution
  that computes avoidance vectors for aircraft in conflict.
- **Performance** (`traffic/performance/`) — [OpenAP](https://github.com/junzis/openap)-based
  model that limits speeds, climb rates, and computes fuel flow per aircraft type.
- **Wind and turbulence** — optional wind fields and turbulence affecting ground speed.

Units follow the BlueSky convention: internal state is SI (metres, m/s, seconds, degrees),
while stack commands and scenario files use aviation units (FL/ft, knots, Mach) that the
argument parsers convert on the way in.

## I/O: how output gets back to you

Simulation code reports through [`runtime.console`][minisky.simulation.console.ConsoleIO] (a
[`ConsoleIO`][minisky.simulation.console.ConsoleIO]), which buffers echo text instead of
printing it. The REST API's `stack/` endpoint sends a command, waits for the stack to
process it, then reads the buffer back to the HTTP client — which is how the console shows
you command responses from a simulator running in another process.
