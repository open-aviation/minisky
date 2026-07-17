# Architecture

MiniSky keeps BlueSky's core simulation model but removes the GUI, networking, and node
management. What remains is a single-process simulator built around a handful of global
singleton objects and a text-command stack.

## The singletons

Calling [`minisky.init()`][minisky.init] creates the module-level singletons that the rest
of the code refers to:

| Singleton | Class | Role |
| --- | --- | --- |
| `minisky.sim` | [`Simulation`][minisky.simulation.simulation.Simulation] | Simulation clock, timestep, and state machine |
| `minisky.traf` | [`Traffic`][minisky.traffic.traffic.Traffic] | All per-aircraft state and the flight-dynamics update |
| `minisky.runner` | [`Runner`][minisky.simulation.runner.Runner] | Async loop that calls `sim.step()` at a controllable rate |
| `minisky.scr` | [`ConsoleIO`][minisky.simulation.console.ConsoleIO] | Text output buffer (console and REST API read from it) |
| `minisky.navdb` | [`Navdatabase`][minisky.tools.navdata.Navdatabase] | Waypoints, airports, and airways loaded from parquet files |

```python
import minisky

minisky.init()          # create the singletons
minisky.load_plugins()  # optional: load plugins enabled in settings.yml
```

## The simulation loop

The simulation advances in discrete timesteps of `sim.simdt` seconds (default 1 s). One
call to [`sim.step()`][minisky.simulation.simulation.Simulation.step] does, in order:

1. **Stack processing** — pending text commands are parsed and executed
   ([`stack.process()`][minisky.stack.process]).
2. **Time advance** — `sim.simt` and the simulated UTC clock move forward by `simdt`
   (only in the `OP` state).
3. **Plugin pre-update** — timed plugin functions registered with the `preupdate` hook.
4. **Traffic update** — [`traf.update()`][minisky.traffic.traffic.Traffic.update]
   integrates aircraft state: autopilot/FMS logic, conflict detection and resolution,
   aircraft performance limits, wind, and finally position integration.
5. **Plugin update** — timed plugin functions registered with the `update` hook.

The simulation state machine has four states, exposed as constants on the `minisky`
package: `INIT` (waiting for traffic), `OP` (running), `HOLD` (paused), and `END`.
The simulation switches from `INIT` to `OP` automatically as soon as there is traffic
or pending scenario commands.

### Real time vs. fast time

There are two ways to drive the loop:

- **Manual stepping** — call `sim.step()` yourself in a plain loop. Each call advances the
  simulation by `simdt` simulated seconds, as fast as your CPU allows. This is what you
  want when embedding MiniSky in your own code or experiments.
- **The runner** — `await minisky.runner.run()` steps the simulation once per wall-clock
  interval. `runner.speed = 10` makes simulated time pass 10× faster than wall time, and
  `runner.forward(seconds)` fast-forwards by stepping at the maximum rate until the target
  simulation time is reached. The REST API server and `minisky run` both use the runner.

## Per-aircraft arrays: `TrafficArrays`

Aircraft state is stored as NumPy arrays (and lists for strings), one element per
aircraft, spread across many objects: `traf.lat`, `traf.alt`, `traf.ap.route`,
`traf.perf.mass`, and so on. Keeping all of these in sync when aircraft are created and
deleted is the job of [`TrafficArrays`][minisky.core.trafficarrays.TrafficArrays].

Classes that hold per-aircraft data derive from it and register their arrays:

```python
class Example(Entity):
    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            self.npassengers = np.array([])
```

`TrafficArrays` instances form a tree rooted at `traf`. When an aircraft is created or
deleted, the whole tree is walked and every registered array grows or shrinks in lockstep,
so index `i` refers to the same aircraft everywhere.

## The command stack

Every text command — whether it comes from a scenario file, the REST `stack/` endpoint,
or the console — goes through the same interpreter: [`minisky.stack`](api/stack.md).

- Commands are queued with [`stack.stack("CRE KL001 B738 52 4 90 FL100 250")`][minisky.stack.stack]
  and executed on the next `sim.step()`.
- Each command is a [`Command`][minisky.stack.Command] object with typed
  parameters. Argument strings like `"callsign,wpt,[alt,spd]"` are parsed by
  [`minisky.stack.argparser`](api/stack.md#argument-parsing), which knows aviation types
  (`alt` accepts `FL100`, ft, or m; `spd` accepts CAS knots or Mach; `latlon` resolves
  navaid names to coordinates).
- The built-in command table lives in `minisky/stack/commands.py`; plugins add commands
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
  in `settings.yml`). Candidate pairs are pre-selected with a KD-tree on projected
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

Simulation code reports through `minisky.scr` (a
[`ConsoleIO`][minisky.simulation.console.ConsoleIO]), which buffers echo text instead of
printing it. The REST API's `stack/` endpoint sends a command, waits for the stack to
process it, then reads the buffer back to the HTTP client — which is how the console shows
you command responses from a simulator running in another process.
