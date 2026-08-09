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
| `runtime.shapes` | [`Shapes`][minisky.tools.shapes.Shapes] | Named geographic areas and graphical lines |
| `runtime.variables` | [`VariableExplorer`][minisky.core.varexplorer.VariableExplorer] | Runtime data inspection |
| `runtime.streaming` | `StreamHub` | Rate-capped snapshot fan-out |

```python
import asyncio

from minisky import MiniSky


async def main() -> None:
    async with MiniSky() as runtime:
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

Aircraft state is stored as NumPy arrays (and lists for strings), with an element per
aircraft, spread across many objects: [`runtime.traffic.lat`][minisky.traffic.traffic.Traffic], [`runtime.traffic.alt`][minisky.traffic.traffic.Traffic], [`runtime.traffic.ap.route`][minisky.traffic.route.Route],
[`runtime.traffic.perf.mass`][minisky.traffic.performance.perfoap.OpenAP], and so on. Keeping all of these in sync when aircraft are created and
deleted is the job of [`TrafficArrays`][minisky.core.trafficarrays.TrafficArrays].

Classes that hold per-aircraft data derive from it and register their arrays:

```python
class Example(Entity):
    def __init__(self):
        super().__init__()
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
- Each command is compiled from its Python signature, see [commands](./guides/commands.md).

See the [stack command reference](reference/commands.md) for every available command.

## Traffic internals

[`Traffic`][minisky.traffic.traffic.Traffic] owns the aircraft state and composes the
subsystems that act on it each timestep:

- **Autopilot / FMS** ([`autopilot.py`](api/traffic.md)) — selected altitude/speed/heading,
  LNAV/VNAV logic following a [`Route`][minisky.traffic.route.Route] of waypoints.
- **Kinematics** ([`Kinematics`][minisky.traffic.kinematics.Kinematics], `runtime.traffic.kinematics`) —
  the flight integration: accelerate towards the commanded airspeed within the
  performance limits, turn towards the commanded heading at the bank-angle turn
  rate, combine with wind into a ground-speed vector, and integrate position.
  It lives in its own first-level entity (rather than on `Traffic` itself) so
  that alternative flight models can be swapped in — see
  [replaceable components](#replaceable-components) below.
- **Conflict detection** (`traffic/asas/detection.py`) — pairwise state-based detection
  within a lookahead time against a protected zone (default 5 NM / 1000 ft, configurable
  in the [config file](guides/configuration.md)). Candidate pairs are pre-selected with a KD-tree on projected
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

## Replaceable components

The traffic subsystems above are *replaceable*: a subclass can be selected in
place of the default implementation at runtime, and the live instance is swapped
immediately (its per-aircraft arrays re-seeded for the current fleet). The
runtime's [`ReplaceableManager`][minisky.core.trafficarrays.ReplaceableManager]
registers these base classes:

| Name | Base class | Role |
| --- | --- | --- |
| `ACTIVEWAYPOINT` | [`ActiveWaypoint`][minisky.traffic.activewpdata.ActiveWaypoint] | Active-leg data and waypoint-capture criterion |
| `APORASAS` | [`APorASAS`][minisky.traffic.aporasas.APorASAS] | Pilot logic selecting between autopilot and resolution commands |
| `AUTOPILOT` | [`Autopilot`][minisky.traffic.autopilot.Autopilot] | FMS / LNAV / VNAV guidance |
| `CONFLICTDETECTION` | `ConflictDetection` | Pairwise conflict detection |
| `CONFLICTRESOLUTION` | `ConflictResolution` | Conflict resolution (the core registers `MVP`) |
| `KINEMATICS` | [`Kinematics`][minisky.traffic.kinematics.Kinematics] | Flight integration |
| `OPENAP` | [`OpenAP`][minisky.traffic.performance.perfoap.OpenAP] | Aircraft performance |

Select an implementation with the `SELECTIMPL` stack command
(`SELECTIMPL AUTOPILOT MYAUTOPILOT`; without arguments it lists the
alternatives), or programmatically through
`runtime.traffic.select_implementation`. `RESET` reverts every replaceable to
its default implementation.

Plugins provide implementations by decorating a subclass with
[`@plugin.replacement`][minisky.plugin.plugin_decorators.replacement] and
declaring it when the build finishes; such replacements are local to the
runtime that loaded the plugin. The
[multicopter plugin](guides/multicopters.md) is a worked example: it registers
subclasses of five of these bases and keeps them selected from its hooks.

## I/O: how output gets back to you

Simulation code reports through [`runtime.console`][minisky.simulation.console.ConsoleIO] (a
[`ConsoleIO`][minisky.simulation.console.ConsoleIO]), which buffers echo text instead of
printing it. The REST API's `stack/` endpoint sends a command, waits for the stack to
process it, then reads the buffer back to the HTTP client — which is how the console shows
you command responses from a simulator running in another process.
