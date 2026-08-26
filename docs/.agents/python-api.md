# Python library

MiniSky can be embedded in your own Python code. Construct an explicit runtime,
step its simulation, and read aircraft state directly from NumPy arrays.

## Minimal example

```python
from minisky import MiniSky
from minisky import quantities as q
from minisky.types import CasMps, StdPressureAltM

with MiniSky() as runtime:
    runtime.simulation.reset()
    runtime.traffic.cre(
        "KL315",
        lat=52.0,
        lon=4.0,
        hdg=45,
        alt=StdPressureAltM(q.ft_to_m(5000.0)),
        airspeed=CasMps(q.kt_to_mps(250.0)),
    )
    runtime.commands.stack("KL315 ADDWPT HELEN FL100 250KT[CAS]")

    runtime.simulation.simdt = 10
    for _ in range(5):
        runtime.simulation.step()
        print(f"t={runtime.simulation.simt}s  lat={runtime.traffic.lat}  lon={runtime.traffic.lon}")
```


## Loading a config file

Python applications choose their own config path rather than relying on the CLI convention:

```python
from minisky import MiniSkyConfig

config = MiniSkyConfig.from_path("experiment.toml")
with MiniSky(config=config) as runtime:
    runtime.simulation.step()
```

See [Configuration](configuration.md) for the default user config location used by `minisky run` and `minisky server`.

## Runtime ownership

A [`MiniSky`][minisky.MiniSky] instance owns all mutable simulator state:

```python
runtime.simulation  # clock, timestep, and state machine
runtime.traffic  # aircraft state and traffic subsystems
runtime.runner  # optional async real-time loop
runtime.console  # buffered text output
runtime.navigation  # waypoints, airports, and airways
runtime.commands  # command registry, queue, and scenario state
runtime.plugins  # plugin declarations, hooks, and lifespans
runtime.streaming  # per-runtime snapshot fan-out
```

Pass a scenario to the constructor to queue it immediately:

```python
with MiniSky(scenario="scenarios/kl204.scn") as runtime:
    runtime.simulation.step()
```

Use `with MiniSky(...)` for manually stepped synchronous work without active plugin lifespans. Use `async with MiniSky(...)` when you load plugins or run asynchronously so plugin lifespans are closed correctly.

`await runtime.run()` runs in the current task. If your application creates a background task for it, your application owns that task and must cancel or await it before closing the runtime.

## Creating and commanding aircraft

Directly through the traffic object:

```python
runtime.traffic.cre(
    "KL315",
    actype="B738",
    lat=52.0,
    lon=4.0,
    hdg=45,
    alt=StdPressureAltM(q.ft_to_m(5000.0)),
    airspeed=CasMps(q.kt_to_mps(250.0)),
)
```

Or through the runtime-owned command stack:

```python
runtime.commands.stack("CRE KL315 B738 52.0 4.0 45 5000FT[STD] 250KT[CAS]")
runtime.commands.stack("KL315 ALT FL200")
runtime.commands.stack("KL315 ADDWPT HELEN FL100 250KT[CAS]")
```

Commands are queued and execute on the next `runtime.simulation.step()`.

## Reading state

Aircraft state lives in parallel per-aircraft arrays on [`runtime.traffic`][minisky.traffic.traffic.Traffic]:

```python
traffic = runtime.traffic

traffic.ntraf
traffic.callsign
traffic.lat, traffic.lon
traffic.alt
traffic.tas
traffic.cas
traffic.gs
traffic.hdg, traffic.trk
traffic.vs
```

!!! warning "Units"
    Internal state is SI. Convert with constants in
    [`minisky.tools.aero`](../api/tools.md):

    ```python
    from minisky.tools import aero

    alt_ft = runtime.traffic.alt / aero.ft
    tas_kts = runtime.traffic.tas / aero.kts
    ```

Conflict-detection results are available on [`runtime.traffic.cd`][minisky.traffic.asas.detection.ConflictDetection]:

```python
runtime.traffic.cd.confpairs
runtime.traffic.cd.tcpa
runtime.traffic.cd.tLOS
```

## Stepping vs. running

For experiments, step manually:

```python
runtime.simulation.simdt = 1
while runtime.simulation.simt < 3600:
    runtime.simulation.step()
```

To run a scenario with scaled wall-clock pacing:

```python
import asyncio


async def main() -> None:
    async with MiniSky(scenario="scenarios/kl204.scn") as runtime:
        await runtime.plugins.load_configured()
        runtime.runner.speed = 10
        await runtime.run()


asyncio.run(main())
```

## Resetting between runs

[`Simulation.reset`][minisky.simulation.simulation.Simulation.reset] clears
traffic, command state, areas, plugin timers, replaceable selections, and the
clock:

```python
runtime.simulation.reset()
```
