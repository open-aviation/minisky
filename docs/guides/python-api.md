# Python library

MiniSky can be embedded in your own Python code — step the simulation yourself, read
aircraft state straight from NumPy arrays, and drive everything programmatically. This is
the fastest way to run large numbers of simulations for experiments.

## Minimal example

```python
import minisky

minisky.init()

minisky.sim.reset()
minisky.traf.cre("KL315", lat=52.0, lon=4.0, hdg=45, alt=5000, spd=250)
minisky.stack.stack("KL315 ADDWPT HELEN FL100 250")

minisky.sim.simdt = 10  # advance 10 simulated seconds per step

for _ in range(5):
    minisky.sim.step()
    print(f"t={minisky.sim.simt}s  lat={minisky.traf.lat}  lon={minisky.traf.lon}")
```

## The singletons

[`minisky.init()`][minisky.init] creates the global objects everything else hangs off
(see [Architecture](../architecture.md) for how they interact):

```python
minisky.sim      # Simulation: clock, timestep, state machine
minisky.traf     # Traffic: all per-aircraft state and subsystems
minisky.runner   # Runner: async real-time loop (optional — you can step manually)
minisky.scr      # ConsoleIO: buffered text output
minisky.navdb    # Navdatabase: waypoints, airports, airways
```

Pass a scenario to start from a file: `minisky.init(scenario="scenarios/kl204.scn")`.

## Creating and commanding aircraft

Directly through the Traffic object:

```python
minisky.traf.cre(
    "KL315",       # callsign
    actype="B738", # aircraft type (OpenAP performance model)
    lat=52.0,      # deg
    lon=4.0,       # deg
    hdg=45,        # deg
    alt=5000,      # ft (stack units)
    spd=250,       # CAS kts
)
```

Or through the stack, using the same command language as scenario files:

```python
minisky.stack.stack("CRE KL315 B738 52.0 4.0 45 5000 250")
minisky.stack.stack("KL315 ALT FL200")
minisky.stack.stack("KL315 ADDWPT HELEN FL100 250")
```

Stack commands are queued and execute on the next `sim.step()`.

## Reading state

Aircraft state lives in per-aircraft NumPy arrays on `minisky.traf` — index `i` is the
same aircraft in every array:

```python
traf = minisky.traf

traf.ntraf        # number of aircraft
traf.callsign     # list of callsigns
traf.lat, traf.lon  # position [deg]
traf.alt          # altitude [m]
traf.tas          # true airspeed [m/s]
traf.cas          # calibrated airspeed [m/s]
traf.gs           # ground speed [m/s]
traf.hdg, traf.trk  # heading / track [deg]
traf.vs           # vertical speed [m/s]
```

!!! warning "Units"
    Internal state is SI (metres, m/s). Convert with the constants in
    [`minisky.tools.aero`](../api/tools.md):

    ```python
    from minisky.tools import aero

    alt_ft = minisky.traf.alt / aero.ft
    tas_kts = minisky.traf.tas / aero.kts
    ```

Conflict detection results are on `traf.cd`:

```python
traf.cd.confpairs   # list of conflicting callsign pairs
traf.cd.tcpa        # time to closest point of approach [s]
traf.cd.tLOS        # time to loss of separation [s]
```

## Stepping vs. running

For experiments, call [`sim.step()`][minisky.simulation.simulation.Simulation.step] in a
loop — each call advances the simulation by `sim.simdt` seconds as fast as the CPU allows:

```python
minisky.sim.simdt = 1
while minisky.sim.simt < 3600:
    minisky.sim.step()
```

To run in (scaled) real time instead, use the async runner:

```python
import asyncio

minisky.runner.speed = 10       # 10x wall time
asyncio.run(minisky.runner.run())
```

## Resetting between runs

[`sim.reset()`][minisky.simulation.simulation.Simulation.reset] clears traffic, the
stack, areas, and plugin state, and rewinds the clock — use it between repeated
experiments in the same process rather than re-importing.
