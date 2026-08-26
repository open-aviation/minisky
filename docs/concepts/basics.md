# Minisky 101: Basics

Minisky is a discrete-time simulator. At each timestep $i$, it takes the current state $x_i$ and computes the next state $x_{i+1}$ with some timestep $\Delta t$:
$$
x_{i+1} = f(x_i, \Delta t).
$$
This page focuses on how the state $x_i$ is represented internally and how the timestep $\Delta t$ controls simulation time.

## State

minisky stores most core aircraft state in what we call "traffic arrays", inside [`minisky.Traffic`][].

Unlike typical game engines that model a list of objects, minisky uses the struct-of-arrays (SoA) architecture, where the *attributes* of an aircraft (e.g. its position, speed, altitude) are the "columns" of a table.

<!-- TODO(abraham): add an image here -->

Take a simple example of creating an aircraft:

```py
from minisky import MiniSky


with MiniSky() as runtime:
    runtime.traffic.cre(...)
```

Here, the attributes of the aircraft (`runtime.traffic.{callsign, lat, lon...}`) are stored as separate numpy arrays. An aircraft is identified by its *row index* in these arrays. Whenever minisky creates, reads, updates or deletes aircraft, these arrays are kept aligned at all times.

This SoA architecture is also used in many minisky subsystems, including autopilot, performance modelling and conflict detection.

## Stepping

<!-- TODO(abraham): we need a graphic for this -->

When you execute [`MiniSky.run()`][minisky.MiniSky.run], the [**runner**][minisky.Runner] repeatedly calls [`Simulation.step()`][minisky.Simulation.step], which updates the state and advances the *simulation time* by the timestep [`simdt` $\Delta t$][minisky.Simulation.simdt]. Conceptually:

```python hl_lines="1 4"
runtime.simulation.simdt = 0.5  # (1)!

for _ in range(4):
    runtime.simulation.step()  # (2)!
    wait()

print(runtime.simulation.simt)
# 2.0
```

1. Each step represents half a simulation second.
2. Four steps advance two simulation seconds.

Here, the time that `wait()` depends on the *playback speed* defined by the runner.

```python hl_lines="1"
runtime.runner.speed = 10
await runtime.run()
```

With these settings, the runner targets 10 simulation seconds per real second, i.e. one step every 0.05 real seconds.

<!-- TODO(abraham): we really should update the terminology -->