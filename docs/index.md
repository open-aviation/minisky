# MiniSky

MiniSky is a hackable air traffic control simulator, a fork of
[BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) stripped down to its essentials.

It is designed to be a minimal tool for coders: no integrated graphical interface and no
complex network architecture. Uncommon commands and features are progressively removed to
reach a bare-minimum simulator that is easy to read, embed, and extend.

## Three ways to use it

<div class="grid cards" markdown>

- **Command line** — run a scenario file to completion, no interaction needed.

    ```bash
    uv run minisky run --scenario scenarios/kl204.scn
    ```

    → [Running scenarios](guides/running-scenarios.md)

- **REST API** — start a FastAPI server and drive the simulation over HTTP.

    ```bash
    uv run minisky server
    httpx "http://localhost:8000/stack/MCRE 3"
    ```

    → [REST API server](guides/rest-api.md)

- **Python library** — import `minisky` and step the simulation from your own code.

    ```python
    import minisky

    minisky.init()
    minisky.traf.cre("KL315", lat=52.0, lon=4.0, hdg=45, alt=5000, spd=250)
    minisky.sim.step()
    ```

    → [Python library](guides/python-api.md)

</div>

## What's inside

| Component | Module | What it does |
| --- | --- | --- |
| Simulation loop | [`minisky.simulation`](api/simulation.md) | Time keeping, state machine (INIT/HOLD/OP/END), async runner |
| Traffic | [`minisky.traffic`](api/traffic.md) | Per-aircraft state arrays, autopilot, routes, conflict detection & resolution, OpenAP performance |
| Command stack | [`minisky.stack`](api/stack.md) | Text-command interpreter shared by scenario files, the console, and the REST API |
| Plugins | [`minisky.plugin`](api/plugin.md) | Discover and load user plugins with per-aircraft data and stack commands |
| Tools | [`minisky.tools`](api/tools.md) | Aeronautics conversions (ISA atmosphere, CAS/TAS/Mach) and geodesy |
| Core | [`minisky.core`](api/core.md) | Settings, per-aircraft array bookkeeping (`TrafficArrays`) |

## Where to start

1. [Getting started](getting-started.md) — install and run your first simulation.
2. [Architecture](architecture.md) — how the pieces fit together.
3. [Stack commands](reference/commands.md) — every command the simulator understands.
