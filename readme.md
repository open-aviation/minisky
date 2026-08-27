# minisky

minisky is an air traffic simulation library, a stripped-down and modernised fork of [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky) and a successor to [AirTrafficSim](https://github.com/HKUST-OCTAD-LAB/AirTrafficSim). It removes much of the infrastructure tied to BlueSky's application, including the bundled GUI, distributed simulation nodes, and uncommon command. The goal of minisky is to create a compact library that is easy to understand and extend.

minisky:

- retains familiar and proven BlueSky workflows: `.scn` scenario files, the core traffic/navigation model, OpenAP performance models, and conflict detection/resolution algorithms
- provides independently distributable plugins with explicit lifecycle management
- replaces process-wide mutable globals with explicit runtime ownership
- uses declarative, typed command parsing with machine-readable command schemas
- uses strong static typing and explicit units
- is headless-first, with interactive visualisation provided separately by [tangram](https://github.com/open-aviation/tangram) as a plugin
- is being extended towards first-class support for heterogeneous traffic and performance models, spanning fixed-wing aircraft, advanced air mobility, and airport surface operations

## Quickstart

minisky is currently under heavy development and is not yet available on PyPI.

Install from a source checkout:

```bash
git clone https://github.com/open-aviation/minisky.git
cd minisky

# with uv
uv sync --all-packages --all-extras
# with pip
cd packages/minisky && pip install .
```

### CLI Usage

```sh
# run a scenario file at ten times real-time speed
uv run minisky run --scenario scenarios/kl204.scn --speed 10
# or, start a REST API (0.0.0.0:8000 by default)
uv run minisky server

uvx httpx "http://localhost:8000/stack/POS EHAM"
uvx httpx "http://localhost:8000/stack/MCRE 3"
uvx httpx "http://localhost:8000/all"
uvx httpx "http://localhost:8000/conflicts"
uvx httpx "http://localhost:8000/commands"
# with the REST API running, attach an interactive console
uv run minisky console
> POS EHAM  # show all aircraft at Schiphol
> MCRE 3    # create random aircraft
> /all
> /conflicts
> /commands
```

### Python Usage

Manually step through a simulation for 50 seconds, with intervals of 10 seconds using the *typed API*:

```py
from minisky import MiniSky
from minisky import quantities as q
from minisky.types import CasMps, StdPressureAltM

with MiniSky() as runtime:
    runtime.traffic.cre(
        "KL315",
        lat=52.0,
        lon=4.0,
        hdg=45,
        alt=StdPressureAltM(q.ft_to_m(5000.0)),
        airspeed=CasMps(q.kt_to_mps(250.0)),
    )

    runtime.simulation.simdt = 10
    for _ in range(5):
        runtime.simulation.step()

    print(runtime.simulation.simt)
    print(runtime.traffic.lat, runtime.traffic.lon)
```

Alternatively, use the scenario command language:

```py
from minisky import MiniSky

with MiniSky() as runtime:
    runtime.commands.stack("CRE KL315 A320 52 4 45 FL050 250KT[CAS]")
    runtime.commands.stack("KL315 ADDWPT HELEN FL100 250KT[CAS]")
    runtime.simulation.step()

    print(runtime.traffic.callsign)
```

Or, drive a scenario file:

```py
import asyncio

from minisky import MiniSky


async def main() -> None:
    async with MiniSky(scenario="scenarios/kl204.scn") as runtime:
        runtime.runner.speed = 10
        await runtime.run()


asyncio.run(main())
```

Visit the documentation for more details on the commands, scenarios, TOML configuration and plugins!
