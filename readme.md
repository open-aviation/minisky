# MiniSky

MiniSky is a hackable air traffic control simulator, a fork of [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky).

It is designed to be a minimal tool for coders. There will be no integrated graphical interface, no complex network architecture. Uncommon commands and features will be slowly removed to reach a bare minimum simulator.

MiniSky is being optimized for:

- use in command-line
- interact with the simulator through REST API
- call simulations in your own Python code
- running multiple independent simulations side by side in one process, with each runtime owning its own state and lifecycle

## Usage

### 1. Run a scenario

```bash
uv sync
uv run minisky run --scenario scenarios/kl204.scn
uv run minisky run --scenario scenarios/kl204.scn --speed 10
```

### 2. Run the API server

```bash
uv run minisky server  # serves on 0.0.0.0:8000 by default
uv run minisky server --reload
```

### 3. Interaction with API

Once the fastapi server is running, some simple examples:

```bash
httpx "http://localhost:8000/stack/POS EHAM"

httpx "http://localhost:8000/stack/mcre 3"

httpx "http://localhost:8000/all"

httpx "http://localhost:8000/conflicts"

```

In summary:

- `stack/CMD` is the endpoint for any bluesky stack commands
- `all` is the endpoint to show all aircraft
- `conflicts` is the endpoint to show all conflicts
- `commands` returns `{name: usage}` for every stack command (autocomplete/help)

#### Real-time stream

For clients that need live updates instead of polling, connect to the `/stream`
WebSocket. It pushes one JSON snapshot per simulation step (rate-capped, default
10 Hz) in SI units, containing `siminfo` (sim time, speed, state, ...) and
`acdata` (parallel per-aircraft arrays plus conflict data):

```python
import json
from websockets.sync.client import connect

with connect("ws://localhost:8000/stream") as ws:
    while True:
        tick = json.loads(ws.recv())
        print(tick["siminfo"]["simt"], tick["acdata"]["callsign"])
```

The stream is deliberately consumer-agnostic — raw SI on the wire, so any unit
conversion or field mapping is left to the client. You can change the simulation
speed from the stack with the `DTMULT` command (e.g. `DTMULT 10`) in addition to
the `/speed/10` REST endpoint.

#### Console interaction

You can also use the control console to interact with the API server:

```bash
uv run minisky console

# bluesky stack commands, without prefix "/"
> POS EHAM                     # show all aircraft in EHAM
> mcre 3                       # create 3 aircraft

# miniscky specific commands, with prefix "/"
> /load scenario/kl204.scn       # load a local scenario file with POST
> /all                           # show all aircraft
> /conflicts                     # show all conflicts
> /exit                          # exit the console
> /speed/10                      # set simulation speed to 10
> /forward/30                    # forward simulation 30 seconds
```

Note that commands are case-insensitive.

### 3. Use the simulation in a package

Use the simulator in your Python code:

```python
from minisky import DEFAULT_SETTINGS_FILE, MiniSky, MiniSkySettings

settings = MiniSkySettings.from_file(DEFAULT_SETTINGS_FILE)
with MiniSky(settings) as runtime:
    runtime.traffic.cre(
        "KL315", lat=52.0, lon=4.0, hdg=45, alt=5000, spd=250
    )
    runtime.commands.stack("KL315 ADDWPT HELEN FL100 250")

    runtime.simulation.simdt = 10
    for _ in range(5):
        runtime.simulation.step()
        print(runtime.simulation.simt, runtime.traffic.lat, runtime.traffic.lon)
```

## Documentation

The documentation lives in `docs/` and is built with Zensical; the API reference
is generated from the docstrings with mkdocstrings.

```bash
uv sync --group docs
just docs-serve                  # live preview at http://localhost:8000
just docs-build                  # static site in site/
```

## Tests

Run the test suite with the repository `just` recipes:

```bash
just test       # unit + integration tests
just test-unit  # fast pure-function tests only
just test-api   # REST API smoke tests (separate process)
```
