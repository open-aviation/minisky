# MiniSky - A minimal command line air traffic simulator with REST API

MiniSky is a hackable air traffic control simulator, a fork of [BlueSky](https://github.com/TUDelft-CNS-ATM/bluesky).

It is designed to be a minimal tool for coders. There will be no integrated graphical interface, no complex network architecture, and no support for plugins. Uncommon commands and features will be slowly removed to reach a bare minimum simulator.

MiniSky is being optimized for:

- use in command-line
- interact with the simulator through REST API
- call simulations in your own Python code

## Usage

### 1. Run a scenario file without interaction

Run the simulator with a scenario file:

```bash
python minisky-run.py --scenario scenarios/kl204.scn
```

### 2. Run simulator with REST API server

Start the simulator with a REST API endpoint for interactions:

```bash
fastapi dev minisky-api.py    # development server

# or, after `pip install` / `uv sync`, the installed console script:
minisky-server                # serves on 0.0.0.0:8000 (MINISKY_HOST / MINISKY_PORT)
```

#### Interaction with API

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
python minisky-console.py

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
import minisky

minisky.init()

minisky.sim.reset()
minisky.traf.cre('KL315', lat=52.0, lon=4.0, hdg=45, alt=5000, spd=250)
minisky.stack.stack('KL315 ADDWPT HELEN FL100 250')

minisky.sim.simdt = 10

for i in range(5):
    minisky.sim.step()
    print(f"time-{minisky.sim.simt}s, positions: {minisky.traf.lat} {minisky.traf.lon}")
```

## Documentation

The documentation lives in `docs/` and is built with MkDocs Material; the API reference
is generated from the docstrings with mkdocstrings.

```bash
uv sync --group docs
uv run --group docs mkdocs serve     # live preview at http://localhost:8000
uv run --group docs mkdocs build     # static site in site/
```

Regenerate the stack command reference after adding or changing commands:

```bash
uv run python scripts/gen_command_docs.py
```

## Tests

Run the test suite with pytest:

```bash
uv run pytest                        # unit + integration tests
uv run pytest tests/unit             # fast pure-function tests only
uv run pytest -m api tests/test_api.py   # REST API smoke tests (separate process)
```

## Tasks

- [x] remove discoverable mode
- [x] remove server and client mode
- [x] remove legacy performance model
- [x] remove BADA performance model
- [x] remove pygame
- [x] remove GUI
- [x] remove plugin
- [x] remove multiple nodes
- [x] remove calculator
- [x] remove data logger 
- [x] remove plotter
- [x] removed metaclass and replaceable classes
- [x] remove datalog
- [x] remove cachefile, load data from parquet instead
- [x] remove signals and wall-time events
- [x] refactor resource/cache data
- [x] implement REST API
- [x] add real-time streaming API (`/stream`, `/commands`) and installable `minisky-server`
- [x] implement control console
- [x] better time and simulation speed control
- [x] refactor route functions
- [x] refactor acid to callsign
- [x] check all echo, ensure print and scr.echo are consistent
- [x] add new tests
- [x] add docstrings and documentation website (mkdocs)
- [x] remove stale `docs/commands.csv` and `docs/tutorial.pdf` (superseded by the generated command reference)
- [x] fix latent bugs found during the documentation pass
