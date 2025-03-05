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
python minisky-run.py --scenario scenario/kl204.scn
```

### 2. Run simulator with REST API server

Start the simulator with a REST API endpoint for interactions:

```bash
fastapi dev minisky-api.py
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


#### Console interaction

You can also use the control console to interact with the API server:

```bash
python minisky-console.py

# bluesky stack commands, with prefix "/"
> /POS EHAM                     # show all aircraft in EHAM
> /mcre 3                       # create 3 aircraft

# miniscky specific commands, without prefix
> load scenario/kl204.scn       # load a local scenario file with POST
> all                           # show all aircraft
> conflicts                     # show all conflicts
> exit                          # exit the console
```

### 3. Use the simulation in a package

Use the simulator in your Python code:

```python
import minisky

minisky.init()

minisky.traf.cre('KL315', lat=52.0, lon=4.0, hdg=45, alt=10000, spd=250)

for i in range(5):
    minisky.sim.step(10)
    print(f"step-{i} positions: {minisky.traf.lat} {minisky.traf.lon}")
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
- [x] implement control console
- [ ] check all echo, ensure print and scr.echo are consistent
- [ ] add new tests
- [ ] refactor code so import and simulation is easier