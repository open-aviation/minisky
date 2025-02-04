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
$ python minisky-run.py --scenario scenario/kl204.scn
```

### 2. Run simulator with REST API server

Start the simulator with a REST API endpoint for interactions:

```bash
$ fastapi run minisky-api.py
```

#### Interaction with API

Once the fastapi server is running, A simple stack command example through API:

```bash
$ curl http://localhost:8000/stack/POS%20EHAM
```

#### Console interaction

You can also use the control console to interact with the API server:

```bash
$ python minisky-console.py

# example commands
> POS EHAM                     # run any stack command
> /ic scenario/kl204.scn       # load scenario
> /all                         # show all aircraft
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

## To-do list

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
- [x] implement REST API
- [x] implement control console
- [ ] redo resource/cache data
- [ ] remove cachefile, load data from parquet instead
- [ ] check all echo, ensure print and scr.echo are consistent
- [ ] refactor datalog with pandas
- [ ] write new tests
- [ ] refactor code so import and simulation is easier