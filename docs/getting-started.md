# Getting started

## Requirements

- Python ≥ 3.11
- [uv](https://docs.astral.sh/uv/) (recommended) or pip

## Installation

Clone the repository and install the dependencies:

```bash
git clone https://github.com/open-aviation/minisky.git
cd minisky
uv sync
```

## Your first simulation

Run one of the bundled scenarios to completion:

```bash
uv run python minisky-run.py --scenario scenarios/kl204.scn
```

The simulator reads the scenario file, executes its timed stack commands, and steps the
simulation until the scenario ends. Use `--speed` to run faster than real time:

```bash
uv run python minisky-run.py --scenario scenarios/2ac_converging.scn --speed 10
```

## Interactive use

Start the REST API server:

```bash
uv run fastapi dev minisky-api.py
```

Then, in another terminal, talk to it — either directly over HTTP:

```bash
httpx "http://localhost:8000/stack/MCRE 3"   # create 3 random aircraft
httpx "http://localhost:8000/all"            # list all aircraft states
```

or through the interactive console:

```bash
uv run python minisky-console.py
> MCRE 3
> POS KL204
> /all
```

See the [REST API](guides/rest-api.md) and [console](guides/console.md) guides for the
full set of endpoints and console commands.

## From Python

```python
import minisky

minisky.init()

minisky.sim.reset()
minisky.traf.cre("KL315", lat=52.0, lon=4.0, hdg=45, alt=5000, spd=250)
minisky.stack.stack("KL315 ADDWPT HELEN FL100 250")

minisky.sim.simdt = 10  # 10-second timesteps

for _ in range(5):
    minisky.sim.step()
    print(f"t={minisky.sim.simt}s  lat={minisky.traf.lat}  lon={minisky.traf.lon}")
```

See the [Python library guide](guides/python-api.md) for details on the singleton objects
and stepping the simulation yourself.

## Configuration

Runtime settings live in `settings.yml` at the repository root, e.g. conflict-detection
lookahead time and protected-zone sizes, the plugin search directory, and which plugins to
load at startup:

```yaml
asas_dtlookahead: 300      # ASAS lookahead time [sec]
asas_pzr: 5                # ASAS horizontal protected zone radius [nm]
asas_pzh: 1000             # ASAS vertical protected zone height [ft]

plugin_path: example_plugins
# enabled_plugins: ['EXAMPLE']
```

## Running the tests

```bash
uv run pytest                             # unit + integration tests
uv run pytest tests/unit                  # fast pure-function tests only
uv run pytest -m api tests/test_api.py    # REST API smoke tests (separate process)
```

## Building this documentation

The documentation is built with [MkDocs Material](https://squidfunk.github.io/mkdocs-material/)
and [mkdocstrings](https://mkdocstrings.github.io/); the API reference is generated from the
docstrings in the source code.

```bash
uv sync --group docs
uv run --group docs mkdocs serve    # live preview at http://localhost:8000
uv run --group docs mkdocs build    # static site in site/
```

To refresh the [stack command reference](reference/commands.md) after adding or changing
commands:

```bash
uv run python scripts/gen_command_docs.py
```
