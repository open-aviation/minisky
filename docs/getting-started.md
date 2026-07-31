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
uv run minisky run --scenario scenarios/kl204.scn
```

The simulator reads the scenario file, executes its timed stack commands, and steps the
simulation until the scenario ends. Use `--speed` to run faster than real time:

```bash
uv run minisky run --scenario scenarios/2ac_converging.scn --speed 10
```

## Interactive use

Start the REST API server:

```bash
uv run minisky server
```

Then, in another terminal, talk to it — either directly over HTTP:

```bash
httpx "http://localhost:8000/stack/MCRE 3"   # create 3 random aircraft
httpx "http://localhost:8000/all"            # list all aircraft states
```

or through the interactive console:

```bash
uv run minisky console
> MCRE 3
> POS KL204
> /all
```

See the [command-line interface](guides/cli.md), [REST API](guides/rest-api.md), and
[console](guides/console.md) guides for the full set of commands and endpoints.

## From Python

```python
from minisky import DEFAULT_SETTINGS_FILE, MiniSky, MiniSkySettings

settings = MiniSkySettings.from_file(DEFAULT_SETTINGS_FILE)
with MiniSky(settings) as runtime:
    runtime.simulation.reset()
    runtime.traffic.cre(
        "KL315", lat=52.0, lon=4.0, hdg=45, alt=5000, spd=250
    )
    runtime.commands.stack("KL315 ADDWPT HELEN FL100 250")

    runtime.simulation.simdt = 10  # 10-second timesteps

    for _ in range(5):
        runtime.simulation.step()
        print(
            f"t={runtime.simulation.simt}s  "
            f"lat={runtime.traffic.lat}  lon={runtime.traffic.lon}"
        )
```

See the [Python library guide](guides/python-api.md) for details on runtime
ownership and stepping the simulation yourself.

## Configuration

Runtime settings live in `packages/minisky/settings.toml`, e.g. conflict-detection
lookahead time and protected-zone sizes, plus the plugin startup list and configuration:

```toml
asas_dtlookahead = 300
asas_pzr = 5
asas_pzh = 1000
asas_marh = 1.05
asas_marv = 1.05

enabled_plugins = []

[plugins]
```

## Running the tests

```bash
just test      # unit + integration tests
just test-unit # fast pure-function tests only
just test-api  # REST API smoke tests (separate process)
```

## Building this documentation

The documentation is built with [Zensical](https://zensical.org/)
and [mkdocstrings](https://mkdocstrings.github.io/); the API reference is generated from the
docstrings in the source code.

```bash
uv sync --group docs
just docs-serve    # live preview at http://localhost:8000
just docs-build    # static site in site/
```
