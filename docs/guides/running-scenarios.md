# Running scenarios

`minisky-run.py` runs a scenario file without any interaction — useful for batch
experiments and reproducing simulations.

```bash
uv run python minisky-run.py --scenario scenarios/kl204.scn
uv run python minisky-run.py --scenario scenarios/kl204.scn --speed 10
```

| Option | Default | Meaning |
| --- | --- | --- |
| `--scenario` | (required) | Scenario file to load |
| `--speed` | `1` | Simulation speed multiplier relative to wall time |

The script initialises the simulator with the scenario, loads any plugins enabled in
`settings.yml`, and runs the [`Runner`][minisky.simulation.runner.Runner] loop until the
scenario ends the simulation.

## Scenario files

A scenario file (`.scn`) is a list of time-stamped [stack commands](../reference/commands.md).
Each line has the form `HH:MM:SS.ss > COMMAND`:

```text
00:00:00.00 > CRE KL204 B738 52.0 4.0 90 FL100 250
00:00:00.00 > KL204 ADDWPT HELEN FL100 250
00:00:10.00 > KL204 ALT FL200
00:30:00.00 > QUIT
```

Commands execute when the simulation clock reaches their timestamp. Anything you can type
interactively works in a scenario file, and vice versa.

Useful commands inside scenarios:

- `IC filename` — load (chain to) another scenario file.
- `DT dt` — set the simulation timestep in seconds.
- `DTMULT factor` — fast-time multiplier.
- `HOLD` / `OP` — pause and resume.
- `QUIT` — end the simulation (this is what terminates a `minisky-run.py` run).

!!! tip
    Scenario timestamps are *simulation* time. Combined with `--speed`, a 30-minute
    scenario can run in seconds of wall time.

## Loading scenarios interactively

From the console or REST API, load a scenario with the `IC` stack command
(`IC scenarios/kl204.scn`), or POST a local file to the running server:

```bash
curl -F "file=@scenarios/kl204.scn" http://localhost:8000/scn
```

The console's `/load path/to/file.scn` command does this POST for you — handy because the
server may be running on another machine that doesn't have your scenario file.
