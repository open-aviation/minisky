# Writing plugins

Plugins extend the simulator without touching its code: they can hold per-aircraft data,
run periodic update functions inside the simulation loop, and add new stack commands.
The `example_plugins/` directory contains working examples.

## Anatomy of a plugin

A plugin is a Python file in the plugin directory (`plugin_path` in `settings.yml`,
default `example_plugins`) that defines an `init_plugin()` function:

```python
"""My example plugin."""
from random import randint
import numpy as np

import minisky
from minisky import plugin, stack


example = None

def init_plugin():
    """Required entry point. Returns the plugin config dict."""
    global example
    example = Example()

    return {
        "plugin_name": "EXAMPLE",     # name used by PLUGIN LOAD / settings.yml
        "update_interval": 5,         # seconds of sim time between update calls
        "update": example.update,     # called every update_interval
        # "preupdate": ...,           # called before traf.update()
        # "reset": ...,               # called on simulation reset
    }
```

The config dict registers the plugin's hooks:

| Key | Meaning |
| --- | --- |
| `plugin_name` | Uppercase name the plugin is known by |
| `update_interval` | Simulation seconds between hook calls (minimum: `sim.simdt`) |
| `preupdate` | Called each interval *before* the traffic update |
| `update` | Called each interval *after* the traffic update |
| `reset` | Called when the simulation resets |

## Per-aircraft data: `Entity`

Derive from [`Entity`][minisky.plugin.entity.Entity] and register arrays inside a
`settrafarrays()` block — they then grow and shrink automatically with aircraft creation
and deletion, staying index-aligned with `minisky.traf` (see
[Architecture](../architecture.md#per-aircraft-arrays-trafficarrays)):

```python
class Example(plugin.Entity):
    def __init__(self):
        super().__init__()
        with self.settrafarrays():
            self.npassengers = np.array([])

    def create(self, n=1):
        """Called automatically when n new aircraft are created."""
        super().create(n)
        self.npassengers[-n:] = [randint(50, 250) for _ in range(n)]

    def update(self):
        if minisky.traf.ntraf > 0:
            print(f"{minisky.traf.ntraf} aircraft, {int(sum(self.npassengers))} pax")
```

## Adding stack commands

Use the [`@stack.command`][minisky.plugin.plugin_decorators.command] decorator. The
function's docstring becomes the in-simulator help text (shown by `HELP PASSENGERS`),
and the `arguments` string declares the parameter types the
[argument parser](../api/stack.md) should use:

```python
@stack.command(name="PASSENGERS", arguments="txt,[int]")
def passengers(callsign: str, count: int = -1):
    """Set or get the number of passengers on an aircraft.

    Arguments:
    - callsign: Aircraft callsign
    - count: Number of passengers (optional, omit to query)
    """
    callsign = callsign.upper()
    if callsign not in minisky.traf.callsign:
        return False, f"Aircraft {callsign} not found"

    idx = minisky.traf.callsign.index(callsign)
    if count < 0:
        return True, f"{callsign} has {int(example.npassengers[idx])} passengers"

    example.npassengers[idx] = count
    return True, f"Set {callsign} passengers to {count}"
```

Command handlers return `(success, message)`; the message is echoed to the console or
REST client. Returning `None` counts as success with no message.

## Discovery and loading

Plugin files are *discovered* at startup by parsing their source (no import happens until
the plugin is loaded), so a broken plugin can't crash the simulator at startup.

Load plugins in any of three ways:

- **At startup** — list them in `settings.yml`:

    ```yaml
    plugin_path: example_plugins
    enabled_plugins: ['EXAMPLE']
    ```

    (Requires the host program to call [`minisky.load_plugins()`][minisky.load_plugins]
    after `init()` — `minisky run` and `minisky server` both do.)

- **From the stack** — `PLUGINS LIST` to see what's available, `PLUGINS LOAD EXAMPLE` to
  load one (`PLUGIN` works as a synonym).

- **Over the REST API** — `GET /plugins` and `GET /plugins/load/EXAMPLE`.

## A complete second example

`example_plugins/customautopilot.py` shows a plugin that subclasses a core simulator
class — have a look at both examples before writing your own.
