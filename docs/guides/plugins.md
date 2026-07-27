# Writing plugins

Plugins extend a [`MiniSky`][minisky.MiniSky] runtime without modifying
core code. A plugin can own per-aircraft data, register timed lifecycle hooks,
and add stack commands. The `example_plugins/` directory contains working
examples.

## Anatomy of a plugin

A plugin is a Python file in the directory configured by `plugin_path` that
defines `init_plugin(runtime)`:

```python
"""My example plugin."""

from typing import TYPE_CHECKING

import numpy as np

from minisky import plugin

if TYPE_CHECKING:
    from minisky import MiniSky
    from minisky.traffic import Traffic


def init_plugin(runtime: MiniSky):
    instance = Example(runtime.traffic)
    config = {
        "plugin_name": "EXAMPLE",
        "update_interval": 5,
        "update": instance.update,
        "state": instance,
    }
    commands = {
        "PASSENGERS": [
            instance.passengers,
            "txt,[int]",
            "PASSENGERS callsign, [count]",
            "Set or get the number of passengers on an aircraft.",
        ]
    }
    return config, commands
```

The runtime is passed explicitly. Plugin code should retain only the specific
runtime components it needs instead of reading package-level aliases.

The config dictionary supports these lifecycle entries:

| Key | Meaning |
| --- | --- |
| `plugin_name` | Name used by `PLUGINS LOAD` and `enabled_plugins` |
| `update_interval` | Simulation seconds between timed callbacks |
| `preupdate` | Callback before the traffic update |
| `update` | Callback after the traffic update |
| `reset` | Callback when the simulation resets |
| `hold` | Callback when the simulation enters hold |
| `shutdown` | Callback when the owning runtime shuts down |
| `state` | Optional plugin-owned object exposed through the variable explorer |

Plugin records, loaded state, timers, hooks, and returned state belong to
`runtime.plugins`. Loading the same plugin into two runtimes creates separate
records and hook sets.

## Per-aircraft data: `Entity`

Derive from [`Entity`][minisky.plugin.entity.Entity], pass the owning traffic
object to `super().__init__()`, and register arrays inside a
`settrafarrays()` block. They then grow, shrink, and reset with that traffic
tree.

```python
class Example(plugin.Entity):
    def __init__(self, traffic: Traffic) -> None:
        super().__init__(traffic)
        with self.settrafarrays():
            self.npassengers = np.array([])

    def create(self, n: int = 1) -> None:
        super().create(n)
        self.npassengers[-n:] = 100

    def update(self) -> None:
        if self.traffic.ntraf:
            print(f"{self.traffic.ntraf} aircraft")
```

`Entity` is not a singleton and does not use a proxy. Each plugin load creates
an ordinary object attached to one runtime's traffic-array tree.

## Adding stack commands

Return a command dictionary as the second value from `init_plugin()`. Binding a
method from the plugin-owned state object keeps the command attached to the
correct runtime:

```python
class Example(plugin.Entity):
    # ...

    def passengers(self, callsign: str, count: int = -1):
        callsign = callsign.upper()
        if callsign not in self.traffic.callsign:
            return False, f"Aircraft {callsign} not found"

        index = self.traffic.callsign.index(callsign)
        if count < 0:
            return True, f"{callsign} has {int(self.npassengers[index])} passengers"

        self.npassengers[index] = count
        return True, f"Set {callsign} passengers to {count}"
```

A command entry contains the callback, argument parser specification, brief
usage text, and help text. Command handlers return `(success, message)`;
returning `None` counts as success with no message.

The [`@stack.command`][minisky.plugin.plugin_decorators.command] decorator is
also available for stateless module-level declarations. Importing a decorated
function only stores metadata. The command is registered when the owning
runtime loads that plugin module.

## Discovery and loading

Discovery parses plugin source without importing it. `MiniSky` performs this
discovery during construction.

Load plugins in any of these ways:

- **At startup** — list names under `enabled_plugins`, then call
  `runtime.load_plugins()`.
- **From the stack** — use `PLUGINS LIST` and `PLUGINS LOAD EXAMPLE`.
- **From Python** — call `runtime.plugins.load("EXAMPLE")`.
- **Over the REST API** — use `GET /plugins` and
  `GET /plugins/load/EXAMPLE`.

```python
from minisky import MiniSky, MiniSkySettings

settings = MiniSkySettings.from_file("settings.toml")
runtime = MiniSky(settings)
runtime.load_plugins()
```

## Replaceable implementations

A plugin can declare a subclass of a replaceable traffic component, such as
[`Autopilot`][minisky.traffic.autopilot.Autopilot]. Importing the class adds it
to the shared declaration catalog, while selection belongs to each runtime:

```python
runtime.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT")
```

The `SELECTIMPL` stack command calls the same runtime-owned manager. Resetting a
simulation restores base implementations only on that runtime's traffic tree.
See `example_plugins/customautopilot.py` for a complete example.
