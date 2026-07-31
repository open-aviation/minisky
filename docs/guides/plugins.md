# Writing plugins

Plugins extend a [`MiniSky`][minisky.MiniSky] runtime without modifying
core code. A plugin can own per-aircraft data, register timed lifecycle hooks,
and add stack commands. The `packages/minisky-example*/` directory contains working
examples.

## Anatomy of a plugin

A plugin is an installed Python package. You should expose its
`init_plugin(runtime)` function through package metadata:

```toml
[project.entry-points."minisky.plugins"]
example = "my_package:init_plugin"
```

The entry-point name is the plugin ID used by `PLUGINS LOAD` and
`enabled_plugins`. The callable keeps the existing plugin contract:

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

Plugin records, timers, hooks, and returned state belong to
[`runtime.plugins`][minisky.plugin.plugin.PluginManager]. Each `init_plugin` call
must create runtime-specific state; imported Python modules and class declarations
are still process-wide.

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

    def passengers(self, callsign: str, count: int = -1) -> tuple[bool, str]:
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

The [`@plugin.command`][minisky.plugin.plugin_decorators.command] decorator is
also available for stateless module-level declarations. Importing a decorated
function only stores metadata. The command is registered when the owning
runtime loads that plugin module.

## Discovery and loading

Discovery reads installed entry-point metadata without importing plugin code.
`MiniSky` performs this discovery during construction. You should install the
plugin package in the same environment as MiniSky before trying to load it.

Load plugins in any of these ways:

- At startup: list names under `enabled_plugins`, then call `runtime.load_plugins()`.
- From the stack: use `PLUGINS LIST` and `PLUGINS LOAD EXAMPLE`.
- From Python: call [`runtime.plugins.load("EXAMPLE")`][minisky.plugin.plugin.PluginManager.load].
- REST API: use `GET /plugins` and `GET /plugins/load/EXAMPLE`.

```python
from minisky import MiniSky, MiniSkySettings

settings = MiniSkySettings.from_file("packages/minisky/settings.toml")
with MiniSky(settings) as runtime:
    runtime.load_plugins()
```

## Replaceable implementations

Decorate a subclass of a supported traffic component with
[`@plugin.replacement`][minisky.plugin.plugin_decorators.replacement], then pass
the class explicitly to [`PluginContext.finish`][minisky.plugin.plugin.PluginContext.finish]:

```python
@plugin.replacement
class CustomAutoPilot(Autopilot):
    ...

def build(context):
    return context.finish(replacements=(CustomAutoPilot,))
```

Loading the plugin adds that implementation only to the owning runtime. Select
it from Python or with the `SELECTIMPL` stack command:

```python
runtime.replaceables.select("AUTOPILOT", "CUSTOMAUTOPILOT")
```

Resetting the simulation restores base implementations. Plugin shutdown removes
its replacement entries from that runtime. See
`packages/minisky-example-customautopilot/src/minisky_example_customautopilot/__init__.py`
for a complete example.
