# Writing plugins

You should use a plugin when you want to add commands, simulation hooks, per-aircraft data, external services, or a replaceable traffic component without changing MiniSky itself.

The example packages under `packages/minisky-example*` and `packages/minisky-tangram` are complete plugins you can use as starting points.

## Create a plugin package

Your package should expose a [`Plugin`][minisky.plugin.Plugin] value through the `minisky.plugins` entry-point group:

```toml
--8<-- "packages/minisky-example/pyproject.toml:entry-point"
```

The entry-point name is the plugin ID. Use lowercase letters, digits, and underscores, such as `example`, `custom_autopilot`, or `tangram`.

## Build your plugin

A build function creates fresh components for a MiniSky runtime. Mount the components you want MiniSky to manage, finish the context, and export the resulting plugin declaration:

```python
--8<-- "packages/minisky-example/src/minisky_example/__init__.py:declaration"
```

!!! important
    Create fresh component instances in every build. Do not reuse components between runtimes or load attempts.

By default, the first mounted component is also available through MiniSky's variable explorer. Pass `expose=False` when you mount additional internal components that should not be exposed.

## Accept configuration

When your plugin accepts config, define a Pydantic-compatible model and pass it as `config_class`. You can then read the validated config from `context.config`:

```python
--8<-- "packages/minisky-tangram/src/minisky_tangram/__init__.py:configuration"
```

Users place the config under the plugin ID in their [MiniSky config file](configuration.md):

```toml
[plugins.example]
interval = 2.0
```

A table under `[plugins.<id>]` both configures the plugin and loads it during normal startup. An empty table is enough for a plugin that uses only default values.

## Add per-aircraft data

Derive from [`Entity`][minisky.plugin.Entity] when your plugin needs an array or list with an entry for every aircraft. Register those values inside `settrafarrays()` so MiniSky keeps them aligned when aircraft are created, deleted, or reset.

The example plugin also shows a command and a periodic hook on the same component:

```python
--8<-- "packages/minisky-example/src/minisky_example/__init__.py:entity"
```

You can use `self.traffic` after the plugin has loaded. Do not use it in `__init__`; the entity is still detached while the plugin is being built.

## Add a command

See [commands](./commands.md)


## Add simulation hooks

Use [`@plugin.hook`][minisky.plugin.plugin_decorators.hook] for work tied to the simulation cycle:

```python
class Component:
    @plugin_api.hook("preupdate")
    def before_traffic(self) -> None: ...

    @plugin_api.hook("update", interval=2.0)
    def every_two_seconds(self, dt: float) -> None: ...
```

Available phases are `preupdate`, `update`, `reset`, and `hold`. Intervals use simulated seconds. A hook must be synchronous; if it raises an exception, MiniSky disables that hook and continues running the others.

## Own threads, tasks, and subscriptions

Use an async lifespan when your plugin opens files, starts tasks or threads, connects to a service, or subscribes to console output.

Tangram uses its lifespan to start and stop the Redis bridge:

```python
--8<-- "packages/minisky-tangram/src/minisky_tangram/__init__.py:lifespan"
```

The lifespan receives a [`PluginRuntime`][minisky.plugin.PluginRuntime] with the small set of runtime operations intended for plugins: read status or a snapshot, write to the console, subscribe to console messages, and submit a stack command after startup completes.

Put cleanup in the lifespan's `finally` block so it runs when the MiniSky runtime closes. You do not need to close console subscriptions separately when they are created through `PluginRuntime`; MiniSky owns them and closes them during teardown.

!!! note
    Lifespan startup happens before your commands, hooks, entities, and replacements are available. Start external resources there, but do not depend on your own registrations or submit stack commands until startup completes.

## Add a replaceable implementation

Use [`@plugin.replacement`][minisky.plugin.plugin_decorators.replacement] on a supported traffic component subclass, then include it in `context.finish()`:

```python
--8<-- "packages/minisky-example-customautopilot/src/minisky_example_customautopilot/__init__.py:replacement"
```

After the plugin loads, select the implementation with the stack:

```text
SELECTIMPL AUTOPILOT CUSTOMAUTOPILOT
```

You can also select it from Python with `runtime.replaceables.select(...)`. Keep replacement construction synchronous and use the plugin lifespan for external resources.

## Load and manage plugins

To attempt every plugin configured under `[plugins.<id>]`, call the plugin manager before you start the runner. A failed plugin is reported to the console without preventing later configured plugins from loading:

```python
from minisky import MiniSky, MiniSkyConfig

config = MiniSkyConfig.from_path("experiment.toml")
async with MiniSky(config=config) as runtime:
    loaded = await runtime.plugins.load_configured()
    print(f"Loaded: {', '.join(loaded)}")
    await runtime.run()
```

To load a single installed plugin, call `load()` with its plugin ID:

```python
from minisky import Err, Ok

match await runtime.plugins.load("example"):
    case Ok(message):
        print(message)
    case Err(error):
        # ... handle the error
```

Plugin IDs are case-insensitive. `load()` returns [`Result[str, str]`][minisky.result.Result]`.

To inspect the plugins known to the runtime, use `listing()`:

```python
match runtime.plugins.listing():
    case Ok(text):
        print(text)
    case Err(error):
        # ... handle the error
```

While the simulator is running, you can manage plugins through the stack instead:

```text
PLUGINS LIST
PLUGINS LOAD EXAMPLE
```

The REST API provides the same operations through `GET /plugins` and `GET /plugins/load/<id>`.
