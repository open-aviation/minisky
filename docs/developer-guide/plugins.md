# Writing plugins

!!! note "TLDR"

    A minisky plugin is just an ordinary Python package that exports a [`Plugin`][minisky.Plugin] object and advertises it through the `minisky.plugins` [Python entry-point group](https://packaging.python.org/en/latest/specifications/entry-points/). Specify a build function that mounts Python objects through [`PluginContext.mount`][minisky.PluginContext.mount]. Mounted objects can define stack commands with [`@command`][minisky.command], simulation callbacks with [`@hook`][minisky.hook], and per-aircraft arrays with [`Entity`][minisky.Entity]. Provide alternative simulator implementations with [`@replacement`][minisky.replacement].
    
    To validate plugin-specific configuration under the `[plugins.<plugin_id>]` table, set [`Plugin.config_class`][minisky.Plugin.config_class] to a pydantic BaseModel. To manage external resources, use an async context manager.

First, scaffold out a new Python library:

```sh
uv init --lib minisky-example
```

```python title="src/minisky_example/__init__.py" hl_lines="9 13"
from minisky import Plugin, PluginContext, PluginSpec


class Example:
    pass


def build(context: PluginContext) -> PluginSpec:
    context.mount(Example())  # (2)!
    return context.finish()


plugin = Plugin(build=build)  # (1)!
```

1. [`Plugin`][minisky.Plugin] describes how the package creates its plugin. Its `build` function is called each time a [`MiniSky`][minisky.MiniSky] runtime loads the plugin.
2. [`context.mount(...)`][minisky.PluginContext.mount] attaches an object to the runtime. Note that here, we create the `Example()` object *inside* the `build` function so multiple minisky runtimes can have their own independent state.

## Register the plugin

Now that the plugin is defined, we need to *advertise* its location to minisky. Define a new [entry point](https://packaging.python.org/specifications/entry-points/) in your `pyproject.toml`:

<!-- fmt:off -->
```toml title="pyproject.toml" hl_lines="2"
--8<-- "packages/minisky-example/pyproject.toml:entry-point"
```
<!-- fmt:on -->

The left-hand side, `example`, is the **plugin ID**, used in the [`PLUGIN LOAD <plugin_id>` stack command][command.PLUGIN] and the user configuration TOML.

The right-hand side should point to the Python object we defined above.

## Configuration

minisky's [configuration TOML](../user-guide/configuration.md) supports defining *plugin-specific configuration*.

Suppose you want to accept the following in the user TOML:

```toml
[plugins.example]
message = "hello from my plugin :)"
```

Define the shape with a [Pydantic `BaseModel`](https://pydantic.dev/docs/validation/latest/concepts/models/):

```python title="src/minisky_example/__init__.py" hl_lines="5-8 21"
from minisky import Plugin, PluginContext, PluginSpec
from pydantic import BaseModel, ConfigDict


class ExampleConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    message: str


class Example:
    def __init__(self, config: ExampleConfig) -> None:
        self.config = config


def build(context: PluginContext[ExampleConfig]) -> PluginSpec:
    context.mount(Example(context.config))
    return context.finish()


plugin = Plugin(build=build, config_class=ExampleConfig)  # (1)!
```

1. When you specify [`config_class=ExampleConfig`][minisky.Plugin.config_class], minisky will first validate the plugin's configuration using Pydantic. The validated configuration can then be accessed in the `build` function via [`context.config`][minisky.PluginContext.config].

<!-- TODO(abraham): should we have a context.validate_config(context.config_raw)? -->

## Commands

To add a new stack command, use the [`@command` decorator][minisky.command]. As a simple example:

```python
from minisky import Ok, Result, command


class Example:
    def __init__(self, config: ExampleConfig) -> None:
        self.config = config

    @command
    def hello(self) -> Result[str, str]:
        """Print the configured message."""
        return Ok(self.config.message)
```

```text title="minisky console"
> PLUGIN LOAD EXAMPLE
Successfully loaded plugin EXAMPLE
> HELLO
hello from my plugin
```

More details (the decorator, parser, argument types, validation, generated documentation) are covered in the [command developer guide](./commands.md).

## Simulation hooks

To run code automatically as the [simulation advances](../concepts/basics.md#stepping), *hook* plugin methods into the simulation loop with [`minisky.hook`][].

For example, to run a method once during every normal update:

```python hl_lines="5"
from minisky import hook


class Example:
    @hook  # (1)!
    def update(self) -> None:
        self.nupdates += 1
```

1. With no argument, [`@hook`][minisky.hook] uses the method name as the hook name. A method named `update()` attaches to minisky's `update` phase.

Or periodically:

```python hl_lines="2"
class Example:
    @hook("update", interval=5.0)  # (1)!
    def sample(self, dt: float) -> None:
        self.elapsed += dt
```

1. Here, `interval=5.0` runs `sample()` every five **simulation seconds**. A `dt` parameter receives the simulated time elapsed since the previous call.

## Per-aircraft state

!!! warning

    This API may be confusing to use; a new one is currently being designed.

minisky stores aircraft state with the [structure-of-arrays model](../concepts/basics.md#state). The $i$th row of each numpy array refers to an aircraft.

To manipulate these arrays, inherit from [`Entity`][minisky.Entity] and use [`settrafarrays()`][minisky.TrafficArrays.settrafarrays] to register arrays or lists as per-aircraft state:

```python hl_lines="9 12-14"
import numpy as np

from minisky import Entity


class Example(Entity):
    def __init__(self) -> None:
        super().__init__()
        with self.settrafarrays():  # (1)!
            self.npassengers = np.array([], dtype=int)

    def create(self, n: int = 1) -> None:
        super().create(n)  # (2)!
        self.npassengers[-n:] = 0
```

Here, [`Entity.create()`][minisky.Entity.create] overrides the base class behaviour. The arrays can grow and shrink as aircraft are added, removed or reset.

Remember to mount the entity in the [`Plugin.build` function][minisky.Plugin.build]. The [`self.traffic`][minisky.Entity.traffic] object will then be available after attachment.

??? example "Detailed example"

    <!-- fmt:off -->
    ```python
    --8<-- "packages/minisky-example/src/minisky_example/__init__.py:entity"
    ```
    <!-- fmt:on -->

## External resources

In some cases, you may want to configure resources (e.g. `httpx2.AsyncClient` or background threads) on plugin startup, and shut it down cleanly. To do so, use an **async context manager**, for example:

```py
@asynccontextmanager
async def lifespan(runtime: PluginRuntime):
    bridge.start(runtime)  # (1)!
    try:
        yield  # (2)!
    finally:
        bridge.stop()  # (3)!


return context.finish(lifespan=lifespan)  # (4)!
```

1. This starts the external resource as the plugin starts up.
2. Execution is suspended here (plugin continues to load).
3. Run any cleanup here during plugin shutdown.
4. [`context.finish()`][minisky.PluginContext.finish] attaches the lifespan to this plugin.

Here, code before `yield` runs during plugin startup, and cleans up after `yield` runs during plugin shutdown.

??? example "Detailed example in the tangram plugin"

    <!-- fmt:off -->
    ```python
    --8<-- "packages/minisky-tangram/src/minisky_tangram/__init__.py:lifespan"
    ```
    <!-- fmt:on -->

## Replacing a simulator component

!!! warning

    This is a legacy API: it does not permit multiple performance models to coexist, making multi-plugin coordination difficult. It is currently being redesigned and may change without notice.

Most plugins add behavior alongside minisky. However, in some cases you may want to replace an entire core implementation. For example, to override the internal autopilot implementation:

<!-- TODO(abraham): show __init__ clearly -->

```python hl_lines="1 9"
@replacement  # (1)!
class CustomAutoPilot(Autopilot):
    def update(self) -> None:
        super().update()
        self.new_variable += 1


def build(context: PluginContext) -> PluginSpec:
    return context.finish(replacements=(CustomAutoPilot,))  # (2)!
```

1. [`@replacement`][minisky.replacement] declares `CustomAutoPilot` as an alternative implementation of its supported base component.
2. Pass the class to `replacements=` to advertise it to the core.

Users can then select an implementation with the [`SELECTIMPL` stack command][command.SELECTIMPL]:

```text
SELECTIMPL AUTOPILOT CUSTOMAUTOPILOT
```
