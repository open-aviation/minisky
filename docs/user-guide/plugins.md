# Using plugins

A plugin is just a standard Python package installed alongside minisky.

If you are a developer writing a new plugin, see the [developer guide on writing plugins](../developer-guide/plugins.md), which will explain the internals, including entry-point groups.

___

Plugins are **not auto-loaded** by default on startup. To see installed plugins and explicitly load them:

=== "Python API"

    ```pycon title="uv run python"
    >>> from minisky import MiniSky
    >>> async def main():
    ...     async with MiniSky() as runtime:
    ...         # show all plugins
    ...         print(runtime.plugins.listing().unwrap())
    ...         # manually load one plugin
    ...         print((await runtime.plugins.load("MULTICOPTER")).unwrap())
    >>> import asyncio
    >>> asyncio.run(main())
    Loaded plugins: (none)
    Available plugins: CUSTOMAUTOPILOT, EXAMPLE, MULTICOPTER, TANGRAM
    Successfully loaded plugin MULTICOPTER
    ```

=== "CLI"

    Use the [`PLUGINS` stack command](../reference/commands.md):

    ```text title="uv run minisky console"
    > PLUGINS
    Loaded plugins: (none)
    Available plugins: CUSTOMAUTOPILOT, EXAMPLE, MULTICOPTER, TANGRAM
    > PLUGINS LOAD MULTICOPTER
    Successfully loaded plugin MULTICOPTER
    ```

To automatically load plugins on startup, add the entry in your [configuration file](./configuration.md):

```toml
[plugins.multicopter]
```

Add any plugin-specific settings into the same table.

=== "Python API"

    Explicitly load all configured plugins on startup.

    ```py
    async with MiniSky() as runtime:
        await runtime.plugins.load_configured()
        ...
    ```

=== "CLI"

    No action is needed. `minisky run` and `minisky server` should load configured plugins during setup.
