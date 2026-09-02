# Configuration

Minisky can be customised with a TOML file. Minisky (by default) expects it under your user config directory:

| System | Default path |
| --- | --- |
| Linux | `$XDG_CONFIG_HOME/minisky/config.toml`, or `~/.config/minisky/config.toml` when `XDG_CONFIG_HOME` is not set |
| macOS | `$XDG_CONFIG_HOME/minisky/config.toml`, or `~/Library/Application Support/minisky/config.toml` when `XDG_CONFIG_HOME` is not set |
| Windows | `%LOCALAPPDATA%\minisky\config.toml` |

If you are unsure, run:

```py
from minisky import default_user_config_toml_path

fp = default_user_config_toml_path()
print(fp)
# create an empty file so you can edit it
fp.parent.mkdir(parents=True, exist_ok=True)
fp.write_text("")
```

!!! tip

    MiniSky comes with reasonable runtime defaults, a config file is not mandatory. To learn more about the expected key value pairs and the defaults, read the [`MiniSkyConfig` API][minisky.MiniSkyConfig] reference.


=== "CLI"

    To explicitly pass a config:

    ```bash
    uv run minisky run --scenario packages/minisky/scenarios/kl204.scn --config ./experiment.toml
    uv run minisky server --config ./server.toml
    ```

=== "Python"

    ```python
    from minisky import MiniSky, MiniSkyConfig

    # if unspecified, it tries to load from the default user path,
    # and if it doesn't exist, use defaults.
    with MiniSky() as runtime:
        ...

    # to explicitly pass a config:
    with MiniSky(config=MiniSkyConfig.from_path("experiment.toml")) as runtime:
        ...

    # or, forcefully use built-in defaults
    with MiniSky(config=MiniSkyConfig()) as runtime:
        ...
    ```

Note that all key value pairs are validated on runtime with the [pydantic](https://github.com/pydantic/pydantic) library.

## Configure plugins

Plugins are **not auto-loaded** by default on startup. To ensure that [`runtime.plugins.load_configured()`][minisky.PluginManager.load_configured] discovers the plugin, enable it in your config TOML.

```toml title="config.toml"
[plugins.example]
```

See the [plugin user guide](./plugins.md) for more information on how discovery and manual loading works.
