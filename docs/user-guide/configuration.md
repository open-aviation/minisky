# Configuration

Minisky can be customised with a TOML file. The CLI looks for it under your user config directory by default:

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
    minisky run --scenario example.scn --config ./experiment.toml
    minisky server --config ./server.toml
    ```

=== "Python"

    Construct the configuration explicitly:

    ```python
    from minisky import MagneticDeclinationGrid, MiniSky, MiniSkyConfig, NavData

    config = MiniSkyConfig.from_path("experiment.toml")
    with MiniSky(
        config,
        navdata=NavData(),
        magnetic_declination=MagneticDeclinationGrid.load_default(),
    ) as runtime:
        ...
    ```

    To use the default configuration values, use `MiniSkyConfig()`.

Note that all key value pairs are validated on runtime against the [`MiniSkyConfig` class][minisky.MiniSkyConfig].

## Configure plugins

Plugins are **not auto-loaded** by default on startup. To ensure that [`runtime.plugins.load_configured()`][minisky.PluginManager.load_configured] discovers the plugin, enable it in your config TOML.

```toml title="config.toml"
[plugins.example]
```

See the [plugin user guide](./plugins.md) for more information on how discovery and manual loading works.
