# Configuration

MiniSky comes with reasonable runtime defaults, so you do not need a config file to start a simulation. To learn more about the shape and defaults, read the [`MiniSkyConfig` API][minisky.MiniSkyConfig] reference.

Create a config file only when you want to override those values or load plugins automatically.

## Default location

The command-line tools look for an optional `config.toml` in MiniSky's platform-specific user config directory:

| System | Default path |
| --- | --- |
| Linux | `$XDG_CONFIG_HOME/minisky/config.toml`, or `~/.config/minisky/config.toml` when `XDG_CONFIG_HOME` is not set |
| macOS | `$XDG_CONFIG_HOME/minisky/config.toml`, or `~/Library/Application Support/minisky/config.toml` when `XDG_CONFIG_HOME` is not set |
| Windows | `%LOCALAPPDATA%\minisky\config.toml` |

Ask the installed package for the exact path on your machine:

```bash
uv run python -c "from minisky import default_user_config_toml_path; print(default_user_config_toml_path())"
```

`default_user_config_dir()` and `default_user_config_toml_path()` describe the CLI convention. They return defaults, not mandatory locations.

## Create your config

Create the directory and an empty file when you are ready to customise MiniSky:

```bash
config_path="$(uv run python -c 'from minisky import default_user_config_toml_path; print(default_user_config_toml_path())')"
mkdir -p "$(dirname "$config_path")"
touch "$config_path"
```

## Choose another file

Pass `--config` when a command should use a different TOML file:

```bash
uv run minisky run --scenario scenarios/kl204.scn --config ./experiment.toml
uv run minisky server --config ./server.toml
```

## Use config from Python

```python
from minisky import MiniSky, MiniSkyConfig

# try to load from default user path, and if it doesn't exist use defauls
with MiniSky() as runtime:
    ...

# or explicitly pass a config
with MiniSky(config=MiniSkyConfig.from_path("experiment.toml")) as runtime:
    ...

# forcefully use built-in defaults
with MiniSky(config=MiniSkyConfig()) as runtime:
    ...
```

## Configure plugins

A table under `[plugins.<id>]` supplies that plugin's config and asks `load_configured()` to load it during startup:

```toml
[plugins.example]
interval = 2.0
```

Remove the table when you do not want that plugin loaded. An empty table is enough for a plugin that uses only its own defaults:

```toml
[plugins.example]
```

Plugin config is validated by the plugin's `config_class`. Unknown fields or invalid values stop that plugin from loading.
