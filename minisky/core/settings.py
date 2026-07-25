"""MiniSky settings loader.

Reads settings.toml from the project root at import time and exposes every
key/value pair as a module-level attribute (e.g.,
``minisky.core.settings.prefer_compiled``). Nested tables (e.g. ``[tangram]``)
are exposed as the corresponding ``dict``. Also provides the data() helper
that resolves paths inside the package data directory.
"""

# %%
import tomllib
from pathlib import Path

filename_settings = Path(__file__).parent.parent.parent / "settings.toml"

with open(filename_settings, "rb") as file:
    _settings = tomllib.load(file)

for key, value in _settings.items():
    globals()[key] = value

# Explicit type declarations for pyright (set dynamically above via globals())
prefer_compiled: bool
asas_dtlookahead: float
asas_pzr: float
asas_pzh: float
asas_marh: float
asas_marv: float
plugin_path: str
enabled_plugins: list[str]


def data(path: str) -> Path:
    """Return the absolute path of a file or folder in the package data directory.

    Args:
        path: Path relative to the minisky/data directory
            (e.g., "navigation").

    Returns:
        Path: Absolute path to minisky/data/<path>.
    """
    return Path(__file__).parent.parent / "data" / path
