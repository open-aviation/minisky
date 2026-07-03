"""MiniSky settings loader.

Reads settings.yml from the project root at import time and exposes every
key/value pair as a module-level attribute (e.g.,
``minisky.core.settings.prefer_compiled``). Also provides the data()
helper that resolves paths inside the package data directory.
"""

# %%
from pathlib import Path

import yaml

filename_settings = Path(__file__).parent.parent.parent / "settings.yml"

with open(filename_settings, "r", encoding="utf-8") as file:
    data = yaml.safe_load(file)

for key, value in data.items():
    globals()[key] = value


def data(path: str):
    """Return the absolute path of a file or folder in the package data directory.

    Args:
        path: Path relative to the minisky/data directory
            (e.g., "navigation").

    Returns:
        Path: Absolute path to minisky/data/<path>.
    """
    return Path(__file__).parent.parent / "data" / path
