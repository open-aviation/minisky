"""MiniSky configuration."""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Annotated

import annotated_types
from pydantic import BaseModel, ConfigDict, Field


class MiniSkySettings(BaseModel):
    """Validated, immutable settings for the MiniSky runtime."""

    model_config = ConfigDict(frozen=True, extra="allow")

    prefer_compiled: bool = True
    asas_dtlookahead: Annotated[float, Field(), annotated_types.Ge(0)] = 300.0
    asas_pzr: Annotated[float, Field(), annotated_types.Gt(0)] = 5.0
    asas_pzh: Annotated[float, Field(), annotated_types.Gt(0)] = 1000.0
    asas_marh: Annotated[float, Field(), annotated_types.Gt(0)] = 1.05
    asas_marv: Annotated[float, Field(), annotated_types.Gt(0)] = 1.05
    plugin_path: Annotated[str, Field(), annotated_types.MinLen(1)] = "plugins"
    # TODO(abraham): remove when we implement out-of-tree plugins
    enabled_plugins: tuple[str, ...] = ()

    @classmethod
    def from_file(cls, path: str | Path) -> MiniSkySettings:
        """Load and validate settings from a TOML file."""
        with Path(path).expanduser().open("rb") as file:
            return cls.model_validate(tomllib.load(file))


#
# compat
#

DEFAULT_SETTINGS_FILE = Path(__file__).parent.parent.parent / "settings.toml"
PACKAGE_DATA_DIR = Path(__file__).parent.parent / "data"

# TODO(abraham): remove these module-level compatibility settings once all
# callers receive MiniSkySettings explicitly.
filename_settings = DEFAULT_SETTINGS_FILE
default_settings = MiniSkySettings.from_file(filename_settings)
prefer_compiled = default_settings.prefer_compiled
asas_dtlookahead = default_settings.asas_dtlookahead
asas_pzr = default_settings.asas_pzr
asas_pzh = default_settings.asas_pzh
asas_marh = default_settings.asas_marh
asas_marv = default_settings.asas_marv
plugin_path = default_settings.plugin_path
enabled_plugins = list(default_settings.enabled_plugins)


def data(path: str) -> Path:
    """Return an absolute path inside the package data directory."""
    # NOTE(abraham): in the case where we need to distribute as a wheel this
    # should be removed.
    return PACKAGE_DATA_DIR / path
