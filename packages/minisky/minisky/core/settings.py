"""MiniSky configuration."""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Annotated

import annotated_types
from pydantic import BaseModel, ConfigDict, Field


class MiniSkySettings(BaseModel):
    """Validated settings for the MiniSky runtime."""

    # TODO(abraham): when we work on issue #24 we should add a [plugin]
    # namespace for plugin-specific config and disallow extras
    model_config = ConfigDict(frozen=True, extra="allow")

    asas_dtlookahead: Annotated[float, Field(), annotated_types.Ge(0)] = 300.0
    asas_pzr: Annotated[float, Field(), annotated_types.Gt(0)] = 5.0
    asas_pzh: Annotated[float, Field(), annotated_types.Gt(0)] = 1000.0
    asas_marh: Annotated[float, Field(), annotated_types.Gt(0)] = 1.05
    asas_marv: Annotated[float, Field(), annotated_types.Gt(0)] = 1.05
    # TODO(abraham): delete this.
    plugin_path: Annotated[str, Field(), annotated_types.MinLen(1)] = "plugins"
    enabled_plugins: tuple[str, ...] = ()

    @classmethod
    def from_file(cls, path: str | Path) -> MiniSkySettings:
        """Load and validate settings from a TOML file."""
        with Path(path).expanduser().open("rb") as file:
            return cls.model_validate(tomllib.load(file))

# TODO(abraham): delete this, require users to pass in an explicit path and
# use platformdirs for default loading
DEFAULT_SETTINGS_FILE = Path(__file__).parent.parent.parent / "settings.toml"
PACKAGE_DATA_DIR = Path(__file__).parent.parent / "data"


def data(path: str) -> Path:
    """Return an absolute path inside the package data directory."""
    return PACKAGE_DATA_DIR / path
