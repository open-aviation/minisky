"""MiniSky configuration."""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Annotated, Any, TypeAlias

import annotated_types
from pydantic import BaseModel, ConfigDict, Field
from pydantic.functional_validators import BeforeValidator

from minisky.identifiers import validate_plugin_id

PluginId: TypeAlias = Annotated[str, BeforeValidator(validate_plugin_id)]


class MiniSkySettings(BaseModel):
    """Validated settings for the MiniSky runtime."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    asas_dtlookahead: Annotated[float, Field(), annotated_types.Ge(0)] = 300.0
    asas_pzr: Annotated[float, Field(), annotated_types.Gt(0)] = 5.0
    asas_pzh: Annotated[float, Field(), annotated_types.Gt(0)] = 1000.0
    asas_marh: Annotated[float, Field(), annotated_types.Gt(0)] = 1.05
    asas_marv: Annotated[float, Field(), annotated_types.Gt(0)] = 1.05
    plugins: dict[PluginId, dict[str, Any]] = Field(default_factory=dict)

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
