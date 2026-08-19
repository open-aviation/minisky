"""MiniSky configuration."""

from __future__ import annotations

import tomllib
from os import PathLike
from pathlib import Path
from typing import Annotated, Any, TypeAlias

from annotated_types import Ge, Gt, Le
from pydantic import BaseModel, ConfigDict, Field
from pydantic.functional_validators import BeforeValidator

from minisky import quantities as q
from minisky._internal.identifiers import validate_plugin_id

PluginId: TypeAlias = Annotated[str, BeforeValidator(validate_plugin_id)]


class ServerConfig(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")

    host: str = "0.0.0.0"
    port: Annotated[int, Ge(0), Le(65535)] = 8000


class MiniSkyConfig(BaseModel):
    """Validated configuration for a MiniSky runtime."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    asas_dtlookahead: Annotated[q.DurationS[float], Ge(0)] = 300.0
    asas_pzr: Annotated[q.DistanceNM[float], Gt(0)] = 5.0
    asas_pzh: Annotated[q.VerticalDistanceFt[float], Gt(0)] = 1000.0
    asas_marh: Annotated[float, Gt(0)] = 1.05
    asas_marv: Annotated[float, Gt(0)] = 1.05
    server: ServerConfig = Field(default_factory=ServerConfig)
    plugins: dict[PluginId, dict[str, Any]] = Field(default_factory=dict)

    @classmethod
    def from_path(cls, path: str | PathLike[str]) -> MiniSkyConfig:
        """Load and validate configuration from an explicit TOML path."""
        with Path(path).expanduser().open("rb") as file:
            return cls.model_validate(tomllib.load(file))


PACKAGE_DATA_DIR = Path(__file__).parent.parent / "data"


def default_user_config_dir() -> Path:
    """Return the platform-specific default MiniSky config directory."""
    from platformdirs import user_config_path

    return user_config_path("minisky", appauthor=False)


def default_user_config_toml_path() -> Path:
    """Return the optional default MiniSky TOML config path."""
    return default_user_config_dir() / "config.toml"


def data(path: str) -> Path:
    """Return an absolute path inside the package data directory."""
    return PACKAGE_DATA_DIR / path
