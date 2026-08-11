"""Plugin configuration and the multicopter performance table.

The plugin is configured from the `[plugins.multicopter]` table of the
MiniSky user config file (validated into `MulticopterConfig` on load), and
reads its per-typecode performance data from TOML: a built-in table shipped
with the package, optionally merged with a user file so new multicopter
types can be added — or built-in ones re-tuned — without modifying source
code. The user file lives in the platform cache directory by default
(`platformdirs.user_cache_path`, e.g. `~/Library/Caches/minisky/multicopter.toml`
on macOS) or wherever `performance_path` points. Both files are validated
against the Pydantic shapes below on plugin startup; an invalid file fails
the plugin load with the validation message.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

#: Default waypoint capture radius for multicopters [m].
DEFAULT_CAPTURE_RADIUS = 10.0

#: Default state of charge below which the flight envelope is tightened [-].
DEFAULT_SOC_LOW = 0.2

#: Default maximum-speed factor applied to low-battery multicopters [-].
DEFAULT_LOWBATT_SPD_FACTOR = 0.6

#: Default maximum-climb-rate factor applied to low-battery multicopters [-].
DEFAULT_LOWBATT_VS_FACTOR = 0.5

#: Default ground speed below which a multicopter counts as stopped [m/s].
DEFAULT_GS_HOVER = 0.1

#: Default altitude tolerance for holding the selected hover altitude [m].
DEFAULT_ALT_CAPTURE = 0.5

#: Default cruise speed as a fraction of the envelope maximum, for the
#: range-derived battery-energy fallback [-].
DEFAULT_CRUISE_SPEED_FRACTION = 0.8

#: Default thrust-to-weight ratio, typical for camera/delivery multirotors [-].
DEFAULT_TWR = 2.0

#: Default flat-plate parasite drag area [m2].
DEFAULT_CDS = 0.01

#: Performance table shipped with the plugin (the built-in typecodes).
BUILTIN_PERFORMANCE_PATH = Path(__file__).parent / "data" / "multicopter.toml"

#: Airframe fields that together define a new rotor-database entry.
AIRFRAME_FIELDS = (
    "oew",
    "mtow",
    "n_engines",
    "engine_kw",
    "v_min",
    "v_max",
    "vs_min",
    "vs_max",
    "h_max",
)


class MulticopterTypeSpec(BaseModel):
    """Performance data for one multicopter typecode.

    The electric-model fields always apply. The airframe fields are only
    needed for typecodes unknown to the OpenAP rotor database: given
    together (all-or-none), they define a complete rotor entry that the
    plugin installs at runtime, so a new type needs no source edits.

    Attributes:
        battery_wh: Usable pack energy [Wh]; None derives it from
            `d_range_max` flown at cruise speed.
        cds: Flat-plate parasite drag area [m2].
        twr: Thrust-to-weight ratio at maximum thrust [-].
        oew: Operating empty weight [kg].
        mtow: Maximum take-off weight [kg].
        n_engines: Number of motors.
        engine_kw: Maximum power per motor [kW].
        v_min: Minimum speed [m/s]; make it negative so the aircraft may
            stop (`SPD 0`).
        v_max: Maximum speed [m/s].
        vs_min: Maximum descent rate (negative) [m/s].
        vs_max: Maximum climb rate [m/s].
        h_max: Ceiling [m].
        d_range_max: Maximum flight range [km], used for the range-derived
            pack energy when `battery_wh` is omitted.
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    battery_wh: float | None = Field(default=None, gt=0.0)
    cds: float = Field(default=DEFAULT_CDS, gt=0.0)
    twr: float = Field(default=DEFAULT_TWR, gt=0.0)

    oew: float | None = Field(default=None, gt=0.0)
    mtow: float | None = Field(default=None, gt=0.0)
    n_engines: int | None = Field(default=None, ge=1)
    engine_kw: float | None = Field(default=None, gt=0.0)
    v_min: float | None = None
    v_max: float | None = Field(default=None, gt=0.0)
    vs_min: float | None = Field(default=None, lt=0.0)
    vs_max: float | None = Field(default=None, gt=0.0)
    h_max: float | None = Field(default=None, gt=0.0)
    d_range_max: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def _airframe_all_or_none(self) -> MulticopterTypeSpec:
        given = [name for name in AIRFRAME_FIELDS if getattr(self, name) is not None]
        if given and len(given) < len(AIRFRAME_FIELDS):
            missing = ", ".join(sorted(set(AIRFRAME_FIELDS) - set(given)))
            raise ValueError(f"incomplete airframe data: missing {missing}")
        return self

    def has_airframe(self) -> bool:
        """Return whether this spec carries a complete airframe block."""
        return self.oew is not None


class MulticopterTypeTable(BaseModel):
    """Validated shape of a multicopter performance TOML file."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    types: dict[str, MulticopterTypeSpec] = {}

    @field_validator("types", mode="before")
    @classmethod
    def _uppercase_typecodes(cls, value: object) -> object:
        if isinstance(value, dict):
            return {str(key).upper(): spec for key, spec in value.items()}
        return value


class MulticopterConfig(BaseModel):
    """Validated `[plugins.multicopter]` configuration.

    Attributes:
        capture_radius: Waypoint capture radius for multicopters [m].
        performance_path: Explicit path of the user performance TOML; None
            reads the default cache-dir file when it exists.
        soc_low: State of charge below which the envelope is tightened [-].
        lowbatt_spd_factor: Maximum-speed factor at low battery [-].
        lowbatt_vs_factor: Maximum-climb-rate factor at low battery [-].
        gs_hover: Ground speed below which a hover counts as stopped [m/s].
        alt_capture: Altitude tolerance for holding the hover altitude [m].
        cruise_speed_fraction: Cruise speed as a fraction of the envelope
            maximum, for the range-derived battery-energy fallback [-].
    """

    model_config = ConfigDict(extra="forbid", frozen=True)

    capture_radius: float = Field(default=DEFAULT_CAPTURE_RADIUS, gt=0.0)
    performance_path: Path | None = None
    soc_low: float = Field(default=DEFAULT_SOC_LOW, ge=0.0, lt=1.0)
    lowbatt_spd_factor: float = Field(default=DEFAULT_LOWBATT_SPD_FACTOR, gt=0.0, le=1.0)
    lowbatt_vs_factor: float = Field(default=DEFAULT_LOWBATT_VS_FACTOR, gt=0.0, le=1.0)
    gs_hover: float = Field(default=DEFAULT_GS_HOVER, gt=0.0)
    alt_capture: float = Field(default=DEFAULT_ALT_CAPTURE, gt=0.0)
    cruise_speed_fraction: float = Field(default=DEFAULT_CRUISE_SPEED_FRACTION, gt=0.0, le=1.0)


def default_performance_path() -> Path:
    """Return the default user performance TOML path (platform cache dir)."""
    from platformdirs import user_cache_path

    return user_cache_path("minisky", appauthor=False) / "multicopter.toml"


def _parse_table(path: Path) -> dict[str, MulticopterTypeSpec]:
    with path.open("rb") as file:
        return MulticopterTypeTable.model_validate(tomllib.load(file)).types


def load_type_table(path: Path | None = None) -> dict[str, MulticopterTypeSpec]:
    """Load the built-in performance table, merged with the user file.

    The merge replaces whole per-typecode entries: a user entry for a
    built-in typecode overrides it completely, and new typecodes extend the
    membership set.

    Args:
        path: Explicit user TOML path (must exist). None falls back to
            `default_performance_path()`, which is optional.

    Raises:
        FileNotFoundError: An explicit path does not exist.
        pydantic.ValidationError: A file does not match the table shape.
    """
    table = _parse_table(BUILTIN_PERFORMANCE_PATH)
    if path is not None:
        table |= _parse_table(Path(path).expanduser())
    else:
        user_path = default_performance_path()
        if user_path.is_file():
            table |= _parse_table(user_path)
    return table
