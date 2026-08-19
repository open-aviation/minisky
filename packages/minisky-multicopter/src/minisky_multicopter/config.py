"""Validated multicopter plugin configuration and performance data."""

from __future__ import annotations

import tomllib
from importlib.resources import files
from importlib.resources.abc import Traversable
from pathlib import Path
from typing import Annotated, TypeAlias

from annotated_types import Ge, IsFinite, Le, Lt, MinLen
from minisky import default_user_config_dir
from minisky import quantities as q
from minisky.types import AircraftTypeCode, Ge0, Gt0
from pydantic import BaseModel, ConfigDict, StringConstraints

from minisky_multicopter import quantities as mq

TypeCode: TypeAlias = Annotated[
    AircraftTypeCode,
    StringConstraints(min_length=1, pattern=r"^[A-Z0-9]+$"),
]


class RotorAirframeSpec(BaseModel):
    """Complete rotor-airframe data in MiniSky's internal SI units."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    oew: q.OewKg[IsFinite[Gt0[float]]]
    mtow: q.MtowKg[IsFinite[Gt0[float]]]
    n_engines: Annotated[int, Ge(1)]
    engine_power: q.PowerW[IsFinite[Gt0[float]]]
    v_min: Annotated[q.VelocityMps[IsFinite[float]], Le(0)]
    v_max: q.TrueAirspeedMps[IsFinite[Gt0[float]]]
    vs_min: Annotated[q.VerticalRateMps[IsFinite[float]], Lt(0)]
    vs_max: q.VerticalRateMps[IsFinite[Gt0[float]]]
    h_max: q.PressureAltitudeM[IsFinite[Gt0[float]]]
    range_max: q.DistanceM[IsFinite[Gt0[float]]]


class MulticopterTypeSpec(BaseModel):
    """Electric model and complete airframe for a multicopter type."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    battery_energy: q.EnergyWh[IsFinite[Gt0[float]]] | None = None
    cds: mq.FlatPlateDragAreaM2[IsFinite[Gt0[float]]] = 0.01
    twr: mq.ThrustToWeightRatio[IsFinite[Gt0[float]]] = 2.0
    airframe: RotorAirframeSpec


class MulticopterTypeTable(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    types: Annotated[dict[TypeCode, MulticopterTypeSpec], MinLen(1)]


class MulticopterConfig(BaseModel):
    """Validated `[plugins.multicopter]` configuration."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    capture_radius: q.DistanceM[IsFinite[Gt0[float]]] = 10.0
    performance_path: Path | None = None
    soc_low: Annotated[IsFinite[Ge0[float]], Lt(1)] = 0.2
    lowbatt_spd_factor: Annotated[IsFinite[Gt0[float]], Le(1)] = 0.6
    lowbatt_vs_factor: Annotated[IsFinite[Gt0[float]], Le(1)] = 0.5
    gs_hover: q.GroundSpeedMps[IsFinite[Gt0[float]]] = 0.1
    alt_capture: q.VerticalDistanceM[IsFinite[Gt0[float]]] = 0.5
    cruise_speed_fraction: mq.CruiseSpeedFraction[Annotated[IsFinite[Gt0[float]], Le(1)]] = 0.8


def default_user_performance_path() -> Path:
    """Return the conventional user-owned multicopter performance path."""
    return default_user_config_dir() / "minisky_multicopter" / "config.toml"


def packaged_performance_resource() -> Traversable:
    """Return the packaged fallback performance table."""
    return files("minisky_multicopter").joinpath("data", "config.toml")


def _parse_table(source: Path | Traversable) -> dict[str, MulticopterTypeSpec]:
    with source.open("rb") as file:
        return MulticopterTypeTable.model_validate(tomllib.load(file)).types


def load_type_table(path: Path | None = None) -> dict[str, MulticopterTypeSpec]:
    if path is not None:
        return _parse_table(Path(path).expanduser())

    user_path = default_user_performance_path()
    if user_path.is_file():
        return _parse_table(user_path)

    return _parse_table(packaged_performance_resource())
