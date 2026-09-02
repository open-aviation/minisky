"""OpenAP performance library.

Loads and prepares aircraft performance coefficients for the OpenAP
performance model: aircraft and engine properties, kinematic flight
envelopes (WRAP model), and drag polars for fixed-wing aircraft from the
OpenAP database, plus a small local JSON database for rotorcraft. All
values are stored in SI units. The [`Coefficient`][.Coefficient] container is instantiated once by the
[`OpenAP`][minisky.OpenAP] performance model.
"""

import json
import warnings
from collections.abc import Mapping
from dataclasses import dataclass
from enum import IntEnum
from typing import Literal, TypeAlias, cast

import numpy as np
from openap import WRAP, drag, prop

from minisky import quantities as q
from minisky._internal.config import data
from minisky.types import AircraftTypeCode, IcaoAircraftTypeCode

EngineModelIdentifier: TypeAlias = str
"""Engine model identifier used by OpenAP, for example `CFM56-5B4`."""


class LiftType(IntEnum):
    FIXED_WING = 1
    ROTORCRAFT = 2


# NOTE(abraham): core currently owns OpenAP-specific rotor data because the
# multicopter plugin has to inherit from the core OpenAP implementation.
# TODO move away from core


@dataclass(frozen=True, slots=True)
class RotorEngine:
    name: str
    power: q.PowerW[float]


@dataclass(frozen=True, slots=True)
class RotorEnvelope:
    v_min: q.VelocityMps[float]
    v_max: q.TrueAirspeedMps[float]
    vs_min: q.VerticalRateMps[float]
    vs_max: q.VerticalRateMps[float]
    h_max: q.PressureAltitudeM[float]


@dataclass(frozen=True, slots=True)
class RotorAircraft:
    name: str
    n_engines: int
    mtow: q.MtowKg[float]
    oew: q.OewKg[float]
    engines: tuple[RotorEngine, ...]
    envelope: RotorEnvelope


@dataclass(frozen=True, slots=True)
class FixedWingEngine:
    max_thrust: q.ForceN[float]
    bpr: q.BypassRatio[float]
    ff_idl: q.MassFlowKgPerS[float]
    ff_app: q.MassFlowKgPerS[float]
    ff_co: q.MassFlowKgPerS[float]
    ff_to: q.MassFlowKgPerS[float]


@dataclass(frozen=True, slots=True)
class FixedWingAircraft:
    oew: q.OewKg[float]
    mtow: q.MtowKg[float]
    wing_area: q.AreaM2[float]
    wing_span: q.LengthM[float]
    engine_count: int
    engine_mount: Literal["rear", "wing"]
    default_engine: EngineModelIdentifier
    engines: dict[EngineModelIdentifier, FixedWingEngine]
    variant_engines: dict[IcaoAircraftTypeCode, EngineModelIdentifier]


@dataclass(frozen=True, slots=True)
class FixedWingLimits:
    vminto: q.CalibratedAirspeedMps[float]
    vmaxto: q.CalibratedAirspeedMps[float]
    vminic: q.CalibratedAirspeedMps[float]
    vmaxic: q.CalibratedAirspeedMps[float]
    vminer: q.CalibratedAirspeedMps[float]
    vmaxer: q.CalibratedAirspeedMps[float]
    vminap: q.CalibratedAirspeedMps[float]
    vmaxap: q.CalibratedAirspeedMps[float]
    vminld: q.CalibratedAirspeedMps[float]
    vmaxld: q.CalibratedAirspeedMps[float]
    vmo: q.CalibratedAirspeedMps[float]
    mmo: q.MachNumber[float]
    hmax: q.PressureAltitudeM[float]
    crosscl: q.PressureAltitudeM[float]
    crossde: q.PressureAltitudeM[float]
    axmax: q.AccelerationMps2[float]
    vsmax: q.VerticalRateMps[float]
    vsmin: q.VerticalRateMps[float]


@dataclass(frozen=True, slots=True)
class FixedWingDragPolar:
    cd0_clean: q.ZeroLiftDragCoefficient[float]
    k_clean: q.InducedDragFactor[float]
    e_clean: q.OswaldEfficiency[float]
    cd0_to: q.ZeroLiftDragCoefficient[float]
    k_to: q.InducedDragFactor[float]
    cd0_ld: q.ZeroLiftDragCoefficient[float]
    k_ld: q.InducedDragFactor[float]
    delta_cd_gear: q.DragCoefficient[float]


# TODO(abraham): remove the unused engine-type codes and OpenAP.engtype array?
# seems like it is dead?
ENG_TYPE_TF = 1  # turbofan, fixwing
ENG_TYPE_TP = 2  # turboprop, fixwing
ENG_TYPE_TS = 3  # turboshaft, rotor

OPENAP_DIR = data("performance/openap")


WrapStatistic = Literal["default", "minimum", "maximum"]


def _wrap_scalar(values: Mapping[str, object], statistic: WrapStatistic) -> float:
    """Read one scalar statistic from OpenAP's wider WRAP result mapping."""
    return cast(float, values[statistic])


class Coefficient:
    """Container for all aircraft performance coefficient databases.

    On construction, loads everything the OpenAP performance model needs,
    keyed by upper-case ICAO aircraft type code:
    """

    def __init__(self) -> None:
        with warnings.catch_warnings(action="ignore"):
            self.actypes_fixwing: list[IcaoAircraftTypeCode] = [
                model.upper() for model in prop.available_aircraft(use_synonym=True)
            ]
            """Uppercase ICAO aircraft type codes available through OpenAP."""
            self.acs_fixwing: dict[IcaoAircraftTypeCode, FixedWingAircraft] = (
                self._load_all_fixwing_flavor()
            )
            """Normalized fixed-wing aircraft records keyed by ICAO aircraft type code."""
            self.limits_fixwing: dict[IcaoAircraftTypeCode, FixedWingLimits] = (
                self._load_all_fixwing_envelop()
            )
            """Fixed-wing operating envelopes keyed by ICAO aircraft type code."""

            self.acs_rotor: dict[AircraftTypeCode, RotorAircraft] = self._load_all_rotor_flavor()

            self.dragpolar_fixwing: dict[IcaoAircraftTypeCode, FixedWingDragPolar] = (
                self._load_fixedwing_dragpolar()
            )
            """Fixed-wing drag polars keyed by ICAO aircraft type code."""

    def _load_all_fixwing_flavor(self) -> dict[IcaoAircraftTypeCode, FixedWingAircraft]:
        """Normalize the OpenAP aircraft and engine fields MiniSky consumes."""
        aircraft: dict[IcaoAircraftTypeCode, FixedWingAircraft] = {}
        for model in self.actypes_fixwing:
            source = prop.aircraft(model, use_synonym=True)
            engine_source = source["engine"]
            default_engine: EngineModelIdentifier = str(engine_source["default"])
            options = engine_source["options"]
            if isinstance(options, dict):
                variant_engines: dict[IcaoAircraftTypeCode, EngineModelIdentifier] = {
                    str(variant).upper(): str(engine) for variant, engine in options.items()
                }
                option_engines = variant_engines.values()
            else:
                variant_engines = {}
                option_engines = (str(engine) for engine in options)

            engine_models = dict.fromkeys((default_engine, *option_engines))
            engines: dict[EngineModelIdentifier, FixedWingEngine] = {}
            for engine_model in engine_models:
                try:
                    engine = prop.engine(engine_model)
                except ValueError:
                    # NOTE(abraham): oap can advertise optional engines without
                    # coefficient data, for example, three A21N LEAP engines.
                    # we keep their identifiers.
                    continue
                engines[engine_model] = FixedWingEngine(
                    max_thrust=float(engine["max_thrust"]),
                    bpr=float(engine["bpr"]),
                    ff_idl=float(engine["ff_idl"]),
                    ff_app=float(engine["ff_app"]),
                    ff_co=float(engine["ff_co"]),
                    ff_to=float(engine["ff_to"]),
                )

            aircraft[model] = FixedWingAircraft(
                oew=float(source["oew"]),
                mtow=float(source["mtow"]),
                wing_area=float(source["wing"]["area"]),
                wing_span=float(source["wing"]["span"]),
                engine_count=int(engine_source["number"]),
                engine_mount=cast(Literal["rear", "wing"], engine_source["mount"]),
                default_engine=default_engine,
                engines=engines,
                variant_engines=variant_engines,
            )
        return aircraft

    def _load_all_rotor_flavor(self) -> dict[AircraftTypeCode, RotorAircraft]:
        # NOTE(abraham): this legacy rotor JSON has mixed units: mass is kg,
        # speeds are m/s and altitude is m, but engine power is kW and range is
        # km. we normalise it at this boundary
        with (OPENAP_DIR / "rotor/aircraft.json").open() as file:
            raw = json.load(file)
        raw.pop("__comment")

        aircraft: dict[AircraftTypeCode, RotorAircraft] = {}
        for model, source in raw.items():
            envelope_source = source["envelop"]
            envelope = RotorEnvelope(
                v_min=float(envelope_source["v_min"]),
                v_max=float(envelope_source["v_max"]),
                vs_min=float(envelope_source["vs_min"]),
                vs_max=float(envelope_source["vs_max"]),
                h_max=float(envelope_source["h_max"]),
            )
            engines = tuple(
                RotorEngine(str(engine[0]), q.kw_to_w(float(engine[1])))
                for engine in source["engines"]
            )
            aircraft[model.upper()] = RotorAircraft(
                name=str(source["name"]),
                n_engines=int(source["n_engines"]),
                mtow=float(source["mtow"]),
                oew=float(source["oew"]),
                engines=engines,
                envelope=envelope,
            )
        return aircraft

    def _load_all_fixwing_envelop(self) -> dict[IcaoAircraftTypeCode, FixedWingLimits]:
        """Derive fixed-wing kinematic limits from the OpenAP WRAP model."""
        maximum = "maximum"
        minimum = "minimum"
        default = "default"
        limits: dict[IcaoAircraftTypeCode, FixedWingLimits] = {}
        for model in self.actypes_fixwing:
            wrap = WRAP(ac=model)
            key = model
            vminer = min(
                _wrap_scalar(wrap.initclimb_vcas(), minimum),
                _wrap_scalar(wrap.climb_const_vcas(), minimum),
                _wrap_scalar(wrap.cruise_mean_vcas(), minimum),
                _wrap_scalar(wrap.descent_const_vcas(), minimum),
                _wrap_scalar(wrap.finalapp_vcas(), minimum),
            )
            vmaxer = max(
                _wrap_scalar(wrap.initclimb_vcas(), maximum),
                _wrap_scalar(wrap.climb_const_vcas(), maximum),
                _wrap_scalar(wrap.cruise_mean_vcas(), maximum),
                _wrap_scalar(wrap.descent_const_vcas(), maximum),
                _wrap_scalar(wrap.finalapp_vcas(), maximum),
            )
            limits[key] = FixedWingLimits(
                vminto=_wrap_scalar(wrap.takeoff_speed(), minimum),
                vmaxto=_wrap_scalar(wrap.takeoff_speed(), maximum),
                vminic=_wrap_scalar(wrap.initclimb_vcas(), minimum),
                vmaxic=_wrap_scalar(wrap.initclimb_vcas(), maximum),
                vminer=float(vminer),
                vmaxer=float(vmaxer),
                vminap=_wrap_scalar(wrap.finalapp_vcas(), minimum),
                vmaxap=_wrap_scalar(wrap.finalapp_vcas(), maximum),
                vminld=_wrap_scalar(wrap.landing_speed(), minimum),
                vmaxld=_wrap_scalar(wrap.landing_speed(), maximum),
                vmo=float(vmaxer),
                mmo=_wrap_scalar(wrap.cruise_max_mach(), default),
                hmax=q.km_to_m(_wrap_scalar(wrap.cruise_max_alt(), default)),
                crosscl=q.km_to_m(_wrap_scalar(wrap.climb_cross_alt_conmach(), default)),
                crossde=q.km_to_m(_wrap_scalar(wrap.descent_cross_alt_concas(), default)),
                axmax=_wrap_scalar(wrap.takeoff_acceleration(), maximum),
                vsmax=float(
                    max(
                        _wrap_scalar(wrap.initclimb_vs(), maximum),
                        _wrap_scalar(wrap.climb_vs_pre_concas(), maximum),
                        _wrap_scalar(wrap.climb_vs_concas(), maximum),
                        _wrap_scalar(wrap.climb_vs_conmach(), maximum),
                    )
                ),
                vsmin=float(
                    min(
                        _wrap_scalar(wrap.initclimb_vs(), minimum),
                        _wrap_scalar(wrap.descent_vs_post_concas(), minimum),
                        _wrap_scalar(wrap.descent_vs_concas(), minimum),
                        _wrap_scalar(wrap.descent_vs_conmach(), minimum),
                    )
                ),
            )
        return limits

    def _load_fixedwing_dragpolar(self) -> dict[IcaoAircraftTypeCode, FixedWingDragPolar]:
        """Derive clean, takeoff, and landing drag polars from OpenAP.

        OpenAP computes non-clean drag from flap deflection; since MiniSky
        has no flap-angle concept, fixed deflections of 15 deg (takeoff) and
        40 deg (landing) are assumed.
        """
        polars: dict[IcaoAircraftTypeCode, FixedWingDragPolar] = {}
        # openap relies on flap angles to caculate nonclean drag, BS doesn't have a flap angle concept
        # we assume 15 degrees flap during takeoff and 40 degrees during landing
        flap_to = 15  # degs
        flap_ld = 40  # degs

        for model in self.actypes_fixwing:
            key = model
            source = drag.Drag(key, use_synonym=True).polar
            lambda_f = source["flaps"]["lambda_f"]
            cfc = source["flaps"]["cf/c"]
            sfs = source["flaps"]["Sf/S"]
            delta_cd_flap_to = lambda_f * cfc**1.38 * sfs * np.sin(np.deg2rad(flap_to)) ** 2
            delta_cd_flap_ld = lambda_f * cfc**1.38 * sfs * np.sin(np.deg2rad(flap_ld)) ** 2

            if self.acs_fixwing[key].engine_mount == "rear":
                delta_e_flap_to = 0.0046 * flap_to
                delta_e_flap_ld = 0.0046 * flap_ld
            else:
                delta_e_flap_to = 0.0026 * flap_to
                delta_e_flap_ld = 0.0026 * flap_ld

            aircraft = self.acs_fixwing[key]
            aspect_ratio = aircraft.wing_span**2 / aircraft.wing_area
            clean_k = float(source["clean"]["k"])
            polars[key] = FixedWingDragPolar(
                cd0_clean=float(source["clean"]["cd0"]),
                k_clean=clean_k,
                e_clean=float(source["clean"]["e"]),
                cd0_to=round(float(source["clean"]["cd0"] + delta_cd_flap_to), 3),
                k_to=round(1 / (1 / clean_k + np.pi * aspect_ratio * delta_e_flap_to), 3),
                cd0_ld=round(float(source["clean"]["cd0"] + delta_cd_flap_ld), 3),
                k_ld=round(1 / (1 / clean_k + np.pi * aspect_ratio * delta_e_flap_ld), 3),
                delta_cd_gear=float(source["gears"]),
            )
        return polars
