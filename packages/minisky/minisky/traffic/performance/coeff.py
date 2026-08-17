"""OpenAP performance library.

Loads and prepares aircraft performance coefficients for the OpenAP
performance model: aircraft and engine properties, kinematic flight
envelopes (WRAP model), and drag polars for fixed-wing aircraft from the
OpenAP database, plus a small local JSON database for rotorcraft. All
values are stored in SI units. The [`Coefficient`][] container is instantiated once by the
[`OpenAP`][minisky.traffic.performance.perfoap.OpenAP] performance model.
"""

import json
import warnings
from collections.abc import Mapping
from enum import IntEnum
from typing import Literal, NamedTuple, TypedDict, cast

import numpy as np
from openap import WRAP, drag, prop

from minisky import quantities as q
from minisky.core.config import data
from minisky.values import AircraftTypeCode, IcaoAircraftTypeCode


class LiftType(IntEnum):
    FIXED_WING = 1
    ROTORCRAFT = 2


# NOTE(abraham): core currently owns OpenAP-specific rotor data because the
# multicopter plugin has to inherit from the core OpenAP implementation.
# TODO move away from core


class RotorEngine(NamedTuple):
    name: str
    power: q.PowerW[float]


class RotorEnvelope(TypedDict):
    v_min: q.VelocityMps[float]
    v_max: q.TrueAirspeedMps[float]
    vs_min: q.VerticalRateMps[float]
    vs_max: q.VerticalRateMps[float]
    h_max: q.PressureAltitudeM[float]


class RotorAircraft(TypedDict):
    name: str
    n_engines: int
    mtow: q.MtowKg[float]
    oew: q.OewKg[float]
    engines: list[RotorEngine]
    envelop: RotorEnvelope


class RotorLimits(TypedDict):
    vmin: q.VelocityMps[float]
    vmax: q.TrueAirspeedMps[float]
    vsmin: q.VerticalRateMps[float]
    vsmax: q.VerticalRateMps[float]
    hmax: q.PressureAltitudeM[float]


class FixedWingEngine(TypedDict):
    max_thrust: q.ForceN[float]
    bpr: q.BypassRatio[float]
    ff_idl: q.MassFlowKgPerS[float]
    ff_app: q.MassFlowKgPerS[float]
    ff_co: q.MassFlowKgPerS[float]
    ff_to: q.MassFlowKgPerS[float]


class FixedWingWing(TypedDict):
    area: q.AreaM2[float]
    span: q.LengthM[float]


class FixedWingEngineInstallation(TypedDict):
    number: int
    mount: Literal["rear", "wing"]


class FixedWingAircraft(TypedDict):
    oew: q.OewKg[float]
    mtow: q.MtowKg[float]
    wing: FixedWingWing
    engine: FixedWingEngineInstallation
    engines: dict[str, FixedWingEngine]


class FixedWingLimits(TypedDict):
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


class FixedWingDragPolar(TypedDict):
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
ENG_TYPE_TS = 3  # turboshlft, rotor

OPENAP_DIR = data("performance/openap")


WrapStatistic = Literal["default", "minimum", "maximum"]


def _wrap_scalar(values: Mapping[str, object], statistic: WrapStatistic) -> float:
    """Read one scalar statistic from OpenAP's wider WRAP result mapping."""
    return values[statistic]  # type: ignore


class Coefficient:
    """Container for all aircraft performance coefficient databases.

    On construction, loads everything the OpenAP performance model needs,
    keyed by upper-case ICAO aircraft type code:
    """

    def __init__(self) -> None:
        with warnings.catch_warnings(action="ignore"):
            self.actypes_fixwing: list[str] = prop.available_aircraft(use_synonym=True)
            """Fixed wing type codes from OpenAP"""
            self.acs_fixwing: dict[str, FixedWingAircraft] = self._load_all_fixwing_flavor()
            """OpenAP fixed-wing aircraft and engine records keyed by ICAO type."""
            self.limits_fixwing: dict[str, FixedWingLimits] = self._load_all_fixwing_envelop()
            """OpenAP fixed-wing operating envelopes keyed by ICAO type."""

            self.acs_rotor: dict[str, RotorAircraft] = self._load_all_rotor_flavor()
            self.limits_rotor: dict[str, RotorLimits] = self._load_all_rotor_envelop()
            self.actypes_rotor: list[str] = list(self.acs_rotor.keys())

            self.dragpolar_fixwing: dict[str, FixedWingDragPolar] = self._load_fixedwing_dragpolar()
            """Fixed-wing drag-polar records keyed by ICAO type."""

    def _load_all_fixwing_flavor(self) -> dict[IcaoAircraftTypeCode, FixedWingAircraft]:
        """Normalize the OpenAP aircraft and default-engine fields MiniSky consumes."""
        aircraft: dict[IcaoAircraftTypeCode, FixedWingAircraft] = {}
        for model in self.actypes_fixwing:
            source = prop.aircraft(model, use_synonym=True)
            engine_source = prop.engine(source["engine"]["default"])
            engine_name = str(engine_source["name"])
            aircraft[model.upper()] = {
                "oew": float(source["oew"]),
                "mtow": float(source["mtow"]),
                "wing": {
                    "area": float(source["wing"]["area"]),
                    "span": float(source["wing"]["span"]),
                },
                "engine": {
                    "number": int(source["engine"]["number"]),
                    "mount": cast(Literal["rear", "wing"], source["engine"]["mount"]),
                },
                "engines": {
                    engine_name: {
                        "max_thrust": float(engine_source["max_thrust"]),
                        "bpr": float(engine_source["bpr"]),
                        "ff_idl": float(engine_source["ff_idl"]),
                        "ff_app": float(engine_source["ff_app"]),
                        "ff_co": float(engine_source["ff_co"]),
                        "ff_to": float(engine_source["ff_to"]),
                    }
                },
            }
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
            envelope: RotorEnvelope = {
                "v_min": float(envelope_source["v_min"]),
                "v_max": float(envelope_source["v_max"]),
                "vs_min": float(envelope_source["vs_min"]),
                "vs_max": float(envelope_source["vs_max"]),
                "h_max": float(envelope_source["h_max"]),
            }
            engines = [
                RotorEngine(str(engine[0]), q.kw_to_w(float(engine[1])))
                for engine in source["engines"]
            ]
            aircraft[model.upper()] = {
                "name": str(source["name"]),
                "n_engines": int(source["n_engines"]),
                "mtow": float(source["mtow"]),
                "oew": float(source["oew"]),
                "engines": engines,
                "envelop": envelope,
            }
        return aircraft

    def _load_all_fixwing_envelop(self) -> dict[IcaoAircraftTypeCode, FixedWingLimits]:
        """Derive fixed-wing kinematic limits from the OpenAP WRAP model."""
        maximum = "maximum"
        minimum = "minimum"
        default = "default"
        limits: dict[IcaoAircraftTypeCode, FixedWingLimits] = {}
        for model in self.actypes_fixwing:
            wrap = WRAP(ac=model)
            key = model.upper()
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
            limits[key] = {
                "vminto": _wrap_scalar(wrap.takeoff_speed(), minimum),
                "vmaxto": _wrap_scalar(wrap.takeoff_speed(), maximum),
                "vminic": _wrap_scalar(wrap.initclimb_vcas(), minimum),
                "vmaxic": _wrap_scalar(wrap.initclimb_vcas(), maximum),
                "vminer": float(vminer),
                "vmaxer": float(vmaxer),
                "vminap": _wrap_scalar(wrap.finalapp_vcas(), minimum),
                "vmaxap": _wrap_scalar(wrap.finalapp_vcas(), maximum),
                "vminld": _wrap_scalar(wrap.landing_speed(), minimum),
                "vmaxld": _wrap_scalar(wrap.landing_speed(), maximum),
                "vmo": float(vmaxer),
                "mmo": _wrap_scalar(wrap.cruise_max_mach(), default),
                "hmax": q.km_to_m(_wrap_scalar(wrap.cruise_max_alt(), default)),
                "crosscl": q.km_to_m(_wrap_scalar(wrap.climb_cross_alt_conmach(), default)),
                "crossde": q.km_to_m(_wrap_scalar(wrap.descent_cross_alt_concas(), default)),
                "axmax": _wrap_scalar(wrap.takeoff_acceleration(), maximum),
                "vsmax": float(
                    max(
                        _wrap_scalar(wrap.initclimb_vs(), maximum),
                        _wrap_scalar(wrap.climb_vs_pre_concas(), maximum),
                        _wrap_scalar(wrap.climb_vs_concas(), maximum),
                        _wrap_scalar(wrap.climb_vs_conmach(), maximum),
                    )
                ),
                "vsmin": float(
                    min(
                        _wrap_scalar(wrap.initclimb_vs(), minimum),
                        _wrap_scalar(wrap.descent_vs_post_concas(), minimum),
                        _wrap_scalar(wrap.descent_vs_concas(), minimum),
                        _wrap_scalar(wrap.descent_vs_conmach(), minimum),
                    )
                ),
            }
        return limits

    def _load_all_rotor_envelop(self) -> dict[str, RotorLimits]:
        # NOTE(abraham): the same envelope is stored twice (`acs_rotor.envelop`
        # and `limits_rotor`) because the inherited OpenAP code expects both
        # shapes
        return {
            model: {
                "vmin": aircraft["envelop"]["v_min"],
                "vmax": aircraft["envelop"]["v_max"],
                "vsmin": aircraft["envelop"]["vs_min"],
                "vsmax": aircraft["envelop"]["vs_max"],
                "hmax": aircraft["envelop"]["h_max"],
            }
            for model, aircraft in self.acs_rotor.items()
        }

    def _load_fixedwing_dragpolar(self) -> dict[str, FixedWingDragPolar]:
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
            key = model.upper()
            source = drag.Drag(key, use_synonym=True).polar
            lambda_f = source["flaps"]["lambda_f"]
            cfc = source["flaps"]["cf/c"]
            sfs = source["flaps"]["Sf/S"]
            delta_cd_flap_to = lambda_f * cfc**1.38 * sfs * np.sin(np.deg2rad(flap_to)) ** 2
            delta_cd_flap_ld = lambda_f * cfc**1.38 * sfs * np.sin(np.deg2rad(flap_ld)) ** 2

            if self.acs_fixwing[key]["engine"]["mount"] == "rear":
                delta_e_flap_to = 0.0046 * flap_to
                delta_e_flap_ld = 0.0046 * flap_ld
            else:
                delta_e_flap_to = 0.0026 * flap_to
                delta_e_flap_ld = 0.0026 * flap_ld

            wing = self.acs_fixwing[key]["wing"]
            aspect_ratio = wing["span"] ** 2 / wing["area"]
            clean_k = float(source["clean"]["k"])
            polars[key] = {
                "cd0_clean": float(source["clean"]["cd0"]),
                "k_clean": clean_k,
                "e_clean": float(source["clean"]["e"]),
                "cd0_to": round(float(source["clean"]["cd0"] + delta_cd_flap_to), 3),
                "k_to": round(1 / (1 / clean_k + np.pi * aspect_ratio * delta_e_flap_to), 3),
                "cd0_ld": round(float(source["clean"]["cd0"] + delta_cd_flap_ld), 3),
                "k_ld": round(1 / (1 / clean_k + np.pi * aspect_ratio * delta_e_flap_ld), 3),
                "delta_cd_gear": float(source["gears"]),
            }
        return polars
