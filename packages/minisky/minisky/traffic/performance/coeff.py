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
from enum import IntEnum
from typing import NamedTuple, TypedDict

import numpy as np
from openap import WRAP, drag, prop

from minisky import quantities as q
from minisky.core.config import data


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


# TODO(abraham): remove the unused engine-type codes and OpenAP.engtype array?
# seems like it is dead?
ENG_TYPE_TF = 1  # turbofan, fixwing
ENG_TYPE_TP = 2  # turboprop, fixwing
ENG_TYPE_TS = 3  # turboshlft, rotor

OPENAP_DIR = data("performance/openap")


class Coefficient:
    """Container for all aircraft performance coefficient databases.

    On construction, loads everything the OpenAP performance model needs,
    keyed by upper-case ICAO aircraft type code:

    Attributes:
        actypes_fixwing (list): Fixed-wing type codes available in OpenAP.
        acs_fixwing (dict): Fixed-wing aircraft properties (mass [kg], wing
            area [m^2], engine data incl. max thrust [N] and fuel flows
            [kg/s]) per type.
        limits_fixwing (dict): Fixed-wing kinematic envelopes from the OpenAP
            WRAP model: CAS limits per flight phase [m/s], vertical rate
            limits [m/s], ceiling [m], maximum Mach [-], and takeoff
            acceleration [m/s^2].
        actypes_rotor (list): Available rotorcraft type codes.
        acs_rotor (dict): Rotorcraft properties from the local JSON database.
        limits_rotor (dict): Rotorcraft envelopes (speed [m/s], vertical
            speed [m/s], ceiling [m]), with defaults for missing values.
        dragpolar_fixwing (dict): Drag polar coefficients (cd0, k) per type
            for clean, takeoff (15 deg flaps), and landing (40 deg flaps)
            configurations, plus the landing-gear drag increment.
    """

    def __init__(self) -> None:
        with warnings.catch_warnings(action="ignore"):
            self.actypes_fixwing = prop.available_aircraft(
                use_synonym=True
            )  # fixed wing types from openap
            self.acs_fixwing = self._load_all_fixwing_flavor()
            self.limits_fixwing = self._load_all_fixwing_envelop()

            self.acs_rotor = self._load_all_rotor_flavor()
            self.limits_rotor = self._load_all_rotor_envelop()
            self.actypes_rotor = list(self.acs_rotor.keys())

            self.dragpolar_fixwing = self._load_fixedwing_dragpolar()

    def _load_all_fixwing_flavor(self) -> dict:
        """Load fixed-wing aircraft and default engine data from OpenAP."""
        # load fixwing aircraft and engine from openap
        acs = {}
        # match acs_ with openap native data
        for mdl in self.actypes_fixwing:
            ac = prop.aircraft(mdl, use_synonym=True)
            acs[mdl.upper()] = ac.copy()
            engines = []
            engines.append(prop.engine(ac["engine"]["default"]))
            # options can have repeated strings as default or dicts (with model variant as key), do we handle this?
            # engines.append([prop.engine(e) for e in ac_['engine']['options']])
            acs[mdl.upper()]["engines"] = {}
            for e in engines:
                acs[mdl.upper()]["engines"][e["name"]] = e.copy()
        return acs

    def _load_all_rotor_flavor(self) -> dict[str, RotorAircraft]:
        # NOTE(abraham): this legacy rotor JSON has mixed units: mass is kg,
        # speeds are m/s and altitude is m, but engine power is kW and range is
        # km. we normalise it at this boundary
        with (OPENAP_DIR / "rotor/aircraft.json").open() as file:
            raw = json.load(file)
        raw.pop("__comment")

        aircraft: dict[str, RotorAircraft] = {}
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

    def _load_all_fixwing_envelop(self) -> dict:
        """load aircraft envelop from the openap database,
        All unit in SI

        Derives per-type kinematic limits from the OpenAP WRAP model:
        takeoff/initial-climb/en-route/approach CAS limits [m/s], climb and
        descent rate limits [m/s], maximum Mach number [-], ceiling [m],
        Mach/CAS crossover altitudes, and takeoff acceleration [m/s^2].
        En-route limits are the extremes over all flight phases.
        """
        _MAX = "maximum"
        _MIN = "minimum"
        _OPT = "default"
        limits_fixwing = {}
        for mdl in self.actypes_fixwing:
            wrap = WRAP(ac=mdl)
            mdl = mdl.upper()
            limits_fixwing[mdl] = {}
            limits_fixwing[mdl]["vminto"] = wrap.takeoff_speed()[_MIN]
            limits_fixwing[mdl]["vmaxto"] = wrap.takeoff_speed()[_MAX]
            limits_fixwing[mdl]["vminic"] = wrap.initclimb_vcas()[_MIN]
            limits_fixwing[mdl]["vmaxic"] = wrap.initclimb_vcas()[_MAX]
            limits_fixwing[mdl]["vminer"] = min(
                wrap.initclimb_vcas()[_MIN],
                wrap.climb_const_vcas()[_MIN],
                wrap.cruise_mean_vcas()[_MIN],
                wrap.descent_const_vcas()[_MIN],
                wrap.finalapp_vcas()[_MIN],
            )
            limits_fixwing[mdl]["vmaxer"] = max(
                wrap.initclimb_vcas()[_MAX],
                wrap.climb_const_vcas()[_MAX],
                wrap.cruise_mean_vcas()[_MAX],
                wrap.descent_const_vcas()[_MAX],
                wrap.finalapp_vcas()[_MAX],
            )
            limits_fixwing[mdl]["vminap"] = wrap.finalapp_vcas()[_MIN]
            limits_fixwing[mdl]["vmaxap"] = wrap.finalapp_vcas()[_MAX]
            limits_fixwing[mdl]["vminld"] = wrap.landing_speed()[_MIN]
            limits_fixwing[mdl]["vmaxld"] = wrap.landing_speed()[_MAX]

            limits_fixwing[mdl]["vmo"] = limits_fixwing[mdl]["vmaxer"]
            limits_fixwing[mdl]["mmo"] = wrap.cruise_max_mach()[_OPT]

            limits_fixwing[mdl]["hmax"] = q.km_to_m(wrap.cruise_max_alt()[_OPT])
            limits_fixwing[mdl]["crosscl"] = wrap.climb_cross_alt_conmach()[_OPT]
            limits_fixwing[mdl]["crossde"] = wrap.descent_cross_alt_concas()[_OPT]

            limits_fixwing[mdl]["axmax"] = wrap.takeoff_acceleration()[_MAX]

            limits_fixwing[mdl]["vsmax"] = max(
                wrap.initclimb_vs()[_MAX],
                wrap.climb_vs_pre_concas()[_MAX],
                wrap.climb_vs_concas()[_MAX],
                wrap.climb_vs_conmach()[_MAX],
            )

            limits_fixwing[mdl]["vsmin"] = min(
                wrap.initclimb_vs()[_MIN],
                wrap.descent_vs_post_concas()[_MIN],
                wrap.descent_vs_concas()[_MIN],
                wrap.descent_vs_conmach()[_MIN],
            )

        return limits_fixwing

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

    def _load_fixedwing_dragpolar(self) -> dict:
        """Derive clean, takeoff, and landing drag polars from OpenAP.

        OpenAP computes non-clean drag from flap deflection; since MiniSky
        has no flap-angle concept, fixed deflections of 15 deg (takeoff) and
        40 deg (landing) are assumed. The flap drag increment and the change
        in Oswald factor (dependent on engine mount position and wing aspect
        ratio) are applied to the clean cd0 and k coefficients.
        """
        dragpolar = {}
        # openap relies on flap angles to caculate nonclean drag, BS doesn't have a flap angle concept
        # we assume 15 degrees flap during takeoff and 40 degrees during landing
        flap_to = 15  # degs
        flap_ld = 40  # degs

        for mdl in self.actypes_fixwing:
            mdl = mdl.upper()
            _polar = drag.Drag(mdl, use_synonym=True).polar
            dragpolar[mdl] = {}
            dragpolar[mdl]["cd0_clean"] = _polar["clean"]["cd0"]
            dragpolar[mdl]["k_clean"] = _polar["clean"]["k"]
            dragpolar[mdl]["e_clean"] = _polar["clean"]["e"]

            lambda_f = _polar["flaps"]["lambda_f"]
            cfc = _polar["flaps"]["cf/c"]
            SfS = _polar["flaps"]["Sf/S"]
            delta_cd_flap_to = lambda_f * (cfc) ** 1.38 * SfS * np.sin(np.deg2rad(flap_to)) ** 2
            delta_cd_flap_ld = lambda_f * (cfc) ** 1.38 * SfS * np.sin(np.deg2rad(flap_ld)) ** 2
            dragpolar[mdl]["cd0_to"] = round(float(_polar["clean"]["cd0"] + delta_cd_flap_to), 3)
            dragpolar[mdl]["cd0_ld"] = round(float(_polar["clean"]["cd0"] + delta_cd_flap_ld), 3)

            if self.acs_fixwing[mdl]["engine"]["mount"] == "rear":
                delta_e_flap_to = 0.0046 * flap_to
                delta_e_flap_ld = 0.0046 * flap_ld
            else:
                delta_e_flap_to = 0.0026 * flap_to
                delta_e_flap_ld = 0.0026 * flap_ld

            ar = self.acs_fixwing[mdl]["wing"]["span"] ** 2 / self.acs_fixwing[mdl]["wing"]["area"]
            dragpolar[mdl]["k_to"] = round(
                1 / (1 / _polar["clean"]["k"] + np.pi * ar * delta_e_flap_to), 3
            )
            dragpolar[mdl]["k_ld"] = round(
                1 / (1 / _polar["clean"]["k"] + np.pi * ar * delta_e_flap_ld), 3
            )
            dragpolar[mdl]["delta_cd_gear"] = _polar["gears"]

        return dragpolar
