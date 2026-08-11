r"""Electric performance for multicopters.

Adds what the core [`OpenAP`][minisky.traffic.performance.perfoap.OpenAP]
model lacks for rotor aircraft: required thrust from mass and acceleration,
electrical power from a momentum-theory scaling anchored to the installed
power (`engnum * engpower`), and a battery state of charge integrated each
step that feeds back into the flight envelope. Fixed-wing rows keep the
base behaviour; the plugin keeps `SELECTIMPL OPENAP MULTICOPTERPERF`
selected.

Per-typecode electric data comes from the validated performance table on
the `Multicopter` entity (see `minisky_multicopter.config`); table entries
with a full airframe block also get a rotor-database entry installed on
this instance, so user-defined types need no edits to the shipped
`aircraft.json`.

The power curve is momentum-theory shape, not measured propeller data, so
absolute forward-flight power is approximate and there is no voltage or
current modelling. Hover figures and the qualitative trends are sound —
the right level for a traffic simulator.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from minisky import plugin as plugin_api
from minisky import quantities as q
from minisky.result import Err, Ok, Result
from minisky.tools import aero
from minisky.traffic.performance import coeff
from minisky.traffic.performance.perfoap import OpenAP

from minisky_multicopter.config import MulticopterTypeSpec
from minisky_multicopter.entity import get_multicopter

if TYPE_CHECKING:
    from minisky.traffic import Traffic


@plugin_api.replacement
class MulticopterPerf(OpenAP):
    """OpenAP performance with an electric model for multicopter rows.

    Attributes:
        soc (ndarray): Battery state of charge [0-1].
        capacity (ndarray): Usable pack energy [J]; 0 = no battery model.
        power (ndarray): Current electrical power draw [W] — the electric
            analogue of `fuelflow`.
        twr (ndarray): Thrust-to-weight ratio at maximum thrust [-].
        cds (ndarray): Flat-plate parasite drag area [m2].
    """

    capacity: q.EnergyJ[np.ndarray]
    power: q.PowerW[np.ndarray]
    cds: q.AreaM2[np.ndarray]

    def __init__(self, traffic: Traffic) -> None:
        super().__init__(traffic)
        self._install_custom_types()
        with self.settrafarrays():
            self.soc = np.array([])
            self.capacity = np.array([])
            self.power = np.array([])
            self.twr = np.array([])
            self.cds = np.array([])

    def _typespecs(self) -> dict[str, MulticopterTypeSpec]:
        """Return the performance table of the mounted Multicopter entity.

        Empty when the implementation was selected without the plugin
        loaded, so this class degrades to base behaviour instead of
        crashing.
        """
        mc = get_multicopter(self.traffic)
        return mc.typespecs if mc is not None else {}

    def _install_custom_types(self) -> None:
        """Install rotor-database entries for table types with airframe data.

        The coefficient database is per-instance, so user-defined types from
        the performance TOML become full rotor entries (properties and
        envelope) without touching the shipped `aircraft.json`; an entry for
        a shipped typecode overrides it.
        """
        for actype, spec in self._typespecs().items():
            if not spec.has_airframe():
                continue
            envelop = {
                "v_min": spec.v_min,
                "v_max": spec.v_max,
                "vs_min": spec.vs_min,
                "vs_max": spec.vs_max,
                "h_max": spec.h_max,
            }
            if spec.d_range_max is not None:
                envelop["d_range_max"] = spec.d_range_max
            self.coeff.acs_rotor[actype] = {
                "name": actype,
                "n_engines": spec.n_engines,
                "engine_type": "TS",
                "mtow": spec.mtow,
                "oew": spec.oew,
                "mfc": 0,
                "engines": [[f"{actype}-motor", spec.engine_kw]],
                "envelop": envelop,
                "lifttype": coeff.LiftType.ROTORCRAFT,
            }
            self.coeff.limits_rotor[actype] = {
                "vmin": spec.v_min,
                "vmax": spec.v_max,
                "vsmin": spec.vs_min,
                "vsmax": spec.vs_max,
                "hmax": spec.h_max,
            }
        self.coeff.actypes_rotor = list(self.coeff.acs_rotor.keys())

    def create(self, n: int = 1) -> None:
        """Seed the electric state of n newly created aircraft.

        Multicopters start on a full battery with their typecode's pack
        energy, drag area and thrust-to-weight ratio; other rows keep zeros
        (no battery model). Seeded per row rather than per batch, so a swap
        onto a mixed fleet stays correct. Membership comes from the
        performance table because the Multicopter entity may sit after this
        object in the traffic tree.

        Args:
            n: Number of aircraft appended to the traffic arrays.
        """
        super().create(n)
        mc = get_multicopter(self.traffic)
        if mc is None:
            return
        for offset, typecode in enumerate(self.traffic.typecode[-n:], start=-n):
            actype = typecode.upper()
            spec = mc.typespecs.get(actype)
            ac = self.coeff.acs_rotor.get(actype)
            if spec is None or ac is None:
                continue
            self.twr[offset] = spec.twr
            self.cds[offset] = spec.cds
            wh = spec.battery_wh
            if wh is None:
                wh = self._range_derived_wh(ac, spec.cds, spec.twr, mc.config.cruise_speed_fraction)
            self.capacity[offset] = q.wh_to_j(wh)
            self.soc[offset] = 1.0

    @staticmethod
    def _range_derived_wh(
        ac: dict, cds: q.AreaM2[float], twr: float, cruise_speed_fraction: float
    ) -> q.EnergyWh[float]:
        """Derive the pack energy of a type without a pack spec [Wh].

        Energy to fly the rotor entry's `d_range_max` at cruise speed,
        evaluated with the same momentum-theory power model used at runtime.

        Args:
            ac: OpenAP rotor `aircraft.json` entry for the typecode.
            cds: Flat-plate parasite drag area.
            twr: Thrust-to-weight ratio at maximum thrust.
            cruise_speed_fraction: Cruise speed as a fraction of `v_max`.
        """
        envelop = ac["envelop"]
        d_range = q.km_to_m(envelop.get("d_range_max", 0.0))
        v_max = envelop.get("v_max", 0.0)
        if d_range <= 0.0 or v_max <= 0.0:
            return 0.0
        mass = 0.5 * (ac["oew"] + ac["mtow"])
        p_max = q.kw_to_w(int(ac["n_engines"]) * ac["engines"][0][1])
        v_cruise = cruise_speed_fraction * v_max
        drag = 0.5 * aero.rho0 * v_cruise**2 * cds
        thrust = float(np.hypot(mass * aero.g0, drag))
        power = p_max * min(thrust / (twr * mass * aero.g0), 1.0) ** 1.5
        return q.j_to_wh(power * (d_range / v_cruise))

    def required_thrust(self) -> q.ForceN[np.ndarray]:
        r"""Return the thrust each aircraft would need as a multicopter [N].

        The thrust vector supports the weight — including any vertical
        acceleration, $m \sqrt{g^2 + a_z^2}$ — while its horizontal
        component overcomes the flat-plate parasite drag of translation,
        $\tfrac{1}{2} \rho v^2 C_D S$. Meaningful for multicopter rows
        (other rows have a zero drag area).
        """
        traf = self.traffic
        rho = aero.vdensity(traf.alt)
        drag = 0.5 * rho * traf.tas**2 * self.cds
        lift = self.mass * np.hypot(aero.g0, traf.kinematics.az)
        return np.hypot(lift, drag)

    def update(self) -> None:
        r"""Update performance, then the electric model for multicopter rows.

        After the base update, computes each multicopter's required thrust,
        derives the electrical power from the momentum-theory scaling
        $P = P_\text{max} (T / T_\text{max})^{1.5}$, and integrates the
        battery state of charge as an ideal energy tank.
        """
        super().update()
        mc = get_multicopter(self.traffic)
        if mc is None:
            return
        m = mc.ismulticopter & (self.capacity > 0.0)
        if not m.any():
            return

        thrust = self.required_thrust()[m]
        t_max = self.twr[m] * self.mass[m] * aero.g0
        p_max = self.engnum[m] * self.engpower[m]
        power = p_max * np.clip(thrust / t_max, 0.0, 1.0) ** 1.5

        self.thrust[m] = thrust
        self.power[m] = power
        simdt = self.traffic._get_simulation().simdt
        self.soc[m] = np.clip(self.soc[m] - power * simdt / self.capacity[m], 0.0, 1.0)

    def limits(
        self,
        intent_v_tas: q.TrueAirspeedMps[np.ndarray],
        intent_vs: q.VerticalRateMps[np.ndarray],
        intent_h: q.PressureAltitudeM[np.ndarray],
        ax: q.AccelerationMps2[np.ndarray],
    ) -> OpenAP.PerformanceLimits:
        """Clip the intended state to the flight envelope.

        Runs the base envelope, then tightens the maximum speed and climb
        rate of multicopter rows below the state-of-charge threshold (the
        `soc_low` / `lowbatt_*_factor` plugin settings). Descent stays
        unrestricted — a low battery should not keep an aircraft airborne.
        """
        allowed = super().limits(intent_v_tas, intent_vs, intent_h, ax)
        mc = get_multicopter(self.traffic)
        if mc is None:
            return allowed
        low = mc.ismulticopter & (self.capacity > 0.0) & (self.soc < mc.config.soc_low)
        if not low.any():
            return allowed

        tas, vs, alt = allowed
        tas[low] = np.minimum(tas[low], mc.config.lowbatt_spd_factor * self.vmax[low])
        vs[low] = np.minimum(vs[low], mc.config.lowbatt_vs_factor * self.vsmax[low])
        return self.PerformanceLimits(tas, vs, alt)

    def batt(self, idx: int) -> Result[str, str]:
        """Report battery state of charge, power draw and endurance.

        Backs the `BATT` stack command declared on the Multicopter entity,
        which delegates here at call time so the command survives the
        performance instance being swapped on reset.

        Args:
            idx: Aircraft index.
        """
        callsign = self.traffic.callsign[idx]
        if self.capacity[idx] <= 0.0:
            return Err(f"BATT: no battery model for {callsign} ({self.actype[idx]})")

        soc = self.soc[idx]
        power = self.power[idx]
        if soc <= 0.0:
            endurance = "battery empty"
        elif power > 0.0:
            endurance = f"endurance {q.s_to_min(soc * self.capacity[idx] / power):.0f} min"
        else:
            endurance = "endurance --"
        return Ok(f"BATT {callsign}: {soc:.0%}, drawing {power:.0f} W, {endurance}")
