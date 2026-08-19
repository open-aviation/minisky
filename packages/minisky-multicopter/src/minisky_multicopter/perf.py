r"""Electric performance for multicopters.

Adds what the core [`OpenAP`][minisky.OpenAP]
model lacks for rotor aircraft: required thrust from mass and acceleration,
electrical power from a momentum-theory scaling anchored to the installed
power (`engnum * engpower`), and a battery state of charge integrated each
step that feeds back into the flight envelope. Fixed-wing rows keep the
base behaviour; the plugin keeps `SELECTIMPL OPENAP MULTICOPTERPERF`
selected.

Per-typecode electric and airframe data comes from the validated performance
table on the `Multicopter` entity (see `minisky_multicopter.config`).

The power curve is momentum-theory shape, not measured propeller data, so
absolute forward-flight power is approximate and there is no voltage or
current modelling. Hover figures and the qualitative trends are sound —
the right level for a traffic simulator.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import minisky._internal.performance.coefficients as coeff
import numpy as np
from minisky import Ok, OpenAP, Result, aero, replacement
from minisky import quantities as q
from minisky.types import AircraftIndex, AircraftTypeCode

from minisky_multicopter import quantities as mq
from minisky_multicopter.config import MulticopterTypeSpec, RotorAirframeSpec
from minisky_multicopter.entity import get_multicopter

if TYPE_CHECKING:
    from minisky import Traffic


@replacement
class MulticopterPerf(OpenAP):
    """OpenAP performance with an electric model for multicopter rows."""

    def __init__(self, traffic: Traffic) -> None:
        # NOTE(abraham): miniSky currently has one globally selected performance impl,
        # in the future we should have independent performance backends operating on
        # aircraft subsets with openap moved out of core
        super().__init__(traffic)
        self._install_types()
        with self.settrafarrays():
            self.soc = np.array([])
            """Battery state of charge as a fraction of usable capacity."""
            self.capacity: q.EnergyJ[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Usable pack energy; zero disables the battery model."""
            self.power: q.PowerW[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            self.twr: mq.ThrustToWeightRatio[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
            """Maximum-thrust-to-weight ratio."""
            self.cds: mq.FlatPlateDragAreaM2[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]

    def _typespecs(self) -> dict[AircraftTypeCode, MulticopterTypeSpec]:
        """Return the performance table of the mounted Multicopter entity.

        Empty when the implementation was selected without the plugin
        loaded, so this class degrades to base behaviour instead of
        crashing.
        """
        mc = get_multicopter(self.traffic)
        return mc.typespecs if mc is not None else {}

    def _install_types(self) -> None:
        # NOTE(abraham): right now we have to translate its own typed
        # airframe model into OpenAP's coefficient database and mutate that
        # database in place.
        # TODO replace it with our own performance backend and dont impersonate openap
        for actype, spec in self._typespecs().items():
            airframe = spec.airframe
            envelope = coeff.RotorEnvelope(
                v_min=airframe.v_min,
                v_max=airframe.v_max,
                vs_min=airframe.vs_min,
                vs_max=airframe.vs_max,
                h_max=airframe.h_max,
            )
            aircraft = coeff.RotorAircraft(
                name=actype,
                n_engines=airframe.n_engines,
                mtow=airframe.mtow,
                oew=airframe.oew,
                engines=(coeff.RotorEngine(f"{actype}-motor", airframe.engine_power),),
                envelope=envelope,
            )
            self.coeff.acs_rotor[actype] = aircraft

    def create(self, n: int = 1) -> None:
        """Seed the electric state of n newly created aircraft.

        Multicopters start on a full battery with their typecode's pack
        energy, drag area and thrust-to-weight ratio; other rows keep zeros
        (no battery model). Seeded per row rather than per batch, so a swap
        onto a mixed fleet stays correct. Membership comes from the
        performance table because the Multicopter entity may sit after this
        object in the traffic tree.
        """
        super().create(n)
        mc = get_multicopter(self.traffic)
        if mc is None:
            return
        for offset, typecode in enumerate(self.traffic.typecode[-n:], start=-n):
            actype = typecode.upper()
            spec = mc.typespecs.get(actype)
            if spec is None:
                continue
            self.twr[offset] = spec.twr
            self.cds[offset] = spec.cds
            if (energy := spec.battery_energy) is None:
                energy = self._range_derived_wh(
                    spec.airframe, spec.cds, spec.twr, mc.config.cruise_speed_fraction
                )
            self.capacity[offset] = q.wh_to_j(energy)
            self.soc[offset] = 1.0

    @staticmethod
    def _range_derived_wh(
        airframe: RotorAirframeSpec,
        cds: mq.FlatPlateDragAreaM2,
        twr: mq.ThrustToWeightRatio,
        cruise_speed_fraction: mq.CruiseSpeedFraction,
    ) -> q.EnergyWh[float]:
        """Derive usable pack energy from the typed airframe range envelope."""
        mass: q.MassKg[float] = 0.5 * (airframe.oew + airframe.mtow)
        p_max = airframe.n_engines * airframe.engine_power
        v_cruise = cruise_speed_fraction * airframe.v_max
        drag = 0.5 * aero.rho0 * v_cruise**2 * cds
        thrust = float(np.hypot(mass * aero.g0, drag))
        power = p_max * min(thrust / (twr * mass * aero.g0), 1.0) ** 1.5
        return q.j_to_wh(power * (airframe.range_max / v_cruise))

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
        # NOTE(abraham): OpenAP updates the shared performance arrays for the
        # entire fleet first, after which this subclass overwrites multicopter
        # rows!
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

    def batt(self, idx: AircraftIndex) -> Result[str, str]:
        """Report battery state of charge, power draw and endurance.

        Backs the `BATT` stack command declared on the Multicopter entity,
        which delegates here at call time so the command survives the
        performance instance being swapped on reset.
        """
        callsign = self.traffic.callsign[idx]
        soc = self.soc[idx]
        power = self.power[idx]
        if soc <= 0.0:
            endurance = "battery empty"
        elif power > 0.0:
            endurance = f"endurance {q.s_to_min(soc * self.capacity[idx] / power):.0f} min"
        else:
            endurance = "endurance --"
        return Ok(f"BATT {callsign}: {soc:.0%}, drawing {power:.0f} W, {endurance}")
