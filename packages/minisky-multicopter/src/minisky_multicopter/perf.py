"""Electric performance for multicopters.

Fills the ``# TODO: implement thrust computation for rotor aircraft`` gap in
the core :class:`OpenAP` model for multicopter rows: required thrust from
the mass and acceleration, electrical power from a momentum-theory scaling
anchored to the installed power already shipped in the OpenAP rotor
coefficients (``engnum * engpower``), and a battery state of charge that is
integrated each step and feeds back into the flight envelope.

Fixed-wing rows keep the ``super()`` behaviour untouched. Selected with
``SELECTIMPL OPENAP MULTICOPTERPERF`` (the plugin's hooks keep this
selected, like the other multicopter implementations).

The only data the shipped rotor ``aircraft.json`` lacks is battery capacity
(``mfc`` is 0 for every rotor type), supplied by the small per-typecode
spec-sheet constants dict below; types without a public pack spec get an
energy derived from their ``d_range_max`` at cruise speed. No PyThrust
anywhere: a measured-prop-data upgrade is future work (see the plan doc).

Fidelity caveat: the power curve is momentum-theory shape
(``P = P_max * (T / T_max) ** 1.5``), not measured prop data, so absolute
forward-flight power is approximate and there is no terminal-voltage or
current modelling — the envelope feedback is keyed on state of charge
directly. Hover figures and the qualitative trends (power against thrust,
endurance, envelope shrink at low battery) are sound — the right level for
a traffic simulator.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from minisky import plugin as plugin_api
from minisky.result import Err, Ok, Result
from minisky.tools import aero
from minisky.traffic.performance.perfoap import OpenAP

from minisky_multicopter.entity import MULTICOPTER_TYPES, get_multicopter

if TYPE_CHECKING:
    from minisky.traffic import Traffic

#: State of charge below which the flight envelope is tightened [-].
SOC_LOW = 0.2

#: Maximum-speed factor applied to low-battery multicopters [-].
LOWBATT_SPD_FACTOR = 0.6

#: Maximum-climb-rate factor applied to low-battery multicopters [-].
LOWBATT_VS_FACTOR = 0.5

#: Default thrust-to-weight ratio, typical for camera/delivery multirotors [-].
DEFAULT_TWR = 2.0

#: Default flat-plate parasite drag area [m2].
DEFAULT_CDS = 0.01

#: Cruise speed as a fraction of the envelope maximum, for the
#: range-derived battery-energy fallback [-].
CRUISE_SPEED_FRACTION = 0.8

#: Spec-sheet constants per multicopter typecode: usable pack energy
#: ``battery_wh`` [Wh] (the one datum missing from the OpenAP rotor
#: ``aircraft.json``), and optional ``cds`` [m2] / ``twr`` [-] overrides.
#: MNET, AMZN and HORSEFLY have no public pack spec and fall back to an
#: energy derived from ``d_range_max`` at cruise speed.
CONSTANTS: dict[str, dict[str, float]] = {
    "MAVIC": {"battery_wh": 43.6},  # 3830 mAh 11.4 V
    "PHAN4": {"battery_wh": 81.3},  # 5350 mAh 15.2 V
    "M100": {"battery_wh": 99.9},  # TB47D
    "M200": {"battery_wh": 349.2},  # 2x TB55
    "M600": {"battery_wh": 599.4},  # 6x TB47S
}


@plugin_api.replacement
class MulticopterPerf(OpenAP):
    """OpenAP performance with an electric model for multicopter rows.

    Attributes:
        soc (ndarray): Battery state of charge [0-1].
        capacity (ndarray): Usable pack energy [J]; 0 = no battery model.
        power (ndarray): Current electrical power draw [W] — the electric
            analogue of ``fuelflow``.
        twr (ndarray): Thrust-to-weight ratio at maximum thrust [-].
        cds (ndarray): Flat-plate parasite drag area [m2].
    """

    def __init__(self, traffic: Traffic) -> None:
        super().__init__(traffic)
        with self.settrafarrays():
            self.soc = np.array([])
            self.capacity = np.array([])
            self.power = np.array([])
            self.twr = np.array([])
            self.cds = np.array([])

    def create(self, n: int = 1) -> None:
        """Seed the electric state of n newly created aircraft.

        Multicopters start on a full battery with the pack energy, drag area
        and thrust-to-weight ratio of their typecode; other aircraft keep
        zeros (no battery model). Seeded per row from the typecode — unlike
        the base class this does not assume one type per batch, so a swap
        onto an existing mixed fleet stays correct. Membership is checked by
        typecode because the Multicopter entity may sit after this object in
        the traffic tree, so its arrays cannot be relied upon here.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)
        for offset, typecode in enumerate(self.traffic.typecode[-n:], start=-n):
            actype = typecode.upper()
            ac = self.coeff.acs_rotor.get(actype)
            if actype not in MULTICOPTER_TYPES or ac is None:
                continue
            spec = CONSTANTS.get(actype, {})
            self.twr[offset] = spec.get("twr", DEFAULT_TWR)
            self.cds[offset] = spec.get("cds", DEFAULT_CDS)
            wh = spec.get("battery_wh")
            if wh is None:
                wh = self._range_derived_wh(ac, self.cds[offset], self.twr[offset])
            self.capacity[offset] = wh * 3600.0
            self.soc[offset] = 1.0

    @staticmethod
    def _range_derived_wh(ac: dict, cds: float, twr: float) -> float:
        """Derive the pack energy of an unlisted type from its range [Wh].

        Energy to fly the ``d_range_max`` of the OpenAP rotor entry at
        cruise speed (a fixed fraction of the envelope maximum), evaluated
        with the same momentum-theory power model used at runtime.

        Args:
            ac: OpenAP rotor ``aircraft.json`` entry for the typecode.
            cds: Flat-plate parasite drag area [m2].
            twr: Thrust-to-weight ratio at maximum thrust [-].
        """
        envelop = ac["envelop"]
        d_range = envelop.get("d_range_max", 0.0) * 1000.0
        v_max = envelop.get("v_max", 0.0)
        if d_range <= 0.0 or v_max <= 0.0:
            return 0.0
        mass = 0.5 * (ac["oew"] + ac["mtow"])
        p_max = int(ac["n_engines"]) * ac["engines"][0][1] * 1000.0  # kW -> W
        v_cruise = CRUISE_SPEED_FRACTION * v_max
        drag = 0.5 * aero.rho0 * v_cruise**2 * cds
        thrust = float(np.hypot(mass * aero.g0, drag))
        power = p_max * min(thrust / (twr * mass * aero.g0), 1.0) ** 1.5
        return power * (d_range / v_cruise) / 3600.0

    def required_thrust(self) -> np.ndarray:
        """Return the thrust each aircraft would need as a multicopter [N].

        The thrust vector supports the weight — including any vertical
        acceleration, ``m * sqrt(g^2 + az^2)`` — while its horizontal
        component overcomes the flat-plate parasite drag of translation,
        ``0.5 * rho * v^2 * CdS``. Meaningful for multicopter rows (other
        rows have a zero drag area).
        """
        traf = self.traffic
        rho = aero.vdensity(traf.alt)
        drag = 0.5 * rho * traf.tas**2 * self.cds
        lift = self.mass * np.hypot(aero.g0, traf.kinematics.az)
        return np.hypot(lift, drag)

    def update(self, dt: float = 1) -> None:
        """Update performance, then the electric model for multicopter rows.

        After the base update, computes the thrust each multicopter needs to
        support its weight and overcome parasite drag, derives the
        electrical power from the momentum-theory scaling
        ``P = P_max * (T / T_max) ** 1.5`` anchored to the installed power,
        and integrates the battery state of charge as an ideal energy tank.

        Args:
            dt: Update timestep [s] (unused; the simulation timestep is read
                from the owning runtime, like the base class does elsewhere).
        """
        super().update(dt)
        mc = get_multicopter(self.traffic)
        if mc is None:
            return
        m = mc.ismulticopter & (self.capacity > 0.0)
        if not m.any():
            return

        thrust = self.required_thrust()[m]
        t_max = self.twr[m] * self.mass[m] * aero.g0
        p_max = self.engnum[m] * self.engpower[m] * 1000.0  # engpower is in kW
        power = p_max * np.clip(thrust / t_max, 0.0, 1.0) ** 1.5

        self.thrust[m] = thrust
        self.power[m] = power
        simdt = self.traffic._get_simulation().simdt
        self.soc[m] = np.clip(self.soc[m] - power * simdt / self.capacity[m], 0.0, 1.0)

    def limits(
        self,
        intent_v_tas: np.ndarray,
        intent_vs: np.ndarray,
        intent_h: np.ndarray,
        ax: np.ndarray,
    ) -> OpenAP.PerformanceLimits:
        """Clip the intended state to the flight envelope.

        Runs the base envelope, then tightens the maximum speed and climb
        rate of multicopter rows below the state-of-charge threshold, so
        performance degrades as the battery empties. Descent stays
        unrestricted — a low battery should not keep an aircraft airborne.

        Args:
            intent_v_tas: Intended true airspeed [m/s].
            intent_vs: Intended vertical speed [m/s].
            intent_h: Intended altitude [m].
            ax: Current longitudinal acceleration [m/s2].

        Returns:
            Allowed TAS [m/s], vertical speed [m/s] and altitude [m].
        """
        allowed = super().limits(intent_v_tas, intent_vs, intent_h, ax)
        mc = get_multicopter(self.traffic)
        if mc is None:
            return allowed
        low = mc.ismulticopter & (self.capacity > 0.0) & (self.soc < SOC_LOW)
        if not low.any():
            return allowed

        tas, vs, alt = allowed
        tas[low] = np.minimum(tas[low], LOWBATT_SPD_FACTOR * self.vmax[low])
        vs[low] = np.minimum(vs[low], LOWBATT_VS_FACTOR * self.vsmax[low])
        return self.PerformanceLimits(tas, vs, alt)

    def batt(self, idx: int) -> Result[str, str]:
        """Report battery state of charge, power draw and endurance.

        Backs the ``BATT`` stack command declared on the Multicopter entity,
        which delegates here at call time so the command survives the
        performance instance being swapped on reset.

        Args:
            idx: Aircraft index.

        Returns:
            Result containing the report message or an error.
        """
        callsign = self.traffic.callsign[idx]
        if self.capacity[idx] <= 0.0:
            return Err(f"BATT: no battery model for {callsign} ({self.actype[idx]})")

        soc = self.soc[idx]
        power = self.power[idx]
        if soc <= 0.0:
            endurance = "battery empty"
        elif power > 0.0:
            endurance = f"endurance {soc * self.capacity[idx] / power / 60.0:.0f} min"
        else:
            endurance = "endurance --"
        return Ok(f"BATT {callsign}: {soc:.0%}, drawing {power:.0f} W, {endurance}")
