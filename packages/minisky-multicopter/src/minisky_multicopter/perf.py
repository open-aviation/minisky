"""Electric performance for multicopters (Phase 3 skeleton).

Fills the ``# TODO: implement thrust computation for rotor aircraft`` gap in
the core :class:`OpenAP` model for multicopter rows: required thrust from
the mass and acceleration, electrical power from a momentum-theory scaling
anchored to the installed power already shipped in the OpenAP rotor
coefficients (``engnum * engpower``), and a battery state of charge that is
integrated each step and feeds back into the flight envelope.

Fixed-wing rows keep the ``super()`` behaviour untouched. Selected with
``SELECTIMPL OPENAP MULTICOPTERPERF`` once registered (it joins the plugin's
replacements when Phase 3 lands).

The only data the shipped rotor ``aircraft.json`` lacks is battery capacity,
supplied by a small per-typecode spec-sheet constants dict here. No PyThrust
anywhere: a measured-prop-data upgrade is future work (see the plan doc).

Fidelity caveat: the power curve is momentum-theory shape
(``P = P_max * (T / T_max) ** 1.5``), not measured prop data, so absolute
forward-flight power is approximate and there is no terminal-voltage or
current modelling. Hover figures and the qualitative trends (power against
thrust, endurance, envelope shrink at low battery) are sound — the right
level for a traffic simulator.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from minisky.traffic.performance.perfoap import OpenAP

if TYPE_CHECKING:
    from minisky.traffic import Traffic


class MulticopterPerf(OpenAP):
    """OpenAP performance with an electric model for multicopter rows.

    Attributes:
        soc (ndarray): Battery state of charge [0-1].
        capacity (ndarray): Usable pack energy [J].
        power (ndarray): Current electrical power draw [W] — the electric
            analogue of ``fuelflow``.
        nrotors (ndarray): Number of rotors [-].
        cds (ndarray): Flat-plate parasite drag area [m2].
    """

    def __init__(self, traffic: Traffic) -> None:
        super().__init__(traffic)
        with self.settrafarrays():
            self.soc = np.array([])
            self.capacity = np.array([])
            self.power = np.array([])
            self.nrotors = np.array([])
            self.cds = np.array([])

    def create(self, n: int = 1) -> None:
        """Seed the electric state of n newly created aircraft.

        Multicopters start on a full battery with the pack, rotor count and
        drag area of their typecode; other aircraft get zeros.

        Args:
            n: Number of aircraft that were appended to the traffic arrays.
        """
        super().create(n)
        # TODO: look up the per-typecode constants dict (battery Wh, CdS,
        # thrust-to-weight ratio), seed soc = 1.0, capacity, nrotors and cds.

    def update(self, dt: float = 1) -> None:
        """Update performance, then the electric model for multicopter rows.

        After the base update, computes the thrust each multicopter needs to
        hold its current acceleration and overcome parasite drag, derives the
        electrical power from the momentum-theory scaling, and integrates the
        battery state of charge.

        Args:
            dt: Update timestep [s].
        """
        super().update(dt)
        # TODO: required thrust -> P = Pmax * (T / Tmax) ** 1.5 ->
        # self.thrust/self.power
        # TODO: integrate self.soc (ideal energy tank: soc -= P * dt / capacity)

    def limits(
        self,
        intent_v_tas: np.ndarray,
        intent_vs: np.ndarray,
        intent_h: np.ndarray,
        ax: np.ndarray,
    ) -> OpenAP.PerformanceLimits:
        """Clip the intended state to the flight envelope.

        Runs the base envelope, then tightens the speed and climb-rate limits
        of multicopter rows below a state-of-charge threshold, so performance
        degrades as the battery empties.

        Args:
            intent_v_tas: Intended true airspeed [m/s].
            intent_vs: Intended vertical speed [m/s].
            intent_h: Intended altitude [m].
            ax: Current longitudinal acceleration [m/s2].

        Returns:
            Allowed TAS [m/s], vertical speed [m/s] and altitude [m].
        """
        limits = super().limits(intent_v_tas, intent_vs, intent_h, ax)
        # TODO: shrink vmax/vsmax for multicopter rows at low state of charge
        return limits

    def required_thrust(self) -> np.ndarray:
        """Return the thrust each multicopter needs right now [N].

        Hover and climb need ``m * sqrt(g^2 + a^2)`` spread over the rotors;
        translating additionally costs a flat-plate parasite term
        ``0.5 * rho * v^2 * CdS``.
        """
        # TODO
        return np.zeros(len(self.mass))

    def batt(self, idx: int) -> tuple[bool, str]:
        """Report battery state of charge, power draw and endurance estimate.

        Arguments:
        - idx: Aircraft callsign
        """
        # TODO: report soc/power and a remaining-endurance estimate at the
        # current draw
        return False, "BATT: not implemented yet"
