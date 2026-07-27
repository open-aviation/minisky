"""Electric performance for multicopters.

Fills the ``# TODO: implement thrust computation for rotor aircraft`` gap in
the core :class:`OpenAP` model for multicopter rows: required thrust from
the mass and acceleration, power and current from a precomputed
``(airspeed, thrust) -> (power, current, feasible)`` map per typecode, and a
battery state of charge that is integrated each step and feeds back into the
flight envelope as the pack voltage sags.

Fixed-wing rows keep the ``super()`` behaviour untouched. Selected with
``SELECTIMPL OPENAP MULTICOPTERPERF``.

The maps are generated offline by ``scripts/gen_multicopter_perf.py`` from
propeller, motor and battery data vendored under ``data/pythrust/``, and
checked in under ``data/``. PyThrust itself is *not* a runtime dependency:
only its data is used, and only through ``np.interp``-style lookups.

Fidelity caveat: the APC propeller coefficients are axial-flow, so
forward-flight power for a translating multicopter is approximate. Hover
figures and the qualitative trends (power against speed, voltage sag) are
sound — the right level for a traffic simulator.
"""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

from minisky.traffic.performance.perfoap import OpenAP

if TYPE_CHECKING:
    from minisky.traffic import Traffic

#: Directory holding the generated per-typecode performance maps.
DATA_PATH = Path(__file__).parent / "data"


class MulticopterPerf(OpenAP):
    """OpenAP performance with an electric model for multicopter rows.

    Attributes:
        soc (ndarray): Battery state of charge [0-1].
        capacity (ndarray): Usable pack capacity [As].
        current (ndarray): Current battery current draw [A].
        voltage (ndarray): Current battery terminal voltage [V].
        power (ndarray): Current electrical power draw [W] — the electric
            analogue of ``fuelflow``.
        nrotors (ndarray): Number of rotors [-].
        cds (ndarray): Flat-plate parasite drag area [m2].
    """

    def __init__(self, traffic: Traffic) -> None:
        super().__init__(traffic)
        # TODO: load the generated maps and battery curves from DATA_PATH
        with self.settrafarrays():
            self.soc = np.array([])
            self.capacity = np.array([])
            self.current = np.array([])
            self.voltage = np.array([])
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
        # TODO: look up the per-typecode config, seed soc = 1.0, capacity,
        # nrotors and cds.

    def update(self, dt: float = 1) -> None:
        """Update performance, then the electric model for multicopter rows.

        After the base update, computes the thrust each multicopter needs to
        hold its current acceleration and overcome parasite drag, reads power
        and current off the per-typecode map, and integrates the battery
        state of charge.

        Args:
            dt: Update timestep [s].
        """
        super().update(dt)
        # TODO: required thrust -> map lookup -> self.thrust/power/current
        # TODO: integrate self.soc; update self.voltage from the OCV/R curves

    def limits(
        self,
        intent_v_tas: np.ndarray,
        intent_vs: np.ndarray,
        intent_h: np.ndarray,
        ax: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Clip the intended state to the flight envelope.

        Runs the base envelope, then tightens the speed and climb-rate limits
        of multicopter rows wherever the performance map is infeasible at the
        current pack voltage, so performance genuinely degrades as the
        battery empties.

        Args:
            intent_v_tas: Intended true airspeed [m/s].
            intent_vs: Intended vertical speed [m/s].
            intent_h: Intended altitude [m].
            ax: Current longitudinal acceleration [m/s2].

        Returns:
            Allowed TAS [m/s], vertical speed [m/s] and altitude [m].
        """
        allow_v_tas, allow_vs, allow_h = super().limits(intent_v_tas, intent_vs, intent_h, ax)
        # TODO: shrink vmax/vsmax for multicopter rows at low state of charge
        return allow_v_tas, allow_vs, allow_h

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
        # TODO: report soc/voltage/current/power and a remaining-endurance
        # estimate at the current draw
        return False, "BATT: not implemented yet"
