from __future__ import annotations

import itertools

import numpy as np
import pytest
from minisky import MiniSky
from minisky import quantities as q
from minisky.aero import g0
from minisky_multicopter.entity import get_multicopter
from minisky_multicopter.perf import MulticopterPerf
from tests._types import RunCommand, StepUntil

MAVIC_PMAX: q.PowerW = 4 * 66.9
"""Installed power of the MAVIC entry in teh OpenAP rotor database"""

MAVIC_MASS: q.MassKg = 0.5 * (0.494 + 0.734)
"""Effective mass of the MAVIC entry, mean of OEW and MTOW"""


def hovering_mavic(mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil) -> MulticopterPerf:
    """Create a MAVIC, bring it to a stationary hover, return the perf."""
    traf = mcruntime.traffic
    run_mc("CRE D1,MAVIC,52,4,90,100FT[STD],20KT[CAS]")
    run_mc("SPD D1 0KT[CAS]")
    kin = traf.kinematics
    step_mc(lambda: traf.gs[0] == 0.0 and traf.vs[0] == 0.0 and kin.az[0] == 0.0, 30)
    mcruntime.simulation.step()  # one settled step so power reflects the hover
    perf = traf.perf
    assert isinstance(perf, MulticopterPerf)
    return perf


class TestPowerModel:
    def test_hover_power_matches_hand_computed_point(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        perf = hovering_mavic(mcruntime, run_mc, step_mc)

        # At hover: T = m * g (no drag, no vertical acceleration), and the
        # momentum-theory scaling gives P = P_max * (1 / TWR) ** 1.5.
        assert perf.thrust[0] == pytest.approx(MAVIC_MASS * g0, rel=1e-6)
        assert perf.power[0] == pytest.approx(MAVIC_PMAX * 0.5**1.5, rel=1e-6)

    def test_translation_needs_more_thrust_than_hover(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        traf = mcruntime.traffic
        run_mc("CRE D1,MAVIC,52,4,90,100FT[STD],30KT[CAS]")
        step_mc(lambda: traf.gs[0] > 10.0, 20)
        perf = traf.perf
        assert isinstance(perf, MulticopterPerf)

        hover_thrust = MAVIC_MASS * g0
        assert perf.thrust[0] > hover_thrust
        assert perf.power[0] > MAVIC_PMAX * 0.5**1.5

    def test_soc_decreases_monotonically(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        perf = hovering_mavic(mcruntime, run_mc, step_mc)

        history = [float(perf.soc[0])]
        for _ in range(20):
            mcruntime.simulation.step()
            history.append(float(perf.soc[0]))
        assert all(a > b for a, b in itertools.pairwise(history))

    def test_mavic_hover_endurance_within_sanity_bounds(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        perf = hovering_mavic(mcruntime, run_mc, step_mc)

        endurance_min = perf.soc[0] * perf.capacity[0] / perf.power[0] / 60.0
        assert 20.0 < endurance_min < 35.0

    def test_fixed_wing_rows_have_no_electric_model(
        self, mcruntime: MiniSky, run_mc: RunCommand
    ) -> None:
        run_mc("CRE KL001,A320,52,4,90,FL100,250KT[CAS]", steps=5)
        perf = mcruntime.traffic.perf
        assert isinstance(perf, MulticopterPerf)
        assert perf.capacity[0] == 0.0
        assert perf.power[0] == 0.0
        assert perf.fuelflow[0] > 0.0

    def test_unlisted_types_get_range_derived_capacity(
        self, mcruntime: MiniSky, run_mc: RunCommand
    ) -> None:
        # AMZN has no public pack spec: energy derives from d_range_max
        run_mc("CRE D1,AMZN,52,4,90,100FT[STD],20KT[CAS]")
        perf = mcruntime.traffic.perf
        assert isinstance(perf, MulticopterPerf)
        assert perf.capacity[0] > 0.0
        assert perf.soc[0] == 1.0


class TestEnvelopeFeedback:
    def test_envelope_tightens_below_soc_threshold(
        self, mcruntime: MiniSky, run_mc: RunCommand
    ) -> None:
        run_mc("CRE D1,MAVIC,52,4,90,100FT[STD],20KT[CAS]", steps=2)
        perf = mcruntime.traffic.perf
        mc = get_multicopter(mcruntime.traffic)
        assert isinstance(perf, MulticopterPerf)
        assert mc is not None

        vmax, vsmax = perf.vmax[0], perf.vsmax[0]
        intent = (np.array([vmax]), np.array([vsmax]), np.array([100.0]), np.array([0.0]))

        perf.soc[0] = mc.config.soc_low + 0.1
        healthy = perf.limits(*intent)
        assert healthy.tas[0] == pytest.approx(vmax)
        assert healthy.vertical_speed[0] == pytest.approx(vsmax)

        perf.soc[0] = mc.config.soc_low - 0.1
        low = perf.limits(*intent)
        assert low.tas[0] == pytest.approx(mc.config.lowbatt_spd_factor * vmax)
        assert low.vertical_speed[0] == pytest.approx(mc.config.lowbatt_vs_factor * vsmax)
