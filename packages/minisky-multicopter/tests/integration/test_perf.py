"""Tests for the multicopter electric performance model (Phase 3).

Power-model checks against hand-computed points, battery state-of-charge
integration and envelope feedback, plus the BATT stack command — on the
plugin-loaded `mcruntime` from conftest.py.

Hand-computed anchor (MAVIC, hover): mass = (0.494 + 0.734) / 2 kg,
installed power P_max = 4 x 66.9 W, T_max = 2 * m * g, so hover power is
P_max * 0.5 ** 1.5 ~= 94.6 W from a 43.6 Wh pack ~= 27.7 min endurance.
"""

from __future__ import annotations

import itertools

import numpy as np
import pytest
from minisky import MiniSky
from minisky import quantities as q
from minisky.tools.aero import g0
from minisky_multicopter.perf import (
    LOWBATT_SPD_FACTOR,
    LOWBATT_VS_FACTOR,
    SOC_LOW,
    MulticopterPerf,
)
from tests._types import RunCommand, StepUntil

#: Installed power of the MAVIC entry in the OpenAP rotor database [W].
MAVIC_PMAX = 4 * 66.9

#: Effective mass of the MAVIC entry, mean of OEW and MTOW [kg].
MAVIC_MASS = 0.5 * (0.494 + 0.734)


def hovering_mavic(mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil) -> MulticopterPerf:
    """Create a MAVIC, bring it to a stationary hover, return the perf."""
    traf = mcruntime.traffic
    run_mc("CRE D1,MAVIC,52,4,90,100,20")
    run_mc("SPD D1 0")
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
        run_mc("CRE D1,MAVIC,52,4,90,100,30")
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

        endurance_min = q.s_to_min(perf.soc[0] * perf.capacity[0] / perf.power[0])
        assert 20.0 < endurance_min < 35.0

    def test_fixed_wing_rows_have_no_electric_model(
        self, mcruntime: MiniSky, run_mc: RunCommand
    ) -> None:
        run_mc("CRE KL001,A320,52,4,90,FL100,250", steps=5)
        perf = mcruntime.traffic.perf
        assert isinstance(perf, MulticopterPerf)
        assert perf.capacity[0] == 0.0
        assert perf.power[0] == 0.0
        assert perf.fuelflow[0] > 0.0

    def test_unlisted_types_get_range_derived_capacity(
        self, mcruntime: MiniSky, run_mc: RunCommand
    ) -> None:
        # AMZN has no public pack spec: energy derives from d_range_max
        run_mc("CRE D1,AMZN,52,4,90,100,20")
        perf = mcruntime.traffic.perf
        assert isinstance(perf, MulticopterPerf)
        assert perf.capacity[0] > 0.0
        assert perf.soc[0] == 1.0


class TestEnvelopeFeedback:
    def test_envelope_tightens_below_soc_threshold(
        self, mcruntime: MiniSky, run_mc: RunCommand
    ) -> None:
        # two steps: the first only processes CRE, the second runs perf.update
        run_mc("CRE D1,MAVIC,52,4,90,100,20", steps=2)
        perf = mcruntime.traffic.perf
        assert isinstance(perf, MulticopterPerf)
        vmax, vsmax = perf.vmax[0], perf.vsmax[0]
        intent = (np.array([vmax]), np.array([vsmax]), np.array([100.0]), np.array([0.0]))

        perf.soc[0] = SOC_LOW + 0.1
        healthy = perf.limits(*intent)
        assert healthy.tas[0] == pytest.approx(vmax)
        assert healthy.vertical_speed[0] == pytest.approx(vsmax)

        perf.soc[0] = SOC_LOW - 0.1
        low = perf.limits(*intent)
        assert low.tas[0] == pytest.approx(LOWBATT_SPD_FACTOR * vmax)
        assert low.vertical_speed[0] == pytest.approx(LOWBATT_VS_FACTOR * vsmax)

    def test_low_battery_descent_stays_unrestricted(
        self, mcruntime: MiniSky, run_mc: RunCommand
    ) -> None:
        run_mc("CRE D1,MAVIC,52,4,90,100,20", steps=2)
        perf = mcruntime.traffic.perf
        assert isinstance(perf, MulticopterPerf)

        perf.soc[0] = 0.0
        vsmin = perf.vsmin[0]
        low = perf.limits(np.array([1.0]), np.array([vsmin]), np.array([100.0]), np.array([0.0]))
        assert low.vertical_speed[0] == pytest.approx(vsmin)


class TestBattCommand:
    def test_batt_reports_soc_power_and_endurance(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        hovering_mavic(mcruntime, run_mc, step_mc)
        report = run_mc("BATT D1")
        assert "BATT D1" in report
        assert "%" in report
        assert "W" in report
        assert "min" in report

    def test_batt_rejects_non_multicopter(self, mcruntime: MiniSky, run_mc: RunCommand) -> None:
        run_mc("CRE KL001,A320,52,4,90,FL100,250")
        assert "not a multicopter" in run_mc("BATT KL001")

    def test_batt_reports_no_model_for_custom_multicopter(
        self, mcruntime: MiniSky, run_mc: RunCommand
    ) -> None:
        # MCOPT ON gives an A320 multicopter kinematics, but there is no
        # rotor performance entry to build an electric model from.
        run_mc("CRE KL001,A320,52,4,90,FL100,250")
        run_mc("MCOPT KL001 ON")
        assert "no battery model" in run_mc("BATT KL001")
