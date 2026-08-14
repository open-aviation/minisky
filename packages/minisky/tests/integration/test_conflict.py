"""Integration tests for conflict detection and resolution (ASAS)."""

from __future__ import annotations

import pytest
from minisky import MiniSky
from minisky import quantities as q
from minisky.simulation import Simulation
from minisky.traffic.asas import MVP
from tests._types import RunCommand, StepUntil


@pytest.fixture
def converging(runtime: MiniSky, run_cmd: RunCommand) -> None:
    """Two converging aircraft at the same flight level (from 2ac_converging.scn)."""
    run_cmd("ASAS ON")
    run_cmd("CRE FLIGHT1,B744,0.6655,0.0,180,FL200,290KT[CAS]")
    run_cmd("CRE FLIGHT2,B744,0.4706,0.4706,225,FL200,290KT[CAS]")
    assert runtime.traffic.ntraf == 2


class TestConflictDetection:
    def test_converging_pair_detected(
        self, runtime: MiniSky, step_until: StepUntil, converging: None
    ) -> None:
        step_until(lambda: len(runtime.traffic.cd.confpairs) > 0, max_steps=400)
        callsigns = {ac for pair in runtime.traffic.cd.confpairs for ac in pair}
        assert callsigns == {"FLIGHT1", "FLIGHT2"}

    def test_conflict_pairs_symmetric(
        self, runtime: MiniSky, step_until: StepUntil, converging: None
    ) -> None:
        step_until(lambda: len(runtime.traffic.cd.confpairs) > 0, max_steps=400)
        pairs = set(runtime.traffic.cd.confpairs)
        for a, b in pairs:
            assert (b, a) in pairs

    def test_tcpa_positive_before_cpa(
        self, runtime: MiniSky, step_until: StepUntil, converging: None
    ) -> None:
        step_until(lambda: len(runtime.traffic.cd.confpairs) > 0, max_steps=400)
        assert all(t > 0 for t in runtime.traffic.cd.tcpa)

    def test_lookahead_metrics_present(
        self, runtime: MiniSky, step_until: StepUntil, converging: None
    ) -> None:
        step_until(lambda: len(runtime.traffic.cd.confpairs) > 0, max_steps=400)
        n = len(runtime.traffic.cd.confpairs)
        assert len(runtime.traffic.cd.tcpa) == n
        assert len(runtime.traffic.cd.dcpa) == n


class TestResolutionCommands:
    def test_reso_off_via_stack(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("RESO MVP")
        assert runtime.traffic.cr.activate
        output = run_cmd("RESO OFF")
        assert not runtime.traffic.cr.activate
        assert "turned off" in output

    def test_reso_status_reports_current_method(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        run_cmd("RESO MVP")
        output = run_cmd("RESO")
        assert "Current CR method: MVP" in output

    def test_reso_status_reports_off(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("RESO OFF")
        output = run_cmd("RESO")
        assert "Current CR method: OFF" in output

    def test_rmethh_returns_ok_result(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("RESO MVP")
        result = runtime.traffic.cr.configure_horizontal_method("SPD")
        assert result.is_ok()
        assert result.unwrap() == "Horizontal resolution method set to SPD"

    def test_rmethv_returns_ok_result(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("RESO MVP")
        result = runtime.traffic.cr.configure_vertical_method("ON")
        assert result.is_ok()
        assert result.unwrap() == "Vertical resolution method set to ON"

    def test_rmethh_via_stack(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("RESO MVP")
        output = run_cmd("RMETHH SPD")
        assert "Horizontal resolution method set to SPD" in output
        assert isinstance(runtime.traffic.cr, MVP)
        assert runtime.traffic.cr.swresospd
        assert not runtime.traffic.cr.swresohdg

    def test_rmethv_via_stack(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("RESO MVP")
        output = run_cmd("RMETHV ON")
        assert "Vertical resolution method set to ON" in output
        assert isinstance(runtime.traffic.cr, MVP)
        assert runtime.traffic.cr.swresovert

    def test_rmethh_requires_mvp(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        output = run_cmd("RMETHH SPD")
        assert "not available" in output

    def test_resooff_report_mentions_resooff(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        message = run_cmd("RESOOFF")
        assert "RESOOFF" in message
        assert "NORESO" not in message


class TestDetectionCommands:
    def test_zoner_status_query(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350KT[CAS]")
        output = run_cmd("ZONER")
        assert "Current default PZ radius" in output

    def test_zonedh_status_query(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350KT[CAS]")
        output = run_cmd("ZONEDH")
        assert "Current default PZ height" in output

    def test_sethpz_status_uses_default(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL204,B744,52,4,45,FL250,350KT[CAS]")
        output = run_cmd("ZONEDH")
        assert f"{q.m_to_ft(runtime.traffic.cd.hpz_def):.2f} ft" in output

    def test_hpz_default_consistent_after_reset(self, runtime: MiniSky, sim: Simulation) -> None:
        # reset() must restore the same default as __init__
        assert runtime.traffic.cd.hpz_def == pytest.approx(q.ft_to_m(runtime.config.asas_pzh))

    def test_zoner_with_callsign_sets_aircraft_rpz(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        # The ZONER/ZONEDH specs had an unparseable "callsign..." token,
        # so per-aircraft zone sizes could not be set from the stack
        run_cmd("CRE KL204,B744,52,4,45,FL250,350KT[CAS]")
        out = run_cmd("ZONER 6.0,KL204")
        assert "Error" not in out
        assert runtime.traffic.cd.rpz[0] == pytest.approx(q.nmi_to_m(6.0))

    def test_resooff_with_callsign_sets_flag(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        # The RESOOFF/NORESO specs had an unparseable "callsign..." token,
        # so the per-aircraft variants of these commands never worked
        run_cmd("CRE KL204,B744,52,4,45,FL250,350KT[CAS]")
        out = run_cmd("RESOOFF KL204")
        assert "Error" not in out
        assert runtime.traffic.cr.resooffac[0]
        out = run_cmd("NORESO KL204")
        assert "Error" not in out
        assert runtime.traffic.cr.noresoac[0]


class TestNoConflict:
    def test_single_aircraft_no_conflicts(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("ASAS ON")
        run_cmd("CRE SOLO,A320,52,4,90,FL100,250KT[CAS]")
        for _ in range(50):
            runtime.simulation.step()
        assert len(runtime.traffic.cd.confpairs) == 0

    def test_vertically_separated_aircraft_no_conflict(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        run_cmd("ASAS ON")
        # Same converging geometry but 10000 ft apart vertically
        run_cmd("CRE HIGH1,B744,0.6655,0.0,180,FL300,290KT[CAS]")
        run_cmd("CRE LOW1,B744,0.4706,0.4706,225,FL200,290KT[CAS]")
        for _ in range(100):
            runtime.simulation.step()
        assert len(runtime.traffic.cd.confpairs) == 0

    def test_reset_clears_conflicts(
        self, runtime: MiniSky, step_until: StepUntil, converging: None
    ) -> None:
        step_until(lambda: len(runtime.traffic.cd.confpairs) > 0, max_steps=400)
        runtime.simulation.reset()
        assert len(runtime.traffic.cd.confpairs) == 0
