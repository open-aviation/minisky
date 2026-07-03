"""Integration tests for scenario loading (IC) and timed command execution."""

import pytest

import minisky

FT = 0.3048


class TestIcLoading:
    def test_ic_kl204_creates_aircraft(self, bs, run_cmd):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        assert bs.traf.ntraf == 1
        assert bs.traf.callsign[0] == "KL204"

    def test_ic_sets_scenario_name(self, bs, run_cmd):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        assert minisky.stack.get_scenname() == "kl204"

    def test_ic_missing_file_reports_error(self, bs, run_cmd):
        output = run_cmd("IC scenarios/doesnotexist.scn")
        assert "not found" in output.lower()
        assert bs.traf.ntraf == 0

    def test_ic_resets_previous_state(self, bs, run_cmd):
        run_cmd("CRE OLD1,A320,50,3,90,FL100,250")
        assert bs.traf.ntraf == 1
        run_cmd("IC scenarios/kl204.scn", steps=2)
        assert "OLD1" not in bs.traf.callsign
        assert bs.traf.callsign[0] == "KL204"


class TestTimedCommands:
    def test_timed_commands_fire_at_simtime(self, bs, run_cmd, step_until):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        # The t=2s commands (ALT FL260, HDG 340) have been processed once
        # simt reaches 3; at t=3s ADDWPT re-enables LNAV, overriding HDG,
        # so assert exactly at simt == 3
        step_until(lambda: bs.sim.simt >= 3.0, max_steps=20)
        assert bs.traf.selalt[0] == pytest.approx(26000 * FT, rel=1e-3)
        # scenario wind makes the commanded track deviate a few degrees from 340
        assert bs.traf.ap.trk[0] == pytest.approx(340.0, abs=5.0)

    def test_future_commands_not_executed_early(self, bs, run_cmd):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        # Before t=2s the FL260 command must not have fired yet
        assert bs.sim.simt < 2.0
        assert bs.traf.selalt[0] == pytest.approx(25000 * FT, rel=1e-3)

    def test_scenario_waypoint_added(self, bs, run_cmd, step_until):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        # At t=1s the scenario adds waypoint RIVER
        step_until(lambda: bs.sim.simt > 2.0, max_steps=20)
        route = bs.traf.ap.route[0]
        assert any("RIVER" in name for name in route.wpname)


class TestConvergingScenario:
    def test_2ac_scenario_produces_conflict(self, bs, run_cmd, step_until):
        run_cmd("IC scenarios/2ac_converging.scn", steps=2)
        assert bs.traf.ntraf == 2
        step_until(lambda: len(bs.traf.cd.confpairs) > 0, max_steps=400)
