"""Integration tests for scenario loading (IC) and timed command execution."""

import pytest

FT = 0.3048


class TestIcLoading:
    def test_ic_kl204_creates_aircraft(self, runtime, run_cmd):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        assert runtime.traffic.ntraf == 1
        assert runtime.traffic.callsign[0] == "KL204"

    def test_ic_sets_scenario_name(self, runtime, run_cmd):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        assert runtime.commands.get_scenname() == "kl204"

    def test_ic_missing_file_reports_error(self, runtime, run_cmd):
        output = run_cmd("IC scenarios/doesnotexist.scn")
        assert "not found" in output.lower()
        assert runtime.traffic.ntraf == 0

    def test_ic_resets_previous_state(self, runtime, run_cmd):
        run_cmd("CRE OLD1,A320,50,3,90,FL100,250")
        assert runtime.traffic.ntraf == 1
        run_cmd("IC scenarios/kl204.scn", steps=2)
        assert "OLD1" not in runtime.traffic.callsign
        assert runtime.traffic.callsign[0] == "KL204"


class TestTimedCommands:
    def test_timed_commands_fire_at_simtime(self, runtime, run_cmd, step_until):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        # The t=2s commands (ALT FL260, HDG 340) have been processed once
        # simt reaches 3; at t=3s ADDWPT re-enables LNAV, overriding HDG,
        # so assert exactly at simt == 3
        step_until(lambda: runtime.simulation.simt >= 3.0, max_steps=20)
        assert runtime.traffic.selalt[0] == pytest.approx(26000 * FT, rel=1e-3)
        # scenario wind makes the commanded track deviate a few degrees from 340
        assert runtime.traffic.ap.trk[0] == pytest.approx(340.0, abs=5.0)

    def test_future_commands_not_executed_early(self, runtime, run_cmd):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        # Before t=2s the FL260 command must not have fired yet
        assert runtime.simulation.simt < 2.0
        assert runtime.traffic.selalt[0] == pytest.approx(25000 * FT, rel=1e-3)

    def test_scenario_waypoint_added(self, runtime, run_cmd, step_until):
        run_cmd("IC scenarios/kl204.scn", steps=2)
        # At t=1s the scenario adds waypoint RIVER
        step_until(lambda: runtime.simulation.simt > 2.0, max_steps=20)
        route = runtime.traffic.ap.route[0]
        assert any("RIVER" in name for name in route.wpname)


class TestConvergingScenario:
    def test_2ac_scenario_produces_conflict(self, runtime, run_cmd, step_until):
        run_cmd("IC scenarios/2ac_converging.scn", steps=2)
        assert runtime.traffic.ntraf == 2
        step_until(lambda: len(runtime.traffic.cd.confpairs) > 0, max_steps=400)
