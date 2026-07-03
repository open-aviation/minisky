"""Integration tests for the command stack (queueing, processing, echo output)."""

import pytest

import minisky

FT = 0.3048
KTS = 0.514444


class TestQueueing:
    def test_stack_only_queues(self, bs, sim):
        minisky.stack.stack("CRE KL204,B744,52,4,45,FL250,350")
        assert bs.traf.ntraf == 0  # not executed yet

    def test_command_executes_on_step(self, bs, sim):
        minisky.stack.stack("CRE KL204,B744,52,4,45,FL250,350")
        bs.sim.step()
        assert bs.traf.ntraf == 1
        assert bs.traf.callsign[0] == "KL204"


class TestCommands:
    def test_cre_via_stack(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert bs.traf.ntraf == 1
        assert bs.traf.alt[0] == pytest.approx(25000 * FT, rel=1e-3)

    def test_pos_outputs_callsign(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        output = run_cmd("POS KL204")
        assert "KL204" in output

    def test_bare_callsign_defaults_to_pos(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        output = run_cmd("KL204")
        assert "KL204" in output

    def test_alt_sets_selected_altitude(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        run_cmd("ALT KL204 FL260")
        assert bs.traf.selalt[0] == pytest.approx(26000 * FT, rel=1e-3)

    def test_hdg_sets_autopilot_track(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        run_cmd("HDG KL204 340")
        assert bs.traf.ap.trk[0] == pytest.approx(340.0)
        assert not bs.traf.swlnav[0]

    def test_spd_sets_selected_speed(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        run_cmd("SPD KL204 300")
        assert bs.traf.selspd[0] == pytest.approx(300 * KTS, rel=1e-3)

    def test_del_removes_aircraft(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert bs.traf.ntraf == 1
        run_cmd("DEL KL204")
        assert bs.traf.ntraf == 0

    def test_mcre_via_stack(self, bs, run_cmd):
        run_cmd("MCRE 3")
        assert bs.traf.ntraf == 3


class TestErrors:
    def test_unknown_command_echoes_error(self, bs, run_cmd):
        output = run_cmd("BOGUSCMD 42")
        assert "unknown command" in output.lower()

    def test_command_on_missing_aircraft_reports_error(self, bs, run_cmd):
        output = run_cmd("ALT NOSUCH FL100")
        assert output  # some error text is echoed
        assert bs.traf.ntraf == 0

    def test_sim_survives_bad_command(self, bs, run_cmd):
        run_cmd("THISDOESNOTEXIST")
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert bs.traf.ntraf == 1
