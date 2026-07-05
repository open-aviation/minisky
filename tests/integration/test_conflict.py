"""Integration tests for conflict detection and resolution (ASAS)."""

import pytest

import minisky

FT = 0.3048


@pytest.fixture
def converging(bs, run_cmd):
    """Two converging aircraft at the same flight level (from 2ac_converging.scn)."""
    run_cmd("ASAS ON")
    run_cmd("CRE FLIGHT1,B744,0.6655,0.0,180,FL200,290")
    run_cmd("CRE FLIGHT2,B744,0.4706,0.4706,225,FL200,290")
    assert bs.traf.ntraf == 2


class TestConflictDetection:
    def test_converging_pair_detected(self, bs, step_until, converging):
        step_until(lambda: len(bs.traf.cd.confpairs) > 0, max_steps=400)
        callsigns = {ac for pair in bs.traf.cd.confpairs for ac in pair}
        assert callsigns == {"FLIGHT1", "FLIGHT2"}

    def test_conflict_pairs_symmetric(self, bs, step_until, converging):
        step_until(lambda: len(bs.traf.cd.confpairs) > 0, max_steps=400)
        pairs = set(bs.traf.cd.confpairs)
        for a, b in pairs:
            assert (b, a) in pairs

    def test_tcpa_positive_before_cpa(self, bs, step_until, converging):
        step_until(lambda: len(bs.traf.cd.confpairs) > 0, max_steps=400)
        assert all(t > 0 for t in bs.traf.cd.tcpa)

    def test_lookahead_metrics_present(self, bs, step_until, converging):
        step_until(lambda: len(bs.traf.cd.confpairs) > 0, max_steps=400)
        n = len(bs.traf.cd.confpairs)
        assert len(bs.traf.cd.tcpa) == n
        assert len(bs.traf.cd.dcpa) == n


class TestResolutionCommands:
    def test_reso_off_via_stack(self, bs, run_cmd):
        run_cmd("RESO MVP")
        assert bs.traf.cr.activate
        output = run_cmd("RESO OFF")
        assert not bs.traf.cr.activate
        assert "turned off" in output

    def test_reso_status_reports_current_method(self, bs, run_cmd):
        run_cmd("RESO MVP")
        output = run_cmd("RESO")
        assert "Current CR method: MVP" in output

    def test_reso_status_reports_off(self, bs, run_cmd):
        run_cmd("RESO OFF")
        output = run_cmd("RESO")
        assert "Current CR method: OFF" in output

    def test_rmethh_returns_success_tuple(self, bs, run_cmd):
        run_cmd("RESO MVP")
        result = bs.traf.cr.setresometh("SPD")
        assert result == (True, "Horizontal resolution method set to SPD")

    def test_rmethv_returns_success_tuple(self, bs, run_cmd):
        run_cmd("RESO MVP")
        result = bs.traf.cr.setresometv("ON")
        assert result == (True, "Vertical resolution method set to ON")

    def test_rmethh_via_stack(self, bs, run_cmd):
        run_cmd("RESO MVP")
        output = run_cmd("RMETHH SPD")
        assert "Horizontal resolution method set to SPD" in output
        assert bs.traf.cr.swresospd
        assert not bs.traf.cr.swresohdg

    def test_rmethv_via_stack(self, bs, run_cmd):
        run_cmd("RESO MVP")
        output = run_cmd("RMETHV ON")
        assert "Vertical resolution method set to ON" in output
        assert bs.traf.cr.swresovert

    def test_rmethh_requires_mvp(self, bs, run_cmd):
        output = run_cmd("RMETHH SPD")
        assert "not available" in output

    def test_resooff_report_mentions_resooff(self, bs, sim):
        success, message = bs.traf.cr.setresooff()
        assert success
        assert "RESOOFF" in message
        assert "NORESO" not in message


class TestDetectionCommands:
    def test_zoner_status_query(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        output = run_cmd("ZONER")
        assert "Current default PZ radius" in output

    def test_zonedh_status_query(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        output = run_cmd("ZONEDH")
        assert "Current default PZ height" in output

    def test_sethpz_status_uses_default(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        success, message = bs.traf.cd.sethpz()
        assert success
        assert f"{bs.traf.cd.hpz_def / FT:.2f} ft" in message

    def test_hpz_default_consistent_after_reset(self, bs, sim):
        # reset() must restore the same default as __init__
        assert bs.traf.cd.hpz_def == pytest.approx(minisky.core.settings.asas_pzh * FT)

    def test_zoner_with_callsign_sets_aircraft_rpz(self, bs, run_cmd):
        # The ZONER/ZONEDH specs had an unparseable "callsign..." token,
        # so per-aircraft zone sizes could not be set from the stack
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        out = run_cmd("ZONER 6.0,KL204")
        assert "Error" not in out
        assert bs.traf.cd.rpz[0] == pytest.approx(6.0 * 1852.0)

    def test_resooff_with_callsign_sets_flag(self, bs, run_cmd):
        # The RESOOFF/NORESO specs had an unparseable "callsign..." token,
        # so the per-aircraft variants of these commands never worked
        run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        out = run_cmd("RESOOFF KL204")
        assert "Error" not in out
        assert bs.traf.cr.resooffac[0]
        out = run_cmd("NORESO KL204")
        assert "Error" not in out
        assert bs.traf.cr.noresoac[0]


class TestNoConflict:
    def test_single_aircraft_no_conflicts(self, bs, run_cmd):
        run_cmd("ASAS ON")
        run_cmd("CRE SOLO,A320,52,4,90,FL100,250")
        for _ in range(50):
            bs.sim.step()
        assert len(bs.traf.cd.confpairs) == 0

    def test_vertically_separated_aircraft_no_conflict(self, bs, run_cmd):
        run_cmd("ASAS ON")
        # Same converging geometry but 10000 ft apart vertically
        run_cmd("CRE HIGH1,B744,0.6655,0.0,180,FL300,290")
        run_cmd("CRE LOW1,B744,0.4706,0.4706,225,FL200,290")
        for _ in range(100):
            bs.sim.step()
        assert len(bs.traf.cd.confpairs) == 0

    def test_reset_clears_conflicts(self, bs, step_until, converging):
        step_until(lambda: len(bs.traf.cd.confpairs) > 0, max_steps=400)
        bs.sim.reset()
        assert len(bs.traf.cd.confpairs) == 0
