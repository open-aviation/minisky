"""Integration tests for conflict detection (ASAS)."""

import pytest


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
