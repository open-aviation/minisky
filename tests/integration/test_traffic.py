"""Integration tests for aircraft creation/deletion (minisky.traffic.Traffic)."""

import pytest

FT = 0.3048


class TestCreate:
    def test_cre_single(self, bs, sim):
        ok, msg = bs.traf.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)
        assert ok
        assert bs.traf.ntraf == 1
        assert bs.traf.callsign[0] == "KL001"
        assert bs.traf.lat[0] == pytest.approx(52.0)
        assert bs.traf.lon[0] == pytest.approx(4.0)
        assert bs.traf.hdg[0] == pytest.approx(90.0)

    def test_cre_lowercase_callsign_is_uppercased(self, bs, sim):
        bs.traf.cre("kl002")
        assert bs.traf.callsign[0] == "KL002"

    def test_cre_duplicate_callsign_rejected(self, bs, sim):
        bs.traf.cre("KL001")
        ok, msg = bs.traf.cre("KL001")
        assert not ok
        assert bs.traf.ntraf == 1

    def test_mcre_multiple(self, bs, sim):
        ok, _ = bs.traf.mcre(5)
        assert ok
        assert bs.traf.ntraf == 5
        assert len(set(bs.traf.callsign)) == 5

    def test_idx_lookup(self, bs, sim):
        bs.traf.cre("KL001")
        bs.traf.cre("KL002")
        assert bs.traf.idx("KL002") == 1
        assert bs.traf.idx("kl001") == 0
        assert bs.traf.idx("MISSING") == -1


class TestArrays:
    def test_array_sizes_consistent(self, bs, sim):
        bs.traf.mcre(3)
        n = bs.traf.ntraf
        for attr in ("lat", "lon", "alt", "hdg", "tas", "cas", "gs", "vs"):
            assert len(getattr(bs.traf, attr)) == n, attr
        assert len(bs.traf.callsign) == n

    def test_speed_arrays_initialized(self, bs, sim):
        bs.traf.cre("KL001", spd=150, alt=3000)
        assert bs.traf.tas[0] > 0
        assert bs.traf.gs[0] == pytest.approx(bs.traf.tas[0])


class TestDelete:
    def test_delete_shrinks_arrays(self, bs, sim):
        bs.traf.cre("KL001")
        bs.traf.cre("KL002")
        bs.traf.delete(0)
        assert bs.traf.ntraf == 1
        assert bs.traf.callsign[0] == "KL002"
        assert len(bs.traf.lat) == 1

    def test_delete_all(self, bs, sim):
        bs.traf.mcre(3)
        bs.traf.delete([0, 1, 2])
        assert bs.traf.ntraf == 0


class TestReset:
    def test_sim_reset_clears_traffic(self, bs, sim):
        bs.traf.mcre(4)
        assert bs.traf.ntraf == 4
        bs.sim.reset()
        assert bs.traf.ntraf == 0
        assert len(bs.traf.lat) == 0

    def test_reset_clears_simtime(self, bs, sim):
        bs.traf.cre("KL001")
        for _ in range(5):
            bs.sim.step()
        assert bs.sim.simt > 0
        bs.sim.reset()
        assert bs.sim.simt == 0


class TestStep:
    def test_step_advances_time_with_traffic(self, bs, sim):
        bs.traf.cre("KL001")
        bs.sim.step()  # INIT -> OP transition + first update
        t0 = bs.sim.simt
        bs.sim.step()
        assert bs.sim.simt == pytest.approx(t0 + bs.sim.simdt)

    def test_no_time_advance_without_traffic(self, bs, sim):
        bs.sim.step()
        assert bs.sim.simt == 0

    def test_aircraft_moves_when_stepped(self, bs, sim):
        bs.traf.cre("KL001", lat=52.0, lon=4.0, hdg=90, alt=10000 * FT, spd=250)
        for _ in range(10):
            bs.sim.step()
        # eastbound: longitude increases, latitude nearly constant
        assert bs.traf.lon[0] > 4.0
        assert bs.traf.lat[0] == pytest.approx(52.0, abs=0.05)
