"""Integration tests for aircraft creation/deletion (minisky.traffic.Traffic)."""

import numpy as np
import pytest

FT = 0.3048
KTS = 0.514444


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

    def test_cre_defaults_are_25000ft_300kts(self, bs, sim):
        # Defaults used to be 25000 m / 300 m/s; they are meant as ft/kts.
        bs.traf.cre("KL001")
        assert bs.traf.alt[0] == pytest.approx(25000 * FT)
        assert bs.traf.cas[0] == pytest.approx(300 * KTS)

    def test_cre_via_stack_without_alt_spd_uses_defaults(self, bs, run_cmd):
        run_cmd("CRE KL204,B744,52,4")
        assert bs.traf.ntraf == 1
        assert bs.traf.alt[0] == pytest.approx(25000 * FT, rel=1e-3)
        assert bs.traf.cas[0] == pytest.approx(300 * KTS, rel=1e-3)

    def test_cre_echoes_confirmation(self, bs, run_cmd):
        # Command results must reach the output buffer (scr.echo), not stdout only
        out = run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert out == "Aircraft KL204 created"


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


class TestCreCmd:
    def test_clrcrecmd_with_pending_commands(self, bs, run_cmd):
        run_cmd("CRECMD SPD 250")
        assert bs.traf.crecmdlist == ["SPD 250"]
        out = run_cmd("CLRCRECMD")
        assert bs.traf.crecmdlist == []
        assert "All 1 crecmd commands deleted" in out

    def test_clrcrecmd_with_empty_list(self, bs, run_cmd):
        out = run_cmd("CLRCRECMD")
        assert bs.traf.crecmdlist == []
        assert "CLRCRECMD" in out


class TestConditional:
    def test_atspd_seeds_condition_with_cas(self, bs, sim):
        bs.traf.cre("KL001", alt=25000 * FT, spd=150)
        cas, tas = bs.traf.cas[0], bs.traf.tas[0]
        assert tas > cas  # TAS exceeds CAS at altitude
        # Target between current CAS and TAS: not crossed in CAS terms
        target = 0.5 * (cas + tas)
        bs.traf.cond.atspdcmd(0, target, "KL001 LNAV ON")
        # Seed must be based on CAS, like the comparison in update()
        assert bs.traf.cond.lastdif[-1] == pytest.approx(target - cas)
        # The speed did not cross the target, so nothing may trigger
        ncond = bs.traf.cond.ncond
        bs.traf.cond.update()
        assert bs.traf.cond.ncond == ncond
        bs.traf.cond.__init__()  # drop pending conditions (not cleared by reset)

    def test_renameac_updates_pending_conditions(self, bs, sim):
        bs.traf.cre("KL001", alt=10000 * FT, spd=150)
        bs.traf.cond.ataltcmd(0, 5000 * FT, "KL001 SPD 200")
        bs.traf.cond.renameac("KL001", "KL999")
        assert "KL999" in bs.traf.cond.id
        assert "KL001" not in bs.traf.cond.id
        # Unknown callsign takes the early-return path without errors
        bs.traf.cond.renameac("MISSING", "XX123")
        assert "XX123" not in bs.traf.cond.id
        bs.traf.cond.__init__()  # drop pending conditions (not cleared by reset)


class TestWind:
    def test_wind_add_get_roundtrip(self, bs, sim):
        wind = bs.traf.wind
        assert wind.add(52.0, 4.0, 270.0, 20.0) is True  # from 270 deg, 20 kts
        vn, ve = wind.getdata(52.0, 4.0, 0.0)
        assert ve == pytest.approx(20 * KTS)  # westerly wind blows eastward
        assert vn == pytest.approx(0.0, abs=1e-9)

    def test_windfield_remove_keeps_lat_lon_paired(self, bs, sim):
        wind = bs.traf.wind
        wind.addpoint(52.0, 4.0, 270.0, 20.0)
        idx = wind.addpoint(54.0, 6.0, 180.0, 10.0)
        wind.remove(idx)
        assert list(wind.lat) == [52.0]
        assert list(wind.lon) == [4.0]  # used to become a copy of lat
        assert wind.winddim == 1

    def test_wind_del_clears_field(self, bs, sim):
        wind = bs.traf.wind
        wind.add(52.0, 4.0, 270.0, 20.0)
        assert wind.winddim > 0
        assert wind.add(52.0, 4.0, "DEL") is True
        assert wind.winddim == 0
        assert len(wind.lat) == 0

    def test_wind_del_not_shadowed_by_altitude_form(self, bs, sim):
        wind = bs.traf.wind
        wind.add(52.0, 4.0, 270.0, 20.0)
        # With 3+ winddata elements DEL used to fall into the alt/dir/spd branch
        assert wind.add(52.0, 4.0, "DEL", None, None) is True
        assert wind.winddim == 0

    def test_wind_via_stack_two_element_form(self, bs, run_cmd):
        # The WIND spec ran the direction through the altitude parser
        # (ft -> m), silently mangling WIND lat,lon,dir,spd
        out = run_cmd("WIND 52,4,270,20")
        assert "Error" not in out
        vn, ve = bs.traf.wind.getdata(52.0, 4.0, 0.0)
        assert ve == pytest.approx(20 * KTS, rel=1e-6)
        assert vn == pytest.approx(0.0, abs=1e-9)

    def test_wind_del_via_stack(self, bs, run_cmd):
        # WIND lat,lon,DEL used to be rejected by the altitude parser
        run_cmd("WIND 52,4,270,20")
        assert bs.traf.wind.winddim > 0
        out = run_cmd("WIND 52,4,DEL")
        assert "Error" not in out
        assert bs.traf.wind.winddim == 0


class TestNoise:
    def test_surveillance_noise_differs_per_aircraft(self, bs, sim):
        bs.traf.mcre(3)
        bs.traf.setnoise(True)
        bs.traf.noise.lastupdate[:] = -1.0  # make every aircraft due for update
        bs.traf.noise.update()
        offsets = bs.traf.noise.lat - bs.traf.lat
        # One noise sample used to be broadcast to all due aircraft
        assert np.unique(offsets).size == bs.traf.ntraf

    def test_turbulence_registered_in_traffic_tree(self, bs, sim):
        assert bs.traf.turbulence in bs.traf._children

    def test_noise_on_via_stack_steps_without_crash(self, bs, run_cmd):
        run_cmd("CRE KL001,A320,52,4,90,FL250,300")
        run_cmd("NOISE ON")
        assert bs.traf.turbulence.active
        for _ in range(5):
            bs.sim.step()
        assert bs.traf.ntraf == 1


class TestTrails:
    def test_fresh_trails_object_has_background_buffers(self, bs, sim):
        from minisky.traffic.trails import Trails

        trails = Trails()
        try:
            assert trails.bgacid == []  # used to exist only after clearbg()
            assert not hasattr(trails, "pygame")
        finally:
            bs.traf._children.remove(trails)

    def test_trail_on_update_and_buffer(self, bs, run_cmd):
        run_cmd("CRE KL001,A320,52,4,90,FL250,300")
        run_cmd("TRAIL ON 1")
        assert bs.traf.trails.active
        for _ in range(5):
            bs.sim.step()
        assert len(bs.traf.trails.newlat0) > 0  # segments were recorded
        bs.traf.trails.buffer()  # must not crash on bgacid
        assert "KL001" in bs.traf.trails.bgacid
        run_cmd("TRAIL OFF")  # clears all trail data
