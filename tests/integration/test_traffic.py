"""Integration tests for aircraft creation/deletion (minisky.traffic.Traffic)."""

import numpy as np
import pytest

FT = 0.3048
KTS = 0.514444


class TestCreate:
    def test_cre_single(self, runtime, sim):
        ok, msg = runtime.traffic.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=3000, spd=150)
        assert ok
        assert runtime.traffic.ntraf == 1
        assert runtime.traffic.callsign[0] == "KL001"
        assert runtime.traffic.lat[0] == pytest.approx(52.0)
        assert runtime.traffic.lon[0] == pytest.approx(4.0)
        assert runtime.traffic.hdg[0] == pytest.approx(90.0)

    def test_cre_lowercase_callsign_is_uppercased(self, runtime, sim):
        runtime.traffic.cre("kl002")
        assert runtime.traffic.callsign[0] == "KL002"

    def test_cre_duplicate_callsign_rejected(self, runtime, sim):
        runtime.traffic.cre("KL001")
        ok, msg = runtime.traffic.cre("KL001")
        assert not ok
        assert runtime.traffic.ntraf == 1

    def test_mcre_multiple(self, runtime, sim):
        ok, _ = runtime.traffic.mcre(5)
        assert ok
        assert runtime.traffic.ntraf == 5
        assert len(set(runtime.traffic.callsign)) == 5

    def test_idx_lookup(self, runtime, sim):
        runtime.traffic.cre("KL001")
        runtime.traffic.cre("KL002")
        assert runtime.traffic.idx("KL002") == 1
        assert runtime.traffic.idx("kl001") == 0
        assert runtime.traffic.idx("MISSING") == -1

    def test_cre_defaults_are_25000ft_300kts(self, runtime, sim):
        # Defaults used to be 25000 m / 300 m/s; they are meant as ft/kts.
        runtime.traffic.cre("KL001")
        assert runtime.traffic.alt[0] == pytest.approx(25000 * FT)
        assert runtime.traffic.cas[0] == pytest.approx(300 * KTS)

    def test_cre_via_stack_without_alt_spd_uses_defaults(self, runtime, run_cmd):
        run_cmd("CRE KL204,B744,52,4")
        assert runtime.traffic.ntraf == 1
        assert runtime.traffic.alt[0] == pytest.approx(25000 * FT, rel=1e-3)
        assert runtime.traffic.cas[0] == pytest.approx(300 * KTS, rel=1e-3)

    def test_cre_echoes_confirmation(self, runtime, run_cmd):
        # Command results must reach the output buffer (scr.echo), not stdout only
        out = run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert out == "Aircraft KL204 created"


class TestArrays:
    def test_array_sizes_consistent(self, runtime, sim):
        runtime.traffic.mcre(3)
        n = runtime.traffic.ntraf
        for attr in ("lat", "lon", "alt", "hdg", "tas", "cas", "gs", "vs"):
            assert len(getattr(runtime.traffic, attr)) == n, attr
        assert len(runtime.traffic.callsign) == n

    def test_speed_arrays_initialized(self, runtime, sim):
        runtime.traffic.cre("KL001", spd=150, alt=3000)
        assert runtime.traffic.tas[0] > 0
        assert runtime.traffic.gs[0] == pytest.approx(runtime.traffic.tas[0])


class TestDelete:
    def test_delete_shrinks_arrays(self, runtime, sim):
        runtime.traffic.cre("KL001")
        runtime.traffic.cre("KL002")
        runtime.traffic.delete(0)
        assert runtime.traffic.ntraf == 1
        assert runtime.traffic.callsign[0] == "KL002"
        assert len(runtime.traffic.lat) == 1

    def test_delete_all(self, runtime, sim):
        runtime.traffic.mcre(3)
        runtime.traffic.delete([0, 1, 2])
        assert runtime.traffic.ntraf == 0


class TestReset:
    def test_sim_reset_clears_traffic(self, runtime, sim):
        runtime.traffic.mcre(4)
        assert runtime.traffic.ntraf == 4
        runtime.simulation.reset()
        assert runtime.traffic.ntraf == 0
        assert len(runtime.traffic.lat) == 0

    def test_reset_clears_simtime(self, runtime, sim):
        runtime.traffic.cre("KL001")
        for _ in range(5):
            runtime.simulation.step()
        assert runtime.simulation.simt > 0
        runtime.simulation.reset()
        assert runtime.simulation.simt == 0


class TestStep:
    def test_step_advances_time_with_traffic(self, runtime, sim):
        runtime.traffic.cre("KL001")
        runtime.simulation.step()  # INIT -> OP transition + first update
        t0 = runtime.simulation.simt
        runtime.simulation.step()
        assert runtime.simulation.simt == pytest.approx(t0 + runtime.simulation.simdt)

    def test_no_time_advance_without_traffic(self, runtime, sim):
        runtime.simulation.step()
        assert runtime.simulation.simt == 0

    def test_aircraft_moves_when_stepped(self, runtime, sim):
        runtime.traffic.cre("KL001", lat=52.0, lon=4.0, hdg=90, alt=10000 * FT, spd=250)
        for _ in range(10):
            runtime.simulation.step()
        # eastbound: longitude increases, latitude nearly constant
        assert runtime.traffic.lon[0] > 4.0
        assert runtime.traffic.lat[0] == pytest.approx(52.0, abs=0.05)


class TestCreCmd:
    def test_clrcrecmd_with_pending_commands(self, runtime, run_cmd):
        run_cmd("CRECMD SPD 250")
        assert runtime.traffic.crecmdlist == ["SPD 250"]
        out = run_cmd("CLRCRECMD")
        assert runtime.traffic.crecmdlist == []
        assert "All 1 crecmd commands deleted" in out

    def test_clrcrecmd_with_empty_list(self, runtime, run_cmd):
        out = run_cmd("CLRCRECMD")
        assert runtime.traffic.crecmdlist == []
        assert "CLRCRECMD" in out


class TestConditional:
    def test_atspd_seeds_condition_with_cas(self, runtime, sim):
        runtime.traffic.cre("KL001", alt=25000 * FT, spd=150)
        cas, tas = runtime.traffic.cas[0], runtime.traffic.tas[0]
        assert tas > cas  # TAS exceeds CAS at altitude
        # Target between current CAS and TAS: not crossed in CAS terms
        target = 0.5 * (cas + tas)
        runtime.traffic.cond.atspdcmd(0, target, "KL001 LNAV ON")
        # Seed must be based on CAS, like the comparison in update()
        assert runtime.traffic.cond.lastdif[-1] == pytest.approx(target - cas)
        # The speed did not cross the target, so nothing may trigger
        ncond = runtime.traffic.cond.ncond
        runtime.traffic.cond.update()
        assert runtime.traffic.cond.ncond == ncond

    def test_renameac_updates_pending_conditions(self, runtime, sim):
        runtime.traffic.cre("KL001", alt=10000 * FT, spd=150)
        runtime.traffic.cond.ataltcmd(0, 5000 * FT, "KL001 SPD 200")
        runtime.traffic.cond.renameac("KL001", "KL999")
        assert "KL999" in runtime.traffic.cond.id
        assert "KL001" not in runtime.traffic.cond.id
        # Unknown callsign takes the early-return path without errors
        runtime.traffic.cond.renameac("MISSING", "XX123")
        assert "XX123" not in runtime.traffic.cond.id


class TestWind:
    def test_wind_add_get_roundtrip(self, runtime, sim):
        wind = runtime.traffic.wind
        assert wind.add(52.0, 4.0, 270.0, 20.0) is True  # from 270 deg, 20 kts
        vn, ve = wind.getdata(52.0, 4.0, 0.0)
        assert ve == pytest.approx(20 * KTS)  # westerly wind blows eastward
        assert vn == pytest.approx(0.0, abs=1e-9)

    def test_windfield_remove_keeps_lat_lon_paired(self, runtime, sim):
        wind = runtime.traffic.wind
        wind.addpoint(52.0, 4.0, 270.0, 20.0)
        idx = wind.addpoint(54.0, 6.0, 180.0, 10.0)
        wind.remove(idx)
        assert list(wind.lat) == [52.0]
        assert list(wind.lon) == [4.0]  # used to become a copy of lat
        assert wind.winddim == 1

    def test_wind_del_clears_field(self, runtime, sim):
        wind = runtime.traffic.wind
        wind.add(52.0, 4.0, 270.0, 20.0)
        assert wind.winddim > 0
        assert wind.add(52.0, 4.0, "DEL") is True
        assert wind.winddim == 0
        assert len(wind.lat) == 0

    def test_wind_del_not_shadowed_by_altitude_form(self, runtime, sim):
        wind = runtime.traffic.wind
        wind.add(52.0, 4.0, 270.0, 20.0)
        # With 3+ winddata elements DEL used to fall into the alt/dir/spd branch
        assert wind.add(52.0, 4.0, "DEL", None, None) is True
        assert wind.winddim == 0

    def test_wind_via_stack_two_element_form(self, runtime, run_cmd):
        # The WIND spec ran the direction through the altitude parser
        # (ft -> m), silently mangling WIND lat,lon,dir,spd
        out = run_cmd("WIND 52,4,270,20")
        assert "Error" not in out
        vn, ve = runtime.traffic.wind.getdata(52.0, 4.0, 0.0)
        assert ve == pytest.approx(20 * KTS, rel=1e-6)
        assert vn == pytest.approx(0.0, abs=1e-9)

    def test_wind_del_via_stack(self, runtime, run_cmd):
        # WIND lat,lon,DEL used to be rejected by the altitude parser
        run_cmd("WIND 52,4,270,20")
        assert runtime.traffic.wind.winddim > 0
        out = run_cmd("WIND 52,4,DEL")
        assert "Error" not in out
        assert runtime.traffic.wind.winddim == 0


class TestNoise:
    def test_surveillance_noise_differs_per_aircraft(self, runtime, sim):
        runtime.traffic.mcre(3)
        runtime.traffic.setnoise(True)
        runtime.traffic.noise.lastupdate[:] = -1.0  # make every aircraft due for update
        runtime.traffic.noise.update()
        offsets = runtime.traffic.noise.lat - runtime.traffic.lat
        # One noise sample used to be broadcast to all due aircraft
        assert np.unique(offsets).size == runtime.traffic.ntraf

    def test_turbulence_registered_in_traffic_tree(self, runtime, sim):
        assert runtime.traffic.turbulence in runtime.traffic._children

    def test_noise_on_via_stack_steps_without_crash(self, runtime, run_cmd):
        run_cmd("CRE KL001,A320,52,4,90,FL250,300")
        run_cmd("NOISE ON")
        assert runtime.traffic.turbulence.active
        for _ in range(5):
            runtime.simulation.step()
        assert runtime.traffic.ntraf == 1


class TestTrails:
    def test_fresh_trails_object_has_background_buffers(self, runtime, sim):
        from minisky.traffic.trails import Trails

        trails = Trails(runtime.traffic, lambda: runtime.simulation)
        try:
            assert trails.bgacid == []  # used to exist only after clearbg()
            assert not hasattr(trails, "pygame")
        finally:
            runtime.traffic._children.remove(trails)

    def test_trail_on_update_and_buffer(self, runtime, run_cmd):
        run_cmd("CRE KL001,A320,52,4,90,FL250,300")
        run_cmd("TRAIL ON 1")
        assert runtime.traffic.trails.active
        for _ in range(5):
            runtime.simulation.step()
        assert len(runtime.traffic.trails.newlat0) > 0  # segments were recorded
        runtime.traffic.trails.buffer()  # must not crash on bgacid
        assert "KL001" in runtime.traffic.trails.bgacid
        run_cmd("TRAIL OFF")  # clears all trail data
