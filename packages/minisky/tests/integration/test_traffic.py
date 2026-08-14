"""Integration tests for aircraft creation/deletion (minisky.traffic.Traffic)."""

from __future__ import annotations

import numpy as np
import pytest
from minisky import MiniSky
from minisky import quantities as q
from minisky.simulation import Simulation
from minisky.traffic.conditional import AltitudeCondition, SpeedCondition
from minisky.traffic.wind import WindFieldKind
from minisky.values import StdPressureAltM
from tests._types import RunCommand


class TestCreate:
    def test_cre_single(self, runtime: MiniSky, sim: Simulation) -> None:
        result = runtime.traffic.cre("KL001", "A320", lat=52.0, lon=4.0, hdg=90, alt=StdPressureAltM(3000), spd=150)
        assert result.is_ok()
        assert runtime.traffic.ntraf == 1
        assert runtime.traffic.callsign[0] == "KL001"
        assert runtime.traffic.lat[0] == pytest.approx(52.0)
        assert runtime.traffic.lon[0] == pytest.approx(4.0)
        assert runtime.traffic.hdg[0] == pytest.approx(90.0)

    def test_cre_lowercase_callsign_is_uppercased(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("kl002")
        assert runtime.traffic.callsign[0] == "KL002"

    def test_cre_duplicate_callsign_rejected(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001")
        result = runtime.traffic.cre("KL001")
        assert result.is_err()
        assert runtime.traffic.ntraf == 1

    def test_mcre_multiple(self, runtime: MiniSky, sim: Simulation) -> None:
        result = runtime.traffic.mcre(5)
        assert result.is_ok()
        assert runtime.traffic.ntraf == 5
        assert len(set(runtime.traffic.callsign)) == 5

    def test_idx_lookup(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001")
        runtime.traffic.cre("KL002")
        assert runtime.traffic.idx("KL002") == 1
        assert runtime.traffic.idx("kl001") == 0
        assert runtime.traffic.idx("MISSING") is None
        assert runtime.traffic.idx(["KL001", "MISSING"]) == [0, None]

    def test_cre_defaults_are_25000ft_300kts(self, runtime: MiniSky, sim: Simulation) -> None:
        # Defaults used to be 25000 m / 300 m/s; they are meant as ft/kts.
        runtime.traffic.cre("KL001")
        assert runtime.traffic.alt[0] == pytest.approx(q.ft_to_m(25000.0))
        assert runtime.traffic.cas[0] == pytest.approx(q.kt_to_mps(300.0))

    def test_cre_via_stack_without_alt_spd_uses_defaults(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        run_cmd("CRE KL204,B744,52,4")
        assert runtime.traffic.ntraf == 1
        assert runtime.traffic.alt[0] == pytest.approx(q.ft_to_m(25000.0), rel=1e-3)
        assert runtime.traffic.cas[0] == pytest.approx(q.kt_to_mps(300.0), rel=1e-3)

    def test_cre_echoes_confirmation(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        # Command results must reach the output buffer (scr.echo), not stdout only
        out = run_cmd("CRE KL204,B744,52,4,45,FL250,350")
        assert out == "Aircraft KL204 created"


class TestArrays:
    def test_array_sizes_consistent(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.mcre(3)
        n = runtime.traffic.ntraf
        for attr in ("lat", "lon", "alt", "hdg", "tas", "cas", "gs", "vs"):
            assert len(getattr(runtime.traffic, attr)) == n, attr
        assert len(runtime.traffic.callsign) == n

    def test_speed_arrays_initialized(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001", spd=150, alt=StdPressureAltM(3000))
        assert runtime.traffic.tas[0] > 0
        assert runtime.traffic.gs[0] == pytest.approx(runtime.traffic.tas[0])


class TestDelete:
    def test_delete_shrinks_arrays(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001")
        runtime.traffic.cre("KL002")
        runtime.traffic.delete(0)
        assert runtime.traffic.ntraf == 1
        assert runtime.traffic.callsign[0] == "KL002"
        assert len(runtime.traffic.lat) == 1

    def test_delete_all(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.mcre(3)
        runtime.traffic.delete(np.array([0, 1, 2]))
        assert runtime.traffic.ntraf == 0


class TestReset:
    def test_sim_reset_clears_traffic(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.mcre(4)
        assert runtime.traffic.ntraf == 4
        runtime.simulation.reset()
        assert runtime.traffic.ntraf == 0
        assert len(runtime.traffic.lat) == 0

    def test_reset_clears_simtime(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001")
        for _ in range(5):
            runtime.simulation.step()
        assert runtime.simulation.simt > 0
        runtime.simulation.reset()
        assert runtime.simulation.simt == 0


class TestStep:
    def test_step_advances_time_with_traffic(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001")
        runtime.simulation.step()  # INIT -> OP transition + first update
        t0 = runtime.simulation.simt
        runtime.simulation.step()
        assert runtime.simulation.simt == pytest.approx(t0 + runtime.simulation.simdt)

    def test_no_time_advance_without_traffic(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.simulation.step()
        assert runtime.simulation.simt == 0

    def test_aircraft_moves_when_stepped(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001", lat=52.0, lon=4.0, hdg=90, alt=StdPressureAltM(q.ft_to_m(10000.0)), spd=250)
        for _ in range(10):
            runtime.simulation.step()
        # eastbound: longitude increases, latitude nearly constant
        assert runtime.traffic.lon[0] > 4.0
        assert runtime.traffic.lat[0] == pytest.approx(52.0, abs=0.05)


class TestCreCmd:
    def test_clrcrecmd_with_pending_commands(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRECMD SPD 250")
        assert runtime.traffic.crecmdlist == ["SPD 250"]
        out = run_cmd("CLRCRECMD")
        assert runtime.traffic.crecmdlist == []
        assert "All 1 crecmd commands deleted" in out

    def test_clrcrecmd_with_empty_list(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        out = run_cmd("CLRCRECMD")
        assert runtime.traffic.crecmdlist == []
        assert "CLRCRECMD" in out


class TestConditional:
    def test_atspd_seeds_condition_with_cas(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001", alt=StdPressureAltM(q.ft_to_m(25000.0)), spd=150)
        cas, tas = runtime.traffic.cas[0], runtime.traffic.tas[0]
        assert tas > cas  # TAS exceeds CAS at altitude
        # Target between current CAS and TAS: not crossed in CAS terms
        target = 0.5 * (cas + tas)
        runtime.traffic.cond.atspdcmd(0, target, "KL001 LNAV ON")
        # Seed must be based on CAS, like the comparison in update().
        condition = runtime.traffic.cond.conditions[-1]
        assert isinstance(condition, SpeedCondition)
        assert condition.last_difference == pytest.approx(target - cas)
        # The speed did not cross the target, so nothing may trigger
        ncond = runtime.traffic.cond.ncond
        runtime.traffic.cond.update()
        assert runtime.traffic.cond.ncond == ncond

    def test_renameac_updates_pending_conditions(self, runtime: MiniSky, sim: Simulation) -> None:
        runtime.traffic.cre("KL001", alt=StdPressureAltM(q.ft_to_m(10000.0)), spd=150)
        runtime.traffic.cond.ataltcmd(0, StdPressureAltM(q.ft_to_m(5000.0)), "KL001 SPD 200")
        runtime.traffic.cond.renameac("KL001", "KL999")
        condition = runtime.traffic.cond.conditions[-1]
        assert isinstance(condition, AltitudeCondition)
        assert condition.callsign == "KL999"
        # Unknown callsign takes the early-return path without errors
        runtime.traffic.cond.renameac("MISSING", "XX123")
        assert all(condition.callsign != "XX123" for condition in runtime.traffic.cond.conditions)


class TestWind:
    def test_wind_add_get_roundtrip(self, runtime: MiniSky, sim: Simulation) -> None:
        wind = runtime.traffic.wind
        wind.addpoint(52.0, 4.0, 270.0, q.kt_to_mps(20.0))
        vn, ve = wind.getdata(52.0, 4.0, 0.0)
        assert ve == pytest.approx(q.kt_to_mps(20.0))  # westerly wind blows eastward
        assert vn == pytest.approx(0.0, abs=1e-9)

    def test_windfield_remove_keeps_lat_lon_paired(self, runtime: MiniSky, sim: Simulation) -> None:
        wind = runtime.traffic.wind
        wind.addpoint(52.0, 4.0, 270.0, 20.0)
        idx = wind.addpoint(54.0, 6.0, 180.0, 10.0)
        wind.remove(idx)
        assert list(wind.lat) == [52.0]
        assert list(wind.lon) == [4.0]  # used to become a copy of lat
        assert wind.kind is WindFieldKind.CONSTANT

    def test_wind_del_clears_field(self, runtime: MiniSky, sim: Simulation) -> None:
        wind = runtime.traffic.wind
        wind.addpoint(52.0, 4.0, 270.0, q.kt_to_mps(20.0))
        assert wind.has_wind
        wind.clear()
        assert wind.kind is WindFieldKind.NONE
        assert len(wind.lat) == 0

    def test_wind_via_stack_two_element_form(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        # The WIND spec ran the direction through the altitude parser
        # (ft -> m), silently mangling WIND lat,lon,dir,spd
        out = run_cmd("WIND 52,4,270,20")
        assert "Error" not in out
        vn, ve = runtime.traffic.wind.getdata(52.0, 4.0, 0.0)
        assert ve == pytest.approx(q.kt_to_mps(20.0), rel=1e-6)
        assert vn == pytest.approx(0.0, abs=1e-9)

    def test_wind_del_via_stack(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        # WIND lat,lon,DEL used to be rejected by the altitude parser
        run_cmd("WIND 52,4,270,20")
        assert runtime.traffic.wind.has_wind
        out = run_cmd("WIND 52,4,DEL")
        assert "Error" not in out
        assert runtime.traffic.wind.kind is WindFieldKind.NONE


class TestNoise:
    def test_surveillance_noise_differs_per_aircraft(
        self, runtime: MiniSky, sim: Simulation
    ) -> None:
        runtime.traffic.mcre(3)
        runtime.traffic.configure_noise(True)
        runtime.traffic.noise.lastupdate[:] = -1.0  # make every aircraft due for update
        runtime.traffic.noise.update()
        offsets = runtime.traffic.noise.lat - runtime.traffic.lat
        # One noise sample used to be broadcast to all due aircraft
        assert np.unique(offsets).size == runtime.traffic.ntraf

    def test_turbulence_registered_in_traffic_tree(self, runtime: MiniSky, sim: Simulation) -> None:
        assert runtime.traffic.turbulence in runtime.traffic._children

    def test_noise_on_via_stack_steps_without_crash(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        run_cmd("CRE KL001,A320,52,4,90,FL250,300")
        run_cmd("NOISE ON")
        assert runtime.traffic.turbulence.active
        for _ in range(5):
            runtime.simulation.step()
        assert runtime.traffic.ntraf == 1


class TestTrails:
    def test_fresh_trails_object_has_background_buffers(
        self, runtime: MiniSky, sim: Simulation
    ) -> None:
        from minisky.traffic.trails import Trails

        trails = Trails(runtime.traffic, lambda: runtime.simulation)
        try:
            assert trails.bgacid == []  # used to exist only after clearbg()
            assert not hasattr(trails, "pygame")
        finally:
            runtime.traffic._children.remove(trails)

    def test_trail_on_update_and_buffer(self, runtime: MiniSky, run_cmd: RunCommand) -> None:
        run_cmd("CRE KL001,A320,52,4,90,FL250,300")
        run_cmd("TRAIL ON 1")
        assert runtime.traffic.trails.active
        for _ in range(5):
            runtime.simulation.step()
        assert len(runtime.traffic.trails.newlat0) > 0  # segments were recorded
        runtime.traffic.trails.buffer()  # must not crash on bgacid
        assert "KL001" in runtime.traffic.trails.bgacid
        run_cmd("TRAIL OFF")  # clears all trail data
