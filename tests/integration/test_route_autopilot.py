"""Integration tests for route management (ADDWPT/DEST) and autopilot guidance."""

import pytest

from minisky.tools import geo

FT = 0.3048


@pytest.fixture
def aircraft(bs, run_cmd):
    """A single aircraft at (52, 4) heading east at FL100."""
    run_cmd("CRE KL001,A320,52,4,90,FL100,250")
    assert bs.traf.ntraf == 1
    return "KL001"


class TestAddwpt:
    def test_addwpt_by_latlon(self, bs, run_cmd, aircraft):
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        route = bs.traf.ap.route[0]
        assert len(route.wpname) == 1
        assert route.wplat[0] == pytest.approx(52.5)
        assert route.wplon[0] == pytest.approx(5.0)

    def test_addwpt_by_navdb_name(self, bs, run_cmd, aircraft):
        # SUGOL is a real waypoint near EHAM in the bundled navdata
        run_cmd(f"ADDWPT {aircraft} SUGOL")
        route = bs.traf.ap.route[0]
        assert len(route.wpname) == 1
        assert "SUGOL" in route.wpname[0]

    def test_addwpt_multiple_in_order(self, bs, run_cmd, aircraft):
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = bs.traf.ap.route[0]
        assert len(route.wpname) == 2
        assert route.wplat == [52.5, 53.0]

    def test_addwpt_with_altitude_constraint(self, bs, run_cmd, aircraft):
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0 FL150")
        route = bs.traf.ap.route[0]
        assert route.wpalt[0] == pytest.approx(15000 * FT, rel=1e-3)

    def test_dest_resolves_airport(self, bs, run_cmd, aircraft):
        run_cmd(f"DEST {aircraft} EHAM")
        route = bs.traf.ap.route[0]
        # EHAM (Schiphol) is at approximately (52.31, 4.76)
        assert route.wplat[-1] == pytest.approx(52.31, abs=0.1)
        assert route.wplon[-1] == pytest.approx(4.76, abs=0.1)


class TestLnav:
    def test_lnav_turns_toward_waypoint(self, bs, run_cmd, step_until, aircraft):
        # Waypoint to the north; aircraft initially heading east
        run_cmd(f"ADDWPT {aircraft} 54.0,4.0")
        run_cmd(f"LNAV {aircraft} ON")
        assert bs.traf.swlnav[0]

        def heading_north():
            hdg = bs.traf.hdg[0] % 360.0
            return hdg > 350.0 or hdg < 10.0

        step_until(heading_north, max_steps=300)

    def test_lnav_off_keeps_heading(self, bs, run_cmd, aircraft):
        run_cmd(f"ADDWPT {aircraft} 54.0,4.0")
        run_cmd(f"LNAV {aircraft} OFF")
        for _ in range(30):
            bs.sim.step()
        assert bs.traf.hdg[0] == pytest.approx(90.0, abs=1.0)


class TestVerticalGuidance:
    def test_alt_command_captures_altitude(self, bs, run_cmd, step_until, aircraft):
        target = 11000 * FT
        run_cmd(f"ALT {aircraft} FL110")
        step_until(lambda: abs(bs.traf.alt[0] - target) < 50 * FT, max_steps=600)

    def test_vertical_speed_settles_after_capture(self, bs, run_cmd, step_until, aircraft):
        target = 11000 * FT
        run_cmd(f"ALT {aircraft} FL110")
        step_until(lambda: abs(bs.traf.alt[0] - target) < 20 * FT, max_steps=600)
        for _ in range(20):
            bs.sim.step()
        assert bs.traf.vs[0] == pytest.approx(0.0, abs=0.5)
        assert bs.traf.alt[0] == pytest.approx(target, rel=1e-2)

    def test_descent(self, bs, run_cmd, step_until, aircraft):
        target = 8000 * FT
        run_cmd(f"ALT {aircraft} FL080")
        step_until(lambda: abs(bs.traf.alt[0] - target) < 50 * FT, max_steps=600)


class TestGuidanceGeometry:
    def test_aircraft_approaches_waypoint_with_lnav(self, bs, run_cmd, step_until, aircraft):
        wplat, wplon = 52.6, 4.0
        run_cmd(f"ADDWPT {aircraft} {wplat},{wplon}")
        run_cmd(f"LNAV {aircraft} ON")

        def dist_nm():
            return geo.kwikdist(bs.traf.lat[0], bs.traf.lon[0], wplat, wplon)

        start = dist_nm()
        step_until(lambda: dist_nm() < start / 2, max_steps=600)
