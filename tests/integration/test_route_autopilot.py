"""Integration tests for route management (ADDWPT/DEST) and autopilot guidance."""

import pytest

from minisky.tools import geo

FT = 0.3048
KTS = 0.514444


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


class TestRouteEditing:
    """Regression tests for route-editing bugs from docs/known-issues.md."""

    def test_addwpt_accepts_string_callsign(self, bs, run_cmd, aircraft):
        # addwpt() with a callsign string used to crash on the callsign lookup
        result = bs.traffic.route.addwpt(aircraft, "52.5,5.0")
        assert result is True
        route = bs.traf.ap.route[0]
        assert route.wplat[0] == pytest.approx(52.5)
        assert route.wplon[0] == pytest.approx(5.0)

    def test_direct_switches_active_waypoint(self, bs, run_cmd, aircraft):
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = bs.traf.ap.route[0]
        assert bs.traffic.route.direct(0, route.wpname[1]) is True
        assert route.iactwp == 1
        assert bs.traf.actwp.lat[0] == pytest.approx(53.0)

    def test_direct_with_turn_heading_rate(self, bs, run_cmd, aircraft):
        # direct() used bare `pi` in the heading-rate branch (NameError)
        run_cmd(f"ADDWPT {aircraft} TURNHDG 3")
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        # Second waypoint activates the first one via direct()
        out = run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        assert "Error" not in out
        route = bs.traf.ap.route[0]
        assert route.wpturnhdgr == [3.0, 3.0]
        assert route.iactwp == 0
        assert bs.traf.swlnav[0]

    def test_delwpt_active_waypoint_redirects(self, bs, run_cmd, aircraft):
        # delwpt() used to call the nonexistent Route.direct method
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = bs.traf.ap.route[0]
        first, second = route.wpname
        out = run_cmd(f"DELWPT {aircraft} {first}")
        assert "Error" not in out
        assert route.wpname == [second]
        assert route.iactwp == 0
        assert bs.traf.actwp.lat[0] == pytest.approx(53.0)

    def test_at_wpt_sets_alt_and_spd_constraints(self, bs, run_cmd, aircraft):
        # The alt/spd branch wrote the speed into the altitude constraint
        # and called the nonexistent Route.direct method
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = bs.traf.ap.route[0]
        result = bs.traffic.route.at_wpt(0, route.wpname[1], "FL090/250")
        assert result is True
        assert route.wpalt[1] == pytest.approx(9000 * FT, rel=1e-3)
        assert route.wpspd[1] == pytest.approx(250 * KTS, rel=1e-3)

    def test_lnav_reengage_issues_direct(self, bs, run_cmd, aircraft):
        # setLNAV used to call the nonexistent Route.direct method
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        run_cmd(f"LNAV {aircraft} OFF")
        assert not bs.traf.swlnav[0]
        out = run_cmd(f"LNAV {aircraft} ON")
        assert "Error" not in out
        assert bs.traf.swlnav[0]

    def test_at_via_stack_sets_constraints(self, bs, run_cmd, aircraft):
        # The AT registration used help text as its argument spec, so the
        # command never reached at_wpt() from the stack
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = bs.traf.ap.route[0]
        out = run_cmd(f"{aircraft} AT {route.wpname[1]} FL090/250")
        assert "Error" not in out
        assert route.wpalt[1] == pytest.approx(9000 * FT, rel=1e-3)
        assert route.wpspd[1] == pytest.approx(250 * KTS, rel=1e-3)

    def test_direct_via_stack(self, bs, run_cmd, aircraft):
        # The DIRECT argument spec had a stray space (" wpt"), dropping the
        # waypoint parameter so DIRECT always rejected its second argument
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = bs.traf.ap.route[0]
        out = run_cmd(f"DIRECT {aircraft} {route.wpname[1]}")
        assert "Error" not in out
        assert route.iactwp == 1

    def test_after_and_before_via_stack(self, bs, run_cmd, aircraft):
        # AFTER/BEFORE specs contained unparseable tokens, and the ADDWPT
        # keyword parameter shadowed the addwpt() function
        run_cmd(f"ADDWPT {aircraft} EH007")
        run_cmd(f"ADDWPT {aircraft} HELEN")
        route = bs.traf.ap.route[0]
        out = run_cmd(f"{aircraft} AFTER EH007 ADDWPT SPY")
        assert "Error" not in out
        out = run_cmd(f"{aircraft} BEFORE HELEN ADDWPT PAM")
        assert "Error" not in out
        assert route.wpname == ["EH007", "SPY", "PAM", "HELEN"]


class TestStatusQueries:
    def test_vnav_query_reports_state(self, bs, run_cmd, aircraft):
        # The VNAV query path referenced nonexistent minisky.traf.id
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0 FL110")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        run_cmd(f"VNAV {aircraft} ON")
        out = run_cmd(f"VNAV {aircraft}")
        assert f"{aircraft}: VNAV is ON" in out
        run_cmd(f"VNAV {aircraft} OFF")
        out = run_cmd(f"VNAV {aircraft}")
        assert f"{aircraft}: VNAV is OFF" in out

    def test_swtod_status_reflects_switch(self, bs, run_cmd, aircraft):
        # SWTOD status output used to read swtoc instead of swtod
        out = run_cmd(f"SWTOD {aircraft}")
        assert f"{aircraft}: SWTOD is ON" in out
        run_cmd(f"SWTOD {aircraft} OFF")
        assert bs.traf.ap.swtoc[0]  # ToC switch must stay untouched
        out = run_cmd(f"SWTOD {aircraft}")
        assert f"{aircraft}: SWTOD is OFF" in out


class TestActiveWaypointDefaults:
    def test_mcre_initialises_nextaltco_for_all(self, bs, run_cmd):
        # ActiveWaypoint.create() used nextaltco[-n] instead of [-n:],
        # leaving all but one new aircraft without the -999 sentinel
        run_cmd("MCRE 3")
        assert bs.traf.ntraf == 3
        assert (bs.traf.actwp.nextaltco == -999.0).all()


class TestGuidanceGeometry:
    def test_aircraft_approaches_waypoint_with_lnav(self, bs, run_cmd, step_until, aircraft):
        wplat, wplon = 52.6, 4.0
        run_cmd(f"ADDWPT {aircraft} {wplat},{wplon}")
        run_cmd(f"LNAV {aircraft} ON")

        def dist_nm():
            return geo.kwikdist(bs.traf.lat[0], bs.traf.lon[0], wplat, wplon)

        start = dist_nm()
        step_until(lambda: dist_nm() < start / 2, max_steps=600)
