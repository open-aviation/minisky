"""Integration tests for route management (ADDWPT/DEST) and autopilot guidance."""

from __future__ import annotations

from typing import ClassVar

import numpy as np
import pytest
from minisky import MiniSky
from minisky import quantities as q
from minisky.tools import geo
from minisky.traffic import route as route_commands
from minisky.traffic.route import Route, TurnHeadingRate
from tests._types import RunCommand, StepUntil


@pytest.fixture
def aircraft(runtime: MiniSky, run_cmd: RunCommand) -> str:
    """A single aircraft at (52, 4) heading east at FL100."""
    run_cmd("CRE KL001,A320,52,4,90,FL100,250")
    assert runtime.traffic.ntraf == 1
    return "KL001"


class TestAddwpt:
    def test_addwpt_by_latlon(self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str) -> None:
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        route = runtime.traffic.ap.route[0]
        assert len(route.wpname) == 1
        assert route.wplat[0] == pytest.approx(52.5)
        assert route.wplon[0] == pytest.approx(5.0)
        assert route.wpalt[0] is None
        assert route.wpspd[0] is None
        assert route.wprta[0] is None
        assert route.wpprofile[0].altitude is None
        assert route.wpprofile[0].rta is None
        assert route.iactwp == 0
        assert runtime.traffic.swlnav[0]

    def test_addwpt_by_navdb_name(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # SUGOL is a real waypoint near EHAM in the bundled navdata
        run_cmd(f"ADDWPT {aircraft} SUGOL")
        route = runtime.traffic.ap.route[0]
        assert len(route.wpname) == 1
        assert "SUGOL" in route.wpname[0]

    def test_addwpt_multiple_in_order(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = runtime.traffic.ap.route[0]
        assert len(route.wpname) == 2
        assert route.wplat == [52.5, 53.0]

    def test_addwpt_with_altitude_constraint(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0 FL150")
        route = runtime.traffic.ap.route[0]
        assert route.wpalt[0] == pytest.approx(q.ft_to_m(15000.0), rel=1e-3)
        target = route.wpprofile[0].altitude
        assert target is not None
        assert target.altitude == pytest.approx(q.ft_to_m(15000.0), rel=1e-3)
        assert target.distance == 0.0

    @pytest.mark.parametrize("runway", ["EHAM/RW06", "EHAM RW06"])
    def test_addwpt_takeoff_with_explicit_runway(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str, runway: str
    ) -> None:
        out = run_cmd(f"ADDWPT {aircraft} TAKEOFF {runway}")
        route = runtime.traffic.ap.route[0]
        assert "error:" not in out.lower()
        assert route.wpname == [f"T/O-{aircraft}"]
        rwylat, rwylon, _ = runtime.traffic.navigation.rwythresholds["EHAM"]["06"]
        _, distance = geo.qdrdist(rwylat, rwylon, route.wplat[0], route.wplon[0])
        assert distance == pytest.approx(q.nmi_to_m(2.0), rel=1e-3)

    def test_dest_resolves_airport(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        run_cmd(f"DEST {aircraft} EHAM")
        route = runtime.traffic.ap.route[0]
        # EHAM (Schiphol) is at approximately (52.31, 4.76)
        assert route.wplat[-1] == pytest.approx(52.31, abs=0.1)
        assert route.wplon[-1] == pytest.approx(4.76, abs=0.1)


class TestLnav:
    def test_lnav_turns_toward_waypoint(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil, aircraft: str
    ) -> None:
        # Waypoint to the north; aircraft initially heading east
        run_cmd(f"ADDWPT {aircraft} 54.0,4.0")
        run_cmd(f"LNAV {aircraft} ON")
        assert runtime.traffic.swlnav[0]

        def heading_north() -> bool:
            hdg = runtime.traffic.hdg[0] % 360.0
            return hdg > 350.0 or hdg < 10.0

        step_until(heading_north, max_steps=300)


class TestVerticalGuidance:
    def test_alt_command_captures_altitude(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil, aircraft: str
    ) -> None:
        target = q.ft_to_m(11000.0)
        run_cmd(f"ALT {aircraft} FL110")
        step_until(lambda: abs(runtime.traffic.alt[0] - target) < q.ft_to_m(50.0), max_steps=600)

    def test_vertical_speed_settles_after_capture(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil, aircraft: str
    ) -> None:
        target = q.ft_to_m(11000.0)
        run_cmd(f"ALT {aircraft} FL110")
        step_until(lambda: abs(runtime.traffic.alt[0] - target) < q.ft_to_m(20.0), max_steps=600)
        for _ in range(20):
            runtime.simulation.step()
        assert runtime.traffic.vs[0] == pytest.approx(0.0, abs=0.5)
        assert runtime.traffic.alt[0] == pytest.approx(target, rel=1e-2)

    def test_descent(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil, aircraft: str
    ) -> None:
        target = q.ft_to_m(8000.0)
        run_cmd(f"ALT {aircraft} FL080")
        step_until(lambda: abs(runtime.traffic.alt[0] - target) < q.ft_to_m(50.0), max_steps=600)


class TestRouteEditing:
    """Regression tests for route-editing bugs from docs/known-issues.md."""

    def test_direct_switches_active_waypoint(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = runtime.traffic.ap.route[0]
        assert route_commands.direct(runtime.traffic, 0, route.wpname[1]) is True
        assert route.iactwp == 1
        assert runtime.traffic.actwp.lat[0] == pytest.approx(53.0)

    def test_direct_with_turn_heading_rate(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # direct() used bare `pi` in the heading-rate branch (NameError)
        run_cmd(f"ADDWPT {aircraft} TURNHDG 3")
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        out = run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        assert "Error" not in out
        route = runtime.traffic.ap.route[0]
        assert all(
            turn is not None and turn.geometry == TurnHeadingRate(3.0) for turn in route.wpturn
        )
        assert route.iactwp == 0
        assert runtime.traffic.swlnav[0]

    def test_delwpt_active_waypoint_redirects(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # delwpt() used to call the nonexistent Route.direct method
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = runtime.traffic.ap.route[0]
        first, second = route.wpname
        out = run_cmd(f"DELWPT {aircraft} {first}")
        assert "Error" not in out
        assert route.wpname == [second]
        assert route.iactwp == 0
        assert runtime.traffic.actwp.lat[0] == pytest.approx(53.0)

    def test_at_wpt_sets_alt_and_spd_constraints(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # The alt/spd branch wrote the speed into the altitude constraint
        # and called the nonexistent Route.direct method
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = runtime.traffic.ap.route[0]
        output = run_cmd(f"{aircraft} AT {route.wpname[1]} FL090/250")
        assert "Error" not in output
        assert route.wpalt[1] == pytest.approx(q.ft_to_m(9000.0), rel=1e-3)
        assert route.wpspd[1] == pytest.approx(q.kt_to_mps(250.0), rel=1e-3)

    def test_rta_profile_pairs_time_and_distance(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = runtime.traffic.ap.route[0]
        assert route_commands.set_rta(runtime.traffic, 0, route.wpname[1], 1000.0)

        first_target = route.wpprofile[0].rta
        second_target = route.wpprofile[1].rta
        assert first_target is not None
        assert second_target is not None
        assert first_target.time == second_target.time == 1000.0
        assert first_target.distance > 0.0
        assert second_target.distance == 0.0

    def test_lnav_reengage_issues_direct(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # setLNAV used to call the nonexistent Route.direct method
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        run_cmd(f"LNAV {aircraft} OFF")
        assert not runtime.traffic.swlnav[0]
        out = run_cmd(f"LNAV {aircraft} ON")
        assert "Error" not in out
        assert runtime.traffic.swlnav[0]

    def test_at_stacked_command_is_stored_verbatim(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        route = runtime.traffic.ap.route[0]
        waypoint = route.wpname[0]
        output = run_cmd(f"{aircraft} AT {waypoint} DO {aircraft} DELRTE")
        assert "Error" not in output
        assert route.wpstack[0] == [f"{aircraft} DELRTE"]

        # we break compatibility with bluesky here, the aircraft target must be
        # explicit.
        output = run_cmd(f"{aircraft} AT {waypoint} STACK ALT 95")
        assert "Error" not in output
        assert route.wpstack[0][-1] == "ALT 95"

    def test_at_via_stack_sets_constraints(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # The AT registration used help text as its argument spec, so the
        # command never reached the AT implementation from the stack
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = runtime.traffic.ap.route[0]
        out = run_cmd(f"{aircraft} AT {route.wpname[1]} FL090/250")
        assert "Error" not in out
        assert route.wpalt[1] == pytest.approx(q.ft_to_m(9000.0), rel=1e-3)
        assert route.wpspd[1] == pytest.approx(q.kt_to_mps(250.0), rel=1e-3)

    def test_direct_via_stack(self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str) -> None:
        # The DIRECT argument spec had a stray space (" wpt"), dropping the
        # waypoint parameter so DIRECT always rejected its second argument
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        route = runtime.traffic.ap.route[0]
        out = run_cmd(f"DIRECT {aircraft} {route.wpname[1]}")
        assert "Error" not in out
        assert route.iactwp == 1

    def test_after_and_before_via_stack(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # AFTER/BEFORE specs contained unparseable tokens, and the ADDWPT
        # keyword parameter shadowed the addwpt() function
        run_cmd(f"ADDWPT {aircraft} EH007")
        run_cmd(f"ADDWPT {aircraft} HELEN")
        route = runtime.traffic.ap.route[0]
        out = run_cmd(f"{aircraft} AFTER EH007 ADDWPT SPY")
        assert "Error" not in out
        out = run_cmd(f"{aircraft} BEFORE HELEN ADDWPT PAM")
        assert "Error" not in out
        assert route.wpname == ["EH007", "SPY", "PAM", "HELEN"]


class TestStatusQueries:
    def test_vnav_query_reports_state(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # The VNAV query path referenced nonexistent traffic.id
        run_cmd(f"ADDWPT {aircraft} 52.5,5.0 FL110")
        run_cmd(f"ADDWPT {aircraft} 53.0,6.0")
        run_cmd(f"VNAV {aircraft} ON")
        out = run_cmd(f"VNAV {aircraft}")
        assert f"{aircraft}: VNAV is ON" in out
        run_cmd(f"VNAV {aircraft} OFF")
        out = run_cmd(f"VNAV {aircraft}")
        assert f"{aircraft}: VNAV is OFF" in out

    def test_swtod_status_reflects_switch(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # SWTOD status output used to read swtoc instead of swtod
        out = run_cmd(f"SWTOD {aircraft}")
        assert f"{aircraft}: SWTOD is ON" in out
        run_cmd(f"SWTOD {aircraft} OFF")
        assert runtime.traffic.ap.swtoc[0]  # ToC switch must stay untouched
        out = run_cmd(f"SWTOD {aircraft}")
        assert f"{aircraft}: SWTOD is OFF" in out


class TestActiveWaypointDefaults:
    def test_mcre_initialises_nextaltco_for_all(
        self, runtime: MiniSky, run_cmd: RunCommand
    ) -> None:
        # ActiveWaypoint.create() used nextaltco[-n] instead of [-n:],
        # leaving all but one new aircraft with an invalid absence state.
        run_cmd("MCRE 3")
        assert runtime.traffic.ntraf == 3
        assert not runtime.traffic.actwp.nextaltco.present.any()


class TestGuidanceGeometry:
    def test_aircraft_approaches_waypoint_with_lnav(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil, aircraft: str
    ) -> None:
        wplat, wplon = 52.6, 4.0
        run_cmd(f"ADDWPT {aircraft} {wplat},{wplon}")
        run_cmd(f"LNAV {aircraft} ON")

        def dist_nm() -> float:
            return float(
                np.asarray(
                    geo.kwikdist(runtime.traffic.lat[0], runtime.traffic.lon[0], wplat, wplon)
                ).item()
            )

        start = dist_nm()
        step_until(lambda: dist_nm() < start / 2, max_steps=600)


class TestWaypointSwitching:
    """End-to-end LNAV waypoint switching through wppassingcheck().

    The scenario files perform no waypoint switches (kl204.scn disengages
    LNAV with a HDG command), so these tests are the only coverage of the
    switching path: getnextwp()/getnextturnwp() and the vectorized leg
    update in the autopilot.
    """

    # Zig-zag legs of ~2 nm force a real heading change at every waypoint
    WPTS: ClassVar[list[tuple[float, float]]] = [
        (52.00, 4.05),
        (52.03, 4.10),
        (52.00, 4.15),
        (52.03, 4.20),
    ]

    @pytest.fixture
    def route(self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str) -> Route:
        for lat, lon in self.WPTS:
            run_cmd(f"ADDWPT {aircraft} {lat},{lon}")
        run_cmd(f"LNAV {aircraft} ON")
        run_cmd(f"VNAV {aircraft} ON")
        return runtime.traffic.ap.route[0]

    def test_switches_through_route_and_disengages_at_end(
        self, runtime: MiniSky, step_until: StepUntil, route: Route
    ) -> None:
        assert route.iactwp == 0
        for target in range(1, len(self.WPTS)):
            step_until(lambda target=target: route.iactwp == target, max_steps=200)
            assert runtime.traffic.actwp.lat[0] == pytest.approx(self.WPTS[target][0])
            assert runtime.traffic.actwp.lon[0] == pytest.approx(self.WPTS[target][1])
        # Passing the final waypoint switches LNAV and VNAV off
        step_until(lambda: not runtime.traffic.swlnav[0], max_steps=200)
        assert not runtime.traffic.swvnav[0]

    def test_next_qdr_matches_next_leg_bearing(
        self, runtime: MiniSky, step_until: StepUntil, route: Route
    ) -> None:
        step_until(lambda: route.iactwp == 1, max_steps=200)
        expected, _ = geo.qdrdist(*self.WPTS[1], *self.WPTS[2])
        assert runtime.traffic.actwp.next_qdr.values[0] == pytest.approx(expected)

    def test_last_waypoint_has_no_next_leg(
        self, runtime: MiniSky, step_until: StepUntil, route: Route
    ) -> None:
        step_until(lambda: route.iactwp == len(self.WPTS) - 1, max_steps=600)
        assert route.getnextleg() is None
        assert route.getnextqdr() is None

    def test_nextturn_data_tracks_upcoming_flyturn(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil, aircraft: str
    ) -> None:
        # Waypoint 2 is a fly-turn waypoint with a turn speed; 0, 1 and 3 are fly-by
        run_cmd(f"ADDWPT {aircraft} {self.WPTS[0][0]},{self.WPTS[0][1]}")
        run_cmd(f"ADDWPT {aircraft} {self.WPTS[1][0]},{self.WPTS[1][1]}")
        run_cmd(f"ADDWPT {aircraft} TURNSPD 250")
        run_cmd(f"ADDWPT {aircraft} {self.WPTS[2][0]},{self.WPTS[2][1]}")
        run_cmd(f"ADDWPT {aircraft} FLYBY")
        run_cmd(f"ADDWPT {aircraft} {self.WPTS[3][0]},{self.WPTS[3][1]}")
        run_cmd(f"LNAV {aircraft} ON")
        route = runtime.traffic.ap.route[0]

        # After passing waypoint 0, the next fly-turn waypoint is index 2
        step_until(lambda: route.iactwp == 1, max_steps=200)
        next_turn = route.getnextturnwp()
        assert next_turn is not None
        assert next_turn.waypoint_index == 2
        assert next_turn.turn.speed == pytest.approx(q.kt_to_mps(250.0), rel=1e-3)
        assert runtime.traffic.actwp.nextturnidx.values[0] == 2
        assert runtime.traffic.actwp.nextturnlat.values[0] == pytest.approx(self.WPTS[2][0])
        assert runtime.traffic.actwp.nextturnlon.values[0] == pytest.approx(self.WPTS[2][1])
        assert runtime.traffic.actwp.nextturnspd.values[0] == pytest.approx(
            q.kt_to_mps(250.0), rel=1e-3
        )

        # The active waypoint itself counts: still index 2 while flying to it
        step_until(lambda: route.iactwp == 2, max_steps=200)
        assert runtime.traffic.actwp.nextturnidx.values[0] == 2
        assert runtime.traffic.actwp.turnspd.values[0] == pytest.approx(
            q.kt_to_mps(250.0), rel=1e-3
        )

        # Once past the fly-turn waypoint there is no upcoming turn.
        step_until(lambda: route.iactwp == 3, max_steps=600)
        assert route.getnextturnwp() is None
        assert not runtime.traffic.actwp.nextturnidx.present[0]

    def test_flyturn_without_turn_speed_does_not_override_speed(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # BlueSky 55c641e (2023-06-21) treated next-turn presence as turn-speed presence.
        run_cmd(f"ADDWPT {aircraft} {self.WPTS[0][0]},{self.WPTS[0][1]}")
        run_cmd(f"ADDWPT {aircraft} TURNRAD 1")
        run_cmd(f"ADDWPT {aircraft} {self.WPTS[1][0]},{self.WPTS[1][1]}")
        run_cmd(f"VNAV {aircraft} ON")

        assert not runtime.traffic.actwp.nextturnspd.present[0]
        runtime.simulation.step()
        assert runtime.traffic.selspd[0] > 0.0

    def test_first_waypoint_turn_speed_keeps_index_zero_valid(
        self, runtime: MiniSky, run_cmd: RunCommand, aircraft: str
    ) -> None:
        # BlueSky 55c641e (2023-06-21) used nextturnidx > 0; 08194fa (2023-06-22) fixed index 0.
        run_cmd(f"ADDWPT {aircraft} TURNSPD 200")
        run_cmd(f"ADDWPT {aircraft} {self.WPTS[0][0]},{self.WPTS[0][1]}")
        route = runtime.traffic.ap.route[0]

        assert route.iactwp == 0
        assert runtime.traffic.actwp.nextturnidx.values[0] == 0
        assert runtime.traffic.actwp.nextturnspd.values[0] == pytest.approx(
            q.kt_to_mps(200.0), rel=1e-3
        )

    def test_no_flyturn_waypoints_has_no_next_turn(
        self, runtime: MiniSky, step_until: StepUntil, route: Route
    ) -> None:
        step_until(lambda: route.iactwp == 1, max_steps=200)
        assert route.getnextturnwp() is None
