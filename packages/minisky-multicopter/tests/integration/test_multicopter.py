from __future__ import annotations

from minisky import MiniSky
from minisky import quantities as q
from minisky.simulation import Simulation
from minisky_multicopter.autopilot import MulticopterAutopilot
from tests._types import RunCommand, StepUntil


class TestPluginWiring:
    def test_membership_from_typecode(self, mcruntime: MiniSky, run_mc: RunCommand) -> None:
        run_mc("CRE D1,MAVIC,52,4,90,100FT[STD],20KT[CAS]")
        run_mc("CRE KL001,A320,53,4,90,FL100,250KT[CAS]")
        assert "ON" in run_mc("MCOPT D1")
        assert "OFF" in run_mc("MCOPT KL001")

    def test_yaw_rejects_non_multicopter(self, mcruntime: MiniSky, run_mc: RunCommand) -> None:
        run_mc("CRE KL001,A320,53,4,90,FL100,250KT[CAS]")
        assert "not a multicopter" in run_mc("YAW KL001 90")
        assert "not a multicopter" in run_mc("HOVER KL001")

    def test_yawrate_set_and_report(self, mcruntime: MiniSky, run_mc: RunCommand) -> None:
        run_mc("CRE D1,MAVIC,52,4,90,100FT[STD],20KT[CAS]")
        run_mc("YAWRATE D1 45")
        assert "45" in run_mc("YAWRATE D1")


class TestHoverAndYaw:
    def test_spd_zero_holds_position(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        traf = mcruntime.traffic
        run_mc("CRE D1,MAVIC,52,4,90,100FT[STD],20KT[CAS]")
        run_mc("SPD D1 0KT[CAS]")
        step_mc(lambda: traf.gs[0] == 0.0, 20)

        lat0, lon0 = float(traf.lat[0]), float(traf.lon[0])
        for _ in range(30):
            mcruntime.simulation.step()
        assert traf.gs[0] == 0.0
        assert float(traf.lat[0]) == lat0
        assert float(traf.lon[0]) == lon0

    def test_yaw_at_hover_slews_at_yawrate(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        traf = mcruntime.traffic
        run_mc("CRE D1,MAVIC,52,4,0,100FT[STD],20KT[CAS]")
        run_mc("SPD D1 0KT[CAS]")
        step_mc(lambda: traf.gs[0] == 0.0, 20)
        run_mc("YAWRATE D1 10")
        lat0, lon0 = float(traf.lat[0]), float(traf.lon[0])

        # HDG on a multicopter yaws the nose, rate-limited (not instant)
        run_mc("HDG D1 90")
        assert 5.0 < traf.hdg[0] < 15.0
        step_mc(lambda: abs(traf.hdg[0] - 90.0) < 0.1, 20)

        # rotated in place: no translation while hovering
        assert traf.gs[0] == 0.0
        assert float(traf.lat[0]) == lat0
        assert float(traf.lon[0]) == lon0

    def test_strafe_decouples_track_and_heading(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        traf = mcruntime.traffic
        run_mc("CRE D1,MAVIC,52,4,90,100FT[STD],20KT[CAS]")
        run_mc("YAWRATE D1 30")
        lon0 = float(traf.lon[0])

        # nose to north while the velocity vector keeps flying east
        run_mc("YAW D1 0")
        step_mc(lambda: abs(traf.hdg[0]) < 0.1 or abs(traf.hdg[0] - 360.0) < 0.1, 10)
        assert abs(traf.trk[0] - 90.0) < 0.1
        assert traf.gs[0] > 5.0
        assert float(traf.lon[0]) > lon0


class TestRouteFollowing:
    def test_leg_to_leg_course_capture(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        traf = mcruntime.traffic
        run_mc("CRE D1,MAVIC,52,4,90,100FT[STD],30KT[CAS]")
        run_mc("ADDWPT D1 52,4.005")
        run_mc("ADDWPT D1 52.005,4.005")
        run_mc("LNAV D1 ON")

        # fly-over route default for multicopters
        assert traf.ap.route[0].swflyby is False

        # reach the corner waypoint: active waypoint switches to the second
        step_mc(lambda: traf.actwp.lat[0] > 52.004, 60)
        lonmax = float(traf.lon[0])

        # course snaps to the new leg with no turn-anticipation overshoot arc
        mcruntime.simulation.step()
        mcruntime.simulation.step()
        assert min(traf.trk[0], 360.0 - traf.trk[0]) < 2.0  # northbound
        # never went further east than the corner + capture radius + one step
        assert lonmax < 4.005 + 0.0005
        step_mc(lambda: traf.lat[0] > 52.001, 30)
        assert abs(float(traf.lon[0]) - lonmax) < 0.0005

    def test_hover_mission_freezes_position_then_resumes(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        traf = mcruntime.traffic
        run_mc("CRE D1,MAVIC,52,4,90,100FT[STD],30KT[CAS]")
        run_mc("ADDWPT D1 52,4.02")
        run_mc("LNAV D1 ON")
        step_mc(lambda: traf.gs[0] > 5.0, 10)

        run_mc("HOVER D1 15")
        assert not traf.swlnav[0]
        step_mc(lambda: traf.gs[0] == 0.0, 20)

        # position frozen while the hold timer runs
        lat0, lon0 = float(traf.lat[0]), float(traf.lon[0])
        for _ in range(10):
            mcruntime.simulation.step()
        assert float(traf.lat[0]) == lat0
        assert float(traf.lon[0]) == lon0

        # after 15 s of held position the route resumes
        step_mc(lambda: bool(traf.swlnav[0]), 30)
        step_mc(lambda: traf.gs[0] > 5.0, 20)

    def test_hover_at_altitude_descends_in_place_and_resumes(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        traf = mcruntime.traffic
        ap = traf.ap
        assert isinstance(ap, MulticopterAutopilot)
        run_mc("CRE D1,MAVIC,52,4,90,400FT[STD],20KT[CAS]")
        run_mc("ADDWPT D1 52,4.02")
        run_mc("LNAV D1 ON")
        step_mc(lambda: traf.gs[0] > 5.0, 10)

        # hold 5 s at 100 ft: brakes to a hover, then moves vertically
        run_mc("HOVER D1 5 100FT[STD]")
        step_mc(lambda: traf.gs[0] == 0.0, 20)
        lat0, lon0 = float(traf.lat[0]), float(traf.lon[0])

        step_mc(lambda: abs(traf.alt[0] - q.ft_to_m(100.0)) < 0.5, 120)
        assert float(traf.lat[0]) == lat0
        assert float(traf.lon[0]) == lon0

        # after 5 s held at altitude, the route resumes at the hover altitude
        step_mc(lambda: not ap.swhover[0], 30)
        assert abs(traf.alt[0] - q.ft_to_m(100.0)) < 0.5
        assert float(traf.lat[0]) == lat0
        assert float(traf.lon[0]) == lon0
        assert traf.swlnav[0]
        step_mc(lambda: traf.gs[0] > 5.0, 20)

    def test_hover_composes_with_alt_and_lnav(
        self, mcruntime: MiniSky, run_mc: RunCommand, step_mc: StepUntil
    ) -> None:
        """A delivery profile written in scenario commands: hover, ALT down,
        ALT back up, LNAV ON to resume."""
        traf = mcruntime.traffic
        run_mc("CRE D1,MAVIC,52,4,90,400FT[STD],20KT[CAS]")
        run_mc("ADDWPT D1 52,4.02")
        run_mc("LNAV D1 ON")
        step_mc(lambda: traf.gs[0] > 5.0, 10)

        run_mc("HOVER D1")  # indefinite
        step_mc(lambda: traf.gs[0] == 0.0, 20)
        lat0, lon0 = float(traf.lat[0]), float(traf.lon[0])

        run_mc("ALT D1 100FT[STD]")  # plain ALT works inside a hover
        step_mc(lambda: abs(traf.alt[0] - q.ft_to_m(100.0)) < 0.5, 120)
        run_mc("ALT D1 400FT[STD]")
        step_mc(lambda: abs(traf.alt[0] - q.ft_to_m(400.0)) < 0.5, 120)
        assert traf.gs[0] == 0.0
        assert float(traf.lat[0]) == lat0
        assert float(traf.lon[0]) == lon0

        run_mc("LNAV D1 ON")  # cancels the indefinite hover
        step_mc(lambda: traf.gs[0] > 5.0, 20)


class TestFixedWingRegression:
    def test_fixed_wing_unaffected_by_plugin(
        self,
        mcruntime: MiniSky,
        mcsim: Simulation,
        runtime: MiniSky,
        sim: Simulation,
    ) -> None:
        """An A320 flies identically with and without the plugin loaded.

        The reference runtime is the shared session runtime (core
        implementations); the plugin runtime also carries a hovering
        multicopter to exercise the fleet-wide override paths.
        """
        commands = [
            "CRE KL001,A320,52,4,90,FL100,250KT[CAS]",
            "ALT KL001 FL120",
            "SPD KL001 280KT[CAS]",
        ]
        mcruntime.commands.stack("CRE D2,MAVIC,52.1,4,90,100FT[STD],20KT[CAS]")
        mcruntime.commands.stack("SPD D2 0KT[CAS]")
        for cmd in commands:
            mcruntime.commands.stack(cmd)
            runtime.commands.stack(cmd)

        for _ in range(60):
            mcruntime.simulation.step()
            runtime.simulation.step()

        i = mcruntime.traffic.idx("KL001")
        j = runtime.traffic.idx("KL001")
        for name in ("lat", "lon", "alt", "hdg", "trk", "tas", "gs", "vs"):
            assert getattr(mcruntime.traffic, name)[i] == getattr(runtime.traffic, name)[j], name
