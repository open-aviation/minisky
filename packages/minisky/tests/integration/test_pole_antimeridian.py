"""Integration tests for pole crossings and antimeridian wraparound.

Past a pole, a great-circle track continues on the opposite meridian: the
local north/east directions flip, so heading, track and the active leg
direction must flip by 180 degrees along with the mirrored latitude and
longitude (see Kinematics.update_pos). Antimeridian wraparound is the more
common case of keeping longitude in [-180, 180) without any pole involved.

Reported in https://github.com/open-aviation/minisky/issues/14 and fixed
in BlueSky by https://github.com/TUDelft-CNS-ATM/bluesky/pull/664.
"""

from __future__ import annotations

import pytest
from minisky import MiniSky
from minisky._internal.simulation import Simulation
from tests._types import RunCommand, StepUntil


class TestPoleCrossing:
    def test_due_north_crossing_wraps_latitude_and_flips_heading_and_track(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil
    ) -> None:
        run_cmd("CRE KL001,A320,89.9,0,0,FL300,400KT[CAS]")
        step_until(lambda: runtime.traffic.hdg[0] != 0.0, max_steps=200)

        assert runtime.traffic.lat[0] <= 90.0
        assert runtime.traffic.hdg[0] == pytest.approx(180.0)
        assert runtime.traffic.trk[0] == pytest.approx(180.0)

    def test_south_pole_crossing_wraps_latitude_and_flips_heading(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil
    ) -> None:
        run_cmd("CRE KL002,A320,-89.9,0,180,FL300,400KT[CAS]")
        step_until(lambda: runtime.traffic.hdg[0] != 180.0, max_steps=200)

        assert runtime.traffic.lat[0] >= -90.0
        assert runtime.traffic.hdg[0] == pytest.approx(0.0)
        assert runtime.traffic.trk[0] == pytest.approx(0.0)

    def test_oblique_heading_crossing_flips_heading_and_track_together(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil
    ) -> None:
        run_cmd("CRE KL003,A320,89.9,4,45,FL300,400KT[CAS]")
        step_until(lambda: runtime.traffic.hdg[0] != 45.0, max_steps=200)

        assert runtime.traffic.lat[0] <= 90.0
        assert runtime.traffic.hdg[0] == pytest.approx(225.0)
        assert runtime.traffic.trk[0] == pytest.approx(runtime.traffic.hdg[0])

    def test_crossing_flips_groundspeed_components(
        self, runtime: MiniSky, run_cmd: RunCommand, sim: Simulation
    ) -> None:
        run_cmd("CRE KL004,A320,89.9,0,0,FL300,400KT[CAS]")

        # capture ground speed from the step right before the crossing flips
        # heading, since TAS is still ramping up to the commanded speed
        gsnorth_before = float(runtime.traffic.gsnorth[0])
        for _ in range(200):
            sim.step()
            if runtime.traffic.hdg[0] != 0.0:
                break
            gsnorth_before = float(runtime.traffic.gsnorth[0])
        else:
            pytest.fail("aircraft never crossed the pole")

        # loose tolerance: TAS is still ramping toward the commanded speed, so
        # the magnitude can shift slightly between the last pre-crossing step
        # and the crossing step itself
        assert runtime.traffic.gsnorth[0] == pytest.approx(-gsnorth_before, abs=2.0)


class TestPoleCrossingActiveLegDirection:
    def test_crossing_flips_active_leg_direction_when_present(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil
    ) -> None:
        run_cmd("CRE KL005,A320,89.9,0,0,FL300,400KT[CAS]")
        run_cmd("ADDWPT KL005 89.0,180")
        assert runtime.traffic.actwp.curlegdir.present[0]
        curlegdir_before = float(runtime.traffic.actwp.curlegdir.values[0])

        step_until(lambda: runtime.traffic.hdg[0] != 0.0, max_steps=200)

        assert runtime.traffic.actwp.curlegdir.present[0]
        assert runtime.traffic.actwp.curlegdir.values[0] == pytest.approx(
            (curlegdir_before + 180.0) % 360.0
        )

    def test_crossing_leaves_absent_leg_direction_untouched(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil
    ) -> None:
        run_cmd("CRE KL006,A320,89.9,0,0,FL300,400KT[CAS]")
        assert not runtime.traffic.actwp.curlegdir.present[0]

        step_until(lambda: runtime.traffic.hdg[0] != 0.0, max_steps=200)

        assert not runtime.traffic.actwp.curlegdir.present[0]


class TestAntimeridianCrossing:
    def test_eastbound_crossing_wraps_longitude_negative(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil
    ) -> None:
        run_cmd("CRE KL007,A320,0,179.9,90,FL300,400KT[CAS]")
        step_until(lambda: runtime.traffic.lon[0] < 0.0, max_steps=200)

        assert -180.0 <= runtime.traffic.lon[0] < 180.0
        # heading/track are unaffected away from the poles
        assert runtime.traffic.hdg[0] == pytest.approx(90.0)

    def test_westbound_crossing_wraps_longitude_positive(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil
    ) -> None:
        run_cmd("CRE KL008,A320,0,-179.9,270,FL300,400KT[CAS]")
        step_until(lambda: runtime.traffic.lon[0] > 0.0, max_steps=200)

        assert -180.0 <= runtime.traffic.lon[0] < 180.0
        assert runtime.traffic.hdg[0] == pytest.approx(270.0)

    def test_waypoint_route_survives_antimeridian_crossing(
        self, runtime: MiniSky, run_cmd: RunCommand, step_until: StepUntil
    ) -> None:
        run_cmd("CRE KL009,A320,0,179.9,90,FL300,400KT[CAS]")
        run_cmd("ADDWPT KL009 0,-179.0")
        assert runtime.traffic.actwp.curlegdir.present[0]
        curlegdir_before = float(runtime.traffic.actwp.curlegdir.values[0])

        step_until(lambda: runtime.traffic.lon[0] < 0.0, max_steps=200)

        # no pole involved: the active leg direction is untouched by wraparound
        assert runtime.traffic.actwp.curlegdir.present[0]
        assert runtime.traffic.actwp.curlegdir.values[0] == pytest.approx(curlegdir_before, abs=1.0)
