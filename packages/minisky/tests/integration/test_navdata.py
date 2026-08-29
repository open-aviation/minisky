"""Integration tests for navigation data."""

from __future__ import annotations

import numpy as np
from minisky import MiniSky
from minisky._internal.simulation import Simulation


class TestDefwpt:
    def test_defwpt_adds_waypoint(self, runtime: MiniSky, sim: Simulation) -> None:
        waypoints = runtime.waypoints
        n = len(waypoints.identifiers)
        result = waypoints.defwpt("TSTWPT1", 52.0, 4.0, "FIX")
        assert result.is_ok()
        assert len(waypoints.identifiers) == n + 1
        idx = int(np.flatnonzero(waypoints.identifiers == "TSTWPT1")[0])
        assert waypoints.latitudes[idx] == 52.0
        assert waypoints.longitudes[idx] == 4.0

    def test_delwpt_removes_aligned_row(self, runtime: MiniSky, sim: Simulation) -> None:
        # Regression: delwpt discarded the result of np.delete, so
        # wplat/wplon kept the deleted waypoint's coordinates
        waypoints = runtime.waypoints
        waypoints.defwpt("TSTWPTA", 52.0, 4.0, "FIX")
        waypoints.defwpt("TSTWPTB", 10.0, 20.0, "FIX")

        assert waypoints.delwpt("TSTWPTA").is_ok()
        # Remaining waypoint's coordinates must still be index-aligned
        idx = int(np.flatnonzero(waypoints.identifiers == "TSTWPTB")[0])
        assert waypoints.latitudes[idx] == 10.0
        assert waypoints.longitudes[idx] == 20.0

    def test_defwpt_delete_via_stack(self, runtime: MiniSky, sim: Simulation, run_cmd) -> None:
        runtime.waypoints.defwpt("TSTWPT3", 52.0, 4.0)
        output = run_cmd("DEFWPT TSTWPT3 DELETE")
        assert "deleted" in output.lower()
        assert "TSTWPT3" not in runtime.waypoints.identifiers

    def test_delwpt_accepts_lowercase_name(self, runtime: MiniSky, sim: Simulation) -> None:
        # Regression: delwpt uppercased the name for the existence check but
        # searched wpid with the raw name, raising ValueError for lowercase input
        runtime.waypoints.defwpt("TSTWPT4", 52.0, 4.0, "FIX")
        assert runtime.waypoints.delwpt("tstwpt4").is_ok()
        assert "TSTWPT4" not in runtime.waypoints.identifiers
