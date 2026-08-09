"""Integration tests for the navigation database (defwpt/delwpt).

These need an initialized runtime: defwpt/delwpt update its console, and
simulation reset reloads the navigation database.
"""

from __future__ import annotations

from minisky import MiniSky
from minisky.simulation import Simulation


class TestDefwpt:
    def test_defwpt_adds_waypoint(self, runtime: MiniSky, sim: Simulation) -> None:
        navdb = runtime.navigation
        n = len(navdb.wpid)
        result = navdb.defwpt("TSTWPT1", 52.0, 4.0, "FIX")
        assert result.is_ok()
        assert "TSTWPT1" in result.unwrap()
        assert len(navdb.wpid) == n + 1
        assert len(navdb.wplat) == n + 1
        assert len(navdb.wplon) == n + 1
        idx = navdb.wpid.index("TSTWPT1")
        assert navdb.wplat[idx] == 52.0
        assert navdb.wplon[idx] == 4.0

    def test_delwpt_removes_coordinates(self, runtime: MiniSky, sim: Simulation) -> None:
        # Regression: delwpt discarded the result of np.delete, so
        # wplat/wplon kept the deleted waypoint's coordinates
        navdb = runtime.navigation
        n = len(navdb.wpid)
        navdb.defwpt("TSTWPTA", 52.0, 4.0, "FIX")
        navdb.defwpt("TSTWPTB", 10.0, 20.0, "FIX")

        result = navdb.delwpt("TSTWPTA")
        assert result.is_ok()
        assert "TSTWPTA" not in navdb.wpid
        assert len(navdb.wpid) == n + 1
        assert len(navdb.wplat) == n + 1
        assert len(navdb.wplon) == n + 1
        # Remaining waypoint's coordinates must still be index-aligned
        idx = navdb.wpid.index("TSTWPTB")
        assert navdb.wplat[idx] == 10.0
        assert navdb.wplon[idx] == 20.0

    def test_defwpt_delete_via_stack(self, runtime: MiniSky, sim: Simulation, run_cmd) -> None:
        navdb = runtime.navigation
        navdb.defwpt("TSTWPT3", 52.0, 4.0)
        output = run_cmd("DEFWPT TSTWPT3 DELETE")
        assert "deleted" in output.lower()
        assert "TSTWPT3" not in navdb.wpid

    def test_delwpt_accepts_lowercase_name(self, runtime: MiniSky, sim: Simulation) -> None:
        # Regression: delwpt uppercased the name for the existence check but
        # searched wpid with the raw name, raising ValueError for lowercase input
        navdb = runtime.navigation
        navdb.defwpt("TSTWPT4", 52.0, 4.0, "FIX")
        result = navdb.delwpt("tstwpt4")
        assert result.is_ok()
        assert "TSTWPT4" not in navdb.wpid
