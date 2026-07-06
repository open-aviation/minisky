"""Integration tests for the navigation database (defwpt/delwpt).

These need an initialized simulator: defwpt/delwpt update the screen
singleton (minisky.scr), and sim.reset() reloads the navdatabase.
"""


class TestDefwpt:
    def test_defwpt_adds_waypoint(self, bs, sim):
        navdb = bs.navdb
        n = len(navdb.wpid)
        ok, msg = navdb.defwpt("TSTWPT1", 52.0, 4.0, "FIX")
        assert ok
        assert "TSTWPT1" in msg
        assert len(navdb.wpid) == n + 1
        assert len(navdb.wplat) == n + 1
        assert len(navdb.wplon) == n + 1
        idx = navdb.wpid.index("TSTWPT1")
        assert navdb.wplat[idx] == 52.0
        assert navdb.wplon[idx] == 4.0

    def test_delwpt_removes_coordinates(self, bs, sim):
        # Regression: delwpt discarded the result of np.delete, so
        # wplat/wplon kept the deleted waypoint's coordinates
        navdb = bs.navdb
        n = len(navdb.wpid)
        navdb.defwpt("TSTWPTA", 52.0, 4.0, "FIX")
        navdb.defwpt("TSTWPTB", 10.0, 20.0, "FIX")

        ok, _ = navdb.delwpt("TSTWPTA")
        assert ok
        assert "TSTWPTA" not in navdb.wpid
        assert len(navdb.wpid) == n + 1
        assert len(navdb.wplat) == n + 1
        assert len(navdb.wplon) == n + 1
        # Remaining waypoint's coordinates must still be index-aligned
        idx = navdb.wpid.index("TSTWPTB")
        assert navdb.wplat[idx] == 10.0
        assert navdb.wplon[idx] == 20.0

    def test_defwpt_delete_via_lon_delete_keyword(self, bs, sim):
        # Regression: `lon.upper == "DELETE"` (missing call parentheses)
        # made deletion via the DELETE keyword silently impossible
        navdb = bs.navdb
        n = len(navdb.wpid)
        navdb.defwpt("TSTWPT2", 52.0, 4.0)

        ok, msg = navdb.defwpt("TSTWPT2", 0.0, "delete")
        assert ok
        assert "deleted" in msg
        assert "TSTWPT2" not in navdb.wpid
        assert len(navdb.wpid) == n
        assert len(navdb.wplat) == n
        assert len(navdb.wplon) == n

    def test_defwpt_delete_via_wptype_del(self, bs, sim):
        navdb = bs.navdb
        navdb.defwpt("TSTWPT3", 52.0, 4.0)
        ok, msg = navdb.defwpt("TSTWPT3", 52.0, 4.0, "DEL")
        assert ok
        assert "TSTWPT3" not in navdb.wpid

    def test_delwpt_accepts_lowercase_name(self, bs, sim):
        # Regression: delwpt uppercased the name for the existence check but
        # searched wpid with the raw name, raising ValueError for lowercase input
        navdb = bs.navdb
        navdb.defwpt("TSTWPT4", 52.0, 4.0, "FIX")
        ok, _ = navdb.delwpt("tstwpt4")
        assert ok
        assert "TSTWPT4" not in navdb.wpid
