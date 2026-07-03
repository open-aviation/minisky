"""Unit tests for minisky.tools.geo (bearings, distances, projections).

Units: qdrdist/kwikdist return distance in nautical miles,
latlondist returns meters, rwgs84 returns meters.
"""

import pytest

from minisky.tools import geo

NM_IN_M = 1852.0


class TestQdrDist:
    def test_eastbound_along_equator(self):
        qdr, dist = geo.qdrdist(0.0, 0.0, 0.0, 1.0)
        assert qdr == pytest.approx(90.0, abs=0.1)
        assert dist == pytest.approx(60.1, abs=0.2)  # 1 deg lon at equator

    def test_northbound_along_meridian(self):
        qdr, dist = geo.qdrdist(0.0, 0.0, 1.0, 0.0)
        assert qdr == pytest.approx(0.0, abs=0.1)
        assert dist == pytest.approx(60.1, abs=0.5)

    def test_reciprocal_bearing(self):
        qdr_fwd, dist_fwd = geo.qdrdist(52.0, 4.0, 53.0, 5.0)
        qdr_rev, dist_rev = geo.qdrdist(53.0, 5.0, 52.0, 4.0)
        assert dist_fwd == pytest.approx(dist_rev, rel=1e-6)
        assert (qdr_rev - qdr_fwd) % 360.0 == pytest.approx(180.0, abs=1.0)

    def test_zero_distance_same_point(self):
        _, dist = geo.qdrdist(52.0, 4.0, 52.0, 4.0)
        assert dist == pytest.approx(0.0, abs=1e-6)


class TestDistanceFunctions:
    def test_latlondist_matches_qdrdist(self):
        _, dist_nm = geo.qdrdist(52.0, 4.0, 52.5, 4.5)
        dist_m = geo.latlondist(52.0, 4.0, 52.5, 4.5)
        assert dist_m / NM_IN_M == pytest.approx(dist_nm, rel=1e-3)

    def test_kwikdist_approximates_latlondist(self):
        # kwikdist is a fast flat-earth approximation, good at short range
        dist_kwik_nm = geo.kwikdist(52.0, 4.0, 52.1, 4.1)
        dist_m = geo.latlondist(52.0, 4.0, 52.1, 4.1)
        assert dist_kwik_nm == pytest.approx(dist_m / NM_IN_M, rel=0.01)

    def test_kwikqdrdist_approximates_qdrdist(self):
        qdr, dist = geo.qdrdist(52.0, 4.0, 52.1, 4.1)
        kqdr, kdist = geo.kwikqdrdist(52.0, 4.0, 52.1, 4.1)
        assert kqdr == pytest.approx(qdr, abs=1.0)
        assert kdist == pytest.approx(dist, rel=0.01)


class TestProjection:
    @pytest.mark.parametrize("qdr,dist", [(0.0, 60.0), (45.0, 100.0), (270.0, 30.0)])
    def test_qdrpos_roundtrip(self, qdr, dist):
        lat2, lon2 = geo.qdrpos(52.0, 4.0, qdr, dist)
        qdr_back, dist_back = geo.qdrdist(52.0, 4.0, lat2, lon2)
        assert qdr_back % 360.0 == pytest.approx(qdr % 360.0, abs=0.5)
        assert dist_back == pytest.approx(dist, rel=1e-3)

    def test_qdrpos_north_increases_latitude(self):
        lat2, lon2 = geo.qdrpos(52.0, 4.0, 0.0, 60.0)
        assert lat2 > 52.0
        assert lon2 == pytest.approx(4.0, abs=1e-6)


class TestWgs84:
    def test_equatorial_radius(self):
        assert geo.rwgs84(0.0) == pytest.approx(6378137.0, rel=1e-6)

    def test_polar_radius(self):
        assert geo.rwgs84(90.0) == pytest.approx(6356752.3, rel=1e-6)

    def test_radius_within_bounds(self):
        for lat in (10.0, 30.0, 45.0, 60.0, 80.0):
            r = geo.rwgs84(lat)
            assert 6356752.0 < r < 6378138.0
