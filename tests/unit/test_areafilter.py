"""Unit tests for minisky.tools.areafilter (geometric areas).

Coordinates are flat [lat, lon, ...] lists; circle radius is in NM;
altitudes in meters (default vertical range is unbounded).
"""

import numpy as np
import pytest

from minisky.tools import areafilter


@pytest.fixture(autouse=True)
def clean_shapes():
    areafilter.reset()
    yield
    areafilter.reset()


def check_single(name, lat, lon, alt=0.0):
    return bool(
        areafilter.checkInside(
            name, np.array([lat]), np.array([lon]), np.array([alt])
        )[0]
    )


class TestDefineArea:
    def test_define_box_and_has_area(self):
        ok, msg = areafilter.define_area("BOX1", "BOX", [52.0, 4.0, 53.0, 5.0])
        assert ok
        assert areafilter.has_area("BOX1")

    def test_unknown_area_absent(self):
        assert not areafilter.has_area("NOPE")

    def test_checkinside_unknown_area_returns_false(self):
        result = areafilter.checkInside("NOPE", np.array([52.0]), np.array([4.0]), np.array([0.0]))
        assert not result.any()

    def test_reset_clears_areas(self):
        areafilter.define_area("TMP", "BOX", [52.0, 4.0, 53.0, 5.0])
        areafilter.reset()
        assert not areafilter.has_area("TMP")


class TestBox:
    def test_inside_and_outside(self):
        areafilter.define_area("B", "BOX", [52.0, 4.0, 53.0, 5.0])
        assert check_single("B", 52.5, 4.5)
        assert not check_single("B", 51.0, 4.5)
        assert not check_single("B", 52.5, 6.0)

    def test_altitude_bounds(self):
        areafilter.define_area("B", "BOX", [52.0, 4.0, 53.0, 5.0], top=3000.0, bottom=1000.0)
        assert check_single("B", 52.5, 4.5, alt=2000.0)
        assert not check_single("B", 52.5, 4.5, alt=500.0)
        assert not check_single("B", 52.5, 4.5, alt=5000.0)

    def test_array_input(self):
        areafilter.define_area("B", "BOX", [52.0, 4.0, 53.0, 5.0])
        lat = np.array([52.5, 51.0, 52.9])
        lon = np.array([4.5, 4.5, 4.1])
        alt = np.zeros(3)
        inside = areafilter.checkInside("B", lat, lon, alt)
        assert inside.tolist() == [True, False, True]


class TestCircle:
    def test_center_inside_far_point_outside(self):
        # 50 NM radius around (52, 4)
        areafilter.define_area("C", "CIRCLE", [52.0, 4.0, 50.0])
        assert check_single("C", 52.0, 4.0)
        # ~0.5 deg lat is about 30 NM: inside
        assert check_single("C", 52.5, 4.0)
        # 2 deg lat is about 120 NM: outside
        assert not check_single("C", 54.0, 4.0)


class TestPoly:
    def test_triangle_centroid_inside(self):
        # Triangle (52,4) (53,4) (52.5,5)
        areafilter.define_area("P", "POLY", [52.0, 4.0, 53.0, 4.0, 52.5, 5.0])
        assert check_single("P", 52.5, 4.3)
        assert not check_single("P", 52.5, 5.5)
