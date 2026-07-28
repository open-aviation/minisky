"""Unit tests for minisky.tools.areafilter (geometric areas).

Coordinates are flat [lat, lon, ...] lists; circle radius is in NM;
altitudes in meters (default vertical range is unbounded).
"""

import numpy as np
import pytest

from minisky.tools.areafilter import AreaFilter


@pytest.fixture
def area_filter() -> AreaFilter:
    return AreaFilter()


def check_single(
    area_filter: AreaFilter, name: str, lat: float, lon: float, alt: float = 0.0
) -> bool:
    return bool(area_filter.checkInside(name, np.array([lat]), np.array([lon]), np.array([alt]))[0])


class TestDefineArea:
    def test_define_box_and_has_area(self, area_filter: AreaFilter) -> None:
        ok, msg = area_filter.define_area("BOX1", "BOX", [52.0, 4.0, 53.0, 5.0])
        assert ok
        assert area_filter.has_area("BOX1")

    def test_unknown_area_absent(self, area_filter: AreaFilter) -> None:
        assert not area_filter.has_area("NOPE")

    def test_checkinside_unknown_area_returns_false(self, area_filter: AreaFilter) -> None:
        result = area_filter.checkInside("NOPE", np.array([52.0]), np.array([4.0]), np.array([0.0]))
        assert not result.any()

    def test_reset_clears_areas(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("TMP", "BOX", [52.0, 4.0, 53.0, 5.0])
        area_filter.reset()
        assert not area_filter.has_area("TMP")


class TestBox:
    def test_inside_and_outside(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("B", "BOX", [52.0, 4.0, 53.0, 5.0])
        assert check_single(area_filter, "B", 52.5, 4.5)
        assert not check_single(area_filter, "B", 51.0, 4.5)
        assert not check_single(area_filter, "B", 52.5, 6.0)

    def test_altitude_bounds(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("B", "BOX", [52.0, 4.0, 53.0, 5.0], top=3000.0, bottom=1000.0)
        assert check_single(area_filter, "B", 52.5, 4.5, alt=2000.0)
        assert not check_single(area_filter, "B", 52.5, 4.5, alt=500.0)
        assert not check_single(area_filter, "B", 52.5, 4.5, alt=5000.0)

    def test_array_input(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("B", "BOX", [52.0, 4.0, 53.0, 5.0])
        lat = np.array([52.5, 51.0, 52.9])
        lon = np.array([4.5, 4.5, 4.1])
        alt = np.zeros(3)
        inside = area_filter.checkInside("B", lat, lon, alt)
        assert inside.tolist() == [True, False, True]


class TestCircle:
    def test_center_inside_far_point_outside(self, area_filter: AreaFilter) -> None:
        # 50 NM radius around (52, 4)
        area_filter.define_area("C", "CIRCLE", [52.0, 4.0, 50.0])
        assert check_single(area_filter, "C", 52.0, 4.0)
        # ~0.5 deg lat is about 30 NM: inside
        assert check_single(area_filter, "C", 52.5, 4.0)
        # 2 deg lat is about 120 NM: outside
        assert not check_single(area_filter, "C", 54.0, 4.0)


class TestPoly:
    def test_triangle_centroid_inside(self, area_filter: AreaFilter) -> None:
        # Triangle (52,4) (53,4) (52.5,5)
        area_filter.define_area("P", "POLY", [52.0, 4.0, 53.0, 4.0, 52.5, 5.0])
        assert check_single(area_filter, "P", 52.5, 4.3)
        assert not check_single(area_filter, "P", 52.5, 5.5)
