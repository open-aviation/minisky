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
    return bool(area_filter.contains(name, np.array([lat]), np.array([lon]), np.array([alt]))[0])


class TestDefineArea:
    def test_define_box_and_has_area(self, area_filter: AreaFilter) -> None:
        result = area_filter.define_area("BOX1", "BOX", [52.0, 4.0, 53.0, 5.0])
        assert result.is_ok()
        assert area_filter.has_area("BOX1")

    def test_unknown_area_absent(self, area_filter: AreaFilter) -> None:
        assert not area_filter.has_area("NOPE")

    def test_checkinside_unknown_area_returns_false(self, area_filter: AreaFilter) -> None:
        result = area_filter.contains("NOPE", np.array([52.0]), np.array([4.0]), np.array([0.0]))
        assert not result.any()

    def test_reset_clears_areas(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("TMP", "BOX", [52.0, 4.0, 53.0, 5.0])
        area_filter.reset()
        assert not area_filter.has_area("TMP")

    def test_unknown_shape_type_is_err(self, area_filter: AreaFilter) -> None:
        result = area_filter.define_area("X", "BLOB", [52.0, 4.0, 53.0, 5.0])
        assert result.is_err()
        assert not area_filter.has_area("X")


class TestTracking:
    def test_each_shape_type_tracked(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("B", "BOX", [52.0, 4.0, 53.0, 5.0])
        area_filter.define_area("C", "CIRCLE", [52.0, 4.0, 50.0])
        area_filter.define_area("P", "POLY", [52.0, 4.0, 53.0, 4.0, 52.5, 5.0])
        area_filter.define_area("L", "LINE", [52.0, 4.0, 53.0, 5.0])
        for name in ("B", "C", "P", "L"):
            assert area_filter.has_area(name)

    def test_list_reports_defined_shapes(self, area_filter: AreaFilter) -> None:
        result = area_filter.define_area("LIST", "BOX", [])
        assert result.is_ok()
        assert "No shapes" in result.unwrap()

        area_filter.define_area("B1", "BOX", [52.0, 4.0, 53.0, 5.0])
        area_filter.define_area("C1", "CIRCLE", [52.0, 4.0, 50.0])
        listing = area_filter.define_area("LIST", "BOX", []).unwrap()
        assert "B1" in listing
        assert "C1" in listing

    def test_inspect_shape_by_name(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("C1", "CIRCLE", [52.0, 4.0, 50.0])
        result = area_filter.define_area("C1", "CIRCLE", [])
        assert result.is_ok()
        assert "CIRCLE" in result.unwrap()

        assert area_filter.define_area("NOPE", "BOX", []).is_err()

    def test_delete_area(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("TMP", "BOX", [52.0, 4.0, 53.0, 5.0])
        result = area_filter.deleteArea("TMP")
        assert result.is_ok()
        assert not area_filter.has_area("TMP")
        assert not check_single(area_filter, "TMP", 52.5, 4.5)

    def test_delete_unknown_area_is_err(self, area_filter: AreaFilter) -> None:
        assert area_filter.deleteArea("NOPE").is_err()

    def test_redefine_replaces_shape(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("B", "BOX", [52.0, 4.0, 53.0, 5.0])
        assert check_single(area_filter, "B", 52.5, 4.5)

        # Redefine the same name elsewhere; the old geometry must be gone
        area_filter.define_area("B", "BOX", [10.0, 10.0, 11.0, 11.0])
        assert not check_single(area_filter, "B", 52.5, 4.5)
        assert check_single(area_filter, "B", 10.5, 10.5)

    def test_redefine_can_change_shape_type(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("A", "BOX", [52.0, 4.0, 53.0, 5.0])
        area_filter.define_area("A", "CIRCLE", [52.0, 4.0, 50.0])
        assert "CIRCLE" in str(area_filter.basic_shapes["A"])

    def test_delete_leaves_other_shapes(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("B1", "BOX", [52.0, 4.0, 53.0, 5.0])
        area_filter.define_area("B2", "BOX", [10.0, 10.0, 11.0, 11.0])
        area_filter.deleteArea("B1")
        assert not area_filter.has_area("B1")
        assert area_filter.has_area("B2")
        assert check_single(area_filter, "B2", 10.5, 10.5)


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
        inside = area_filter.contains("B", lat, lon, alt)
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

    def test_circle_near_pole(self, area_filter: AreaFilter) -> None:
        # 100 NM radius centred close to the north pole must remain valid
        result = area_filter.define_area("C", "CIRCLE", [89.9, 0.0, 100.0])
        assert result.is_ok()
        assert check_single(area_filter, "C", 89.9, 0.0)
        # 2 deg of latitude south is about 120 NM: outside
        assert not check_single(area_filter, "C", 87.9, 0.0)


class TestPoly:
    def test_triangle_centroid_inside(self, area_filter: AreaFilter) -> None:
        # Triangle (52,4) (53,4) (52.5,5)
        area_filter.define_area("P", "POLY", [52.0, 4.0, 53.0, 4.0, 52.5, 5.0])
        assert check_single(area_filter, "P", 52.5, 4.3)
        assert not check_single(area_filter, "P", 52.5, 5.5)

    def test_antimeridian_polygon_rejected(self, area_filter: AreaFilter) -> None:
        # Quad spanning lon 170 to -170 across the antimeridian
        result = area_filter.define_area(
            "P", "POLY", [10.0, 170.0, 10.0, -170.0, -10.0, -170.0, -10.0, 170.0]
        )
        assert result.is_err()
        assert not area_filter.has_area("P")


class TestLine:
    def test_line_has_no_inside(self, area_filter: AreaFilter) -> None:
        area_filter.define_area("L", "LINE", [52.0, 4.0, 53.0, 5.0])
        assert area_filter.has_area("L")
        # A point exactly on the line is still not "inside" it
        assert not check_single(area_filter, "L", 52.5, 4.5)
