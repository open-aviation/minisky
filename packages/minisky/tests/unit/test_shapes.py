"""Unit tests for geographic shape behavior."""

import numpy as np
import pytest
from minisky import quantities as q
from minisky.command import LatLonDegrees
from minisky.tools.shapes import Box, Circle, HasArea, Poly, Shapes


def contains(shape: HasArea, lat: float, lon: float, alt: float = 0.0) -> bool:
    return bool(shape.contains(np.array([lat]), np.array([lon]), np.array([alt]))[0])


def test_mutually_exclusive_shape_names() -> None:
    # the same name should not exist simultaneously in
    # shapes.areas and shapes.lines
    shapes = Shapes()
    first = LatLonDegrees(52.0, 4.0)
    second = LatLonDegrees(53.0, 5.0)

    shapes.define_box("A", first, second)
    assert "A" in shapes.areas
    assert "A" not in shapes.lines

    shapes.define_line("A", first, second)
    assert "A" not in shapes.areas
    assert "A" in shapes.lines

    shapes.define_box("A", first, second)
    assert "A" in shapes.areas
    assert "A" not in shapes.lines


#
# see issue #32 / PR #42
#


def test_box_containment_includes_altitude_bounds() -> None:
    box = Box(LatLonDegrees(52.0, 4.0), LatLonDegrees(53.0, 5.0), top=3000.0, bottom=1000.0)

    assert contains(box, 52.5, 4.5, 2000.0)
    assert not contains(box, 51.0, 4.5, 2000.0)
    assert not contains(box, 52.5, 4.5, 5000.0)


def test_circle_near_pole_uses_geographic_distance() -> None:
    circle = Circle(LatLonDegrees(89.9, 0.0), 100.0)

    assert contains(circle, 89.9, 0.0)
    assert not contains(circle, 87.9, 0.0)


def test_polygon_contains_points_across_antimeridian() -> None:
    polygon = Poly(
        (
            LatLonDegrees(10.0, 170.0),
            LatLonDegrees(10.0, -170.0),
            LatLonDegrees(-10.0, -170.0),
            LatLonDegrees(-10.0, 170.0),
        )
    )

    assert contains(polygon, 0.0, 179.0)
    assert contains(polygon, 0.0, -179.0)
    assert not contains(polygon, 0.0, 0.0)


def test_polygon_enclosing_pole_is_rejected() -> None:
    with pytest.raises(ValueError, match="Polygon must not enclose a pole"):
        Poly(
            (
                LatLonDegrees(80.0, -135.0),
                LatLonDegrees(80.0, -45.0),
                LatLonDegrees(80.0, 45.0),
                LatLonDegrees(80.0, 135.0),
            )
        )
