"""Named geographic areas and graphical lines."""

from __future__ import annotations

from collections.abc import Mapping
from types import MappingProxyType
from typing import Protocol

import numpy as np
import shapely

from minisky.command import AltM, Keyword, LatLonDeg, LatLonDegrees, command
from minisky.result import Err, Ok, Result
from minisky.tools.geo import kwikdist


class Shapes:
    """Own named containable areas and graphical lines."""

    def __init__(self) -> None:
        self._areas: dict[str, HasArea] = {}
        self._lines: dict[str, Line] = {}
        self.areas: Mapping[str, HasArea] = MappingProxyType(self._areas)
        self.lines: Mapping[str, Line] = MappingProxyType(self._lines)

    # NOTE: we want area and line names to be mutually exclusive.

    def _store_area(self, name: str, area: HasArea) -> None:
        self._lines.pop(name, None)
        self._areas[name] = area

    def _store_line(self, name: str, line: Line) -> None:
        self._areas.pop(name, None)
        self._lines[name] = line

    @command(name="BOX")
    def define_box(
        self,
        name: Keyword,
        first: LatLonDeg,
        second: LatLonDeg,
        top: AltM = 1e9,
        bottom: AltM = -1e9,
    ) -> Result[str, str]:
        """Define a box-shaped area from two opposite corners."""
        self._store_area(name, Box(first, second, top, bottom))
        return Ok(f"Created BOX {name}")

    @command(name="CIRCLE")
    def define_circle(
        self,
        name: Keyword,
        center: LatLonDeg,
        radius: float,
        top: AltM = 1e9,
        bottom: AltM = -1e9,
    ) -> Result[str, str]:
        """Define a circular area from a center and radius in nautical miles."""
        self._store_area(name, Circle(center, radius, top, bottom))
        return Ok(f"Created CIRCLE {name}")

    @command(name="LINE")
    def define_line(self, name: Keyword, start: LatLonDeg, end: LatLonDeg) -> Result[str, str]:
        """Draw a line between two positions."""
        self._store_line(name, Line((start, end)))
        return Ok(f"Created LINE {name}")

    @command(name="POLY", aliases=("POLYGON",))
    def define_poly(self, name: Keyword, *points: LatLonDeg) -> Result[str, str]:
        """Define a polygon from position vertices."""
        try:
            area = Poly(points)
        except ValueError as error:
            return Err(str(error))
        self._store_area(name, area)
        return Ok(f"Created POLY {name}")

    @command(name="POLYALT")
    def define_polyalt(
        self,
        name: Keyword,
        top: AltM,
        bottom: AltM,
        first: LatLonDeg,
        *additional: LatLonDeg,
    ) -> Result[str, str]:
        """Define a polygon between top and bottom altitudes."""
        try:
            area = Poly((first, *additional), top, bottom)
        except ValueError as error:
            return Err(str(error))
        self._store_area(name, area)
        return Ok(f"Created POLYALT {name}")

    @command(name="POLYLINE", aliases=("LINES", "POLYLINES"))
    def define_polyline(
        self, name: Keyword, first: LatLonDeg, *additional: LatLonDeg
    ) -> Result[str, str]:
        """Draw a multi-segment line through position vertices."""
        self._store_line(name, Line((first, *additional)))
        return Ok(f"Created POLYLINE {name}")

    def reset(self) -> None:
        self._areas.clear()
        self._lines.clear()

    def delete(self, name: str) -> Result[str, str]:
        if self._areas.pop(name, None) is not None:
            return Ok(f"Area {name} deleted.")
        if self._lines.pop(name, None) is not None:
            return Ok(f"Line {name} deleted.")
        return Err(f"No shape found with name {name}.")


class HasArea(Protocol):
    """An area shape that supports point-inside tests.

    Lines are deliberately not part of this protocol: a line has zero area,
    so asking whether an aircraft is inside one is a category error.
    """

    def contains(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
        """Return whether points (lat [deg], lon [deg], alt [m]) lie inside
        this shape's geometry and altitude bounds."""
        ...


# NOTE(abraham): it seems like no one is using line so we should consider
# removing it. maybe it came from the old GUI that we removed?
class Line:
    """A graphical line through latitude/longitude positions."""

    def __init__(self, points: tuple[LatLonDegrees, ...]) -> None:
        self.points = points


# TODO(abraham): stop using sentinels to indicate the lack of top / bottom.
# see issue #40


class Box(HasArea):
    """A lat/lon-aligned box shape.

    Defined by two opposite corner points [deg] and optional altitude bounds [m].
    """

    def __init__(
        self, first: LatLonDegrees, second: LatLonDegrees, top: float = 1e9, bottom: float = -1e9
    ) -> None:
        self.top = np.maximum(bottom, top)
        self.bottom = np.minimum(bottom, top)
        self.lat0 = min(first.lat, second.lat)
        self.lon0 = min(first.lon, second.lon)
        self.lat1 = max(first.lat, second.lat)
        self.lon1 = max(first.lon, second.lon)

    def contains(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
        """Return whether points (lat [deg], lon [deg], alt [m]) lie inside this box."""
        return (
            ((self.lat0 <= lat) & (lat <= self.lat1))
            & ((self.lon0 <= lon) & (lon <= self.lon1))
            & ((self.bottom <= alt) & (alt <= self.top))
        )


class Circle(HasArea):
    """A circle shape.

    Defined by a center position [deg], a radius [nm], and optional
    altitude bounds [m].
    """

    def __init__(
        self, center: LatLonDegrees, radius: float, top: float = 1e9, bottom: float = -1e9
    ) -> None:
        self.center = center
        self.radius = radius
        self.top = np.maximum(bottom, top)
        self.bottom = np.minimum(bottom, top)

    def contains(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
        """Return whether points (lat [deg], lon [deg], alt [m]) lie within
        the circle radius [nm] and altitude bounds."""
        distance = kwikdist(self.center.lat, self.center.lon, lat, lon)  # [NM]
        return (distance <= self.radius) & (self.bottom <= alt) & (alt <= self.top)


class Poly(HasArea):
    """A polygon shape.

    Defined by a sequence of lat/lon vertices [deg] and optional altitude
    bounds [m]. Longitudes are unwrapped before planar Shapely containment;
    polygons that touch or enclose a pole are rejected.
    """

    def __init__(
        self, points: tuple[LatLonDegrees, ...], top: float = 1e9, bottom: float = -1e9
    ) -> None:
        self.top = np.maximum(bottom, top)
        self.bottom = np.minimum(bottom, top)
        vertices = np.asarray([(point.lat, point.lon) for point in points], dtype=float)
        if len(vertices) < 3:
            raise ValueError("Polygon requires at least three vertices")
        if np.any(np.isclose(np.abs(vertices[:, 0]), 90.0)):
            raise ValueError("Polygon must not touch a pole")

        lon = vertices[:, 1]
        unwrapped_ring = np.unwrap(np.append(lon, lon[0]), period=360.0)
        if not np.isclose(unwrapped_ring[-1], unwrapped_ring[0]):
            raise ValueError("Polygon must not enclose a pole")
        unwrapped_lon = unwrapped_ring[:-1]
        self._reference_lon = float((unwrapped_lon.min() + unwrapped_lon.max()) * 0.5)
        self._geom = shapely.Polygon(np.column_stack((vertices[:, 0], unwrapped_lon)))
        if not shapely.is_valid(self._geom):
            raise ValueError(f"Invalid polygon: {shapely.is_valid_reason(self._geom)}")

    def contains(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
        """Return whether points (lat [deg], lon [deg], alt [m]) lie inside
        the polygon border and altitude bounds."""
        unwrapped_lon = lon + 360.0 * np.floor((self._reference_lon - lon + 180.0) / 360.0)
        return (
            shapely.contains_xy(self._geom, lat, unwrapped_lon)
            & (self.bottom <= alt)
            & (alt <= self.top)
        )
