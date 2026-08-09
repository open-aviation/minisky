"""Area filter module

Defines named geometric shapes - boxes, circles, polygons, and lines - on
the map, optionally bounded by a top and bottom altitude, and provides
point-inside-shape tests for (vectors of) aircraft positions. This backs
the BOX, CIRCLE, POLY, POLYALT, LINE, and POLYLINE stack commands, and is
used by plugins and traffic logic that need to know which aircraft are
inside an area. Each `AreaFilter` stores its defined shapes by name.

Boxes and circles are tested directly in geographic coordinates; polygons
use a planar shapely geometry internally, which is only valid for polygons
that do not cross the antimeridian or enclose a pole - those are rejected
at definition time.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np
import shapely

from minisky.command import AltM, Keyword, LatLonDeg, LatLonDegrees, command
from minisky.result import Err, Ok, Result
from minisky.tools.geo import kwikdist


class AreaFilter:
    """Named geometric shapes for a MiniSky runtime."""

    def __init__(self) -> None:
        # Dictionary of all basic shapes (The shape classes defined in this file) by name
        self.basic_shapes: dict[str, HasArea | Line] = {}

    def has_area(self, areaname: str) -> bool:
        """Check if area with name 'areaname' exists."""
        return areaname in self.basic_shapes

    def define_area(
        self,
        areaname: str,
        areatype: str,
        coordinates: tuple[float, ...] | list[float],
        top: float = 1e9,
        bottom: float = -1e9,
    ) -> Result[str, str]:
        """Define a new area, or list/inspect existing areas.

        Args:
            areaname: Name of the area, or "LIST" to list all defined shapes.
            areatype: Shape type: "BOX", "CIRCLE", "POLY"/"POLYALT", or "LINE".
            coordinates: Flat sequence of lat/lon pairs [deg]; for a circle:
                (lat [deg], lon [deg], radius [nm]). When empty, information
                about the existing area with the given name is returned.
            top: Top altitude bound [m] (default: effectively unbounded).
            bottom: Bottom altitude bound [m] (default: effectively unbounded).
        """
        if areaname == "LIST":
            if not self.basic_shapes:
                return Ok("No shapes are currently defined.")
            else:
                return Ok("Currently defined shapes:\n" + ", ".join(self.basic_shapes))
        if not coordinates:
            if areaname in self.basic_shapes:
                return Ok(str(self.basic_shapes[areaname]))
            else:
                return Err(f"Unknown shape: {areaname}")

        try:
            if areatype == "BOX":
                shape = Box(areaname, coordinates, top, bottom)
            elif areatype == "CIRCLE":
                shape = Circle(areaname, coordinates, top, bottom)
            elif areatype[:4] == "POLY":
                shape = Poly(areaname, coordinates, top, bottom)
            elif areatype == "LINE":
                shape = Line(areaname, coordinates)
            else:
                return Err(f"Unknown shape type: {areatype}")
        except ValueError as e:
            return Err(str(e))

        self.basic_shapes[areaname] = shape
        return Ok(f"Created {areatype} {areaname}")

    @staticmethod
    def _coordinates(points: tuple[LatLonDegrees, ...]) -> tuple[float, ...]:
        return tuple(value for point in points for value in (point.lat, point.lon))

    @command(name="BOX")
    def define_box_area(
        self,
        name: Keyword,
        first: LatLonDeg,
        second: LatLonDeg,
        top: AltM = 1e9,
        bottom: AltM = -1e9,
    ) -> Result[str, str]:
        """Define a box-shaped area from two opposite corners."""
        return self.define_area(name, "BOX", self._coordinates((first, second)), top, bottom)

    @command(name="CIRCLE")
    def define_circle_area(
        self,
        name: Keyword,
        center: LatLonDeg,
        radius: float,
        top: AltM = 1e9,
        bottom: AltM = -1e9,
    ) -> Result[str, str]:
        """Define a circular area from a center and radius in nautical miles."""
        return self.define_area(
            name, "CIRCLE", (*self._coordinates((center,)), radius), top, bottom
        )

    @command(name="LINE")
    def define_line_area(self, name: Keyword, start: LatLonDeg, end: LatLonDeg) -> Result[str, str]:
        """Draw a line between two positions."""
        return self.define_area(name, "LINE", self._coordinates((start, end)))

    @command(name="POLY", aliases=("POLYGON",))
    def define_poly_area(self, name: Keyword, *points: LatLonDeg) -> Result[str, str]:
        """Define a polygon from position vertices."""
        return self.define_area(name, "POLY", self._coordinates(points))

    @command(name="POLYALT")
    def define_polyalt_area(
        self,
        name: Keyword,
        top: AltM,
        bottom: AltM,
        first: LatLonDeg,
        *additional: LatLonDeg,
    ) -> Result[str, str]:
        """Define a polygon between top and bottom altitudes."""
        return self.define_area(
            name, "POLYALT", self._coordinates((first, *additional)), top, bottom
        )

    @command(name="POLYLINE", aliases=("LINES", "POLYLINES"))
    def define_polyline_area(
        self, name: Keyword, first: LatLonDeg, *additional: LatLonDeg
    ) -> Result[str, str]:
        """Draw a multi-segment line through position vertices."""
        return self.define_area(name, "LINE", self._coordinates((first, *additional)))

    def contains(
        self, areaname: str, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray
    ) -> np.ndarray:
        """Check if points with coordinates lat, lon, alt are inside area with name 'areaname'.

        Args:
            areaname: Name of the area to test against.
            lat: Latitude(s) [deg].
            lon: Longitude(s) [deg].
            alt: Altitude(s) [m].

        Returns:
            Array of booleans, True == Inside. All False when no area with
            the given name exists, or when the named shape is a line.
        """
        area = self.basic_shapes.get(areaname)
        if area is None or isinstance(area, Line):
            return np.zeros(len(lat), dtype=bool)
        return area.contains(lat, lon, alt)

    def reset(self) -> None:
        """Clear all data."""
        self.basic_shapes.clear()

    def deleteArea(self, name: str) -> Result[str, str]:
        """Delete a previously defined area by name.

        Args:
            name: Name of the area shape to remove.
        """
        if self.basic_shapes.pop(name, None) is not None:
            return Ok(f"Area {name} deleted.")
        return Err(f"No area found with name {name}.")


def _vrange_str(top: float, bottom: float) -> str:
    """Describe an altitude range [m] for shape __str__ output."""
    if top < 9e8:
        if bottom > -9e8:
            return f" with altitude between {bottom} and {top}"
        else:
            return f" with altitude below {top}"
    if bottom > -9e8:
        return f" with altitude above {bottom}"
    return ""


class HasArea(Protocol):
    """An area shape that supports point-inside tests.

    Lines are deliberately not part of this protocol: a line has zero area,
    so asking whether an aircraft is inside one is a category error.
    """

    def contains(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
        """Return whether points (lat [deg], lon [deg], alt [m]) lie inside
        this shape's geometry and altitude bounds."""
        ...


class Line:
    """A line shape between two lat/lon positions [deg].

    Purely graphical: a line has no inside, and no contains().
    """

    def __init__(self, name: str, coordinates) -> None:
        self.name = name
        self.coordinates = coordinates

    def __str__(self) -> str:
        return (
            f"{self.name} is a LINE with "
            f"start point ({self.coordinates[0]}, {self.coordinates[1]}), "
            f"and end point ({self.coordinates[2]}, {self.coordinates[3]})."
        )


class Box(HasArea):
    """A lat/lon-aligned box shape.

    Defined by two opposite corner points [deg] (sorted at construction)
    and optional altitude bounds [m].
    """

    def __init__(self, name: str, coordinates, top: float = 1e9, bottom: float = -1e9) -> None:
        self.name = name
        self.coordinates = coordinates
        self.top = np.maximum(bottom, top)
        self.bottom = np.minimum(bottom, top)
        # Sort the order of the corner points
        self.lat0 = min(coordinates[0], coordinates[2])
        self.lon0 = min(coordinates[1], coordinates[3])
        self.lat1 = max(coordinates[0], coordinates[2])
        self.lon1 = max(coordinates[1], coordinates[3])

    def contains(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
        """Return whether points (lat [deg], lon [deg], alt [m]) lie inside this box."""
        return (
            ((self.lat0 <= lat) & (lat <= self.lat1))
            & ((self.lon0 <= lon) & (lon <= self.lon1))
            & ((self.bottom <= alt) & (alt <= self.top))
        )

    def __str__(self) -> str:
        return (
            f"{self.name} is a BOX with coordinates "
            + ", ".join(str(c) for c in self.coordinates)
            + _vrange_str(self.top, self.bottom)
        )


class Circle(HasArea):
    """A circle shape.

    Defined by a center position [deg], a radius [nm], and optional
    altitude bounds [m].
    """

    def __init__(self, name: str, coordinates, top: float = 1e9, bottom: float = -1e9) -> None:
        self.name = name
        self.coordinates = coordinates
        self.top = np.maximum(bottom, top)
        self.bottom = np.minimum(bottom, top)
        self.clat = coordinates[0]
        self.clon = coordinates[1]
        self.r = coordinates[2]

    def contains(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
        """Return whether points (lat [deg], lon [deg], alt [m]) lie within
        the circle radius [nm] and altitude bounds."""
        distance = kwikdist(self.clat, self.clon, lat, lon)  # [NM]
        return (distance <= self.r) & (self.bottom <= alt) & (alt <= self.top)

    def __str__(self) -> str:
        return (
            f"{self.name} is a CIRCLE with "
            f"center ({self.clat}, {self.clon}) "
            f"and radius {self.r}." + _vrange_str(self.top, self.bottom)
        )


class Poly(HasArea):
    """A polygon shape.

    Defined by a sequence of lat/lon vertices [deg] and optional altitude
    bounds [m]. Containment is tested against a planar shapely polygon in
    (lat, lon) space, which cannot represent polygons that cross the
    antimeridian or enclose a pole; those are rejected with ValueError.
    """

    def __init__(self, name: str, coordinates, top: float = 1e9, bottom: float = -1e9) -> None:
        self.name = name
        self.coordinates = coordinates
        self.top = np.maximum(bottom, top)
        self.bottom = np.minimum(bottom, top)
        vertices = np.reshape(coordinates, (-1, 2))
        # A ring with an edge spanning more than 180 deg of longitude either
        # crosses the antimeridian or winds around a pole; both are invalid
        # in the planar (lat, lon) space the containment test runs in.
        lons = np.append(vertices[:, 1], vertices[0, 1])
        if np.any(np.abs(np.diff(lons)) > 180.0):
            raise ValueError(
                f"Polygon {name} crosses the antimeridian or encloses a pole; "
                "split it into separate polygons."
            )
        self._geom = shapely.Polygon(vertices)

    def contains(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
        """Return whether points (lat [deg], lon [deg], alt [m]) lie inside
        the polygon border and altitude bounds."""
        return shapely.contains_xy(self._geom, lat, lon) & (self.bottom <= alt) & (alt <= self.top)

    def __str__(self) -> str:
        return (
            f"{self.name} is a POLY with coordinates "
            + ", ".join(str(c) for c in self.coordinates)
            + _vrange_str(self.top, self.bottom)
        )
