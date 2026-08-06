"""Area filter module

Defines named geometric shapes - boxes, circles, polygons, and lines - on
the map, optionally bounded by a top and bottom altitude, and provides
point-inside-shape tests for (vectors of) aircraft positions. This backs
the BOX, CIRCLE, POLY, POLYALT, LINE, and POLYLINE stack commands, and is
used by plugins and traffic logic that need to know which aircraft are
inside an area. Each `AreaFilter` stores its defined shapes by name.
"""

from __future__ import annotations

import numpy as np
from matplotlib.path import Path

from minisky.command import AltM, Keyword, LatLonDeg, LatLonDegrees, command
from minisky.result import Err, Ok, Result
from minisky.tools.geo import kwikdist


class AreaFilter:
    """Named geometric shapes for a MiniSky runtime."""

    def __init__(self) -> None:
        # Dictionary of all basic shapes (The shape classes defined in this file) by name
        self.basic_shapes: dict[str, Shape] = {}

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
        """BOX: Define a box-shaped area.

        Args:
            name: Area name.
            first: First corner position.
            second: Opposite corner position.
            top: Top altitude bound [m].
            bottom: Bottom altitude bound [m].
        """
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
        """CIRCLE: Define a circle-shaped area.

        Args:
            name: Area name.
            center: Center position.
            radius: Radius [nm].
            top: Top altitude bound [m].
            bottom: Bottom altitude bound [m].
        """
        return self.define_area(
            name, "CIRCLE", (*self._coordinates((center,)), radius), top, bottom
        )

    @command(name="LINE")
    def define_line_area(self, name: Keyword, start: LatLonDeg, end: LatLonDeg) -> Result[str, str]:
        """LINE: Draw a line between two positions on the radar screen.

        Args:
            name: Line name.
            start: First line position.
            end: Second line position.
        """
        return self.define_area(name, "LINE", self._coordinates((start, end)))

    @command(name="POLY", aliases=("POLYGON",))
    def define_poly_area(self, name: Keyword, *points: LatLonDeg) -> Result[str, str]:
        """POLY: Define a polygon-shaped area.

        Args:
            name: Area name.
            *points: Polygon vertex positions.
        """
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
        """POLYALT: Define a polygon-shaped area in 3D, between two altitudes.

        Args:
            name: Area name.
            top: Top altitude bound [m].
            bottom: Bottom altitude bound [m].
            first: First polygon vertex.
            *additional: Additional polygon vertices.
        """
        return self.define_area(
            name, "POLYALT", self._coordinates((first, *additional)), top, bottom
        )

    @command(name="POLYLINE", aliases=("LINES", "POLYLINES"))
    def define_polyline_area(
        self, name: Keyword, first: LatLonDeg, *additional: LatLonDeg
    ) -> Result[str, str]:
        """POLYLINE: Draw a multi-segment line on the radar screen.

        Args:
            name: Line name.
            first: First line position.
            *additional: Additional line positions.
        """
        return self.define_area(name, "LINE", self._coordinates((first, *additional)))

    def checkInside(
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
            the given name exists.
        """
        if areaname not in self.basic_shapes:
            return np.zeros(len(lat), dtype=bool)
        area = self.basic_shapes[areaname]
        return area.checkInside(lat, lon, alt)

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


class Shape:
    """
    Base class of BlueSky shapes

    Handles the naming and altitude bounds common to all shape types.
    Derived classes implement checkInside() for their specific geometry.

    Attributes:
        name: Area name.
        coordinates: Flat list of lat/lon coordinates in deg defining the
            shape (plus radius in nm for circles).
        top: Upper altitude bound [m].
        bottom: Lower altitude bound [m].
        raw: Dictionary with the raw shape definition (name, kind,
            coordinates).
    """

    def __init__(self, name: str, coordinates, top: float = 1e9, bottom: float = -1e9) -> None:
        self.raw = {"name": name, "shape": self.kind(), "coordinates": coordinates}
        self.name = name
        self.coordinates = coordinates
        self.top = np.maximum(bottom, top)
        self.bottom = np.minimum(bottom, top)

    def checkInside(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
        """Returns True (or boolean array) if coordinate lat, lon, alt lies
        within this shape.

        Reimplement this function in the derived shape classes for this to
        work.
        """
        return np.zeros(len(lat), dtype=bool)

    def _str_vrange(self) -> str:
        if self.top < 9e8:
            if self.bottom > -9e8:
                return f" with altitude between {self.bottom} and {self.top}"
            else:
                return f" with altitude below {self.top}"
        if self.bottom > -9e8:
            return f" with altitude above {self.bottom}"
        return ""

    def __str__(self) -> str:
        return (
            f"{self.name} is a {self.raw['shape']} with coordinates "
            + ", ".join(str(c) for c in self.coordinates)
            + self._str_vrange()
        )

    @classmethod
    def kind(cls) -> str:
        """Return a string describing what kind of shape this is."""
        return cls.__name__.upper()


class Line(Shape):
    """A line shape between two lat/lon positions [deg].

    Purely graphical: the inherited checkInside() always returns False.
    """

    def __init__(self, name: str, coordinates) -> None:
        super().__init__(name, coordinates)

    def __str__(self) -> str:
        return (
            f"{self.name} is a LINE with "
            f"start point ({self.coordinates[0]}, {self.coordinates[1]}), "
            f"and end point ({self.coordinates[2]}, {self.coordinates[3]})."
        )


class Box(Shape):
    """A lat/lon-aligned box shape.

    Defined by two opposite corner points [deg] (sorted at construction)
    and optional altitude bounds [m].
    """

    def __init__(self, name: str, coordinates, top: float = 1e9, bottom: float = -1e9) -> None:
        super().__init__(name, coordinates, top, bottom)
        # Sort the order of the corner points
        self.lat0 = min(coordinates[0], coordinates[2])
        self.lon0 = min(coordinates[1], coordinates[3])
        self.lat1 = max(coordinates[0], coordinates[2])
        self.lon1 = max(coordinates[1], coordinates[3])

    def checkInside(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray):
        """Return whether points (lat [deg], lon [deg], alt [m]) lie inside this box."""
        return (
            ((self.lat0 <= lat) & (lat <= self.lat1))
            & ((self.lon0 <= lon) & (lon <= self.lon1))
            & ((self.bottom <= alt) & (alt <= self.top))
        )


class Circle(Shape):
    """A circle shape.

    Defined by a center position [deg], a radius [nm], and optional
    altitude bounds [m].
    """

    def __init__(self, name: str, coordinates, top: float = 1e9, bottom: float = -1e9) -> None:
        super().__init__(name, coordinates, top, bottom)
        self.clat = coordinates[0]
        self.clon = coordinates[1]
        self.r = coordinates[2]

    def checkInside(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray):
        """Return whether points (lat [deg], lon [deg], alt [m]) lie within
        the circle radius [nm] and altitude bounds."""
        distance = kwikdist(self.clat, self.clon, lat, lon)  # [NM]
        inside = (distance <= self.r) & (self.bottom <= alt) & (alt <= self.top)
        return inside

    def __str__(self) -> str:
        return (
            f"{self.name} is a CIRCLE with "
            f"center ({self.clat}, {self.clon}) "
            f"and radius {self.r}." + self._str_vrange()
        )


class Poly(Shape):
    """A polygon shape.

    Defined by a sequence of lat/lon vertices [deg] and optional altitude
    bounds [m]; the border is stored as a matplotlib Path for fast
    point-in-polygon tests.
    """

    def __init__(self, name: str, coordinates, top: float = 1e9, bottom: float = -1e9) -> None:
        super().__init__(name, coordinates, top, bottom)
        self.border = Path(np.reshape(coordinates, (len(coordinates) // 2, 2)))

    def checkInside(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray):
        """Return whether points (lat [deg], lon [deg], alt [m]) lie inside
        the polygon border and altitude bounds."""
        points = np.vstack((lat, lon)).T
        inside = np.all(
            (self.border.contains_points(points), self.bottom <= alt, alt <= self.top),
            axis=0,
        )
        return inside
