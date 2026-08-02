"""Area filter module

Defines named geometric shapes - boxes, circles, polygons, and lines - on
the map, optionally bounded by a top and bottom altitude, and provides
point-inside-shape tests for (vectors of) aircraft positions. This backs
the BOX, CIRCLE, POLY, POLYALT, LINE, and POLYLINE stack commands, and is
used by plugins and traffic logic that need to know which aircraft are
inside an area. Each `AreaFilter` stores its defined shapes by name and
indexes them in an R-tree for fast geospatial queries.
"""

from __future__ import annotations

from contextlib import suppress
from weakref import WeakValueDictionary

import numpy as np
from matplotlib.path import Path

from minisky.result import Err, Ok, Result

try:
    from rtree.index import Index  # type: ignore[assignment]
except (ImportError, OSError):
    print(
        "Warning: RTree could not be loaded. areafilter get_intersecting and get_knearest won't work"
    )

    class Index:
        """Dummy index class for installations where rtree is missing
        or doesn't work.
        """

        @staticmethod
        def intersection(*args, **kwargs):
            return []

        @staticmethod
        def nearest(*args, **kwargs):
            return []

        @staticmethod
        def insert(*args, **kwargs):
            return

        @staticmethod
        def delete(*args, **kwargs):
            return


from minisky.tools.geo import kwikdist


class AreaFilter:
    """Named geometric shapes and spatial index for a MiniSky runtime."""

    def __init__(self) -> None:
        # Dictionary of all basic shapes (The shape classes defined in this file) by name
        self.basic_shapes: dict[str, Shape] = {}

        # Counter to keep track of used shape ids
        self.max_area_id = 0

        # Weak-value dictionary of all Shape-derived objects by name, and id
        self.areas_by_id: WeakValueDictionary[int, Shape] = WeakValueDictionary()
        self.areas_by_name: WeakValueDictionary[str, Shape] = WeakValueDictionary()

        # RTree of all areas for efficient geospatial searching
        self.areatree = Index()

    def _register(self, shape: Shape) -> None:
        # Owner-local weak reference and tree storage
        shape.area_id = self.max_area_id
        self.max_area_id += 1
        self.areas_by_id[shape.area_id] = shape
        self.areas_by_name[shape.name] = shape
        self.areatree.insert(shape.area_id, shape.bbox)
        shape._registered = True

    def _unregister(self, shape: Shape) -> None:
        if not shape._registered:
            return
        self.areatree.delete(shape.area_id, shape.bbox)
        self.areas_by_id.pop(shape.area_id, None)
        self.areas_by_name.pop(shape.name, None)
        shape._registered = False

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

        old_shape = self.basic_shapes.get(areaname)
        if old_shape is not None:
            self._unregister(old_shape)

        if areatype == "BOX":
            shape = Box(self, areaname, coordinates, top, bottom)
        elif areatype == "CIRCLE":
            shape = Circle(self, areaname, coordinates, top, bottom)
        elif areatype[:4] == "POLY":
            shape = Poly(self, areaname, coordinates, top, bottom)
        elif areatype == "LINE":
            shape = Line(self, areaname, coordinates)
        else:
            return Err(f"Unknown shape type: {areatype}")

        self.basic_shapes[areaname] = shape
        return Ok(f"Created {areatype} {areaname}")

    def define_box_area(self, name: str, *coords: float) -> Result[str, str]:
        """BOX: Define a box-shaped area.

        Args:
            name: Area name.
            *coords: lat1, lon1, lat2, lon2 [deg] of two opposite corners,
                optionally followed by top and bottom altitude [m].
        """
        return self.define_area(name, "BOX", coords[:4], *coords[4:])

    def define_circle_area(self, name: str, *coords: float) -> Result[str, str]:
        """CIRCLE: Define a circle-shaped area.

        Args:
            name: Area name.
            *coords: lat, lon [deg] of the center and radius [nm], optionally
                followed by top and bottom altitude [m].
        """
        return self.define_area(name, "CIRCLE", coords[:3], *coords[3:])

    def define_line_area(self, name: str, *coords: float) -> Result[str, str]:
        """LINE: Draw a line between two positions on the radar screen.

        Args:
            name: Line name.
            *coords: lat1, lon1, lat2, lon2 [deg] of the two end points.
        """
        return self.define_area(name, "LINE", coords)

    def define_poly_area(self, name: str, *coords: float) -> Result[str, str]:
        """POLY: Define a polygon-shaped area.

        Args:
            name: Area name.
            *coords: lat, lon pairs [deg] of the polygon vertices.
        """
        return self.define_area(name, "POLY", coords)

    def define_polyalt_area(
        self, name: str, top: float, bottom: float, *coords: float
    ) -> Result[str, str]:
        """POLYALT: Define a polygon-shaped area in 3D, between two altitudes.

        Args:
            name: Area name.
            top: Top altitude bound [m].
            bottom: Bottom altitude bound [m].
            *coords: lat, lon pairs [deg] of the polygon vertices.
        """
        return self.define_area(name, "POLYALT", coords, top, bottom)

    def define_polyline_area(self, name: str, *coords: float) -> Result[str, str]:
        """POLYLINE: Draw a multi-segment line on the radar screen.

        Args:
            name: Line name.
            *coords: lat, lon pairs [deg] of the line points.
        """
        return self.define_area(name, "LINE", coords)

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
        for shape in list(self.basic_shapes.values()):
            self._unregister(shape)
        self.basic_shapes.clear()
        self.areas_by_id.clear()
        self.areas_by_name.clear()
        self.areatree = Index()
        self.max_area_id = 0

    def deleteArea(self, name: str) -> Result[str, str]:
        """Delete a previously defined area by name.

        Args:
            name: Name of the area shape to remove.
        """
        shape = self.basic_shapes.pop(name, None)
        if shape is not None:
            self._unregister(shape)
            return Ok(f"Area {name} deleted.")
        return Err(f"No area found with name {name}.")

    def get_intersecting(self, lat0: float, lon0: float, lat1: float, lon1: float) -> list[Shape]:
        """Return all shapes that intersect with a specified rectangular area.

        Arguments:
        - lat0/1, lon0/1: Coordinates of the top-left and bottom-right corner
          of the intersection area.
        """
        ids = self.areatree.intersection((lat0, lon0, lat1, lon1))
        return [self.areas_by_id[area_id] for area_id in ids if area_id in self.areas_by_id]

    def get_knearest(
        self, lat0: float, lon0: float, lat1: float, lon1: float, k: int = 1
    ) -> list[Shape]:
        """Return the k nearest shapes to a specified rectangular area.

        Arguments:
        - lat0/1, lon0/1: Coordinates of the top-left and bottom-right corner
          of the relevant area.
        - k: The (maximum) number of results to return.
        """
        ids = self.areatree.nearest((lat0, lon0, lat1, lon1), k)
        return [self.areas_by_id[area_id] for area_id in ids if area_id in self.areas_by_id]


class Shape:
    """
    Base class of BlueSky shapes

    Handles the naming, altitude bounds, bounding box, and R-tree
    registration common to all shape types. Derived classes implement
    checkInside() for their specific geometry.

    Attributes:
        name: Area name.
        coordinates: Flat list of lat/lon coordinates in deg defining the
            shape (plus radius in nm for circles).
        top: Upper altitude bound [m].
        bottom: Lower altitude bound [m].
        bbox: Bounding box (latmin, lonmin, latmax, lonmax) in deg.
        area_id: Unique numeric id of this shape in the R-tree.
        raw: Dictionary with the raw shape definition (name, kind,
            coordinates).
    """

    area_id: int

    def __init__(
        self, owner: AreaFilter, name: str, coordinates, top: float = 1e9, bottom: float = -1e9
    ) -> None:
        self.owner = owner
        self._registered = False
        self.raw = {"name": name, "shape": self.kind(), "coordinates": coordinates}
        self.name = name
        self.coordinates = coordinates
        self.top = np.maximum(bottom, top)
        self.bottom = np.minimum(bottom, top)
        lat = coordinates[::2]
        lon = coordinates[1::2]
        self.bbox = [min(lat), min(lon), max(lat), max(lon)]

        # Owner-local weak reference and tree storage
        owner._register(self)

    def __del__(self) -> None:
        # Objects are removed automatically from the weak-value dicts,
        # but need to be manually removed from the rtree
        with suppress(Exception):
            self.owner._unregister(self)

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

    def __init__(self, owner: AreaFilter, name: str, coordinates) -> None:
        super().__init__(owner, name, coordinates)

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

    def __init__(
        self, owner: AreaFilter, name: str, coordinates, top: float = 1e9, bottom: float = -1e9
    ) -> None:
        super().__init__(owner, name, coordinates, top, bottom)
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

    def __init__(
        self, owner: AreaFilter, name: str, coordinates, top: float = 1e9, bottom: float = -1e9
    ) -> None:
        super().__init__(owner, name, coordinates, top, bottom)
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

    def __init__(
        self, owner: AreaFilter, name: str, coordinates, top: float = 1e9, bottom: float = -1e9
    ) -> None:
        super().__init__(owner, name, coordinates, top, bottom)
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
