"""Area filter module

Defines named geometric shapes - boxes, circles, polygons, and lines - on
the map, optionally bounded by a top and bottom altitude, and provides
point-inside-shape tests for (vectors of) aircraft positions. This backs
the BOX, CIRCLE, POLY, POLYALT, LINE, and POLYLINE stack commands, and is
used by plugins and traffic logic that need to know which aircraft are
inside an area. All defined shapes are stored by name in ``basic_shapes``
and indexed in an R-tree for fast geospatial queries.
"""

from weakref import WeakValueDictionary

import numpy as np
from matplotlib.path import Path

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

# Dictionary of all basic shapes (The shape classes defined in this file) by name
basic_shapes = {}


def has_area(areaname: str) -> bool:
    """Check if area with name 'areaname' exists."""
    return areaname in basic_shapes


def define_area(
    areaname: str, areatype: str, coordinates: tuple[float, ...], top: float = 1e9, bottom: float = -1e9
) -> tuple[bool, str]:
    """Define a new area, or list/inspect existing areas.

    Args:
        areaname: Name of the area, or "LIST" to list all defined shapes.
        areatype: Shape type: "BOX", "CIRCLE", "POLY"/"POLYALT", or "LINE".
        coordinates: Flat sequence of lat/lon pairs [deg]; for a circle:
            (lat [deg], lon [deg], radius [nm]). When empty, information
            about the existing area with the given name is returned.
        top: Top altitude bound [m] (default: effectively unbounded).
        bottom: Bottom altitude bound [m] (default: effectively unbounded).

    Returns:
        tuple: (success (bool), message (str)).
    """
    if areaname == "LIST":
        if not basic_shapes:
            return True, "No shapes are currently defined."
        else:
            return True, "Currently defined shapes:\n" + ", ".join(basic_shapes)
    if not coordinates:
        if areaname in basic_shapes:
            return True, str(basic_shapes[areaname])
        else:
            return False, f"Unknown shape: {areaname}"
    if areatype == "BOX":
        basic_shapes[areaname] = Box(areaname, coordinates, top, bottom)
    elif areatype == "CIRCLE":
        basic_shapes[areaname] = Circle(areaname, coordinates, top, bottom)
    elif areatype[:4] == "POLY":
        basic_shapes[areaname] = Poly(areaname, coordinates, top, bottom)
    elif areatype == "LINE":
        basic_shapes[areaname] = Line(areaname, coordinates)

    return True, f"Created {areatype} {areaname}"


def define_box_area(name: str, *coords: float) -> tuple[bool, str]:
    """BOX: Define a box-shaped area.

    Args:
        name: Area name.
        *coords: lat1, lon1, lat2, lon2 [deg] of two opposite corners,
            optionally followed by top and bottom altitude [m].
    """
    return define_area(name, "BOX", coords[:4], *coords[4:])


def define_circle_area(name: str, *coords: float) -> tuple[bool, str]:
    """CIRCLE: Define a circle-shaped area.

    Args:
        name: Area name.
        *coords: lat, lon [deg] of the center and radius [nm], optionally
            followed by top and bottom altitude [m].
    """
    return define_area(name, "CIRCLE", coords[:3], *coords[3:])


def define_line_area(name: str, *coords: float) -> tuple[bool, str]:
    """LINE: Draw a line between two positions on the radar screen.

    Args:
        name: Line name.
        *coords: lat1, lon1, lat2, lon2 [deg] of the two end points.
    """
    return define_area(name, "LINE", coords)


def define_poly_area(name: str, *coords: float) -> tuple[bool, str]:
    """POLY: Define a polygon-shaped area.

    Args:
        name: Area name.
        *coords: lat, lon pairs [deg] of the polygon vertices.
    """
    return define_area(name, "POLY", coords)


def define_polyalt_area(name: str, top: float, bottom: float, *coords: float) -> tuple[bool, str]:
    """POLYALT: Define a polygon-shaped area in 3D, between two altitudes.

    Args:
        name: Area name.
        top: Top altitude bound [m].
        bottom: Bottom altitude bound [m].
        *coords: lat, lon pairs [deg] of the polygon vertices.
    """
    return define_area(name, "POLYALT", coords, top, bottom)


def define_polyline_area(name: str, *coords: float) -> tuple[bool, str]:
    """POLYLINE: Draw a multi-segment line on the radar screen.

    Args:
        name: Line name.
        *coords: lat, lon pairs [deg] of the line points.
    """
    return define_area(name, "LINE", coords)


def checkInside(areaname: str, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray) -> np.ndarray:
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
    if areaname not in basic_shapes:
        return np.zeros(len(lat), dtype=bool)
    area = basic_shapes[areaname]
    return area.checkInside(lat, lon, alt)


def reset() -> None:
    """Clear all data."""
    basic_shapes.clear()
    Shape.reset()


def deleteArea(name: str) -> tuple[bool, str]:
    """Delete a previously defined area by name.

    Args:
        name: Name of the area shape to remove.

    Returns:
        tuple: (success (bool), message (str)).
    """
    if name in basic_shapes:
        del basic_shapes[name]
        return True, f"Area {name} deleted."
    return False, f"No area found with name {name}."


def get_intersecting(lat0: float, lon0: float, lat1: float, lon1: float) -> list:
    """Return all shapes that intersect with a specified rectangular area.

    Arguments:
    - lat0/1, lon0/1: Coordinates of the top-left and bottom-right corner
      of the intersection area.
    """
    items = Shape.areatree.intersection((lat0, lon0, lat1, lon1))
    return [Shape.areas_by_id[i.id] for i in items]


def get_knearest(lat0: float, lon0: float, lat1: float, lon1: float, k: int = 1) -> list:
    """Return the k nearest shapes to a specified rectangular area.

    Arguments:
    - lat0/1, lon0/1: Coordinates of the top-left and bottom-right corner
      of the relevant area.
    - k: The (maximum) number of results to return.
    """
    items = Shape.areatree.nearest((lat0, lon0, lat1, lon1), k)
    return [Shape.areas_by_id[i.id] for i in items]


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

    # Global counter to keep track of used shape ids
    max_area_id = 0

    # Weak-value dictionary of all Shape-derived objects by name, and id
    areas_by_id = WeakValueDictionary()
    areas_by_name = WeakValueDictionary()

    # RTree of all areas for efficient geospatial searching
    areatree = Index()

    @classmethod
    def reset(cls) -> None:
        """Reset shape data when simulation is reset."""
        # Weak dicts and areatree should be cleared automatically
        # Reset max area id
        cls.max_area_id = 0

    def __init__(self, name: str, coordinates, top: float = 1e9, bottom: float = -1e9) -> None:
        self.raw = {"name": name, "shape": self.kind(), "coordinates": coordinates}
        self.name = name
        self.coordinates = coordinates
        self.top = np.maximum(bottom, top)
        self.bottom = np.minimum(bottom, top)
        lat = coordinates[::2]
        lon = coordinates[1::2]
        self.bbox = [min(lat), min(lon), max(lat), max(lon)]

        # Global weak reference and tree storage
        self.area_id = Shape.max_area_id
        Shape.max_area_id += 1
        Shape.areas_by_id[self.area_id] = self
        Shape.areas_by_name[self.name] = self
        Shape.areatree.insert(self.area_id, self.bbox)

    def __del__(self) -> None:
        # Objects are removed automatically from the weak-value dicts,
        # but need to be manually removed from the rtree
        Shape.areatree.delete(self.area_id, self.bbox)

    def checkInside(self, lat: np.ndarray, lon: np.ndarray, alt: np.ndarray):
        """Returns True (or boolean array) if coordinate lat, lon, alt lies
        within this shape.

        Reimplement this function in the derived shape classes for this to
        work.
        """
        return False

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
