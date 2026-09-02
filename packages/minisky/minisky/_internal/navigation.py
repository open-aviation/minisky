"""Navigation data for MiniSky, including waypoint, airport, airway, FIR,
country and runway data.

Note that the exact storage format (e.g. parquet, CSV, JSON) or file I/O do not
belong here. External providers should handle it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Literal, NamedTuple, Protocol, TypeAlias

import numpy as np
import numpy.typing as npt

import minisky.geo as geo  # noqa: PLR0402
from minisky import quantities as q
from minisky._internal.command import Keyword, LatLonDeg, command
from minisky._internal.result import Err, Ok, Result
from minisky.types import AirportIdentifier, AirwayIdentifier, RunwayIdentifier, WaypointIdentifier

_COLOCATED_DISTANCE: q.DistanceM[float] = q.nmi_to_m(1.0)

WaypointIndex: TypeAlias = int
AirportIndex: TypeAlias = int
NavigationIndex: TypeAlias = int


class AirportSize(IntEnum):
    LARGE = 1
    MEDIUM = 2
    SMALL = 3


class AirwayConnection(NamedTuple):
    airway: AirwayIdentifier
    waypoint: WaypointIdentifier


@dataclass(frozen=True, slots=True)
class RunwayThreshold:
    """Runway threshold position and true runway heading."""

    latitude: q.LatitudeDeg[float]
    longitude: q.LongitudeDeg[float]
    heading: q.TrueHeadingDegrees[float]


class LatLonReference(Protocol):
    """A position record usable as a waypoint disambiguation reference."""

    @property
    def lat(self) -> q.LatitudeDeg[float]: ...

    @property
    def lon(self) -> q.LongitudeDeg[float]: ...


@dataclass(slots=True, eq=False)
class WaypointData:
    identifiers: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    latitudes: q.LatitudeDeg[np.ndarray] = field(default_factory=lambda: np.array([], dtype=float))
    longitudes: q.LongitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    categories: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    """Navigation-dataset waypoint category. Not to be confused with route `WaypointType`."""
    elevations: q.MslAltitudeM[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    magnetic_variations: q.MagneticDeclinationDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    frequencies: q.FrequencyHz[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    descriptions: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))


@dataclass(slots=True, init=False, eq=False)
class Waypoints:
    identifiers: npt.NDArray[np.str_]
    latitudes: q.LatitudeDeg[np.ndarray]
    longitudes: q.LongitudeDeg[np.ndarray]
    categories: npt.NDArray[np.str_]
    elevations: q.MslAltitudeM[np.ndarray]
    magnetic_variations: q.MagneticDeclinationDeg[np.ndarray]
    frequencies: q.FrequencyHz[np.ndarray]
    descriptions: npt.NDArray[np.str_]
    _source: WaypointData

    def __init__(self, source: WaypointData) -> None:
        self._source = WaypointData(
            identifiers=source.identifiers.copy(),
            latitudes=source.latitudes.copy(),
            longitudes=source.longitudes.copy(),
            categories=source.categories.copy(),
            elevations=source.elevations.copy(),
            magnetic_variations=source.magnetic_variations.copy(),
            frequencies=source.frequencies.copy(),
            descriptions=source.descriptions.copy(),
        )
        self.reset()

    def reset(self) -> None:
        """Restore scenario-mutated waypoints from the loaded source data."""
        self.identifiers = self._source.identifiers.copy()
        self.latitudes = self._source.latitudes.copy()
        self.longitudes = self._source.longitudes.copy()
        self.categories = self._source.categories.copy()
        self.elevations = self._source.elevations.copy()
        self.magnetic_variations = self._source.magnetic_variations.copy()
        self.frequencies = self._source.frequencies.copy()
        self.descriptions = self._source.descriptions.copy()

    @command(name="DEFWPT")
    def describe_from_scenario(self, name: Keyword) -> Result[str, str]:
        """Inspect a scenario-specific waypoint or report that its name is available."""
        return self.describe_waypoint(name)

    @command(name="DEFWPT")
    def delete_from_scenario(
        self, name: Keyword, _action: Literal["DEL", "DELETE"]
    ) -> Result[str, str]:
        """Delete a scenario-specific waypoint."""
        return self.delwpt(name)

    @command(name="DEFWPT", examples=("DEFWPT MYWPT 52.3 4.7",))
    def define_from_scenario(self, name: Keyword, position: LatLonDeg) -> Result[str, str]:
        """Define a scenario-specific waypoint from a position."""
        return self.defwpt(name, position.lat, position.lon)

    @command(name="DEFWPT")
    def delete_from_position_form(
        self, name: Keyword, position: LatLonDeg, _action: Literal["DEL", "DELETE"]
    ) -> Result[str, str]:
        """Delete a waypoint from the DEFWPT name,position,DEL form."""
        return self.delwpt(name)

    @command(name="DEFWPT")
    def define_typed_from_scenario(
        self, name: Keyword, position: LatLonDeg, waypoint_type: Keyword
    ) -> Result[str, str]:
        """Define a scenario-specific waypoint with an explicit waypoint type."""
        return self.defwpt(name, position.lat, position.lon, waypoint_type)

    def describe_waypoint(self, name: WaypointIdentifier) -> Result[str, str]:
        """Describe a waypoint or report that its name is available."""
        normalized = name.upper()
        indices = np.flatnonzero(self.identifiers == normalized)
        if len(indices) == 0:
            return Ok(f"Waypoint {normalized} does not yet exist.")
        index = int(indices[-1])
        description = (
            f"{self.identifiers[index]} : {self.latitudes[index]},{self.longitudes[index]}"
        )
        if self.categories[index]:
            description += f"  {self.categories[index]}"
        return Ok(description)

    def defwpt(
        self,
        name: WaypointIdentifier,
        lat: q.LatitudeDeg[float],
        lon: q.LongitudeDeg[float],
        waypoint_type: str | None = None,
    ) -> Result[str, str]:
        """Add a scenario-specific waypoint."""
        normalized = name.upper()
        if not normalized:
            return Err("Waypoint name is required")
        if normalized.isdigit():
            return Err("Waypoint name must start with an alphabetical character")
        self.identifiers = np.append(self.identifiers, normalized)
        self.latitudes = np.append(self.latitudes, lat)
        self.longitudes = np.append(self.longitudes, lon)
        self.categories = np.append(
            self.categories, "" if waypoint_type is None else waypoint_type.upper()
        )
        self.elevations = np.append(self.elevations, 0.0)
        self.magnetic_variations = np.append(self.magnetic_variations, 0.0)
        self.frequencies = np.append(self.frequencies, 0.0)
        self.descriptions = np.append(self.descriptions, "Custom waypoint")
        return Ok(f"{normalized} added to navigation data.")

    def delwpt(self, name: WaypointIdentifier) -> Result[str, str]:
        """Delete the last-added occurrence of a waypoint."""
        normalized = name.upper()
        indices = np.flatnonzero(self.identifiers == normalized)
        if len(indices) == 0:
            return Err(f"Waypoint {normalized} does not exist.")
        index = int(indices[-1])  # Search from back of list
        self.identifiers = np.delete(self.identifiers, index)
        self.latitudes = np.delete(self.latitudes, index)
        self.longitudes = np.delete(self.longitudes, index)
        self.categories = np.delete(self.categories, index)
        self.elevations = np.delete(self.elevations, index)
        self.magnetic_variations = np.delete(self.magnetic_variations, index)
        self.frequencies = np.delete(self.frequencies, index)
        self.descriptions = np.delete(self.descriptions, index)
        return Ok(f"{normalized} deleted from navigation data.")

    def getidx(
        self, identifier: WaypointIdentifier, reference: LatLonReference | None = None
    ) -> WaypointIndex | None:
        """Return the first matching waypoint, or the closest duplicate to `reference`."""
        indices = np.flatnonzero(self.identifiers == identifier.upper())
        if len(indices) == 0:
            return None
        if reference is None or len(indices) == 1:
            return int(indices[0])
        distances = geo.kwikdist(
            reference.lat, reference.lon, self.latitudes[indices], self.longitudes[indices]
        )
        return int(indices[int(np.argmin(distances))])

    def getindices(
        self,
        identifier: WaypointIdentifier,
        reference: LatLonReference | None = None,
        crit: q.DistanceM[float] = _COLOCATED_DISTANCE,
    ) -> list[WaypointIndex]:
        """Return matching waypoint indices.

        Without a reference, only the first occurrence is returned. With a
        reference, the closest occurrence is followed by duplicates within
        `crit` of it.
        """
        indices = np.flatnonzero(self.identifiers == identifier.upper())
        if len(indices) == 0:
            return []
        if reference is None:
            return [int(indices[0])]
        distances = geo.kwikdist(
            reference.lat, reference.lon, self.latitudes[indices], self.longitudes[indices]
        )
        primary = int(indices[int(np.argmin(distances))])
        colocated = geo.kwikdist(
            self.latitudes[indices],
            self.longitudes[indices],
            self.latitudes[primary],
            self.longitudes[primary],
        )
        return [
            primary,
            *(
                int(index)
                for index, distance in zip(indices, colocated)
                if index != primary and distance <= crit
            ),
        ]


@dataclass(slots=True, eq=False)
class AirportData:
    identifiers: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    names: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    latitudes: q.LatitudeDeg[np.ndarray] = field(default_factory=lambda: np.array([], dtype=float))
    longitudes: q.LongitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    max_runway_lengths: q.LengthM[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    sizes: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int64))
    """Integer airport-size category codes from the navigation dataset."""
    countries: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    elevations: q.MslAltitudeM[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )

    def getidx(self, identifier: AirportIdentifier) -> AirportIndex | None:
        """Get the index of an airport by its navigation-dataset identifier."""
        indices = np.flatnonzero(self.identifiers == identifier.upper())
        return None if len(indices) == 0 else int(indices[0])

    def getnearest(
        self, latitude: q.LatitudeDeg[float], longitude: q.LongitudeDeg[float]
    ) -> AirportIndex:
        """Get the airport index nearest to the given position."""
        f = np.cos(np.radians(latitude))
        dlat = (self.latitudes - latitude + 180.0) % 360.0 - 180.0
        dlon = f * ((self.longitudes - longitude + 180.0) % 360.0 - 180.0)
        return int(np.argmin(dlat * dlat + dlon * dlon))


@dataclass(slots=True, eq=False)
class AirwayData:
    identifiers: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    """Airway identifier for each leg, for example `UL620`."""
    from_waypoints: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    from_latitudes: q.LatitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    from_longitudes: q.LongitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    to_waypoints: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    to_latitudes: q.LatitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    to_longitudes: q.LongitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    directions: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int64))
    """Number of permitted traversal directions for each airway leg: one or two."""
    lower_altitudes: q.PressureAltitudeM[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    upper_altitudes: q.PressureAltitudeM[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )

    def listairway(self, airwayid: AirwayIdentifier) -> list[list[WaypointIdentifier]]:
        """Return the waypoint sequence(s) of an airway.

        Collects all legs of the airway and chains them into ordered
        segments of waypoint identifiers; an airway may consist of
        multiple separate segments. Missing airways return an empty list.
        """
        indices = np.flatnonzero(self.identifiers == airwayid.upper())
        if len(indices) == 0:
            return []

        legs: set[str] = set()
        left: list[str] = []
        right: list[str] = []
        for index in indices:
            source = str(self.from_waypoints[index])
            destination = str(self.to_waypoints[index])
            leg = f"{source}-{destination}"
            if leg not in legs:
                legs.add(leg)
                left.append(source)
                right.append(destination)

        airways: list[list[WaypointIdentifier]] = []
        unused = len(left) + len(right)
        while unused > 0 and left != len(left) * [""]:
            # Find start of a segment
            waypoints = left + right
            waypoint_index = 0
            while (
                waypoint_index < len(waypoints) and waypoints.count(waypoints[waypoint_index]) > 1
            ):
                waypoint_index += 1

            index = waypoint_index % len(left)
            side = int(waypoint_index / len(left))

            # Catch single lost wps
            if side > 1 or waypoint_index > len(waypoints):
                break

            columns = [left, right]
            segment: list[WaypointIdentifier] = []
            next_waypoint = ""
            ready = False
            while not ready:
                # Get leg
                current = columns[side][index]
                next_waypoint = columns[1 - side][index]

                # Update admin of to do wplist
                unused -= 2
                columns[side][index] = ""
                columns[1 - side][index] = ""
                segment.append(current)

                # Find next lef with nextwp
                if next_waypoint in columns[0]:
                    side = 0
                    index = columns[0].index(next_waypoint)
                    found = True
                elif next_waypoint in columns[1]:
                    side = 1
                    index = columns[1].index(next_waypoint)
                    found = True
                else:
                    found = False
                ready = not found or current == "" or next_waypoint == ""

            # Also add final nextwp of this segment
            segment.append(next_waypoint)

            # Airway cab have multiple separate segments
            airways.append(segment)
            left, right = columns

        return airways

    def listconnections(
        self, wpid: WaypointIdentifier, wplat: q.LatitudeDeg[float], wplon: q.LongitudeDeg[float]
    ) -> list[AirwayConnection]:
        """Return the airway legs connecting to a given waypoint.

        Only legs whose stored endpoint lies within 10 nm of the given
        position are returned.
        """
        connections: list[AirwayConnection] = []
        for index in np.flatnonzero(self.from_waypoints == wpid):
            connection = AirwayConnection(
                str(self.identifiers[index]), str(self.to_waypoints[index])
            )
            if connection not in connections and geo.kwikdist(
                self.from_latitudes[index],
                self.from_longitudes[index],
                wplat,
                wplon,
            ) < q.nmi_to_m(10.0):
                connections.append(connection)

        for index in np.flatnonzero(self.to_waypoints == wpid):
            connection = AirwayConnection(
                str(self.identifiers[index]), str(self.from_waypoints[index])
            )
            if connection not in connections and geo.kwikdist(
                self.to_latitudes[index],
                self.to_longitudes[index],
                wplat,
                wplon,
            ) < q.nmi_to_m(10.0):
                connections.append(connection)

        return connections


@dataclass(slots=True, eq=False)
class FirBoundary:
    identifier: str
    latitudes: q.LatitudeDeg[np.ndarray] = field(default_factory=lambda: np.array([], dtype=float))
    longitudes: q.LongitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )


@dataclass(slots=True, eq=False)
class FirData:
    boundaries: tuple[FirBoundary, ...] = field(default_factory=tuple)
    segment_start_latitudes: q.LatitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    segment_start_longitudes: q.LongitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    segment_end_latitudes: q.LatitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )
    segment_end_longitudes: q.LongitudeDeg[np.ndarray] = field(
        default_factory=lambda: np.array([], dtype=float)
    )


@dataclass(slots=True, eq=False)
class CountryData:
    names: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    """Country full names."""
    codes2: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    """Two-character country codes."""
    codes3: npt.NDArray[np.str_] = field(default_factory=lambda: np.array([], dtype=str))
    """Three-character country codes."""
    numbers: np.ndarray = field(default_factory=lambda: np.array([], dtype=np.int64))
    """Country ICAO numbers."""


RunwayThresholdData: TypeAlias = dict[AirportIdentifier, dict[RunwayIdentifier, RunwayThreshold]]


@dataclass(frozen=True, slots=True, eq=False)
class NavData:
    waypoints: WaypointData = field(default_factory=WaypointData)
    airports: AirportData = field(default_factory=AirportData)
    airways: AirwayData = field(default_factory=AirwayData)
    firs: FirData = field(default_factory=FirData)
    countries: CountryData = field(default_factory=CountryData)
    runway_thresholds: RunwayThresholdData = field(default_factory=dict)
