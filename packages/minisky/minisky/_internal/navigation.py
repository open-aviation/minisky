"""Navigation database of MiniSky.

Loads waypoint, airport, airway, FIR, and country data from the package
data directory and provides lookup functions to find navaids and airports
by identifier or position. Each `MiniSky` runtime owns a Navdatabase at
[`runtime.navigation`][.Navdatabase].
The database backs the DEFWPT stack command and every position
argument that references a navaid, airport, or runway.
"""

from __future__ import annotations

import json
from collections.abc import Sequence
from enum import IntEnum
from pathlib import Path
from typing import Literal, NamedTuple, Protocol, TypeAlias, TypeVar

import numpy as np
import numpy.typing as npt
import pandas as pd

import minisky.geo as geo  # noqa: PLR0402
from minisky import quantities as q
from minisky._internal.command import Keyword, LatLonDeg, command
from minisky._internal.result import Err, Ok, Result
from minisky.types import AirportIdentifier, AirwayIdentifier, RunwayIdentifier, WaypointIdentifier

_COLOCATED_DISTANCE: q.DistanceM[float] = q.nmi_to_m(1.0)

WaypointIndex: TypeAlias = int
AirportIndex: TypeAlias = int
NavigationIndex: TypeAlias = int
_T = TypeVar("_T")


class AirportSize(IntEnum):
    LARGE = 1
    MEDIUM = 2
    SMALL = 3


class AirwayConnection(NamedTuple):
    airway: AirwayIdentifier
    waypoint: WaypointIdentifier


class RunwayThreshold(NamedTuple):
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


def findall(lst: Sequence[_T], x: _T) -> list[NavigationIndex]:
    """Find every occurrence of `x` in `lst`."""
    idx = []
    i = 0
    found = True
    while i < len(lst) and found:
        try:
            i = lst[i:].index(x) + i
            idx.append(i)
            i = i + 1
            found = True
        except ValueError:
            found = False
    return idx


class Navdatabase:
    """
    Navigation database: waypoint, airway, airport, FIR, and country data.

    All data are loaded from the package data directory on construction
    and on [`reset`][.reset]. The database is stored as parallel arrays, indexed per
    waypoint, per airway leg, or per airport.

    Created by  : Jacco M. Hoekstra (TU Delft)
    """

    def __init__(self, data_path: Path) -> None:
        """The navigation database: Contains waypoint, airport, airway, and sector data, but also
        geographical graphics data."""
        self.data_path = data_path
        self.reset()

    def reset(self) -> None:
        """(Re)load all navigation data from the package data directory."""

        nav_data_path = self.data_path

        wptdata = pd.read_parquet(nav_data_path / "waypoint.parquet")
        aptdata = pd.read_parquet(nav_data_path / "airport.parquet")
        awydata = pd.read_parquet(nav_data_path / "airway.parquet")
        codata = pd.read_parquet(nav_data_path / "country.parquet")

        with (nav_data_path / "fir.json").open() as f:
            firdata = json.load(f)
        with (nav_data_path / "runway_thresholds.json").open() as f:
            rwythresholds = json.load(f)

        self.wpid: npt.NDArray[np.str_] = np.asarray(wptdata["wpid"], dtype=str)
        self.wplat: q.LatitudeDeg[np.ndarray] = np.asarray(wptdata["wplat"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        self.wplon: q.LongitudeDeg[np.ndarray] = np.asarray(wptdata["wplon"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        # TODO(abraham): rename this navigation-database category; Route.wptype is a
        # different WaypointType domain, and the shared name makes the two easy to confuse.
        self.wptype: npt.NDArray[np.str_] = np.asarray(wptdata["wptype"], dtype=str)
        """Navigation-database waypoint category; not to be confused with the route `WaypointType`."""
        self.wpelev: q.MslAltitudeM[np.ndarray] = np.asarray(wptdata["wpelev"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        self.wpvar: np.ndarray = np.asarray(wptdata["wpvar"], dtype=float)
        """Magnetic variation at each waypoint, in degrees."""
        self.wpfreq: np.ndarray = np.asarray(wptdata["wpfreq"], dtype=float)
        """Navaid frequencies in the source dataset's kHz/MHz convention."""
        self.wpdesc: npt.NDArray[np.str_] = np.asarray(wptdata["wpdesc"], dtype=str)

        self.awfromwpid: npt.NDArray[np.str_] = np.asarray(awydata["awfromwpid"], dtype=str)
        """Starting waypoint identifier for each airway leg."""
        self.awfromlat: q.LatitudeDeg[np.ndarray] = np.asarray(awydata["awfromlat"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        self.awfromlon: q.LongitudeDeg[np.ndarray] = np.asarray(awydata["awfromlon"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        self.awtowpid: npt.NDArray[np.str_] = np.asarray(awydata["awtowpid"], dtype=str)
        """Ending waypoint identifier for each airway leg."""
        self.awtolat: q.LatitudeDeg[np.ndarray] = np.asarray(awydata["awtolat"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        self.awtolon: q.LongitudeDeg[np.ndarray] = np.asarray(awydata["awtolon"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        self.awid: npt.NDArray[np.str_] = np.asarray(awydata["awid"], dtype=str)
        """Airway identifier for each leg, for example `UL620`."""
        self.awndir: np.ndarray = np.asarray(awydata["awndir"], dtype=np.int64)
        """Number of permitted traversal directions for each airway leg: one or two."""
        self.awlowfl: np.ndarray = np.asarray(awydata["awlowfl"], dtype=np.int64)
        """Lower published flight-level bound for each airway leg."""
        self.awupfl: np.ndarray = np.asarray(awydata["awupfl"], dtype=np.int64)
        """Upper published flight-level bound for each airway leg."""

        self.aptid: npt.NDArray[np.str_] = np.asarray(aptdata["apid"], dtype=str)
        self.aptname: npt.NDArray[np.str_] = np.asarray(aptdata["apname"], dtype=str)
        self.aptlat: q.LatitudeDeg[np.ndarray] = np.asarray(aptdata["aplat"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        self.aptlon: q.LongitudeDeg[np.ndarray] = np.asarray(aptdata["aplon"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        self.aptmaxrwy: q.LengthM[np.ndarray] = np.asarray(aptdata["apmaxrwy"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]
        self.apsize: np.ndarray = np.asarray(aptdata["aptype"], dtype=np.int64)
        """Airport size category from the navigation dataset."""
        self.aptco: npt.NDArray[np.str_] = np.asarray(aptdata["apco"], dtype=str)
        self.aptelev: q.MslAltitudeM[np.ndarray] = np.asarray(aptdata["apelev"], dtype=float)  # pyright: ignore[reportGeneralTypeIssues]

        self.fir: list[str] = firdata["fir"]
        self.firlat0: np.ndarray = np.asarray(firdata["firlat0"], dtype=float)
        """Latitude of the start point of each FIR border segment."""
        self.firlon0: np.ndarray = np.asarray(firdata["firlon0"], dtype=float)
        """Longitude of the start point of each FIR border segment."""
        self.firlat1: np.ndarray = np.asarray(firdata["firlat1"], dtype=float)
        """Latitude of the end point of each FIR border segment."""
        self.firlon1: np.ndarray = np.asarray(firdata["firlon1"], dtype=float)
        """Longitude of the end point of each FIR border segment."""

        self.coname: npt.NDArray[np.str_] = np.asarray(codata["coname"], dtype=str)
        """Country full names"""
        self.cocode2: npt.NDArray[np.str_] = np.asarray(codata["cocode2"], dtype=str)
        """2-character country codes"""
        self.cocode3: npt.NDArray[np.str_] = np.asarray(codata["cocode3"], dtype=str)
        """3-character country codes"""
        self.conr: np.ndarray = np.asarray(codata["conr"], dtype=np.int64)
        """Country ICAO numbers."""

        self.rwythresholds: dict[AirportIdentifier, dict[RunwayIdentifier, RunwayThreshold]] = {
            airport: {
                runway: RunwayThreshold(float(values[0]), float(values[1]), float(values[2]))
                for runway, values in runways.items()
            }
            for airport, runways in rwythresholds.items()
        }
        """Runway thresholds keyed by airport and runway identifier."""

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
        if normalized not in self.wpid:
            return Ok(f"Waypoint {normalized} does not yet exist.")
        wpids = self.wpid.tolist()
        index = len(wpids) - wpids[::-1].index(normalized) - 1
        description = f"{self.wpid[index]} : {self.wplat[index]},{self.wplon[index]}"
        if self.wptype[index]:
            description += f"  {self.wptype[index]}"
        return Ok(description)

    def defwpt(
        self,
        name: WaypointIdentifier,
        lat: q.LatitudeDeg[float],
        lon: q.LongitudeDeg[float],
        waypoint_type: str | None = None,
    ) -> Result[str, str]:
        """Add a scenario-specific waypoint to the navigation database."""
        normalized = name.upper()
        if not normalized:
            return Err("Waypoint name is required")
        if normalized.isdigit():
            return Err("Waypoint name must start with an alphabetical character")

        self.wpid = np.append(self.wpid, normalized)
        self.wplat = np.append(self.wplat, lat)
        self.wplon = np.append(self.wplon, lon)
        self.wptype = np.append(self.wptype, "" if waypoint_type is None else waypoint_type.upper())
        self.wpelev = np.append(self.wpelev, 0.0)
        self.wpvar = np.append(self.wpvar, 0.0)
        self.wpfreq = np.append(self.wpfreq, 0.0)
        self.wpdesc = np.append(self.wpdesc, "Custom waypoint")

        return Ok(f"{normalized} added to navdb.")

    def delwpt(self, name: WaypointIdentifier) -> Result[str, str]:
        """Delete a waypoint from the database.

        The last-added occurrence of the name is removed.
        """
        wpids = self.wpid.tolist()
        if wpids.count(name.upper()) <= 0:
            return Err(f"Waypoint {name.upper()} does not exist.")

        idx = len(wpids) - wpids[::-1].index(name.upper()) - 1  # Search from back of list

        self.wpid = np.delete(self.wpid, idx)
        self.wplat = np.delete(self.wplat, idx)
        self.wplon = np.delete(self.wplon, idx)
        self.wptype = np.delete(self.wptype, idx)
        self.wpelev = np.delete(self.wpelev, idx)
        self.wpvar = np.delete(self.wpvar, idx)
        self.wpfreq = np.delete(self.wpfreq, idx)
        self.wpdesc = np.delete(self.wpdesc, idx)

        return Ok(name.upper() + " deleted from navdb.")

    def getwpidx(
        self, txt: WaypointIdentifier, reference: LatLonReference | None = None
    ) -> WaypointIndex | None:
        """Get a waypoint index by identifier.

        When duplicate identifiers exist, `reference` selects the geographically
        closest occurrence; without a reference, the first occurrence wins.

        Args:
            txt: Navigation-dataset waypoint identifier.
            reference: When supplied and the identifier occurs more than once,
                choose the occurrence nearest this position.

        Returns:
            The selected waypoint index, or `None` when the identifier is absent.
        """
        name = txt.upper()
        wpids = self.wpid.tolist()
        try:
            i = wpids.index(name)
        except ValueError:
            return None

        if reference is None:
            return i

        else:
            idx = []
            idx.append(i)
            found = True
            while i < len(self.wpid) - 1 and found:
                try:
                    i = wpids.index(name, i + 1)
                    idx.append(i)
                except ValueError:
                    found = False
            if len(idx) == 1:
                return idx[0]
            else:
                imin = idx[0]
                dmin = geo.kwikdist(
                    reference.lat, reference.lon, self.wplat[imin], self.wplon[imin]
                )
                for i in idx[1:]:
                    d = geo.kwikdist(reference.lat, reference.lon, self.wplat[i], self.wplon[i])
                    if d < dmin:
                        imin = i
                        dmin = d
                return imin

    def getwpindices(
        self,
        txt: WaypointIdentifier,
        reference: LatLonReference | None = None,
        crit: q.DistanceM[float] = _COLOCATED_DISTANCE,
    ) -> list[WaypointIndex]:
        """Get indices of a waypoint and its co-located duplicates.

        Finds the occurrence of the identifier closest to the reference
        position, plus all other occurrences within a distance criterion.
        Args:
            txt: Navigation-dataset waypoint identifier.
            reference: Reference used to select the primary occurrence when duplicates exist.
            crit: Maximum distance from that occurrence for additional co-located matches; defaults to 1 NM.

        Returns:
            The primary occurrence followed by co-located duplicates,
            or an empty list when the identifier is absent.
        """
        name = txt.upper()
        wpids = self.wpid.tolist()
        try:
            i = wpids.index(name)
        except ValueError:
            return []

        if reference is None:
            return [i]

        else:
            idx = findall(wpids, name)

            if len(idx) == 1:
                return [idx[0]]
            else:
                imin = idx[0]
                dmin = geo.kwikdist(
                    reference.lat, reference.lon, self.wplat[imin], self.wplon[imin]
                )
                for i in idx[1:]:
                    d = geo.kwikdist(reference.lat, reference.lon, self.wplat[i], self.wplon[i])
                    if d < dmin:
                        imin = i
                        dmin = d
                indices = [imin]
                for i in idx:
                    if i != imin:
                        dist = geo.kwikdist(
                            self.wplat[i],
                            self.wplon[i],
                            self.wplat[imin],
                            self.wplon[imin],
                        )
                        if dist <= crit:
                            indices.append(i)

                return indices

    def getaptidx(self, txt: AirportIdentifier) -> AirportIndex | None:
        """Get the index of an airport by its navigation-dataset identifier."""
        try:
            return self.aptid.tolist().index(txt.upper())
        except ValueError:
            return None

    def getinear(
        self,
        wlat: q.LatitudeDeg,
        wlon: q.LongitudeDeg,
        lat: q.LatitudeDeg[float],
        lon: q.LongitudeDeg[float],
    ) -> NavigationIndex:
        """Get the index of the entry nearest to a given position.

        Uses a fast flat-earth squared-distance comparison.
        """
        wlat = np.asarray(wlat)
        wlon = np.asarray(wlon)
        f = np.cos(np.radians(lat))
        dlat = (wlat - lat + 180.0) % 360.0 - 180.0
        dlon = f * ((wlon - lon + 180.0) % 360.0 - 180.0)
        d2 = dlat * dlat + dlon * dlon
        idx = np.argmin(d2)
        return int(idx)

    def getwpinear(self, lat: q.LatitudeDeg[float], lon: q.LongitudeDeg[float]) -> WaypointIndex:
        """Get the waypoint index nearest to the given position."""
        return self.getinear(self.wplat, self.wplon, lat, lon)

    def getapinear(self, lat: q.LatitudeDeg[float], lon: q.LongitudeDeg[float]) -> AirportIndex:
        """Get the airport index nearest to the given position."""
        return self.getinear(self.aptlat, self.aptlon, lat, lon)

    def getinside(
        self,
        wlat: q.LatitudeDeg,
        wlon: q.LongitudeDeg,
        lat0: q.LatitudeDeg[float],
        lat1: q.LatitudeDeg[float],
        lon0: q.LongitudeDeg[float],
        lon1: q.LongitudeDeg[float],
    ) -> list[NavigationIndex]:
        """Get indices of positions inside the given lat/lon box."""
        wlat = np.asarray(wlat)
        wlon = np.asarray(wlon)
        if lat0 < lat1:
            arr = np.where((wlat > lat0) * (wlat < lat1) * (wlon > lon0) * (wlon < lon1))
        else:
            arr = np.where((wlat > lat1) + (wlat < lat0) * (wlon > lon0) * (wlon < lon1))

        return [int(i) for i in arr[0]]

    def getwpinside(
        self,
        lat0: q.LatitudeDeg[float],
        lat1: q.LatitudeDeg[float],
        lon0: q.LongitudeDeg[float],
        lon1: q.LongitudeDeg[float],
    ) -> list[WaypointIndex]:
        """Get waypoint indices inside the given lat/lon box."""
        return self.getinside(self.wplat, self.wplon, lat0, lat1, lon0, lon1)

    def getapinside(
        self,
        lat0: q.LatitudeDeg[float],
        lat1: q.LatitudeDeg[float],
        lon0: q.LongitudeDeg[float],
        lon1: q.LongitudeDeg[float],
    ) -> list[AirportIndex]:
        """Get airport indices inside the given lat/lon box."""
        return self.getinside(self.aptlat, self.aptlon, lat0, lat1, lon0, lon1)

    def listairway(self, airwayid: AirwayIdentifier) -> list[list[WaypointIdentifier]]:
        """Return the waypoint sequence(s) of an airway.

        Collects all legs of the airway and chains them into ordered
        segments of waypoint identifiers; an airway may consist of
        multiple separate segments. Missing airways return an empty list.
        """
        awkey = airwayid.upper()

        airway = []  # identifier of waypoint   0 .. N-1

        awids = self.awid.tolist()
        if awids.count(awkey) > 0:
            i = 0
            found = True
            legs: list[str] = []  # Alle leg incl. duplicate legs
            left = []  # wps in left column in file
            right = []  # wps in right coumn in file

            idx = findall(awids, awkey)
            for i in idx:
                newleg = self.awfromwpid[i] + "-" + self.awtowpid[i]
                if newleg not in legs:
                    legs.append(newleg)
                    left.append(self.awfromwpid[i])
                    right.append(self.awtowpid[i])

            if len(legs) == 0:
                return []

            unused = len(left) + len(right)

            while unused > 0 and left != len(left) * [""]:
                # Find start of a segment
                wps = left + right
                iwps = 0
                while iwps < len(wps) and wps.count(wps[iwps]) > 1:
                    iwps = iwps + 1

                i = iwps % len(left)
                j = int(iwps / len(left))

                # Catch single lost wps
                if j > 1 or iwps > len(wps):
                    break

                wps = [left, right]
                segment = []
                nextwp = ""

                segready = False
                while not segready:
                    # Get leg
                    curwp = wps[j][i]
                    nextwp = wps[1 - j][i]

                    # Update admin of to do wplist
                    unused = unused - 2
                    wps[j][i] = ""
                    wps[1 - j][i] = ""

                    segment.append(curwp)

                    # Find next lef with nextwp
                    if wps[0].count(nextwp) > 0:
                        j = 0
                        i = wps[0].index(nextwp)
                        found = True

                    elif wps[1].count(nextwp) > 0:
                        i = wps[1].index(nextwp)
                        j = 1
                        found = True
                    else:
                        found = False

                    segready = (not found) or curwp == "" or nextwp == ""

                # Also add final nextwp of this segment
                segment.append(nextwp)

                # Airway cab have multiple separate segments
                airway.append(segment)

                left = wps[0]
                right = wps[1]

        return airway

    def listconnections(
        self, wpid: WaypointIdentifier, wplat: q.LatitudeDeg[float], wplon: q.LongitudeDeg[float]
    ) -> list[AirwayConnection]:
        """Return the airway legs connecting to a given waypoint.

        Only legs whose stored endpoint lies within 10 nm of the given
        position are returned.
        """
        connect: list[AirwayConnection] = []

        if wpid in self.awfromwpid:
            idx = findall(self.awfromwpid.tolist(), wpid)
            for i in idx:
                newitem = AirwayConnection(self.awid[i], self.awtowpid[i])
                if (newitem not in connect) and geo.kwikdist(
                    self.awfromlat[i], self.awfromlon[i], wplat, wplon
                ) < q.nmi_to_m(10.0):
                    connect.append(newitem)

        if wpid in self.awtowpid:
            idx = findall(self.awtowpid.tolist(), wpid)
            for i in idx:
                newitem = AirwayConnection(self.awid[i], self.awfromwpid[i])
                if (newitem not in connect) and geo.kwikdist(
                    self.awtolat[i], self.awtolon[i], wplat, wplon
                ) < q.nmi_to_m(10.0):
                    connect.append(newitem)

        return connect
