"""Navigation database of MiniSky.

Loads waypoint, airport, airway, FIR, and country data from the package
data directory and provides lookup functions to find navaids and airports
by identifier or position. Each `MiniSky` runtime owns a Navdatabase at
[`runtime.navigation`][minisky.tools.navdata.Navdatabase].
The database backs the DEFWPT stack command and every position
argument that references a navaid, airport, or runway.
"""

from __future__ import annotations

import json
from enum import IntEnum
from pathlib import Path
from typing import Literal, Protocol

import numpy as np
import pandas as pd

from minisky import quantities as q
from minisky.command import Keyword, LatLonDeg, command
from minisky.result import Err, Ok, Result
from minisky.tools import geo

_COLOCATED_DISTANCE: q.DistanceM[float] = q.nmi_to_m(1.0)


class AirportSize(IntEnum):
    LARGE = 1
    MEDIUM = 2
    SMALL = 3


class LatLonReference(Protocol):
    """A position record usable as a waypoint disambiguation reference."""

    @property
    def lat(self) -> q.LatitudeDeg[float]: ...

    @property
    def lon(self) -> q.LongitudeDeg[float]: ...


def _tolist(column) -> list:
    """Return a pandas column as a plain Python list.

    Wrapper around `Series.to_list()` that gives a concrete `list`
    return type (the pandas `__getitem__` overloads otherwise widen the
    result to include `str`).
    """
    return column.to_list()


def findall(lst, x) -> list:
    """Find indices of multiple occurences of x in lst.

    Args:
        lst: List to search.
        x: Element to find.

    Returns:
        list: Indices of all occurrences of x in lst.
    """
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
    and on reset(). The database is stored as parallel lists, indexed per
    waypoint, per airway leg, or per airport.

    Attributes:
        wpid: Waypoint identifiers / short names (strings).
        wplat: Waypoint latitudes [deg].
        wplon: Waypoint longitudes [deg].
        wptype: Waypoint types (strings).
        wpelev: Waypoint elevations [m].
        wpvar: Magnetic variation at the waypoints [deg].
        wpfreq: Navaid frequencies [kHz/MHz].
        wpdesc: Waypoint descriptions.
        awid: Airway identifiers, per leg (strings).
        awfromwpid: Identifiers of the start waypoint of each leg.
        awfromlat: Start waypoint latitudes [deg].
        awfromlon: Start waypoint longitudes [deg].
        awtowpid: Identifiers of the end waypoint of each leg.
        awtolat: End waypoint latitudes [deg].
        awtolon: End waypoint longitudes [deg].
        awndir: Number of directions of each leg (1 or 2).
        awlowfl: Lower flight level of each leg (int).
        awupfl: Upper flight level of each leg (int).
        aptid: Airport 4-character ICAO identifiers (strings).
        aptname: Airport full names.
        aptlat: Airport latitudes [deg].
        aptlon: Airport longitudes [deg].
        aptmaxrwy: Longest runway length per airport [m].
        apsize: Airport size.
        aptco: Two-character country codes (strings).
        aptelev: Airport elevations [m] above MSL.
        fir: FIR names.
        firlat0, firlon0, firlat1, firlon1: Start and end points of FIR
            border line segments [deg].
        coname: Country full names.
        cocode2: 2-character country codes.
        cocode3: 3-character country codes.
        conr: Country ICAO numbers.
        rwythresholds: Runway threshold positions [deg] and headings [deg]
            per airport and runway.

    Created by  : Jacco M. Hoekstra (TU Delft)
    """

    wplat: q.LatitudeDeg[np.ndarray]
    wplon: q.LongitudeDeg[np.ndarray]
    awfromlat: q.LatitudeDeg[np.ndarray]
    awfromlon: q.LongitudeDeg[np.ndarray]
    awtolat: q.LatitudeDeg[np.ndarray]
    awtolon: q.LongitudeDeg[np.ndarray]
    aptlat: q.LatitudeDeg[np.ndarray]
    aptlon: q.LongitudeDeg[np.ndarray]
    aptmaxrwy: q.LengthM[np.ndarray]
    wpelev: list[q.MslAltitudeM[float]]
    aptelev: list[q.MslAltitudeM[float]]

    def __init__(self, data_path: Path) -> None:
        """The navigation database: Contains waypoint, airport, airway, and sector data, but also
        geographical graphics data."""
        self.data_path = data_path
        self.reset()

    def reset(self) -> None:
        """(Re)load all navigation data from the package data directory."""
        # print("Loading global navigation database...")
        # wptdata, aptdata, awydata, firdata, codata, rwythresholds = load_navdata()

        nav_data_path = self.data_path

        wptdata = pd.read_parquet(nav_data_path / "waypoint.parquet")
        aptdata = pd.read_parquet(nav_data_path / "airport.parquet")
        awydata = pd.read_parquet(nav_data_path / "airway.parquet")
        codata = pd.read_parquet(nav_data_path / "country.parquet")

        with (nav_data_path / "fir.json").open() as f:
            firdata = json.load(f)
        with (nav_data_path / "runway_thresholds.json").open() as f:
            rwythresholds = json.load(f)

        # Get waypoint data
        self.wpid = _tolist(wptdata["wpid"])  # identifier (string)
        self.wplat = np.asarray(wptdata["wplat"], dtype=float)
        self.wplon = np.asarray(wptdata["wplon"], dtype=float)
        # TODO(abraham): rename this navigation-database category; Route.wptype is a
        # different WaypointType domain, and the shared name makes the two easy to confuse.
        self.wptype = _tolist(wptdata["wptype"])  # type (string)
        self.wpelev = _tolist(wptdata["wpelev"])  # elevation [m]
        self.wpvar = _tolist(wptdata["wpvar"])  # magn variation [deg]
        self.wpfreq = _tolist(wptdata["wpfreq"])  # frequency [kHz/MHz]
        self.wpdesc = _tolist(wptdata["wpdesc"])  # description

        # Get airway legs data
        self.awfromwpid = _tolist(awydata["awfromwpid"])  # identifier (string)
        self.awfromlat = np.asarray(awydata["awfromlat"], dtype=float)  # latitude [deg]
        self.awfromlon = np.asarray(awydata["awfromlon"], dtype=float)  # longitude [deg]
        self.awtowpid = _tolist(awydata["awtowpid"])  # identifier (string)
        self.awtolat = np.asarray(awydata["awtolat"], dtype=float)  # latitude [deg]
        self.awtolon = np.asarray(awydata["awtolon"], dtype=float)  # longitude [deg]
        self.awid = _tolist(awydata["awid"])  # airway identifier (string)
        self.awndir = _tolist(awydata["awndir"])  # number of directions (1 or 2)
        self.awlowfl = _tolist(awydata["awlowfl"])  # lower flight level (int)
        self.awupfl = _tolist(awydata["awupfl"])  # upper flight level (int)

        # Get airpoint data
        self.aptid = _tolist(aptdata["apid"])  # 4 char identifier (string)
        self.aptname = _tolist(aptdata["apname"])  # full name
        self.aptlat = np.asarray(aptdata["aplat"], dtype=float)  # latitude [deg]
        self.aptlon = np.asarray(aptdata["aplon"], dtype=float)  # longitude [deg]
        self.aptmaxrwy = np.asarray(aptdata["apmaxrwy"], dtype=float)  # max runway length [m]
        self.apsize = [AirportSize(int(value)) for value in _tolist(aptdata["aptype"])]
        self.aptco = _tolist(aptdata["apco"])  # two char country code (string)
        self.aptelev = _tolist(aptdata["apelev"])  # elevation in meters [m] MSL

        # Get FIR data
        self.fir = firdata["fir"]  # fir name
        self.firlat0 = firdata["firlat0"]  # start lat of a line of border
        self.firlon0 = firdata["firlon0"]  # start lon of a line of border
        self.firlat1 = firdata["firlat1"]  # end lat of a line of border
        self.firlon1 = firdata["firlon1"]  # end lon of a line of border

        # Get country code data
        self.coname = _tolist(codata["coname"])  # full name
        self.cocode2 = _tolist(codata["cocode2"])  # 2 chars
        self.cocode3 = _tolist(codata["cocode3"])  # 3 chars
        self.conr = _tolist(codata["conr"])  # country icao number

        self.rwythresholds = rwythresholds

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

    @command(name="DEFWPT")
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

    def describe_waypoint(self, name: str) -> Result[str, str]:
        """Describe a waypoint or report that its name is available."""
        normalized = name.upper()
        if normalized not in self.wpid:
            return Ok(f"Waypoint {normalized} does not yet exist.")
        index = len(self.wpid) - self.wpid[::-1].index(normalized) - 1
        description = f"{self.wpid[index]} : {self.wplat[index]},{self.wplon[index]}"
        if self.wptype[index]:
            description += f"  {self.wptype[index]}"
        return Ok(description)

    def defwpt(
        self,
        name: str,
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

        # Still here? So there is data, then we add this waypoint
        self.wpid.append(normalized)
        self.wplat = np.append(self.wplat, lat)
        self.wplon = np.append(self.wplon, lon)
        self.wptype.append("" if waypoint_type is None else waypoint_type.upper())
        self.wpelev.append(0.0)  # elevation [m]
        self.wpvar.append(0.0)  # magn variation [deg]
        self.wpfreq.append(0.0)  # frequency [kHz/MHz]
        self.wpdesc.append("Custom waypoint")  # description

        return Ok(f"{normalized} added to navdb.")

    def delwpt(self, name: str) -> Result[str, str]:
        """Delete a waypoint from the database.

        The last-added occurrence of the name is removed.

        Args:
            name: Waypoint name.
        """
        if self.wpid.count(name.upper()) <= 0:
            return Err(f"Waypoint {name.upper()} does not exist.")

        idx = len(self.wpid) - self.wpid[::-1].index(name.upper()) - 1  # Search from back of list

        del self.wpid[idx]  # wp name

        self.wplat = np.delete(self.wplat, idx)  # wp lat
        self.wplon = np.delete(self.wplon, idx)  # wp lon

        del self.wptype[idx]  # Waypoint type
        del self.wpelev[idx]  # elevation [m]
        del self.wpvar[idx]  # magn variation [deg]
        del self.wpfreq[idx]  # frequency [kHz/MHz]
        del self.wpdesc[idx]  # description

        return Ok(name.upper() + " deleted from navdb.")

    def getwpidx(self, txt: str, reference: LatLonReference | None = None) -> int | None:
        """Get waypoint index to access data.

        Args:
            txt: Waypoint identifier.
            reference: Optional reference position [deg]; when given, the
                occurrence closest to it is returned.

        Returns:
            Waypoint index, or None when not found.
        """
        name = txt.upper()
        try:
            i = self.wpid.index(name)
        except ValueError:
            return None

        # if no pos is specified, get first occurence
        if reference is None:
            return i

        # If pos is specified check for more and return closest
        else:
            idx = []
            idx.append(i)
            found = True
            while i < len(self.wpid) - 1 and found:
                try:
                    i = self.wpid.index(name, i + 1)
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
        txt: str,
        reference: LatLonReference | None = None,
        crit: q.DistanceM[float] = _COLOCATED_DISTANCE,
    ) -> list[int]:
        """Get indices of a waypoint and its co-located duplicates.

        Finds the occurrence of the identifier closest to the reference
        position, plus all other occurrences within a distance criterion.

        Args:
            txt: Waypoint identifier.
            reference: Optional reference position [deg].
            crit: Co-location distance criterion [m] (default 1 NM).

        Returns:
            Waypoint indices, empty when not found.
        """
        name = txt.upper()
        try:
            i = self.wpid.index(name)
        except ValueError:
            return []

        # if no pos is specified, get first occurence
        if reference is None:
            return [i]

        # If pos is specified check for more and return closest
        else:
            idx = findall(self.wpid, name)  # find indices of al occurences

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
                # Find co-located
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

    def getaptidx(self, txt: str) -> int | None:
        """Get the index of an airport by ICAO identifier.

        Args:
            txt: Airport identifier (e.g. "EHAM").

        Returns:
            Airport index, or None when not found.
        """
        try:
            return self.aptid.index(txt.upper())
        except ValueError:
            return None

    def getinear(
        self,
        wlat: q.LatitudeDeg,
        wlon: q.LongitudeDeg,
        lat: q.LatitudeDeg[float],
        lon: q.LongitudeDeg[float],
    ) -> int:  # lat,lon in degrees
        """Get the index of the entry nearest to a given position.

        Uses a fast flat-earth squared-distance comparison.

        Args:
            wlat: Array of latitudes to search [deg].
            wlon: Array of longitudes to search [deg].
            lat: Reference latitude [deg].
            lon: Reference longitude [deg].

        Returns:
            int: Index of the nearest entry.
        """
        # t0 = time.clock()
        wlat = np.asarray(wlat)
        wlon = np.asarray(wlon)
        f = np.cos(np.radians(lat))
        dlat = (wlat - lat + 180.0) % 360.0 - 180.0
        dlon = f * ((wlon - lon + 180.0) % 360.0 - 180.0)
        d2 = dlat * dlat + dlon * dlon
        idx = np.argmin(d2)
        # dt = time.clock()-t0
        # print dt
        return int(idx)

    def getwpinear(
        self, lat: q.LatitudeDeg[float], lon: q.LongitudeDeg[float]
    ) -> int:  # lat,lon in degrees
        """Get the index of the waypoint closest to position (lat, lon) [deg]."""
        return self.getinear(self.wplat, self.wplon, lat, lon)

    def getapinear(
        self, lat: q.LatitudeDeg[float], lon: q.LongitudeDeg[float]
    ) -> int:  # lat,lon in degrees
        """Get the index of the airport closest to position (lat, lon) [deg]."""
        return self.getinear(self.aptlat, self.aptlon, lat, lon)

    def getinside(
        self,
        wlat: q.LatitudeDeg,
        wlon: q.LongitudeDeg,
        lat0: q.LatitudeDeg[float],
        lat1: q.LatitudeDeg[float],
        lon0: q.LongitudeDeg[float],
        lon1: q.LongitudeDeg[float],
    ) -> list:
        """Get indices of positions inside the given lat/lon box.

        Args:
            wlat: Array of latitudes to filter [deg].
            wlon: Array of longitudes to filter [deg].
            lat0: First latitude bound [deg].
            lat1: Second latitude bound [deg].
            lon0: First longitude bound [deg].
            lon1: Second longitude bound [deg].

        Returns:
            list: Indices of the positions inside the box.
        """
        # t0 = time.clock()
        wlat = np.asarray(wlat)
        wlon = np.asarray(wlon)
        if lat0 < lat1:
            arr = np.where((wlat > lat0) * (wlat < lat1) * (wlon > lon0) * (wlon < lon1))
        else:
            arr = np.where((wlat > lat1) + (wlat < lat0) * (wlon > lon0) * (wlon < lon1))

        # dt = time.clock()-t0
        # print dt
        return list(arr[0])  # Get indices

    def getwpinside(
        self,
        lat0: q.LatitudeDeg[float],
        lat1: q.LatitudeDeg[float],
        lon0: q.LongitudeDeg[float],
        lon1: q.LongitudeDeg[float],
    ) -> list:
        """Get waypoint indices inside the given lat/lon box [deg]."""
        return self.getinside(self.wplat, self.wplon, lat0, lat1, lon0, lon1)

    def getapinside(
        self,
        lat0: q.LatitudeDeg[float],
        lat1: q.LatitudeDeg[float],
        lon0: q.LongitudeDeg[float],
        lon1: q.LongitudeDeg[float],
    ) -> list:
        """Get airport indices inside the given lat/lon box [deg]."""
        return self.getinside(self.aptlat, self.aptlon, lat0, lat1, lon0, lon1)

    # returns all runways of given airport
    def listairway(self, airwayid: str) -> list:
        """Return the waypoint sequence(s) of an airway.

        Collects all legs of the airway and chains them into ordered
        segments of waypoint identifiers; an airway may consist of
        multiple separate segments.

        Args:
            airwayid: Airway identifier (e.g. "UL620").

        Returns:
            list: List of segments, each a list of waypoint identifiers
            (empty when the airway is not found).
        """
        awkey = airwayid.upper()

        airway = []  # identifier of waypoint   0 .. N-1

        # Does this airway exist?
        if self.awid.count(awkey) > 0:
            # Collect leg indices
            i = 0
            found = True
            legs = []  # Alle leg incl. duplicate legs
            left = []  # wps in left column in file
            right = []  # wps in right coumn in file

            idx = findall(self.awid, awkey)
            for i in idx:
                newleg = self.awfromwpid[i] + "-" + self.awtowpid[i]
                if newleg not in legs:
                    legs.append(newleg)
                    left.append(self.awfromwpid[i])
                    right.append(self.awtowpid[i])

            # Not found: return
            if len(legs) == 0:
                return []

            # Count wps to see when we have all segments
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

                # Sort
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

                    # Add first wp to segment
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

                    # This segemnt done?
                    segready = (not found) or curwp == "" or nextwp == ""

                # Also add final nextwp of this segment
                segment.append(nextwp)

                # Airway cab have multiple separate segments
                airway.append(segment)

                # Ready for next segment
                left = wps[0]
                right = wps[1]

        return airway  # ,connect

    def listconnections(
        self, wpid: str, wplat: q.LatitudeDeg[float], wplon: q.LongitudeDeg[float]
    ) -> list:
        """Return the airway legs connecting to a given waypoint.

        Only legs whose stored endpoint lies within 10 nm of the given
        position are returned.

        Args:
            wpid: Waypoint identifier.
            wplat: Waypoint latitude [deg].
            wplon: Waypoint longitude [deg].

        Returns:
            list: List of [airway id, connected waypoint id] pairs.
        """
        # Return list of connecting airway legs
        connect = []

        # Check from-list first
        if wpid in self.awfromwpid:
            idx = findall(self.awfromwpid, wpid)
            for i in idx:
                newitem = [self.awid[i], self.awtowpid[i]]
                if (newitem not in connect) and geo.kwikdist(
                    self.awfromlat[i], self.awfromlon[i], wplat, wplon
                ) < q.nmi_to_m(10.0):
                    connect.append(newitem)

        # Check to-list nextt
        if wpid in self.awtowpid:
            idx = findall(self.awtowpid, wpid)
            for i in idx:
                newitem = [self.awid[i], self.awfromwpid[i]]
                if (newitem not in connect) and geo.kwikdist(
                    self.awtolat[i], self.awtolon[i], wplat, wplon
                ) < q.nmi_to_m(10.0):
                    connect.append(newitem)

        return connect  # return list of [awid,wpid]
