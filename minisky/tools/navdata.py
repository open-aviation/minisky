"""Navigation database of MiniSky.

Loads waypoint, airport, airway, FIR, and country data from the package
data directory and provides lookup functions to find navaids and airports
by identifier or position. The global Navdatabase instance is available
as ``minisky.navdb``; it backs the DEFWPT stack command and every position
argument that references a navaid, airport, or runway.
"""

import json
from typing import Any

import numpy as np
import pandas as pd

import minisky
from minisky.tools import geo
from minisky.tools.aero import nm


def _tolist(column: Any) -> list:
    """Return a pandas column as a plain Python list.

    Wrapper around ``Series.to_list()`` that gives a concrete ``list``
    return type (the pandas ``__getitem__`` overloads otherwise widen the
    result to include ``str``).
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
        aptype: Airport type (1=large, 2=medium, 3=small).
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

    def __init__(self) -> None:
        """The navigation database: Contains waypoint, airport, airway, and sector data, but also
        geographical graphics data."""
        # Variables are initialized in reset()
        self.reset()

    def reset(self) -> None:
        """(Re)load all navigation data from the package data directory."""
        # print("Loading global navigation database...")
        # wptdata, aptdata, awydata, firdata, codata, rwythresholds = load_navdata()

        nav_data_path = minisky.data("navigation")

        wptdata = pd.read_parquet(nav_data_path / "waypoint.parquet")
        aptdata = pd.read_parquet(nav_data_path / "airport.parquet")
        awydata = pd.read_parquet(nav_data_path / "airway.parquet")
        codata = pd.read_parquet(nav_data_path / "country.parquet")

        with open(nav_data_path / "fir.json") as f:
            firdata = json.load(f)
        with open(nav_data_path / "runway_thresholds.json") as f:
            rwythresholds = json.load(f)

        # Get waypoint data
        self.wpid = _tolist(wptdata["wpid"])  # identifier (string)
        # wplat/wplon start as lists but are reassigned to ndarrays by addwpt
        self.wplat: Any = _tolist(wptdata["wplat"])  # latitude [deg]
        self.wplon: Any = _tolist(wptdata["wplon"])  # longitude [deg]
        self.wptype = _tolist(wptdata["wptype"])  # type (string)
        self.wpelev = _tolist(wptdata["wpelev"])  # elevation [m]
        self.wpvar = _tolist(wptdata["wpvar"])  # magn variation [deg]
        self.wpfreq = _tolist(wptdata["wpfreq"])  # frequency [kHz/MHz]
        self.wpdesc = _tolist(wptdata["wpdesc"])  # description

        # Get airway legs data
        self.awfromwpid = _tolist(awydata["awfromwpid"])  # identifier (string)
        self.awfromlat = _tolist(awydata["awfromlat"])  # latitude [deg]
        self.awfromlon = _tolist(awydata["awfromlon"])  # longitude [deg]
        self.awtowpid = _tolist(awydata["awtowpid"])  # identifier (string)
        self.awtolat = _tolist(awydata["awtolat"])  # latitude [deg]
        self.awtolon = _tolist(awydata["awtolon"])  # longitude [deg]
        self.awid = _tolist(awydata["awid"])  # airway identifier (string)
        self.awndir = _tolist(awydata["awndir"])  # number of directions (1 or 2)
        self.awlowfl = _tolist(awydata["awlowfl"])  # lower flight level (int)
        self.awupfl = _tolist(awydata["awupfl"])  # upper flight level (int)

        # Get airpoint data
        self.aptid = _tolist(aptdata["apid"])  # 4 char identifier (string)
        self.aptname = _tolist(aptdata["apname"])  # full name
        self.aptlat = _tolist(aptdata["aplat"])  # latitude [deg]
        self.aptlon = _tolist(aptdata["aplon"])  # longitude [deg]
        self.aptmaxrwy = _tolist(aptdata["apmaxrwy"])  # max runway length [m]
        self.aptype = _tolist(aptdata["aptype"])  # 1=large, 2=medium, 3=small
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

    def defwpt(self, name: str | None = None, lat: float | None = None, lon: float | None = None, wptype: str | None = None) -> tuple[bool, str]:
        """DEFWPT: Define, inspect, or delete a scenario-specific waypoint.

        Without lat/lon, information about the existing waypoint is
        returned; with wptype DEL/DELETE the waypoint is deleted;
        otherwise the waypoint is appended to the database and shown on
        the screen.

        Args:
            name: Waypoint name (must not be purely numeric).
            lat: Latitude [deg].
            lon: Longitude [deg].
            wptype: Optional waypoint type (e.g. FIX, VOR, DME, NDB), or
                DEL/DELETE to remove the waypoint.

        Returns:
            tuple: (success (bool), message (str)).
        """
        # Prevent polluting the database: check arguments
        if name == None or name == "":
            return False, "Insufficient arguments"
        elif name.isdigit():
            return False, "Name needs to start with an alphabetical character"

        # DEL command: give info on waypoint (shudl work wit or without lat,lon, may be clicked by accident
        elif (wptype != None and (wptype.upper() == "DEL" or wptype.upper() == "DELETE")) or (
            type(lon) == str and (lon.upper() == "DEL" or lon.upper() == "DELETE")
        ):
            return self.delwpt(name)

        # No data: give info on waypoint
        elif lat == None or lon == None:
            reflat, reflon = minisky.scr.getviewctr()
            if self.wpid.count(name.upper()) > 0:
                i = self.getwpidx(name.upper(), reflat, reflon)
                txt = self.wpid[i] + " : " + str(self.wplat[i]) + "," + str(self.wplon[i])
                if len(self.wptype[i]) > 0:
                    txt = txt + "  " + self.wptype[i]
                return True, txt

            # Waypoint name is free
            else:
                return True, "Waypoint " + name.upper() + " does not yet exist."

        # Still here? So there is data, then we add this waypoint
        self.wpid.append(name.upper())
        self.wplat = np.append(self.wplat, lat)
        self.wplon = np.append(self.wplon, lon)

        if wptype == None:
            self.wptype.append("")
        else:
            self.wptype.append(wptype)

        self.wpelev.append(0.0)  # elevation [m]
        self.wpvar.append(0.0)  # magn variation [deg]
        self.wpfreq.append(0.0)  # frequency [kHz/MHz]
        self.wpdesc.append("Custom waypoint")  # description

        # Update screen info
        minisky.scr.addnavwpt(name.upper(), lat, lon)

        return True, name.upper() + " added to navdb."

    def delwpt(self, name: str | None = None) -> tuple[bool, str]:
        """Delete a waypoint from the database.

        The last-added occurrence of the name is removed.

        Args:
            name: Waypoint name.

        Returns:
            tuple: (success (bool), message (str)).
        """
        if name is None:
            return False, "No waypoint name given"

        if self.wpid.count(name.upper()) <= 0:
            return False, "Waypoint " + name.upper() + " does not exist."

        idx = len(self.wpid) - self.wpid[::-1].index(name.upper()) - 1  # Search from back of list

        del self.wpid[idx]  # wp name

        self.wplat = np.delete(self.wplat, idx)  # wp lat
        self.wplon = np.delete(self.wplon, idx)  # wp lon

        del self.wptype[idx]  # Waypoint type
        del self.wpelev[idx]  # elevation [m]
        del self.wpvar[idx]  # magn variation [deg]
        del self.wpfreq[idx]  # frequency [kHz/MHz]
        del self.wpdesc[idx]  # description

        # Update screen info 9delete necessary there?)
        minisky.scr.removenavwpt(name.upper())

        return True, name.upper() + " deleted from navdb."

    def getwpidx(self, txt: str, reflat: float = 999999.0, reflon: float = 999999) -> int:
        """Get waypoint index to access data.

        Args:
            txt: Waypoint identifier.
            reflat: Optional reference latitude [deg]; when given, the
                occurrence closest to the reference position is returned.
            reflon: Optional reference longitude [deg].

        Returns:
            int: Waypoint index, or -1 when not found.
        """
        name = txt.upper()
        try:
            i = self.wpid.index(name)
        except ValueError:
            return -1

        # if no pos is specified, get first occurence
        if not reflat < 99999.0:
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
                dmin = geo.kwikdist(reflat, reflon, self.wplat[imin], self.wplon[imin])
                for i in idx[1:]:
                    d = geo.kwikdist(reflat, reflon, self.wplat[i], self.wplon[i])
                    if d < dmin:
                        imin = i
                        dmin = d
                return imin

    def getwpindices(
        self, txt: str, reflat: float = 999999.0, reflon: float = 999999, crit: float = 1852.0
    ) -> list:
        """Get indices of a waypoint and its co-located duplicates.

        Finds the occurrence of the identifier closest to the reference
        position, plus all other occurrences within a distance criterion.

        Args:
            txt: Waypoint identifier.
            reflat: Optional reference latitude [deg].
            reflon: Optional reference longitude [deg].
            crit: Co-location distance criterion [m] (default 1852 m = 1 nm).

        Returns:
            list: Waypoint indices ([-1] when not found).
        """
        name = txt.upper()
        try:
            i = self.wpid.index(name)
        except ValueError:
            return [-1]

        # if no pos is specified, get first occurence
        if not reflat < 99999.0:
            return [i]

        # If pos is specified check for more and return closest
        else:
            idx = findall(self.wpid, name)  # find indices of al occurences

            if len(idx) == 1:
                return [idx[0]]
            else:
                imin = idx[0]
                dmin = geo.kwikdist(reflat, reflon, self.wplat[imin], self.wplon[imin])
                for i in idx[1:]:
                    d = geo.kwikdist(reflat, reflon, self.wplat[i], self.wplon[i])
                    if d < dmin:
                        imin = i
                        dmin = d
                # Find co-located
                indices = [imin]
                for i in idx:
                    if i != imin:
                        dist = nm * geo.kwikdist(
                            self.wplat[i],
                            self.wplon[i],
                            self.wplat[imin],
                            self.wplon[imin],
                        )
                        if dist <= crit:
                            indices.append(i)

                return indices

    def getaptidx(self, txt: str) -> int:
        """Get the index of an airport by ICAO identifier.

        Args:
            txt: Airport identifier (e.g. "EHAM").

        Returns:
            int: Airport index, or -1 when not found.
        """
        try:
            return self.aptid.index(txt.upper())
        except ValueError:
            return -1

    def getinear(self, wlat: "np.ndarray | list", wlon: "np.ndarray | list", lat: float, lon: float) -> int:  # lat,lon in degrees
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

    def getwpinear(self, lat: float, lon: float):  # lat,lon in degrees
        """Get the index of the waypoint closest to position (lat, lon) [deg]."""
        return self.getinear(self.wplat, self.wplon, lat, lon)

    def getapinear(self, lat: float, lon: float):  # lat,lon in degrees
        """Get the index of the airport closest to position (lat, lon) [deg]."""
        return self.getinear(self.aptlat, self.aptlon, lat, lon)

    def getinside(self, wlat: "np.ndarray | list", wlon: "np.ndarray | list", lat0: float, lat1: float, lon0: float, lon1: float) -> list:
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

    def getwpinside(self, lat0: float, lat1: float, lon0: float, lon1: float) -> list:
        """Get waypoint indices inside the given lat/lon box [deg]."""
        return self.getinside(self.wplat, self.wplon, lat0, lat1, lon0, lon1)

    def getapinside(self, lat0: float, lat1: float, lon0: float, lon1: float) -> list:
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

    def listconnections(self, wpid: str, wplat: float, wplon: float) -> list:
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
                ) < 10.0:
                    connect.append(newitem)

        # Check to-list nextt
        if wpid in self.awtowpid:
            idx = findall(self.awtowpid, wpid)
            for i in idx:
                newitem = [self.awid[i], self.awfromwpid[i]]
                if (newitem not in connect) and geo.kwikdist(
                    self.awtolat[i], self.awtolon[i], wplat, wplon
                ) < 10.0:
                    connect.append(newitem)

        return connect  # return list of [awid,wpid]
