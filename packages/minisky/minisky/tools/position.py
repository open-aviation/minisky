"""Parsing of position texts in MiniSky.

Translates the position notations used in stack commands - lat/lon pairs
(decimal or degrees/minutes/seconds), navaid and fix names, airport ICAO
identifiers, runways (e.g. "EHAM/RW06"), and aircraft callsigns - into
latitude/longitude coordinates [deg] via the Position class.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple

from minisky.result import Err, Ok, Result

from .convert import txt2lat, txt2lon

if TYPE_CHECKING:
    from minisky.tools.navdata import Navdatabase
    from minisky.traffic import Traffic


class _ReferencePosition(NamedTuple):
    lat: float
    lon: float


def txt2pos(
    name: str,
    reflat: float,
    reflon: float,
    navigation: Navdatabase,
    traffic: Traffic,
) -> Result[Position, str]:
    """Parse a position text into a Position object.

    Args:
        name: Position text: lat/lon pair, navaid/fix, airport, runway
            (e.g. "EHAM/RW06"), or aircraft callsign.
        reflat: Reference latitude [deg], used to resolve ambiguous names.
        reflon: Reference longitude [deg], used to resolve ambiguous names.
    """
    pos = Position(name.upper().strip(), reflat, reflon, navigation, traffic)
    if not pos.error:
        return Ok(pos)
    return Err(name + " not found in database")


def islat(txt: str) -> bool:
    """Check whether a text looks like a latitude.

    Accepts decimal or degrees/minutes/seconds notation, with an optional
    leading N or S and sign.

    Args:
        txt: Candidate latitude text.

    Returns:
        bool: True when the text has a latitude-like format.
    """
    # Is it a latitude-like format or not?

    # Take out non-digit chars which are allowed
    testtxt = (
        txt.upper()
        .strip()
        .strip("-")
        .strip("+")
        .strip("\n")
        .strip(",")
        .replace('"', "")
        .replace("'", "")
        .replace(".", "")
    )

    # Take away one leading N or S if present before other chars
    if (testtxt[0] == "N" or testtxt[0] == "S") and len(testtxt) > 1:
        testtxt = testtxt[1:]

    try:
        float(testtxt)
    except ValueError:
        return False
    return True


# TODO(abraham): return a Result of typed position variants instead of mutating an
# object with string type/error flags and conditionally initialized fields
class Position:
    """Position class: container for position data

    Resolves a position text into coordinates, trying in order: lat/lon
    pair, runway ("apt/RWxx"), airport, navaid/fix (closest occurrence to
    the reference position), aircraft callsign, and pan direction keyword.

    Attributes:
        name: Source name (empty for plain lat/lon and aircraft positions).
        lat: Latitude [deg] (set when parsing succeeded).
        lon: Longitude [deg] (set when parsing succeeded).
        type: Position type: "latlon", "rwy", "apt", "nav", or "dir".
        refhdg: Runway heading [deg] for runway positions, else None.
        error: True when the text could not be resolved to a position.
    """

    # position types: "latlon","nav","apt","rwy"

    # Initialize using text
    def __init__(
        self,
        name: str,
        reflat: float,
        reflon: float,
        navigation: Navdatabase,
        traffic: Traffic,
    ) -> None:
        """Resolve a position text relative to a reference position.

        Args:
            name: Position text (upper case).
            reflat: Reference latitude [deg].
            reflon: Reference longitude [deg].
        """
        self.name = name  # default: copy source name
        self.error = False  # we're optmistic about our succes
        self.refhdg = None

        # lat,lon type ?
        if name.count(",") > 0:  # lat,lon or apt,rwy type
            txt1, txt2 = name.split(",")
            if islat(txt1):
                self.lat = txt2lat(txt1)
                self.lon = txt2lon(txt2)
                self.name = ""
                self.type = "latlon"

        # runway type ? "EHAM/RW06","EHGG/RWY27"
        elif name.count("/RW") > 0:
            try:
                aptname, rwytxt = name.split("/RW")
                rwyname = rwytxt.lstrip("Y").upper()  # remove Y and spaces
                self.lat, self.lon, self.refhdg = navigation.rwythresholds[aptname][rwyname]
            except KeyError:
                self.error = True
            self.type = "rwy"

        # airport?
        elif navigation.aptid.count(name) > 0:
            idx = navigation.aptid.index(name.upper())

            self.lat = navigation.aptlat[idx]
            self.lon = navigation.aptlon[idx]
            self.type = "apt"

        # fix or navaid?
        elif navigation.wpid.count(name) > 0:
            idx = navigation.getwpidx(name, _ReferencePosition(reflat, reflon))
            assert idx is not None
            self.lat = navigation.wplat[idx]
            self.lon = navigation.wplon[idx]
            self.type = "nav"

        # aircraft id?
        elif name in traffic.callsign:
            idx = traffic.idx(name)
            assert idx is not None
            self.name = ""
            self.type = "latlon"
            self.lat = traffic.lat[idx]
            self.lon = traffic.lon[idx]

            # exception for pan, check for LEFT, RIGHT, ABOVE or DOWN
        elif name.upper() in ["LEFT", "RIGHT", "ABOVE", "DOWN"]:
            self.lat = reflat
            self.lon = reflon
            self.type = "dir"

        # Not used now, but save this code for future use
        #            # Make a N52E004 type waypoint name
        #            clat = "SN"[lat>0]
        #            clon = "WE"[lon>0]
        #            name = clat + "%02d"%int(abs(round(lat))) + \
        #                   clon + "%03d"%int(abs(round(lon)))
        else:
            self.error = True
            # raise error with missing data... (empty position object)
