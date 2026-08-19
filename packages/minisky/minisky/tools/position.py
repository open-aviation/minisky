"""Parsing of position texts in MiniSky.

Translates stack-command position syntax-coordinates, navaids and fixes,
airport identifiers, runways such as `EHAM/RW06`, and aircraft callsigns into
typed latitude/longitude positions.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, NamedTuple, TypeAlias

from minisky import quantities as q
from minisky.result import Err, Ok, Result
from minisky.tools.convert import txt2lat, txt2lon

if TYPE_CHECKING:
    from minisky.tools.navdata import Navdatabase
    from minisky.traffic.traffic import Traffic


class _ReferencePosition(NamedTuple):
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]


@dataclass(frozen=True, slots=True)
class LatLonPosition:
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]


@dataclass(frozen=True, slots=True)
class ResolvedRunwayPosition:
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]
    heading: q.TrueHeadingDegrees[float]


@dataclass(frozen=True, slots=True)
class AirportPosition:
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]


@dataclass(frozen=True, slots=True)
class NavaidPosition:
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]


@dataclass(frozen=True, slots=True)
class AircraftPosition:
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]


@dataclass(frozen=True, slots=True)
class DirectionPosition:
    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]


Position: TypeAlias = (
    LatLonPosition
    | ResolvedRunwayPosition
    | AirportPosition
    | NavaidPosition
    | AircraftPosition
    | DirectionPosition
)


def txt2pos(
    name: str,
    reflat: q.LatitudeDeg[float],
    reflon: q.LongitudeDeg[float],
    navigation: Navdatabase,
    traffic: Traffic,
) -> Result[Position, str]:
    """Resolve position text relative to a reference position."""
    normalized = name.upper().strip()
    not_found = Err(name + " not found in database")

    if "," in normalized:
        parts = normalized.split(",")
        if len(parts) != 2 or not islat(parts[0]):
            return not_found
        try:
            return Ok(LatLonPosition(txt2lat(parts[0]), txt2lon(parts[1])))
        except (IndexError, ValueError):
            return not_found

    if "/RW" in normalized:
        aptname, rwytxt = normalized.split("/RW", 1)
        rwyname = rwytxt.lstrip("Y").upper()
        try:
            lat, lon, heading = navigation.rwythresholds[aptname][rwyname]
        except KeyError:
            return not_found
        return Ok(ResolvedRunwayPosition(float(lat), float(lon), float(heading)))

    if normalized in navigation.aptid:
        idx = navigation.aptid.index(normalized)
        return Ok(AirportPosition(float(navigation.aptlat[idx]), float(navigation.aptlon[idx])))

    if normalized in navigation.wpid:
        idx = navigation.getwpidx(normalized, _ReferencePosition(reflat, reflon))
        assert idx is not None
        return Ok(NavaidPosition(float(navigation.wplat[idx]), float(navigation.wplon[idx])))

    if normalized in traffic.callsign:
        idx = traffic.idx(normalized)
        assert idx is not None
        return Ok(AircraftPosition(float(traffic.lat[idx]), float(traffic.lon[idx])))

    if normalized in {"LEFT", "RIGHT", "ABOVE", "DOWN"}:
        return Ok(DirectionPosition(reflat, reflon))

    return not_found


def islat(txt: str) -> bool:
    """Check whether a text looks like a latitude.

    Accepts decimal or degrees/minutes/seconds notation, with an optional
    leading N or S and sign.
    """
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
    if not testtxt:
        return False
    if testtxt[0] in {"N", "S"} and len(testtxt) > 1:
        testtxt = testtxt[1:]

    try:
        float(testtxt)
    except ValueError:
        return False
    return True
