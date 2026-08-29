"""Parsing of position texts in MiniSky.

Translates stack-command position syntax-coordinates, navaids and fixes,
airport identifiers, runways such as `EHAM/RW06`, and aircraft callsigns into
typed latitude/longitude positions.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, NamedTuple, TypeAlias

from minisky import quantities as q
from minisky._internal.convert import txt2lat, txt2lon
from minisky._internal.result import Err, Ok, Result

if TYPE_CHECKING:
    from minisky._internal.navigation import AirportData, RunwayThresholdData, Waypoints
    from minisky._internal.traffic import Traffic


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
    waypoints: Waypoints,
    airports: AirportData,
    runway_thresholds: RunwayThresholdData,
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
            threshold = runway_thresholds[aptname][rwyname]
        except KeyError:
            return not_found
        return Ok(
            ResolvedRunwayPosition(
                float(threshold.latitude),
                float(threshold.longitude),
                float(threshold.heading),
            )
        )

    if (idx := airports.getidx(normalized)) is not None:
        return Ok(
            AirportPosition(
                float(airports.latitudes[idx]),
                float(airports.longitudes[idx]),
            )
        )

    if (idx := waypoints.getidx(normalized, _ReferencePosition(reflat, reflon))) is not None:
        return Ok(
            NavaidPosition(
                float(waypoints.latitudes[idx]),
                float(waypoints.longitudes[idx]),
            )
        )

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
