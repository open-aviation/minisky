"""
Converters and other utility functions

Text-to-value and value-to-text converters still shared by command parsing
and simulation code: times, vertical speeds, booleans, latitude/longitude
(including DMS notation), and angle-domain helpers. Text input is converted
to the SI units used internally by the simulator (m, m/s, s, deg).
"""

from time import gmtime, strftime
from typing import NamedTuple

from minisky import quantities as q


def tim2txt(t: q.DurationS[float]) -> str:
    """Convert time to timestring: HH:MM:SS.hh"""
    return strftime("%H:%M:%S.", gmtime(t)) + i2txt(int((t - int(t)) * 100.0), 2)


def txt2tim(txt: str) -> q.DurationS[float]:
    """Convert text to time in seconds:
    SS.hh
    MM:SS.hh
    HH.MM.SS.hh

    Raises:
        ValueError: When the text cannot be parsed as a time.
    """
    timlst = txt.strip().split(":")

    try:
        # Always SS.hh
        t = float(timlst[-1])

        # MM
        if len(timlst) > 1 and timlst[-2]:
            t += q.min_to_s(int(timlst[-2]))

        # HH
        if len(timlst) > 2 and timlst[-3]:
            t += q.hour_to_s(int(timlst[-3]))

        return t
    except (ValueError, IndexError):
        raise ValueError(f'Could not parse "{txt}" as time') from None


def i2txt(i: int, n: int) -> str:
    """Convert integer to string with leading zeros to make it n chars long"""
    return f"{i:0{n}d}"


def degto180(angle: q.AngleDeg) -> q.AngleDeg:
    """Change an angle to the domain [-180, 180) degrees."""
    return (angle + 180.0) % 360 - 180.0


def _parse_coordinate(text: str, positive: str, negative: str) -> float:
    raw = text.strip().upper()
    try:
        return float(raw)
    except ValueError:
        pass

    sign = 1.0
    if raw.startswith(positive):
        raw = raw[1:]
    elif raw.startswith((negative, "-")):
        sign = -1.0
        raw = raw[1:]

    raw = raw.replace('"', "'").replace(chr(176), "'")
    parts = [part for part in raw.split("'") if part]
    if not parts:
        raise ValueError(f"could not parse coordinate {text!r}")

    try:
        values = [abs(float(part)) for part in parts]
    except ValueError:
        raise ValueError(f"could not parse coordinate {text!r}") from None
    if len(values) > 3:
        raise ValueError(f"could not parse coordinate {text!r}")

    return sign * sum(value / (60**index) for index, value in enumerate(values))


def txt2lat(lattxt: str) -> q.LatitudeDeg[float]:
    """Convert latitude text to decimal degrees.

    Raises:
        ValueError: When the text cannot be parsed as a latitude.
    """
    return _parse_coordinate(lattxt, "N", "S")


def txt2lon(lontxt: str) -> q.LongitudeDeg[float]:
    """Convert longitude text to decimal degrees.

    Raises:
        ValueError: When the text cannot be parsed as a longitude.
    """
    return _parse_coordinate(lontxt, "E", "W")


def lat2txt(lat: q.LatitudeDeg[float]) -> str:
    """Convert latitude into string (N/Sdegrees'minutes'seconds)."""
    d, m, s = float2degminsec(abs(lat))
    return "NS"[int(lat < 0)] + f"{int(d):02d}'{int(m):02d}'" + str(s) + '"'


def lon2txt(lon: q.LongitudeDeg[float]) -> str:
    """Convert longitude into string (E/Wdegrees'minutes'seconds)."""
    d, m, s = float2degminsec(abs(lon))
    return "EW"[int(lon < 0)] + f"{int(d):03d}'{int(m):02d}'" + str(s) + '"'


def latlon2txt(lat: q.LatitudeDeg[float], lon: q.LongitudeDeg[float]) -> str:
    """Convert latitude and longitude in latlon string."""
    return lat2txt(lat) + "  " + lon2txt(lon)


class DegreesMinutesSeconds(NamedTuple):
    degrees: int
    """Whole degrees [deg]."""
    minutes: float
    """Whole arcminutes [arcmin]."""
    seconds: float
    """Whole arcseconds [arcsec]."""


def float2degminsec(x: q.AngleDeg[float]) -> DegreesMinutesSeconds:
    """Split a positive angle in degrees into whole degrees, minutes, and seconds."""
    deg = int(x)
    fractional_arcminutes = q.deg_to_arcmin(x - deg)
    minutes = int(fractional_arcminutes)
    seconds = int(q.arcmin_to_arcsec(fractional_arcminutes - minutes))
    return DegreesMinutesSeconds(deg, float(minutes), float(seconds))
