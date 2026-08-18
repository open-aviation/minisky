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

    Args:
        txt: Time text, with colon-separated fields.

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


def txt2bool(txt: str) -> bool:
    """Convert string to boolean.

    Args:
        txt: Boolean text: "true"/"yes"/"y"/"1"/"on" or
            "false"/"no"/"n"/"0"/"off" (case insensitive).

    Raises:
        ValueError: When the text is not a recognized boolean.
    """
    ltxt = txt.lower()
    if ltxt in ("true", "yes", "y", "1", "on"):
        return True
    if ltxt in ("false", "no", "n", "0", "off"):
        return False
    raise ValueError(f"Could not parse {txt} as bool.")


def i2txt(i: int, n: int) -> str:
    """Convert integer to string with leading zeros to make it n chars long"""
    return f"{i:0{n}d}"


def degto180(angle: q.AngleDeg) -> q.AngleDeg:
    """Change an angle to the domain [-180, 180) degrees."""
    return (angle + 180.0) % 360 - 180.0


# TODO(abraham): return None if parsing fails
def txt2lat(lattxt: str) -> q.LatitudeDeg[float]:
    """Convert a latitude text to degrees.

    Accepts decimal degrees or degrees/minutes/seconds separated by
    quotes or the degree symbol, with N/S prefix (North positive, South
    negative). Example inputs: "N52'14'13.5", "N52", "N52'", "-52.25".

    Returns:
        0.0 when DMS parsing fails.
    """
    txt = lattxt.upper().replace("N", "").replace("S", "-")  # North positive, South negative
    neg = txt.count("-") > 0

    # Use of "'" and '"' as delimiter for degrees/minutes/seconds
    # (also accept degree symbol chr(176))
    if txt.count("'") > 0 or txt.count('"') > 0 or txt.count(chr(176)) > 0:
        txt = txt.replace('"', "'").replace(chr(176), "'")  # replace " or degree symbol and  by a '
        degs = txt.split("'")
        div = 1
        lat = 0
        f = -1.0 if neg else 1.0
        for xtxt in degs:
            if len(xtxt) > 0:
                try:
                    lat = lat + f * abs(float(xtxt)) / float(div)
                    div = div * 60
                except ValueError:
                    print("txt2lat value error:", lattxt)
                    return 0.0
    else:
        lat = float(txt)
    return lat


# TODO(abraham): return None if parsing fails
def txt2lon(lontxt: str) -> q.LongitudeDeg[float]:
    """Convert a longitude text to degrees.

    Accepts decimal degrees or degrees/minutes/seconds separated by
    quotes or the degree symbol, with E/W prefix (East positive, West
    negative). Example inputs: "E004'23'10", "W65", "4.5".

    Returns:
        0.0 when DMS parsing fails.
    """
    # It should first be checked if lontxt is a regular float, to avoid removing
    # the 'e' in a scientific-notation number.
    try:
        lon = float(lontxt)

    # Leading E will trigger error ansd means simply East,just as  W = West = Negative
    except ValueError:
        txt = lontxt.upper().replace("E", "").replace("W", "-")  # East positive, West negative
        neg = txt.count("-") > 0

        # Use of "'" and '"' as delimiter for degrees/minutes/seconds
        # (also accept degree symbol chr(176)). Also "W002'"
        if txt.count("'") > 0 or txt.count('"') or txt.count(chr(176)) > 0:
            # replace " or degree symbol and  by a '
            txt = txt.replace('"', "'").replace(chr(176), "'")
            degs = txt.split("'")
            div = 1
            lon = 0.0
            f = -1.0 if neg else 1.0
            for xtxt in degs:
                if len(xtxt) > 0.0:
                    try:
                        lon = lon + f * abs(float(xtxt)) / float(div)
                    except ValueError:
                        print("txt2lon value error:", lontxt)
                        return 0.0

                div = div * 60
        else:  # Cope with "W65"without "'" or '"', also "-65" or "--65"
            try:
                neg = txt.count("-") > 0
                f = -1.0 if neg else 1.0
                lon = f * abs(float(txt))
            except ValueError:
                print("txt2lon value error:", lontxt)
                return 0.0

    return lon


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
