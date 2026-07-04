"""
Converters and other utility functions

Text-to-value and value-to-text converters, mainly used by the stack
argument parsers: altitudes, times, headings, speeds, vertical speeds,
booleans, latitudes/longitudes (including degrees-minutes-seconds
notation), colours, and angle-domain helpers. Text input is converted to
the SI units used internally by the simulator (m, m/s, s, deg).
"""

from time import gmtime, strftime

import numpy as np

from .aero import cas2tas, fpm, ft, kts, mach2tas
from .geo import magdec


def txt2alt(txt: str) -> float:
    """Convert text to altitude in meter: also FL300 => 30000. as float

    Accepts a flight level ("FL300" = 30000 ft) or a plain number in feet.

    Args:
        txt: Altitude text, e.g. "FL300" or "25000".

    Returns:
        Altitude [m] as float.

    Raises:
        ValueError: When the text cannot be parsed as an altitude.
    """
    # First check for FL otherwise feet
    try:
        if txt.upper()[:2] == "FL" and len(txt) >= 4:  # Syntax check Flxxx or Flxx
            return 100.0 * int(txt[2:]) * ft
        return float(txt) * ft
    except ValueError:
        pass
    raise ValueError(f'Could not parse "{txt}" as altitude"')


def tim2txt(t: float) -> str:
    """Convert time to timestring: HH:MM:SS.hh

    Args:
        t: Time [s].

    Returns:
        Formatted time string.
    """
    return strftime("%H:%M:%S.", gmtime(t)) + i2txt(int((t - int(t)) * 100.0), 2)


def txt2tim(txt: str) -> float:
    """Convert text to time in seconds:
    SS.hh
    MM:SS.hh
    HH.MM.SS.hh

    Args:
        txt: Time text, with colon-separated fields.

    Returns:
        Time [s] as float.

    Raises:
        ValueError: When the text cannot be parsed as a time.
    """
    timlst = txt.strip().split(":")

    try:
        # Always SS.hh
        t = float(timlst[-1])

        # MM
        if len(timlst) > 1 and timlst[-2]:
            t += 60.0 * int(timlst[-2])

        # HH
        if len(timlst) > 2 and timlst[-3]:
            t += 3600.0 * int(timlst[-3])

        return t
    except (ValueError, IndexError):
        raise ValueError(f'Could not parse "{txt}" as time') from None


def txt2bool(txt: str) -> bool:
    """Convert string to boolean.

    Args:
        txt: Boolean text: "true"/"yes"/"y"/"1"/"on" or
            "false"/"no"/"n"/"0"/"off" (case insensitive).

    Returns:
        bool: The parsed value.

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


def txt2hdg(txt: str, lat: float | None = None, lon: float | None = None) -> float:
    """Convert text to true or magnetic heading.
    Modified by : Yaofu Zhou

    A trailing "T" marks the value as true heading; a trailing "M" marks
    it as magnetic heading, which is converted to true heading using the
    magnetic declination at the given reference position.

    Args:
        txt: Heading text, e.g. "090", "090T", or "090M".
        lat: Reference latitude [deg] (required for magnetic headings).
        lon: Reference longitude [deg] (required for magnetic headings).

    Returns:
        True heading [deg] as float.

    Raises:
        ValueError: When a magnetic heading is given without a reference
            position.
    """
    heading = float(txt.upper().replace("T", "").replace("M", ""))

    if "M" in txt.upper():
        if None in (lat, lon):
            raise ValueError(
                "txt2hdg needs a reference latitude and longitude "
                "when a magnetic heading is parsed."
            )
        magnetic_declination = magdec(lat, lon)
        heading = (heading + magnetic_declination) % 360.0

    return heading


def txt2vs(txt: str) -> float:
    """Convert text to vertical speed.

    Arguments:
    - txt: text string representing vertical speed in feet per minute.

    Returns:
    - Vertical Speed (float) in meters per second.
    """
    return fpm * float(txt)


def txt2spd(txt: str) -> float:
    """Convert text to speed, keep type (EAS/TAS/MACH) unchanged.

    Values written with an "M" prefix (e.g. "M.8", "M0.8") or between 0.1
    and 1.0 are kept as Mach numbers; all other values are interpreted as
    knots and converted to m/s.

    Args:
        txt: Text string representing speed.

    Returns:
        Speed in meters per second [m/s], or Mach number [-].

    Raises:
        ValueError: When the text cannot be parsed as a speed.
    """
    try:
        txt = txt.upper()
        spd = float(txt.replace("M0.", ".").replace("M", ".").replace("..", "."))

        if not (0.1 < spd < 1.0 or txt.count("M") > 0):
            spd *= kts
        return spd
    except ValueError:
        raise ValueError(f"Could not parse {txt} as speed.") from None


def txt2tas(txt: str, h: float) -> float:
    """Convert speed text to true airspeed at a given altitude.

    Mach notation ("M.8", "M95", ".95") is converted with mach2tas; plain
    numbers are interpreted as CAS in knots and converted with cas2tas.

    Args:
        txt: Speed text.
        h: Altitude [m].

    Returns:
        True airspeed [m/s], or -1.0 when the text cannot be parsed.
    """
    if len(txt) == 0:
        return -1.0
    try:
        if txt[0] == "M":
            M_ = float(txt[1:])
            if M_ >= 20:  # Handle M95 notation as .95
                M_ = M_ * 0.01
            acspd = mach2tas(M_, h)  # m/s

        elif txt[0] == "." or (len(txt) >= 2 and txt[:2] == "0."):
            spd_ = float(txt)
            acspd = mach2tas(spd_, h)  # m/s

        else:
            spd_ = float(txt) * kts
            acspd = cas2tas(spd_, h)  # m/s
    except ValueError:
        return -1.0

    return acspd


def col2rgb(txt: str) -> tuple[int, int, int]:
    """Convert named color to R,G,B values (integer per component, 0-255).

    Args:
        txt: Colour name (e.g. "red", "amber"); unknown names default
            to white.

    Returns:
        tuple: (R, G, B) integer components in the range 0-255.
    """
    cols = {
        "black": (0, 0, 0),
        "white": (255, 255, 255),
        "green": (0, 255, 0),
        "red": (255, 0, 0),
        "blue": (0, 0, 255),
        "magenta": (255, 0, 255),
        "yellow": (240, 255, 127),
        "amber": (255, 255, 0),
        "cyan": (0, 255, 255),
    }
    try:
        rgb = cols[txt.lower().strip()]
    except KeyError:
        rgb = cols["white"]  # default

    return rgb


def degto180(angle: "float | np.ndarray") -> "float | np.ndarray":
    """Change an angle to the domain [-180, 180) degrees.

    Args:
        angle: Angle [deg].

    Returns:
        Equivalent angle [deg] in [-180, 180).
    """
    return (angle + 180.0) % 360 - 180.0


def radtopi(angle: float) -> float:
    """Change an angle to the domain [-pi, pi) radians.

    Args:
        angle: Angle [rad].

    Returns:
        Equivalent angle [rad] in [-pi, pi).
    """
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def txt2lat(lattxt: str) -> float:
    """Convert a latitude text to degrees.

    Accepts decimal degrees or degrees/minutes/seconds separated by
    quotes or the degree symbol, with N/S prefix (North positive, South
    negative). Example inputs: "N52'14'13.5", "N52", "N52'", "-52.25".

    Args:
        lattxt: Latitude text.

    Returns:
        Latitude [deg] as float (0.0 when parsing fails).
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
    # Return float


def txt2lon(lontxt: str) -> float:
    """Convert a longitude text to degrees.

    Accepts decimal degrees or degrees/minutes/seconds separated by
    quotes or the degree symbol, with E/W prefix (East positive, West
    negative). Example inputs: "E004'23'10", "W65", "4.5".

    Args:
        lontxt: Longitude text.

    Returns:
        Longitude [deg] as float (0.0 when parsing fails).
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


def lat2txt(lat: float) -> str:
    """Convert latitude into string (N/Sdegrees'minutes'seconds)."""
    d, m, s = float2degminsec(abs(lat))
    return "NS"[int(lat < 0)] + f"{int(d):02d}'{int(m):02d}'" + str(s) + '"'


def lon2txt(lon: float) -> str:
    """Convert longitude into string (E/Wdegrees'minutes'seconds)."""
    d, m, s = float2degminsec(abs(lon))
    return "EW"[int(lon < 0)] + f"{int(d):03d}'{int(m):02d}'" + str(s) + '"'


def latlon2txt(lat: float, lon: float) -> str:
    """Convert latitude and longitude in latlon string."""
    return lat2txt(lat) + "  " + lon2txt(lon)


def deg180(dangle: float) -> float:
    """Convert any difference in angles to interval [ -180,180 ).

    Args:
        dangle: Angle difference [deg].

    Returns:
        Equivalent angle difference [deg] in [-180, 180).
    """
    return (dangle + 180.0) % 360.0 - 180.0


def float2degminsec(x: float) -> tuple[int, float, float]:
    """Split a positive angle in degrees into whole degrees, minutes, and seconds.

    Args:
        x: Angle [deg] (positive).

    Returns:
        tuple: (degrees, minutes, seconds) of the angle.
    """
    deg = int(x)
    minutes = int(x * 60.0) - deg * 60.0
    sec = int(x * 3600.0) - deg * 3600.0 - minutes * 60.0
    return deg, minutes, sec
