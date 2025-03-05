"""
Converters and other utility functions
"""

from time import gmtime, strftime

import numpy as np

from .aero import cas2tas, fpm, ft, kts, mach2tas
from .geo import magdec


def txt2alt(txt):
    """Convert text to altitude in meter: also FL300 => 30000. as float"""
    # First check for FL otherwise feet
    try:
        if txt.upper()[:2] == "FL" and len(txt) >= 4:  # Syntax check Flxxx or Flxx
            return 100.0 * int(txt[2:]) * ft
        return float(txt) * ft
    except ValueError:
        pass
    raise ValueError(f'Could not parse "{txt}" as altitude"')


def tim2txt(t):
    """Convert time to timestring: HH:MM:SS.hh"""
    return strftime("%H:%M:%S.", gmtime(t)) + i2txt(int((t - int(t)) * 100.0), 2)


def txt2tim(txt):
    """Convert text to time in seconds:
    SS.hh
    MM:SS.hh
    HH.MM.SS.hh
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
        raise ValueError(f'Could not parse "{txt}" as time')


def txt2bool(txt):
    """Convert string to boolean."""
    ltxt = txt.lower()
    if ltxt in ("true", "yes", "y", "1", "on"):
        return True
    if ltxt in ("false", "no", "n", "0", "off"):
        return False
    raise ValueError(f"Could not parse {txt} as bool.")


def i2txt(i, n):
    """Convert integer to string with leading zeros to make it n chars long"""
    return f"{i:0{n}d}"


def txt2hdg(txt, lat=None, lon=None):
    """Convert text to true or magnetic heading.
    Modified by : Yaofu Zhou"""
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


def txt2vs(txt):
    """Convert text to vertical speed.

    Arguments:
    - txt: text string representing vertical speed in feet per minute.

    Returns:
    - Vertical Speed (float) in meters per second.
    """
    return fpm * float(txt)


def txt2spd(txt):
    """Convert text to speed, keep type (EAS/TAS/MACH) unchanged.

    Arguments:
    - txt: text string representing speed

    Returns:
    - Speed in meters per second or Mach.
    """
    try:
        txt = txt.upper()
        spd = float(txt.replace("M0.", ".").replace("M", ".").replace("..", "."))

        if not (0.1 < spd < 1.0 or txt.count("M") > 0):
            spd *= kts
        return spd
    except ValueError:
        raise ValueError(f"Could not parse {txt} as speed.")


def txt2tas(txt, h):
    """Convert text to speed (EAS [kts]/MACH[-] to TAS[m/s])"""
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


def col2rgb(txt):
    """Convert named color to R,G,B values (integer per component, 0-255)"""
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


def degto180(angle):
    """Change to domain -180,180 degrees"""
    return (angle + 180.0) % 360 - 180.0


def radtopi(angle):
    """Change to domain -pi,pi radians"""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def txt2lat(lattxt):
    """txt2lat: input txt: N52'14'13.5 or N52 or N52'"""
    txt = (
        lattxt.upper().replace("N", "").replace("S", "-")
    )  # North positive, South negative
    neg = txt.count("-") > 0

    # Use of "'" and '"' as delimiter for degrees/minutes/seconds
    # (also accept degree symbol chr(176))
    if txt.count("'") > 0 or txt.count('"') > 0 or txt.count(chr(176)) > 0:
        txt = txt.replace('"', "'").replace(
            chr(176), "'"
        )  # replace " or degree symbol and  by a '
        degs = txt.split("'")
        div = 1
        lat = 0
        if neg:
            f = -1.0
        else:
            f = 1.0
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


def txt2lon(lontxt):
    """txt2lat: input txt: N52'14'13.5 or N52"""
    # It should first be checked if lontxt is a regular float, to avoid removing
    # the 'e' in a scientific-notation number.
    try:
        lon = float(lontxt)

    # Leading E will trigger error ansd means simply East,just as  W = West = Negative
    except ValueError:
        txt = (
            lontxt.upper().replace("E", "").replace("W", "-")
        )  # East positive, West negative
        neg = txt.count("-") > 0

        # Use of "'" and '"' as delimiter for degrees/minutes/seconds
        # (also accept degree symbol chr(176)). Also "W002'"
        if txt.count("'") > 0 or txt.count('"') or txt.count(chr(176)) > 0:
            # replace " or degree symbol and  by a '
            txt = txt.replace('"', "'").replace(chr(176), "'")
            degs = txt.split("'")
            div = 1
            lon = 0.0
            if neg:
                f = -1.0
            else:
                f = 1.0
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
                if neg:
                    f = -1.0
                else:
                    f = 1.0
                lon = f * abs(float(txt))
            except ValueError:
                print("txt2lon value error:", lontxt)
                return 0.0

    return lon


def lat2txt(lat):
    """Convert latitude into string (N/Sdegrees'minutes'seconds)."""
    d, m, s = float2degminsec(abs(lat))
    return "NS"[lat < 0] + "%02d'%02d'" % (int(d), int(m)) + str(s) + '"'


def lon2txt(lon):
    """Convert longitude into string (E/Wdegrees'minutes'seconds)."""
    d, m, s = float2degminsec(abs(lon))
    return "EW"[lon < 0] + "%03d'%02d'" % (int(d), int(m)) + str(s) + '"'


def latlon2txt(lat, lon):
    """Convert latitude and longitude in latlon string."""
    return lat2txt(lat) + "  " + lon2txt(lon)


def deg180(dangle):
    """Convert any difference in angles to interval [ -180,180 )"""
    return (dangle + 180.0) % 360.0 - 180.0


def float2degminsec(x):
    """Convert an angle into a string describing the angle in degrees,
    minutes, and seconds."""
    deg = int(x)
    minutes = int(x * 60.0) - deg * 60.0
    sec = int(x * 3600.0) - deg * 3600.0 - minutes * 60.0
    return deg, minutes, sec
