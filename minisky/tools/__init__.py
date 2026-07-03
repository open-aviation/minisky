"""Aeronautics and geodesy tool library of MiniSky.

Bundles the utility modules used throughout the simulator: unit
conversions and the ISA atmosphere (aero), geodesy functions (geo, or the
compiled cgeo variant when available and preferred via settings),
text/value converters (convert), named area shapes and inside-tests
(areafilter), the navigation database (navdata), and position-text
parsing (position).
"""

from minisky.core import settings

# Register settings defaults
if settings.prefer_compiled:
    try:
        from . import cgeo as geo

        # print("Using compiled geo functions")
    except ImportError:
        from . import geo

        # print("Using Python-based geo functions")
else:
    from . import geo

    print("Using Python-based geo functions")


def init():
    """Initialise the tools package by loading the magnetic declination table."""
    # print("Reading magnetic variation data")
    geo.load_magnetic_declination()
