"""Aeronautics and geodesy tool library of MiniSky.

Bundles the utility modules used throughout the simulator: unit
conversions and the ISA atmosphere (aero), geodesy functions (geo, or the
compiled cgeo variant when available),
text/value converters (convert), named area shapes and inside-tests
(areafilter), the navigation database (navdata), and position-text
parsing (position).
"""

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from . import geo as geo
else:
    try:
        from . import cgeo as geo  # type: ignore[import-not-found]
    except ImportError:
        from . import geo

from . import aero, areafilter, convert, navdata, position  # noqa: E402
