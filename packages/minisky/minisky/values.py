"""Semantic runtime values used at parser and vectorized-state boundaries.

MiniSky keeps ordinary simulation arrays as typed numerical quantities from
[`minisky.quantities`][minisky.quantities]. These wrappers are for cases where
multiple values share the same Python carrier but their reference or variant
must survive at runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from minisky import quantities as q

#
# airspeed
#


class AirspeedKind(IntEnum):
    """Discriminant for required selected airspeed state."""

    CAS = 0
    """[Calibrated airspeed in m/s][minisky.values.CasMps]."""
    MACH = 1
    """[Mach][minisky.values.Mach]."""


class OptionalAirspeedKind(IntEnum):
    """Discriminant for optional airspeed state."""

    NONE = 0
    CAS = 1
    """[Calibrated airspeed in m/s][minisky.values.CasMps]."""
    MACH = 2
    """[Mach][minisky.values.Mach]."""


@dataclass(frozen=True, slots=True)
class CasMps:
    """Calibrated airspeed normalized to metres per second.

    Note that in minisky, IAS is used interchangeably with CAS, assuming zero
    instrument and position errors.
    """

    value: q.CalibratedAirspeedMps[float]


@dataclass(frozen=True, slots=True)
class Mach:
    """Mach number preserved as a dimensionless value."""

    value: q.MachNumber[float]


#
# altitude / height
#


@dataclass(frozen=True, slots=True)
class StdPressureAltM:
    """Barometric pressure altitude on the standard-pressure reference.

    QNE is the standard altimeter setting and flight levels are the operational
    designation for the standard-pressure reference (1013.25 hPa).

    In ICAO Field 15:

    - `F + 3 digits` = flight level, digits in hundreds of feet, and
    - `S + 4 digits` = standard metric level, digits in tens of metres
    """

    value: q.PressureAltitudeM[float]


@dataclass(frozen=True, slots=True)
class MslAltM:
    """Altitude above mean sea level.

    QNH is the altimeter subscale pressure setting chosen so a correctly
    calibrated pressure altimeter indicates altitude relative to mean sea
    level.

    In ICAO Field 15:

    - `A + 3 digits` = altitude in hundreds of feet
    - `M + 4 digits` = altitude in tens of metres
    """

    value: q.MslAltitudeM[float]


# TODO(abraham): we don't know how to handle local/ground datum (see #22)
# keeping them internal for now
@dataclass(frozen=True, slots=True)
class _QfeHeightM:  # pyright: ignore[reportUnusedClass]
    """Barometric height above the local QFE reference datum."""

    value: q.BarometricHeightM[float]


@dataclass(frozen=True, slots=True)
class _AglHeightM:  # pyright: ignore[reportUnusedClass]
    """Height above the ground surface directly beneath the aircraft."""

    value: q.AglHeightM[float]


#
# other
#


@dataclass(frozen=True, slots=True)
class TrueHeadingDeg:
    """Heading relative to true north in degrees."""

    degrees: q.TrueHeadingDegrees[float]


@dataclass(frozen=True, slots=True)
class MagneticHeadingDeg:
    """Heading relative to magnetic north in degrees."""

    degrees: q.MagneticHeadingDegrees[float]


@dataclass(frozen=True, slots=True)
class GroundTrackDeg:
    """Ground-track direction relative to true north in degrees."""

    degrees: q.GroundTrackDeg[float]


@dataclass(frozen=True, slots=True)
class LatLonDegrees:
    """Resolved latitude and longitude in degrees."""

    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]
