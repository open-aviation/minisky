"""Semantic runtime values used at parser and vectorized-state boundaries.

MiniSky keeps ordinary simulation arrays as typed numerical quantities from
[`minisky.quantities`][minisky.quantities]. These wrappers are for cases where
multiple values share the same Python carrier but their reference or variant
must survive at runtime.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass
from enum import IntEnum
from typing import TypeAlias

from minisky import quantities as q

AircraftTypeCode: TypeAlias = str
"""Aircraft model/type designator used by MiniSky and plugins."""

IcaoAircraftTypeCode: TypeAlias = AircraftTypeCode
"""Uppercase ICAO aircraft type designator used by OpenAP, for example `A320`."""

WaypointReference: TypeAlias = str
"""Stored waypoint source, e.g. `EHAM`, `EHAM/RW18L`, or `52.0,4.0`."""

AirportIdentifier: TypeAlias = str
"""Navigation-database airport identifier, commonly a four-letter ICAO code such as `EHAM`."""

RunwayIdentifier: TypeAlias = str
"""Runway designator without an `RW` prefix, for example `09` or `25L`."""

AirwayIdentifier: TypeAlias = str
"""Published airway identifier, for example `UL620`."""


class RuntimeNewType(ABC):
    """Base class for runtime newtypes.

    Subclassing this will ask the command parser to treat this as a transparent
    wrapper. It will allow you to stack more constraints (such as
    `annotated_types.Gt`).

    See: https://doc.rust-lang.org/rust-by-example/generics/new_types.html
    """

    # when registering newtypes, remember to subclass this so
    # the command parser knows to unwrap the inner value for annotated_types
    # checking!

    __slots__ = ()
    value: float


# NOTE(abraham): maybe switch to generics so callers can customise the inner types
# for example CasMps[Annotated[float, Gt(0), Le(...)]] instead of Annotatd[CasMps, Gt(0), Le(...)].

#
# airspeed
#


class AirspeedKind(IntEnum):
    """Discriminant for required selected airspeed state."""

    CAS = 0
    """[Calibrated airspeed in m/s][minisky.types.CasMps]."""
    MACH = 1
    """[Mach][minisky.types.Mach]."""


class OptionalAirspeedKind(IntEnum):
    """Discriminant for optional airspeed state."""

    NONE = 0
    CAS = 1
    """[Calibrated airspeed in m/s][minisky.types.CasMps]."""
    MACH = 2
    """[Mach][minisky.types.Mach]."""


@dataclass(frozen=True, slots=True)
class CasMps(RuntimeNewType):
    """Calibrated airspeed normalized to metres per second.

    Note that in minisky, IAS is used interchangeably with CAS, assuming zero
    instrument and position errors.
    """

    value: q.CalibratedAirspeedMps[float]


@dataclass(frozen=True, slots=True)
class Mach(RuntimeNewType):
    """Mach number preserved as a dimensionless value."""

    value: q.MachNumber[float]


#
# altitude / height
#


@dataclass(frozen=True, slots=True)
class StdPressureAltM(RuntimeNewType):
    """Barometric pressure altitude on the standard-pressure reference.

    QNE is the standard altimeter setting and flight levels are the operational
    designation for the standard-pressure reference (1013.25 hPa).

    In ICAO Field 15:

    - `F + 3 digits` = flight level, digits in hundreds of feet, and
    - `S + 4 digits` = standard metric level, digits in tens of metres
    """

    value: q.PressureAltitudeM[float]


@dataclass(frozen=True, slots=True)
class MslAltM(RuntimeNewType):
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
class _QfeHeightM(RuntimeNewType):  # pyright: ignore[reportUnusedClass]
    """Barometric height above the local QFE reference datum."""

    value: q.BarometricHeightM[float]


@dataclass(frozen=True, slots=True)
class _AglHeightM(RuntimeNewType):  # pyright: ignore[reportUnusedClass]
    """Height above the ground surface directly beneath the aircraft."""

    value: q.AglHeightM[float]


#
# other
#


@dataclass(frozen=True, slots=True)
class TrueHeadingDeg(RuntimeNewType):
    """Heading relative to true north in degrees."""

    value: q.TrueHeadingDegrees[float]


@dataclass(frozen=True, slots=True)
class MagneticHeadingDeg(RuntimeNewType):
    """Heading relative to magnetic north in degrees."""

    value: q.MagneticHeadingDegrees[float]


@dataclass(frozen=True, slots=True)
class GroundTrackDeg(RuntimeNewType):
    """Ground-track direction relative to true north in degrees."""

    value: q.GroundTrackDeg[float]


@dataclass(frozen=True, slots=True)
class LatLonDegrees:
    """Resolved latitude and longitude in degrees."""

    lat: q.LatitudeDeg[float]
    lon: q.LongitudeDeg[float]
