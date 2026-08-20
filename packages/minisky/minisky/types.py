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
from typing import Annotated, Generic, TypeAlias

from annotated_types import Ge, Gt
from typing_extensions import TypeVar

from minisky import quantities as q

AircraftTypeCode: TypeAlias = str
"""Aircraft model/type designator used by MiniSky and plugins."""

AircraftCallsign: TypeAlias = str
"""Aircraft identifier used as a callsign within a simulation runtime."""

AircraftIndex: TypeAlias = int
"""Index into the traffic arrays for an aircraft."""

RouteWaypointIndex: TypeAlias = int
"""Index of a waypoint within an aircraft route."""

IcaoAircraftTypeCode: TypeAlias = AircraftTypeCode
"""Uppercase ICAO aircraft type designator used by OpenAP, for example `A320`."""

WaypointReference: TypeAlias = str
"""Stored waypoint source, e.g. `EHAM`, `EHAM/RW18L`, or `52.0,4.0`."""

WaypointIdentifier: TypeAlias = str
"""Navigation-dataset waypoint identifier, for example `SUGOL`."""

AirportIdentifier: TypeAlias = str
"""Navigation-database airport identifier, commonly a four-letter ICAO code such as `EHAM`."""

RunwayIdentifier: TypeAlias = str
"""Runway designator without an `RW` prefix, for example `09` or `25L`."""

AirwayIdentifier: TypeAlias = str
"""Published airway identifier, for example `UL620`."""


_T = TypeVar("_T")

Ge0: TypeAlias = Annotated[_T, Ge(0)]
"""A value greater than or equal to zero."""

Gt0: TypeAlias = Annotated[_T, Gt(0)]
"""A value greater than zero."""


class RuntimeNewType(ABC, Generic[_T]):
    """Base class for runtime newtypes.

    The generic carrier preserves constraints and container types independently
    of the semantic wrapper, for example `CasMps[IsFinite[Gt0[float]]]` or
    `CasMps[np.ndarray]`.

    See: https://doc.rust-lang.org/rust-by-example/generics/new_types.html
    """

    __slots__ = ()
    value: _T


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
class CasMps(RuntimeNewType[_T]):
    """Calibrated airspeed normalized to metres per second.

    Note that in minisky, IAS is used interchangeably with CAS, assuming zero
    instrument and position errors.
    """

    value: q.CalibratedAirspeedMps[_T]


@dataclass(frozen=True, slots=True)
class Mach(RuntimeNewType[_T]):
    """Mach number preserved as a dimensionless value."""

    value: q.MachNumber[_T]


#
# altitude / height
#


@dataclass(frozen=True, slots=True)
class StdPressureAltM(RuntimeNewType[_T]):
    """Barometric pressure altitude on the standard-pressure reference.

    QNE is the standard altimeter setting and flight levels are the operational
    designation for the standard-pressure reference (1013.25 hPa).

    In ICAO Field 15:

    - `F + 3 digits` = flight level, digits in hundreds of feet, and
    - `S + 4 digits` = standard metric level, digits in tens of metres
    """

    value: q.PressureAltitudeM[_T]


@dataclass(frozen=True, slots=True)
class MslAltM(RuntimeNewType[_T]):
    """Altitude above mean sea level.

    QNH is the altimeter subscale pressure setting chosen so a correctly
    calibrated pressure altimeter indicates altitude relative to mean sea
    level.

    In ICAO Field 15:

    - `A + 3 digits` = altitude in hundreds of feet
    - `M + 4 digits` = altitude in tens of metres
    """

    value: q.MslAltitudeM[_T]


# TODO(abraham): we don't know how to handle local/ground datum (see #22)
# keeping them internal for now
@dataclass(frozen=True, slots=True)
class _QfeHeightM(RuntimeNewType[_T]):  # pyright: ignore[reportUnusedClass]
    """Barometric height above the local QFE reference datum."""

    value: q.BarometricHeightM[_T]


@dataclass(frozen=True, slots=True)
class _AglHeightM(RuntimeNewType[_T]):  # pyright: ignore[reportUnusedClass]
    """Height above the ground surface directly beneath the aircraft."""

    value: q.AglHeightM[_T]


#
# other
#


@dataclass(frozen=True, slots=True)
class TrueHeadingDeg(RuntimeNewType[_T]):
    """Heading relative to true north in degrees."""

    value: q.TrueHeadingDegrees[_T]


@dataclass(frozen=True, slots=True)
class MagneticHeadingDeg(RuntimeNewType[_T]):
    """Heading relative to magnetic north in degrees."""

    value: q.MagneticHeadingDegrees[_T]


@dataclass(frozen=True, slots=True)
class GroundTrackDeg(RuntimeNewType[_T]):
    """Ground-track direction relative to true north in degrees."""

    value: q.GroundTrackDeg[_T]


@dataclass(frozen=True, slots=True)
class LatLonDegrees(Generic[_T]):
    """Resolved latitude and longitude in degrees."""

    lat: q.LatitudeDeg[_T]
    lon: q.LongitudeDeg[_T]
