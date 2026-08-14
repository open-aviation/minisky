"""Semantic runtime values used at parser and vectorized-state boundaries.

MiniSky keeps ordinary simulation arrays as typed numerical quantities from
[`minisky.quantities`][minisky.quantities]. These wrappers are for cases where
multiple values share the same Python carrier but their reference must survive
at runtime.
"""

from __future__ import annotations

from dataclasses import dataclass

from minisky import quantities as q


@dataclass(frozen=True, slots=True)
class StdPressureAltM:
    """Barometric pressure altitude on the standard-pressure reference."""

    value: q.PressureAltitudeM[float]


@dataclass(frozen=True, slots=True)
class MslAltM:
    """Altitude above mean sea level."""

    value: q.MslAltitudeM[float]


# TODO(abraham): local/ground datum support needs an explicit elevation model.
@dataclass(frozen=True, slots=True)
class _QfeHeightM:  # pyright: ignore[reportUnusedClass]
    """Barometric height above the local QFE reference datum."""

    value: q.BarometricHeightM[float]


@dataclass(frozen=True, slots=True)
class _AglHeightM:  # pyright: ignore[reportUnusedClass]
    """Height above the ground surface directly beneath the aircraft."""

    value: q.AglHeightM[float]


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
