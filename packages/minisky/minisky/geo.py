"""Geodesy utilities for MiniSky.

Provides bearing and great-circle
distance calculations on the WGS'84 ellipsoid (qdrdist, latlondist),
fast flat-earth approximations for short distances (the kwik* functions),
position projection from a reference position with bearing and distance
(qdrpos, kwikpos), local earth radius and gravity according to WGS'84,
and magnetic declination lookup from a WMM data table (magdec).

Matrix variants (suffixed with `_matrix`) operate on vectors of positions
and return results for every combination of the input positions.
"""

from pathlib import Path
from typing import Protocol, Self

import numpy as np

from minisky import quantities as q

_WGS84_SEMI_MAJOR_AXIS: q.LengthM[float] = 6378137.0
_WGS84_SEMI_MINOR_AXIS: q.LengthM[float] = 6356752.314245
_MEAN_EARTH_RADIUS: q.LengthM[float] = 6371000.0
_METERS_PER_LATITUDE_DEGREE: q.DistanceM[float] = q.nmi_to_m(60.0)


def rwgs84(latd: q.LatitudeDeg) -> q.LengthM:
    """Calculate the Earth radius from the WGS'84 ellipsoid."""
    lat = np.radians(latd)
    a = _WGS84_SEMI_MAJOR_AXIS
    b = _WGS84_SEMI_MINOR_AXIS
    coslat = np.cos(lat)
    sinlat = np.sin(lat)

    an = a * a * coslat
    bn = b * b * sinlat
    ad = a * coslat
    bd = b * sinlat

    r = np.sqrt((an * an + bn * bn) / (ad * ad + bd * bd))

    return r


def rwgs84_matrix(latd: q.LatitudeDeg) -> q.LengthM[np.ndarray]:
    """Calculate the Earth radius from the WGS'84 ellipsoid (vectorized)."""

    lat = np.radians(latd)
    a = _WGS84_SEMI_MAJOR_AXIS
    b = _WGS84_SEMI_MINOR_AXIS
    coslat = np.cos(lat)
    sinlat = np.sin(lat)
    an = a * a * coslat
    bn = b * b * sinlat
    ad = a * coslat
    bd = b * sinlat

    anan = np.multiply(an, an)
    bnbn = np.multiply(bn, bn)
    adad = np.multiply(ad, ad)
    bdbd = np.multiply(bd, bd)
    r = np.sqrt(np.divide(anan + bnbn, adad + bdbd))

    return r


def qdrdist(
    latd1: q.LatitudeDeg,
    lond1: q.LongitudeDeg,
    latd2: q.LatitudeDeg,
    lond2: q.LongitudeDeg,
) -> tuple[q.BearingDeg, q.DistanceM]:
    """Calculate initial bearing and great-circle distance, using WGS'84.

    The distance uses the WGS'84 earth radius at the average latitude of
    the two positions, with a correction when the positions lie on
    different hemispheres. Bearing formula from
    http://www.movable-type.co.uk/scripts/latlong.html

    """

    # Check for hemisphere crossing,
    # when simple average would not work

    res1 = rwgs84(0.5 * (latd1 + latd2))  # same hemisphere

    a = _WGS84_SEMI_MAJOR_AXIS
    r1 = rwgs84(latd1)
    r2 = rwgs84(latd2)
    res2 = (
        0.5
        * (abs(latd1) * (r1 + a) + abs(latd2) * (r2 + a))
        / (np.maximum(0.000001, abs(latd1) + abs(latd2)))
    )  # different hemisphere

    sw = latd1 * latd2 >= 0.0

    r = sw * res1 + (1 - sw) * res2

    lat1 = np.radians(latd1)
    lon1 = np.radians(lond1)
    lat2 = np.radians(latd2)
    lon2 = np.radians(lond2)

    # Corrected to avoid "nan" at westward direction
    d = r * np.arccos(
        np.cos(lat1) * np.cos(lat2) * np.cos(lon2 - lon1) + np.sin(lat1) * np.sin(lat2)
    )
    coslat1 = np.cos(lat1)
    coslat2 = np.cos(lat2)

    qdr = np.degrees(
        np.arctan2(
            np.sin(lon2 - lon1) * coslat2,
            coslat1 * np.sin(lat2) - np.sin(lat1) * coslat2 * np.cos(lon2 - lon1),
        )
    )

    return qdr, d


def qdrdist_matrix(
    lat1: q.LatitudeDeg,
    lon1: q.LongitudeDeg,
    lat2: q.LatitudeDeg,
    lon2: q.LongitudeDeg,
) -> tuple[q.BearingDeg[np.ndarray], q.DistanceM[np.ndarray]]:
    """Calculate bearing and distance matrices between position vectors, using WGS'84.

    Computes bearing and haversine distance for every combination of a
    position in vectors 1 and a position in vectors 2.

    """
    # Convert inputs to 2-D row arrays, so that .T gives column arrays and
    # broadcasting yields a result for every combination of positions.
    lat1 = np.atleast_2d(np.asarray(lat1))
    lon1 = np.atleast_2d(np.asarray(lon1))
    lat2 = np.atleast_2d(np.asarray(lat2))
    lon2 = np.atleast_2d(np.asarray(lon2))

    prodla = lat1.T * lat2
    condition = prodla < 0

    r = np.zeros(prodla.shape)
    r = np.where(condition, r, rwgs84_matrix(0.5 * (lat1.T + lat2)))

    a = _WGS84_SEMI_MAJOR_AXIS

    r = np.where(
        np.invert(condition),
        r,
        (
            np.divide(
                np.multiply(
                    0.5,
                    (
                        (np.multiply(abs(lat1), (rwgs84_matrix(lat1) + a))).T
                        + np.multiply(abs(lat2), (rwgs84_matrix(lat2) + a))
                    ),
                ),
                (abs(lat1)).T + (abs(lat2) + (lat1 == 0.0) * 0.000001),
            )
        ),
    )  # different hemisphere

    diff_lat = lat2 - lat1.T
    diff_lon = lon2 - lon1.T

    sin1 = np.radians(diff_lat)
    sin2 = np.radians(diff_lon)

    sinlat1 = np.sin(np.radians(lat1))
    sinlat2 = np.sin(np.radians(lat2))
    coslat1 = np.cos(np.radians(lat1))
    coslat2 = np.cos(np.radians(lat2))

    sin21 = np.sin(sin2)
    cos21 = np.cos(sin2)
    y = np.multiply(sin21, coslat2)

    x1 = np.multiply(coslat1.T, sinlat2)

    x2 = np.multiply(sinlat1.T, coslat2)
    x3 = np.multiply(x2, cos21)
    x = x1 - x3

    qdr = np.degrees(np.arctan2(y, x))

    sin10 = np.abs(np.sin(sin1 / 2.0))
    sin20 = np.abs(np.sin(sin2 / 2.0))
    sin1sin1 = np.multiply(sin10, sin10)
    sin2sin2 = np.multiply(sin20, sin20)
    sqrt = sin1sin1 + np.multiply((coslat1.T * coslat2), sin2sin2)
    dist_c = np.multiply(2.0, np.arctan2(np.sqrt(sqrt), np.sqrt(1 - sqrt)))
    dist = np.multiply(r, dist_c)

    return qdr, dist


def latlondist(
    latd1: q.LatitudeDeg,
    lond1: q.LongitudeDeg,
    latd2: q.LatitudeDeg,
    lond2: q.LongitudeDeg,
) -> q.DistanceM:
    """Calculates only distance using haversine notation of the same formulae
    and average r from wgs'84.
    """
    res1 = rwgs84(0.5 * (latd1 + latd2))  # same hemisphere

    # res2 :different hemisphere
    a = _WGS84_SEMI_MAJOR_AXIS
    r1 = rwgs84(latd1)
    r2 = rwgs84(latd2)
    res2 = (
        0.5 * (abs(latd1) * (r1 + a) + abs(latd2) * (r2 + a)) / (abs(latd1) + abs(latd2))
    )  # different hemisphere

    sw = latd1 * latd2 >= 0.0

    r = sw * res1 + (1 - sw) * res2

    lat1 = np.radians(latd1)
    lon1 = np.radians(lond1)
    lat2 = np.radians(latd2)
    lon2 = np.radians(lond2)

    sin1 = np.sin(0.5 * (lat2 - lat1))
    sin2 = np.sin(0.5 * (lon2 - lon1))

    coslat1 = np.cos(lat1)
    coslat2 = np.cos(lat2)

    root = sin1 * sin1 + coslat1 * coslat2 * sin2 * sin2
    d = 2.0 * r * np.arctan2(np.sqrt(root), np.sqrt(1.0 - root))

    return d


def latlondist_matrix(
    lat1: q.LatitudeDeg,
    lon1: q.LongitudeDeg,
    lat2: q.LatitudeDeg,
    lon2: q.LongitudeDeg,
) -> q.DistanceM[np.ndarray]:
    """Calculates distance matrix using haversine formulae and average r from wgs'84."""
    # Convert inputs to 2-D row arrays, so that .T gives column arrays and
    # broadcasting yields a result for every combination of positions.
    lat1 = np.atleast_2d(np.asarray(lat1))
    lon1 = np.atleast_2d(np.asarray(lon1))
    lat2 = np.atleast_2d(np.asarray(lat2))
    lon2 = np.atleast_2d(np.asarray(lon2))

    prodla = lat1.T * lat2
    condition = prodla < 0

    r = np.zeros(prodla.shape)
    r = np.where(condition, r, rwgs84_matrix(0.5 * (lat1.T + lat2)))

    a = _WGS84_SEMI_MAJOR_AXIS
    r = np.where(
        np.invert(condition),
        r,
        (
            np.divide(
                np.multiply(
                    0.5,
                    (
                        (np.multiply(abs(lat1), (rwgs84_matrix(lat1) + a))).T
                        + np.multiply(abs(lat2), (rwgs84_matrix(lat2) + a))
                    ),
                ),
                (abs(lat1)).T + (abs(lat2)),
            )
        ),
    )  # different hemisphere

    diff_lat = lat2 - lat1.T
    diff_lon = lon2 - lon1.T

    sin1 = np.radians(diff_lat)
    sin2 = np.radians(diff_lon)

    coslat1 = np.cos(np.radians(lat1))
    coslat2 = np.cos(np.radians(lat2))

    sin10 = np.sin(sin1 / 2)
    sin20 = np.sin(sin2 / 2)
    sin1sin1 = np.multiply(sin10, sin10)
    sin2sin2 = np.multiply(sin20, sin20)
    root = sin1sin1 + np.multiply((coslat1.T * coslat2), sin2sin2)

    dist_c = np.multiply(2, np.arctan2(np.sqrt(root), np.sqrt(1.0 - root)))
    dist = np.multiply(r, dist_c)

    return dist


def wgsg(latd: q.LatitudeDeg) -> q.GravitationalAccelerationMps2:
    """Gravity acceleration at a given latitude according to WGS'84."""
    geq = 9.7803  # m/s2 g at equator
    e2 = 6.694e-3  # eccentricity
    k = 0.001932  # derived from flattening f, 1/f = 298.257223563

    sinlat = np.sin(np.radians(latd))
    g = geq * (1.0 + k * sinlat * sinlat) / np.sqrt(1.0 - e2 * sinlat * sinlat)

    return g


def qdrpos(
    latd1: q.LatitudeDeg,
    lond1: q.LongitudeDeg,
    qdr: q.BearingDeg,
    dist: q.DistanceM,
) -> tuple[q.LatitudeDeg, q.LongitudeDeg]:
    """Calculate vector with positions from vectors of reference position,
    bearing and distance.

    Great-circle projection using the WGS'84 earth radius at the reference
    latitude. Ref for qdrpos:
    http://www.movable-type.co.uk/scripts/latlong.html

    """

    R = rwgs84(latd1)
    lat1 = np.radians(latd1)
    lon1 = np.radians(lond1)

    lat2 = np.arcsin(
        np.sin(lat1) * np.cos(dist / R) + np.cos(lat1) * np.sin(dist / R) * np.cos(np.radians(qdr))
    )

    lon2 = lon1 + np.arctan2(
        np.sin(np.radians(qdr)) * np.sin(dist / R) * np.cos(lat1),
        np.cos(dist / R) - np.sin(lat1) * np.sin(lat2),
    )
    return np.degrees(lat2), np.degrees(lon2)


def kwikdist(
    lata: q.LatitudeDeg,
    lona: q.LongitudeDeg,
    latb: q.LatitudeDeg,
    lonb: q.LongitudeDeg,
) -> q.DistanceM:
    """Quick and dirty distance calculation.

    Equirectangular (flat-earth) approximation with the mean earth radius;
    fast, but accurate for short distances only.

    """

    re = _MEAN_EARTH_RADIUS
    dlat = np.radians(latb - lata)
    dlon = np.radians(((lonb - lona) + 180) % 360 - 180)
    cavelat = np.cos(np.radians(lata + latb) * 0.5)

    dangle = np.sqrt(dlat * dlat + dlon * dlon * cavelat * cavelat)
    dist = re * dangle

    return dist


def kwikdist_matrix(
    lata: q.LatitudeDeg[np.ndarray],
    lona: q.LongitudeDeg[np.ndarray],
    latb: q.LatitudeDeg[np.ndarray],
    lonb: q.LongitudeDeg[np.ndarray],
) -> q.DistanceM[np.ndarray]:
    """Quick and dirty distance matrix between two sets of positions.

    Equirectangular (flat-earth) approximation with the mean earth radius;
    fast, but accurate for short distances only.

    """

    re = _MEAN_EARTH_RADIUS
    dlat = np.radians(latb - lata.T)
    dlon = np.radians(((lonb - lona.T) + 180) % 360 - 180)
    cavelat = np.cos(np.radians(lata + latb.T) * 0.5)

    dangle = np.sqrt(
        np.multiply(dlat, dlat)
        + np.multiply(np.multiply(dlon, dlon), np.multiply(cavelat, cavelat))
    )
    dist = re * dangle

    return dist


def kwikqdrdist(
    lata: q.LatitudeDeg,
    lona: q.LongitudeDeg,
    latb: q.LatitudeDeg,
    lonb: q.LongitudeDeg,
) -> tuple[q.BearingDeg, q.DistanceM]:
    """Quick bearing/distance using a flat-earth approximation.

    Uses the mean earth radius and does not work well close to the poles.


    Bearings are normalized to [0, 360).
    """

    re = _MEAN_EARTH_RADIUS
    dlat = np.radians(latb - lata)
    dlon = np.radians(((lonb - lona) + 180) % 360 - 180)
    cavelat = np.cos(np.radians(lata + latb) * 0.5)

    dangle = np.sqrt(dlat * dlat + dlon * dlon * cavelat * cavelat)
    dist = re * dangle

    qdr = np.degrees(np.arctan2(dlon * cavelat, dlat)) % 360.0

    return qdr, dist


def kwikqdrdist_matrix(
    lata: q.LatitudeDeg[np.ndarray],
    lona: q.LongitudeDeg[np.ndarray],
    latb: q.LatitudeDeg[np.ndarray],
    lonb: q.LongitudeDeg[np.ndarray],
) -> tuple[q.BearingDeg[np.ndarray], q.DistanceM[np.ndarray]]:
    """Quick bearing/distance matrices using a flat-earth approximation.

    Uses the mean earth radius and does not work well close to the poles.


    Bearings are normalized to [0, 360).
    """

    re = _MEAN_EARTH_RADIUS
    dlat = np.radians(latb - lata.T)
    dlon = np.radians(((lonb - lona.T) + 180) % 360 - 180)
    cavelat = np.cos(np.radians(latb + lata.T) * 0.5)

    dangle = np.sqrt(
        np.multiply(dlat, dlat)
        + np.multiply(np.multiply(dlon, dlon), np.multiply(cavelat, cavelat))
    )
    dist = re * dangle

    qdr = np.degrees(np.arctan2(np.multiply(dlon, cavelat), dlat)) % 360.0

    return qdr, dist


def kwikpos(
    latd1: q.LatitudeDeg,
    lond1: q.LongitudeDeg,
    qdr: q.BearingDeg,
    dist: q.DistanceM,
) -> tuple[q.LatitudeDeg, q.LongitudeDeg]:
    """Fast, but quick and dirty, position calculation from vectors of reference position,
    bearing and distance using flat earth approximation.

    Use for flat earth purposes e.g. flat display.


    Longitude is wrapped to [-180, 180).
    """

    dx = dist * np.sin(np.radians(qdr))
    dy = dist * np.cos(np.radians(qdr))
    dlat = dy / _METERS_PER_LATITUDE_DEGREE
    dlon = dx / np.maximum(0.01, _METERS_PER_LATITUDE_DEGREE * np.cos(np.radians(latd1)))
    latd2 = latd1 + dlat
    lond2 = ((lond1 + dlon) + 180) % 360 - 180

    return latd2, lond2


class MagneticDeclination(Protocol):
    def __call__(
        self, latd: q.LatitudeDeg[float], lond: q.LongitudeDeg[float], /
    ) -> q.MagneticDeclinationDeg[float]: ...


class MagneticDeclinationGrid(MagneticDeclination):
    def __init__(self, declinations: q.MagneticDeclinationDeg[np.ndarray]) -> None:
        values = np.asarray(declinations, dtype=float)
        self.declinations = values.copy()
        self.declinations.setflags(write=False)

    @classmethod
    def load_default(cls) -> Self:
        """Load the magnetic-declination grid bundled with MiniSky core.

        Based on the data table calculated from:
        <https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#igrfgrid>
        with the following input:
            Southern most lat:  90 S
            Northern most lat:  90 N
            Lat Step Size:      1.0
            Western most long:  180 W
            Eastern most long:  179 E
            Lon Step Size:      1.0
            Elevation:          Mean sea level 0 Feet
            Magnetic component: Declination
            Model:              WMM (2019-2024)
            Start Date:         2020 09 20
            End Date:           2020 09 20
            Step size:          1.0
        The grid size can be adjusted but the (1 deg by 1 deg) size should suffice
        for practical purpose, as long as the the grids cover the entire Earth
        surface. The interpolation is performed at sea-level, but no significant
        difference would be noticed up to FL600 or beyond.
        Based on original version created by  : Yaofu Zhou
        Modified to read at init and use linear interpolation by J.M. Hoekstra
        """
        from minisky._internal.config import data

        return cls.from_csv(data("geo") / "geo_declination_data.csv")

    @classmethod
    def from_csv(cls, path: Path) -> Self:
        """Read a NOAA/WMM CSV (a 181x361 lookup grid).

        The source table is expected to contain one row per whole-degree point
        for latitude +89 through -90 and longitude -180 through +179, with
        declination in the fifth column. The +90 latitude row and +180 longitude
        column are derived from the adjacent/wrapped source values.
        """
        # NOTE(abraham): shape hardcoding is inherited from bluesky, reconsider
        source = np.genfromtxt(
            path, delimiter=",", comments="#", usecols=(4,), dtype=float
        ).reshape((180, 360))
        source = np.vstack((source[0:1, :], source))
        source = np.hstack((source, source[:, 0:1]))
        return cls(source)

    def __call__(
        self, latd: q.LatitudeDeg[float], lond: q.LongitudeDeg[float], /
    ) -> q.MagneticDeclinationDeg[float]:
        return _interpolate_magnetic_declination(latd, lond, self.declinations)


def _interpolate_magnetic_declination(
    latd: q.LatitudeDeg[float],
    lond: q.LongitudeDeg[float],
    decl_lat_lon: q.MagneticDeclinationDeg[np.ndarray],
) -> q.MagneticDeclinationDeg[float]:
    """Return magnetic declination at a position from a lookup grid.

    Created by  : Yaofu Zhou
    Modified by J.M. Hoekstra
    Reason: Segmentation fault caused by Scipy's BiVariateSpline interpolation
    for some data on some machines, so it was changed to linear interpolation.
    Difference in methods has been inspected: it is way less than the inaccuracy
    of the actual data.
    The direct manual linear interpolation is also about 6 times faster.
    """
    # Use fact that whole degrees are used as ticks on both lat & lon axis
    i_lat = min(max(0, int(90.0 - latd)), 180)
    f_lat = (90.0 - latd) - int(90.0 - latd)
    i_lon = min(max(0, int(lond + 180)), 360)
    f_lon = lond + 180.0 - int(lond + 180)

    # 2D linear interpolation
    declon0 = (
        decl_lat_lon[i_lat, i_lon] * (1.0 - f_lat)
        + f_lat * decl_lat_lon[min(180, i_lat + 1), i_lon]
    )
    declon1 = (
        decl_lat_lon[i_lat, i_lon + 1] * (1.0 - f_lat)
        + f_lat * decl_lat_lon[min(180, i_lat + 1), min(i_lon + 1, 360)]
    )

    d_hdg = declon0 * (1.0 - f_lon) + f_lon * declon1

    return d_hdg
