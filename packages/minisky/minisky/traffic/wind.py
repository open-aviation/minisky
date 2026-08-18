"""Simulate wind in BlueSky.

Implements a wind field defined by wind vectors at arbitrary lat/lon
positions, optionally with altitude profiles. The field is interpolated
(inverse-distance weighting horizontally, linear in altitude) to obtain
the wind at any aircraft position. [`Windfield`][] contains the field
data and interpolation; [`Wind`][] adds the stack-command interface
(WIND to define wind, GETWIND to query it) and is available at runtime as
[`runtime.traffic.wind`][minisky.traffic.wind.Wind]. The traffic model uses the wind to compute ground
speed and track from heading and airspeed.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from itertools import pairwise
from math import isfinite
from typing import Annotated, Literal

import numpy as np
from annotated_types import Ge
from scipy.interpolate import LinearNDInterpolator, interp1d

from minisky import quantities as q
from minisky.command import (
    ArgumentIssue,
    CmdParser,
    CommandCursor,
    CommandParseContext,
    FiniteFloat,
    LatLonDeg,
    Omitted,
    ParseResult,
    SourceSpan,
    Spanned,
    SpeedMps,
    command,
    parse_field,
    parse_pressure_altitude_value,
    parse_speed_value,
)
from minisky.core.trafficarrays import TrafficArrays
from minisky.result import Err, Ok, Result
from minisky.types import StdPressureAltM

WindDirectionArg = q.WindDirectionDeg[FiniteFloat]
NonNegativeSpeedMps = Annotated[SpeedMps, Ge(0)]


class WindFieldKind(Enum):
    NONE = auto()
    CONSTANT = auto()
    HORIZONTAL = auto()
    ALTITUDE_DEPENDENT = auto()


class Windfield:
    """Windfield class."""

    def __init__(self) -> None:
        # For altitude use fixed axis to allow vectorisation later
        self.altmax: q.PressureAltitudeM[float] = q.ft_to_m(45000.0)  # pyright: ignore[reportGeneralTypeIssues]
        self.altstep: q.VerticalDistanceM[float] = q.ft_to_m(100.0)  # pyright: ignore[reportGeneralTypeIssues]
        self.altaxis: q.PressureAltitudeM[np.ndarray] = np.arange(  # pyright: ignore[reportGeneralTypeIssues]
            0.0, self.altmax + self.altstep, self.altstep
        )
        self.idxalt = np.arange(0, len(self.altaxis), 1.0)
        self.nalt = len(self.altaxis)

        self.clear()

    @property
    def kind(self) -> WindFieldKind:
        """Classify the field from its actual point/profile data."""
        if np.any(self.profiled):
            return WindFieldKind.ALTITUDE_DEPENDENT
        if len(self.lat) == 0:
            return WindFieldKind.NONE
        if len(self.lat) == 1:
            return WindFieldKind.CONSTANT
        return WindFieldKind.HORIZONTAL

    @property
    def has_wind(self) -> bool:
        return self.kind is not WindFieldKind.NONE

    @property
    def nvec(self) -> int:
        return len(self.lat)

    def clear(self) -> None:
        """Remove all wind vectors."""
        self.lat: q.LatitudeDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
        self.lon: q.LongitudeDeg[np.ndarray] = np.array([])  # pyright: ignore[reportGeneralTypeIssues]
        self.profiled: np.ndarray = np.array([], dtype=bool)
        """Whether each wind definition includes an altitude profile."""
        self.vnorth: q.WindSpeedMps[np.ndarray] = np.array([[]])  # pyright: ignore[reportGeneralTypeIssues]
        self.veast: q.WindSpeedMps[np.ndarray] = np.array([[]])  # pyright: ignore[reportGeneralTypeIssues]
        self.fe: LinearNDInterpolator | None = None
        self.fn: LinearNDInterpolator | None = None

    def addpointvne(
        self,
        lat: q.LatitudeDeg[np.ndarray],
        lon: q.LongitudeDeg[np.ndarray],
        vnorth: q.WindSpeedMps[np.ndarray],
        veast: q.WindSpeedMps[np.ndarray],
        windalt: q.PressureAltitudeM[np.ndarray] | None = None,
    ) -> None:
        """Add wind vectors given as north/east speed components.

        Vectorized alternative to addpoint() for defining many wind points
        at once. When altitudes are given, a scipy interpolator over
        (altitude, lat, lon) is set up for regular grids; otherwise the
        profiles are resampled onto the fixed altitude axis.

        Args:
            vnorth: North component; altitude-by-position when `windalt` is given.
            veast: East component with the same shape as `vnorth`.
            windalt: Optional profile altitudes corresponding to component-array rows.
        """
        has_profile = windalt is not None and len(windalt) > 1
        if has_profile:
            # Set altitude interpolation functions
            fnorth = interp1d(
                windalt,
                vnorth.T,
                bounds_error=False,
                fill_value=(vnorth[0], vnorth[-1]),  # type: ignore[arg-type]
                assume_sorted=True,
            )
            feast = interp1d(
                windalt,
                veast.T,
                bounds_error=False,
                fill_value=(veast[0], veast[-1]),  # type: ignore[arg-type]
                assume_sorted=True,
            )

            # Assume regular grid and set RGI for interpolation
            if len(lat) > 3:
                try:
                    # Interpolate along windalt axis
                    altaxis = np.concatenate((np.array([0.0]), windalt))
                    vnaxis = fnorth(altaxis).T
                    veaxis = feast(altaxis).T

                    # Get unique latitudes and longitudes for RGI
                    lats = np.unique(lat)
                    lons = np.unique(lon)

                    # Set RGI interpolation functions
                    vevalues = veaxis.reshape((len(altaxis), len(lats), len(lons)))
                    vnvalues = vnaxis.reshape((len(altaxis), len(lats), len(lons)))
                    self.fe = LinearNDInterpolator(
                        (altaxis, lats, lons),
                        vevalues,
                        bounds_error=False,
                        fill_value=0.0,
                    )
                    self.fn = LinearNDInterpolator(
                        (altaxis, lats, lons),
                        vnvalues,
                        bounds_error=False,
                        fill_value=0.0,
                    )
                except Exception:  # ruff: ignore[BLE001] scipy interpolation may fail broadly
                    # Create vn, ve if RGI is not possible
                    vnaxis = fnorth(self.altaxis).T
                    veaxis = feast(self.altaxis).T
            else:
                # Create vn, ve if less than 4 coords are present
                vnaxis = fnorth(self.altaxis).T
                veaxis = feast(self.altaxis).T

        else:
            vnaxis = vnorth
            veaxis = veast

        self.lat = np.append(self.lat, lat)
        self.lon = np.append(self.lon, lon)
        self.profiled = np.append(self.profiled, np.full(len(lat), has_profile, dtype=bool))

        if self.vnorth.size == 0:
            self.vnorth = vnaxis
            self.veast = veaxis
        else:
            self.vnorth = np.concatenate((self.vnorth, vnaxis), axis=1)
            self.veast = np.concatenate((self.veast, veaxis), axis=1)

    def addpoint(
        self,
        lat: q.LatitudeDeg[float],
        lon: q.LongitudeDeg[float],
        winddir: q.WindDirectionDeg,
        windspd: q.WindSpeedMps,
        windalt: q.PressureAltitudeM | None = None,
    ) -> int:
        """Add a wind vector (direction/speed) at a lat/lon position.

        The wind is converted to north/east components and stored on the
        fixed altitude axis. When an altitude array is given, the wind
        profile is interpolated onto that axis and the field becomes 3D
        (altitude dependent).

        Args:
            winddir: Direction the wind comes from; an array for an altitude profile.
            windspd: Wind speed with the same dimensionality as `winddir`.
            windalt: Optional altitudes defining the profile at this position.

        Returns:
            Index of the added wind point, suitable for `remove()`.
        """

        # If scalar, copy into table for altitude axis
        if not (isinstance(windalt, (np.ndarray, list))) and windalt is None:
            prof3D = False  # no wind profile, just one value
            wspd = np.ones(self.nalt) * windspd
            wdir = np.ones(self.nalt) * winddir
            vnaxis = wspd * np.cos(np.radians(wdir) + np.pi)
            veaxis = wspd * np.sin(np.radians(wdir) + np.pi)

        # if list or array, convert to alt axis of wind field
        else:
            prof3D = True  # switch on 3D parameter as an altitude array is given
            wspd = np.array(windspd)
            wdir = np.array(winddir)
            altvn = wspd * np.cos(np.radians(wdir) + np.pi)
            altve = wspd * np.sin(np.radians(wdir) + np.pi)
            alttab = windalt

            vnaxis = np.interp(self.altaxis, alttab, altvn)
            veaxis = np.interp(self.altaxis, alttab, altve)

        #        print array([vnaxis]).transpose()
        self.lat = np.append(self.lat, lat)
        self.lon = np.append(self.lon, lon)
        self.profiled = np.append(self.profiled, prof3D)

        idx = len(self.lat) - 1

        if self.vnorth.size == 0:
            self.vnorth = np.array([vnaxis]).transpose()
            self.veast = np.array([veaxis]).transpose()

        else:
            self.vnorth = np.append(self.vnorth, np.array([vnaxis]).transpose(), axis=1)
            self.veast = np.append(self.veast, np.array([veaxis]).transpose(), axis=1)

        return idx  # return index of added point

    def getdata(
        self,
        userlat: q.LatitudeDeg,
        userlon: q.LongitudeDeg,
        useralt: q.PressureAltitudeM = 0.0,
    ) -> tuple[
        q.WindSpeedMps, q.WindSpeedMps
    ]:  # in case no altitude specified and field is 3D, use sea level wind
        """Interpolate the wind field at one or more positions.

        Uses inverse-distance-squared weighting between the defined wind
        points horizontally, and linear interpolation along the altitude
        axis for 3D fields. Constant and empty fields are handled as
        special cases. When no altitude is given for a 3D field, sea-level
        wind is returned.

        Args:
            userlat: Scalar, list, or ndarray of query latitudes.
            userlon: Query longitudes with the same shape as `userlat`.
            useralt: Query altitudes; defaults to sea level.

        Returns:
            North/east wind components with the same container shape as the query positions.
        """
        eps = 1e-20  # [m2] to avoid divison by zero for using exact same points

        swvector = isinstance(userlat, (list, np.ndarray))
        npos = len(userlat) if swvector else 1
        # Convert user input to right shape: columns for positions
        lat = np.array(userlat).reshape((1, npos))
        lon = np.array(userlon).reshape((1, npos))

        # Make altitude into an array, with zero or float value broadcast over npos
        if isinstance(useralt, np.ndarray):
            alt = useralt
        elif isinstance(useralt, list):
            alt = np.array(useralt)
        elif isinstance(useralt, float):
            alt = useralt * np.ones(npos)
        else:
            alt = np.zeros(npos)

        vnorth = np.zeros(npos)
        veast = np.zeros(npos)

        # Check if RGI functions are present, if so use them for interpolation
        if self.fe is not None and self.fn is not None:
            vnorth = self.fn(np.concatenate((alt.reshape(1, -1), lat, lon), axis=0).T)
            veast = self.fe(np.concatenate((alt.reshape(1, -1), lat, lon), axis=0).T)
        else:
            if self.kind is WindFieldKind.NONE:
                vnorth = np.zeros(npos)
                veast = np.zeros(npos)

            elif self.kind is WindFieldKind.CONSTANT:
                vnorth = np.ones(npos) * self.vnorth[0, 0]
                veast = np.ones(npos) * self.veast[0, 0]

            else:
                # ---- Get horizontal weight factors

                # Average cosine for flat-eartyh approximation
                cavelat = np.cos(np.radians(0.5 * (lat + np.array([self.lat]).transpose())))

                # Lat and lon distance in 60 nm units (1 lat degree)
                dy = lat - np.array([self.lat]).transpose()  # (nvec,npos)
                dx = cavelat * (lon - np.array([self.lon]).transpose())

                # Calulate invesre distance squared
                invd2 = 1.0 / (eps + dx * dx + dy * dy)  # inverse of distance squared

                # Normalize weights
                sumsid2 = np.ones((1, self.nvec)).dot(invd2)  # totals to normalize weights
                totals = np.repeat(sumsid2, self.nvec, axis=0)  # scale up dims to (nvec,npos)

                horfact = invd2 / totals  # rows x col = nvec x npos, weight factors

                # ---- Altitude interpolation

                # No altitude profiles used: do 2D planar interpolation only
                if self.kind is WindFieldKind.HORIZONTAL or (
                    not isinstance(useralt, (list, np.ndarray)) and useralt == 0.0
                ):  # horizontal field or sea-level query
                    vnorth = self.vnorth[0, :].dot(horfact)
                    veast = self.veast[0, :].dot(horfact)

                # 3D interpolation as one or more points contain altitude profile
                else:
                    # Get altitude index as float for alt interpolation
                    idxalt = np.maximum(
                        0.0, np.minimum(self.altaxis[-1] - eps, alt) / self.altstep
                    )  # find right index

                    # Convert to index and factor
                    ialt = np.floor(idxalt).astype(int)  # index array for lower altitude
                    falt = idxalt - ialt  # factor for upper value

                    # Altitude interpolation combined with horizontal
                    nvec = len(self.lon)  # Get number of definition points

                    # North wind (y-direction ot lat direction)
                    vn0 = (self.vnorth[ialt, :] * horfact.T).dot(
                        np.ones((nvec, 1))
                    )  # hor interpolate lower alt (npos x)
                    vn1 = (self.vnorth[ialt + 1, :] * horfact.T).dot(
                        np.ones((nvec, 1))
                    )  # hor interpolate lower alts (npos x)
                    vnorth = (1.0 - falt) * (vn0.reshape(npos)) + falt * (
                        vn1.reshape(npos)
                    )  # As 1D array

                    # East wind (x-direction or lon direction)
                    ve0 = (self.veast[ialt, :] * horfact.T).dot(np.ones((nvec, 1)))
                    ve1 = (self.veast[ialt + 1, :] * horfact.T).dot(np.ones((nvec, 1)))
                    veast = (1.0 - falt) * (ve0.reshape(npos)) + falt * (
                        ve1.reshape(npos)
                    )  # As 1D array

        # Return same type as positons were given
        if isinstance(userlat, np.ndarray):
            return vnorth, veast

        elif isinstance(userlat, list):
            return list(vnorth), list(veast)

        else:
            return float(np.asarray(vnorth).item()), float(np.asarray(veast).item())

    def remove(self, idx: int) -> None:  # remove a point using the returned index when it was added
        """Remove a wind definition point by index.

        Args:
            idx: Point index returned by `addpoint()`.
        """
        if idx < len(self.lat):
            self.lat = np.delete(self.lat, idx)
            self.lon = np.delete(self.lon, idx)

            self.vnorth = np.delete(self.vnorth, idx, axis=1)
            self.veast = np.delete(self.veast, idx, axis=1)
            self.profiled = np.delete(self.profiled, idx)

            # TODO(abraham): make the scipy interpolation functions a derived cache;
            # add/remove can currently leave fe/fn describing an older point set.


@dataclass(frozen=True, slots=True)
class WindLevel:
    """A validated altitude-dependent wind vector in SI units."""

    altitude_meters: q.PressureAltitudeM[float]
    direction_degrees: q.WindDirectionDeg[float]
    speed_meters_per_second: q.WindSpeedMps[float]


def _parse_wind_level(
    _context: CommandParseContext, cursor: CommandCursor
) -> ParseResult[WindLevel]:
    altitude_text = cursor.next_value("a finite altitude")
    if isinstance(altitude_text, Err):
        return altitude_text
    altitude_token = altitude_text.ok()
    try:
        altitude = parse_pressure_altitude_value(altitude_token.value).value
    except ValueError:
        return Err(
            ArgumentIssue.expected("a finite altitude", altitude_token.value, altitude_token.span)
        )
    if not isfinite(altitude):
        return Err(
            ArgumentIssue.expected("a finite altitude", altitude_token.value, altitude_token.span)
        )

    direction_result = parse_field(cursor, float, "a finite direction")
    if isinstance(direction_result, Err):
        return direction_result
    direction_token = direction_result.ok()
    direction = direction_token.value
    if not isfinite(direction):
        return Err(ArgumentIssue.expected("a finite direction", direction, direction_token.span))

    speed_result = parse_field(cursor, parse_speed_value, "a wind speed such as 20KT or 10MPS")
    if isinstance(speed_result, Err):
        return speed_result
    speed_token = speed_result.ok()
    speed = speed_token.value
    span = SourceSpan(altitude_token.span.start, speed_token.span.end)
    return Ok(Spanned(WindLevel(altitude, direction % 360.0, speed), span))


WindLevelArg = Annotated[
    WindLevel,
    CmdParser.fields(_parse_wind_level, ("altitude", "direction", "speed")),
]


class Wind(TrafficArrays, Windfield):
    """Wind field with the stack-command interface of the simulation.

    Combines the [`Windfield`][minisky.traffic.wind.Windfield] data and interpolation with the
    TrafficArrays machinery so the field is cleared on simulation reset.
    Available at runtime as [`runtime.traffic.wind`][minisky.traffic.wind.Wind].
    """

    @command(name="WIND")
    def clear_wind(
        self, position: LatLonDeg, _action: Literal["DEL", "DELETE"]
    ) -> Result[str, str]:
        """Clear wind defined at a position."""
        # Delete the wind field: WIND lat,lon,DEL(ETE)
        self.clear()
        return Ok("")

    @command(name="WIND")
    def set_constant_wind(
        self,
        position: LatLonDeg,
        direction: WindDirectionArg,
        speed: NonNegativeSpeedMps,
    ) -> Result[str, str]:
        """Define altitude-independent wind; use an explicit speed unit such as `20KT`."""
        # No altitude: use the same wind for all altitudes at this position.
        self.addpoint(position.lat, position.lon, direction % 360.0, speed)
        return Ok("")

    @command(name="WIND")
    def set_constant_wind_with_omitted_field(
        self,
        position: LatLonDeg,
        _omitted: Omitted,
        direction: WindDirectionArg,
        speed: NonNegativeSpeedMps,
    ) -> Result[str, str]:
        """Define constant wind with an omitted altitude; use an explicit speed unit such as `20KT`."""
        self.addpoint(position.lat, position.lon, direction % 360.0, speed)
        return Ok("")

    @command(name="WIND")
    def set_wind_profile(
        self, position: LatLonDeg, first: WindLevelArg, *additional: WindLevelArg
    ) -> Result[str, str]:
        """Define altitude-dependent wind vectors; use explicit speed units such as `20KT`."""
        # Several altitude levels are given: build a vertical wind profile.
        levels = (first, *additional)
        if any(
            current.altitude_meters <= previous.altitude_meters
            for previous, current in pairwise(levels)
        ):
            return Err("WIND profile altitudes must be strictly increasing")
        altitude = np.asarray([level.altitude_meters for level in levels])
        direction = np.asarray([level.direction_degrees for level in levels])
        speed = np.asarray([level.speed_meters_per_second for level in levels])
        self.addpoint(position.lat, position.lon, direction, speed, altitude)
        return Ok("")

    @command(name="GETWIND")
    def report(self, position: LatLonDeg, alt: StdPressureAltM | None = None) -> Result[str, str]:
        """Report wind at an aviation position and optional altitude."""
        north, east = self.getdata(position.lat, position.lon, None if alt is None else alt.value)
        direction = (np.degrees(np.arctan2(east, north)) + 180.0) % 360.0
        speed = np.sqrt(north * north + east * east)
        return Ok(
            f"WIND AT {position.lat:.5f}, {position.lon:.5f}: "
            f"{round(direction):03d}/{round(q.mps_to_kt(speed))}"
        )
