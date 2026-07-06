"""Simulate wind in BlueSky.

Implements a wind field defined by wind vectors at arbitrary lat/lon
positions, optionally with altitude profiles. The field is interpolated
(inverse-distance weighting horizontally, linear in altitude) to obtain
the wind at any aircraft position. :class:`Windfield` contains the field
data and interpolation; :class:`Wind` adds the stack-command interface
(WIND to define wind, GETWIND to query it) and is available at runtime as
``minisky.traf.wind``. The traffic model uses the wind to compute ground
speed and track from heading and airspeed.
"""

from typing import Any

import numpy as np
from scipy.interpolate import LinearNDInterpolator, interp1d

from minisky.core.trafficarrays import TrafficArrays
from minisky.stack.argparser import Alt, Lat, Lon
from minisky.tools.aero import ft, kts


class Windfield:
    """Windfield class:
    Methods:
        clear()    = clear windfield, no wind vectors defined

        addpoint(lat,lon,winddir,winddspd,windalt=None)
                   = add a wind vector to a position,
                     windvector can be arrays for altitudes (optional)
                     returns index of vector (0,1,2,3,..)
                     all units are SI units, angles in degrees

        get(lat,lon,alt=0)
                   = get wind vector for given position and optional
                     altitude, all can be arrays,
                     vnorth and veast will be returned in same dimension

        remove(idx) = remove a defined profile using the index

    Members:
        lat(nvec)          = latitudes of wind definitions
        lon(nvec)          = longitudes of wind definitions
        altaxis(nalt)      = altitude axis (fixed, 250 m resolution)

        vnorth(nalt,nvec)  = wind north component [m/s]
        veast(nalt,nvec)   = wind east component [m/s]

        winddim   = Windfield dimension, will automatically be detected:
                      0 = no wind
                      1 = constant wind
                      2 = 2D field (no alt profiles),
                      3 = 3D field (alt dependent wind at some points)

    """

    def __init__(self) -> None:
        # For altitude use fixed axis to allow vectorisation later
        self.altmax = 45000.0 * ft  # [m]
        self.altstep = 100.0 * ft  # [m]

        # Axis
        self.altaxis = np.arange(0.0, self.altmax + self.altstep, self.altstep)
        self.idxalt = np.arange(0, len(self.altaxis), 1.0)
        self.nalt = len(self.altaxis)

        # List of indices of points with an altitude profile (for 3D check)
        self.iprof = []

        # Clear actual field
        self.clear()
        return

    def clear(self) -> None:  # Clear actual field
        """Remove all wind vectors, leaving a windless (winddim 0) field."""
        # Windfield dimension will automatically be detected:
        # 0 = no wind, 1 = constant wind, 2 = 2D field (no alt profiles),
        # 3 = 3D field (alt matters), used to speed up interpolation
        self.winddim = 0
        self.lat = np.array([])
        self.lon = np.array([])
        self.vnorth = np.array([[]])
        self.veast = np.array([[]])
        self.nvec = 0
        self.fe = None
        self.fn = None
        return

    def addpointvne(
        self,
        lat: np.ndarray,
        lon: np.ndarray,
        vnorth: np.ndarray,
        veast: np.ndarray,
        windalt: "np.ndarray | None" = None,
    ) -> None:
        """Add wind vectors given as north/east speed components.

        Vectorized alternative to addpoint() for defining many wind points
        at once. When altitudes are given, a scipy interpolator over
        (altitude, lat, lon) is set up for regular grids; otherwise the
        profiles are resampled onto the fixed altitude axis.

        Args:
            lat: Latitudes of the wind definition points [deg].
            lon: Longitudes of the wind definition points [deg].
            vnorth: North wind component [m/s]; 2D (altitude x position)
                when windalt is given.
            veast: East wind component [m/s]; same shape as vnorth.
            windalt: Optional array of altitudes [m] belonging to the rows
                of vnorth/veast; makes the field 3D.
        """
        if windalt is not None and len(windalt) > 1:
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
                except Exception:
                    # Create vn, ve if RGI is not possible
                    vnaxis = fnorth(self.altaxis).T
                    veaxis = feast(self.altaxis).T
            else:
                # Create vn, ve if less than 4 coords are present
                vnaxis = fnorth(self.altaxis).T
                veaxis = feast(self.altaxis).T

            self.winddim = 3
            self.iprof.append(len(self.lat) + 1)

        else:
            vnaxis = vnorth
            veaxis = veast

        self.nvec += len(lat)
        self.lat = np.append(self.lat, lat)
        self.lon = np.append(self.lon, lon)

        if self.vnorth.size == 0:
            self.vnorth = vnaxis
            self.veast = veaxis
        else:
            self.vnorth = np.concatenate((self.vnorth, vnaxis), axis=1)
            self.veast = np.concatenate((self.veast, veaxis), axis=1)

        if self.winddim < 3:  # No 3D => set dim to 0,1 or 2 dep on nr of points
            self.winddim = min(2, len(self.lat))

    def addpoint(
        self,
        lat: float,
        lon: float,
        winddir: Any,
        windspd: Any,
        windalt: Any = None,
    ) -> int:
        """Add a wind vector (direction/speed) at a lat/lon position.

        The wind is converted to north/east components and stored on the
        fixed altitude axis. When an altitude array is given, the wind
        profile is interpolated onto that axis and the field becomes 3D
        (altitude dependent).

        Args:
            lat: Latitude of the wind definition point [deg].
            lon: Longitude of the wind definition point [deg].
            winddir: Direction the wind comes from [deg]; array when an
                altitude profile is given.
            windspd: Wind speed [m/s]; same dimension as winddir.
            windalt: Optional altitudes [m] belonging to winddir/windspd,
                defining an altitude profile at this position.

        Returns:
            int: Index of the added wind point (for use with remove()).
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

        idx = len(self.lat) - 1

        if self.nvec == 0:
            self.vnorth = np.array([vnaxis]).transpose()
            self.veast = np.array([veaxis]).transpose()

        else:
            self.vnorth = np.append(self.vnorth, np.array([vnaxis]).transpose(), axis=1)
            self.veast = np.append(self.veast, np.array([veaxis]).transpose(), axis=1)

        if self.winddim < 3:  # No 3D => set dim to 0,1 or 2 dep on nr of points
            self.winddim = min(2, len(self.lat))

        if prof3D:
            self.winddim = 3
            self.iprof.append(idx)

        self.nvec = self.nvec + 1

        return idx  # return index of added point

    def getdata(
        self, userlat: Any, userlon: Any, useralt: Any = 0.0
    ) -> "tuple[Any, Any]":  # in case no altitude specified and field is 3D, use sea level wind
        """Interpolate the wind field at one or more positions.

        Uses inverse-distance-squared weighting between the defined wind
        points horizontally, and linear interpolation along the altitude
        axis for 3D fields. Constant and empty fields are handled as
        special cases. When no altitude is given for a 3D field, sea-level
        wind is returned.

        Args:
            userlat: Latitude(s) [deg]; scalar, list or ndarray.
            userlon: Longitude(s) [deg]; same shape as userlat.
            useralt: Altitude(s) [m]; scalar, list or ndarray (default 0).

        Returns:
            tuple: (vnorth, veast): north and east wind components [m/s],
            with the same type/shape as the given positions.
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

        # Check if RGI functions are present, if so use them for interpolation
        if self.fe is not None and self.fn is not None:
            vnorth = self.fn(np.concatenate((alt.reshape(1, -1), lat, lon), axis=0).T)
            veast = self.fe(np.concatenate((alt.reshape(1, -1), lat, lon), axis=0).T)
        else:
            # Check dimension of wind field
            if self.winddim == 0:  # None = no wind
                vnorth = np.zeros(npos)
                veast = np.zeros(npos)

            elif self.winddim == 1:  # Constant = one point defined, so constant wind
                vnorth = np.ones(npos) * self.vnorth[0, 0]
                veast = np.ones(npos) * self.veast[0, 0]

            elif self.winddim >= 2:  # 2D/3D field = more points defined but no altitude profile
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
                if self.winddim == 2 or (
                    not isinstance(useralt, (list, np.ndarray)) and useralt == 0.0
                ):  # 2D field no altitude interpolation
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
            idx: Index of the point, as returned by addpoint(). The field
                dimension (winddim) is re-evaluated after removal.
        """
        if idx < len(self.lat):
            self.lat = np.delete(self.lat, idx)
            self.lon = np.delete(self.lon, idx)

            self.vnorth = np.delete(self.vnorth, idx, axis=1)
            self.veast = np.delete(self.veast, idx, axis=1)

            if idx in self.iprof:
                self.iprof.remove(idx)

            if self.winddim < 3 or len(self.iprof) == 0 or len(self.lat) == 0:
                self.winddim = min(2, len(self.lat))  # Check for 0, 1D, 2D or 3D

        return


class Wind(TrafficArrays, Windfield):
    """Wind field with the stack-command interface of the simulation.

    Combines the :class:`Windfield` data and interpolation with the
    TrafficArrays machinery so the field is cleared on simulation reset.
    Implements the WIND (add()) and GETWIND (get()) stack commands.
    Available at runtime as ``minisky.traf.wind``.
    """

    def add(self, lat: Lat, lon: Lon, *winddata: float) -> "bool | tuple[bool, str]":
        """Define a wind vector as part of the 2D or 3D wind field.

        Implements the WIND stack command.

        Arguments:
        - lat/lon: Horizonal position to define wind vector(s)
        - winddata:
          - If the wind at this location is independent of altitude
            winddata has two elements:
            - direction [degrees]
            - speed (magnitude) [knots]
          - If the wind varies with altitude winddata has three elements:
            - altitude [ft]
            - direction [degrees]
            - speed (magnitude) [knots]
            In this case, repeating combinations of alt/dir/spd can be provided
            to specify wind at multiple altitudes.
          - If winddata contains "DEL" or "DELETE" the whole wind field is
            deleted (e.g. WIND lat,lon,DEL), like the DEL WIND command.
        """
        ndata = len(winddata)

        # Delete the wind field: WIND lat,lon,DEL(ETE)
        # Check this first: it would otherwise be shadowed by the numeric forms
        if "DEL" in winddata or "DELETE" in winddata:
            self.clear()

        # No altitude or just one: same wind for all altitudes at this position
        elif ndata == 2 or (ndata == 3 and winddata[0] is None):  # only one point, ignore altitude
            if winddata[-2] is None or winddata[-1] is None:
                return False, "Wind direction and speed needed."

            self.addpoint(lat, lon, winddata[-2], winddata[-1] * kts)

        # More than one altitude is given
        elif ndata >= 3:
            windarr = np.array(winddata)
            dirarr = windarr[1::3]
            spdarr = windarr[2::3] * kts
            altarr = windarr[0::3] * ft

            self.addpoint(lat, lon, dirarr, spdarr, altarr)

        else:  # Something is wrong
            return False, "Winddata not recognized"

        return True

    def get(self, lat: Lat, lon: Lon, alt: Alt | None = None) -> "tuple[bool, str]":
        """Get wind at a specified position (and optionally at altitude)

        Implements the GETWIND stack command. The result is reported as
        direction/speed text (e.g. "270/25", speed in kts).

        Arguments:
        - lat, lon: Horizontal position where wind should be determined [deg]
        - alt: Altitude at which wind should be determined [m]
          (stack input in ft)

        Returns:
            tuple: (True, text with wind direction [deg] and speed [kts]).
        """
        vn, ve = self.getdata(lat, lon, alt)

        wdir = (np.degrees(np.arctan2(ve, vn)) + 180.0) % 360.0
        wspd = np.sqrt(vn * vn + ve * ve)

        txt = f"WIND AT {lat:.5f}, {lon:.5f}: {int(round(wdir)):03d}/{int(round(wspd / kts))}"

        return True, txt
