"""BlueSky traffic implementation."""

from __future__ import print_function

from collections.abc import Collection
from math import *
from random import randint
from typing import Iterable

import numpy as np

import minisky
from minisky.core import timed_function
from minisky.core.trafficarrays import TrafficArrays
from minisky.stack.argparser import refdata
from minisky.tools import geo
from minisky.tools.aero import (
    Rearth,
    cas2tas,
    casormach2tas,
    fpm,
    ft,
    g0,
    kts,
    nm,
    tas2cas,
    vatmos,
    vcasormach,
    vtas2cas,
    vtas2mach,
)
from minisky.tools.misc import latlon2txt
from minisky.traffic.asas import ConflictDetection, ConflictResolution

from .activewpdata import ActiveWaypoint
from .aporasas import APorASAS
from .autopilot import Autopilot
from .conditional import Condition
from .performance.base import PerfBase
from .trafficgroups import TrafficGroups
from .trails import Trails
from .turbulence import Turbulence
from .uncertainty import SurveillanceUncertainty
from .windsim import WindSim


class Traffic(TrafficArrays):
    """
    Traffic class definition    : Traffic data
    Methods:
        Traffic()            :  constructor
        reset()              :  Reset traffic database w.r.t a/c data
        create(acid,actype,aclat,aclon,achdg,acalt,acspd) : create aircraft
        delete(acid)         : delete an aircraft from traffic data
        deletall()           : delete all traffic
        update(sim)          : do a numerical integration step
        id2idx(name)         : return index in traffic database of given call sign
        engchange(i,engtype) : change engine type of an aircraft
        setnoise(A)          : Add turbulence
    Members: see create
    Created by  : Jacco M. Hoekstra
    """

    def __init__(self):
        super().__init__()

        # Traffic is the toplevel trafficarrays object
        self.setroot(self)

        self.ntraf = 0

        self.cond = Condition()  # Conditional commands list
        self.wind = WindSim()
        self.turbulence = Turbulence()
        self.translvl = 5000.0 * ft  # [m] Default transition level

        # Default commands issued for an aircraft after creation
        self.crecmdlist = []

        with self.settrafarrays():
            # Aircraft Info
            self.id = []  # identifier (string)
            self.type = []  # aircaft type (string)

            # Positions
            self.lat = np.array([])  # latitude [deg]
            self.lon = np.array([])  # longitude [deg]
            self.distflown = np.array([])  # distance travelled [m]
            self.alt = np.array([])  # altitude [m]
            self.hdg = np.array([])  # traffic heading [deg]
            self.trk = np.array([])  # track angle [deg]

            # Velocities
            self.tas = np.array([])  # true airspeed [m/s]
            self.gs = np.array([])  # ground speed [m/s]
            self.gsnorth = np.array([])  # ground speed [m/s]
            self.gseast = np.array([])  # ground speed [m/s]
            self.cas = np.array([])  # calibrated airspeed [m/s]
            self.M = np.array([])  # mach number
            self.vs = np.array([])  # vertical speed [m/s]

            # Acceleration
            self.ax = np.array([])  # [m/s2] current longitudinal acceleration

            # Atmosphere
            self.p = np.array([])  # air pressure [N/m2]
            self.rho = np.array([])  # air density [kg/m3]
            self.Temp = np.array([])  # air temperature [K]
            self.dtemp = np.array([])  # delta t for non-ISA conditions

            # Wind speeds
            self.windnorth = np.array([])  # wind speed north component a/c pos [m/s]
            self.windeast = np.array([])  # wind speed east component a/c pos [m/s]

            # Traffic autopilot settings
            self.selspd = np.array([])  # selected spd(CAS or Mach) [m/s or -]
            self.aptas = np.array([])  # just for initializing
            self.selalt = np.array([])  # selected alt[m]
            self.selvs = np.array([])  # selected vertical speed [m/s]

            # Whether to perform LNAV and VNAV
            self.swlnav = np.array([], dtype=bool)
            self.swvnav = np.array([], dtype=bool)
            self.swvnavspd = np.array([], dtype=bool)

            # Flight Models
            self.cd = ConflictDetection()
            self.cr = ConflictResolution()
            self.ap = Autopilot()
            self.aporasas = APorASAS()
            self.noise = SurveillanceUncertainty()
            self.trails = Trails()
            self.actwp = ActiveWaypoint()
            self.perf = PerfBase()

            # Group Logic
            self.groups = TrafficGroups()

            # Traffic autopilot data
            self.swhdgsel = np.array(
                [], dtype=bool
            )  # determines whether aircraft is turning

            # Traffic autothrottle settings
            self.swats = np.array(
                [], dtype=bool
            )  # Switch indicating whether autothrottle system is on/off
            self.thr = np.array(
                []
            )  # Thottle seeting (0.0-1.0), negative = non-valid/auto

            # Display information on label
            self.label = []  # Text and bitmap of traffic label

            # Miscallaneous
            self.coslat = np.array([])  # Cosine of latitude for computations
            self.eps = np.array([])  # Small nonzero numbers
            self.work = np.array([])  # Work done throughout the flight

        # Default bank angles per flight phase
        self.bphase = np.deg2rad(np.array([15, 35, 35, 35, 15, 45]))

    def reset(self):
        """Clear all traffic data upon simulation reset."""
        # Some child reset functions depend on a correct value of self.ntraf
        self.ntraf = 0
        # This ensures that the traffic arrays (which size is dynamic)
        # are all reset as well, so all lat,lon,sdp etc but also objects adsb
        super().reset()

        # reset performance model
        self.perf.reset()

        # Reset models
        self.wind.clear()

        # Build new modules for turbulence
        self.turbulence.reset()

        # Trajectory noise (turbulence, navigation uncertainties)
        self.setnoise(False)

        # Reset transition level to default value
        self.translvl = 5000.0 * ft

    def cre(
        self,
        acid: str,
        actype: str = "A320",
        lat: float = 53.0,
        lon: float = 4.0,
        hdg: float = 45.0,
        alt: int = 25000,
        spd: int = 300,
    ):
        """Create one or more aircraft."""

        if acid in self.id:
            return False, f"aircraft {acid} already exists"

        # covert to array with 1 element
        acid_ = np.array([acid.upper()])
        actype = np.array([actype])
        lat = np.array([lat])
        lon = np.array([lon])
        alt = np.array([alt])
        hdg = np.array([hdg])
        spd = np.array([spd])

        self.__create_aircraft(acid_, actype, lat, lon, hdg, alt, spd)

        return True, f"arcraft {acid} created"

    def mcre(
        self,
        n: int,
        lat_min: float = 53.0,
        lon_min: float = 0.0,
        lat_max: float = 60.0,
        lon_max: float = 10.0,
        actype: str = "A320",
        acalt: int = None,
        acspd: int = None,
    ):
        """
        Create one or more random aircraft in a specified area.
        By default, use North Sea region.
        """

        # Generate random callsigns
        idtmp = chr(randint(65, 90)) + chr(randint(65, 90)) + "{:>05}"
        acid = [idtmp.format(i) for i in range(n)]

        actype = [actype] * n

        # Generate random positions
        aclat = np.random.rand(n) * (lat_max - lat_min) + lat_min
        aclon = np.random.rand(n) * (lon_max - lon_min) + lon_min
        achdg = np.random.randint(1, 360, n)
        acalt = acalt or np.random.randint(2000, 39000, n) * ft
        acspd = acspd or np.random.randint(250, 450, n) * kts

        self.__create_aircraft(acid, actype, aclat, aclon, achdg, acalt, acspd)

        return True, f"{n} aircraft created"

    def __create_aircraft(
        self,
        acid: Iterable[str],
        actype: Iterable[str],
        lat: Iterable[float],
        lon: Iterable[float],
        hdg: Iterable[int],
        alt: Iterable[int],
        spd: Iterable[int],
    ):
        """Create one or more aircraft."""

        n = len(acid)

        # Adjust the size of all traffic arrays
        super().create(n)
        self.ntraf += n

        # Limit longitude to [-180.0, 180.0]
        lon[lon > 180.0] -= 360.0
        lon[lon < -180.0] += 360.0

        hdg = (refdata.hdg or 0.0) if hdg is None else hdg

        # Aircraft Info
        self.id[-n:] = acid
        self.type[-n:] = actype

        # Positions
        self.lat[-n:] = lat
        self.lon[-n:] = lon
        self.alt[-n:] = alt

        self.hdg[-n:] = hdg
        self.trk[-n:] = hdg

        # Velocities
        self.tas[-n:], self.cas[-n:], self.M[-n:] = vcasormach(spd, alt)
        self.gs[-n:] = self.tas[-n:]
        hdgrad = np.radians(hdg)
        self.gsnorth[-n:] = self.tas[-n:] * np.cos(hdgrad)
        self.gseast[-n:] = self.tas[-n:] * np.sin(hdgrad)

        # Atmosphere
        self.p[-n:], self.rho[-n:], self.Temp[-n:] = vatmos(alt)

        # Wind
        if self.wind.winddim > 0:
            applywind = self.alt[-n:] > 50.0 * ft
            self.windnorth[-n:], self.windeast[-n:] = self.wind.getdata(
                self.lat[-n:], self.lon[-n:], self.alt[-n:]
            )
            self.gsnorth[-n:] = self.gsnorth[-n:] + self.windnorth[-n:] * applywind
            self.gseast[-n:] = self.gseast[-n:] + self.windeast[-n:] * applywind
            self.trk[-n:] = np.logical_not(applywind) * hdg + applywind * np.degrees(
                np.arctan2(self.gseast[-n:], self.gsnorth[-n:])
            )
            self.gs[-n:] = np.sqrt(self.gsnorth[-n:] ** 2 + self.gseast[-n:] ** 2)
        else:
            self.windnorth[-n:] = 0.0
            self.windeast[-n:] = 0.0

        # Traffic autopilot settings
        self.selspd[-n:] = self.cas[-n:]
        self.aptas[-n:] = self.tas[-n:]
        self.selalt[-n:] = self.alt[-n:]

        # Display information on label
        self.label[-n:] = n * [["", "", "", 0]]

        # Miscallaneous: Cosine of latitude for flat-earth aproximations
        self.coslat[-n:] = np.cos(np.radians(lat))
        self.eps[-n:] = 0.01

        # Finally call create for child TrafficArrays. This only needs to be done
        # manually in Traffic.
        self.create_children(n)

        # Record as individual CRE commands for repeatability
        # print(self.ntraf-n,self.ntraf)
        # for j in range(self.ntraf - n, self.ntraf):
        #     # Reconstruct CRE command
        #     line = "CRE " + ",".join(
        #         [
        #             self.id[j],
        #             self.type[j],
        #             str(self.lat[j]),
        #             str(self.lon[j]),
        #             str(round(self.trk[j])),
        #             str(round(self.alt[j] / ft)),
        #             str(round(self.cas[j] / kts)),
        #         ]
        #     )
        #     # Savecmd(cmd,line): line is saved, cmd is used to prevent recording PAN & ZOOM commands and CRE
        #     # So insert a dummy command to record the line
        #     savecmd("---", line)

        # Check for crecmdlist: contains commands to be issued for this a/c
        # If any are there, then stack them for all aircraft
        for j in range(self.ntraf - n, self.ntraf):
            for cmdtxt in self.crecmdlist:
                minisky.stack.stack(self.id[j] + " " + cmdtxt)

    def creconfs(
        self, acid, actype, targetidx, dpsi, dcpa, tlosh, dH=None, tlosv=None, spd=None
    ):
        """Create an aircraft in conflict with target aircraft.

        Arguments:
        - acid: callsign of new aircraft
        - actype: aircraft type of new aircraft
        - targetidx: id (callsign) of target aircraft
        - dpsi: Conflict angle (angle between tracks of ownship and intruder) (deg)
        - cpa: Predicted distance at closest point of approach (NM)
        - tlosh: Horizontal time to loss of separation ((hh:mm:)sec)
        - dH: Vertical distance (ft)
        - tlosv: Vertical time to loss of separation
        - spd: Speed of new aircraft (CAS/Mach, kts/-)
        """
        latref = self.lat[targetidx]  # deg
        lonref = self.lon[targetidx]  # deg
        altref = self.alt[targetidx]  # m
        trkref = radians(self.trk[targetidx])
        gsref = self.gs[targetidx]  # m/s
        tasref = self.tas[targetidx]  # m/s
        vsref = self.vs[targetidx]  # m/s
        cpa = dcpa * nm
        pzr = minisky.core.settings.asas_pzr * nm
        pzh = minisky.core.settings.asas_pzh * ft
        trk = trkref + radians(dpsi)

        if dH is None:
            acalt = altref
            acvs = 0.0
        else:
            acalt = altref + dH
            tlosv = tlosh if tlosv is None else tlosv
            acvs = vsref - np.sign(dH) * (abs(dH) - pzh) / tlosv

        if spd:
            # CAS or Mach provided: convert to groundspeed, assuming that
            # wind at intruder position is similar to wind at ownship position
            tas = tasref if spd is None else casormach2tas(spd, acalt)
            tasn, tase = tas * cos(trk), tas * sin(trk)
            wn, we = self.wind.getdata(latref, lonref, acalt)
            gsn, gse = tasn + wn, tase + we
        else:
            # Groundspeed is the same as ownship
            gsn, gse = gsref * cos(trk), gsref * sin(trk)

        # Horizontal relative velocity vector
        vreln, vrele = gsref * cos(trkref) - gsn, gsref * sin(trkref) - gse
        # Relative velocity magnitude
        vrel = sqrt(vreln * vreln + vrele * vrele)
        # Relative travel distance to closest point of approach
        drelcpa = tlosh * vrel + (0 if cpa > pzr else sqrt(pzr * pzr - cpa * cpa))
        # Initial intruder distance
        dist = sqrt(drelcpa * drelcpa + cpa * cpa)
        # Rotation matrix diagonal and cross elements for distance vector
        rd = drelcpa / dist
        rx = cpa / dist
        # Rotate relative velocity vector to obtain intruder bearing
        brn = degrees(atan2(-rx * vreln + rd * vrele, rd * vreln + rx * vrele))

        # Calculate intruder lat/lon
        aclat, aclon = geo.kwikpos(latref, lonref, brn, dist / nm)
        # convert groundspeed to CAS, and track to heading using actual
        # intruder position
        wn, we = self.wind.getdata(aclat, aclon, acalt)
        tasn, tase = gsn - wn, gse - we
        acspd = tas2cas(sqrt(tasn * tasn + tase * tase), acalt)
        achdg = degrees(atan2(tase, tasn))

        # Create and, when necessary, set vertical speed
        self.cre(acid, actype, aclat, aclon, achdg, acalt, acspd)
        self.ap.selaltcmd(len(self.lat) - 1, altref, acvs)
        self.vs[-1] = acvs

    def delete(self, idx):
        """Delete an aircraft"""
        # If this is a multiple delete, sort first for list delete
        # (which will use list in reverse order to avoid index confusion)
        if isinstance(idx, Collection):
            idx = np.sort(idx)

        # Call the actual delete function
        super().delete(idx)

        # Update number of aircraft
        self.ntraf = len(self.lat)
        return True

    def update(self):
        # Update only if there is traffic ---------------------
        if self.ntraf == 0:
            return

        # ---------- Atmosphere --------------------------------
        self.p, self.rho, self.Temp = vatmos(self.alt)

        # ---------- Trajectory Noise Update -------------------------------
        self.noise.update()

        # ---------- Fly the Aircraft --------------------------
        self.ap.update()  # Autopilot logic
        self.update_asas()  # Airborne Separation Assurance
        self.aporasas.update()  # Decide to use autopilot or ASAS for commands

        # ---------- Performance Update ------------------------
        self.perf.update()

        # ---------- Limit commanded speeds based on performance ------------------------------
        self.aporasas.tas, self.aporasas.vs, self.aporasas.alt = self.perf.limits(
            self.aporasas.tas, self.aporasas.vs, self.aporasas.alt, self.ax
        )

        # ---------- Kinematics --------------------------------
        self.update_airspeed()
        self.update_groundspeed()
        self.update_pos()

        # ---------- Simulate Turbulence -----------------------
        self.turbulence.update()

        # Check whether new traffic state triggers conditional commands
        self.cond.update()

        # ---------- Aftermath ---------------------------------
        self.trails.update()

    @timed_function(name="asas", dt=minisky.core.settings.asas_dt, manual=True)
    def update_asas(self):
        # Conflict detection and resolution
        self.cd.update(self, self)
        self.cr.update(self.cd, self, self)

    def update_airspeed(self):
        # Compute horizontal acceleration
        delta_spd = self.aporasas.tas - self.tas
        need_ax = np.abs(delta_spd) > np.abs(minisky.sim.simdt * self.perf.axmax)
        self.ax = need_ax * np.sign(delta_spd) * self.perf.axmax
        # Update velocities
        self.tas = np.where(
            need_ax, self.tas + self.ax * minisky.sim.simdt, self.aporasas.tas
        )
        self.cas = vtas2cas(self.tas, self.alt)
        self.M = vtas2mach(self.tas, self.alt)

        # Turning bank triangle
        # tan phi = a centrigugal/a grav = omega^2 * R / g = omega * V /g
        # => omega = (g tan phi)/V
        turnrate = np.degrees(
            g0
            * np.tan(
                np.where(
                    self.ap.turnphi > self.eps * self.eps,
                    self.ap.turnphi,
                    self.ap.bankdef,
                )
            )
            / np.maximum(self.tas, self.eps)
        )
        delhdg = (self.aporasas.hdg - self.hdg + 180) % 360 - 180  # [deg]
        self.swhdgsel = np.abs(delhdg) > np.abs(minisky.sim.simdt * turnrate)

        # Update heading
        self.hdg = (
            np.where(
                self.swhdgsel,
                self.hdg + minisky.sim.simdt * turnrate * np.sign(delhdg),
                self.aporasas.hdg,
            )
            % 360.0
        )

        # Update vertical speed (alt select, capture and hold autopilot mode)
        delta_alt = self.aporasas.alt - self.alt
        # Old dead band version:
        #        self.swaltsel = np.abs(delta_alt) > np.maximum(
        #            10 * ft, np.abs(2 * minisky.sim.simdt * self.vs))

        # Update version: time based engage of altitude capture (to adapt for UAV vs airliner scale)
        self.swaltsel = np.abs(delta_alt) > 1.05 * np.maximum(
            np.abs(minisky.sim.simdt * self.aporasas.vs),
            np.abs(minisky.sim.simdt * self.vs),
        )
        target_vs = self.swaltsel * np.sign(delta_alt) * np.abs(self.aporasas.vs)
        delta_vs = target_vs - self.vs
        # print(delta_vs / fpm)
        need_az = np.abs(delta_vs) > 300 * fpm  # small threshold
        self.az = (
            need_az * np.sign(delta_vs) * (300 * fpm)
        )  # fixed vertical acc approx 1.6 m/s^2
        self.vs = np.where(need_az, self.vs + self.az * minisky.sim.simdt, target_vs)
        self.vs = np.where(np.isfinite(self.vs), self.vs, 0)  # fix vs nan issue

    def update_groundspeed(self):
        # Compute ground speed and track from heading, airspeed and wind
        if self.wind.winddim == 0:  # no wind
            self.gsnorth = self.tas * np.cos(np.radians(self.hdg))
            self.gseast = self.tas * np.sin(np.radians(self.hdg))

            self.gs = self.tas
            self.trk = self.hdg
            self.windnorth[:], self.windeast[:] = 0.0, 0.0

        else:
            applywind = self.alt > 50.0 * ft  # Only apply wind when airborne

            vnwnd, vewnd = self.wind.getdata(self.lat, self.lon, self.alt)
            self.windnorth[:], self.windeast[:] = vnwnd, vewnd
            self.gsnorth = (
                self.tas * np.cos(np.radians(self.hdg)) + self.windnorth * applywind
            )
            self.gseast = (
                self.tas * np.sin(np.radians(self.hdg)) + self.windeast * applywind
            )

            self.gs = np.logical_not(applywind) * self.tas + applywind * np.sqrt(
                self.gsnorth**2 + self.gseast**2
            )

            self.trk = (
                np.logical_not(applywind) * self.hdg
                + applywind * np.degrees(np.arctan2(self.gseast, self.gsnorth)) % 360.0
            )

        self.work += (
            self.perf.thrust
            * minisky.sim.simdt
            * np.sqrt(self.gs * self.gs + self.vs * self.vs)
        )

    def update_pos(self):
        # Update position
        self.alt = np.where(
            self.swaltsel,
            np.round(self.alt + self.vs * minisky.sim.simdt, 6),
            self.aporasas.alt,
        )
        self.lat = self.lat + np.degrees(minisky.sim.simdt * self.gsnorth / Rearth)
        self.coslat = np.cos(np.deg2rad(self.lat))
        self.lon = self.lon + np.degrees(
            minisky.sim.simdt * self.gseast / self.coslat / Rearth
        )
        self.distflown += self.gs * minisky.sim.simdt

    def id2idx(self, acid):
        # print(acid)
        """Find index of aircraft id"""
        if not isinstance(acid, str):
            # id2idx is called for multiple id's
            # Fast way of finding indices of all ACID's in a given list
            tmp = dict((v, i) for i, v in enumerate(self.id))
            return [tmp.get(acidi, -1) for acidi in acid]
        else:
            # Catch last created id (* or # symbol)
            if acid in ("#", "*"):
                return self.ntraf - 1

            try:
                return self.id.index(acid.upper())
            except:
                return -1

    def setnoise(self, noise=None):
        """Noise (turbulence, ADBS-transmission noise, ADSB-truncated effect)"""
        if noise is None:
            return True, "Noise is currently " + (
                "on" if self.turbulence.active else "off"
            )

        self.turbulence.setnoise(noise)
        self.noise.setnoise(noise)
        return True

    def engchange(self, acid, engid):
        """Change of engines"""
        self.perf.engchange(acid, engid)
        return

    def move(self, idx, lat, lon, alt=None, hdg=None, casmach=None, vspd=None):
        self.lat[idx] = lat
        self.lon[idx] = lon

        if alt is not None:
            self.alt[idx] = alt
            self.selalt[idx] = alt

        if hdg is not None:
            self.hdg[idx] = hdg
            self.ap.trk[idx] = hdg

        if casmach is not None:
            self.tas[idx], self.selspd[idx], _ = vcasormach(casmach, alt)

        if vspd is not None:
            self.vs[idx] = vspd
            self.swvnav[idx] = False

    def poscommand(self, id_or_name):
        """POS command: Show info or an aircraft, airport, waypoint or navaid"""

        # Aircraft index
        if type(id_or_name) == int:
            return self.position_aircraft(id_or_name)

        # Waypoint: airport, navaid or fix
        else:
            return self.position_by_name(id_or_name)

    def position_aircraft(self, idx: int):
        """POS command: Show info or an aircraft, airport, waypoint or navaid"""

        acid = self.id[idx]

        actype = self.type[idx]
        latlon = latlon2txt(self.lat[idx], self.lon[idx])
        alt = round(self.alt[idx] / ft)
        hdg = round(self.hdg[idx])
        trk = round(self.trk[idx])
        cas = round(self.cas[idx] / kts)
        tas = round(self.tas[idx] / kts)
        gs = round(self.gs[idx] / kts)
        M = self.M[idx]
        VS = round(self.vs[idx] / ft * 60.0)
        route = self.ap.route[idx]

        # Position report
        info = (
            f"Information on aircraft {acid} (index: {idx})\n"
            f"Aircraft typecde: {actype} \n"
            f"Position: {latlon}\n"
            f"Hdg: {hdg:03d} \tTrk: {trk:03d}\n"
            f"Alt: {alt} ft\tV/S: {VS} fpm\n"
            f"CAS/TAS/GS: {cas}/{tas}/{gs} kts   M: {M:.3f}\n"
        )

        # FMS AP modes
        if self.swlnav[idx] and route.nwp > 0 and route.iactwp >= 0:
            if self.swvnav[idx]:
                if self.swvnavspd[idx]:
                    info = info + "VNAV (incl.VNAVSPD), "
                else:
                    info = info + "VNAV (NOT VNAVSPD), "

            info += "LNAV to " + route.wpname[route.iactwp] + "\n"

        # Flight info: Destination and origin
        if self.ap.orig[idx] != "" or self.ap.dest[idx] != "":
            info = info + "Flying"

            if self.ap.orig[idx] != "":
                info = info + " from " + self.ap.orig[idx]

            if self.ap.dest[idx] != "":
                info = info + " to " + self.ap.dest[idx]

        return True, info

    def position_by_name(self, name: str):
        name = name.upper()

        lines = "Information on " + name + ":\n"

        # First try airports (most used and shorter, hence faster list)
        idx_airport = minisky.navdb.getaptidx(name)
        if idx_airport >= 0:
            airport_sizes = ["large", "medium", "small"]
            airport_size = airport_sizes[max(-1, minisky.navdb.aptype[idx_airport] - 1)]

            aptname = minisky.navdb.aptname[idx_airport]
            aptlat = minisky.navdb.aptlat[idx_airport]
            aptlon = minisky.navdb.aptlon[idx_airport]
            aptelev = minisky.navdb.aptelev[idx_airport]

            # country informatation
            idx_cc = minisky.navdb.cocode2.index(
                minisky.navdb.aptco[idx_airport].upper()
            )
            country_name = minisky.navdb.coname[idx_cc].upper()
            country_code = minisky.navdb.aptco[idx_airport]

            lines += (
                f"{aptname} is a {airport_size} airport in {country_name} ({country_code}):\n"
                f"Position: {latlon2txt(aptlat, aptlon)}\n"
                f"Elevation: {int(round(aptelev / ft))} ft \n"
            )

            if minisky.navdb.aptid[idx_airport] in minisky.navdb.rwythresholds:
                runways = minisky.navdb.rwythresholds[
                    minisky.navdb.aptid[idx_airport]
                ].keys()
                if runways:
                    lines += f"Runways: {', '.join(runways)}\n"

            return True, lines

        # Not found as airport, try waypoints & navaids
        else:
            idx_waypoints = minisky.navdb.getwpindices(name)
            if idx_waypoints[0] >= 0:
                typetxt = ""
                desctxt = ""
                lastdesc = "XXXXXXXX"
                for i in idx_waypoints:
                    # One line type text
                    if typetxt == "":
                        typetxt = typetxt + minisky.navdb.wptype[i]
                    else:
                        typetxt = typetxt + " and " + minisky.navdb.wptype[i]

                    # Description: multi-line
                    samedesc = minisky.navdb.wpdesc[i] == lastdesc
                    if desctxt == "":
                        desctxt = desctxt + minisky.navdb.wpdesc[i]
                        lastdesc = minisky.navdb.wpdesc[i]
                    elif not samedesc:
                        desctxt = desctxt + "\n" + minisky.navdb.wpdesc[i]
                        lastdesc = minisky.navdb.wpdesc[i]

                    # Navaid: frequency
                    if (
                        minisky.navdb.wptype[i] in ["VOR", "DME", "TACAN"]
                        and not samedesc
                    ):
                        desctxt = desctxt + " " + str(minisky.navdb.wpfreq[i]) + " MHz"
                    elif minisky.navdb.wptype[i] == "NDB" and not samedesc:
                        desctxt = desctxt + " " + str(minisky.navdb.wpfreq[i]) + " kHz"

                iwp = idx_waypoints[0]

                # Basic info
                lines += (
                    f"{name} is a {typetxt} with \n"
                    f"Position: {latlon2txt(minisky.navdb.wplat[iwp], minisky.navdb.wplon[iwp])}\n"
                )

                # Navaids have description
                if len(desctxt) > 0:
                    lines += f"{desctxt}\n"

                # VOR give variation
                if minisky.navdb.wptype[iwp] == "VOR":
                    lines += f"Variation: {minisky.navdb.wpvar[iwp]} deg\n"

                # How many others?
                n_other = minisky.navdb.wpid.count(name) - len(idx_waypoints)
                if n_other > 0:
                    lines += (
                        f"Attention: {n_other} other waypoint(s) also has name {name}\n"
                    )

                # In which airways?
                connect = minisky.navdb.listconnections(
                    name, minisky.navdb.wplat[iwp], minisky.navdb.wplon[iwp]
                )
                if len(connect) > 0:
                    awset = set([])
                    for c in connect:
                        awset.add(c[0])

                    lines += f"Connected to airways: {'-'.join(awset)}\n"

                return True, lines

            # Try airway id
            else:  # airway
                awid = name
                airway = minisky.navdb.listairway(awid)
                if len(airway) > 0:
                    lines = ""
                    for segment in airway:
                        lines += f"Airway {awid}: {' - '.join(segment)}\n"
                    return True, lines

        # nothing matched
        return False, f"{name} not found as a/c, airport, navaid, or waypoint"

        # Show what we found on airport and navaid/waypoint

    def settrans(self, alt=-999.0):
        """Set or show transition level"""
        # in case a valid value is ginve set it
        if alt > -900.0:
            if alt > 0.0:
                self.translvl = alt
                return True
            return False, "Transition level needs to be ft/FL and larger than zero"

        # In case no value is given, show it
        tlvl = int(round(self.translvl / ft))
        return True, f"Transition level = {tlvl}/FL{int(round(tlvl / 100.0))}"

    def setbanklim(self, idx, bankangle=None):
        """Set bank limit for given aircraft."""
        if bankangle:
            self.ap.bankdef[idx] = np.radians(bankangle)  # [rad]
            return True
        return (
            True,
            f"Banklimit of {self.id[idx]} is {int(np.degrees(self.ap.bankdef[idx]))} deg",
        )

    def setthrottle(self, idx, throttle=""):
        """Set throttle to given value or AUTO, meaning autothrottle on (default)"""

        if throttle:
            if throttle in ("AUTO", "OFF"):  # throttle mode off, ATS on
                self.swats[idx] = True  # Autothrottle on
                self.thr[idx] = -999.0  # Set to invalid

            elif throttle == "IDLE":
                self.swats[idx] = False
                self.thr[idx] = 0.0

            else:
                # Check for percent unit
                if throttle.count("%") == 1:
                    throttle = throttle.replace("%", "")
                    factor = 0.01
                else:
                    factor = 1.0

                # Remaining option is that it is a float, so try conversion
                try:
                    x = factor * float(throttle)
                except:
                    return False, "THR invalid argument " + throttle

                # Check whether value makes sense
                if x < 0.0 or x > 1.0:
                    return (
                        False,
                        "THR invalid value " + throttle + ". Needs to be [0.0 , 1.0]",
                    )

                # Valid value, set throttle and disable autothrottle
                self.swats[idx] = False
                self.thr[idx] = x

            return True

        if self.swats[idx]:
            return True, "ATS of " + self.id[idx] + " is ON"
        return True, "ATS of " + self.id[idx] + " is OFF. THR is " + str(self.thr[idx])

    def crecmd(self, cmdline):
        """CRECMD command: list of commands to be issued for each aircraft after creation
        This commands adds a command to the list of default commands.
        """
        # Help text need or info on current list?
        if cmdline == "" or cmdline == "?":
            if len(self.crecmdlist) > 0:
                allcmds = ""
                for i, txt in enumerate(self.crecmdlist):
                    if i == 0:
                        allcmds = "[acid] " + txt
                    else:
                        allcmds += "; [acid] " + txt
                return True, "CRECMD list: " + allcmds
            else:
                return (
                    True,
                    "CRECMD will add a/c specific commands to an aircraft after creation",
                )
        # Command to be added to list
        else:
            self.crecmdlist.append(cmdline)
        return True, ""

    def clrcrecmd(self):
        """CRECMD command: list of commands to be issued for each aircraft after creation
        This commands adds a command to the list of default commands.
        """
        ncrecmd = len(self.crecmdlist)
        if ncrecmd == 0:
            return True, "CLRCRECMD deletes all commands on clears command"
        else:
            self.crecmdlist = []
            return True, str("All", ncrecmd, "crecmd commands deleted.")
