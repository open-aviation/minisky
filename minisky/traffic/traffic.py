"""BlueSky traffic implementation.

Defines the :class:`Traffic` class, the top-level traffic database of the
simulator. It holds all per-aircraft state (position, attitude, speeds,
atmosphere, autopilot selections) as numpy arrays, owns the sub-models
(autopilot, performance, conflict detection/resolution, wind, turbulence,
trails, groups), and performs the numerical integration of the aircraft
states each simulation time step.

A single instance is created at simulator start-up and made available as
``minisky.traf``. Several methods double as stack-command implementations
(CRE, MCRE, CRECONFS, MOVE, POS, BANK, THR, NOISE, CRECMD, ...).
"""

from collections.abc import Collection, Iterable
from random import randint
from typing import overload

import numpy as np

import minisky
from minisky.core.trafficarrays import TrafficArrays
from minisky.tools import geo
from minisky.tools.aero import (
    Rearth,
    casormach,
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
from minisky.tools.convert import latlon2txt
from minisky.traffic.asas import ConflictDetection, ConflictResolution

from .activewpdata import ActiveWaypoint
from .aporasas import APorASAS
from .autopilot import Autopilot
from .conditional import Condition
from .performance.perfoap import OpenAP
from .trafficgroups import TrafficGroups
from .trails import Trails
from .turbulence import Turbulence
from .uncertainty import SurveillanceUncertainty
from .wind import Wind


class Traffic(TrafficArrays):
    """Central traffic database holding the state of all simulated aircraft.

    Traffic is the top-level :class:`TrafficArrays` object: all per-aircraft
    arrays registered by its child entities (autopilot, active waypoint data,
    performance model, conflict detection/resolution, etc.) grow and shrink
    together when aircraft are created or deleted. A single instance is
    available at runtime as ``minisky.traf``.

    Every simulation step, :meth:`update` refreshes the atmosphere, runs the
    autopilot and separation-assurance logic, applies performance limits, and
    numerically integrates airspeed, heading, vertical speed and position of
    all aircraft. All internal state is kept in SI units; stack commands use
    aviation units (ft, kts, FL) and are converted on input/output.

    Attributes:
        ntraf (int): Number of aircraft currently in the simulation.
        callsign (list): Aircraft identifier (callsign) strings.
        typecode (list): ICAO aircraft type designators (e.g. "A320").
        lat (ndarray): Latitude [deg].
        lon (ndarray): Longitude [deg].
        distflown (ndarray): Distance flown since creation [m].
        alt (ndarray): Altitude [m].
        hdg (ndarray): Heading [deg].
        trk (ndarray): Track angle over the ground [deg].
        tas (ndarray): True airspeed [m/s].
        gs (ndarray): Ground speed [m/s].
        gsnorth (ndarray): North component of ground speed [m/s].
        gseast (ndarray): East component of ground speed [m/s].
        cas (ndarray): Calibrated airspeed [m/s].
        M (ndarray): Mach number [-].
        vs (ndarray): Vertical speed [m/s].
        ax (ndarray): Current longitudinal acceleration [m/s2].
        p (ndarray): Ambient air pressure [Pa].
        rho (ndarray): Ambient air density [kg/m3].
        Temp (ndarray): Ambient air temperature [K].
        dtemp (ndarray): Temperature offset for non-ISA conditions [K].
        windnorth (ndarray): Wind north component at aircraft position [m/s].
        windeast (ndarray): Wind east component at aircraft position [m/s].
        selspd (ndarray): Selected speed: CAS [m/s] or Mach [-].
        selalt (ndarray): Selected altitude [m].
        selvs (ndarray): Selected vertical speed [m/s].
        swlnav (ndarray): Bool switch: LNAV (lateral FMS guidance) on/off.
        swvnav (ndarray): Bool switch: VNAV (vertical FMS guidance) on/off.
        swvnavspd (ndarray): Bool switch: VNAV speed guidance on/off.
        swhdgsel (ndarray): Bool switch: True while aircraft is turning.
        swats (ndarray): Bool switch: autothrottle on/off.
        thr (ndarray): Throttle setting [0.0-1.0]; negative = invalid/auto.
        work (ndarray): Work done by the engines during the flight [J].
        translvl (float): Transition level [m].
        bphase (ndarray): Default bank angles per flight phase [rad].
        crecmdlist (list): Command lines issued for each new aircraft.
        cond (Condition): Pending conditional (ATALT/ATSPD/ATDIST) commands.
        wind (Wind): Wind-field model.
        turbulence (Turbulence): Turbulence model.
        ap (Autopilot): Autopilot/FMS guidance.
        actwp (ActiveWaypoint): Active waypoint data per aircraft.
        aporasas (APorASAS): Selection between autopilot and ASAS commands.
        cd (ConflictDetection): Conflict detection.
        cr (ConflictResolution): Conflict resolution.
        perf (OpenAP): Aircraft performance model.
        trails (Trails): Radar-display trails.
        groups (TrafficGroups): Aircraft group administration.

    Created by: Jacco M. Hoekstra
    """

    def __init__(self) -> None:
        super().__init__()

        # Traffic is the toplevel trafficarrays object
        self.setroot(self)

        self.ntraf = 0

        self.cond = Condition()  # Conditional commands list
        self.wind = Wind()
        self.turbulence = Turbulence()
        self.translvl = 5000.0 * ft  # [m] Default transition level

        # Default commands issued for an aircraft after creation
        self.crecmdlist = []

        with self.settrafarrays():
            # Aircraft Info
            self.callsign = []  # identifier (string)
            self.typecode = []  # aircaft type (string)

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
            self.perf = OpenAP()

            # Group Logic
            self.groups = TrafficGroups()

            # Traffic autopilot data
            self.swhdgsel = np.array([], dtype=bool)  # determines whether aircraft is turning

            # Traffic autothrottle settings
            self.swats = np.array(
                [], dtype=bool
            )  # Switch indicating whether autothrottle system is on/off
            self.thr = np.array([])  # Thottle seeting (0.0-1.0), negative = non-valid/auto

            # Display information on label
            self.label = []  # Text and bitmap of traffic label

            # Miscallaneous
            self.coslat = np.array([])  # Cosine of latitude for computations
            self.eps = np.array([])  # Small nonzero numbers
            self.work = np.array([])  # Work done throughout the flight

        # Default bank angles per flight phase
        self.bphase = np.deg2rad(np.array([15, 35, 35, 35, 15, 45]))

    def reset(self) -> None:
        """Clear all traffic data upon simulation reset.

        Empties all per-aircraft arrays (including those of child entities),
        resets the performance, wind and turbulence models, switches off
        trajectory noise and restores the default transition level.
        """
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
        callsign: str,
        actype: str = "A320",
        lat: float = 53.0,
        lon: float = 4.0,
        hdg: float = 45.0,
        alt: float = 25000,
        spd: float = 300,
    ) -> tuple[bool, str]:
        """Create a single aircraft and add it to the traffic database.

        Implements the CRE stack command. After creation, any commands stored
        via CRECMD are stacked for the new aircraft.

        Args:
            callsign: Aircraft identifier; converted to upper case, must be
                unique within the simulation.
            actype: ICAO aircraft type designator (default "A320").
            lat: Initial latitude [deg].
            lon: Initial longitude [deg].
            hdg: Initial heading [deg].
            alt: Initial altitude [m] (stack input is given in ft/FL).
            spd: Initial speed: CAS [m/s] or Mach [-] (stack input in kts).

        Returns:
            tuple: (success flag, confirmation or error message).
        """

        if callsign in self.callsign:
            return False, f"aircraft {callsign} already exists"

        # covert to array with 1 element
        acid_ = np.array([callsign.upper()])
        actype_ = np.array([actype])
        lat_ = np.array([lat])
        lon_ = np.array([lon])
        alt_ = np.array([alt])
        hdg_ = np.array([hdg])
        spd_ = np.array([spd])

        self.__create_aircraft(acid_, actype_, lat_, lon_, hdg_, alt_, spd_)

        return True, f"arcraft {callsign} created"

    def mcre(
        self,
        n: int,
        lat_min: float = 53.0,
        lon_min: float = 0.0,
        lat_max: float = 60.0,
        lon_max: float = 10.0,
        actype: str = "A320",
        acalt: int | None = None,
        acspd: int | None = None,
    ) -> tuple[bool, str]:
        """Create multiple aircraft at random positions in a lat/lon box.

        Implements the MCRE stack command. Callsigns are generated randomly
        (two letters plus a sequence number). Heading is drawn uniformly from
        1-360 deg; when not given, altitude is drawn from 2000-39000 ft and
        speed from 250-450 kts. The default area is the North Sea region.

        Args:
            n: Number of aircraft to create.
            lat_min: Southern boundary of the creation area [deg].
            lon_min: Western boundary of the creation area [deg].
            lat_max: Northern boundary of the creation area [deg].
            lon_max: Eastern boundary of the creation area [deg].
            actype: ICAO aircraft type designator for all aircraft.
            acalt: Optional fixed altitude [m]; random when None.
            acspd: Optional fixed speed, CAS [m/s] or Mach; random when None.

        Returns:
            tuple: (True, confirmation message).
        """

        # Generate random callsigns
        idtmp = chr(randint(65, 90)) + chr(randint(65, 90)) + "{:>03}"
        callsign = [idtmp.format(i) for i in range(n)]

        actype_ = np.array([actype] * n)

        # Generate random positions
        aclat = np.random.rand(n) * (lat_max - lat_min) + lat_min
        aclon = np.random.rand(n) * (lon_max - lon_min) + lon_min
        achdg = np.random.randint(1, 360, n)
        acalt_ = np.full(n, acalt) if acalt is not None else np.random.randint(2000, 39000, n) * ft
        acspd_ = np.full(n, acspd) if acspd is not None else np.random.randint(250, 450, n) * kts

        self.__create_aircraft(
            np.array(callsign), actype_, aclat, aclon, achdg, acalt_, acspd_
        )

        return True, f"{n} aircraft created"

    def __create_aircraft(
        self,
        acid: np.ndarray,
        actype: np.ndarray,
        lat: np.ndarray,
        lon: np.ndarray,
        hdg: np.ndarray,
        alt: np.ndarray,
        spd: np.ndarray,
    ) -> None:
        """Append one or more aircraft to all traffic arrays.

        Common backend for cre() and mcre(): resizes all (child) traffic
        arrays, initializes position, heading, speeds, atmosphere and wind
        for the new aircraft, and stacks any CRECMD default commands.
        All array arguments must have the same length; alt is in [m],
        spd is CAS [m/s] or Mach [-].
        """

        n = len(acid)

        # Adjust the size of all traffic arrays
        super().create(n)
        self.ntraf += n

        # Limit longitude to [-180.0, 180.0]
        lon[lon > 180.0] -= 360.0
        lon[lon < -180.0] += 360.0

        # Aircraft Info
        self.callsign[-n:] = acid
        self.typecode[-n:] = actype

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
                minisky.stack.stack(self.callsign[j] + " " + cmdtxt)

    def creconfs(
        self,
        callsign: str,
        actype: str,
        targetidx: int,
        dpsi: float,
        dcpa: float,
        tlosh: float,
        dH: float | None = None,
        tlosv: float | None = None,
        spd: float | None = None,
    ) -> None:
        """Create an aircraft in conflict with a target aircraft.

        Implements the CRECONFS stack command. The intruder position, track
        and speed are computed such that, relative to the target aircraft,
        separation is lost after the given time with the given distance at
        the closest point of approach. The protected-zone radius and height
        from the settings (asas_pzr, asas_pzh) are taken into account.

        Args:
            callsign: Callsign of the new (intruder) aircraft.
            actype: ICAO aircraft type designator of the new aircraft.
            targetidx: Index of the target (ownship) aircraft.
            dpsi: Conflict angle between ownship and intruder tracks [deg].
            dcpa: Predicted distance at closest point of approach [nm].
            tlosh: Horizontal time to loss of separation [s]
                (stack input as (hh:mm:)sec).
            dH: Optional vertical offset of the intruder [m]
                (stack input in ft); level conflict when None.
            tlosv: Optional vertical time to loss of separation [s];
                defaults to tlosh.
            spd: Optional speed of the new aircraft, CAS [m/s] or Mach [-]
                (stack input in kts/-); ownship ground speed when omitted.
        """
        latref = self.lat[targetidx]  # deg
        lonref = self.lon[targetidx]  # deg
        altref = self.alt[targetidx]  # m
        trkref = np.radians(self.trk[targetidx])
        gsref = self.gs[targetidx]  # m/s
        tasref = self.tas[targetidx]  # m/s
        vsref = self.vs[targetidx]  # m/s
        cpa = dcpa * nm
        pzr = minisky.core.settings.asas_pzr * nm
        pzh = minisky.core.settings.asas_pzh * ft
        trk = trkref + np.radians(dpsi)

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
            tasn, tase = tas * np.cos(trk), tas * np.sin(trk)
            wn, we = self.wind.getdata(latref, lonref, acalt)
            gsn, gse = tasn + wn, tase + we
        else:
            # Groundspeed is the same as ownship
            gsn, gse = gsref * np.cos(trk), gsref * np.sin(trk)

        # Horizontal relative velocity vector
        vreln, vrele = gsref * np.cos(trkref) - gsn, gsref * np.sin(trkref) - gse
        # Relative velocity magnitude
        vrel = np.sqrt(vreln * vreln + vrele * vrele)
        # Relative travel distance to closest point of approach
        drelcpa = tlosh * vrel + (0 if cpa > pzr else np.sqrt(pzr * pzr - cpa * cpa))
        # Initial intruder distance
        dist = np.sqrt(drelcpa * drelcpa + cpa * cpa)
        # Rotation matrix diagonal and cross elements for distance vector
        rd = drelcpa / dist
        rx = cpa / dist
        # Rotate relative velocity vector to obtain intruder bearing
        brn = np.degrees(np.atan2(-rx * vreln + rd * vrele, rd * vreln + rx * vrele))

        # Calculate intruder lat/lon
        aclat, aclon = geo.kwikpos(latref, lonref, brn, dist / nm)
        # convert groundspeed to CAS, and track to heading using actual
        # intruder position
        wn, we = self.wind.getdata(aclat, aclon, acalt)
        tasn, tase = gsn - wn, gse - we
        acspd = tas2cas(np.sqrt(tasn * tasn + tase * tase), acalt)
        achdg = np.degrees(np.atan2(tase, tasn))

        # Create and, when necessary, set vertical speed
        self.cre(
            callsign, actype, float(aclat), float(aclon), float(achdg), acalt, float(acspd)
        )
        self.ap.selaltcmd(len(self.lat) - 1, altref, acvs)
        self.vs[-1] = acvs

    def delete(self, idx: int | np.ndarray) -> bool:
        """Delete one or more aircraft from the traffic database.

        Removes the corresponding entries from all (child) traffic arrays
        and updates the aircraft count. Used by the DEL stack command.

        Args:
            idx: Aircraft index, or a collection of indices.

        Returns:
            bool: True (deletion always succeeds for valid indices).
        """
        # If this is a multiple delete, sort first for list delete
        # (which will use list in reverse order to avoid index confusion)
        if isinstance(idx, Collection):
            idx = np.sort(idx)

        # Call the actual delete function
        super().delete(idx)

        # Update number of aircraft
        self.ntraf = len(self.lat)
        return True

    def update(self) -> None:
        """Perform one simulation time step for all aircraft.

        Called every step by the simulation loop. In order: updates the
        atmosphere, surveillance noise, autopilot and airborne separation
        assurance (ASAS) guidance, decides per channel between autopilot and
        ASAS commands, updates the performance model and limits the commanded
        speeds accordingly, integrates airspeed/heading/vertical speed,
        ground speed and position, applies turbulence, triggers conditional
        commands and updates the display trails. Does nothing when there is
        no traffic.
        """
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

    def update_asas(self) -> None:
        """Run conflict detection and conflict resolution for all aircraft."""
        # Conflict detection and resolution
        self.cd.update(self, self)
        self.cr.update(self.cd, self, self)

    def update_airspeed(self) -> None:
        """Integrate true airspeed, heading and vertical speed over one step.

        Accelerates or decelerates towards the commanded TAS using the
        performance-limited longitudinal acceleration, turns towards the
        commanded heading with a turn rate that follows from the bank angle
        (commanded turn bank or default bank limit), and updates the vertical
        speed for the altitude select/capture/hold autopilot logic. Also
        refreshes the derived CAS and Mach values.
        """
        # Compute horizontal acceleration
        delta_spd = self.aporasas.tas - self.tas
        need_ax = np.abs(delta_spd) > np.abs(minisky.sim.simdt * self.perf.axmax)
        self.ax = need_ax * np.sign(delta_spd) * self.perf.axmax
        # Update velocities
        self.tas = np.where(need_ax, self.tas + self.ax * minisky.sim.simdt, self.aporasas.tas)
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
        self.az = need_az * np.sign(delta_vs) * (300 * fpm)  # fixed vertical acc approx 1.6 m/s^2
        self.vs = np.where(need_az, self.vs + self.az * minisky.sim.simdt, target_vs)
        self.vs = np.where(np.isfinite(self.vs), self.vs, 0)  # fix vs nan issue

    def update_groundspeed(self) -> None:
        """Compute ground speed and track from heading, airspeed and wind.

        Without wind, ground speed equals TAS and track equals heading. With
        a wind field defined, the wind vector at each aircraft position is
        added to the airspeed vector (only when airborne, above 50 ft). Also
        accumulates the work done by the engines [J] along the flown path.
        """
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
            self.gsnorth = self.tas * np.cos(np.radians(self.hdg)) + self.windnorth * applywind
            self.gseast = self.tas * np.sin(np.radians(self.hdg)) + self.windeast * applywind

            self.gs = np.logical_not(applywind) * self.tas + applywind * np.sqrt(
                self.gsnorth**2 + self.gseast**2
            )

            self.trk = (
                np.logical_not(applywind) * self.hdg
                + applywind * np.degrees(np.arctan2(self.gseast, self.gsnorth)) % 360.0
            )

        self.work += (
            self.perf.thrust * minisky.sim.simdt * np.sqrt(self.gs * self.gs + self.vs * self.vs)
        )

    def update_pos(self) -> None:
        """Integrate altitude and lat/lon position over one time step.

        Altitude follows the vertical speed while the altitude-select mode is
        engaged, and snaps to the commanded altitude otherwise. Latitude and
        longitude are advanced with the ground speed components using a
        spherical-Earth approximation, and the flown distance is accumulated.
        """
        # Update position
        self.alt = np.where(
            self.swaltsel,
            np.round(self.alt + self.vs * minisky.sim.simdt, 6),
            self.aporasas.alt,
        )
        self.lat = self.lat + np.degrees(minisky.sim.simdt * self.gsnorth / Rearth)
        self.coslat = np.cos(np.deg2rad(self.lat))
        self.lon = self.lon + np.degrees(minisky.sim.simdt * self.gseast / self.coslat / Rearth)
        self.distflown += self.gs * minisky.sim.simdt

    @overload
    def idx(self, callsign: str) -> int: ...
    @overload
    def idx(self, callsign: Iterable[str]) -> list: ...
    def idx(self, callsign: str | Iterable[str]) -> int | list:
        """Find the traffic-array index for one or more callsigns.

        Args:
            callsign: A single callsign string, or an iterable of callsigns.
                The special values "*" and "#" refer to the most recently
                created aircraft.

        Returns:
            int or list: Index of the aircraft (or list of indices when an
            iterable was given); -1 for callsigns that are not found.
        """
        if not isinstance(callsign, str):
            # for multiple callsigns
            # Fast way of finding indices of all ACID's in a given list
            tmp = {v: i for i, v in enumerate(self.callsign)}
            return [tmp.get(acidi, -1) for acidi in callsign]
        else:
            # Catch last created id (* or # symbol)
            if callsign in ("#", "*"):
                return self.ntraf - 1

            try:
                return self.callsign.index(callsign.upper())
            except ValueError:
                return -1

    def setnoise(self, noise: bool | None = None) -> bool | tuple[bool, str]:
        """Switch trajectory noise models on or off, or report their state.

        Implements the NOISE stack command. Controls both the turbulence
        model and the surveillance (ADS-B transmission/truncation) noise.

        Args:
            noise: True/False to enable/disable noise; None to report the
                current state.

        Returns:
            bool or tuple: True on set, or (True, status message) on query.
        """
        if noise is None:
            return True, "Noise is currently " + ("on" if self.turbulence.active else "off")

        self.turbulence.setnoise(noise)
        self.noise.setnoise(noise)
        return True

    def engchange(self, acid: int, engid: str) -> None:
        """Change the engine type of an aircraft in the performance model.

        Args:
            acid: Aircraft index.
            engid: New engine type identifier.
        """
        self.perf.engchange(acid, engid)  # type: ignore[attr-defined]
        return

    def move(
        self,
        idx: int,
        lat: float,
        lon: float,
        alt: float | None = None,
        hdg: float | None = None,
        casmach: float | None = None,
        vspd: float | None = None,
    ) -> None:
        """Instantaneously move an aircraft to a new position/state.

        Implements the MOVE stack command. Optional state values are left
        unchanged when omitted. Setting a vertical speed disengages VNAV.

        Args:
            idx: Aircraft index.
            lat: New latitude [deg].
            lon: New longitude [deg].
            alt: Optional new altitude [m]; also sets the selected altitude.
            hdg: Optional new heading [deg]; also sets the autopilot track.
            casmach: Optional new speed, CAS [m/s] or Mach [-].
            vspd: Optional new vertical speed [m/s].
        """
        self.lat[idx] = lat
        self.lon[idx] = lon

        if alt is not None:
            self.alt[idx] = alt
            self.selalt[idx] = alt

        if hdg is not None:
            self.hdg[idx] = hdg
            self.ap.trk[idx] = hdg

        if casmach is not None:
            h = alt if alt is not None else float(self.alt[idx])
            self.tas[idx], self.selspd[idx], _ = casormach(casmach, h)

        if vspd is not None:
            self.vs[idx] = vspd
            self.swvnav[idx] = False

    def position(self, id_or_name: int | str) -> tuple[bool, str]:
        """Show information on an aircraft, airport, waypoint or navaid.

        Implements the POS stack command. Dispatches to
        :meth:`position_aircraft` when an aircraft index is given, and to
        :meth:`position_by_name` for a name lookup.

        Args:
            id_or_name: Aircraft index (int) or the name of an aircraft,
                airport, waypoint, navaid or airway (str).

        Returns:
            tuple: (success flag, multi-line information text).
        """

        if isinstance(id_or_name, int):
            return self.position_aircraft(id_or_name)
        else:
            return self.position_by_name(id_or_name)

    def position_aircraft(self, idx: int) -> tuple[bool, str]:
        """Generate a position report for a single aircraft.

        The report includes position, heading/track [deg], altitude [ft],
        vertical speed [fpm], CAS/TAS/GS [kts], Mach, active FMS modes
        (LNAV/VNAV) with the active waypoint, and origin/destination.

        Args:
            idx: Aircraft index.

        Returns:
            tuple: (True, multi-line position report).
        """

        acid = self.callsign[idx]

        actype = self.typecode[idx]
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
        if self.swlnav[idx] and len(route.wpname) > 0 and route.iactwp >= 0:
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

    def position_by_name(self, name: str) -> tuple[bool, str]:
        """Look up a name and generate an information report for it.

        Searches, in order: airports, aircraft callsigns, waypoints/navaids,
        and airways in the navigation database. Airport reports include
        position, elevation [ft] and runways; navaid reports include type,
        frequency and airway connections.

        Args:
            name: Name/identifier to look up (case-insensitive).

        Returns:
            tuple: (success flag, multi-line information text).
        """
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
            idx_cc = minisky.navdb.cocode2.index(minisky.navdb.aptco[idx_airport].upper())
            country_name = minisky.navdb.coname[idx_cc].upper()
            country_code = minisky.navdb.aptco[idx_airport]

            lines += (
                f"{aptname} is a {airport_size} airport in {country_name} ({country_code}):\n"
                f"Position: {latlon2txt(aptlat, aptlon)}\n"
                f"Elevation: {int(round(aptelev / ft))} ft \n"
            )

            if minisky.navdb.aptid[idx_airport] in minisky.navdb.rwythresholds:
                runways = minisky.navdb.rwythresholds[minisky.navdb.aptid[idx_airport]].keys()
                if runways:
                    lines += f"Runways: {', '.join(runways)}\n"

            return True, lines

        # try aircraft
        idx_ac = self.idx(name)
        if idx_ac >= 0:
            return self.position_aircraft(idx_ac)

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
                    if minisky.navdb.wptype[i] in ["VOR", "DME", "TACAN"] and not samedesc:
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
                    lines += f"Attention: {n_other} other waypoint(s) also has name {name}\n"

                # In which airways?
                connect = minisky.navdb.listconnections(
                    name, minisky.navdb.wplat[iwp], minisky.navdb.wplon[iwp]
                )
                if len(connect) > 0:
                    awset = set()
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
        return False, f"{name} not found as aircraft, airport, navaid, or waypoint"

        # Show what we found on airport and navaid/waypoint

    def settrans(self, alt: float = -999.0) -> bool | tuple[bool, str]:
        """Set or show the transition level.

        Args:
            alt: New transition level [m] (stack input in ft/FL). With the
                default sentinel value the current level is reported instead.

        Returns:
            bool or tuple: True on set, (True, message) on query, or
            (False, error message) for invalid values.
        """
        # in case a valid value is ginve set it
        if alt > -900.0:
            if alt > 0.0:
                self.translvl = alt
                return True
            return False, "Transition level needs to be ft/FL and larger than zero"

        # In case no value is given, show it
        tlvl = int(round(self.translvl / ft))
        return True, f"Transition level = {tlvl}/FL{int(round(tlvl / 100.0))}"

    def setbanklim(self, idx: int, bankangle: float | None = None) -> bool | tuple[bool, str]:
        """Set or show the bank angle limit for a given aircraft.

        Implements the BANK stack command. The limit is used by the autopilot
        to compute turn rates when no explicit turn is specified.

        Args:
            idx: Aircraft index.
            bankangle: New bank limit [deg]; when omitted, the current limit
                is reported.

        Returns:
            bool or tuple: True on set, or (True, status message) on query.
        """
        if bankangle:
            self.ap.bankdef[idx] = np.radians(bankangle)  # [rad]
            return True
        return (
            True,
            f"Banklimit of {self.callsign[idx]} is {int(np.degrees(self.ap.bankdef[idx]))} deg",
        )

    def setthrottle(self, idx: int, throttle: str = "") -> bool | tuple[bool, str]:
        """Set the throttle of an aircraft, or report the autothrottle state.

        Implements the THR stack command. "AUTO"/"OFF" re-engages the
        autothrottle, "IDLE" sets zero thrust, and a numeric value (0.0-1.0,
        optionally as a percentage like "80%") sets a fixed throttle and
        disables the autothrottle.

        Args:
            idx: Aircraft index.
            throttle: Throttle argument string; empty to query the state.

        Returns:
            bool or tuple: True on set, (True, status message) on query, or
            (False, error message) for invalid input.
        """

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
                except ValueError:
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
            return True, "ATS of " + self.callsign[idx] + " is ON"
        return True, "ATS of " + self.callsign[idx] + " is OFF. THR is " + str(self.thr[idx])

    def crecmd(self, cmdline: str) -> tuple[bool, str]:
        """Add a command to the list issued for every newly created aircraft.

        Implements the CRECMD stack command. Each stored command line is
        stacked as "<acid> <cmdline>" for every aircraft created afterwards.
        With an empty argument or "?", the current list is shown instead.

        Args:
            cmdline: Command line (without callsign) to add to the list, or
                ""/"?" to show the current list.

        Returns:
            tuple: (True, message).
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

    def clrcrecmd(self) -> tuple[bool, str]:
        """Clear the list of commands issued for newly created aircraft.

        Implements the CLRCRECMD stack command, removing all command lines
        previously added with CRECMD.

        Returns:
            tuple: (True, message).
        """
        ncrecmd = len(self.crecmdlist)
        if ncrecmd == 0:
            return True, "CLRCRECMD deletes all commands on clears command"
        else:
            self.crecmdlist = []
            return True, f"All {ncrecmd} crecmd commands deleted."
