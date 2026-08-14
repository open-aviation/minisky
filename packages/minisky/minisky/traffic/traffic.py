"""BlueSky traffic implementation.

Defines the [`Traffic`][minisky.traffic.traffic.Traffic] class, the top-level
traffic database of the simulator. It holds all per-aircraft state (position, attitude, speeds,
atmosphere, autopilot selections) as numpy arrays, owns the sub-models
(autopilot, performance, conflict detection/resolution, wind, turbulence,
trails, groups), and performs the numerical integration of the aircraft
states each simulation time step.

A single instance is created at simulator start-up and made available as
[`runtime.traffic`][minisky.traffic.traffic.Traffic]. Several methods double as stack-command implementations
(CRE, MCRE, CRECONFS, MOVE, POS, BANK, THR, NOISE, CRECMD, ...).
"""

from __future__ import annotations

from collections.abc import Callable, Collection, Iterable
from random import Random
from typing import TYPE_CHECKING, Annotated, Literal, overload

import numpy as np
from annotated_types import Ge, Le, Lt

from minisky import quantities as q
from minisky.command import (
    AcId,
    AcIdSelection,
    CmdParser,
    FiniteFloat,
    HeadingDeg,
    Keyword,
    LatLonDeg,
    OnOff,
    PositiveFiniteFloat,
    ResolvedPositionArg,
    RunwayHeadingRequest,
    RunwayPosition,
    SpeedMpsOrMach,
    Text,
    TimeS,
    UseRunwayHeading,
    VerticalDistanceM,
    VspdMps,
    command,
)
from minisky.core.config import MiniSkyConfig
from minisky.core.trafficarrays import TrafficArrays
from minisky.result import Err, Ok, Result
from minisky.tools import geo
from minisky.tools.aero import (
    DEFAULT_CASMACH_THRESHOLD,
    casormach,
    casormach2tas,
    tas2cas,
    vatmos,
    vcasormach,
)
from minisky.tools.convert import latlon2txt
from minisky.tools.shapes import Shapes
from minisky.traffic.asas import ConflictDetection, ConflictResolution
from minisky.values import MagneticHeadingDeg, StdPressureAltM

from .activewpdata import ActiveWaypoint
from .aporasas import APorASAS
from .autopilot import Autopilot
from .conditional import Condition
from .kinematics import Kinematics
from .performance.perfoap import OpenAP
from .trafficgroups import TrafficGroups
from .trails import Trails
from .turbulence import Turbulence
from .uncertainty import SurveillanceUncertainty
from .wind import Wind

if TYPE_CHECKING:
    from minisky.simulation import ConsoleIO, Simulation
    from minisky.tools.navdata import Navdatabase


def _parse_throttle(value: str) -> float:
    factor = 0.01 if value.endswith("%") else 1.0
    number = value.removesuffix("%")
    if "%" in number:
        raise ValueError
    return factor * float(number)


Throttle = Annotated[
    float, CmdParser.value(_parse_throttle, "a throttle fraction or percentage"), Ge(0), Le(1)
]

LatitudeArg = Annotated[q.LatitudeDeg[float], Ge(-90), Le(90)]
LongitudeArg = Annotated[q.LongitudeDeg[float], Ge(-180), Le(180)]
ConflictAngleDeg = q.AngleDeg[FiniteFloat]
ConflictDistanceNM = q.DistanceNM[FiniteFloat]
BankLimitDeg = Annotated[q.BankAngleDeg[PositiveFiniteFloat], Lt(90)]


_DEFAULT_ALTITUDE = StdPressureAltM(q.ft_to_m(25000.0))
_DEFAULT_SPEED = q.kt_to_mps(300.0)


class Traffic(TrafficArrays):
    """Central traffic database holding the state of all simulated aircraft.

    Traffic is the top-level
    [`TrafficArrays`][minisky.core.trafficarrays.TrafficArrays] object: all per-aircraft
    arrays registered by its child entities (autopilot, active waypoint data,
    performance model, conflict detection/resolution, etc.) grow and shrink
    together when aircraft are created or deleted. A single instance is
    available as [`runtime.traffic`][minisky.traffic.traffic.Traffic].

    Every simulation step,
    [`Traffic.update`][minisky.traffic.traffic.Traffic.update] refreshes the atmosphere,
    runs the
    autopilot and separation-assurance logic, applies performance limits, and
    numerically integrates airspeed, heading, vertical speed and position of
    all aircraft. All internal state is kept in SI units; stack commands use
    aviation units (ft, kts, FL) and are converted on input/output.

    Attributes:
        ntraf (int): Number of aircraft currently in the simulation.
        casmach_threshold: Upper bound below which positive speed values are
            interpreted as Mach numbers.
        callsign (list): Aircraft identifier (callsign) strings.
        typecode (list): ICAO aircraft type designators (e.g. "A320").
        gsnorth (ndarray): North component of ground speed [m/s].
        gseast (ndarray): East component of ground speed [m/s].
        windnorth (ndarray): Wind north component at aircraft position [m/s].
        windeast (ndarray): Wind east component at aircraft position [m/s].
        selspd (ndarray): Selected speed: CAS [m/s] or Mach [-].
        swlnav (ndarray): Bool switch: LNAV (lateral FMS guidance) on/off.
        swvnav (ndarray): Bool switch: VNAV (vertical FMS guidance) on/off.
        swvnavspd (ndarray): Bool switch: VNAV speed guidance on/off.
        swats (ndarray): Bool switch: autothrottle on/off.
        thr (ndarray): Fixed throttle setting [0.0-1.0], used when autothrottle is off.
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
        kinematics (Kinematics): Flight-state integration (airspeed, heading,
            vertical speed, ground speed and position).
        trails (Trails): Radar-display trails.
        groups (TrafficGroups): Aircraft group administration.

    Created by: Jacco M. Hoekstra
    """

    translvl: q.PressureAltitudeM[float]
    lat: q.LatitudeDeg[np.ndarray]
    lon: q.LongitudeDeg[np.ndarray]
    distflown: q.DistanceM[np.ndarray]
    alt: q.PressureAltitudeM[np.ndarray]
    hdg: q.TrueHeadingDegrees[np.ndarray]
    trk: q.GroundTrackDeg[np.ndarray]
    tas: q.TrueAirspeedMps[np.ndarray]
    gs: q.GroundSpeedMps[np.ndarray]
    gsnorth: q.GroundSpeedMps[np.ndarray]
    gseast: q.GroundSpeedMps[np.ndarray]
    cas: q.CalibratedAirspeedMps[np.ndarray]
    M: q.MachNumber[np.ndarray]
    vs: q.VerticalRateMps[np.ndarray]
    p: q.StaticPressurePa[np.ndarray]
    rho: q.DensityKgPerM3[np.ndarray]
    Temp: q.StaticTemperatureK[np.ndarray]
    windnorth: q.WindSpeedMps[np.ndarray]
    windeast: q.WindSpeedMps[np.ndarray]
    aptas: q.TrueAirspeedMps[np.ndarray]
    selalt: q.PressureAltitudeM[np.ndarray]
    selvs: q.VerticalRateMps[np.ndarray]
    work: q.EnergyJ[np.ndarray]

    def __init__(
        self,
        config: MiniSkyConfig,
        python_random: Random,
        numpy_random: np.random.RandomState,
        shapes: Shapes,
        navigation: Navdatabase,
        console: ConsoleIO,
        get_simulation: Callable[[], Simulation],
        stack_command: Callable[..., None],
        select_implementation: Callable[[str, str], Result[str, str]],
    ) -> None:
        super().__init__()
        self.config = config
        self.python_random = python_random
        self.numpy_random = numpy_random
        self.shapes = shapes
        self.navigation = navigation
        self.console = console
        self._get_simulation = get_simulation
        self.stack_command = stack_command
        self.select_implementation = select_implementation

        self.ntraf = 0
        self.casmach_threshold = DEFAULT_CASMACH_THRESHOLD

        self.cond = Condition(self, stack_command)  # Conditional commands list
        self.wind = Wind()
        self.wind.reparent(self)
        self.turbulence = Turbulence(self, get_simulation)
        self.translvl = q.ft_to_m(5000.0)

        # Default commands issued for an aircraft after creation
        self.crecmdlist = []

        with self.settrafarrays():
            # Aircraft Info
            self.callsign: list[str] = []  # identifier (string)
            self.typecode: list[str] = []  # aircaft type (string)

            # Positions
            self.lat = np.array([])
            self.lon = np.array([])
            self.distflown = np.array([])
            self.alt = np.array([])
            self.hdg = np.array([])
            self.trk = np.array([])

            # Velocities
            self.tas = np.array([])
            self.gs = np.array([])
            self.gsnorth = np.array([])  # ground speed [m/s]
            self.gseast = np.array([])  # ground speed [m/s]
            self.cas = np.array([])
            self.M = np.array([])
            self.vs = np.array([])

            # Atmosphere
            self.p = np.array([])
            self.rho = np.array([])
            self.Temp = np.array([])

            # Wind speeds
            self.windnorth = np.array([])  # wind speed north component a/c pos [m/s]
            self.windeast = np.array([])  # wind speed east component a/c pos [m/s]

            # Traffic autopilot settings
            # TODO(abraham): #40 must split selected CAS and Mach before selspd can carry isqx metadata.
            self.selspd = np.array([])  # selected CAS or Mach
            self.aptas = np.array([])  # just for initializing
            self.selalt = np.array([])
            self.selvs = np.array([])

            # Whether to perform LNAV and VNAV
            self.swlnav = np.array([], dtype=bool)
            self.swvnav = np.array([], dtype=bool)
            self.swvnavspd = np.array([], dtype=bool)

            # Flight Models
            self.cd = ConflictDetection(config, self, stack_command)
            self.cr = ConflictResolution(config, self, select_implementation)
            self.ap = Autopilot(self, get_simulation)
            self.aporasas = APorASAS(self)
            self.noise = SurveillanceUncertainty(self, get_simulation)
            self.trails = Trails(self, get_simulation)
            self.actwp = ActiveWaypoint(self)
            self.perf = OpenAP(self)
            self.kinematics = Kinematics(self, get_simulation)

            # Group Logic
            self.groups = TrafficGroups(self, shapes)

            # Traffic autothrottle settings
            self.swats = np.array(
                [], dtype=bool
            )  # Switch indicating whether autothrottle system is on/off
            self.thr = np.array([])  # Fixed throttle setting (0.0-1.0) when autothrottle is off

            # Display information on label
            self.label = []  # Text and bitmap of traffic label

            # Miscallaneous
            self.coslat = np.array([])  # Cosine of latitude for computations
            self.eps = np.array([])  # Small nonzero numbers
            self.work = np.array([])

    @command(name="CASMACHTHR")
    def casmachthr(self, threshold: float | None = None) -> Result[str, str]:
        """Get or set this runtime's CAS/Mach interpretation threshold.

        Positive speed values below this threshold are interpreted as Mach
        numbers by CRE, MOVE, route, and autopilot speed conversions.
        """
        if threshold is None:
            return Ok(
                "CASMACHTHR: The current CAS/Mach threshold is "
                f"{self.casmach_threshold} m/s "
                f"({q.mps_to_kt(self.casmach_threshold)} kts)"
            )

        self.casmach_threshold = threshold
        return Ok(f"CASMACHTHR: Set CAS/Mach threshold to {threshold}")

    @property
    def simulation(self) -> Simulation:
        """Return the simulation that owns this traffic object."""
        return self._get_simulation()

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
        self.cond.reset()

        # Build new modules for turbulence
        self.turbulence.reset()

        # Trajectory noise (turbulence, navigation uncertainties)
        self.configure_noise(False)

        # Reset transition level to default value
        self.translvl = q.ft_to_m(5000.0)

    @command(name="CRE", aliases=("CREATE",))
    def command_cre(
        self,
        callsign: Keyword,
        actype: Keyword,
        position: ResolvedPositionArg,
        hdg: HeadingDeg | UseRunwayHeading | None = None,
        alt: StdPressureAltM = _DEFAULT_ALTITUDE,
        spd: SpeedMpsOrMach = _DEFAULT_SPEED,
    ) -> Result[str, str]:
        """Create an aircraft."""
        if isinstance(position, RunwayPosition):
            coordinates = position.coordinates
            default_heading = position.runway_heading
        else:
            coordinates = position
            default_heading = 45.0

        if isinstance(hdg, RunwayHeadingRequest):
            if not isinstance(position, RunwayPosition):
                return Err("CRE: heading * requires a runway position")
            heading = position.runway_heading
        elif hdg is None:
            heading = default_heading
        elif isinstance(hdg, MagneticHeadingDeg):
            heading = (hdg.degrees + geo.magdec(coordinates.lat, coordinates.lon)) % 360.0
        else:
            heading = hdg.degrees

        return self.cre(
            callsign,
            actype,
            coordinates.lat,
            coordinates.lon,
            heading,
            alt,
            spd,
        )

    def cre(
        self,
        callsign: Keyword,
        actype: Keyword = "A320",
        lat: q.LatitudeDeg[float] = 53.0,
        lon: q.LongitudeDeg[float] = 4.0,
        hdg: q.TrueHeadingDegrees[float] = 45.0,
        alt: StdPressureAltM = _DEFAULT_ALTITUDE,
        spd: SpeedMpsOrMach = _DEFAULT_SPEED,
    ) -> Result[str, str]:
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
            alt: Initial altitude [m] (stack input is given in ft/FL);
                defaults to 25000 ft.
            spd: Initial speed: CAS [m/s] or Mach [-] (stack input in kts);
                defaults to 300 kts.
        """

        name_error = self._aircraft_name_collision(callsign)
        if name_error is not None:
            return Err(name_error)

        # covert to array with 1 element
        acid_ = np.array([callsign.upper()])
        actype_ = np.array([actype])
        lat_ = np.array([lat])
        lon_ = np.array([lon])
        alt_ = np.array([alt.value])
        hdg_ = np.array([hdg])
        spd_ = np.array([spd])

        self.__create_aircraft(acid_, actype_, lat_, lon_, hdg_, alt_, spd_)

        return Ok(f"Aircraft {callsign} created")

    @command(name="MCRE")
    def mcre(
        self,
        n: int,
        lat_min: LatitudeArg = 53.0,
        lon_min: LongitudeArg = 0.0,
        lat_max: LatitudeArg = 60.0,
        lon_max: LongitudeArg = 10.0,
        actype: Keyword = "A320",
        acalt: StdPressureAltM | None = None,
        acspd: SpeedMpsOrMach | None = None,
    ) -> Result[str, str]:
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
        """

        # Generate random callsigns
        idtmp = (
            chr(self.python_random.randint(65, 90))
            + chr(self.python_random.randint(65, 90))
            + "{:>03}"
        )
        callsign = [idtmp.format(i) for i in range(n)]
        for name in callsign:
            name_collision = self._aircraft_name_collision(name)
            if name_collision is not None:
                return Err(name_collision)

        actype_ = np.array([actype] * n)

        # Generate random positions
        aclat = self.numpy_random.rand(n) * (lat_max - lat_min) + lat_min
        aclon = self.numpy_random.rand(n) * (lon_max - lon_min) + lon_min
        achdg = self.numpy_random.randint(1, 360, n)
        acalt_ = (
            np.full(n, acalt.value)
            if acalt is not None
            else q.ft_to_m(self.numpy_random.randint(2000, 39000, n))
        )
        acspd_ = (
            np.full(n, acspd)
            if acspd is not None
            else q.kt_to_mps(self.numpy_random.randint(250, 450, n))
        )

        self.__create_aircraft(np.array(callsign), actype_, aclat, aclon, achdg, acalt_, acspd_)

        return Ok(f"{n} aircraft created")

    def _aircraft_name_collision(self, callsign: str) -> str | None:
        """Return why a new aircraft name is unavailable, if anything.

        Aircraft identifiers must remain unique. BlueSky allowed the same
        text to name an aircraft, group, and area; command-specific resolution
        keeps that scenario compatibility.
        """
        name = callsign.upper()
        if name in self.callsign:
            return f"aircraft {name} already exists"
        return None

    def __create_aircraft(
        self,
        acid: np.ndarray,
        actype: np.ndarray,
        lat: q.LatitudeDeg[np.ndarray],
        lon: q.LongitudeDeg[np.ndarray],
        hdg: q.TrueHeadingDegrees[np.ndarray],
        alt: q.PressureAltitudeM[np.ndarray],
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
        self.tas[-n:], self.cas[-n:], self.M[-n:] = vcasormach(spd, alt, self.casmach_threshold)
        self.gs[-n:] = self.tas[-n:]
        hdgrad = np.radians(hdg)
        self.gsnorth[-n:] = self.tas[-n:] * np.cos(hdgrad)
        self.gseast[-n:] = self.tas[-n:] * np.sin(hdgrad)

        # Atmosphere
        self.p[-n:], self.rho[-n:], self.Temp[-n:] = vatmos(alt)

        # Wind
        if self.wind.has_wind:
            applywind = self.alt[-n:] > q.ft_to_m(50.0)
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
        #         ]
        #     )
        #     # Savecmd(cmd,line): line is saved, cmd is used to prevent recording PAN & ZOOM commands and CRE
        #     # So insert a dummy command to record the line
        #     savecmd("---", line)

        # Check for crecmdlist: contains commands to be issued for this a/c
        # If any are there, then stack them for all aircraft
        for j in range(self.ntraf - n, self.ntraf):
            for cmdtxt in self.crecmdlist:
                self.stack_command(self.callsign[j] + " " + cmdtxt)

    @command(name="CRECONFS")
    def creconfs(
        self,
        callsign: Keyword,
        actype: Keyword,
        targetidx: AcId,
        dpsi: ConflictAngleDeg,
        dcpa: ConflictDistanceNM,
        tlosh: TimeS,
        dH: VerticalDistanceM | None = None,
        tlosv: TimeS | None = None,
        spd: SpeedMpsOrMach | None = None,
    ) -> None:
        """Create an aircraft in conflict with a target aircraft.

        Implements the CRECONFS stack command. The intruder position, track
        and speed are computed such that, relative to the target aircraft,
        separation is lost after the given time with the given distance at
        the closest point of approach. The protected-zone radius and height
        from the config (asas_pzr, asas_pzh) are taken into account.

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
        vsref = self.vs[targetidx]  # m/s
        cpa = q.nmi_to_m(dcpa)
        pzr = q.nmi_to_m(self.config.asas_pzr)
        pzh = q.ft_to_m(self.config.asas_pzh)
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
            tas = casormach2tas(spd, acalt, self.casmach_threshold)
            tasn, tase = tas * np.cos(trk), tas * np.sin(trk)
            wind_north, wind_east = self.wind.getdata(latref, lonref, acalt)
            gsn, gse = tasn + wind_north, tase + wind_east
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
        aclat, aclon = geo.kwikpos(latref, lonref, brn, dist)
        aclat_scalar = float(aclat)
        aclon_scalar = float(aclon)
        # convert groundspeed to CAS, and track to heading using actual
        # intruder position
        wind_north, wind_east = self.wind.getdata(aclat_scalar, aclon_scalar, acalt)
        tasn, tase = gsn - wind_north, gse - wind_east
        acspd = tas2cas(np.sqrt(tasn * tasn + tase * tase), acalt)
        achdg = np.degrees(np.atan2(tase, tasn))

        # Create and, when necessary, set vertical speed
        self.cre(
            callsign,
            actype,
            aclat_scalar,
            aclon_scalar,
            float(achdg),
            StdPressureAltM(float(acalt)),
            float(acspd),
        )
        self.ap.selaltcmd(
            np.asarray([len(self.lat) - 1]), StdPressureAltM(float(altref)), acvs
        )
        self.vs[-1] = acvs

    def delete(self, idx: int | np.ndarray) -> bool:  # type: ignore[override]
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
            self.aporasas.tas, self.aporasas.vs, self.aporasas.alt, self.kinematics.ax
        )

        # ---------- Kinematics --------------------------------
        self.kinematics.update()

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

    @overload
    def idx(self, callsign: str) -> int | None: ...
    @overload
    def idx(self, callsign: list[str] | tuple[str, ...] | set[str]) -> list[int | None]: ...
    def idx(self, callsign: str | Iterable[str]) -> int | None | list[int | None]:
        """Find the traffic-array index for one or more callsigns.

        Args:
            callsign: A single callsign string, or an iterable of callsigns.
                The special values "*" and "#" refer to the most recently
                created aircraft.

        Returns:
            Index of the aircraft (or list of optional indices when an
            iterable was given); None for callsigns that are not found.
        """
        if not isinstance(callsign, str):
            # for multiple callsigns
            # Fast way of finding indices of all ACID's in a given list
            tmp = {v: i for i, v in enumerate(self.callsign)}
            return [tmp.get(acidi) for acidi in callsign]
        else:
            # Catch last created id (* or # symbol)
            if callsign in ("#", "*"):
                return self.ntraf - 1 if self.ntraf else None

            try:
                return self.callsign.index(callsign.upper())
            except ValueError:
                return None

    @command(name="NOISE")
    def noise_status(self) -> Result[str, str]:
        """Report trajectory-noise state."""
        return Ok(f"Noise is currently {'on' if self.turbulence.active else 'off'}")

    @command(name="NOISE")
    def configure_noise(self, noise: OnOff) -> Result[str, str]:
        """Enable or disable trajectory and surveillance noise."""
        self.turbulence.setnoise(noise)
        self.noise.setnoise(noise)
        return Ok("")

    def engchange(self, acid: int, engid: str) -> None:
        """Change the engine type of an aircraft in the performance model.

        Args:
            acid: Aircraft index.
            engid: New engine type identifier.
        """
        self.perf.engchange(acid, engid)  # type: ignore[attr-defined]

    @command(name="MOVE")
    def move(
        self,
        idx: AcId,
        position: LatLonDeg,
        alt: StdPressureAltM | None = None,
        hdg: HeadingDeg | None = None,
        casmach: SpeedMpsOrMach | None = None,
        vspd: VspdMps | None = None,
    ) -> None:
        """Instantaneously move an aircraft to a new position/state.

        Implements the MOVE stack command. Optional state values are left
        unchanged when omitted. Setting a vertical speed disengages VNAV.

        Args:
            idx: Aircraft index.
            position: New latitude and longitude [deg].
            alt: Optional new altitude [m]; also sets the selected altitude.
            hdg: Optional new heading [deg]; also sets the autopilot track.
            casmach: Optional new speed, CAS [m/s] or Mach [-].
            vspd: Optional new vertical speed [m/s].
        """
        self.lat[idx] = position.lat
        self.lon[idx] = position.lon

        if alt is not None:
            self.alt[idx] = alt.value
            self.selalt[idx] = alt.value

        if hdg is not None:
            heading = (
                (hdg.degrees + geo.magdec(position.lat, position.lon)) % 360.0
                if isinstance(hdg, MagneticHeadingDeg)
                else hdg.degrees
            )
            self.hdg[idx] = heading
            self.ap.trk[idx] = heading

        if casmach is not None:
            h = alt.value if alt is not None else float(self.alt[idx])
            self.tas[idx], self.selspd[idx], _ = casormach(casmach, h, self.casmach_threshold)

        if vspd is not None:
            self.vs[idx] = vspd
            self.swvnav[idx] = False

    @command(name="POS", aliases=("AWY", "AIRPORT", "RUNWAYS", "AIRWAY", "AIRWAYS"))
    def position(self, name: Keyword) -> Result[str, str]:
        """Show information on an aircraft, airport, waypoint or navaid."""
        index = self.idx(name)
        if index is not None:
            return self.position_aircraft(index)
        return self.position_by_name(name)

    def position_aircraft(self, idx: int) -> Result[str, str]:
        """Generate a position report for a single aircraft.

        The report includes position, heading/track [deg], altitude [ft],
        vertical speed [fpm], CAS/TAS/GS [kts], Mach, active FMS modes
        (LNAV/VNAV) with the active waypoint, and origin/destination.

        Args:
            idx: Aircraft index.
        """

        acid = self.callsign[idx]

        actype = self.typecode[idx]
        latlon = latlon2txt(self.lat[idx], self.lon[idx])
        alt = round(q.m_to_ft(self.alt[idx]))
        hdg = round(self.hdg[idx])
        trk = round(self.trk[idx])
        cas = round(q.mps_to_kt(self.cas[idx]))
        tas = round(q.mps_to_kt(self.tas[idx]))
        gs = round(q.mps_to_kt(self.gs[idx]))
        M = self.M[idx]
        VS = round(q.mps_to_fpm(self.vs[idx]))
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
        if self.swlnav[idx] and route.wpname and (active_idx := route.iactwp) is not None:
            if self.swvnav[idx]:
                if self.swvnavspd[idx]:
                    info = info + "VNAV (incl.VNAVSPD), "
                else:
                    info = info + "VNAV (NOT VNAVSPD), "

            info += "LNAV to " + route.wpname[active_idx] + "\n"

        # Flight info: Destination and origin
        if self.ap.orig[idx] != "" or self.ap.dest[idx] != "":
            info = info + "Flying"

            if self.ap.orig[idx] != "":
                info = info + " from " + self.ap.orig[idx]

            if self.ap.dest[idx] != "":
                info = info + " to " + self.ap.dest[idx]

        return Ok(info)

    def position_by_name(self, name: str) -> Result[str, str]:
        """Look up a name and generate an information report for it.

        Searches, in order: airports, aircraft callsigns, waypoints/navaids,
        and airways in the navigation database. Airport reports include
        position, elevation [ft] and runways; navaid reports include type,
        frequency and airway connections.

        Args:
            name: Name/identifier to look up (case-insensitive).
        """
        name = name.upper()

        lines = "Information on " + name + ":\n"

        # First try airports (most used and shorter, hence faster list)
        idx_airport = self.navigation.getaptidx(name)
        if idx_airport is not None:
            airport_size = self.navigation.apsize[idx_airport].name.lower()

            aptname = self.navigation.aptname[idx_airport]
            aptlat = self.navigation.aptlat[idx_airport]
            aptlon = self.navigation.aptlon[idx_airport]
            aptelev = self.navigation.aptelev[idx_airport]

            # country informatation
            idx_cc = self.navigation.cocode2.index(self.navigation.aptco[idx_airport].upper())
            country_name = self.navigation.coname[idx_cc].upper()
            country_code = self.navigation.aptco[idx_airport]

            lines += (
                f"{aptname} is a {airport_size} airport in {country_name} ({country_code}):\n"
                f"Position: {latlon2txt(aptlat, aptlon)}\n"
                f"Elevation: {round(q.m_to_ft(aptelev))} ft \n"
            )

            if self.navigation.aptid[idx_airport] in self.navigation.rwythresholds:
                runways = self.navigation.rwythresholds[self.navigation.aptid[idx_airport]].keys()
                if runways:
                    lines += f"Runways: {', '.join(runways)}\n"

            return Ok(lines)

        # try aircraft
        idx_ac = self.idx(name)
        if idx_ac is not None:
            return self.position_aircraft(idx_ac)

        # Not found as airport, try waypoints & navaids
        else:
            idx_waypoints = self.navigation.getwpindices(name)
            if idx_waypoints:
                typetxt = ""
                desctxt = ""
                lastdesc = "XXXXXXXX"
                for i in idx_waypoints:
                    # One line type text
                    if typetxt == "":
                        typetxt = typetxt + self.navigation.wptype[i]
                    else:
                        typetxt = typetxt + " and " + self.navigation.wptype[i]

                    # Description: multi-line
                    samedesc = self.navigation.wpdesc[i] == lastdesc
                    if desctxt == "":
                        desctxt = desctxt + self.navigation.wpdesc[i]
                        lastdesc = self.navigation.wpdesc[i]
                    elif not samedesc:
                        desctxt = desctxt + "\n" + self.navigation.wpdesc[i]
                        lastdesc = self.navigation.wpdesc[i]

                    # Navaid: frequency
                    if self.navigation.wptype[i] in ["VOR", "DME", "TACAN"] and not samedesc:
                        desctxt = desctxt + " " + str(self.navigation.wpfreq[i]) + " MHz"
                    elif self.navigation.wptype[i] == "NDB" and not samedesc:
                        desctxt = desctxt + " " + str(self.navigation.wpfreq[i]) + " kHz"

                iwp = idx_waypoints[0]

                # Basic info
                lines += (
                    f"{name} is a {typetxt} with \n"
                    f"Position: {latlon2txt(self.navigation.wplat[iwp], self.navigation.wplon[iwp])}\n"
                )

                # Navaids have description
                if len(desctxt) > 0:
                    lines += f"{desctxt}\n"

                # VOR give variation
                if self.navigation.wptype[iwp] == "VOR":
                    lines += f"Variation: {self.navigation.wpvar[iwp]} deg\n"

                # How many others?
                n_other = self.navigation.wpid.count(name) - len(idx_waypoints)
                if n_other > 0:
                    lines += f"Attention: {n_other} other waypoint(s) also has name {name}\n"

                # In which airways?
                connect = self.navigation.listconnections(
                    name, self.navigation.wplat[iwp], self.navigation.wplon[iwp]
                )
                if len(connect) > 0:
                    awset = set()
                    for c in connect:
                        awset.add(c[0])

                    lines += f"Connected to airways: {'-'.join(awset)}\n"

                return Ok(lines)

            # Try airway id
            else:  # airway
                awid = name
                airway = self.navigation.listairway(awid)
                if len(airway) > 0:
                    lines = ""
                    for segment in airway:
                        lines += f"Airway {awid}: {' - '.join(segment)}\n"
                    return Ok(lines)

        # nothing matched
        return Err(f"{name} not found as aircraft, airport, navaid, or waypoint")

        # Show what we found on airport and navaid/waypoint

    def settrans(self, alt: StdPressureAltM | None = None) -> Result[str, str]:
        """Set or show the transition level.

        Args:
            alt: Optional new transition level [m] (stack input in ft/FL).
        """
        # In case a new value is given, set it.
        if alt is not None:
            if alt.value > 0.0:
                self.translvl = alt.value
                return Ok("")
            return Err("Transition level needs to be ft/FL and larger than zero")

        # In case no value is given, show it
        tlvl = round(q.m_to_ft(self.translvl))
        return Ok(f"Transition level = {tlvl}/FL{round(tlvl / 100.0)}")

    @command(name="BANK", aliases=("BANKLIM",))
    def bank_limit_status(self, idx: AcIdSelection) -> Result[str, str]:
        """Show the bank-angle limit for an aircraft or selection."""
        return Ok(
            "\n".join(
                f"Banklimit of {self.callsign[index]} is "
                f"{int(np.degrees(self.ap.bankdef[index]))} deg"
                for index in idx
            )
        )

    @command(name="BANK")
    def set_bank_limit(self, idx: AcIdSelection, bankangle: BankLimitDeg) -> Result[str, str]:
        """Set the bank-angle limit for an aircraft or selection."""
        self.ap.bankdef[idx] = np.radians(bankangle)
        return Ok("")

    @command(name="THR")
    def throttle_status(self, idx: AcId) -> Result[str, str]:
        """Report autothrottle state and fixed throttle when applicable."""
        if self.swats[idx]:
            return Ok("ATS of " + self.callsign[idx] + " is ON")
        return Ok("ATS of " + self.callsign[idx] + " is OFF. THR is " + str(self.thr[idx]))

    @command(name="THR")
    def enable_autothrottle(self, idx: AcId, _mode: Literal["AUTO", "OFF"]) -> Result[str, str]:
        """Enable autothrottle."""
        self.swats[idx] = True
        return Ok("")

    @command(name="THR")
    def set_idle_throttle(self, idx: AcId, _mode: Literal["IDLE"]) -> Result[str, str]:
        """Disable autothrottle and select idle thrust."""
        self.swats[idx] = False
        self.thr[idx] = 0.0
        return Ok("")

    @command(name="THR")
    def set_throttle(self, idx: AcId, throttle: Throttle) -> Result[str, str]:
        """Disable autothrottle and set a fixed throttle fraction."""
        self.swats[idx] = False
        self.thr[idx] = throttle
        return Ok("")

    def _crecmd_status(self) -> Result[str, str]:
        if self.crecmdlist:
            commands = "; ".join(f"[acid] {text}" for text in self.crecmdlist)
            return Ok(f"CRECMD list: {commands}")
        return Ok("CRECMD will add a/c specific commands to an aircraft after creation")

    @command(name="CRECMD")
    def crecmd_status(self) -> Result[str, str]:
        """Show commands issued for every newly created aircraft."""
        return self._crecmd_status()

    @command(name="CRECMD")
    def crecmd_status_explicit(self, _query: Literal["?"]) -> Result[str, str]:
        """Show commands issued for every newly created aircraft."""
        return self._crecmd_status()

    @command(name="CRECMD")
    def add_crecmd(self, cmdline: Text) -> Result[str, str]:
        """Add a command to issue for every newly created aircraft."""
        self.crecmdlist.append(cmdline)
        return Ok("")

    @command(name="CLRCRECMD")
    def clrcrecmd(self) -> Result[str, str]:
        """Clear the list of commands issued for newly created aircraft.

        Implements the CLRCRECMD stack command, removing all command lines
        previously added with CRECMD.
        """
        ncrecmd = len(self.crecmdlist)
        if ncrecmd == 0:
            return Ok("CLRCRECMD deletes all commands on clears command")
        else:
            self.crecmdlist = []
            return Ok(f"All {ncrecmd} crecmd commands deleted.")
